#ifndef CTRL_TOOLBOX_ROS2__CONTROLLER_NODE_HPP_
#define CTRL_TOOLBOX_ROS2__CONTROLLER_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"

#include "controller_toolbox/IController.h"

#include "ctrl_toolbox_ros2/visibility_control.h"

namespace ctrl_toolbox_ros2
{

/**
 * @brief Lifecycle node adapter that wraps any ctrl::IController as a ROS 2 node.
 *
 * ## Topics (relative to node namespace)
 * | Topic             | Type                   | Direction |
 * |-------------------|------------------------|-----------|
 * | `~/setpoint`      | std_msgs/Float64       | Subscribed  |
 * | `~/measurement`   | std_msgs/Float64       | Subscribed  |
 * | `~/control_output`| std_msgs/Float64       | Published   |
 *
 * ## ROS 2 Parameters
 * | Name             | Type   | Default | Description                    |
 * |------------------|--------|---------|--------------------------------|
 * | `sample_time_s`  | double | 0.01    | Control loop period (seconds)  |
 *
 * ## Lifecycle transitions
 * - **on_configure** — invokes the user-supplied factory to construct T;
 *   reads `sample_time_s`.
 * - **on_activate** — creates publisher, subscriptions, and the control timer.
 * - **on_deactivate** — destroys the timer; calls `controller_->reset()`.
 * - **on_cleanup / on_shutdown** — releases all resources.
 *
 * ## Control law
 * At each timer tick:
 * ```
 * u = controller_->compute(setpoint - measurement)
 * ```
 * The `compute()` argument is `(r - y)`, matching the sign convention of
 * DiscretePID, ADRC, SMC (compute(y - ref)), and similar controllers.
 * Controllers that use `set_reference()` + `compute(y)` (MRAC, L1Adaptive)
 * should be wrapped in a thin adapter that implements IController::compute().
 *
 * @tparam T  A class publicly derived from ctrl::IController.
 *            Pass ctrl::IController itself to store any controller type.
 */
template<typename T = ctrl::IController>
class ControllerNode : public rclcpp_lifecycle::LifecycleNode
{
    static_assert(
        std::is_base_of<ctrl::IController, T>::value,
        "ControllerNode<T>: T must be publicly derived from ctrl::IController");

public:
    using SharedPtr = std::shared_ptr<ControllerNode<T>>;

    /**
     * @brief Factory function type.
     *
     * Called during on_configure() with a reference to this node so the
     * factory can read/declare ROS 2 parameters before constructing T.
     *
     * @example
     * ```cpp
     * auto factory = [](rclcpp_lifecycle::LifecycleNode & node) {
     *     ctrl::DiscretePIDParams p;
     *     p.Kp = node.declare_parameter<double>("Kp", 1.0);
     *     p.Ts = node.get_parameter("sample_time_s").as_double();
     *     return std::make_shared<ctrl::DiscretePID>(p);
     * };
     * ```
     */
    using FactoryFn = std::function<std::shared_ptr<T>(rclcpp_lifecycle::LifecycleNode &)>;

    /**
     * @param node_name  ROS 2 node name.
     * @param factory    Callable that constructs and returns the controller.
     *                   Called once during on_configure().
     * @param options    Node options (default: empty).
     */
    explicit ControllerNode(
        const std::string & node_name,
        FactoryFn factory,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
    : rclcpp_lifecycle::LifecycleNode(node_name, options)
    , factory_(std::move(factory))
    {
        declare_parameter<double>("sample_time_s", 0.01);
    }

    // -----------------------------------------------------------------------
    // Lifecycle callbacks
    // -----------------------------------------------------------------------

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "[%s] Configuring ...", get_name());
        try {
            controller_ = factory_(*this);
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "Controller factory threw: %s", ex.what());
            return CallbackReturn::FAILURE;
        }
        if (!controller_) {
            RCLCPP_ERROR(get_logger(), "Controller factory returned nullptr.");
            return CallbackReturn::FAILURE;
        }
        sample_time_s_ = get_parameter("sample_time_s").as_double();
        if (sample_time_s_ <= 0.0) {
            RCLCPP_ERROR(get_logger(), "sample_time_s must be positive (got %.6f).", sample_time_s_);
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(get_logger(), "[%s] Configured (Ts = %.6f s).", get_name(), sample_time_s_);
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "[%s] Activating ...", get_name());

        pub_ = create_publisher<std_msgs::msg::Float64>("~/control_output", 10);

        sub_setpoint_ = create_subscription<std_msgs::msg::Float64>(
            "~/setpoint", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                setpoint_ = msg->data;
            });

        sub_meas_ = create_subscription<std_msgs::msg::Float64>(
            "~/measurement", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                measurement_ = msg->data;
            });

        const auto period = std::chrono::duration<double>(sample_time_s_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { timer_callback(); });

        RCLCPP_INFO(get_logger(), "[%s] Active.", get_name());
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "[%s] Deactivating ...", get_name());
        timer_.reset();
        sub_setpoint_.reset();
        sub_meas_.reset();
        pub_.reset();
        if (controller_) {
            controller_->reset();
        }
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override
    {
        controller_.reset();
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &) override
    {
        timer_.reset();
        sub_setpoint_.reset();
        sub_meas_.reset();
        pub_.reset();
        controller_.reset();
        return CallbackReturn::SUCCESS;
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /** @brief Returns the underlying controller (nullptr before on_configure). */
    std::shared_ptr<T> controller() const { return controller_; }

    /** @brief Returns the most recently received setpoint. */
    double setpoint() const { return setpoint_; }

    /** @brief Returns the most recently received measurement. */
    double measurement() const { return measurement_; }

private:
    void timer_callback()
    {
        if (!controller_ || !pub_) return;
        const double u = controller_->compute(setpoint_ - measurement_);
        std_msgs::msg::Float64 out;
        out.data = u;
        pub_->publish(out);
    }

    FactoryFn factory_;
    std::shared_ptr<T> controller_;

    double sample_time_s_{0.01};
    double setpoint_{0.0};
    double measurement_{0.0};

    typename rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    typename rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_setpoint_;
    typename rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_meas_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ctrl_toolbox_ros2

#endif  // CTRL_TOOLBOX_ROS2__CONTROLLER_NODE_HPP_
