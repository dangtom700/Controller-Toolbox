# ctrl_toolbox_ros2

ROS 2 lifecycle node adapter for the [Controller Toolbox](../../README.md) C++ library.
Wraps any `ctrl::IController` as a managed ROS 2 lifecycle node.

## Requirements

| Dependency | Version |
|---|---|
| ROS 2 | Humble or later |
| controller_toolbox | installed via `cmake --install` (DIST-1) |
| ament_cmake | included with ROS 2 |

## Package layout

```
ctrl_toolbox_ros2/
├── include/ctrl_toolbox_ros2/
│   ├── controller_node.hpp   # ControllerNode<T> template (main API)
│   └── visibility_control.h  # DLL export macros
├── example/
│   └── pid_temperature_node.cpp   # Concrete example: PID with DiscretePID
├── CMakeLists.txt
└── package.xml
```

## Build

```bash
# 1. Install controller_toolbox to a prefix (e.g. /opt/ctrl_toolbox)
cmake --install <build_dir> --prefix /opt/ctrl_toolbox

# 2. Source ROS 2
source /opt/ros/humble/setup.bash

# 3. Build this package in a colcon workspace
mkdir -p ~/ros2_ws/src
cp -r <repo>/ros2/ctrl_toolbox_ros2 ~/ros2_ws/src/
cd ~/ros2_ws

colcon build \
  --packages-select ctrl_toolbox_ros2 \
  --cmake-args "-DCMAKE_PREFIX_PATH=/opt/ctrl_toolbox"
```

## Quick start

```bash
source ~/ros2_ws/install/setup.bash

# 1. Start the node (it begins in Unconfigured state)
ros2 run ctrl_toolbox_ros2 pid_temperature_node \
  --ros-args -p Kp:=2.0 -p Ki:=0.1 -p sample_time_s:=0.05

# 2. Drive it through the lifecycle
ros2 lifecycle set /pid_temperature_controller configure
ros2 lifecycle set /pid_temperature_controller activate

# 3. Publish a setpoint and a measurement
ros2 topic pub /pid_temperature_controller/setpoint std_msgs/Float64 '{data: 75.0}'
ros2 topic pub /pid_temperature_controller/measurement std_msgs/Float64 '{data: 20.0}'

# 4. Watch the output
ros2 topic echo /pid_temperature_controller/control_output
```

## API: `ControllerNode<T>`

```cpp
#include "ctrl_toolbox_ros2/controller_node.hpp"

namespace ctrl_toolbox_ros2 {

template<typename T = ctrl::IController>
class ControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    // Factory called during on_configure(); reads/declares ROS 2 params.
    using FactoryFn = std::function<std::shared_ptr<T>(rclcpp_lifecycle::LifecycleNode&)>;

    ControllerNode(const std::string& node_name, FactoryFn factory,
                   const rclcpp::NodeOptions& options = {});

    std::shared_ptr<T> controller() const;  // nullptr before on_configure
    double setpoint() const;
    double measurement() const;
};
```

### Topics

| Topic | Type | Notes |
|---|---|---|
| `~/setpoint` | `std_msgs/Float64` | Reference signal `r` |
| `~/measurement` | `std_msgs/Float64` | Plant output `y` |
| `~/control_output` | `std_msgs/Float64` | `u = controller->compute(r - y)` |

### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `sample_time_s` | double | `0.01` | Control period in seconds |

Additional controller-specific parameters can be declared in the factory lambda.

### Control law

At each timer tick: `u = controller_->compute(setpoint - measurement)`

This matches the `compute(r - y)` convention of `DiscretePID`, `DiscreteADRC`,
`SmithPredictor`, and others. Controllers using `set_reference()` + `compute(y)`
(e.g. `MRACController`, `L1AdaptiveController`) should be wrapped in a thin
`IController` adapter before passing to `ControllerNode`.

## Using any ctrl::IController

```cpp
#include "ctrl_toolbox_ros2/controller_node.hpp"
#include "controller_toolbox/DiscreteMPC.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto factory = [](rclcpp_lifecycle::LifecycleNode& node)
        -> std::shared_ptr<ctrl::DiscreteMPC>
    {
        ctrl::MPCParams p;
        p.Np = node.declare_parameter<int>("Np", 10);
        p.Nu = node.declare_parameter<int>("Nu", 3);
        // ... set up plant SS, weights, etc. ...
        return std::make_shared<ctrl::DiscreteMPC>(plant_ss, p);
    };

    auto node = std::make_shared<
        ctrl_toolbox_ros2::ControllerNode<ctrl::DiscreteMPC>>(
            "mpc_node", factory);

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}
```

## Lifecycle state machine

```
Unconfigured --[configure]--> Configured --[activate]--> Active
                                    ^                        |
                                    |------[deactivate]------|
                                    |
                               [cleanup]--> Unconfigured
```

- **on_configure**: factory constructs the controller; reads `sample_time_s`.
- **on_activate**: creates publisher, subscriptions, and the wall timer.
- **on_deactivate**: destroys timer + I/O; calls `controller_->reset()`.
- **on_cleanup / on_shutdown**: releases the controller.
