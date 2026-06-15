#ifndef CTRL_TOOLBOX_ROS2__VISIBILITY_CONTROL_H_
#define CTRL_TOOLBOX_ROS2__VISIBILITY_CONTROL_H_

// Standard ROS 2 visibility macros.
// Not strictly required for a header-only library but provided for
// any concrete subclasses users may add to this package.
#if defined(_WIN32) || defined(__CYGWIN__)
  #ifdef CTRL_TOOLBOX_ROS2_BUILDING_DLL
    #define CTRL_TOOLBOX_ROS2_PUBLIC __declspec(dllexport)
  #else
    #define CTRL_TOOLBOX_ROS2_PUBLIC __declspec(dllimport)
  #endif
  #define CTRL_TOOLBOX_ROS2_LOCAL
#else
  #define CTRL_TOOLBOX_ROS2_PUBLIC  __attribute__((visibility("default")))
  #define CTRL_TOOLBOX_ROS2_LOCAL   __attribute__((visibility("hidden")))
#endif

#endif  // CTRL_TOOLBOX_ROS2__VISIBILITY_CONTROL_H_
