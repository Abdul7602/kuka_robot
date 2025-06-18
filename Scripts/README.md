# Building Your ROS 2 Workspace

This guide shows you how to build your ROS 2 workspace using two common methods.

---

## Build with Symlink Install (Recommended for Development)

This method uses symbolic links for installed files, making it easy to see code changes immediately without rebuilding.

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

```bash
MAKEFLAGS="-j8 -l8" colcon build --mixin release --parallel-workers 4
```
- **When to use:** Actively developing your packages and want changes to reflect instantly.
- **Benefits:** Faster iteration, no need to rebuild for every code change.

---

## Build a Specific Package

To build only a specific package (replace `your_package_name` with your actual package):


```bash
colcon build --packages-select your_package_name
```

---
# combined_launch

A ROS 2 package for launching multiple nodes or configurations together.

## Create the package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python combined_launch
```

## Source

Always source your workspace after building:
  ```bash
  source install/setup.bash
