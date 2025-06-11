# Building Your ROS 2 Workspace

This guide shows you how to build your ROS 2 workspace using two common methods.

---

## 1. Build with Symlink Install (Recommended for Development)

This method uses symbolic links for installed files, making it easy to see code changes immediately without rebuilding.

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

- **When to use:** Actively developing your packages and want changes to reflect instantly.
- **Benefits:** Faster iteration, no need to rebuild for every code change.

---

- **When to use:** Preparing for release, deployment, or testing in a production-like environment.
- **Benefits:** Mimics the final install layout, ensures all files are properly copied.

---

## 3. Build a Specific Package

To build only a specific package (replace `your_package_name` with your actual package):


```bash
colcon build --packages-select your_package_name
```

---

## Notes

- Always source your workspace after building:
  ```bash
  source install/setup.bash
