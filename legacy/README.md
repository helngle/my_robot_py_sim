# Legacy ROS 2 Packages

This directory keeps superseded packages for reference and rollback.

`COLCON_IGNORE` prevents colcon from discovering or building packages in this
directory. The active real-robot launch path uses the split packages in the
workspace root instead of `legacy/my_robot_py_sim`.
