# ROS Noetic Python Environment

This repository contains ROS Python nodes that must run under the ROS Noetic
Python environment instead of a conda interpreter.

## Why

If the shell is opened in `conda base`, `python3` may resolve to
`/home/zhl/miniconda3/bin/python3`. That breaks ROS Python packages such as
`ros_numpy`, `rospy`, `tf`, and any apt-installed Noetic modules.

## Repository-side fixes already applied

- All ROS Python scripts in this repository use `#!/usr/bin/env python3`.
- `sentry_nav/scripts/ros_noetic_env.sh` creates a clean shell environment.
- `sentry_nav/scripts/launch_isweep_clean.sh` launches the new iSweep chain in
  that clean environment.

## Recommended usage

```bash
cd ~/robot3/nav_ws/src/NEXTE_Sentry_Nav
bash sentry_nav/scripts/launch_isweep_clean.sh
```

## Manual environment check

```bash
bash sentry_nav/scripts/ros_noetic_env.sh
which python3
python3 -c "import rospy, ros_numpy; print('ok')"
rospack find sentry_nav
```

Expected:

- `which python3` should point to `/usr/bin/python3`
- `import rospy, ros_numpy` should succeed
- `rospack find sentry_nav` should resolve to this workspace

## Optional shell hygiene

To avoid repeating this issue, do not auto-activate conda `base` in ROS
terminals, or launch ROS through the wrapper scripts above.
