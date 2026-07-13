# Spark2 application tests

These scripts are practical commissioning checks for a connected robot. Clear
the workspace, prepare the emergency stop, and start with the default low
speed and small trajectory. They are engineering checks inspired by common
robot acceptance tasks, not a formal ISO 9283 certification procedure.

```powershell
cd spark2_python_dist\test
python test_joint_jog.py
python test_cartesian_axes.py
python test_repeatability.py --cycles 10
python test_draw_circle.py --radius 0.03 --speed 15
python test_draw_square.py --side 0.05 --speed 15
python test_feedback_logging.py --output motion_feedback.csv
python test_torque_response.py --duration 15 --threshold 0.5
```

Suggested order:

1. `test_joint_jog.py`: verify each joint direction and basic response.
2. `test_cartesian_axes.py`: verify tool X/Y/Z directions and linear motion.
3. `test_repeatability.py`: perform repeated approaches and report position error.
4. Circle/square: inspect interpolation continuity and Cartesian path behavior.
5. `test_feedback_logging.py`: save position, velocity and torque for plotting.
6. `test_torque_response.py`: verify that external loading appears in feedback.

The circle and square use the current tool pose as their start point and keep
the current orientation and Z height. The torque-response script sends no
motion command. It measures changes relative to a one-second stationary
baseline. The current SDK has torque feedback but no public force-control API,
so this script must not be described or used as closed-loop force control.
