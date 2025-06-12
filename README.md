# COCOHRIW_cell_driver
This package contains the urdf description of the COCOHRIW cell as well as the configuration and the drivers needed to interact with the mounted UR5e robotic manipulator through MoveIt. 

To run everything:
```
ros2 launch cocohrip_control bringup_all.launch.py robot_ip:=<robot_ip> use_mock_hardware:=true #for simulated robot false for real

```
