## 🚀 JawPixArm description and simulation package

# To use rviz2 execute the following commands:

```bash
cd ~/Jaw_roboarm
colcon build
source install/setup.bash
ros2 launch jawarm_description viewarm.launch.py
```
It should open the viewarm.rviz file automatically, if not, open it manually.

---
# To use the gazebo simulator execute the following commands:

```bash
cd ~/Jaw_roboarm
colcon build
source install/setup.bash
gz sim ~/Jaw_roboarm/src/jawarm_description/gazebo/arm.world 
```


