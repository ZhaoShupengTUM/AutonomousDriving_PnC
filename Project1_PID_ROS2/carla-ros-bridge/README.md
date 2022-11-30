# 启动流程
1. cd /carla-ros-bridge
2. source source_env.sh
3. colcon build
4. source source_env.sh
5. ros2 launch carla_shenlan_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
6. 在新的终端里面: ros2 run carla_shenlan_pid_controller carla_shenlan_pid_controller_node

<!-- ![alt](./figures/test.jpg) -->