#!/bin/bash
# Помощь
if [[ $1 == "-h" || $1 == "--help" ]]; then
echo "Скрипт для первичной компиляции ПО робота"
echo "
      -d|--dec
      -k|--kom"
exit 0
fi

# Читать значения аргументов
while [[ "$#" -gt 0 ]]
  do
    case $1 in
      -d|--dec) no_compress="$2"; shift;;
      -k|--kom) kompilete="$2"; shift;;
    esac
    shift
done

source /opt/ros/humble/setup.bash
if [ "$no_compress" = "y" ];
then
unzip -o docs/microrosagent.zip
# unzip -o docs/gz_ros2_control.zip
fi

if [ "$kompilete" = "" ];
then
export CMAKE_POLICY_VERSION_MINIMUM=3.5
colcon build --packages-select micro_ros_msgs
colcon build --packages-select micro_ros_setup
colcon build --packages-select micro_ros_agent
colcon build --packages-select rplidar_ros2
colcon build --packages-select remote_serial
colcon build --packages-select network_bridge
colcon build --packages-select gz_ros2_control
colcon build --packages-select gz_ros2_control_demos
colcon build --packages-select gz_ros2_control_tests
colcon build --packages-select ign_ros2_control
colcon build --packages-select ign_ros2_control_demos
colcon build --packages-select robot-pkg
# colcon build 
fi
source install/setup.bash 
alias run='bash $PWD/run.sh'
