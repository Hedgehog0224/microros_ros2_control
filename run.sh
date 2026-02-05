#!/bin/bash
# Помощь
if [[ $1 == "-h" || $1 == "--help" ]]; then
echo "Скрипт для запуска ПО робота"
echo "|параметр         | описание                          | по умолчанию
|-----------------|-----------------------------------|--------------------------
|-i, --ip         |ip пульта дистанционного управления|192.168.1.3 или 127.0.0.1
|-p, --port       |порт получения данных на роботе    |20000
|-r, --rem-port   |порт ПДУ                           |20001
|-----------------|-----------------------------------|--------------------------
|-t, --test       |y - в тестовом режиме              |НЕ тестовый режим
|-c, --no-camera  |y - без использования камеры       |камера включена
|-s, --no-stm     |y - без использования STM32        |STM32 включена
|-j, --no-joystick|y - без использования джойстика    |джойстик включен
|-w, --ws         |путь до рабочей директории rockchip|директория запуска скрипта
|--serial         |использовать serial для связи с STM|по умолчанию micro-ros"
exit 0
fi

# Читать значения аргументов
while [[ "$#" -gt 0 ]]
  do
    case $1 in
      -i|--ip) robotrack_ip="$2"; shift;;
      -p|--port) robotrack_port="$2"; shift;;
      -m|--ver-msg) robotrack_verification_messages="$2"; shift;;
      -t|--test) robotrack_test="$2"; shift;;
      -c|--no-camera) no_robotrack_camera_use="$2"; shift;;
      -s|--no-stm) no_robotrack_stm_use="$2"; shift;;
      -j|--no-joystick) no_robotrack_joy_use="$2"; shift;;
      -w|--ws) robotrack_workspace="$2"; shift;;
      -r|--rem-port) robotrack_remote_port="$2"; shift;;
      -k) kompilete="$2"; shift;;
      --serial) serial="$2"; shift;;
    esac
    shift
done

# порт приёма
if [ "$robotrack_port" = "" ];
    then
        export ROBOTRACK_PORT=20000
    else
        export ROBOTRACK_PORT=$robotrack_port
fi
# порт передачи
if [ "$robotrack_remote_port" = "" ];
    then
        export ROBOTRACK_REMOTE_PORT=20001
    else
        export ROBOTRACK_REMOTE_PORT=$robotrack_remote_port
fi
# тестовый режим
if [ "$robotrack_test" = "y" ];
    then
        export ROBOTRACK_TEST="True"
        export ROBOTRACK_IP=127.0.0.1
        export ROBOTRACK_LOCAL_IP=127.0.0.1
    else
        export ROBOTRACK_TEST="False"
        export ROBOTRACK_IP=192.168.1.3
        export ROBOTRACK_LOCAL_IP=192.168.1.2
fi
# камера
if [ "$no_robotrack_camera_use" = "y" ];
    then
        export ROBOTRACK_CAMERA_USE="False"
    else
        export ROBOTRACK_CAMERA_USE="True"
fi
# джойстик
if [ "$no_robotrack_joy_use" = "y" ];
    then
        export ROBOTRACK_JOY_USE="False"
    else
        export ROBOTRACK_JOY_USE="True"
fi
# stm
if [ "$no_robotrack_stm_use" = "y" ];
    then
        export ROBOTRACK_STM_USE="False"
    else
        export ROBOTRACK_STM_USE="True"
fi
# передача stm'ке
if [ "$serial" = "y" ];
    then
        export ROBOTRACK_STM_SERIAL="True"
    else
        export ROBOTRACK_STM_SERIAL="False"
fi
# директория
if [ "$robotrack_workspace" = "" ];
    then
        export ROBOTRACK_WORKSPACE=$PWD
    else
        export ROBOTRACK_WORKSPACE=$robotrack_workspace
fi

############## vv ВРЕМЕННО vv ##############
export ROBOTRACK_TEST="True"
export ROBOTRACK_IP=127.0.0.1
export ROBOTRACK_LOCAL_IP=127.0.0.1
export ROBOTRACK_CAMERA_USE="False"
export ROBOTRACK_STM_USE="False"
############## ^^ ВРЕМЕННО ^^ ##############

env | grep ROBOT
source /opt/ros/humble/setup.bash
source $PWD/install/setup.bash
cd ../
source $PWD/ws_yolo/install/setup.bash
cd -

if [ "$kompilete" = "" ] && [ $ROBOTRACK_REMOTE_PORT != 0 ];
then
    if [ "$no_robotrack_camera_use" = "y" ];
    then
        source /home/firefly/Documents/ws_yolo/install/setup.bash
    fi
    source install/setup.bash && ros2 launch robot-pkg rockchip-run.py
    # colcon build && source install/setup.bash && ros2 launch robot-pkg rockchip-run.py
    # colcon build && source install/setup.bash && ros2 launch robot-pkg start.xml
fi