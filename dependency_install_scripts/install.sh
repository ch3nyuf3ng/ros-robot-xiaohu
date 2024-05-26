#!/bin/bash

install_if_not_installed() {
    local software_name="$1"
    if ! dpkg -l | grep -q "$software_name"; then
        echo "软件 $software_name 未安装，正在安装..."
        sudo apt-get update
        sudo apt-get install -y "$software_name"
    else
        echo "软件 $software_name 已安装。"
    fi
}

install_if_not_installed "ros-melodic-teb-local-planner"
install_if_not_installed "ros-melodic-serial"
install_if_not_installed "i2c-tools"