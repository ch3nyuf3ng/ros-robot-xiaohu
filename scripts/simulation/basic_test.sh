#!/bin/bash

rostopic pub -1 /xiaohu_robot/init_position_request geometry_msgs/Pose \
'{position: {x: -0.0270498, y: -3.37887, z: 0}, orientation: {x: 0, y: 0, z: 0.706908, w: 0.707306}}'

rostopic pub -1 /xiaohu_robot/legacy_tasks_request xiaohu_robot/GeneralTaskMessage \
"{type: 'Inspection', mapName: '', savePath: '', pharmacy: '', patient: 'kitchen', prescription: ''}"

rostopic pub -1 /xiaohu_robot/legacy_tasks_request xiaohu_robot/GeneralTaskMessage \
"{type: 'Inspection', mapName: '', savePath: '', pharmacy: '', patient: 'table_front', prescription: ''}"

rostopic pub -1 /xiaohu_robot/legacy_tasks_request xiaohu_robot/GeneralTaskMessage \
"{type: 'Inspection', mapName: '', savePath: '', pharmacy: '', patient: 'table_back', prescription: ''}"

rostopic pub -1 /xiaohu_robot/legacy_tasks_request xiaohu_robot/GeneralTaskMessage \
"{type: 'MedicineDelivery', mapName: '', savePath: '', pharmacy: 'kitchen', patient: 'table_front', prescription: '一日一次，一次两片'}"

rostopic pub -1 /xiaohu_robot/legacy_tasks_request xiaohu_robot/GeneralTaskMessage \
"{type: 'MedicineDelivery', mapName: '', savePath: '', pharmacy: 'kitchen', patient: 'table_back', prescription: '一日一次，一次两片'}"
