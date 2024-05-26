rostopic pub -1 /xiaohu_robot/init_position_request geometry_msgs/Pose \
'{position: {x: -0.0270498, y: -3.37887, z: 0}, orientation: {x: 0, y: 0, z: 0.706908, w: 0.707306}}'

rostopic pub -1 /xiaohu_robot/inspection_task_request \
xiaohu_robot/InspectionTaskRequest "{\
    taskId: '2', \
    patientName: '王明', \
    patientPosition: {\
        position: {x: 3.78687, y: 1.73546, z: 0.0}, \
        orientation: {x: 0.0, y: 0.0, z: 0.999999, w: -0.00112452}\
    }\
}"

rostopic pub -1 /xiaohu_robot/medicine_delivery_task_request \
xiaohu_robot/MedicineDeliveryTaskRequest "{\
    taskId: '1', \
    prescription: '感冒灵颗粒：一天三次，一次一包，开水冲服', \
    pharmacyName: '药房', \
    pharmacyPosition: {\
        position: {x: -3.40732, y: 2.49114, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.692039, w: 0.72186}
    }, \
    patientName: '李建', \
    patientPosition: {\
        position: {x: 0.455775, y: 1.69361, z: 0.0}, \
        orientation: {x: 0.0, y: 0.0, z: 0.00435664, w: 0.999991}\
    }\
}" 