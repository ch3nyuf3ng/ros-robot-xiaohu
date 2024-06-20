rostopic pub -1 /xiaohu_robot/enable_service_mode_request std_msgs/Empty "{}"

rostopic pub -1 /xiaohu_robot/init_position_request \
geometry_msgs/Pose \
'{position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}'

rostopic pub -1 /xiaohu_robot/inspection_task_request \
xiaohu_robot/InspectionTaskRequest "{\
    taskId: '1', \
    patientName: '王明', \
    patientPosition: {\
        position: {x: 5.10567760616, y: -0.193560967495, z: 0.0}, \
        orientation: {x: 0.0, y: 0.0, z: -0.733005164224, w: 0.680223073132}\
    }\
}"

rostopic pub -1 /xiaohu_robot/medicine_delivery_task_request \
xiaohu_robot/MedicineDeliveryTaskRequest "{\
    taskId: '2', \
    prescription: '感冒灵颗粒：一天三次，一次一包，开水冲服', \
    pharmacyName: '药房', \
    pharmacyPosition: {\
        position: {x: 5.96006343863, y: 3.42321639212, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: -0.00597024086586, w: 0.999982177953}
    }, \
    patientName: '李建', \
    patientPosition: {\
        position: {x: 5.10567760616, y: -0.193560967495, z: 0.0}, \
        orientation: {x: 0.0, y: 0.0, z: -0.733005164224, w: 0.680223073132}\
    }\
}" 
