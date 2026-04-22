# 低级状态信息消息
# 
# 字段说明：
# motor_state: 电机状态数组，包含12个电机的状态信息
# 电机顺序：
# 0: 前右髋关节
# 1: 前右大腿关节
# 2: 前右小腿关节
# 3: 前左髋关节
# 4: 前左大腿关节
# 5: 前左小腿关节
# 6: 后右髋关节
# 7: 后右大腿关节
# 8: 后右小腿关节
# 9: 后左髋关节
# 10: 后左大腿关节
# 11: 后左小腿关节
# 
# ee_force: 足部力信息数组，包含4个足端的力信息，顺序为 [前右, 前左, 后右, 后左]
# foot_force: 足部力大小数组，包含4个足端的力大小，顺序为 [前右, 前左, 后右, 后左]
# imu: IMU数据信息

go1_controller/MotorState[] motor_state
geometry_msgs/Vector3[] ee_force
float32[] foot_force
go1_controller/ImuData imu