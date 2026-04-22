# 电机状态消息
# 
# 字段说明：
# mode: 电机控制模式
# q: 关节位置（弧度）
# dq: 关节速度（弧度/秒）
# tau_est: 估计的关节力矩（牛顿·米）

uint8 mode
float32 q
float32 dq
float32 tau_est
