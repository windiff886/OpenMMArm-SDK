# 电机控制指令消息
uint8 mode           # 电机目标模式
float32 q            # 电机目标位置 (rad)
float32 dq           # 电机目标速度 (rad/s)
float32 tau          # 电机目标力矩 (N*m)
float32 kp           # 电机弹簧刚度系数
float32 kd           # 电机阻尼系数
uint32[3] reserve    # 保留字段
