Header   header
uint8    ctrl_fb_gear  # 当前挡位
float32  ctrl_fb_velocity  # 车体反馈速度
float32  ctrl_fb_steering # 当前车体转向角反馈
uint8    ctrl_fb_Brake   # 车体速度反馈
uint8    ctrl_fb_mode   # 车体运行模式
uint16   heading        # 航向角 
int16    pitch          #俯仰角
int16    roll           #横滚角
float32  lr_wheel_fb_velocity # 左后轮车速
int32    rr_wheel_fb_velocity  # 右后轮车速
int64    poslon      #经度
int64    poslat      #纬度
# 后期可增加车辆其他信息 电池信息 
float32 bms_Infor_voltage
float32 bms_Infor_current
float32 bms_Infor_remaining_capacity

