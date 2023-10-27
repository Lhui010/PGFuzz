from pymavlink import mavutil
import math

# 连接到SITL仿真
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
# 设置目标roll、pitch和yaw角度（以弧度为单位）
roll_angle_deg = 30  # 期望的roll角度（以度为单位）
pitch_angle_deg = 0  # 期望的pitch角度（以度为单位）
yaw_angle_deg = 0  # 期望的yaw角度（以度为单位）

roll_angle_rad = math.radians(roll_angle_deg)  # 转换为弧度
pitch_angle_rad = math.radians(pitch_angle_deg)  # 转换为弧度
yaw_angle_rad = math.radians(yaw_angle_deg)  # 转换为弧度

# 计算四元数表示的目标姿态
cos_r2 = math.cos(roll_angle_rad / 2)
sin_r2 = math.sin(roll_angle_rad / 2)
cos_p2 = math.cos(pitch_angle_rad / 2)
sin_p2 = math.sin(pitch_angle_rad / 2)
cos_y2 = math.cos(yaw_angle_rad / 2)
sin_y2 = math.sin(yaw_angle_rad / 2)

q = [
    cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2,
    sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2,
    cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2,
    cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2
]

# 创建SET_ATTITUDE_TARGET消息
msg = master.mav.set_attitude_target_encode(
    0,  # time_boot_ms
    1,  # Target system
    1,  # Target component
    mavutil.mavlink.MAV_FRAME_BODY_NED,  # 使用机体坐标系
    q,  # 目标四元数姿态
    0,  # 机体坐标系下的roll角速度（radian/s）
    0,  # 机体坐标系下的pitch角速度（radian/s）
    0,  # 机体坐标系下的yaw角速度（radian/s）
    0  # 推力（不设置）
)

# 发送消息
master.mav.send(msg)
