from pymavlink import mavutil
import time


# ------------------------------------------------------------------------------------
# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        print("Channel does not exist.")
        return

    # We only have 8 channels
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm

        # global master

        master.mav.rc_channels_override_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.


# 设置与无人机的连接
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

master.wait_heartbeat()

mode_id = master.mode_mapping()['CIRCLE']

master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

# 请求当前飞行模式
master.mav.request_data_stream_send(
    master.target_system,                   # 目标系统ID，通常为1
    master.target_component,                # 目标组件ID，通常为0
    mavutil.mavlink.MAV_DATA_STREAM_ALL,    # 请求所有数据流
    1,                                      # 请求率，1Hz
    1                                       # 开启数据流
)

# 读取并显示当前飞行模式
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)  # 接收HEARTBEAT消息
    if msg:
        flight_mode = mavutil.mode_mapping_acm.get(msg.custom_mode, "Unknown")
        print("当前飞行模式: {}".format(flight_mode))
        break



# 关闭与无人机的连接
master.close()