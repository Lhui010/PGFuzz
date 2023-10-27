from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

master.wait_heartbeat()

def set_param(param_name, new_param_value):
    print("set {} to {}\n".format(param_name, new_param_value))
    master.mav.param_set_send(master.target_system, master.target_component,
                              param_name.encode('utf-8'),
                              new_param_value,
                              mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    while True:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True)
        if msg is not None:
            if msg.param_id == param_name and msg.param_value == new_param_value:
                print(f"参数 {param_name} 已成功设置为 {new_param_value} 度")
                break

set_param("ANGLE_MAX", -1)

mode_id = master.mode_mapping()['RTL']
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


master.close()
