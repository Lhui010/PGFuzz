from pymavlink import mavutil

# 设置与无人机的连接
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')  # 请根据您的连接方式和端口进行修改

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




# 参数名字
param_name = 'ANGLE_MAX'

# 通过MAVLink协议请求参数值
master.mav.param_request_read_send(master.target_system, master.target_component, param_name.encode('utf-8'), -1)

# 从无人机接收参数值
while True:
    msg = master.recv_match(type='PARAM_VALUE', blocking=True)
    message = msg.to_dict()
    if message['param_id'] == param_name:
        param_value = msg.param_value
        print(f"{param_name}: {param_value}")
        break

# 关闭与无人机的连接
master.close()

