

from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

master.wait_heartbeat()

def set_param(param_name, new_param_value):
    print("set %s to %s\n", param_name, new_param_value)
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

set_param("MNT_ANGMAX_ROL", 4500)
set_param("MNT_ANGMAX_PAN", 4500)
set_param("MNT_ANGMAX_TIL", 4500)

# 创建 "DO_MOUNT_CONTROL" 命令消息
msg = master.mav.command_long_encode(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
    0,  # confirmation
    190, 100, 190, 0, 0, 0, 0)

# 发送消息
master.mav.send(msg)