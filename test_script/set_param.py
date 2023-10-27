from pymavlink import mavutil

# 参数名字，即 MNT_ANGMAX_TIL
param_name = 'MNT_ANGMAX_TIL'
# 要设置的新参数值
new_param_value = 30.0  # 例如，将参数值设置为30度

master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

def set_param(param_name, new_param_value):
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