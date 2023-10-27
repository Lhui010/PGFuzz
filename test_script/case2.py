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

# set_param("ATC_RAT_YAW_FLTT", 1)
# set_param("ATC_RAT_YAW_FLTD", 1)
# set_param("ATC_RAT_YAW_FLTE", 1)
# set_param("EK2_ACC_P_NSE", 1)
# set_param("EK2_ALT_M_NSE", 10)
# set_param("AHRS_COMP_BETA", 0.5)

set_param("GND_TEMP", 40)

# set_param("LOIT_SPEED", 1000)

# mode_id = master.mode_mapping()['LOITER']
# master.mav.set_mode_send(
#             master.target_system,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_id)