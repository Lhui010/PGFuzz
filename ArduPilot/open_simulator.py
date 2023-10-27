from subprocess import *
#import subprocess,time,psutil

import time
import os
import signal

#subprocess.call(['~/ardupilot_4_0_3/Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w'], shell=True)

ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")

if ARDUPILOT_HOME is None:
    raise Exception("ARDUPILOT_HOME environment variable is not set!")

c = ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w'
#c = '~/ardupilot_pgfuzz/Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w' 

#handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
handle = Popen(c, shell=True)

#os.killpg(os.getpgid(handle.pid), signal.SIGTERM)

print("start monitoring reboot value")
while True:
	
	f = open("shared_variables.txt", "r")
	
	if f.read() == "reboot":
		print("reboot")
		f.close()
		open("shared_variables.txt", "w").close()	
		fi = open("restart.txt", "w")
		fi.write("restart")
		fi.close()

		os.killpg(os.getpgid(handle.pid), signal.SIGKILL)
		break
	else:
		f.close()
	
	time.sleep(1)

"""
print(os.getpgid(handle.pid))
print(handle.pid)

parent_pid = handle.pid
parent = psutil.Process(parent_pid)
for child in parent.children(recursive=True):  # or parent.children() for recursive=False
    child.kill()
    print("Kill:%d" %child)

parent.kill()
"""
