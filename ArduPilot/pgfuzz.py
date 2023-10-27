import time
from subprocess import *
import os

PGFUZZ_HOME = os.getenv("PGFUZZ_HOME")

if PGFUZZ_HOME is None:
    raise Exception("PGFUZZ_HOME environment variable is not set!")

ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")

if ARDUPILOT_HOME is None:
    raise Exception("ARDUPILOT_HOME environment variable is not set!")

open("restart.txt", "w").close()

print("start execute ArduPilot/open_simulator.py")
c = 'gnome-terminal -- python ' + PGFUZZ_HOME + 'ArduPilot/open_simulator.py &'
try:
	open_sim_handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
except Exception as e:
	print("open_simulator error:", e)

# time.sleep(45)

# print("start execute ArduPilot/fuzzing.py")
# c = 'gnome-terminal -- python ' + PGFUZZ_HOME + 'ArduPilot/fuzzing.py &'
# try:
# 	fuzz_handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
# except Exception as e:
# 	print("fuzz_simulator error:", e)

print("start pgfuzz loop")
while True:
	time.sleep(1)

	f = open("restart.txt", "r")

	if f.read() == "restart":
		print("restart!!!")
		f.close()
		open("restart.txt", "w").close()
		time.sleep(2)
		c = 'gnome-terminal -- python ' + PGFUZZ_HOME + 'ArduPilot/open_simulator.py &'
		open_sim_handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
	
