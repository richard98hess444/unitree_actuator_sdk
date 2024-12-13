import time
import sys
sys.path.append('../lib')
print(sys)
print("test")
from unitree_actuator_sdk import *


serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

while True:
    data.motorType = MotorType.A1
    cmd.motorType = MotorType.A1
    cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
    cmd.id   = 0
    cmd.q    = 0.0 # 1.7 * 6.33
    cmd.dq   = 0.0 #-0.1 * 6.28*queryGearRatio(MotorType.A1)   # W
    cmd.kp   = 0.0 #0.01
    cmd.kd   = 0.0 #0.1 #KW
    cmd.tau  = 0.0
    serial.sendRecv(cmd, data)
    print('\n')
    print(queryGearRatio(MotorType.A1))
    print("q: " + str(data.q/6.33))
    print("dq: " + str(data.dq))
    print("temp: " + str(data.temp))
    print("merror: " + str(data.merror))
    print('\n')
    time.sleep(0.0002) # 200 us

