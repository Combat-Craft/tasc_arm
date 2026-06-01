import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

CMD_STOP = 0
CMD_EXTEND = 1
CMD_RETRACT = 2

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(5)

def send_cmd(shoulder: int, elbow: int) -> None:
    ser.write(bytes([shoulder, elbow]))

print("Shoulder extend for 1 second")
for _ in range(20):
    send_cmd(CMD_EXTEND, CMD_STOP)
    time.sleep(0.05)

print("Stop for 1 second")
for _ in range(20):
    send_cmd(CMD_STOP, CMD_STOP)
    time.sleep(0.05)
    
print("Shoulder retract for 1 second")
for _ in range(20):
    send_cmd(CMD_RETRACT, CMD_STOP)
    time.sleep(0.05)

print("Stop for 1 second")
for _ in range(20):
    send_cmd(CMD_STOP, CMD_STOP)
    time.sleep(0.05)

print("Elbow extend for 1 second")
for _ in range(20):
    send_cmd(CMD_STOP, CMD_EXTEND)
    time.sleep(0.05)

print("Stop for 1 second")
for _ in range(20):
    send_cmd(CMD_STOP, CMD_STOP)
    time.sleep(0.05)

print("Elbow retract for 1 second")
for _ in range(20):
    send_cmd(CMD_STOP, CMD_RETRACT)
    time.sleep(0.05)

print("Stop")
for _ in range(10):
    send_cmd(CMD_STOP, CMD_STOP)
    time.sleep(0.05)

ser.close()
