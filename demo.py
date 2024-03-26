from dynamixel_sdk import *
import time

portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(1.0)

portHandler.openPort()
portHandler.setBaudRate(1000000)

ADDR_GOAL_POSITION = 30


ids = []
for id in range(0, 100):
    model_number, result, error = packetHandler.ping(portHandler, id)
    if model_number == 12:
        print(f"Found AX-12 with id: {id}")
        ids.append(id)
    time.sleep(0.01)

print("Found motors:")
print(ids)

print("Send motors to zero (press enter)")
input()
t = 0.0
while True:
    for id in ids:
        packetHandler.write2ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, 512)
    time.sleep(0.01)