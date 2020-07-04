import time
from LidarX2 import LidarX2

lidar = LidarX2("/dev/cu.SLAB_USBtoUART")  # Name of the serial port, can be /dev/tty*, COM*, etc.

if not lidar.open():
    print("Cannot open lidar")
    exit(1)

t = time.time()
while time.time() - t < 20:  # Run for 20 seconds
    measures = lidar.getMeasures()  # Get latest lidar measures
    print(measures)
    time.sleep(0.01)

lidar.close()