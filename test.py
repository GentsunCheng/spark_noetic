import time
from uarm import SwiftAPI

swift = SwiftAPI("/dev/ttyACM0")

swift.waiting_ready()

swift.set_position(20, 170, 174)
time.sleep(1)
print(swift.get_position())