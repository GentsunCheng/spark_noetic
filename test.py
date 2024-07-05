import time
from uarm.wrapper import SwiftAPI
swift = SwiftAPI(port="/dev/ttyACM0")
time.sleep(3)
print(swift.get_position())