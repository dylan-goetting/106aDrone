from tello import *

start()
power = get_battery()
print("Power Level: ", power, "%")
takeoff()
land()