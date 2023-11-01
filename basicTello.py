from djitellopy import Tello


me = Tello()
me.connect()
power = me.get_battery()
print("Power Level: ", power, "%")
me.takeoff()
attitude = me.query_attitude()
print("Attitude: ", attitude, "%")
me.move_up(50)
me.move_down(25)
me.move_right(50)
me.land()
