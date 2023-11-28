from djitellopy import Tello


me = Tello()
me.connect()
power = me.get_battery()
print("Power Level: ", power, "%")
me.takeoff()
#me.enable_mission_pads()
#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')
attitude = me.query_attitude()
print("Attitude: ", attitude, "%")
print(me.get_distance_tof())
#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')
me.move_up(50)
print(me.get_distance_tof())

#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')
me.move_down(25)
print(me.get_distance_tof())

me.move_right(50)
#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')

me.land()
