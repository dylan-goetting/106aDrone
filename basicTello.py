from djitellopy import Tello
import time

me = Tello()
me.connect()
me.streamon()
#tello.takeoff()
frame_read = me.get_frame_read()
power = me.get_battery()
print("Power Level: ", power, "%")
me.takeoff()
#me.enable_mission_pads()
#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')
#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')

while True:
    frame = frame_read.frame
    c = input("enter u for up and d for down, q to end")
    if c == 'u':
        me.move_up(20)
    if c == 'd':
        me.move_down(20)
    if c == 'q':
        break

me.land()

#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')
# me.move_down(25)
# print(me.get_distance_tof())

# me.move_right(50)
#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')

