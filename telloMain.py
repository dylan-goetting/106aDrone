from djitellopy import Tello
import time

me = Tello()
me.connect()
me.streamon()

frame_read = me.get_frame_read()
power = me.get_battery()
print("Power Level: ", power, "%")
me.initiate_throw_takeoff()
# me.takeoff()

while True:
    frame = frame_read.frame
    c = input("enter r for up and f for down, g to end")
    if c == 'r':
        # me.move_up(20)
        me.go_xyz_speed("z", 10)
    if c == 'f':
        me.move_down(20)
    if c == 'w':
        # me.move_forward(20)
        me.go_xyz_speed(40, 0, 0, 11)
        time.sleep(2)
        me.send_rc_control(0, 0, 0, 0)
    if c == 's':
        # me.move_back(20)
        me.go_xyz_speed(-50, 0, 0, 10)
        time.sleep(2)
        me.stop();
    if c == 'a':
        me.move_left(20)
    if c == 'd':
        me.move_right(20)
    if c == 'q':
        me.rotate_counter_clockwise(45)
    if c == 'e':
        me.rotate_clockwise(45)
    if c == 'g':
        break

me.land()

#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')
# me.move_down(25)
# print(me.get_distance_tof())

# me.move_right(50)
#print(f'X: {me.get_mission_pad_distance_x}, Y: {me.get_mission_pad_distance_y}, Z: {me.get_mission_pad_distance_z}')

