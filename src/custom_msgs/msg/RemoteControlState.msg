# Custom message to hold the state of the joystick at a given time
# Left stick contains the axis values of the left stick, as a JoyAxis custom message
# Right stick contains the axis values of the right stick, as a JoyAxis custom message
# override_switch holds the position of the override switch, which can be 0, 1 or 2

Header header

custom_msgs/JoyAxis left_stick
custom_msgs/JoyAxis right_stick
uint8 override_switch
uint8 safe_mode