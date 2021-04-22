# Topic descriptions

This document attempts to describe the functional meaning of all topics on milliAmpere.

## Topic `/rc_state`

Message type: `custom_msgs/RemoteControlState`

Describes the state of the RC controller.

- `left_stick`: `custom_msgs/JoyAxis`
    - Two `int16` which are the values of the x and y axes on the left control stick. What are the units and ranges?
- `right_stick`: `custom_msgs/JoyAxis`
    - Same as `left_stick`, but for right.
- `override_switch`: `uint8`
    - What are the meaning, range and units here?
- `safe_mode`: `uint8`
    - What are the meaning, range and units here?

## Topic `/actuator_ref_1`

Message type: `custom_msgs/ActuatorSetpoints`

Gives commands to the (front or back?) thruster.

- `throttle_reference`: `int16`
    - Command to send to Torqeedo motor. Value in [-1000, 1000], where its relationship to the Torqeedo's _motor_ is approximately _motor rpm_ = 5.8936 * `throttle_reference`, and the relationship to the _propeller_ is approximately _propeller rpm_ = 1.1787 * `throttle_reference`. Positive value is "forward".
- `angle_reference`: `float64`
    - Command to send to the servo motor which decides azimuth angle. Value in [-180, 180]?, unit is degrees. Neutral position is 0, which is currently when the force direction is "forward". A positive angle means that the force direction turns clockwise when seen from above.

## Topic `/actuator_ref_2`

Equivalent to `/actuator_ref_1`, but for the (front or back?) thruster.

## Topic `/actuators/actuator_1/motor_state`

Message type: `custom_msgs/MotorState`

Gives the state of the (front or back?) Torqeedo motor.

- `set_throttle_stop`: `bool`
	- Range and meaning?
- `motor_in_temp_limit`: `bool`
	- Range and meaning?
- `battery_nearly_empty`: `bool`
	- Range and meaning?
- `master_error_code`: `float64`
	- Range and meaning?
- `motor_voltage `: `float64`
	- Range and meaning?
- `motor_current`: `float64`
	- Range and meaning?
- `motor_speed`: `float64`
	- Speed of the _motor_, not the propeller. This value is in RPM, and is roughly in [-5400, 5300]. The gearing to the propeller is approximately 5:1 (propeller is slower).
- `motor_pcb_temp`: `float64`
	- Range and meaning?
- `motor_stator_temp`: `float64`
	- Range and meaning?
- `battery_voltage`: `float64`
	- Range and meaning?
- `battery_current`: `float64`
	- Range and meaning?
- `temperature_reverse_voltage_protection`: `float64`
	- Range and meaning?

## Topic `/actuators/actuator_2/motor_state`

Same as `/actuators/actuator_1/motor_state`, but for the (back or front?) thruster.

## Topic `/actuators/actuator_1/azimuth_angle`

Message type: `std_msgs/Float64`

Gives the angle of the servo motor for the (front or back?) azimuth thruster.

- `data`: `float64`
    - Value in [-180, 180]?, unit is degrees. Neutral position is 0, which is currently when the force direction is "forward". A positive angle means that the force direction turns clockwise when seen from above.

## Topic `/actuators/actuator_2/azimuth_angle`

Same as `/actuator_1/azimuth_angle`, but for the (back or front?) thruster.

## Topic `/navigation/pose`

Message type: `geometry_msgs/PoseStamped`

Gives the pose of the vessel (in a local NED frame). The frame ID and timestamp are given in the `header` field.

See the [ROS documentation](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) for details.

Hint: The orientation is given as a quaternion - this can be converted using the `euler_from_quaternion` function in `ma_utils`.

## Topic `/navigation/twist_body`

Message type: `geometry_msgs/TwistStamped`

Gives the body-decomposed velocity of the vessel. The frame ID and timestamp are given in the `header` field.
This is currently calculated by finite-differencing of the `/navigation/eta` information.

See the [ROS documentation](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html) for details.

## Topic `/navigation/twist_ned`

Message type: `geometry_msgs/TwistStamped`

Similar as `/navigation/twist_body`, but gives the velocity of the vessel in the `NED` frame.
The frame ID and timestamp are given in the `header` field.

TODO: Add more topics