# Custom message for holding the motor state
# from torqeedo motors

bool set_throttle_stop
bool setup_allowed
bool in_charge
bool in_setup
bool bank_avaiable
bool no_menu
bool menu_off
bool motor_in_temp_limit
bool battery_charge_valid
bool battery_nearly_empty
bool battery_charging
bool range_miles_valid
bool range_minutes_valid

float64 master_error_code
float64 motor_voltage 
float64 motor_current
float64 motor_power
float64 motor_speed
float64 motor_pcb_temp
float64 motor_stator_temp
float64 battery_charge  # Estimated remaining charge in percent. Only valid when battery_charge_valid is set
float64 battery_voltage
float64 battery_current
float64 range_miles  # Estimated range in nautical miles
float64 range_minutes  # Estimated range in minutes
float64 temperature_motor_power_switches  # Temperature of the master PCB close to the motor power switches
float64 temperature_reverse_voltage_protection  # Temperature of the master PCB close to the reverse voltage protection circuit
