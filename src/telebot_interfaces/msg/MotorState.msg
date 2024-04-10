float64 timestamp   # motor state is at this time
int32 id            # motor id
int32 goal          # commanded position (in encoder units)
float32 position      # current position (in radians) NOTE: Not real radians see documentation for radians in Motion package
int32 error         # difference between current and goal positions
int32 speed         # current speed (0.111 rpm per unit)
float32 load        # current load - ratio of applied torque over maximum torque
float64 voltage     # current voltage (V)
int32 temperature   # current temperature (degrees Celsius)
bool moving         # whether the motor is currently in motion