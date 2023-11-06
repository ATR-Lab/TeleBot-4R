# ROS Params for all subsystems
- **system**: Represents the system the subsystem is contained in. Always snake_case. E.g. "telebot", "control"
- **level**: Represents the level of the subsystem. Formatted as "L_n" where n is the number of the level. E.g. "L_1", "L_4"
- **subsystem**: The name of the subsystem. Should be the same as the package name, and unique to the subsystem. Should always be in snake_case. E.g. "motion", "mapping".