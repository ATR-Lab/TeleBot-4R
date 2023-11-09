#ifndef DYNAMIXEL_ADDRESSES_HPP
#define DYNAMIXEL_ADDRESSES_HPP
// This defines a bunch of constants that are for our specific motors
namespace DynamixelAddresses
{
    namespace Pro
    {
        //Persistent memory
        namespace EEPROM
        {
            // Address and Size(Byte)
            #define MODEL_NUMBER_ADDR 0
            #define MODEL_NUMBER_SIZE 2

            #define MODEL_INFORMATION_ADDR 2
            #define MODEL_INFORMATION_SIZE 4

            #define FIRMWARE_VERSION_ADDR 6
            #define FIRMWARE_VERSION_SIZE 1

            #define ID_ADDR 7
            #define ID_SIZE 1

            #define BAUD_RATE_ADDR 8
            #define BAUD_RATE_SIZE 1

            #define RETURN_DELAY_TIME_ADDR 9
            #define RETURN_DELAY_TIME_SIZE 1

            #define OPERATING_MODE_ADDR 11
            #define OPERATING_MODE_SIZE 1

            #define HOMING_OFFSET_ADDR 13
            #define HOMING_OFFSET_SIZE 4

            #define MOVING_THRESHOLD_ADDR 17
            #define MOVING_THRESHOLD_SIZE 4

            #define TEMPERATURE_LIMIT_ADDR 21
            #define TEMPERATURE_LIMIT_SIZE 1

            #define MAX_VOLTAGE_LIMIT_ADDR 22
            #define MAX_VOLTAGE_LIMIT_SIZE 2

            #define MIN_VOLTAGE_LIMIT_ADDR 24
            #define MIN_VOLTAGE_LIMIT_SIZE 2

            #define ACCELERATION_LIMIT_ADDR 26
            #define ACCELERATION_LIMIT_SIZE 4

            #define TORQUE_LIMIT_ADDR 30
            #define TORQUE_LIMIT_SIZE 2

            #define VELOCITY_LIMIT_ADDR 32
            #define VELOCITY_LIMIT_SIZE 4

            #define MAX_POSITION_LIMIT_ADDR 36
            #define MAX_POSITION_LIMIT_SIZE 4

            #define MIN_POSITION_LIMIT_ADDR 40
            #define MIN_POSITION_LIMIT_SIZE 4

            #define EXTERNAL_PORT_MODE_1_ADDR 44
            #define EXTERNAL_PORT_MODE_1_SIZE 1

            #define EXTERNAL_PORT_MODE_2_ADDR 45
            #define EXTERNAL_PORT_MODE_2_SIZE 1

            #define EXTERNAL_PORT_MODE_3_ADDR 46
            #define EXTERNAL_PORT_MODE_3_SIZE 1

            #define EXTERNAL_PORT_MODE_4_ADDR 47
            #define EXTERNAL_PORT_MODE_4_SIZE 1

            #define SHUTDOWN_ADDR 48
            #define SHUTDOWN_SIZE 1
            
        }
        //Non-persistent memory
        namespace RAM{
            #define TORQUE_ENABLE_ADDR_2 562
            #define TORQUE_ENABLE_SIZE_2 1

            #define LED_RED_ADDR_2 563
            #define LED_RED_SIZE_2 1

            #define LED_GREEN_ADDR_2 564
            #define LED_GREEN_SIZE_2 1

            #define LED_BLUE_ADDR_2 565
            #define LED_BLUE_SIZE_2 1

            #define VELOCITY_I_GAIN_ADDR_2 586
            #define VELOCITY_I_GAIN_SIZE_2 2

            #define VELOCITY_P_GAIN_ADDR_2 588
            #define VELOCITY_P_GAIN_SIZE_2 2

            #define POSITION_P_GAIN_ADDR_2 594
            #define POSITION_P_GAIN_SIZE_2 2

            #define GOAL_POSITION_ADDR_2 596
            #define GOAL_POSITION_SIZE_2 4

            #define GOAL_VELOCITY_ADDR_2 600
            #define GOAL_VELOCITY_SIZE_2 4

            #define GOAL_TORQUE_ADDR_2 604
            #define GOAL_TORQUE_SIZE_2 2

            #define GOAL_ACCELERATION_ADDR_2 606
            #define GOAL_ACCELERATION_SIZE_2 4

            #define MOVING_ADDR_2 610
            #define MOVING_SIZE_2 1

            #define PRESENT_POSITION_ADDR_2 611
            #define PRESENT_POSITION_SIZE_2 4

            #define PRESENT_VELOCITY_ADDR_2 615
            #define PRESENT_VELOCITY_SIZE_2 4

            #define PRESENT_CURRENT_ADDR_2 621
            #define PRESENT_CURRENT_SIZE_2 2

            #define PRESENT_INPUT_VOLTAGE_ADDR_2 623
            #define PRESENT_INPUT_VOLTAGE_SIZE_2 2

            #define PRESENT_TEMPERATURE_ADDR_2 625
            #define PRESENT_TEMPERATURE_SIZE_2 1

            #define EXTERNAL_PORT_DATA_1_ADDR_2 626
            #define EXTERNAL_PORT_DATA_1_SIZE_2 2

            #define EXTERNAL_PORT_DATA_2_ADDR_2 628
            #define EXTERNAL_PORT_DATA_2_SIZE_2 2

            #define EXTERNAL_PORT_DATA_3_ADDR_2 630
            #define EXTERNAL_PORT_DATA_3_SIZE_2 2

            #define EXTERNAL_PORT_DATA_4_ADDR_2 632
            #define EXTERNAL_PORT_DATA_4_SIZE_2 2

            #define HARDWARE_ERROR_STATUS_ADDR_2 892
            #define HARDWARE_ERROR_STATUS_SIZE_2 1

        }
    }
    namespace XM540
    {
        namespace EEPROM
        {
            // Address and Size(Byte)
            #define MODEL_NUMBER_ADDR_2 0
            #define MODEL_NUMBER_SIZE_2 2

            #define MODEL_INFORMATION_ADDR_2 2
            #define MODEL_INFORMATION_SIZE_2 4

            #define FIRMWARE_VERSION_ADDR_2 6
            #define FIRMWARE_VERSION_SIZE_2 1

            #define ID_ADDR_2 7
            #define ID_SIZE_2 1

            #define BAUD_RATE_ADDR_2 8
            #define BAUD_RATE_SIZE_2 1

            #define RETURN_DELAY_TIME_ADDR_2 9
            #define RETURN_DELAY_TIME_SIZE_2 1

            #define DRIVE_MODE_ADDR_2 10
            #define DRIVE_MODE_SIZE_2 1

            #define OPERATING_MODE_ADDR_2 11
            #define OPERATING_MODE_SIZE_2 1

            #define SECONDARY_ID_ADDR_2 12
            #define SECONDARY_ID_SIZE_2 1

            #define PROTOCOL_TYPE_ADDR_2 13
            #define PROTOCOL_TYPE_SIZE_2 1

            #define HOMING_OFFSET_ADDR_2 20
            #define HOMING_OFFSET_SIZE_2 4

            #define MOVING_THRESHOLD_ADDR_2 24
            #define MOVING_THRESHOLD_SIZE_2 4

            #define TEMPERATURE_LIMIT_ADDR_2 31
            #define TEMPERATURE_LIMIT_SIZE_2 1

            #define MAX_VOLTAGE_LIMIT_ADDR_2 32
            #define MAX_VOLTAGE_LIMIT_SIZE_2 2

            #define MIN_VOLTAGE_LIMIT_ADDR_2 34
            #define MIN_VOLTAGE_LIMIT_SIZE_2 2

            #define PWM_LIMIT_ADDR_2 36
            #define PWM_LIMIT_SIZE_2 2

            #define CURRENT_LIMIT_ADDR_2 38
            #define CURRENT_LIMIT_SIZE_2 2

            #define VELOCITY_LIMIT_ADDR_2 44
            #define VELOCITY_LIMIT_SIZE_2 4

            #define MAX_POSITION_LIMIT_ADDR_2 48
            #define MAX_POSITION_LIMIT_SIZE_2 4

            #define MIN_POSITION_LIMIT_ADDR_2 52
            #define MIN_POSITION_LIMIT_SIZE_2 4

            #define EXTERNAL_PORT_MODE_1_ADDR_2 56
            #define EXTERNAL_PORT_MODE_1_SIZE_2 1

            #define EXTERNAL_PORT_MODE_2_ADDR_2 57
            #define EXTERNAL_PORT_MODE_2_SIZE_2 1

            #define EXTERNAL_PORT_MODE_3_ADDR_2 58
            #define EXTERNAL_PORT_MODE_3_SIZE_2 1

            #define STARTUP_CONFIGURATION_ADDR_2 60
            #define STARTUP_CONFIGURATION_SIZE_2 1

            #define SHUTDOWN_ADDR_2 63
            #define SHUTDOWN_SIZE_2 1
            
        }
        //Non-persistent memory
        namespace RAM{
            // Address and Size(Byte)
            #define TORQUE_ENABLE_ADDR 64
            #define TORQUE_ENABLE_SIZE 1

            #define LED_ADDR 65
            #define LED_SIZE 1

            #define STATUS_RETURN_LEVEL_ADDR 68
            #define STATUS_RETURN_LEVEL_SIZE 1

            #define REGISTERED_INSTRUCTION_ADDR 69
            #define REGISTERED_INSTRUCTION_SIZE 1

            #define HARDWARE_ERROR_STATUS_ADDR 70
            #define HARDWARE_ERROR_STATUS_SIZE 1

            #define VELOCITY_I_GAIN_ADDR 76
            #define VELOCITY_I_GAIN_SIZE 2

            #define VELOCITY_P_GAIN_ADDR 78
            #define VELOCITY_P_GAIN_SIZE 2

            #define POSITION_D_GAIN_ADDR 80
            #define POSITION_D_GAIN_SIZE 2

            #define POSITION_I_GAIN_ADDR 82
            #define POSITION_I_GAIN_SIZE 2

            #define POSITION_P_GAIN_ADDR 84
            #define POSITION_P_GAIN_SIZE 2

            #define FEEDFORWARD_2ND_GAIN_ADDR 88
            #define FEEDFORWARD_2ND_GAIN_SIZE 2

            #define FEEDFORWARD_1ST_GAIN_ADDR 90
            #define FEEDFORWARD_1ST_GAIN_SIZE 2

            #define BUS_WATCHDOG_ADDR 98
            #define BUS_WATCHDOG_SIZE 1

            #define GOAL_PWM_ADDR 100
            #define GOAL_PWM_SIZE 2

            #define GOAL_CURRENT_ADDR 102
            #define GOAL_CURRENT_SIZE 2

            #define GOAL_VELOCITY_ADDR 104
            #define GOAL_VELOCITY_SIZE 4

            #define PROFILE_ACCELERATION_ADDR 108
            #define PROFILE_ACCELERATION_SIZE 4

            #define PROFILE_VELOCITY_ADDR 112
            #define PROFILE_VELOCITY_SIZE 4

            #define GOAL_POSITION_ADDR 116
            #define GOAL_POSITION_SIZE 4

            #define REALTIME_TICK_ADDR 120
            #define REALTIME_TICK_SIZE 2

            #define MOVING_ADDR 122
            #define MOVING_SIZE 1

            #define MOVING_STATUS_ADDR 123
            #define MOVING_STATUS_SIZE 1

            #define PRESENT_PWM_ADDR 124
            #define PRESENT_PWM_SIZE 2

            #define PRESENT_CURRENT_ADDR 126
            #define PRESENT_CURRENT_SIZE 2

            #define PRESENT_VELOCITY_ADDR 128
            #define PRESENT_VELOCITY_SIZE 4

            #define PRESENT_POSITION_ADDR 132
            #define PRESENT_POSITION_SIZE 4

            #define VELOCITY_TRAJECTORY_ADDR 136
            #define VELOCITY_TRAJECTORY_SIZE 4

            #define POSITION_TRAJECTORY_ADDR 140
            #define POSITION_TRAJECTORY_SIZE 4

            #define PRESENT_INPUT_VOLTAGE_ADDR 144
            #define PRESENT_INPUT_VOLTAGE_SIZE 2

            #define PRESENT_TEMPERATURE_ADDR 146
            #define PRESENT_TEMPERATURE_SIZE 1

            #define BACKUP_READY_ADDR 147
            #define BACKUP_READY_SIZE 1

            #define EXTERNAL_PORT_DATA_1_ADDR 152
            #define EXTERNAL_PORT_DATA_1_SIZE 2

            #define EXTERNAL_PORT_DATA_2_ADDR 154
            #define EXTERNAL_PORT_DATA_2_SIZE 2

            #define EXTERNAL_PORT_DATA_3_ADDR 156
            #define EXTERNAL_PORT_DATA_3_SIZE 2
        }

    }
}
#endif