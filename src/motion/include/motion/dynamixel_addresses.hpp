#ifndef DYNAMIXEL_ADDRESSES_HPP
#define DYNAMIXEL_ADDRESSES_HPP

#include <cstdint>

namespace DynamixelAddresses
{
    namespace Pro
    {
        //Persistent memory
        namespace EEPROM
        {
            // Address and Size(Byte)
            const uint16_t MODEL_NUMBER_ADDR = 0;
            const uint16_t MODEL_NUMBER_SIZE = 2;

            const uint16_t MODEL_INFORMATION_ADDR = 2;
            const uint16_t MODEL_INFORMATION_SIZE = 4;

            const uint16_t FIRMWARE_VERSION_ADDR = 6;
            const uint16_t FIRMWARE_VERSION_SIZE = 1;

            const uint16_t ID_ADDR = 7;
            const uint16_t ID_SIZE = 1;

            const uint16_t BAUD_RATE_ADDR = 8;
            const uint16_t BAUD_RATE_SIZE = 1;

            const uint16_t RETURN_DELAY_TIME_ADDR = 9;
            const uint16_t RETURN_DELAY_TIME_SIZE = 1;

            const uint16_t OPERATING_MODE_ADDR = 11;
            const uint16_t OPERATING_MODE_SIZE = 1;

            const uint16_t HOMING_OFFSET_ADDR = 13;
            const uint16_t HOMING_OFFSET_SIZE = 4;

            const uint16_t MOVING_THRESHOLD_ADDR = 17;
            const uint16_t MOVING_THRESHOLD_SIZE = 4;

            const uint16_t TEMPERATURE_LIMIT_ADDR = 21;
            const uint16_t TEMPERATURE_LIMIT_SIZE = 1;

            const uint16_t MAX_VOLTAGE_LIMIT_ADDR = 22;
            const uint16_t MAX_VOLTAGE_LIMIT_SIZE = 2;

            const uint16_t MIN_VOLTAGE_LIMIT_ADDR = 24;
            const uint16_t MIN_VOLTAGE_LIMIT_SIZE = 2;

            const uint16_t ACCELERATION_LIMIT_ADDR = 26;
            const uint16_t ACCELERATION_LIMIT_SIZE = 4;

            const uint16_t TORQUE_LIMIT_ADDR = 30;
            const uint16_t TORQUE_LIMIT_SIZE = 2;

            const uint16_t VELOCITY_LIMIT_ADDR = 32;
            const uint16_t VELOCITY_LIMIT_SIZE = 4;

            const uint16_t MAX_POSITION_LIMIT_ADDR = 36;
            const uint16_t MAX_POSITION_LIMIT_SIZE = 4;

            const uint16_t MIN_POSITION_LIMIT_ADDR = 40;
            const uint16_t MIN_POSITION_LIMIT_SIZE = 4;

            const uint16_t EXTERNAL_PORT_MODE_1_ADDR = 44;
            const uint16_t EXTERNAL_PORT_MODE_1_SIZE = 1;

            const uint16_t EXTERNAL_PORT_MODE_2_ADDR = 45;
            const uint16_t EXTERNAL_PORT_MODE_2_SIZE = 1;

            const uint16_t EXTERNAL_PORT_MODE_3_ADDR = 46;
            const uint16_t EXTERNAL_PORT_MODE_3_SIZE = 1;

            const uint16_t EXTERNAL_PORT_MODE_4_ADDR = 47;
            const uint16_t EXTERNAL_PORT_MODE_4_SIZE = 1;

            const uint16_t SHUTDOWN_ADDR = 48;
            const uint16_t SHUTDOWN_SIZE = 1;
        }
        //Non-persistent memory
        namespace RAM
        {
            const uint16_t TORQUE_ENABLE_ADDR = 562;
            const uint16_t TORQUE_ENABLE_SIZE = 1;

            const uint16_t LED_RED_ADDR = 563;
            const uint16_t LED_RED_SIZE = 1;

            const uint16_t LED_GREEN_ADDR = 564;
            const uint16_t LED_GREEN_SIZE = 1;

            const uint16_t LED_BLUE_ADDR = 565;
            const uint16_t LED_BLUE_SIZE = 1;

            const uint16_t VELOCITY_I_GAIN_ADDR = 586;
            const uint16_t VELOCITY_I_GAIN_SIZE = 2;

            const uint16_t VELOCITY_P_GAIN_ADDR = 588;
            const uint16_t VELOCITY_P_GAIN_SIZE = 2;

            const uint16_t POSITION_P_GAIN_ADDR = 594;
            const uint16_t POSITION_P_GAIN_SIZE = 2;

            const uint16_t GOAL_POSITION_ADDR = 596;
            const uint16_t GOAL_POSITION_SIZE = 4;

            const uint16_t GOAL_VELOCITY_ADDR = 600;
            const uint16_t GOAL_VELOCITY_SIZE = 4;

            const uint16_t GOAL_TORQUE_ADDR = 604;
            const uint16_t GOAL_TORQUE_SIZE = 2;

            const uint16_t GOAL_ACCELERATION_ADDR = 606;
            const uint16_t GOAL_ACCELERATION_SIZE = 4;

            const uint16_t MOVING_ADDR = 610;
            const uint16_t MOVING_SIZE = 1;

            const uint16_t PRESENT_POSITION_ADDR = 611;
            const uint16_t PRESENT_POSITION_SIZE = 4;

            const uint16_t PRESENT_VELOCITY_ADDR = 615;
            const uint16_t PRESENT_VELOCITY_SIZE = 4;

            const uint16_t PRESENT_CURRENT_ADDR = 621;
            const uint16_t PRESENT_CURRENT_SIZE = 2;

            const uint16_t PRESENT_INPUT_VOLTAGE_ADDR = 623;
            const uint16_t PRESENT_INPUT_VOLTAGE_SIZE = 2;

            const uint16_t PRESENT_TEMPERATURE_ADDR = 625;
            const uint16_t PRESENT_TEMPERATURE_SIZE = 1;

            const uint16_t EXTERNAL_PORT_DATA_1_ADDR = 626;
            const uint16_t EXTERNAL_PORT_DATA_1_SIZE = 2;

            const uint16_t EXTERNAL_PORT_DATA_ADDR = 628;
            const uint16_t EXTERNAL_PORT_DATA_SIZE = 2;

            const uint16_t EXTERNAL_PORT_DATA_3_ADDR = 630;
            const uint16_t EXTERNAL_PORT_DATA_3_SIZE = 2;

            const uint16_t EXTERNAL_PORT_DATA_4_ADDR = 632;
            const uint16_t EXTERNAL_PORT_DATA_4_SIZE = 2;

            const uint16_t HARDWARE_ERROR_STATUS_ADDR = 892;
            const uint16_t HARDWARE_ERROR_STATUS_SIZE = 1;
        }
    }
    namespace XM540
    {
        namespace EEPROM
        {
            // Address and Size(Byte)
            const uint16_t MODEL_NUMBER_ADDR = 0;
            const uint16_t MODEL_NUMBER_SIZE = 2;

            const uint16_t MODEL_INFORMATION_ADDR = 2;
            const uint16_t MODEL_INFORMATION_SIZE = 4;

            const uint16_t FIRMWARE_VERSION_ADDR = 6;
            const uint16_t FIRMWARE_VERSION_SIZE = 1;

            const uint16_t ID_ADDR = 7;
            const uint16_t ID_SIZE = 1;

            const uint16_t BAUD_RATE_ADDR = 8;
            const uint16_t BAUD_RATE_SIZE = 1;

            const uint16_t RETURN_DELAY_TIME_ADDR = 9;
            const uint16_t RETURN_DELAY_TIME_SIZE = 1;

            const uint16_t DRIVE_MODE_ADDR = 10;
            const uint16_t DRIVE_MODE_SIZE = 1;

            const uint16_t OPERATING_MODE_ADDR = 11;
            const uint16_t OPERATING_MODE_SIZE = 1;

            const uint16_t SECONDARY_ID_ADDR = 12;
            const uint16_t SECONDARY_ID_SIZE = 1;

            const uint16_t PROTOCOL_TYPE_ADDR = 13;
            const uint16_t PROTOCOL_TYPE_SIZE = 1;

            const uint16_t HOMING_OFFSET_ADDR = 20;
            const uint16_t HOMING_OFFSET_SIZE = 4;

            const uint16_t MOVING_THRESHOLD_ADDR = 24;
            const uint16_t MOVING_THRESHOLD_SIZE = 4;

            const uint16_t TEMPERATURE_LIMIT_ADDR = 31;
            const uint16_t TEMPERATURE_LIMIT_SIZE = 1;

            const uint16_t MAX_VOLTAGE_LIMIT_ADDR = 32;
            const uint16_t MAX_VOLTAGE_LIMIT_SIZE = 2;

            const uint16_t MIN_VOLTAGE_LIMIT_ADDR = 34;
            const uint16_t MIN_VOLTAGE_LIMIT_SIZE = 2;

            const uint16_t PWM_LIMIT_ADDR = 36;
            const uint16_t PWM_LIMIT_SIZE = 2;

            const uint16_t CURRENT_LIMIT_ADDR = 38;
            const uint16_t CURRENT_LIMIT_SIZE = 2;

            const uint16_t VELOCITY_LIMIT_ADDR = 44;
            const uint16_t VELOCITY_LIMIT_SIZE = 4;

            const uint16_t MAX_POSITION_LIMIT_ADDR = 48;
            const uint16_t MAX_POSITION_LIMIT_SIZE = 4;

            const uint16_t MIN_POSITION_LIMIT_ADDR = 52;
            const uint16_t MIN_POSITION_LIMIT_SIZE = 4;

            const uint16_t EXTERNAL_PORT_MODE_1_ADDR = 56;
            const uint16_t EXTERNAL_PORT_MODE_1_SIZE = 1;

            const uint16_t EXTERNAL_PORT_MODE_2_ADDR = 57;
            const uint16_t EXTERNAL_PORT_MODE_2_SIZE = 1;

            const uint16_t EXTERNAL_PORT_MODE_3_ADDR = 58;
            const uint16_t EXTERNAL_PORT_MODE_3_SIZE = 1;

            const uint16_t STARTUP_CONFIGURATION_ADDR = 60;
            const uint16_t STARTUP_CONFIGURATION_SIZE = 1;

            const uint16_t SHUTDOWN_ADDR = 63;
            const uint16_t SHUTDOWN_SIZE = 1;
        }
        //Non-persistent memory
        namespace RAM
        {
            // Address and Size(Byte)
            const uint16_t TORQUE_ENABLE_ADDR = 64;
            const uint16_t TORQUE_ENABLE_SIZE = 1;

            const uint16_t LED_ADDR = 65;
            const uint16_t LED_SIZE = 1;

            const uint16_t STATUS_RETURN_LEVEL_ADDR = 68;
            const uint16_t STATUS_RETURN_LEVEL_SIZE = 1;

            const uint16_t REGISTERED_INSTRUCTION_ADDR = 69;
            const uint16_t REGISTERED_INSTRUCTION_SIZE = 1;

            const uint16_t HARDWARE_ERROR_STATUS_ADDR = 70;
            const uint16_t HARDWARE_ERROR_STATUS_SIZE = 1;

            const uint16_t VELOCITY_I_GAIN_ADDR = 76;
            const uint16_t VELOCITY_I_GAIN_SIZE = 2;

            const uint16_t VELOCITY_P_GAIN_ADDR = 78;
            const uint16_t VELOCITY_P_GAIN_SIZE = 2;

            const uint16_t POSITION_D_GAIN_ADDR = 80;
            const uint16_t POSITION_D_GAIN_SIZE = 2;

            const uint16_t POSITION_I_GAIN_ADDR = 82;
            const uint16_t POSITION_I_GAIN_SIZE = 2;

            const uint16_t POSITION_P_GAIN_ADDR = 84;
            const uint16_t POSITION_P_GAIN_SIZE = 2;

            const uint16_t FEEDFORWARD_2ND_GAIN_ADDR = 88;
            const uint16_t FEEDFORWARD_2ND_GAIN_SIZE = 2;

            const uint16_t FEEDFORWARD_1ST_GAIN_ADDR = 90;
            const uint16_t FEEDFORWARD_1ST_GAIN_SIZE = 2;

            const uint16_t BUS_WATCHDOG_ADDR = 98;
            const uint16_t BUS_WATCHDOG_SIZE = 1;

            const uint16_t GOAL_PWM_ADDR = 100;
            const uint16_t GOAL_PWM_SIZE = 2;

            const uint16_t GOAL_CURRENT_ADDR = 102;
            const uint16_t GOAL_CURRENT_SIZE = 2;

            const uint16_t GOAL_VELOCITY_ADDR = 104;
            const uint16_t GOAL_VELOCITY_SIZE = 4;

            const uint16_t PROFILE_ACCELERATION_ADDR = 108;
            const uint16_t PROFILE_ACCELERATION_SIZE = 4;

            const uint16_t PROFILE_VELOCITY_ADDR = 112;
            const uint16_t PROFILE_VELOCITY_SIZE = 4;

            const uint16_t GOAL_POSITION_ADDR = 116;
            const uint16_t GOAL_POSITION_SIZE = 4;

            const uint16_t REALTIME_TICK_ADDR = 120;
            const uint16_t REALTIME_TICK_SIZE = 2;

            const uint16_t MOVING_ADDR = 122;
            const uint16_t MOVING_SIZE = 1;

            const uint16_t MOVING_STATUS_ADDR = 123;
            const uint16_t MOVING_STATUS_SIZE = 1;

            const uint16_t PRESENT_PWM_ADDR = 124;
            const uint16_t PRESENT_PWM_SIZE = 2;

            const uint16_t PRESENT_CURRENT_ADDR = 126;
            const uint16_t PRESENT_CURRENT_SIZE = 2;

            const uint16_t PRESENT_VELOCITY_ADDR = 128;
            const uint16_t PRESENT_VELOCITY_SIZE = 4;

            const uint16_t PRESENT_POSITION_ADDR = 132;
            const uint16_t PRESENT_POSITION_SIZE = 4;

            const uint16_t VELOCITY_TRAJECTORY_ADDR = 136;
            const uint16_t VELOCITY_TRAJECTORY_SIZE = 4;

            const uint16_t POSITION_TRAJECTORY_ADDR = 140;
            const uint16_t POSITION_TRAJECTORY_SIZE = 4;

            const uint16_t PRESENT_INPUT_VOLTAGE_ADDR= 144;
            const uint16_t PRESENT_INPUT_VOLTAGE_SIZE =2;

            const uint16_t PRESENT_TEMPERATURE_ADDR =146;
            const uint16_t PRESENT_TEMPERATURE_SIZE =1;

            const uint16_t BACKUP_READY_ADDR =147;
            const uint16_t BACKUP_READY_SIZE =1;

            const uint16_t EXTERNAL_PORT_DATA_1_ADDR =152;
            const uint16_t EXTERNAL_PORT_DATA_1_SIZE =2;

            const uint16_t EXTERNAL_PORT_DATA_2_ADDR =154;
            const uint16_t EXTERNAL_PORT_DATA_2_SIZE =2;

            const uint16_t EXTERNAL_PORT_DATA_3_ADDR =156;
            const uint16_t EXTERNAL_PORT_DATA_3_SIZE =2;
        }

    }
}
#endif