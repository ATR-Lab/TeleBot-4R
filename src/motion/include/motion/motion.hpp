#ifndef MOTION_HPP
#define MOTION_HPP

namespace Motion
{
    enum MovementType
    {
        POSITION = 'P',
        PWM = 'T',
        VELOCITY = 'V',
        TORQUE='Q'
    };
}

#endif