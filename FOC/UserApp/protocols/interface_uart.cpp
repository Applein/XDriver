#include "common_inc.h"
#include "configurations.h"

extern Motor motor;

/*
 * Lightweight decimal float parser.
 * Replaces sscanf("%f") which drags the whole newlib floating-point scanf
 * machinery (strtod/dtoa/gdtoa + ~12KB of UTF-8 conversion tables) into the
 * 128KB flash image. We only need "[-]ddd[.ddd]" here, so parse it manually.
 */
static bool ParseFloat(const char* _str, float& _out)
{
    if (_str == nullptr) return false;
    while (*_str == ' ' || *_str == '\t') _str++;

    float sign = 1.0f;
    if (*_str == '-') { sign = -1.0f; _str++; }
    else if (*_str == '+') { _str++; }
    if (*_str == '\0') return false;

    bool anyDigit = false;
    float intPart = 0.0f;
    while (*_str >= '0' && *_str <= '9')
    {
        intPart = intPart * 10.0f + (float) (*_str - '0');
        _str++;
        anyDigit = true;
    }

    float fracPart = 0.0f;
    float scale = 1.0f;
    if (*_str == '.')
    {
        _str++;
        while (*_str >= '0' && *_str <= '9')
        {
            fracPart = fracPart * 10.0f + (float) (*_str - '0');
            scale *= 10.0f;
            _str++;
            anyDigit = true;
        }
    }
    if (!anyDigit) return false;

    _out = sign * (intPart + fracPart / scale);
    return true;
}


void OnUartCmd(uint8_t* _data, uint16_t _len)
{
    float cur, pos, vel;

    switch (_data[0])
    {
        case 'c':
            if (!ParseFloat((char*) _data + 2, cur))
            {
                printf("[error] Command format error!\r\n");
            } else
            {
                if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT)
                    motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
                motor.controller->SetCurrentSetPoint((int32_t) (cur * 1000));
            }
            break;
        case 'v':
            if (!ParseFloat((char*) _data + 2, vel))
            {
                printf("[error] Command format error!\r\n");
            } else
            {
                if (motor.controller->modeRunning != Motor::MODE_COMMAND_VELOCITY)
                {
                    motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                    motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
                }
                motor.controller->SetVelocitySetPoint(
                    (int32_t) (vel * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            }
            break;
        case 'p':
            if (!ParseFloat((char*) _data + 2, pos))
            {
                printf("[error] Command format error!\r\n");
            } else
            {
                if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
                    motor.controller->requestMode = Motor::MODE_COMMAND_POSITION;

                motor.controller->SetPositionSetPoint(
                    (int32_t) (pos * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            }
            break;
    }
}

