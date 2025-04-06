#include "CtrlStepMotor.h"
#include "CtrlStepFrames.h"

CtrlStepMotor::CtrlStepMotor(HAL_CANHandle _hcan, uint8_t _id, bool _inverse,
                             uint8_t _reduction, float _angleLimitMin, float _angleLimitMax) :
        nodeID(_id), hcan(_hcan), inverseDirection(_inverse), reduction(_reduction),
        angleLimitMin(_angleLimitMin), angleLimitMax(_angleLimitMax)
{
//    txHeader =
//            {
//                    .StdId = 0,
//                    .ExtId = 0,
//                    .IDE = CAN_ID_STD,
//                    .RTR = CAN_RTR_DATA,
//                    .DLC = 8,
//                    .TransmitGlobalTime = DISABLE
//            };
}

void CtrlStepMotor::SetEnable(bool _enable)
{
    state = _enable ? FINISH : STOP;

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_ENABLE_MOTOR, &status);
}

void CtrlStepMotor::SetEnableTemp(bool _enable)
{
    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_ENABLE_MOTOR_TEMERATURE_WATCH, &status);
}

void CtrlStepMotor::DoCalibration()
{
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 0, CMD_API_DO_CALIBRATION, &status);
}

void CtrlStepMotor::SetCurrentSetPoint(float _val)
{
    state = RUNNING;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_CURRENT_SET_POINT, &status);
}


void CtrlStepMotor::SetVelocitySetPoint(float _val)
{
    state = RUNNING;


    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_VELOCITY_SET_POINT, &status);
}


void CtrlStepMotor::SetPositionSetPoint(float _val)
{
    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need ACK
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_POSITION_SET_POINT, &status);
}
void CtrlStepMotor::SetPositionWithVelocityLimit(float _pos, float _vel)
{
    // Float to Bytes
    auto* b = (unsigned char*) &_pos;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    b = (unsigned char*) &_vel;
    for (int i = 4; i < 8; i++)
        canBuf[i] = *(b + i - 4);
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_POSITION_WITH_VELOCITY_LIMIT, &status);
}


void CtrlStepMotor::SetNodeID(uint32_t _id)
{
    // Int to Bytes
    auto* b = (unsigned char*) &_id;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_NODE_ID, &status);
}


void CtrlStepMotor::SetCurrentLimit(float _val)
{
    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_CURRENT_LIMIT, &status);
}


void CtrlStepMotor::SetVelocityLimit(float _val)
{
    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_ENABLE_MOTOR_TEMERATURE_WATCH, &status);
}

void CtrlStepMotor::SetAcceleration(float _val)
{
    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 0; // Need save to EEPROM or not
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_ACCELERATION, &status);
}

void CtrlStepMotor::ApplyPositionAsHome()
{
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_APPLY_HOME_POSITION, &status);
}

void CtrlStepMotor::SetEnableOnBoot(bool _enable)
{
    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_AUTO_ENABLE, &status);
}


void CtrlStepMotor::SetEnableStallProtect(bool _enable)
{
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_ENABLE_STALL_PROTECT, &status);
}

void CtrlStepMotor::Reboot()
{
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_REBOOT, &status);
}

uint32_t CtrlStepMotor::GetTemp()
{
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_GET_TEMPERATURE, &status);
    return temperature;
}

void CtrlStepMotor::EraseConfigs()
{
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_ERASE_CONFIGS, &status);
}


void CtrlStepMotor::SetAngle(float _angle)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionSetPoint(stepMotorCnt);
}

//0x7d~0xFF MISC CMDs
const int CMD_API_ENABLE_MOTOR_TEMERATURE_WATCH= 0x7d;  // enable motor temperature watch


void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionWithVelocityLimit(stepMotorCnt, _vel);
}


void CtrlStepMotor::UpdateAngle()
{
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_ENABLE_MOTOR_TEMERATURE_WATCH, &status);
}


void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;

    float tmp = _pos / (float) reduction * 360;
    angle = inverseDirection ? -tmp : tmp;
}


void CtrlStepMotor::SetDceKp(int32_t _val)
{
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_DEC_KP, &status);
}


void CtrlStepMotor::SetDceKv(int32_t _val)
{
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_DEC_KV, &status);
}


void CtrlStepMotor::SetDceKi(int32_t _val)
{
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_DEC_KI, &status);
}


void CtrlStepMotor::SetDceKd(int32_t _val)
{
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    int32_t  status;
    HAL_WriteCANPacket(m_canHandle, canBuf, 4, CMD_API_SET_DEC_KD, &status);
}
