#include "CtrlStepMotor.h"
#include "CtrlStepFrames.h"
#define CAN_ID_STD                  (0x00000000U)  /*!< Standard Id */
#define CAN_RTR_DATA                (0x00000000U)  /*!< Data frame   */

CtrlStepMotor::CtrlStepMotor( uint8_t _id, bool _inverse,
                             uint8_t _reduction, float _angleLimitMin, float _angleLimitMax) :
        deviceId(_id), inverseDirection(_inverse), reduction(_reduction),
        angleLimitMin(_angleLimitMin), angleLimitMax(_angleLimitMax)
{
    m_canHandle = std::make_shared<CAN>(deviceId);
    txHeader =
            {
                    .StdId = 0,
                    .ExtId = 0,
                    .IDE = CAN_ID_STD,
                    .RTR = CAN_RTR_DATA,
                    .DLC = 8
            };
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
    m_canHandle->WritePacket(canBuf, 4, CMD_API_ENABLE_MOTOR);
}

void CtrlStepMotor::SetEnableTemp(bool _enable)
{
    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    int32_t  status;
    m_canHandle->WritePacket(canBuf, 4, CMD_API_ENABLE_MOTOR_TEMERATURE_WATCH);
}

void CtrlStepMotor::DoCalibration()
{
    m_canHandle->WritePacket(canBuf, 0, CMD_API_DO_CALIBRATION);
}

void CtrlStepMotor::SetCurrentSetPoint(float _val)
{
    state = RUNNING;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_CURRENT_SET_POINT);
}

void CtrlStepMotor::SetVelocitySetPoint(float _val)
{
    state = RUNNING;


    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_VELOCITY_SET_POINT);
}

void CtrlStepMotor::SetPositionSetPoint(float _val)
{
    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need ACK

    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_POSITION_SET_POINT);
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

    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_POSITION_WITH_VELOCITY_LIMIT);
}

void CtrlStepMotor::SetNodeID(uint32_t _id)
{
    // Int to Bytes
    auto* b = (unsigned char*) &_id;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_NODE_ID);
}

void CtrlStepMotor::SetCurrentLimit(float _val)
{
    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_CURRENT_LIMIT);
}

void CtrlStepMotor::SetVelocityLimit(float _val)
{
    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    m_canHandle->WritePacket(canBuf, 4, CMD_API_ENABLE_MOTOR_TEMERATURE_WATCH);
}

void CtrlStepMotor::SetAcceleration(float _val)
{
    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 0; // Need save to EEPROM or not
    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_ACCELERATION);
}

void CtrlStepMotor::ApplyPositionAsHome()
{
    m_canHandle->WritePacket(canBuf, 4, CMD_API_APPLY_HOME_POSITION);
}

void CtrlStepMotor::SetEnableOnBoot(bool _enable)
{
    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_AUTO_ENABLE);
}

void CtrlStepMotor::SetEnableStallProtect(bool _enable)
{
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_ENABLE_STALL_PROTECT);
}

void CtrlStepMotor::Reboot()
{
    m_canHandle->WritePacket(canBuf, 4, CMD_API_REBOOT );
}

uint32_t CtrlStepMotor::GetTemp()
{
    m_canHandle->WritePacket(canBuf, 4, CMD_API_GET_TEMPERATURE);
    return temperature;
}

void CtrlStepMotor::EraseConfigs()
{
    m_canHandle->WritePacket(canBuf, 4, CMD_API_ERASE_CONFIGS);
}

void CtrlStepMotor::SetAngle(float _angle)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionSetPoint(stepMotorCnt);
}

void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionWithVelocityLimit(stepMotorCnt, _vel);
}

void CtrlStepMotor::UpdateAngle()
{
    m_canHandle->WritePacket(canBuf, 4, CMD_API_ENABLE_MOTOR_TEMERATURE_WATCH);
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
    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_DEC_KP);
}

void CtrlStepMotor::SetDceKv(int32_t _val)
{
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_DEC_KV);
}

void CtrlStepMotor::SetDceKi(int32_t _val)
{
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_DEC_KI);
}

void CtrlStepMotor::SetDceKd(int32_t _val)
{
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not
    m_canHandle->WritePacket(canBuf, 4, CMD_API_SET_DEC_KD);
}
