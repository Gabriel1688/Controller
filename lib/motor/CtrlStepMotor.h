#pragma once
#include "CAN.h"
#include <memory>
class CtrlStepMotor {
public:
    enum State {
        RUNNING,
        FINISH,
        STOP
    };
    const uint32_t CTRL_CIRCLE_COUNT = 200 * 256;

    CtrlStepMotor(uint8_t _id, bool _inverse = false, uint8_t _reduction = 1,
                  float _angleLimitMin = -180, float _angleLimitMax = 180);

    uint8_t nodeID;
    uint8_t deviceId;
    float angle = 0;
    float angleLimitMax;
    float angleLimitMin;
    uint32_t temperature = 0.0;
    bool inverseDirection;
    uint8_t reduction;
    State state = STOP;

    void SetAngle(float _angle);
    void SetAngleWithVelocityLimit(float _angle, float _vel);

    // CAN Command
    void SetEnable(bool _enable);
    void SetEnableTemp(bool _enable);
    void DoCalibration();
    void SetCurrentSetPoint(float _val);
    void SetVelocitySetPoint(float _val);
    void SetPositionSetPoint(float _val);
    void SetPositionWithVelocityLimit(float _pos, float _vel);
    void SetNodeID(uint32_t _id);
    void SetCurrentLimit(float _val);
    void SetVelocityLimit(float _val);
    void SetAcceleration(float _val);
    void SetDceKp(int32_t _val);
    void SetDceKv(int32_t _val);
    void SetDceKi(int32_t _val);
    void SetDceKd(int32_t _val);
    void ApplyPositionAsHome();
    void SetEnableOnBoot(bool _enable);
    void SetEnableStallProtect(bool _enable);
    void Reboot();
    uint32_t GetTemp();
    void EraseConfigs();

    void UpdateAngle();
    void UpdateAngleCallback(float _pos, bool _isFinished);

private:
    typedef struct
    {
        uint32_t StdId; /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

        uint32_t ExtId; /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

        uint32_t IDE; /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_identifier_type */

        uint32_t RTR; /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request */

        uint32_t DLC; /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

    } CAN_TxHeaderTypeDef;
    std::shared_ptr<CAN> m_canHandle;
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = {};
};