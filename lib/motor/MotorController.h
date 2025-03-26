#pragma  once

#include <vector>
#include <unordered_map>
#include <memory>
#include <iostream>

using uint8_t=unsigned char;
using uint32_t=unsigned int;
using uint16_t=unsigned short int;
#pragma pack(1)

typedef struct
{
    uint8_t freamHeader;
    uint8_t CMD;
    // 命令 0x00: 心跳
    //     0x01: receive fail 0x11: receive success
    //     0x02: send fail 0x12: send success
    //     0x03: set baudrate fail 0x13: set baudrate success
    //     0xEE: communication error 此时格式段为错误码
    //           8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载
    uint8_t canDataLen: 6; // 数据长度
    uint8_t canIde: 1;     // 0: 标准帧 1: 扩展帧
    uint8_t canRtr: 1;     // 0: 数据帧 1: 远程帧
    uint32_t CANID;        // 电机反馈的ID
    uint8_t canData[8];
    uint8_t freamEnd;      // 帧尾
} CAN_Recv_Fream;

typedef struct
{
    uint8_t freamHeader[2] = {0x55, 0xAA}; // 帧头
    uint8_t freamLen = 0x1e;               // 帧长
    uint8_t CMD = 0x01;                    // 命令 1：转发CAN数据帧 2：PC与设备握手，设备反馈OK 3: 非反馈CAN转发，不反馈发送状态
    uint32_t sendTimes = 1;                // 发送次数
    uint32_t timeInterval = 10;            // 时间间隔
    uint8_t IDType = 0;                    // ID类型 0：标准帧 1：扩展帧
    uint32_t CANID;                        // CAN ID 使用电机ID作为CAN ID
    uint8_t frameType = 0;                 // 帧类型 0： 数据帧 1：远程帧
    uint8_t len = 0x08;                    // len
    uint8_t idAcc;
    uint8_t dataAcc;
    uint8_t data[8];
    uint8_t crc; // 未解析，任意值

    void modify(const id_t id, const uint8_t* send_data)
    {
        CANID = id;
        std::copy(send_data, send_data+8, data);
    }
} CAN_Send_Fream;

#pragma pack()


typedef struct
{
    float Q_MIN = -12.5;
    float Q_MAX = 12.5;
    float DQ_MAX = 30;
    float TAU_MAX = 10;

    struct {
        float kp;
        float kd;
        float q;
        float dq;
        float tau;
    } cmd;

    struct {
        float q;
        float dq;
        float tau;
    } state;

} MotorParam;

/**
 * @brief 达妙科技 DM-J4310-2EC 电机控制
 *
 * 使用USB转CAN进行通信，linux做虚拟串口
 */
class MotorController
{
public:
//    Motor(SerialPort::SharedPtr serial = nullptr)
//            : serial_(serial)
    MotorController()
    {
    }

    void enable(id_t id) { control_cmd(id, 0xFC); }
    void reset(id_t id) { control_cmd(id, 0xFD); }
    void zero_position(id_t id) { control_cmd(id, 0xFE); }

    void control(id_t id, float kp, float kd, float q, float dq, float tau);

    void recv();

    std::unordered_map<id_t, std::shared_ptr<MotorParam>> motors;

    /**
     * @brief 添加电机
     *
     * 实现不同的MOTOR_ID和MASTER_ID都指向同一个MotorParam
     * 确保MOTOR_ID和MASTER_ID都未使用
     */
    void addMotor(id_t MOTOR_ID, id_t MASTER_ID);

private:
    void control_cmd(id_t id , uint8_t cmd);
    CAN_Send_Fream send_data;
    CAN_Recv_Fream recv_data;
};
