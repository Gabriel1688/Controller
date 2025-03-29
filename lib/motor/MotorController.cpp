#include "MotorController.h"

void MotorController::control(id_t id, float kp, float kd, float q, float dq, float tau)
{
    // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
        float span = xmax - xmin;
        float data_norm = (x - xmin) / span;
        uint16_t data_uint = data_norm * ((1 << bits) - 1);
        return data_uint;
    };

    if(motors.find(id) == motors.end())
    {
        throw std::runtime_error("Motor id not found");
    }

    auto& m = motors[id];

    m->cmd = {kp, kd, q, dq, tau}; // 保存控制命令

    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
    uint16_t q_uint = float_to_uint(q, m->Q_MIN, m->Q_MAX, 16);
    uint16_t dq_uint = float_to_uint(dq, -m->DQ_MAX, m->DQ_MAX, 12);
    uint16_t tau_uint = float_to_uint(tau, -m->TAU_MAX, m->TAU_MAX, 12);

    std::array<uint8_t, 8> data_buf;
    data_buf[0] = (q_uint >> 8) & 0xff;
    data_buf[1] = q_uint & 0xff;
    data_buf[2] = dq_uint >> 4;
    data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
    data_buf[4] = kp_uint & 0xff;
    data_buf[5] = kd_uint >> 4;
    data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
    data_buf[7] = tau_uint & 0xff;

    send_data.modify(id, data_buf.data());
    //replace with ethernet frame
    // serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
    this->recv();
}

void MotorController::recv()
{
    //replace with ethernet frame
    //serial_->recv((uint8_t*)&recv_data, 0xAA, sizeof(CAN_Recv_Fream));

    if(recv_data.CMD == 0x11 && recv_data.freamEnd == 0x55) // receive success
    {
        static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
            float span = xmax - xmin;
            float data_norm = float(x) / ((1 << bits) - 1);
            float data = data_norm * span + xmin;
            return data;
        };

        auto & data = recv_data.canData;

        uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
        uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
        uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];

        if(motors.find(recv_data.CANID) == motors.end())
        {
            std::cout << "Unknown motor id: " << std::hex << recv_data.CANID << std::endl;
            return;
        }

        auto & m = motors[recv_data.CANID];
        m->state.q = uint_to_float(q_uint, m->Q_MIN, m->Q_MAX, 16);
        m->state.dq = uint_to_float(dq_uint, -m->DQ_MAX, m->DQ_MAX, 12);
        m->state.tau = uint_to_float(tau_uint, -m->TAU_MAX, m->TAU_MAX, 12);

        return;
    }
    else if (recv_data.CMD == 0x01) // receive fail
    {
        /* code */
    }
    else if (recv_data.CMD == 0x02) // send fail
    {
        /* code */
    }
    else if (recv_data.CMD == 0x03) // send success
    {
        /* code */
    }
    else if (recv_data.CMD == 0xEE) // communication error
    {
        /* code */
    }
}
/**
 * @brief 添加电机
 *
 * 实现不同的MOTOR_ID和MASTER_ID都指向同一个MotorParam
 * 确保MOTOR_ID和MASTER_ID都未使用
 */
void MotorController::addMotor(id_t MOTOR_ID, id_t MASTER_ID)
{
    motors.insert({MOTOR_ID, std::make_shared<MotorParam>()});
    motors[MASTER_ID] = motors[MOTOR_ID];
}

void MotorController::control_cmd(id_t id , uint8_t cmd)
{
    std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
    send_data.modify(id, data_buf.data());
    // replace with ethernet frame.
    //serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
    //usleep(1000);
    recv();
}