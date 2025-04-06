#include "CANAPI.h"
#include <memory>
#include <mutex>
#include <map>
#include <string.h>
#include "CAN.h"
#include "TcpClient.h"

#define HAL_HANDLE_ERROR -1098
#define HAL_CAN_SEND_PERIOD_STOP_REPEATING -1
#define HAL_CAN_SEND_PERIOD_NO_REPEAT 0
#define HAL_CAN_IS_FRAME_REMOTE 0x80000000
#define HAL_CAN_TIMEOUT -1154
//https://github.com/wpilibsuite/frc-docs/blob/main/source/docs/software/can-devices/can-addressing.rst
//https://github.com/wpilibsuite/frc-docs/blob/main/source/docs/software/can-devices/third-party-devices.rst
//https://github.com/REVrobotics/node-can-bridge/blob/main/src/canWrapper.cc
//https://github.com/REVrobotics/node-can-bridge/blob/main/test/test_binding.js
//https://github.com/REVrobotics/MAXSwerve-Cpp-Template/blob/e0ae2fd0c2a55d9719087505cbec8be9fc4293bf/src/main/cpp/subsystems/MAXSwerveModule.cpp#L15
//https://github.com/REVrobotics/MAXSwerve-Cpp-Template/blob/e0ae2fd0c2a55d9719087505cbec8be9fc4293bf/src/main/include/Configs.h#L10

/*
     : m_drivingSpark(drivingCANId, SparkMax::MotorType::kBrushless),
      m_turningSpark(turningCANId, SparkMax::MotorType::kBrushless) {
  // Apply the respective configurations to the SPARKS. Reset parameters before
  // applying the configuration to bring the SPARK to a known good state.
  // Persist the settings to the SPARK to avoid losing them on a power cycle.
  m_drivingSpark.Configure(Configs::MAXSwerveModule::DrivingConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
  m_turningSpark.Configure(Configs::MAXSwerveModule::TurningConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);


class MAXSwerveModule {
 public:
  static SparkMaxConfig& DrivingConfig() {
    static SparkMaxConfig drivingConfig{};

    // Use module constants to calculate conversion factors and feed forward
    // gain.
    double drivingFactor = ModuleConstants::kWheelDiameter.value() *
                           std::numbers::pi /
                           ModuleConstants::kDrivingMotorReduction;
    double drivingVelocityFeedForward =
        1 / ModuleConstants::kDriveWheelFreeSpeedRps;

    drivingConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);
    drivingConfig.encoder
        .PositionConversionFactor(drivingFactor)          // meters
        .VelocityConversionFactor(drivingFactor / 60.0);  // meters per second
    drivingConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(0.04, 0, 0)
        .VelocityFF(drivingVelocityFeedForward)
        .OutputRange(-1, 1);

    return drivingConfig;
  }
//https://github.com/frc3512/Robot-2022/blob/01407465389466b36ccae6731e987436331c4ef7/src/main/include/HWConfig.hpp#L79
 * */
namespace {
    struct Receives {
        uint64_t lastTimeStamp;
        uint8_t data[8];
        uint8_t length;
    };

    struct CANStorage {
        HAL_CANManufacturer manufacturer;
        HAL_CANDeviceType deviceType;
        uint8_t deviceId;
        std::mutex periodicSendsMutex;
        std::mutex receivesMutex;
        std::map<int32_t, Receives> receives;
        std::map<int32_t, int32_t> periodicSends;

        //wpi::SmallDenseMap<int32_t, Receives> receives;
        //wpi::SmallDenseMap<int32_t, int32_t> periodicSends;
    };
}  // namespace

static std::map<HAL_CANHandle, std::shared_ptr<CANStorage> > *  canHandles;
static TcpClient *  g_client;
//static UnlimitedHandleResource<HAL_CANHandle, CANStorage, HAL_HandleEnum::CAN>*  canHandles;

namespace hal {
    namespace init {
        void InitializeCANAPI() {
            static std::map<HAL_CANHandle , std::shared_ptr<CANStorage>>  cH;
            canHandles = &cH;


            static TcpClient client;
            g_client=&client;

            //Connect to TCP server.
            bool connected = false;
            while (!connected) {
                connected = client.connectTo("127.0.0.1", 65123);
            }
        }
    }  // namespace init
}  // namespace hal

// observer callback. will be called for every new message received by the server
static  void onIncomingMsg(const char * msg, size_t size) {
    std::cout << "Got msg from server: " << msg << "\n";
}

// observer callback. will be called when server disconnects
static  void onDisconnection(const std::string& ret) {
    std::cout << "Server disconnected: " << ret << "\n";
}

static int32_t CreateCANId(CANStorage* storage, int32_t apiId) {
    int32_t createdId = 0;
    //Note:: Need to create Dummy / Dm CAN ID.
    switch (storage->manufacturer) {

        case HAL_CAN_Man_Dummy: //Dummy can FrameID
            createdId |= (static_cast<int32_t>(storage->deviceType) & 0x1F) << 24;
            createdId |= (static_cast<int32_t>(storage->manufacturer) & 0xFF) << 16;
            createdId |= (apiId & 0x3FF) << 6;
            createdId |= (storage->deviceId & 0x3F);
            break;
        case HAL_CAN_Man_Dm:   //DmBot can FrameID
            createdId |= (static_cast<int32_t>(storage->deviceType) & 0x1F) << 24;
            createdId |= (static_cast<int32_t>(storage->manufacturer) & 0xFF) << 16;
            createdId |= (apiId & 0x3FF) << 6;
            createdId |= (storage->deviceId & 0x3F);
            break;
    }
    return createdId;
}

extern "C" {

HAL_CANHandle HAL_InitializeCAN(HAL_CANManufacturer manufacturer,
                                int32_t deviceId, HAL_CANDeviceType deviceType,
                                int32_t* status) {
    //hal::init::CheckInit();
    auto can = std::make_shared<CANStorage>();
    int handle = canHandles->size()+1;
    canHandles->insert(std::make_pair(handle, can));

    can->deviceId = deviceId;
    can->deviceType = deviceType;
    can->manufacturer = manufacturer;

    // configure and register observer
    client_observer_t observer;
    observer.wantedIP = "127.0.0.1";
    observer.incomingPacketHandler = onIncomingMsg;
    observer.disconnectionHandler = onDisconnection;
    g_client->subscribe(deviceId, observer);

    return handle;
}

void HAL_CleanCAN(HAL_CANHandle handle) {
#if 0
    auto data = canHandles->Free(handle);
    if (data == nullptr) {
        return;
    }

    std::scoped_lock lock(data->periodicSendsMutex);

    for (auto&& i : data->periodicSends) {
        int32_t s = 0;
        auto id = CreateCANId(data.get(), i.first);
        HAL_CAN_SendMessage(id, nullptr, 0, HAL_CAN_SEND_PERIOD_STOP_REPEATING, &s);
        i.second = -1;
    }
#endif
}

void HAL_WriteCANPacket(HAL_CANHandle handle, const uint8_t* data, int32_t length, int32_t apiId, int32_t* status) {
    auto can = canHandles->find(handle)->second;

    auto id = CreateCANId(can.get(), apiId);

    std::scoped_lock lock(can->periodicSendsMutex);
    g_client->sendMsg(id, data, length, status);
    can->periodicSends[apiId] = -1;
}

void HAL_WriteCANPacketRepeating(HAL_CANHandle handle, const uint8_t* data,
                                 int32_t length, int32_t apiId,
                                 int32_t repeatMs, int32_t* status) {
    //How to send the heartbeat message, get the current/velocity/position/offset.
    auto can = canHandles->find(handle)->second;
    auto id = CreateCANId(can.get(), apiId);

    std::scoped_lock lock(can->periodicSendsMutex);
    g_client->sendMsg(id, data, length, status);
    can->periodicSends[apiId] = repeatMs;
}

void HAL_StopCANPacketRepeating(HAL_CANHandle handle, int32_t apiId, int32_t* status) {
    auto can = canHandles->find(handle)->second;

    auto id = CreateCANId(can.get(), apiId);

    std::scoped_lock lock(can->periodicSendsMutex);
    g_client->sendMsg(id, nullptr, HAL_CAN_SEND_PERIOD_STOP_REPEATING,status);
    can->periodicSends[apiId] = -1;
}

void HAL_ReadCANPacketNew(HAL_CANHandle handle, int32_t apiId, uint8_t* data,
                          int32_t* length, uint64_t* receivedTimestamp,
                          int32_t* status) {
    auto can = canHandles->find(handle)->second;

    uint32_t messageId = CreateCANId(can.get(), apiId);
    uint8_t dataSize = 0;
    uint32_t ts = 0;
    //Note:: need to work in the async mode rather than sync mode.
    //How to store the incoming data to receives?
    g_client->sendMsg(messageId, nullptr, HAL_CAN_SEND_PERIOD_STOP_REPEATING,status);

    if (*status == 0) {
        std::scoped_lock lock(can->receivesMutex);
        auto& msg = can->receives[messageId];
        msg.length = dataSize;
        msg.lastTimeStamp = ts;
        // The NetComm call placed in data, copy into the msg
        memcpy(msg.data, data, dataSize);
    }
    *length = dataSize;
    *receivedTimestamp = ts;
}
}  // extern "C"