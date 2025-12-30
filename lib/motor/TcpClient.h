#pragma once

#include "CANAPI.h"
#include <arpa/inet.h>
#include <atomic>
#include <errno.h>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <memory>

#pragma pack(1)
struct CANFrame {
    uint8_t FrameHeader = 0x08;// 0：标准帧 0： 数据帧, DLC = xx;
    uint32_t FrameId = 0x01;   // CAN ID 使用电机ID作为CAN ID
    uint8_t data[8] = {0};
    void modify(const uint32_t id, const uint8_t *send_data, uint8_t dataSize) {
        FrameId = __builtin_bswap32(id);//change to big endian format.
        std::copy(send_data, send_data + dataSize, data);
    }
};
#pragma pack()

struct client_observer_t {
    std::string wantedIP = "";
    std::function<void(const uint8_t *msg, size_t size)> incomingPacketHandler = nullptr;
    std::function<void(const std::string &ret)> disconnectionHandler = nullptr;
};

class TcpClient {
private:
    int _sockfd;
    std::atomic<bool> _isConnected;
    std::atomic<bool> _isClosed;
    struct sockaddr_in _server;
    std::map<int32_t, client_observer_t> _subscribers;
    std::thread *_receiveTask = nullptr;
    std::mutex _subscribersMtx;
    pthread_t thread_id;

    std::mutex frameIdsMutex;
    std::map<int32_t, HAL_CANHandle> _frameIds;// keep the reply frameId and handle of device.
    std::map<HAL_CANHandle, std::shared_ptr<CANStorage>> *canHandles;

    void publishServerMsg(const uint8_t *msg, size_t msgSize);
    void publishServerDisconnected(const std::string &ret);
    void run();
    static void *EntryOfThread(void *argv);

public:
    TcpClient();
    ~TcpClient();
    void Start();
    bool connectTo(const std::string &address, int port);

    /**
     * Sends a CAN message.
    *
    * @param[in] messageID the CAN ID to send
    * @param[in] data      the data to send (0-8 bytes)
    * @param[in] dataSize  the size of the data to send (0-8 bytes)
    * @param[out] status    Error status variable. 0 on success.
    */
    void sendMsg(CANFrameId frameId, const uint8_t *data, uint8_t dataSize, int32_t *status);

    void subscribe(const int32_t deviceId, const client_observer_t &observer);
    bool isConnected() const { return _isConnected; }
    void setCanHandles(std::map<HAL_CANHandle, std::shared_ptr<CANStorage>> *p_canHandles) {
        canHandles = p_canHandles;
    }
    bool close();
};