#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netdb.h>
#include <vector>
#include <errno.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <functional>
#include <map>

struct client_observer_t {
    std::string wantedIP = "";
    std::function<void(const char * msg, size_t size)> incomingPacketHandler = nullptr;
    std::function<void(const std::string & ret)> disconnectionHandler = nullptr;
};

class TcpClient
{
private:
    int _sockfd;
    std::atomic<bool> _isConnected;
    std::atomic<bool> _isClosed;
    struct sockaddr_in _server;
    std::map<int32_t, client_observer_t> _subscribers;
    std::thread * _receiveTask = nullptr;
    std::mutex _subscribersMtx;

    void publishServerMsg(const char * msg, size_t msgSize);
    void publishServerDisconnected(const std::string& ret);
    void receiveTask();

public:
    TcpClient();
    ~TcpClient();
    bool connectTo(const std::string & address, int port);
    bool sendMsg(const char * msg, size_t size);

    void subscribe(const int32_t deviceId, const client_observer_t & observer);
    bool isConnected() const { return _isConnected; }
    bool close();
};