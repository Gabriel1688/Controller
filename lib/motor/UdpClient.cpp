#include "UdpClient.h"
#include "spdlog/spdlog.h"
#include <fcntl.h>
#include <iomanip>
#include <sys/epoll.h>
#define MAX_PACKET_SIZE 4096

UdpClient::UdpClient() {
    _isConnected = false;
    _isClosed = true;
}

UdpClient::~UdpClient() {
    close();
}

void UdpClient::Start() {
    if (pthread_create(&thread_id, nullptr, EntryOfThread, this) != 0) {
    }
}

/*static*/
void *UdpClient::EntryOfThread(void *argv) {
    UdpClient *client = static_cast<UdpClient *>(argv);
    client->run();
    return (void *) client;
}

bool UdpClient::connectTo(const std::string &address, int port) {
    try {
        _sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        const int inetSuccess = inet_aton(address.c_str(), &_server.sin_addr);

        if (!inetSuccess) {// inet_addr failed to parse address
            // if hostname is not in IP strings and dots format, try resolve it
            struct hostent *host;
            struct in_addr **addrList;
            if ((host = gethostbyname(address.c_str())) == nullptr) {
                throw std::runtime_error("Failed to resolve hostname");
            }
            addrList = (struct in_addr **) host->h_addr_list;
            _server.sin_addr = *addrList[0];
        }
        _server.sin_family = AF_INET;
        _server.sin_port = htons(port);
    } catch (const std::runtime_error &error) {
        std::cout << "client is already closed, " << error.what() << std::endl;
        return false;
    }
    _isConnected = true;
    _isClosed = false;

    return true;
}

void UdpClient::sendMsg(CANFrameId frameId, const uint8_t *data, uint8_t dataSize, __attribute__((unused)) int32_t *status) {
    CANFrame frame;
    frame.modify(frameId.forwardCANId, data, dataSize);
    spdlog::info("<------ {0:04x} : {1:}", frame.FrameId, concatenation(frame.data, 13, " "));
    const size_t numBytesSent = sendto(_sockfd, (uint8_t *) &frame, 5 + dataSize, 0, (struct sockaddr *) &_server, sizeof(_server));
    if (numBytesSent < dataSize) {// not all bytes were sent
        if (numBytesSent <= 0) {  // send failed
            std::cout << "client is already closed" << strerror(errno) << std::endl;
        } else {
            char errorMsg[100];
            sprintf(errorMsg, "Only %lu bytes out of %d was sent to client", numBytesSent, dataSize);
            std::cout << "client is already closed" << errorMsg << std::endl;
        }
    } else {//Add reply frameId/handle to map for receiving reply.
        std::scoped_lock lock(frameIdsMutex);
        if (_frameIds.find(frameId.replyCANId) == _frameIds.end()) {
            _frameIds.insert(std::make_pair(frameId.replyCANId, frameId.hanlde));
        }
    }
}

void UdpClient::subscribe(const int32_t deviceId, const client_observer_t &observer) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    _subscribers.insert(std::make_pair(deviceId, observer));
}

/*
 * Publish incomingPacketHandler client message to observer.
 * Observers get only messages that originated
 * from clients with IP address identical to
 * the specific observer requested IP
 */
void UdpClient::publishServerMsg(const uint8_t *msg, size_t msgSize) {
    //std::lock_guard<std::mutex> lock(_subscribersMtx);
    CANFrame *frame = (CANFrame *) msg;
    //Get handle of message and wake up it.
    {
        auto FrameId = __builtin_bswap32(frame->FrameId);
        std::scoped_lock lock(frameIdsMutex);
        auto itmap = _frameIds.find(FrameId);
        if (itmap != _frameIds.end()) {
            auto can = canHandles->find(itmap->second)->second;
            auto subscriber = _subscribers.find(can->deviceId);
            if (subscriber != _subscribers.end()) {
                subscriber->second.incomingPacketHandler(msg, msgSize);
            }
            can->replyEvent.Set();
        }
    }
}

/*
 * Publish client disconnection to observer.
 * Observers get only notify about clients
 * with IP address identical to the specific
 * observer requested IP
 */
void UdpClient::publishServerDisconnected(const std::string &ret) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    for (const auto &subscriber : _subscribers) {
        if (subscriber.second.disconnectionHandler) {
            subscriber.second.disconnectionHandler(ret);
        }
    }
}

/*
 * Receive server packets, and notify user
 */
void UdpClient::run() {
    /* Disable socket blocking */
    fcntl(_sockfd, F_SETFL, O_NONBLOCK);

    /* Initialize variables for epoll */
    struct epoll_event ev;

    int epfd = epoll_create(2);
    ev.data.fd = _sockfd;
    ev.events = EPOLLIN;
    epoll_ctl(epfd, EPOLL_CTL_ADD, _sockfd, &ev);

    struct epoll_event events[2];
    std::cout << "UdpClient::receiveTask is running. " << std::endl;
    while (_isConnected) {
        int ready = epoll_wait(epfd, events, 2, -1);//20 milliseconds
        if (ready < 0) {
            perror("epoll_wait error.");
            return;
        } else if (ready == 0) {
            /* timeout, no data coming */
            continue;
        } else {
            for (int i = 0; i < ready; i++) {
                if (events[i].data.fd == _sockfd) {
                    uint8_t msg[13];
                    memset(msg, 0, 13);
                    const size_t numOfBytesReceived = recvfrom(_sockfd, msg, MAX_PACKET_SIZE, 0, NULL, NULL);
                    if (numOfBytesReceived < 1) {
                        std::string errorMsg;
                        if (numOfBytesReceived == 0) {
                            errorMsg = "Server closed connection";
                        } else {
                            errorMsg = strerror(errno);
                        }
                        _isConnected = false;
                        publishServerDisconnected(errorMsg);
                        return;
                    } else {
                        publishServerMsg(msg, numOfBytesReceived);
                    }
                }
            }
        }
    }
}

bool UdpClient::close() {
    if (_isClosed) {
        std::cout << "client is already closed" << std::endl;
        return false;
    }
    _isConnected = false;
    void *result;
    if (pthread_join(thread_id, &result) != 0) {
        perror("Failed to join thread 1");
        return false;
    }

    const bool closeFailed = (::close(_sockfd) == -1);
    if (closeFailed) {

        std::cout << "failed to close socket, error " << strerror(errno) << std::endl;
        return false;
    }
    _isClosed = true;
    return true;
}

std::string UdpClient::concatenation(const uint8_t *elements, size_t size, const std::string delimiter) {
    // Manual concatenation
    std::ostringstream oss;
    for (size_t i = 0; i < size; ++i) {
        oss << "0x"
            << std::setfill('0') << std::setw(sizeof(uint8_t) * 2)
            << std::hex << elements[i];
        if (i < size - 1) {
            oss << delimiter;
        }
    }
    return oss.str();
}