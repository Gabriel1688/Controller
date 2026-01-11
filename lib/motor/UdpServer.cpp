#include "UdpServer.h"
#include "spdlog/spdlog.h"
#include <fcntl.h>
#include <iomanip>
#include <sys/epoll.h>
#define MAX_PACKET_SIZE 4096
static const std::string UDP_CLIENT_LEFT = "192.168.4.101";
static const std::string UDP_CLIENT_RIGHT = "192.168.4.103";
static const int UDP_REMOTE_PORT = 12345;

UdpServer::UdpServer() {
    _isClosed = false;
}

UdpServer::~UdpServer() {
    close();
}
bool UdpServer::init(const std::string address, uint16_t port) {
    // bind socket and listen to UDP Server port.
    _sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int status = inet_aton(address.c_str(), &_server.sin_addr);
    if (!status) {
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
    status = bind(_sockfd, (struct sockaddr *) &_server, sizeof(_server));
    if (status == -1) {
        spdlog::error("Error binding socket to local address, errno:{}.", strerror(errno));
        _isClosed = true;
        return false;
    }
    /* Disable socket blocking */
    fcntl(_sockfd, F_SETFL, O_NONBLOCK);

    /* Initialize the UDP client for left right leg*/

    inet_aton(UDP_CLIENT_LEFT.c_str(), &_clientLeft.sin_addr);
    _clientLeft.sin_family = AF_INET;
    _clientLeft.sin_port = htons(UDP_REMOTE_PORT);

    inet_aton(UDP_CLIENT_LEFT.c_str(), &_clientRight.sin_addr);
    _clientRight.sin_family = AF_INET;
    _clientRight.sin_port = htons(UDP_REMOTE_PORT);

    _isClosed = false;
    return true;
}
void UdpServer::Start() {
    if (init("192.168.4.102", 8887) == false) {
        spdlog::error("Failed to initialize sockets");
    } else {
        if (pthread_create(&thread_id, nullptr, EntryOfThread, this) != 0) {
            spdlog::error("Failed to initialize thread");
        }
    }
}

/*static*/
void *UdpServer::EntryOfThread(void *argv) {
    UdpServer *server = static_cast<UdpServer *>(argv);
    server->run();
    return (void *) server;
}

void UdpServer::sendMsg(CANFrameId frameId, const uint8_t *data, uint8_t dataSize, bool reply, __attribute__((unused)) int32_t *status) {
    sockaddr_in *client = getClientAddrByDeviceId(frameId.deviceId);
    CANFrame frame;
    frame.modify(frameId.forwardCANId, data, dataSize);
    spdlog::info("<------ {0:04x} : {1:}", frame.FrameId, concatenation((uint8_t *) (&frame), 13, " "));
    const size_t numBytesSent = sendto(_sockfd, (uint8_t *) &frame, 13, 0, (struct sockaddr *) client, sizeof(_server));
    if (numBytesSent < dataSize) {
        if (numBytesSent <= 0) {
            spdlog::error("Failed to send data to client, error : {} ", strerror(errno));
        } else {
            spdlog::error("Only {} bytes out of {} was sent to client", numBytesSent, dataSize);
        }
    } else {
        if (reply) {
            //Add reply frameId/handle to map for receiving reply.
            std::scoped_lock lock(frameIdsMutex);
            if (_frameIds.find(frameId.replyCANId) == _frameIds.end()) {
                _frameIds.insert(std::make_pair(frameId.replyCANId, frameId.hanlde));
                spdlog::info("Wait for reply from remote CAN device [{0:d}], replay FrameId : [{1:04x}].", frameId.deviceId, frameId.replyCANId);
            }
        }
    }
}

void UdpServer::subscribe(const int32_t deviceId, const client_observer_t &observer) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    _subscribers.insert(std::make_pair(deviceId, observer));
}
sockaddr_in *UdpServer::getClientAddrByDeviceId(int deviceId) {
    auto iter = _deviceIPs.find(deviceId);
    if (iter == _deviceIPs.end()) {
        spdlog::error("Device with device ID {:d} does not exist", deviceId);
        return nullptr;
    }
    return iter->second;
}
void UdpServer::bindDevicesToClient(int deviceId) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    if (deviceId < 5) {
        _deviceIPs.insert(std::make_pair(deviceId, &_clientLeft));
    } else {
        _deviceIPs.insert(std::make_pair(deviceId, &_clientRight));
    }
}

/*
 * Publish incomingPacketHandler client message to observer.
 * Observers get only messages that originated
 * from clients with IP address identical to
 * the specific observer requested IP
 */
void UdpServer::dispatchCanMessage(std::string clientAddr, int port, const uint8_t *msg, size_t msgSize) {
    //split into standard CAN Frame.
    int chunk_size = 13;//standard CAN frame.
    for (int i = 0; i * chunk_size < msgSize; ++i) {
        // Calculate the start pointer for the current chunk
        const uint8_t *current_ptr = msg + (i * chunk_size);
        // Determine the length of the current chunk (last chunk might be smaller)
        int remaining_elements = msgSize - (i * chunk_size);
        int length = (remaining_elements < chunk_size) ? remaining_elements : chunk_size;

        //processCanFrame(current_ptr, length, i);
        CANFrame *frame = (CANFrame *) current_ptr;
        auto FrameId = __builtin_bswap32(frame->FrameId);
        std::scoped_lock lock(frameIdsMutex);
        auto itmap = _frameIds.find(FrameId);
        if (itmap != _frameIds.end()) {
            auto storage = canHandles->find(itmap->second)->second;
            auto subscriber = _subscribers.find(storage->deviceId);
            if (subscriber != _subscribers.end()) {
                spdlog::info("Dispatch frameId [{:d}] to device:[{:d}].", FrameId, storage->deviceId);
                subscriber->second.incomingPacketHandler(msg, msgSize);
            }
            storage->replyEvent.Set();
        } else {
            spdlog::info("Silent drop {}:{}-->{}", clientAddr, port, concatenation(current_ptr, length, " "));
        }
    }
}
/*
 * Receive UDP client packets, and notify subscriber
 */
void UdpServer::run() {
    char address[INET_ADDRSTRLEN];
    sockaddr_in clientAddr;
    socklen_t clientAddrSize = sizeof(clientAddr);

    /* Initialize variables for epoll */
    struct epoll_event ev;

    int epfd = epoll_create(2);
    ev.data.fd = _sockfd;
    ev.events = EPOLLIN;
    epoll_ctl(epfd, EPOLL_CTL_ADD, _sockfd, &ev);

    struct epoll_event events[2];
    std::cout << "UdpServer ::receiveTask is running. " << std::endl;
    while (!_isClosed) {
        int ready = epoll_wait(epfd, events, 2, -1);
        if (ready < 0) {
            perror("epoll_wait error.");
            return;
        } else {
            inet_ntop(AF_INET, &(clientAddr.sin_addr), address, INET_ADDRSTRLEN);
            for (int i = 0; i < ready; i++) {
                if (events[i].data.fd == _sockfd) {
                    uint8_t msg[130];//maximun 10 CAN message in a UDP package
                    memset(msg, 0, 130);
                    const size_t length = recvfrom(_sockfd, msg, sizeof(msg) - 1, 0,
                                                   (struct sockaddr *) &clientAddr, &clientAddrSize);
                    if (length < 1) {
                        if (length == 0) {
                            spdlog::info("receive empty package");
                        } else {
                            spdlog::error("Failed to receive data {}", strerror(errno));
                        }
                    } else {
                        // Extract client IP address and port
                        char client_ip[INET_ADDRSTRLEN];
                        inet_ntop(AF_INET, &(clientAddr.sin_addr), client_ip, INET_ADDRSTRLEN);
                        int port = ntohs(clientAddr.sin_port);
                        dispatchCanMessage(std::string(client_ip), port, msg, length);
                    }
                }
            }
        }
    }
}

bool UdpServer::close() {
    if (_isClosed) {
        std::cout << "server is already closed" << std::endl;
        return false;
    }
    _isClosed = true;
    void *result;
    if (pthread_join(thread_id, &result) != 0) {
        perror("Failed to join thread.");
        return false;
    }

    const bool closeFailed = (::close(_sockfd) == -1);
    if (closeFailed) {
        std::cout << "failed to close socket, error " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

std::string UdpServer::concatenation(const uint8_t *elements, size_t size, const std::string delimiter) {
    std::ostringstream oss;
    for (size_t i = 0; i < size; ++i) {
        oss << "0x" << std::setfill('0') << std::setw(sizeof(uint8_t) * 2)
            << std::hex << static_cast<unsigned int>(elements[i]);
        if (i < size - 1) {
            oss << delimiter;
        }
    }
    return oss.str();
}