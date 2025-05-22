#include "TcpClient.h"
#define MAX_PACKET_SIZE 4096

TcpClient::TcpClient() {
    _isConnected = false;
    _isClosed = true;
}

TcpClient::~TcpClient() {
    close();
}

bool TcpClient::connectTo(const std::string & address, int port) {
    try {
        _sockfd = socket(AF_INET , SOCK_STREAM , 0);

        const int inetSuccess = inet_aton(address.c_str(), &_server.sin_addr);

        if(!inetSuccess) { // inet_addr failed to parse address
            // if hostname is not in IP strings and dots format, try resolve it
            struct hostent *host;
            struct in_addr **addrList;
            if ( (host = gethostbyname( address.c_str() ) ) == nullptr){
                throw std::runtime_error("Failed to resolve hostname");
            }
            addrList = (struct in_addr **) host->h_addr_list;
            _server.sin_addr = *addrList[0];
        }
        _server.sin_family = AF_INET;
        _server.sin_port = htons(port);
    } catch (const std::runtime_error& error) {
        std::cout << "client is already closed"<< error.what() << std::endl;
        return false;
    }

    const int connectResult = connect(_sockfd , (struct sockaddr *)&_server , sizeof(_server));
    const bool connectionFailed = (connectResult == -1);
    if (connectionFailed) {
        std::cout << "client is already closed"<< strerror(errno) << std::endl;
        return false;
    }

    _receiveTask = new std::thread(&TcpClient::receiveTask, this);
    _isConnected = true;
    _isClosed = false;

    return true;
}
#if 0
void TcpClient::sendMsg(uint32_t messageID, const uint8_t* data, uint8_t dataSize, int32_t* status) {

    //const char * msg, size_t size
    //Need to build message based on the messageId/data.
    const char * msg= nullptr;
    const size_t numBytesSent = send(_sockfd, msg, dataSize, 0);

    if (numBytesSent < 0 ) { // send failed
        std::cout << "client is already closed"<<strerror(errno) << std::endl;
    }
    if (numBytesSent < dataSize) { // not all bytes were sent
        char errorMsg[100];
        sprintf(errorMsg, "Only %lu bytes out of %lu was sent to client", numBytesSent, dataSize);
        std::cout << "client is already closed"<< errorMsg << std::endl;
    }
}
#endif
void TcpClient::sendMsg(CANFrameId frameId, const uint8_t* data, uint8_t dataSize, __attribute__((unused)) int32_t* status) {

    CANFrame frame;
    frame.modify(frameId.forwardCANId, data, dataSize);
    const size_t numBytesSent = send(_sockfd, (uint8_t*)&frame, dataSize, 0);

    //Add reply frameId/handle to map for receiving reply.
    {
        std::scoped_lock lock(frameIdsMutex);
        if (_frameIds.find(frameId.replyCANId) == _frameIds.end())
        {
            _frameIds.insert(std::make_pair(frameId.replyCANId, frameId.hanlde));
        }
    }
    if (numBytesSent <= 0 ) { // send failed
        std::cout << "client is already closed"<<strerror(errno) << std::endl;
    }
    if (numBytesSent < dataSize) { // not all bytes were sent
        char errorMsg[100];
        sprintf(errorMsg, "Only %lu bytes out of %d was sent to client", numBytesSent, dataSize);
        std::cout << "client is already closed"<< errorMsg << std::endl;
    }
}


void TcpClient::subscribe(const int32_t deviceId, const client_observer_t & observer) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    _subscribers.insert(std::make_pair(deviceId,observer));
}

/*
 * Publish incomingPacketHandler client message to observer.
 * Observers get only messages that originated
 * from clients with IP address identical to
 * the specific observer requested IP
 */
void TcpClient::publishServerMsg(const char * msg, size_t msgSize) {
    std::lock_guard<std::mutex> lock(_subscribersMtx);
    const int32_t deviceId = msg[0];
    std::map<int32_t, client_observer_t>::iterator itmap = _subscribers.find(deviceId);
    if(itmap != _subscribers.end()) {
        itmap->second.incomingPacketHandler(msg,msgSize);
    }
    CANFrame* frame = (CANFrame*)msg;
    //Get handle of message and wake up it.
    {
        std::scoped_lock lock(frameIdsMutex);
        auto itmap = _frameIds.find(frame->FrameId);
        if ( itmap != _frameIds.end())
        {
            auto can = canHandles->find(itmap->second)->second;
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
void TcpClient::publishServerDisconnected(const std::string & ret) {
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
void TcpClient::receiveTask() {
    while(_isConnected) {

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        fd_set fds;

        FD_ZERO(&fds);
        FD_SET(_sockfd,&fds);

        const int selectRet = select(_sockfd + 1, &fds, nullptr, nullptr, &tv);

        if (selectRet == 0 ) {
            throw std::runtime_error(strerror(errno));
        } else if (selectRet == 1) {
            continue;
        }

        char msg[MAX_PACKET_SIZE];
        const size_t numOfBytesReceived = recv(_sockfd, msg, MAX_PACKET_SIZE, 0);

        if(numOfBytesReceived < 1) {
            std::string errorMsg;
            if (numOfBytesReceived == 0) { //server closed connection
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

bool TcpClient::close(){
    if (_isClosed) {
        std::cout << "client is already closed" << std::endl;
        return false;
    }
    _isConnected = false;

    if (_receiveTask) {
        _receiveTask->join();
        delete _receiveTask;
        _receiveTask = nullptr;
    }

    const bool closeFailed = (::close(_sockfd) == -1);
    if (closeFailed) {

        std::cout << "client is already closed" << strerror(errno) << std::endl;
        return false;
    }
    _isClosed = true;
    return true;
}
