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

bool TcpClient::sendMsg(const char * msg, size_t size) {
    const size_t numBytesSent = send(_sockfd, msg, size, 0);

    if (numBytesSent < 0 ) { // send failed

        std::cout << "client is already closed"<<strerror(errno) << std::endl;
        return false;
    }
    if (numBytesSent < size) { // not all bytes were sent
        char errorMsg[100];
        sprintf(errorMsg, "Only %lu bytes out of %lu was sent to client", numBytesSent, size);
        std::cout << "client is already closed"<< errorMsg << std::endl;
        return false;

    }
    return true;
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
    //parse the device Id before sending to subscriber.
    //TODO:: need to define the frame structure.
    const int32_t deviceId = msg[0];
    std::map<int32_t, client_observer_t>::iterator itmap = _subscribers.find(deviceId);
    if(itmap != _subscribers.end()) {
        itmap->second.incomingPacketHandler(msg,msgSize);
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
