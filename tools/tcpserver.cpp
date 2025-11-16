#include <iostream>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
using namespace std;

int main(int argc, char *argv[]) {
    //grab the port number
    int port = 1180;
    //buffer to send and receive messages with
    unsigned char msg[1500];

    //setup a socket and connection tools
    sockaddr_in servAddr;
    bzero((char *) &servAddr, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servAddr.sin_port = htons(port);

    //open stream oriented socket with internet address
    //also keep track of the socket descriptor
    int serverSd = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSd < 0) {
        cerr << "Error establishing the server socket" << endl;
        exit(0);
    }
    //bind the socket to its local address
    int bindStatus = bind(serverSd, (struct sockaddr *) &servAddr,
                          sizeof(servAddr));
    if (bindStatus < 0) {
        cerr << "Error binding socket to local address" << endl;
        exit(0);
    }
    cout << "Waiting for a client to connect..." << endl;
    //listen for up to 5 requests at a time
    listen(serverSd, 5);
    //receive a request from client using accept
    //we need a new address to connect with the client
    sockaddr_in newSockAddr;
    socklen_t newSockAddrSize = sizeof(newSockAddr);
    //accept, create a new socket descriptor to
    //handle the new connection with client
    int newSd = accept(serverSd, (sockaddr *) &newSockAddr, &newSockAddrSize);
    if (newSd < 0) {
        cerr << "Error accepting request from client!" << endl;
        exit(1);
    }
    cout << "Connected with client!" << endl;
    //lets keep track of the session time
    struct timeval start1, end1;
    gettimeofday(&start1, NULL);
    //also keep track of the amount of data sent as well
    int bytesRead, bytesWritten = 0;

    //receive a message from the client (listen)
    cout << "Awaiting client request..." << endl;
    uint8_t response[] = {0x08, 0x00, 0x00, 0x00, 0x11, 0x11, 0x70, 0x81, 0x7F, 0xF8, 0x01, 0x1C, 0x1B};
    while (1) {
        memset(&msg, 0, sizeof(msg));//clear the buffer
        bytesRead = recv(newSd, (unsigned char *) &msg, sizeof(msg), 0);
        std::cout << "Received client data : " << endl;
        for (int i = 0; i < bytesRead; i++) {
            std::cout << "data [" << i << "] = " << std::showbase << std::hex << (unsigned int) msg[i] << std::endl;
        }
        usleep(10000);

        send(newSd, (char *) &response, 13, 0);
        std::cout << "Send response back to client : " << endl;

        for (int i = 0; i < sizeof(response) / sizeof(uint8_t); i++) {
            std::cout << "data [" << i << "] = " << std::showbase << std::hex << (unsigned int) response[i] << std::endl;
        }
        usleep(10000);
    }

    //we need to close the socket descriptors after all is done
    gettimeofday(&end1, NULL);
    close(newSd);
    close(serverSd);
    cout << "********Session********" << endl;
    cout << "Bytes written: " << bytesWritten << " Bytes read: " << bytesRead << endl;
    cout << "Elapsed time: " << (end1.tv_sec - start1.tv_sec)
         << " secs" << endl;
    cout << "Connection closed..." << endl;
    return 0;
}
