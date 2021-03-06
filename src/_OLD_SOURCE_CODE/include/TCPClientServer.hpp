#ifndef TCPCLIENTSERVER_HPP
#define TCPCLIENTSERVER_HPP

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <thread>
#include <queue>
#include <bitset>
#include <mutex>

#include "Joystick.hpp"
#include "opencv2/opencv.hpp"

class Joystick;

#define SERVER_PORT 7777
#define CLIENT_PORT 7778
#define DRIVERSTATION_ADDRESS "10.0.0.1"

#define MSB 0x80000000
#define HEADER 0xFF000000
#define JOYSTICK_INDEX 0x1F000000
#define METADATA 0x00FF0000
#define DATA 0x0000FFFF
const uint32_t MAT_HEADER = MSB | 0xC;
#define BLANK_STRING_HEADER (MSB | 0xA)

class TCP_Client {
public:
    TCP_Client();
    ~TCP_Client() = default;

    void start();
    void stop();

    void sendMessage(const std::string& message);
    void sendMessage(cv::Mat* image);
private:
    std::queue<cv::Mat*> frameQueue;
    const size_t FRAME_QUEUE_LIMIT = 15; // constrain queue to only be 13 MB
    std::mutex frameLock;
    std::queue<std::string> messageQueue;
    const size_t MESSAGE_QUEUE_LIMIT = 100; // constrain queue to only hold 100 reasonably sized strings (technically no upper bound)
    std::mutex messageLock;
    int sock;
    char buffer[32];
    struct sockaddr_in address;
    bool shouldThreadBeRunning = true;
    std::thread client;

    void handleConnection(std::reference_wrapper<bool> running);
};

class TCP_Server {
public:
    TCP_Server();
    ~TCP_Server() = default;

    void registerJoystick(Joystick* joystick);

    void start();
    void stop();
private:
    int listeningSocket, clientSocket;
    char host[NI_MAXHOST], service[NI_MAXSERV];
    sockaddr_in address, client;
    socklen_t clientSize = sizeof(client);
    char buffer[32] = {0};
    bool shouldThreadBeRunning = true;
    std::thread server;

    Joystick* joystick;

    void handleConnection(std::reference_wrapper<bool> running);
    void decodeMessage(char* buffer);
};

#endif // TCPCLIENTSERVER_HPP
