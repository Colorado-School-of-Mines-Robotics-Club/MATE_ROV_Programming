#include "TCPClientServer.hpp"

TCP_Server::TCP_Server() {
    // create a socket
    listeningSocket = socket(AF_INET, SOCK_STREAM, 0);
    if(listeningSocket == -1) {
        std::cout << "Unable to aquire socket. Quitting and dumping any locks.";
        std::abort();
    }

    // fill sockaddr_in
    address.sin_family = AF_INET;
    address.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, "0.0.0.0", &address.sin_addr);

    // bind socket to port
    bind(listeningSocket, (sockaddr*)&address, sizeof(address));
}

void TCP_Server::registerJoystick(Joystick* joystick) {
    this->joystick = joystick;
}

void TCP_Server::start() {
    // start handling thread
    std::cout << "starting server thread" << std::endl;
    server = std::thread(&TCP_Server::handleConnection, this, std::ref(shouldThreadBeRunning));
    server.detach();
    std::cout << "server thread detached" << std::endl;
}

void TCP_Server::handleConnection(std::reference_wrapper<bool> running) {
    // tell winsock the socket is ready for listening
    listen(listeningSocket, SOMAXCONN);

    // wait for connection
    std::cout << "Listening for connections on 0.0.0.0:" << SERVER_PORT << std::endl;
    clientSocket = accept(listeningSocket, (sockaddr*)&client, &clientSize);

    memset(host, 0, NI_MAXHOST);
    memset(service, 0, NI_MAXSERV);

    // report that a connection has been established
    if(getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0) {
        std::cout << host << " connected on port " << service << std::endl;
    }
    else {
        inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
        std::cout << host << " connected on port " << ntohs(client.sin_port) << std::endl;
    }

    // close the listening socket
    close(listeningSocket);

    // wait for messages
    while(running.get()) {
        memset(buffer, 0, 32);
        
        // wait for client to send
        int bytesReceived = recv(clientSocket, buffer, 32, 0);
        switch(bytesReceived) {
            case -1:
                std::cerr << "Error in recv(). Quitting" << std::endl;
                return;
            case 0:
                std::cout << "Client disconnected" << std::endl;
                return;
            default:
                decodeMessage(buffer);
                break;
        }

        // TESTING ONLY
        // echo back to client
        send(clientSocket, buffer, bytesReceived + 1, 0);
    }

    close(clientSocket);
}

void TCP_Server::decodeMessage(char* buffer) {
    uint32_t message = *reinterpret_cast<uint32_t*>(buffer);

    if(!message & MSB) {
        std::cout << "undefined message type received" << std::endl;
    } else {
        // message received is a joystick event
        JoystickEvent event;
        event.number = (message & JOYSTICK_INDEX) >> 24;
        event.time = 0;
        event.type = (message & METADATA) >> 16;
        event.value =  message & DATA;

        if(event.isButton()) {
            if(event.value)
                joystick->button_pressed(event.number, 0);
            else
                joystick->button_released(event.number, 0);
        } else if(event.isAxis()) {
            joystick->axis_updated(event);
        }
    }
}

void TCP_Server::stop() {
    shouldThreadBeRunning = false;
    server.join();
}

TCP_Client::TCP_Client() {
    // create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1) {
        // THROW ERROR
    }

    // create address structure for server we are connecting to
    address.sin_family = AF_INET;
    address.sin_port = htons(CLIENT_PORT);
    inet_pton(AF_INET, DRIVERSTATION_ADDRESS, &address.sin_addr);
}

void TCP_Client::start() {

    std::cout << "starting client thread" << std::endl;
    client = std::thread(&TCP_Client::handleConnection, this, std::ref(shouldThreadBeRunning));
    client.detach();
    std::cout << "client thread detached" << std::endl;
}

void TCP_Client::handleConnection(std::reference_wrapper<bool> running) {
    // connect to server
    std::cout << "Connecting to " << DRIVERSTATION_ADDRESS << ":" << CLIENT_PORT << std::endl;
    int connectionResult = -1;
    do {
        connectionResult = connect(sock, (sockaddr*)&address, sizeof(address));
    } while(connectionResult == -1);

    int sendResult;
    while(running.get()) {
        while(!frameQueue.empty()) { // send all queued frames first
            frameLock.lock();
            cv::Mat* matBuffer = frameQueue.front(); // get the frame
            // send the header
            sendResult = send(sock, &MAT_HEADER, sizeof(MAT_HEADER), 0);
            send(sock, matBuffer, 921600, 0); // send whole frame to socket
            if(sendResult == -1) {
                std::cout << "Error occured while sending frame. Unable to deliver\r\n";
                delete[] matBuffer; // delete the error frame
                frameQueue.pop(); // remove dangling pointer
                frameLock.unlock();
                continue;
            }
            delete[] matBuffer; // delete the frame
            frameQueue.pop(); // remove dangling pointer
            frameLock.unlock();
        }
        while(!messageQueue.empty()) { // send all queued strings second
            messageLock.lock();
            std::string s = messageQueue.front(); // get the string
            // construct the header
            uint32_t header[1] = {BLANK_STRING_HEADER};
            header[0] = (header[0] & static_cast<uint32_t>(s.length()));
            sendResult = send(sock, header, sizeof(header), 0); // send header
            if(sendResult == -1) {
                std::cout << "Error occured while sending message. Unable to deliver\r\n";
                messageQueue.pop();
                messageLock.unlock();
                continue;
            }
            send(sock, s.c_str(), s.size(), 0); // send string
            messageQueue.pop(); // pop the message from the queue
            messageLock.unlock();
        }
    }
}

/*
* Pushes a message onto the message queue. 
* If this message results in a queue greater than FRAME_QUEUE_LIMIT, 
* queue.pop() is called
*/
void TCP_Client::sendMessage(cv::Mat* image) {
    frameQueue.push(image); 
    if(frameQueue.size() > FRAME_QUEUE_LIMIT) {
        const std::lock_guard<std::mutex> lock(frameLock);
        cv::Mat* DELETE_THIS = frameQueue.front();
        delete[] DELETE_THIS;
        frameQueue.pop();
    }
}

// message's number of bytes should not exceed max representable by 24 bits
void TCP_Client::sendMessage(const std::string& message) {
    messageQueue.push(message);
    if(messageQueue.size() > MESSAGE_QUEUE_LIMIT) {
        messageQueue.pop();
    }
}

void TCP_Client::stop() {
    shouldThreadBeRunning = false;
    client.join();
    close(sock);
}