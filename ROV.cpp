#include <iostream>
#include "Drive.hpp"
#include "Manipulator.hpp"
#include "TCPClientServer.hpp"
#include "Joystick.hpp"
#include "ServoDriver.hpp"
#include "IMU.hpp"

Component* components[1];
// TCP_Client* client = new TCP_Client();
// TCP_Server* server = new TCP_Server();
// ServoDriver* servoDriver = new ServoDriver();
// Joystick js = Joystick(server); // create network joystick
// Joystick js = Joystick("/dev/input/by-id/usb-Logitech_Extreme_3D_pro_00000000002A-joystick"); // local joystick

void init() {
    // components[0] = new Drive();
    // components[0] = new Manipulator(&js, servoDriver);
    components[0] = new IMU();
    // server->start();
    // client->start();
}

void wait() {

}

void stop() {

}

int main() {
    // initialize ROV parts
    init();
    
    while(true) {
        for(Component* component : components) {
           component->Update();
        }
    }
    
    stop();
    return 0;
}
