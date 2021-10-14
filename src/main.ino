#include "Controller.hpp"

Controller* controller;

void setup(){
    controller = new Controller();
}

void loop(){
    controller->debugPrint();
    delay(30);
}