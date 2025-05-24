
#include <libwebsockets.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <assert.h>
#include <iostream>
#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"  
#include "spdlog/fmt/ostr.h" 
#include <memory>
#include <chrono>
#include "ds/GenericHID.h"
#include "ds/XboxController.h"
#include "ds/EventLoop.h"
#include "ds/BooleanEvent.h"
#include "mqtt/wrapper.h"
#include "Constants.h"
#include "motor/CAN.h"
#include "Robot.h"

using namespace spdlog;

static const auto SHOT_VELOCITY = 200;    //200_rpm;
static const auto TOLERANCE = 8;          // 8_rpm;
static const auto KICKER_THRESHOLD = 15;  // 15_mm;

Robot::Robot() {
}

Robot::~Robot() {}

int main() {
    return StartRobot<Robot>();
}