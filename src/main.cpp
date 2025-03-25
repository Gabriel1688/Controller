
#include <libwebsockets.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <assert.h>

#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"  
#include "spdlog/fmt/ostr.h" 
#include <memory>
#include "agent.h"
#include <chrono>
#include "../lib/mqtt/wrapper.h"

using namespace spdlog;

void loadConfig(const std::string& fileName, std::string& mode);

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//#include <frc/Encoder.h>
//#include <frc/Joystick.h>
//#include <frc/TimedRobot.h>
//#include <frc/Ultrasonic.h>
//#include <frc/controller/PIDController.h>
//#include <frc/controller/SimpleMotorFeedforward.h>
//#include <frc/event/BooleanEvent.h>
//#include <frc/event/EventLoop.h>
//#include <frc/motorcontrol/PWMSparkMax.h>
//#include <units/angular_velocity.h>
//#include <units/length.h>
//#include <units/time.h>
//#include <units/voltage.h>

#include "../lib/framework/TimedRobot.h"
#include "../lib/ds/EventLoop.h"

static const auto SHOT_VELOCITY = 200;//200_rpm;
static const auto TOLERANCE = 8; // 8_rpm;
static const auto KICKER_THRESHOLD = 15; // 15_mm;

class Robot : public TimedRobot {
public:
    Robot() {};
#if 0
        m_controller.SetTolerance(TOLERANCE.value());

        frc::BooleanEvent isBallAtKicker{&m_loop, [&kickerSensor = m_kickerSensor] {
            return kickerSensor.GetRange() <
                   KICKER_THRESHOLD;
        }};
        frc::BooleanEvent intakeButton{
                &m_loop, [&joystick = m_joystick] { return joystick.GetRawButton(2); }};

        // if the thumb button is held
        (intakeButton
         // and there is not a ball at the kicker
         && !isBallAtKicker)
                // activate the intake
                .IfHigh([&intake = m_intake] { intake.Set(0.5); });

        // if the thumb button is not held
        (!intakeButton
         // or there is a ball in the kicker
         || isBallAtKicker)
                // stop the intake
                .IfHigh([&intake = m_intake] { intake.Set(0.0); });

        frc::BooleanEvent shootTrigger{
                &m_loop, [&joystick = m_joystick] { return joystick.GetTrigger(); }};

        // if the trigger is held
        shootTrigger
                // accelerate the shooter wheel
                .IfHigh([&shooter = m_shooter, &controller = m_controller, &ff = m_ff,
                                &encoder = m_shooterEncoder] {
                    shooter.SetVoltage(
                            units::volt_t{controller.Calculate(encoder.GetRate(),
                                                               SHOT_VELOCITY.value())} +
                            ff.Calculate(units::radians_per_second_t{SHOT_VELOCITY}));
                });
        // if not, stop
        (!shootTrigger).IfHigh([&shooter = m_shooter] { shooter.Set(0.0); });

        frc::BooleanEvent atTargetVelocity =
                frc::BooleanEvent(
                        &m_loop,
                        [&controller = m_controller] { return controller.AtSetpoint(); })
                        // debounce for more stability
                        .Debounce(0.2_s);

        // if we're at the target velocity, kick the ball into the shooter wheel
        atTargetVelocity.IfHigh([&kicker = m_kicker] { kicker.Set(0.7); });

        // when we stop being at the target velocity, it means the ball was shot
        atTargetVelocity
                .Falling()
                        // so stop the kicker
                .IfHigh([&kicker = m_kicker] { kicker.Set(0.0); });
    }
#endif
    void RobotPeriodic() override { m_loop.Poll(); }

private:
    EventLoop m_loop{};
    //Joystick m_joystick{0};
};

int main() {
    return StartRobot<Robot>();
}

#if 0

int main(int argc, const char **argv)
{
    spdlog::warn("Easy padding in numbers like {:08d}", 12);
    spdlog::info("Welcome to spdlog version {}.{}.{}  !", SPDLOG_VER_MAJOR, SPDLOG_VER_MINOR,SPDLOG_VER_PATCH);
    std::string mode;
    //std::string configFile = "../../config/config.txt";
    std::string configFile = "./config.txt";
    loadConfig(configFile,mode);

    spdlog::info("run as robot.");
    client_create();
    g_mqttClient_ptr->loadConfig(configFile);
    g_mqttClient_ptr->Start();

//    std::shared_ptr<Console> console = std::make_shared<Console>();
//    console->setClient(g_mqttClient_ptr);
//    console->Start();
    while(1) {
         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void loadConfig(const std::string& fileName, std::string& mode)
{
     std::string line;
     std::size_t pos;
     std::string result;

     std::ifstream ifs(fileName);

     while (std::getline(ifs, line)) {
         line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());
         pos = line.find("=");
         if (pos != std::string::npos) {
             std::string key = line.substr(0, pos);
             std::string value = line.substr(pos + 1);
             if (key == "mode") {
                 mode = value;
             } 
         }
     }
}
#endif

