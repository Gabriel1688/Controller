#include "DriverStation.h"
#include "DriverStationTypes.h"
#include "EventVector.h"
#include <array>
#include <string>

#include <mutex>

extern "C" {
namespace hal {
extern void InitializeDriverStation();
}
HAL_Bool HAL_RefreshDSData(void);
}

struct Instance {
    Instance();
    ~Instance();

    EventVector refreshEvents;

    // Joystick button rising/falling edge flags
    std::mutex buttonEdgeMutex;
    std::array<HAL_JoystickButtons, DriverStation::kJoystickPorts> previousButtonStates;
    std::array<uint32_t, DriverStation::kJoystickPorts> joystickButtonsPressed;
    std::array<uint32_t, DriverStation::kJoystickPorts> joystickButtonsReleased;

    // Robot state status variables
    bool userInDisabled = false;
    bool userInAutonomous = false;
    bool userInTeleop = false;
    bool userInTest = false;
    int nextMessageTime = 0;//0_s;
};

static constexpr auto kJoystickUnpluggedMessageInterval = 1;//1_s;

static Instance &GetInstance() {
    static Instance instance;
    return instance;
}

Instance::Instance() {
    //NOTE:: need to implement following logic in the RasperryPi.
    //HAL_Initialize(500, 0);
    //hal::init::InitializeHAL();
    //hal::RestartTiming();
    //hal::InitializeDriverStation();
    hal::InitializeDriverStation();

    // All joysticks should default to having zero axes, povs and buttons, so
    // uninitialized memory doesn't get sent to motor controllers.
    for (unsigned int i = 0; i < DriverStation::kJoystickPorts; i++) {
        joystickButtonsPressed[i] = 0;
        joystickButtonsReleased[i] = 0;
        previousButtonStates[i].count = 0;
        previousButtonStates[i].buttons = 0;
    }
}

Instance::~Instance() {
}
extern int32_t HAL_GetJoystickButtonsInternal(int32_t joystickNum, HAL_JoystickButtons *buttons);

bool DriverStation::GetStickButton(int stick, int button) {

    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);

    return buttons.buttons & 1 << (button - 1);
}

bool DriverStation::GetStickButtonPressed(int stick, int button) {

    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);

    auto &inst = ::GetInstance();
    std::unique_lock lock(inst.buttonEdgeMutex);
    // If button was pressed, clear flag and return true
    if (inst.joystickButtonsPressed[stick] & 1 << (button - 1)) {
        inst.joystickButtonsPressed[stick] &= ~(1 << (button - 1));
        return true;
    }
    return false;
}

bool DriverStation::GetStickButtonReleased(int stick, int button) {
    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);

    auto &inst = ::GetInstance();
    std::unique_lock lock(inst.buttonEdgeMutex);
    // If button was released, clear flag and return true
    if (inst.joystickButtonsReleased[stick] & 1 << (button - 1)) {
        inst.joystickButtonsReleased[stick] &= ~(1 << (button - 1));
        return true;
    }
    return false;
}

double DriverStation::GetStickAxis(int stick, int axis) {
    HAL_JoystickAxes axes;
    HAL_GetJoystickAxes(stick, &axes);
    return axes.axes[axis];
}

int DriverStation::GetStickPOV(int stick, int pov) {
    HAL_JoystickPOVs povs;
    HAL_GetJoystickPOVs(stick, &povs);
    return povs.povs[pov];
}

int DriverStation::GetStickButtons(int stick) {
    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);
    return buttons.buttons;
}

int DriverStation::GetStickAxisCount(int stick) {
    HAL_JoystickAxes axes;
    HAL_GetJoystickAxes(stick, &axes);
    return axes.count;
}

int DriverStation::GetStickPOVCount(int stick) {
    HAL_JoystickPOVs povs;
    HAL_GetJoystickPOVs(stick, &povs);
    return povs.count;
}

int DriverStation::GetStickButtonCount(int stick) {
    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(stick, &buttons);
    return buttons.count;
}

bool DriverStation::IsJoystickConnected(int stick) {
    return GetStickAxisCount(stick) > 0 || GetStickButtonCount(stick) > 0 || GetStickPOVCount(stick) > 0;
}

bool DriverStation::IsEnabled() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.enabled && controlWord.dsAttached;
}

bool DriverStation::IsDisabled() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return !(controlWord.enabled && controlWord.dsAttached);
}

bool DriverStation::IsEStopped() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.eStop;
}

bool DriverStation::IsAutonomous() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.autonomous;
}

bool DriverStation::IsAutonomousEnabled() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.autonomous && controlWord.enabled;
}

bool DriverStation::IsTeleop() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return !(controlWord.autonomous || controlWord.test);
}

bool DriverStation::IsTeleopEnabled() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return !controlWord.autonomous && !controlWord.test && controlWord.enabled;
}

bool DriverStation::IsDSAttached() {
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    return controlWord.dsAttached;
}

// std::string DriverStation::GetEventName() {
//    HAL_MatchInfo info;
//    HAL_GetMatchInfo(&info);
//    return info.eventName;
//}

bool DriverStation::WaitForDsConnection(int timeout) {
    bool result = false;
    wpi::Event event{true, false};
    HAL_ProvideNewDataEventHandle(event.GetHandle());
    if (timeout == 0) {
        result = wpi::WaitForObject(event.GetHandle());
    } else {
        result = wpi::WaitForObject(event.GetHandle(), timeout, nullptr);
    }

    HAL_RemoveNewDataEventHandle(event.GetHandle());
    return result;
}

double DriverStation::GetBatteryVoltage() {
    int32_t status = 0;
    double voltage;
    //    double voltage = HAL_GetVinVoltage(&status);
    //    FRC_CheckErrorStatus(status, "getVinVoltage");

    return voltage;
}

/**
 * Copy data from the DS task for the user.
 *
 * If no new data exists, it will just be returned, otherwise
 * the data will be copied from the DS polling loop.
 */
void DriverStation::RefreshData() {
    HAL_RefreshDSData();
    auto &inst = ::GetInstance();
    {
        // Compute the pressed and released buttons
        HAL_JoystickButtons currentButtons;
        std::unique_lock lock(inst.buttonEdgeMutex);

        for (int32_t i = 0; i < DriverStation::kJoystickPorts; i++) {
            HAL_GetJoystickButtons(i, &currentButtons);

            // If buttons weren't pressed and are now, set flags in m_buttonsPressed
            inst.joystickButtonsPressed[i] |=
                ~inst.previousButtonStates[i].buttons & currentButtons.buttons;

            // If buttons were pressed and aren't now, set flags in m_buttonsReleased
            inst.joystickButtonsReleased[i] |=
                inst.previousButtonStates[i].buttons & ~currentButtons.buttons;

            inst.previousButtonStates[i] = currentButtons;
        }
    }

    inst.refreshEvents.Wakeup();
}

void DriverStation::ProvideRefreshedDataEventHandle(WPI_EventHandle handle) {
    auto &inst = ::GetInstance();
    inst.refreshEvents.Add(handle);
}

void DriverStation::RemoveRefreshedDataEventHandle(WPI_EventHandle handle) {
    auto &inst = ::GetInstance();
    inst.refreshEvents.Remove(handle);
}
