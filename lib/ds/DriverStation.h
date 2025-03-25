#pragma once
#include <optional>
#include <string>
#include "DriverStationTypes.h"
typedef int32_t HAL_Bool;
//#include <wpi/Synchronization.h>

/**
 * Refresh the DS control word.
 *
 * @return true if updated
 */
//HAL_Bool HAL_RefreshDSData(void);

namespace hal {
    void InitializeDriverStation();
}

#ifdef __cplusplus
extern "C" {
#endif

/**
* Gets the current control word of the driver station.
*
* The control word contains the robot state.
*
* @param controlWord the control word (out)
* @return the error code, or 0 for success
*/
int32_t HAL_GetControlWord(HAL_ControlWord* controlWord);

/**
 * Gets the axes of a specific joystick.
 *
 * @param joystickNum the joystick number
 * @param axes        the axes values (output)
 * @return the error code, or 0 for success
 */
int32_t HAL_GetJoystickAxes(int32_t joystickNum, HAL_JoystickAxes* axes);

/**
 * Gets the POVs of a specific joystick.
 *
 * @param joystickNum the joystick number
 * @param povs        the POV values (output)
 * @return the error code, or 0 for success
 */
int32_t HAL_GetJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs* povs);

/**
 * Gets the buttons of a specific joystick.
 *
 * @param joystickNum the joystick number
 * @param buttons     the button values (output)
 * @return the error code, or 0 for success
 */
int32_t HAL_GetJoystickButtons(int32_t joystickNum,
                               HAL_JoystickButtons* buttons);

void HAL_GetAllJoystickData(HAL_JoystickAxes* axes, HAL_JoystickPOVs* povs,
                            HAL_JoystickButtons* buttons);

/**
 * Retrieves the Joystick Descriptor for particular slot.
 *
 * @param joystickNum the joystick number
 * @param[out] desc   descriptor (data transfer object) to fill in. desc is
 *                    filled in regardless of success. In other words, if
 *                    descriptor is not available, desc is filled in with
 *                    default values matching the init-values in Java and C++
 *                    Driver Station for when caller requests a too-large
 *                    joystick index.
 * @return error code reported from Network Comm back-end.  Zero is good,
 *         nonzero is bad.
 */
int32_t HAL_GetJoystickDescriptor(int32_t joystickNum,
                                  HAL_JoystickDescriptor* desc);
/**
 * Gets the type of joystick connected.
 *
 * This is device specific, and different depending on what system input type
 * the joystick uses.
 *
 * @param joystickNum the joystick number
 * @return the enumerated joystick type
 */
int32_t HAL_GetJoystickType(int32_t joystickNum);

/**
 * Gets the name of a joystick.
 *
 * The returned string must be freed with WPI_FreeString
 *
 * @param name the joystick name string
 * @param joystickNum the joystick number
 */
void HAL_GetJoystickName(struct WPI_String* name, int32_t joystickNum);

/**
 * Gets the type of a specific joystick axis.
 *
 * This is device specific, and different depending on what system input type
 * the joystick uses.
 *
 * @param joystickNum the joystick number
 * @param axis        the axis number
 * @return the enumerated axis type
 */
int32_t HAL_GetJoystickAxisType(int32_t joystickNum, int32_t axis);

#ifdef __cplusplus
}  // extern "C"
#endif
/** @} */

/**
 * Provide access to the network communication data to / from the Driver Station.
 */
class DriverStation final {
    public:

     /**
       * The type of robot match that the robot is part of.
       */
    enum MatchType {
            /// None.
            kNone,
            /// Practice.
            kPractice,
            /// Qualification.
            kQualification,
            /// Elimination.
            kElimination
        };

    /// Number of Joystick ports.
    static constexpr int kJoystickPorts = 6;

    /**
      * The state of one joystick button. %Button indexes begin at 1.
      *
      * @param stick  The joystick to read.
      * @param button The button index, beginning at 1.
      * @return The state of the joystick button.
      */
    static bool GetStickButton(int stick, int button);

    /**
      * Whether one joystick button was pressed since the last check. %Button
      * indexes begin at 1.
      *
      * @param stick  The joystick to read.
      * @param button The button index, beginning at 1.
      * @return Whether the joystick button was pressed since the last check.
      */
    static bool GetStickButtonPressed(int stick, int button);

    /**
      * Whether one joystick button was released since the last check. %Button
      * indexes begin at 1.
      *
      * @param stick  The joystick to read.
      * @param button The button index, beginning at 1.
      * @return Whether the joystick button was released since the last check.
      */
    static bool GetStickButtonReleased(int stick, int button);

    /**
      * Get the value of the axis on a joystick.
      *
      * This depends on the mapping of the joystick connected to the specified
      * port.
      *
      * @param stick The joystick to read.
      * @param axis  The analog axis value to read from the joystick.
      * @return The value of the axis on the joystick.
      */
    static double GetStickAxis(int stick, int axis);

    /**
      * Get the state of a POV on the joystick.
      *
      * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
      */
    static int GetStickPOV(int stick, int pov);

    /**
      * The state of the buttons on the joystick.
      *
      * @param stick The joystick to read.
      * @return The state of the buttons on the joystick.
      */
    static int GetStickButtons(int stick);

    /**
      * Returns the number of axes on a given joystick port.
      *
      * @param stick The joystick port number
      * @return The number of axes on the indicated joystick
      */
    static int GetStickAxisCount(int stick);

    /**
      * Returns the number of POVs on a given joystick port.
      *
      * @param stick The joystick port number
      * @return The number of POVs on the indicated joystick
      */
    static int GetStickPOVCount(int stick);

    /**
      * Returns the number of buttons on a given joystick port.
      *
      * @param stick The joystick port number
      * @return The number of buttons on the indicated joystick
      */
    static int GetStickButtonCount(int stick);

    /**
      * Returns a boolean indicating if the controller is an xbox controller.
      *
      * @param stick The joystick port number
      * @return A boolean that is true if the controller is an xbox controller.
      */
    static bool GetJoystickIsXbox(int stick);

    /**
      * Returns the type of joystick at a given port.
      *
      * @param stick The joystick port number
      * @return The HID type of joystick at the given port
      */
    static int GetJoystickType(int stick);

    /**
      * Returns the name of the joystick at the given port.
      *
      * @param stick The joystick port number
      * @return The name of the joystick at the given port
      */
    static std::string GetJoystickName(int stick);

    /**
      * Returns the types of Axes on a given joystick port.
      *
      * @param stick The joystick port number and the target axis
      * @param axis  The analog axis value to read from the joystick.
      * @return What type of axis the axis is reporting to be
      */
    static int GetJoystickAxisType(int stick, int axis);

    /**
      * Returns if a joystick is connected to the Driver Station.
      *
      * This makes a best effort guess by looking at the reported number of axis,
      * buttons, and POVs attached.
      *
      * @param stick The joystick port number
      * @return true if a joystick is connected
      */
    static bool IsJoystickConnected(int stick);

    /**
      * Check if the DS has enabled the robot.
      *
      * @return True if the robot is enabled and the DS is connected
      */
    static bool IsEnabled();

    /**
      * Check if the robot is disabled.
      *
      * @return True if the robot is explicitly disabled or the DS is not connected
      */
    static bool IsDisabled();

    /**
      * Check if the robot is e-stopped.
      *
      * @return True if the robot is e-stopped
      */
    static bool IsEStopped();

    /**
      * Check if the DS is commanding autonomous mode.
      *
      * @return True if the robot is being commanded to be in autonomous mode
      */
    static bool IsAutonomous();

    /**
      * Check if the DS is commanding autonomous mode and if it has enabled the
      * robot.
      *
      * @return True if the robot is being commanded to be in autonomous mode and
      * enabled.
      */
    static bool IsAutonomousEnabled();

    /**
      * Check if the DS is commanding teleop mode.
      *
      * @return True if the robot is being commanded to be in teleop mode
      */
    static bool IsTeleop();

    /**
      * Check if the DS is commanding teleop mode and if it has enabled the robot.
      *
      * @return True if the robot is being commanded to be in teleop mode and
      * enabled.
      */
    static bool IsTeleopEnabled();

     /**
       * Check if the DS is commanding test mode.
       *
       * @return True if the robot is being commanded to be in test mode
       */
    static bool IsTest();

    /**
      * Check if the DS is commanding Test mode and if it has enabled the robot.
      *
      * @return True if the robot is being commanded to be in Test mode and
      * enabled.
      */
    static bool IsTestEnabled();

    /**
      * Check if the DS is attached.
      *
      * @return True if the DS is connected to the robot
      */
    static bool IsDSAttached();

    /**
      * Wait for a DS connection.
      *
      * @param timeout timeout in seconds. 0 for infinite.
      * @return true if connected, false if timeout
      */
    static bool WaitForDsConnection(int  timeout);

    /**
      * Read the battery voltage.
      *
      * @return The battery voltage in Volts.
      */
    static double GetBatteryVoltage();

    /**
      * Copy data from the DS task for the user. If no new data exists, it will
      * just be returned, otherwise the data will be copied from the DS polling
      * loop.
    */
    static void RefreshData();

    /**
      * Registers the given handle for DS data refresh notifications.
      *
      * @param handle The event handle.
      */
//    static void ProvideRefreshedDataEventHandle(WPI_EventHandle handle);

    /**
      * Unregisters the given handle from DS data refresh notifications.
      *
      * @param handle The event handle.
      */
//    static void RemoveRefreshedDataEventHandle(WPI_EventHandle handle);

    /**
      * Starts logging DriverStation data to data log. Repeated calls are ignored.
      *
      * @param log data log
      * @param logJoysticks if true, log joystick data
      */
//    static void StartDataLog(wpi::log::DataLog& log, bool logJoysticks = true);

    private:
        DriverStation() = default;
};
