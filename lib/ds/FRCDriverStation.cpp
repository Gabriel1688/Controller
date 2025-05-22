#include <atomic>
#include <cstring>
#include <limits>
#include "EventVector.h"
#include <mutex>
#include "DriverStation.h"
#include "DriverStationTypes.h"
#include "FRCComm.h"
#include "mqtt/mqttClient.h"

static_assert(sizeof(int32_t) >= sizeof(int),  "FRC_NetworkComm status variable is larger than 32 bits");

namespace {
    struct HAL_JoystickAxesInt {
        int16_t count;
        int16_t axes[HAL_kMaxJoystickAxes];
    };
}  // namespace

struct JoystickDataCache {
        JoystickDataCache() { std::memset(this, 0, sizeof(*this)); }
        void Update(const char* payload);

        HAL_JoystickAxes axes[HAL_kMaxJoysticks];
        HAL_JoystickPOVs povs[HAL_kMaxJoysticks];
        HAL_JoystickButtons buttons[HAL_kMaxJoysticks];
        HAL_ControlWord controlWord;
};

static_assert(std::is_standard_layout_v<JoystickDataCache>);

struct FRCDriverStation {
        EventVector newDataEvents;
};


static ::FRCDriverStation* driverStation;

// Message and Data variables
static std::mutex msgMutex;

static int32_t HAL_GetJoystickAxesInternal(__attribute__((unused)) const char* payload, int32_t joystickNum, HAL_JoystickAxes* axes) {
    HAL_JoystickAxesInt netcommAxes;

    int retVal = FRC_NetworkCommunication_getJoystickAxes( joystickNum, reinterpret_cast<JoystickAxes_t*>(&netcommAxes), HAL_kMaxJoystickAxes);

    // copy integer values to double values
    axes->count = netcommAxes.count;
    // current scaling is -128 to 127, can easily be patched in the future by changing this function.
    for (int32_t i = 0; i < netcommAxes.count; i++) {
        int8_t value = netcommAxes.axes[i];
        axes->raw[i] = value;
        if (value < 0) {
            axes->axes[i] = value / 128.0;
        } else {
            axes->axes[i] = value / 127.0;
        }
    }
    return retVal;
}

static int32_t HAL_GetJoystickPOVsInternal(__attribute__((unused)) const char* payload, int32_t joystickNum, HAL_JoystickPOVs* povs) {
    return FRC_NetworkCommunication_getJoystickPOVs( joystickNum, reinterpret_cast<JoystickPOV_t*>(povs), HAL_kMaxJoystickPOVs);
}

static int32_t HAL_GetJoystickButtonsInternal(__attribute__((unused)) const char* payload, int32_t joystickNum, HAL_JoystickButtons* buttons) {
    return FRC_NetworkCommunication_getJoystickButtons(joystickNum, &buttons->buttons, &buttons->count);
}

#if 0
------------
|    00     |   ->  sent_robot_packets >> 8
------------
|    01     |   ->  sent_robot_packets
------------
|    02     |   ->  cTagCommVersion
------------
|    03     |   ->  control_code
------------
|    04     |   ->  request_code
------------
|    05     |   ->  station_code
------------
|    06     |   ->  get_joystick_size
------------
|    07     |   ->  cTagJoystick
------------
|    08     |   ->  NumAxes
------------
|    09     |   ->  Axis data <FloatToByte>
------------
|    10     |   ->  Axis data <FloatToByte>
------------
|    11     |   ->  NumButtons
------------
|    12     |   ->  button_flags >> 8
------------
|    13     |   ->  button_flags
------------
|    14     |   ->  NumHats
------------
|    15     |   ->  JoystickHat(0, 0) >> 8
------------
|    16     |   ->  JoystickHat(0, 0)
------------
|    17     |   ->  JoystickHat(0, 1) >> 8
------------
|    18     |   ->  JoystickHat(0, 1)
------------
#endif

void JoystickDataCache::Update(const char* payload) {
    HAL_GetJoystickAxesInternal(payload,0, &axes[0]);
    HAL_GetJoystickPOVsInternal(payload,0, &povs[0]);
    HAL_GetJoystickButtonsInternal(payload,0, &buttons[0]);
    FRC_NetworkCommunication_getControlWord(reinterpret_cast<ControlWord_t*>(&controlWord));
}
#if 0
https://github.com/momentumfrc/FRC-Robot-Controller/blob/master/protocol.py

       DS_StrAppend(&data, get_joystick_size(i));
      DS_StrAppend(&data, cTagJoystick);

      /* Add axis data */
DS_StrAppend(&data, DS_GetJoystickNumAxes(i));
for (j = 0; j < DS_GetJoystickNumAxes(i); ++j)
DS_StrAppend(&data, DS_FloatToByte(DS_GetJoystickAxis(i, j), 1));

/* Generate button data */
uint16_t button_flags = 0;
for (j = 0; j < DS_GetJoystickNumButtons(i); ++j)
button_flags += DS_GetJoystickButton(i, j) ? (int)pow(2, j) : 0;

/* Add button data */
DS_StrAppend(&data, DS_GetJoystickNumButtons(i));
DS_StrAppend(&data, (uint8_t)(button_flags >> 8));
DS_StrAppend(&data, (uint8_t)(button_flags));

/* Add hat data */
DS_StrAppend(&data, DS_GetJoystickNumHats(i));
for (j = 0; j < DS_GetJoystickNumHats(i); ++j)
{
DS_StrAppend(&data, (uint8_t)(DS_GetJoystickHat(i, j) >> 8));
DS_StrAppend(&data, (uint8_t)(DS_GetJoystickHat(i, j)));
}
#endif

#define CHECK_JOYSTICK_NUMBER(stickNum)                  \
  if ((stickNum) < 0 || (stickNum) >= HAL_kMaxJoysticks) \
  return PARAMETER_OUT_OF_RANGE

static HAL_ControlWord newestControlWord;
static JoystickDataCache caches[3];
static JoystickDataCache* currentRead = &caches[0];
static JoystickDataCache* currentReadLocal = &caches[0];
static std::atomic<JoystickDataCache*> currentCache{nullptr};
static JoystickDataCache* lastGiven = &caches[1];
static JoystickDataCache* cacheToUpdate = &caches[2];

static std::mutex cacheMutex;

/**
 * Retrieve the Joystick Descriptor for particular slot.
 *
 * @param[out] desc descriptor (data transfer object) to fill in. desc is filled
 *                  in regardless of success. In other words, if descriptor is
 *                  not available, desc is filled in with default values
 *                  matching the init-values in Java and C++ Driverstation for
 *                  when caller requests a too-large joystick index.
 * @return error code reported from Network Comm back-end.  Zero is good,
 *         nonzero is bad.
 */
static int32_t HAL_GetJoystickDescriptorInternal(int32_t joystickNum, HAL_JoystickDescriptor* desc) {
    desc->isXbox = 0;
    desc->type = (std::numeric_limits<uint8_t>::max)();
    desc->name[0] = '\0';
    desc->axisCount = HAL_kMaxJoystickAxes; /* set to the desc->axisTypes's capacity */
    desc->buttonCount = 0;
    desc->povCount = 0;
    int retval = FRC_NetworkCommunication_getJoystickDesc(
            joystickNum, &desc->isXbox, &desc->type,
            reinterpret_cast<char*>(&desc->name), &desc->axisCount,
            reinterpret_cast<uint8_t*>(&desc->axisTypes), &desc->buttonCount,
            &desc->povCount);
    /* check the return, if there is an error and the RIOimage predates FRC2017,
     * then axisCount needs to be cleared */
    if (retval != 0) {
        /* set count to zero so downstream code doesn't decode invalid axisTypes. */
        desc->axisCount = 0;
    }
    return retval;
}
extern "C" {

void InitializeFRCDriverStation() {
        std::memset(&newestControlWord, 0, sizeof(newestControlWord));
        static FRCDriverStation ds;
        driverStation = &ds;
}

int32_t HAL_GetControlWord(HAL_ControlWord *controlWord) {
    std::scoped_lock lock{cacheMutex};
    *controlWord = newestControlWord;
    return 0;
}

int32_t HAL_GetJoystickAxes(int32_t joystickNum, HAL_JoystickAxes *axes) {
    std::scoped_lock lock{cacheMutex};
    *axes = currentRead->axes[joystickNum];
    return 0;
}

int32_t HAL_GetJoystickPOVs(int32_t joystickNum, HAL_JoystickPOVs *povs) {
    std::scoped_lock lock{cacheMutex};
    *povs = currentRead->povs[joystickNum];
    return 0;
}

int32_t HAL_GetJoystickButtons(int32_t joystickNum, HAL_JoystickButtons *buttons) {
    std::scoped_lock lock{cacheMutex};
    *buttons = currentRead->buttons[joystickNum];
    return 0;
}

void HAL_GetAllJoystickData(HAL_JoystickAxes *axes, HAL_JoystickPOVs *povs, HAL_JoystickButtons *buttons) {
    std::scoped_lock lock{cacheMutex};
    std::memcpy(axes, currentRead->axes, sizeof(currentRead->axes));
    std::memcpy(povs, currentRead->povs, sizeof(currentRead->povs));
    std::memcpy(buttons, currentRead->buttons, sizeof(currentRead->buttons));
}

int32_t HAL_GetJoystickDescriptor(__attribute__((unused)) int32_t joystickNum, __attribute__((unused)) HAL_JoystickDescriptor *desc) {
    //std::scoped_lock lock{tcpCacheMutex};
    // *desc = tcpCurrent.descriptors[joystickNum];
    return 0;
}

int32_t HAL_GetJoystickAxisType(int32_t joystickNum, int32_t axis) {
    HAL_JoystickDescriptor joystickDesc;
    if (HAL_GetJoystickDescriptor(joystickNum, &joystickDesc) < 0) {
        return -1;
    } else {
        return joystickDesc.axisTypes[axis];
    }
}

int32_t HAL_SetJoystickOutputs(int32_t joystickNum, int64_t outputs, int32_t leftRumble, int32_t rightRumble) {
    return FRC_NetworkCommunication_setJoystickOutputs(joystickNum, outputs,
                                                       leftRumble, rightRumble);
}

static void newDataOccur(const void* payload, __attribute__((unused)) uint32_t payload_len) {
    cacheToUpdate->Update(static_cast<const char*>(payload));

    JoystickDataCache *given = cacheToUpdate;
    JoystickDataCache *prev = currentCache.exchange(cacheToUpdate);
    if (prev == nullptr) {
        cacheToUpdate = currentReadLocal;
        currentReadLocal = lastGiven;
    } else {
        // Current read local does not update
        cacheToUpdate = prev;
    }
    lastGiven = given;

    driverStation->newDataEvents.Wakeup();
}

HAL_Bool HAL_RefreshDSData(void) {
    HAL_ControlWord controlWord;
    std::memset(&controlWord, 0, sizeof(controlWord));
    FRC_NetworkCommunication_getControlWord(
            reinterpret_cast<ControlWord_t *>(&controlWord));
    JoystickDataCache *prev;
    {
        std::scoped_lock lock{cacheMutex};
        prev = currentCache.exchange(nullptr);
        if (prev != nullptr) {
            currentRead = prev;
        }
        // If newest state shows we have a DS attached, just use the
        // control word out of the cache, As it will be the one in sync
        // with the data. If no data has been updated, at this point,
        // and a DS wasn't attached previously, this will still return
        // a zeroed out control word, with is the correct state for
        // no new data.

        if (!controlWord.dsAttached) {
            // If the DS is not attached, we need to zero out the control word.
            // This is because HAL_RefreshDSData is called asynchronously from
            // the DS data. The dsAttached variable comes directly from netcomm
            // and could be updated before the caches are. If that happens,
            // we would end up returning the previous cached control word,
            // which is out of sync with the current control word and could
            // break invariants such as which alliance station is in used.
            // Also, when the DS has never been connected the rest of the fields
            // in control word are garbage, so we also need to zero out in that
            // case too
            std::memset(&currentRead->controlWord, 0,
                        sizeof(currentRead->controlWord));
        }
        newestControlWord = currentRead->controlWord;
    }
    return prev != nullptr;
}
void HAL_ProvideNewDataEventHandle(WPI_EventHandle handle) {
    driverStation->newDataEvents.Add(handle);
}

void HAL_RemoveNewDataEventHandle(WPI_EventHandle handle) {
    driverStation->newDataEvents.Remove(handle);
}

namespace hal {
    void InitializeDriverStation() {
        // Set up the occur function internally with NetComm
        g_mqttClient_ptr->SetOccurFuncPointer(newDataOccur);
    }
  }
}