#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>

//#include <fmt/format.h>
//#include <wpi/SafeThread.h>
//#include <wpi/SmallVector.h>
//#include "HALInitializer.h"

#include <condition_variable>
#include "EventVector.h"
#include <mutex>
#include "DriverStation.h"
#include "DriverStationTypes.h"
#include "FRCComm.h"

static_assert(sizeof(int32_t) >= sizeof(int),  "FRC_NetworkComm status variable is larger than 32 bits");

namespace {
    struct HAL_JoystickAxesInt {
        int16_t count;
        int16_t axes[HAL_kMaxJoystickAxes];
    };
}  // namespace

struct JoystickDataCache {
        JoystickDataCache() { std::memset(this, 0, sizeof(*this)); }
        void Update();

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

static int32_t HAL_GetJoystickAxesInternal(int32_t joystickNum, HAL_JoystickAxes* axes) {
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

static int32_t HAL_GetJoystickPOVsInternal(int32_t joystickNum, HAL_JoystickPOVs* povs) {
    return FRC_NetworkCommunication_getJoystickPOVs( joystickNum, reinterpret_cast<JoystickPOV_t*>(povs), HAL_kMaxJoystickPOVs);
}

static int32_t HAL_GetJoystickButtonsInternal(int32_t joystickNum, HAL_JoystickButtons* buttons) {
    return FRC_NetworkCommunication_getJoystickButtons(joystickNum, &buttons->buttons, &buttons->count);
}

void JoystickDataCache::Update() {
    for (int i = 0; i < HAL_kMaxJoysticks; i++) {
        HAL_GetJoystickAxesInternal(i, &axes[i]);
        HAL_GetJoystickPOVsInternal(i, &povs[i]);
        HAL_GetJoystickButtonsInternal(i, &buttons[i]);
    }
    FRC_NetworkCommunication_getControlWord(reinterpret_cast<ControlWord_t*>(&controlWord));
}

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
void InitializeFRCDriverStation() {
        std::memset(&newestControlWord, 0, sizeof(newestControlWord));
        static FRCDriverStation ds;
        driverStation = &ds;
}
extern "C" {

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

int32_t HAL_GetJoystickDescriptor(int32_t joystickNum, HAL_JoystickDescriptor *desc) {
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

// Constant number to be used for our occur handle
constexpr int32_t refNumber = 42;
static void udpOccur(void) {
    cacheToUpdate->Update();

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

static void newDataOccur(uint32_t refNum) {
    switch (refNum) {
        case refNumber:
            udpOccur();
            break;
        default:
            std::printf("Unknown occur %u\n", refNum);
            break;
    }
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
        newestControlWord = currentRead->controlWord;
    }
    return prev != nullptr;
}


//void InitializeDriverStation() {
//    // Set up the occur function internally with NetComm
//    // NetCommRPCProxy_SetOccurFuncPointer(newDataOccur);
//    // Set up our occur reference number
////        setNewDataOccurRef(refNumber);
//}
}