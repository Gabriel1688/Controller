#include "CAN.h"
#include <iostream>

CAN::CAN(int deviceId) {
    int32_t status = 0;
    m_handle = HAL_InitializeCAN(kTeamManufacturer, deviceId, kTeamDeviceType, &status);
}

CAN::CAN(int deviceId, int deviceManufacturer, int deviceType) {
    int32_t status = 0;
    m_handle = HAL_InitializeCAN(
        static_cast<HAL_CANManufacturer>(deviceManufacturer), deviceId,
        static_cast<HAL_CANDeviceType>(deviceType), &status);
}

void CAN::WritePacket(const uint8_t *data, int length, int apiId) {
    int32_t status = 0;
    HAL_WriteCANPacket(m_handle, data, length, apiId, &status);
    std::cout << "CAN::WritePacket Received response." << std::endl;
}

void CAN::WritePacketRepeating(const uint8_t *data, int length, int apiId, int repeatMs) {
    int32_t status = 0;
    HAL_WriteCANPacketRepeating(m_handle, data, length, apiId, repeatMs, &status);
}

void CAN::StopPacketRepeating(int apiId) {
    int32_t status = 0;
    HAL_StopCANPacketRepeating(m_handle, apiId, &status);
}

bool CAN::ReadPacketNew(int apiId, CANData *data) {
    int32_t status = 0;
    HAL_ReadCANPacketNew(m_handle, apiId, data->data, &data->length, &data->timestamp, &status);
    if (status != 0) {
        return false;
    }
    return true;
}
bool CAN::ReadPacketLatest(int apiId, CANData *data) {
    int32_t status = 0;
    HAL_ReadCANPacketLatest(m_handle, apiId, data->data, &data->length, &data->timestamp, &status);
    if (status != 0) {
        return false;
    } else {
        return true;
    }
}

bool CAN::ReadPacketTimeout(int apiId, int timeoutMs, CANData *data) {
    int32_t status = 0;
    HAL_ReadCANPacketTimeout(m_handle, apiId, data->data, &data->length, &data->timestamp, timeoutMs, &status);
    //    if (status == HAL_CAN_TIMEOUT ||
    //        status == HAL_ERR_CANSessionMux_MessageNotFound) {
    //        return false;
    //    }
    if (status != 0) {
        return false;
    } else {
        return true;
    }
}