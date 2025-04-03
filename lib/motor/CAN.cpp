#include "CAN.h"
#include "CANAPI.h"

CAN::CAN(int deviceId) {
    int32_t status = 0;
    m_handle = HAL_InitializeCAN(kTeamManufacturer, deviceId, kTeamDeviceType, &status);
}

void CAN::WritePacket(const uint8_t* data, int length, int apiId) {
    int32_t status = 0;
    HAL_WriteCANPacket(m_handle, data, length, apiId, &status);
}

void CAN::WritePacketRepeating(const uint8_t* data, int length, int apiId, int repeatMs) {
    int32_t status = 0;
    HAL_WriteCANPacketRepeating(m_handle, data, length, apiId, repeatMs, &status);
}


void CAN::StopPacketRepeating(int apiId) {
    int32_t status = 0;
    HAL_StopCANPacketRepeating(m_handle, apiId, &status);
}

bool CAN::ReadPacketNew(int apiId, CANData* data) {
    int32_t status = 0;
    HAL_ReadCANPacketNew(m_handle, apiId, data->data, &data->length, &data->timestamp, &status);
    if (status != 0) {
        return false;
    }
    return true;
}
