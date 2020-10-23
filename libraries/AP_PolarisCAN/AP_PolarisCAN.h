/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/Semaphores.h>

#define PolarisCAN_MAX_NUM_ESCS 12

class CANTester;

class AP_PolarisCAN : public AP_CANDriver {
    friend class CANTester;
public:
    AP_PolarisCAN();
    ~AP_PolarisCAN();

    /* Do not allow copies */
    AP_PolarisCAN(const AP_PolarisCAN &other) = delete;
    AP_PolarisCAN &operator=(const AP_PolarisCAN&) = delete;

    // Return PolarisCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_PolarisCAN *get_PolarisCAN(uint8_t driver_index);

    // initialise PolarisCAN bus
    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // Currently not Used
    void update();

    // Called from AP_RPM when using PolarisCAN type
    uint32_t get_current_rpm();

    // Called from AP_ICEEngine
    MAV_ICE_TRANSMISSION_GEAR_STATE get_current_gear();
    float get_coolant_temp();
    float get_fuel_level();

    // send ESC telemetry messages over MAVLink
    void send_mavlink(uint8_t mav_chan);

private:

    enum TransmissionGear {
        UNKNOWN     = 0x00,
        PARK        = 0x50,
        REVERSE     = 0x52,
        NEUTRAL     = 0x4E,
        LOW_GEAR    = 0x4C,
        HIGH_GEAR   = 0x48,
    };

    static const uint16_t PolarisCAN_SEND_TIMEOUT_US;

    // telemetry definitions
    static constexpr uint32_t EX_CAN_ESC_UPDATE_MS = 100;

    // loop to send output to ESCs in background thread
    void loop();

    // write frame on CAN bus, returns true on success
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);

    // read frame on CAN bus, returns true on success
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);

    bool _initialized;
    char _thread_name[9];
    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_EventHandle _event_handle;

    // telemetry data (rpm, voltage)
    HAL_Semaphore _data_sem;

    const uint32_t u32EEC1_ID = 0x0CF00400;
    const uint16_t u16EEC1_PERIOD_MS = 20;
    struct EEC1_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint8_t  u8ActEngPrcntTrqHiRes;
        uint8_t  u8EngTrqMode;
        uint8_t  u8DrvDmdEngPrcntTrq;
        uint8_t  u8ActEngPrcntTrq;
        uint16_t u16EngSpd;
        uint8_t  u8SrcAddCtrlDvcForEngCtrl;
        uint8_t  u8EngStartMode;
    } _EEC1_data;

    const uint32_t u32ENGTEMP_ID = 0x18FEEE00;
    const uint16_t u16ENGTEMP_PERIOD_MS = 1180;
    struct ENGTEMP_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint8_t  u8EngCoolantTemp;
        uint8_t  u8EngFuelTemp;
        uint16_t u16EngOilTemp;
        uint16_t u16EngTrboChrOilTemp;
        uint8_t  u8EngInterClrTemp;
        uint8_t  u8EngInterClrThrmStateOpen;
    } _ENGTEMP_data;

    const uint32_t u32DD_ID = 0x18FEFC17;
    const uint16_t u16DD_PERIOD_MS = 1180;
    struct DD_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint8_t  u8WshrFluidLvl;
        uint8_t  u8FuelLvl;
        uint8_t  u8FuelFltrDiffPress;
        uint8_t  u8EngOilFltrDiffPress;
        uint16_t u16CrgoAmbTemp;        
    } _DD_data;

    const uint32_t u32TRANS1_ID = 0x18F00500;
    const uint16_t u16TRANS1_PERIOD_MS = 110;
    struct TRANS1_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint8_t  u8gear;
    } _TRANS1_data;
};
