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

    // called from SRV_Channels
    void update();

    // send ESC telemetry messages over MAVLink
    void send_mavlink(uint8_t mav_chan);

private:

    enum TransmissionGear {
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

    struct data_info_t {
        uint32_t    rpm;            // rpm
        uint8_t     gear;           // gear

    } _data;
};
