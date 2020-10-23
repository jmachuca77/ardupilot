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

#include <AP_HAL/AP_HAL.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_PolarisCAN.h"
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "PolarisCAN",  fmt, #args); } while (0)


// stupid compiler is not able to optimise this under gnu++11
// move this back when moving to gnu++17
const uint16_t AP_PolarisCAN::PolarisCAN_SEND_TIMEOUT_US = 500;

AP_PolarisCAN::AP_PolarisCAN()
{
    debug_can(AP_CANManager::LOG_INFO, "PolarisCAN: constructed\n\r");
}

AP_PolarisCAN *AP_PolarisCAN::get_PolarisCAN(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_PolarisCAN) {
        return nullptr;
    }
    return static_cast<AP_PolarisCAN*>(AP::can().get_driver(driver_index));
}


bool AP_PolarisCAN::add_interface(AP_HAL::CANIface* can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "PolarisCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "PolarisCAN: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "PolarisCAN: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "PolarisCAN: Cannot add event handle\n\r");
        return false;
    }
    return true;
}


// initialise PolarisCAN bus
void AP_PolarisCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "PolarisCAN: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "PolarisCAN: already initialized\n\r");
        return;
    }

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "PolarisCAN: Interface not found\n\r");
        return;
    }

    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_PolarisCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "PolarisCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "PolarisCAN: init done\n\r");
    return;
}

// loop to send output to ESCs in background thread
void AP_PolarisCAN::loop()
{
    uint64_t timeout = 0;
    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), PolarisCAN_SEND_TIMEOUT_US);
    static uint32_t sendtext_timeref;
    static uint32_t sendtext_timeref1;

    while (true) {
        //gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"PolarisCAN Loop");

        if (!_initialized) {
            // if not initialised wait 2ms
            debug_can(AP_CANManager::LOG_DEBUG, "PolarisCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(2000);
            continue;
        }

        // debug_can(AP_CANManager::LOG_DEBUG, "PolarisCAN: Loop\n\r");

        // check for replies from ESCs
        AP_HAL::CANFrame recv_frame;

        timeout = AP_HAL::native_micros64() + timeout_us;

        while (read_frame(recv_frame, timeout)) {
            uint32_t frameID = recv_frame.id & recv_frame.MaskExtID;
            uint32_t rpm = 0;
            //debug_can(AP_CANManager::LOG_DEBUG, "Recived Frame ID: %d",recv_frame.id);
            //gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"RZRCAN ID: 0x%X", frameID);
            debug_can(AP_CANManager::LOG_DEBUG, "PolarisCAN: Got ID: 0x%X\n\r", frameID);

            // Get data Semaphore
            WITH_SEMAPHORE(_data_sem);

            // decode Gear Data
            if (frameID == 0x18F00500) {
                debug_can(AP_CANManager::LOG_DEBUG, "PolarisCAN: Gear: 0x%X\n\r", recv_frame.data[5]);
                _data.gear = recv_frame.data[5];

                switch (recv_frame.data[5]) {
                    case TransmissionGear::PARK:
                        gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"RZRCAN GEAR: PARK");
                    break;

                    case TransmissionGear::REVERSE:
                        gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"RZRCAN GEAR: Reverse");
                    break;

                    case TransmissionGear::NEUTRAL:
                        gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"RZRCAN GEAR: Neutral");
                    break;

                    case TransmissionGear::LOW_GEAR:
                        gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"RZRCAN GEAR: Low Gear");
                    break;

                    case TransmissionGear::HIGH_GEAR:
                        gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"RZRCAN GEAR: High Gear");
                    break;

                    default:
                        gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"RZRCAN GEAR: Error");
                    break;
                }
            }

            // decode RPM Data
            if (frameID == 0xCF00400) {
                rpm = (recv_frame.data[4] << 8) | (recv_frame.data[3]);
                _data.rpm = rpm * 0.125;
                gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref1,"RZRCAN RPM: %0.2f [0x%X]",rpm*0.125,rpm);
            }
        }
    }
}

// write frame on CAN bus
bool AP_PolarisCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout)
{
    // wait for space in buffer to send command

    bool read_select = false;
    bool write_select = true;
    bool ret;
    do {
        ret = _can_iface->select(read_select, write_select, &out_frame, timeout);
        if (!ret || !write_select) {
            // delay if no space is available to send
            hal.scheduler->delay_microseconds(50);
        }
    } while (!ret || !write_select);

    // send frame and return success
    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on success
bool AP_PolarisCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout)
{
    // wait for space in buffer to read
    bool read_select = true;
    bool write_select = false;
    int ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret || !read_select) {
        // return false if no data is available to read
        return false;
    }
    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    // read frame and return success
    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// called from SRV_Channels
void AP_PolarisCAN::update_rpm()
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger && logger->logging_enabled()) {
        WITH_SEMAPHORE(_data_sem);
    }
}

uint32_t AP_PolarisCAN::get_current_rpm() {
    WITH_SEMAPHORE(_data_sem);
    return _data.rpm;
}

// send ESC telemetry messages over MAVLink
void AP_PolarisCAN::send_mavlink(uint8_t mav_chan)
{
    // output telemetry messages
    {
        // take semaphore to access telemetry data
        WITH_SEMAPHORE(_data_sem);

    }
}

#endif // HAL_NUM_CAN_IFACES
