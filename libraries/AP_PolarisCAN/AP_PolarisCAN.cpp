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
#define AP_POLARISCAN_FUEL_LEVEL_INVALID      -1

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
// TODO:    Get Fuel Level
//          Get Engine Temp
void AP_PolarisCAN::loop()
{
    uint64_t timeout = 0;
    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), PolarisCAN_SEND_TIMEOUT_US);
    // static uint32_t sendtext_timeref, sendtext_timeref1, sendtext_timeref2, sendtext_timeref3;

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
            uint32_t now_ms = AP_HAL::millis();
            //debug_can(AP_CANManager::LOG_DEBUG, "Recived Frame ID: %d",recv_frame.id);
            //gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"RZRCAN ID: 0x%X", frameID);
            debug_can(AP_CANManager::LOG_DEBUG, "PolarisCAN: Got ID: 0x%X\n\r", frameID);

            // Get data Semaphore
            WITH_SEMAPHORE(_data_sem);

            // decode TRANS1 Data
            if (frameID == AP_PolarisCAN::u32TRANS1_ID) {
                debug_can(AP_CANManager::LOG_DEBUG, "PolarisCAN: Gear: 0x%X\n\r", recv_frame.data[5]);
                _TRANS1_data.u32lastRecvFrameTime = now_ms;
                _TRANS1_data.boMessageTimeout = false;
                _TRANS1_data.u8gear = recv_frame.data[5];
            }

            // decode EEC1 Data
            if (frameID == AP_PolarisCAN::u32EEC1_ID) {
                _EEC1_data.u32lastRecvFrameTime      = now_ms;
                _EEC1_data.boMessageTimeout          = false;
                _EEC1_data.u8ActEngPrcntTrqHiRes     = recv_frame.data[0] >> 4;
                _EEC1_data.u8EngTrqMode              = recv_frame.data[0] & 0x0F;
                _EEC1_data.u8DrvDmdEngPrcntTrq       = recv_frame.data[1];
                _EEC1_data.u8ActEngPrcntTrq          = recv_frame.data[2];
                _EEC1_data.u16EngSpd                 = (recv_frame.data[4] << 8) | (recv_frame.data[3]);
                _EEC1_data.u8SrcAddCtrlDvcForEngCtrl = recv_frame.data[5];
                _EEC1_data.u8EngStartMode            = recv_frame.data[6];

                // gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref1,"RZRCAN RPM: %0.2f [0x%X]",_EEC1_data.u16EngSpd * 0.125,_EEC1_data.u16EngSpd);
            }

            // decode DD Data
            if (frameID == AP_PolarisCAN::u32DD_ID) {
                _DD_data.u32lastRecvFrameTime        = now_ms;
                _DD_data.boMessageTimeout            = false;
                _DD_data.u8WshrFluidLvl              = recv_frame.data[0];
                _DD_data.u8FuelLvl                   = recv_frame.data[1];
                _DD_data.u8FuelFltrDiffPress         = recv_frame.data[2];
                _DD_data.u8EngOilFltrDiffPress       = recv_frame.data[3];
                _DD_data.u16CrgoAmbTemp              = (recv_frame.data[5] << 8) | (recv_frame.data[4]);

                // gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref2,"RZRCAN Fuel Level: %0.2f% [0x%X]",_DD_data.u8FuelLvl*0.4, recv_frame.data[1]);
            }

            // decode ENGTEMP
            if (frameID == AP_PolarisCAN::u32ENGTEMP_ID) {
                _ENGTEMP_data.u32lastRecvFrameTime        = now_ms;
                _ENGTEMP_data.boMessageTimeout            = false;
                _ENGTEMP_data.u8EngCoolantTemp            = recv_frame.data[0];
                _ENGTEMP_data.u8EngFuelTemp               = recv_frame.data[1];
                _ENGTEMP_data.u16EngOilTemp               = (recv_frame.data[3] << 8) | (recv_frame.data[2]);
                _ENGTEMP_data.u16EngTrboChrOilTemp        = (recv_frame.data[5] << 8) | (recv_frame.data[4]);
                _ENGTEMP_data.u8EngInterClrTemp           = recv_frame.data[6];
                _ENGTEMP_data.u8EngInterClrThrmStateOpen  = recv_frame.data[7];

                //gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref3,"RZRCAN Coolant Temp: %d [0x%X]",(_ENGTEMP_data.u8EngCoolantTemp-40),recv_frame.data[0]);
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
void AP_PolarisCAN::update()
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger && logger->logging_enabled()) {
        WITH_SEMAPHORE(_data_sem);
    }
}

uint32_t AP_PolarisCAN::get_current_rpm() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _EEC1_data.u32lastRecvFrameTime > 2*AP_PolarisCAN::u16EEC1_PERIOD_MS) && (!_EEC1_data.boMessageTimeout)){
        _EEC1_data.u16EngSpd = 0;
        _EEC1_data.boMessageTimeout = true;
    }
    return _EEC1_data.u16EngSpd * 0.125;
}

float AP_PolarisCAN::get_coolant_temp() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _ENGTEMP_data.u32lastRecvFrameTime > 2*AP_PolarisCAN::u16ENGTEMP_PERIOD_MS) && (!_ENGTEMP_data.boMessageTimeout)){
        _ENGTEMP_data.u8EngCoolantTemp = 0xFF;
        _ENGTEMP_data.boMessageTimeout = true;
    }
    return (float)_ENGTEMP_data.u8EngCoolantTemp-40;
}

float AP_PolarisCAN::get_fuel_level() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _DD_data.u32lastRecvFrameTime > 2*AP_PolarisCAN::u16DD_PERIOD_MS) && (!_DD_data.boMessageTimeout)){
        _DD_data.u8FuelLvl = 0xFF;
        _DD_data.boMessageTimeout = true;
    }
    if (_DD_data.u8FuelLvl == 0xFF) {
        return AP_POLARISCAN_FUEL_LEVEL_INVALID;
    } else {
        return (float)_DD_data.u8FuelLvl*0.4;
    }
    
}

MAV_ICE_TRANSMISSION_GEAR_STATE AP_PolarisCAN::get_current_gear() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _TRANS1_data.u32lastRecvFrameTime > 2*AP_PolarisCAN::u16TRANS1_PERIOD_MS) && (!_TRANS1_data.boMessageTimeout)){
        _TRANS1_data.u8gear = 0x00;
        _TRANS1_data.boMessageTimeout = true;
    }
    MAV_ICE_TRANSMISSION_GEAR_STATE gear_state;

    switch (_TRANS1_data.u8gear) {
        case TransmissionGear::PARK:
            gear_state = MAV_ICE_TRANSMISSION_GEAR_STATE_PARK;
        break;

        case TransmissionGear::REVERSE:
            gear_state = MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE;
        break;

        case TransmissionGear::NEUTRAL:
            gear_state = MAV_ICE_TRANSMISSION_GEAR_STATE_NEUTRAL;
        break;

        case TransmissionGear::LOW_GEAR:
            gear_state = MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1;
        break;

        case TransmissionGear::HIGH_GEAR:
            gear_state = MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_2;
        break;

        default:
            gear_state = MAV_ICE_TRANSMISSION_GEAR_STATE_UNKNOWN;
        break;
    }

    return gear_state;
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
