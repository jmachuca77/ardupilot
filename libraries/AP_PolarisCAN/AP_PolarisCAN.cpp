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
#define AP_POLARISCAN_ODOMETER_INVALID        -1

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

        static uint32_t lastSentTime;
        if ((AP_HAL::millis() - lastSentTime) > nENG_HOUR_REQ_INTERVAL) {
            lastSentTime = AP_HAL::millis();
            // gcs().send_text(MAV_SEVERITY_INFO, "RZRCAN: Requesting Engine Hours");
            req_cmd_t req_eng_hours_cmd;
            req_eng_hours_cmd.data1 = 0xE5;
            req_eng_hours_cmd.data2 = 0xFE;
            req_eng_hours_cmd.data3 = 0x00;
            req_eng_hours_frame = {(0x18EA009E | AP_HAL::CANFrame::FlagEFF), req_eng_hours_cmd.data, sizeof(req_eng_hours_cmd.data)};
            if (!write_frame(req_eng_hours_frame, timeout)) {
                continue;
            }
        }

        static uint32_t lastDTCReqTime;
        if ((AP_HAL::millis() - lastDTCReqTime) > nSTORED_DTC_REQ_INTERVAL) {
            lastDTCReqTime = AP_HAL::millis();
            // gcs().send_text(MAV_SEVERITY_INFO, "RZRCAN: Requesting Engine Hours");
            req_cmd_t req_stored_dtc_cmd;
            req_stored_dtc_cmd.data1 = 0xCB;
            req_stored_dtc_cmd.data2 = 0xFE;
            req_stored_dtc_cmd.data3 = 0x00;
            req_stored_dtcs_frame = {(0x18EAFF9E | AP_HAL::CANFrame::FlagEFF), req_stored_dtc_cmd.data, sizeof(req_stored_dtc_cmd.data)};
            if (!write_frame(req_stored_dtcs_frame, timeout)) {
                continue;
            }
        }

        while (read_frame(recv_frame, timeout)) {
            uint32_t frameID = recv_frame.id & recv_frame.MaskExtID;
            uint32_t now_ms = AP_HAL::millis();
            //debug_can(AP_CANManager::LOG_DEBUG, "Recived Frame ID: %d",recv_frame.id);
            //gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"RZRCAN ID: 0x%X", frameID);
            //debug_can(AP_CANManager::LOG_DEBUG, "PolarisCAN: Got ID: 0x%X\n\r", frameID);

            // Get data Semaphore
            WITH_SEMAPHORE(_data_sem);
            //static uint32_t sendtext_FWDRef;

            switch (frameID) {
                case nTRANS1_ID:
                    _TRANS1_data.u32lastRecvFrameTime = now_ms;
                    _TRANS1_data.boMessageTimeout = false;
                    _TRANS1_data.u8gear = recv_frame.data[5];
                break;

                case nEEC1_ID:
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
                break;

                case nDD_ID:
                    _DD_data.u32lastRecvFrameTime        = now_ms;
                    _DD_data.boMessageTimeout            = false;
                    _DD_data.u8WshrFluidLvl              = recv_frame.data[0];
                    _DD_data.u8FuelLvl                   = recv_frame.data[1];
                    _DD_data.u8FuelFltrDiffPress         = recv_frame.data[2];
                    _DD_data.u8EngOilFltrDiffPress       = recv_frame.data[3];
                    _DD_data.u16CrgoAmbTemp              = (recv_frame.data[5] << 8) | (recv_frame.data[4]);

                    // gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref2,"RZRCAN Fuel Level: %0.2f% [0x%X]",_DD_data.u8FuelLvl*0.4, recv_frame.data[1]);                
                break;

                case nENGTEMP_ID:
                    _ENGTEMP_data.u32lastRecvFrameTime        = now_ms;
                    _ENGTEMP_data.boMessageTimeout            = false;
                    _ENGTEMP_data.u8EngCoolantTemp            = recv_frame.data[0];
                    _ENGTEMP_data.u8EngFuelTemp               = recv_frame.data[1];
                    _ENGTEMP_data.u16EngOilTemp               = (recv_frame.data[3] << 8) | (recv_frame.data[2]);
                    _ENGTEMP_data.u16EngTrboChrOilTemp        = (recv_frame.data[5] << 8) | (recv_frame.data[4]);
                    _ENGTEMP_data.u8EngInterClrTemp           = recv_frame.data[6];
                    _ENGTEMP_data.u8EngInterClrThrmStateOpen  = recv_frame.data[7];

                    //gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref3,"RZRCAN Coolant Temp: %d [0x%X]",(_ENGTEMP_data.u8EngCoolantTemp-40),recv_frame.data[0]);
                break;

                case nFWD_ID:
                    _FWD_data.u32lastRecvFrameTime        = now_ms;
                    _FWD_data.boMessageTimeout            = false;
                    _FWD_data.u8FWDActStatus              = recv_frame.data[0] & 0x03;

                    // gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_FWDRef,"RZRCAN FWD: %d [0x%X]",(_FWD_data.u8FWDActStatus),recv_frame.data[0]);
                break;

                case nADC_ID:
                    _ADC_data.u32lastRecvFrameTime        = now_ms;
                    _ADC_data.boMessageTimeout            = false;
                    _ADC_data.u8ADCActStatus              = recv_frame.data[0] & 0x03;
                    _ADC_data.u84WDSwStatus               = recv_frame.data[4] & 0x0F;
                    // gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_FWDRef,"RZRCAN ADC: %d [0x%X], 4WDSw: %d [0x%X]",(_ADC_data.u8ADCActStatus),recv_frame.data[0],_ADC_data.u84WDSwStatus,recv_frame.data[4]);
                break;

                case nENGHRS_ID: {
                    _ENGHRS_data.u32lastRecvFrameTime     = now_ms;
                    _ENGHRS_data.boMessageTimeout         = false;
                    const uint32_t u32Hours               = (recv_frame.data[3] << 24) + (recv_frame.data[2] << 16) + (recv_frame.data[1] << 8) + (recv_frame.data[0]);
                    const uint32_t u32Revs                = (recv_frame.data[7] << 24) + (recv_frame.data[6] << 16) + (recv_frame.data[5] << 8) + (recv_frame.data[4]);
                    _ENGHRS_data.fTotalEngineHours        = u32Hours * 0.05;
                    _ENGHRS_data.fTotalEngineRevolutions  = u32Revs * 1000;
                    // gcs().send_text(MAV_SEVERITY_INFO,"RZRCAN EngHours: %f [0x%X], Revs: %f [0x%X]",_ENGHRS_data.u32TotalEngineHours,u32Hours,_ENGHRS_data.u32TotalEngineRevolutions,u32Revs);
                    // gcs().send_text(MAV_SEVERITY_INFO, "RZRCAN EngHours: %0.2f [0x%lX]",_ENGHRS_data.fTotalEngineHours,u32Hours);
                    }
                break;

                case nSTRDTC_ID:
                    _STRDTC_data.u32lastRecvFrameTime     = now_ms;
                    _STRDTC_data.boMessageTimeout         = false;
                    _STRDTC_data.u32SPN                   = (((recv_frame.data[4] & 0xE0) >> 5) << 16) + (recv_frame.data[3] << 8) + (recv_frame.data[2]);
                    _STRDTC_data.u8FMI                    = recv_frame.data[4] & 0x1F;
                    _STRDTC_data.u8OC                     = recv_frame.data[5] & 0x7F;
                    _STRDTC_data.u16WarningLightStatus    = (recv_frame.data[1] << 8) + (recv_frame.data[0]);
                    // gcs().send_text(MAV_SEVERITY_INFO, "RZRCAN STRDTC FMI: %d [0x%X] SPN: %ld [0x%lX] OC: %d [0x%X]", _STRDTC_data.u8FMI, _STRDTC_data.u8FMI, _STRDTC_data.u32SPN, _STRDTC_data.u32SPN, _STRDTC_data.u8OC, _STRDTC_data.u8OC);
                break;    

                case nDDDTC_ID:
                    _DDDTC_data.u32lastRecvFrameTime     = now_ms;
                    _DDDTC_data.boMessageTimeout         = false;
                    _DDDTC_data.u32SPN                   = (((recv_frame.data[4] & 0xE0) >> 5) << 16) + (recv_frame.data[3] << 8) + (recv_frame.data[2]);
                    _DDDTC_data.u8FMI                    = recv_frame.data[4] & 0x1F;
                    _DDDTC_data.u8OC                     = recv_frame.data[5] & 0x7F;
                    _DDDTC_data.u16WarningLightStatus    = (recv_frame.data[1] << 8) + (recv_frame.data[0]);
                    // gcs().send_text(MAV_SEVERITY_INFO, "RZRCAN DDDTC FMI: %d [0x%X] SPN: %ld [0x%lX] OC: %d [0x%X]", _DDDTC_data.u8FMI, _DDDTC_data.u8FMI, _DDDTC_data.u32SPN, _DDDTC_data.u32SPN, _DDDTC_data.u8OC, _DDDTC_data.u8OC);
                break;    

                case nENGDTC_ID:
                    _ENGDTC_data.u32lastRecvFrameTime    = now_ms;
                    _ENGDTC_data.boMessageTimeout        = false;
                    _ENGDTC_data.u32SPN                  = (((recv_frame.data[4] & 0xE0) >> 5) << 16) + (recv_frame.data[3] << 8) + (recv_frame.data[2]);
                    _ENGDTC_data.u8FMI                   = recv_frame.data[4] & 0x1F;
                    _ENGDTC_data.u8OC                    = recv_frame.data[5] & 0x7F;
                    _ENGDTC_data.u16WarningLightStatus   = (recv_frame.data[1] << 8) + (recv_frame.data[0]);
                    // gcs().send_text(MAV_SEVERITY_INFO, "RZRCAN ENGDTC FMI: %d [0x%X] SPN: %ld [0x%lX] OC: %d [0x%X]", _ENGDTC_data.u8FMI, _ENGDTC_data.u8FMI, _ENGDTC_data.u32SPN, _ENGDTC_data.u32SPN, _ENGDTC_data.u8OC, _ENGDTC_data.u8OC);
                break;    

                case nVDHR_ID:
                    uint32_t rawOdom = (recv_frame.data[3] << 24) + (recv_frame.data[2] << 16) + (recv_frame.data[1] << 8) + (recv_frame.data[0]);
                    uint32_t rawTrip = (recv_frame.data[7] << 24) + (recv_frame.data[6] << 16) + (recv_frame.data[5] << 8) + (recv_frame.data[4]);
                    _VDHR_data.u32lastRecvFrameTime      = now_ms;
                    _VDHR_data.boMessageTimeout          = false;
                    _VDHR_data.fOdometer                 = rawOdom * 0.005;
                    _VDHR_data.fTripDistance             = rawTrip * 0.005;
                    // gcs().send_text(MAV_SEVERITY_INFO, "RZRCAN Odom: %0.2f [0x%lX] Trip: %0.2f [0x%lX]", _VDHR_data.fOdometer, rawOdom, _VDHR_data.fTripDistance, rawTrip);
                break;   

                case nDDSTDTC_ID:
                case nSTRSTDTC_ID:
                case nENGSTDTC_ID:
                    tDTC recievedDTC;
                    recievedDTC.u32SPN                  = (((recv_frame.data[4] & 0xE0) >> 5) << 16) + (recv_frame.data[3] << 8) + (recv_frame.data[2]);
                    recievedDTC.u8FMI                   = recv_frame.data[4] & 0x1F;
                    recievedDTC.u8OC                    = recv_frame.data[5] & 0x7F;
                    recievedDTC.u16LightStatus          = (recv_frame.data[1] << 8) + (recv_frame.data[0]);

                    
                    static uint8_t lastDTCIndex = 0;
                    uint8_t DTCIndex = lastDTCIndex;
                    for (uint8_t dtc=0; dtc<sizeof(STORED_DTC_ARRAY); dtc++) {
                        if (recievedDTC.u32SPN == STORED_DTC_ARRAY[dtc].u32SPN) {
                            DTCIndex = dtc;
                        }   
                    }

                    STORED_DTC_ARRAY[DTCIndex].index            = 0;
                    STORED_DTC_ARRAY[DTCIndex].u16LightStatus   = recievedDTC.u16LightStatus;
                    STORED_DTC_ARRAY[DTCIndex].u32SPN           = recievedDTC.u32SPN;
                    STORED_DTC_ARRAY[DTCIndex].u8FMI            = recievedDTC.u8FMI;
                    STORED_DTC_ARRAY[DTCIndex].u8OC             = recievedDTC.u8OC;
                    if (DTCIndex == lastDTCIndex) {
                        lastDTCIndex++;
                        if (lastDTCIndex >= sizeof(STORED_DTC_ARRAY)) {
                            lastDTCIndex = 0;
                        };
                    };
                break;             
            };
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

void AP_PolarisCAN::sendClearDTCs() {
    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), PolarisCAN_SEND_TIMEOUT_US);
    uint64_t timeout = AP_HAL::native_micros64() + timeout_us;
    req_cmd_t clear_dtcs_cmd;
    clear_dtcs_cmd.data1 = 0xD3;
    clear_dtcs_cmd.data2 = 0xFE;
    clear_dtcs_cmd.data3 = 0x00;
    clear_dtcs_frame = {(0x18EAFF9E | AP_HAL::CANFrame::FlagEFF), clear_dtcs_cmd.data, sizeof(clear_dtcs_cmd.data)};
    write_frame(clear_dtcs_frame, timeout);
}

void AP_PolarisCAN::sendClearStoredDTCs() {
    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), PolarisCAN_SEND_TIMEOUT_US);
    uint64_t timeout = AP_HAL::native_micros64() + timeout_us;
    req_cmd_t clear_dtcs_cmd;
    clear_dtcs_cmd.data1 = 0xCC;
    clear_dtcs_cmd.data2 = 0xFE;
    clear_dtcs_cmd.data3 = 0x00;
    clear_dtcs_frame = {(0x18EAFF9E | AP_HAL::CANFrame::FlagEFF), clear_dtcs_cmd.data, sizeof(clear_dtcs_cmd.data)};
    write_frame(clear_dtcs_frame, timeout);
    // Clear internal Array
    for (uint8_t dtc=0; dtc<sizeof(STORED_DTC_ARRAY); dtc++) {
        STORED_DTC_ARRAY[dtc].index            = 0;
        STORED_DTC_ARRAY[dtc].u16LightStatus   = 0;
        STORED_DTC_ARRAY[dtc].u32SPN           = 0;
        STORED_DTC_ARRAY[dtc].u8FMI            = 0;
        STORED_DTC_ARRAY[dtc].u8OC             = 0;
    }
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
    if ((now_ms - _EEC1_data.u32lastRecvFrameTime > 2*nEEC1_PERIOD_MS) && (!_EEC1_data.boMessageTimeout)){
        _EEC1_data.u16EngSpd = 0;
        _EEC1_data.boMessageTimeout = true;
    }
    return _EEC1_data.u16EngSpd * 0.125;
}

float AP_PolarisCAN::get_coolant_temp() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _ENGTEMP_data.u32lastRecvFrameTime > 2*nENGTEMP_PERIOD_MS) && (!_ENGTEMP_data.boMessageTimeout)){
        _ENGTEMP_data.u8EngCoolantTemp = 0xFF;
        _ENGTEMP_data.boMessageTimeout = true;
    }
    return (float)_ENGTEMP_data.u8EngCoolantTemp-40;
}

float AP_PolarisCAN::get_fuel_level() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _DD_data.u32lastRecvFrameTime > 2*nDD_PERIOD_MS) && (!_DD_data.boMessageTimeout)){
        _DD_data.u8FuelLvl = 0xFF;
        _DD_data.boMessageTimeout = true;
    }
    if (_DD_data.u8FuelLvl == 0xFF) {
        return AP_POLARISCAN_FUEL_LEVEL_INVALID;
    } else {
        return (float)_DD_data.u8FuelLvl*0.4;
    }
    
}

float AP_PolarisCAN::get_odometer() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _VDHR_data.u32lastRecvFrameTime > 2*nVDHR_PERIOD_MS) && (!_VDHR_data.boMessageTimeout)){
        _VDHR_data.fOdometer = AP_POLARISCAN_ODOMETER_INVALID;
        _VDHR_data.fTripDistance = AP_POLARISCAN_ODOMETER_INVALID;
        _VDHR_data.boMessageTimeout = true;
    }
    
    return _VDHR_data.fOdometer;
}

float AP_PolarisCAN::get_tripDistance() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _VDHR_data.u32lastRecvFrameTime > 2*nVDHR_PERIOD_MS) && (!_VDHR_data.boMessageTimeout)){
        _VDHR_data.fOdometer = AP_POLARISCAN_ODOMETER_INVALID;
        _VDHR_data.fTripDistance = AP_POLARISCAN_ODOMETER_INVALID;
        _VDHR_data.boMessageTimeout = true;
    }
    
    return _VDHR_data.fTripDistance;
}

AP_PolarisCAN::AP_POLARISCAN_AWD_STATUS AP_PolarisCAN::get_FWD_status() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _FWD_data.u32lastRecvFrameTime > 2*nFWD_PERIOD_MS) && (!_FWD_data.boMessageTimeout)){
        _FWD_data.u8FWDActStatus = NOTAVAILABLE;
        _FWD_data.boMessageTimeout = true;
    }
    return (AP_POLARISCAN_AWD_STATUS)_FWD_data.u8FWDActStatus ;
};

AP_PolarisCAN::AP_POLARISCAN_AWD_STATUS AP_PolarisCAN::get_ADC_status() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _ADC_data.u32lastRecvFrameTime > 2*nADC_PERIOD_MS) && (!_ADC_data.boMessageTimeout)){
        _ADC_data.u8ADCActStatus = NOTAVAILABLE;
        _ADC_data.boMessageTimeout = true;
    }
    return (AP_POLARISCAN_AWD_STATUS)_ADC_data.u8ADCActStatus ;
};

float AP_PolarisCAN::get_engine_hours() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _ENGHRS_data.u32lastRecvFrameTime > 2*nENGTEMP_PERIOD_MS) && (!_ENGHRS_data.boMessageTimeout)){
        _ENGHRS_data.fTotalEngineHours = -1;
        _ENGHRS_data.boMessageTimeout = true;
    }
    return (float)_ENGHRS_data.fTotalEngineHours;
};

AP_PolarisCAN::AP_POLARISCAN_DTC AP_PolarisCAN::get_engine_dtc() {
    uint32_t now_ms = AP_HAL::millis();
    AP_PolarisCAN::AP_POLARISCAN_DTC stDTC;
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _ENGDTC_data.u32lastRecvFrameTime > 2*nENGDTC_PERIOD_MS) && (!_ENGDTC_data.boMessageTimeout)){
        _ENGDTC_data.boMessageTimeout = true;
    }
    stDTC.index             = 0;
    stDTC.u32SPN            = _ENGDTC_data.u32SPN;
    stDTC.u8FMI             = _ENGDTC_data.u8FMI;
    stDTC.u8OC              = _ENGDTC_data.u8OC;
    stDTC.u16LightStatus    = _ENGDTC_data.u16WarningLightStatus;
    return stDTC;
};

AP_PolarisCAN::AP_POLARISCAN_DTC AP_PolarisCAN::get_cluster_dtc(){
    uint32_t now_ms = AP_HAL::millis();
    AP_PolarisCAN::AP_POLARISCAN_DTC stDTC;
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _DDDTC_data.u32lastRecvFrameTime > 2*nDDDTC_PERIOD_MS) && (!_DDDTC_data.boMessageTimeout)){
        _DDDTC_data.boMessageTimeout = true;
    }
    stDTC.index             = 0;
    stDTC.u32SPN            = _DDDTC_data.u32SPN;
    stDTC.u8FMI             = _DDDTC_data.u8FMI;
    stDTC.u8OC              = _DDDTC_data.u8OC;
    stDTC.u16LightStatus    = _DDDTC_data.u16WarningLightStatus;
    return stDTC;
};

AP_PolarisCAN::AP_POLARISCAN_DTC AP_PolarisCAN::get_steering_dtc(){
    uint32_t now_ms = AP_HAL::millis();
    AP_PolarisCAN::AP_POLARISCAN_DTC stDTC;
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _STRDTC_data.u32lastRecvFrameTime > 2*nSTRDTC_PERIOD_MS) && (!_STRDTC_data.boMessageTimeout)){
        _STRDTC_data.boMessageTimeout = true;
    }
    stDTC.index             = 0;
    stDTC.u32SPN            = _STRDTC_data.u32SPN;
    stDTC.u8FMI             = _STRDTC_data.u8FMI;
    stDTC.u8OC              = _STRDTC_data.u8OC;
    stDTC.u16LightStatus    = _STRDTC_data.u16WarningLightStatus;
    return stDTC;
};


MAV_ICE_TRANSMISSION_GEAR_STATE AP_PolarisCAN::get_current_gear() {
    uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_data_sem);
    if ((now_ms - _TRANS1_data.u32lastRecvFrameTime > 2*nTRANS1_PERIOD_MS) && (!_TRANS1_data.boMessageTimeout)){
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
    static uint32_t sendtext_timeref;
    gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"PolarisCAN send mavlink");
    // output telemetry messages
    {
        // take semaphore to access telemetry data
        WITH_SEMAPHORE(_data_sem);

    }
}

#endif // HAL_NUM_CAN_IFACES
