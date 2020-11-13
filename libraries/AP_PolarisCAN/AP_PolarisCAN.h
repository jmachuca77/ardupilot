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

// Message IDs and Periods, 0 means on demand
// EEC1 Contains Engine Speed (RPM)
#define nEEC1_ID      0x0CF00400
#define nEEC1_PERIOD_MS       20

// ENGTEMP Contains Coolant Temperature
#define nENGTEMP_ID   0x18FEEE00
#define nENGTEMP_PERIOD_MS  1180

// DD Contains Fuel Level
#define nDD_ID        0x18FEFC17
#define nDD_PERIOD_MS       1180

// TRANS1 Contains Gear Transmission State
#define nTRANS1_ID    0x18F00500
#define nTRANS1_PERIOD_MS    110

// FWD Contains Front Wheel Drive Status
#define nFWD_ID       0x1CFDDF00
#define nFWD_PERIOD_MS       550

// ADC Contains Automatic Descent Control Status
#define nADC_ID       0x1CFF6A00
#define nADC_PERIOD_MS       550

// Contains Total Engine hours and revolutions 
#define nENGHRS_ID    0x18FEE500
// Engine hours are requested periodically every 500ms
#define nENGHRS_PERIOD_MS    500

// These messages contain the Diagnostic Trouble Codes
#define nDDDTC_ID     0x18FECA17
#define nDDSTDTC_ID   0x18FECB17
#define nDDDTC_PERIOD_MS    1000

#define nENGDTC_ID    0x18FECA00
#define nENGSTDTC_ID  0x18FECB00
#define nENGDTC_PERIOD_MS   1000

#define nSTRDTC_ID    0x18FECA13
#define nSTRSTDTC_ID  0x18FECB13
#define nSTRDTC_PERIOD_MS   1000

// VHDR contains odometer and trip distance
#define nVDHR_ID      0x18FEC117
#define nVDHR_PERIOD_MS     1100

// Request Engine hours every 5 min.
#define nENG_HOUR_REQ_INTERVAL 300000
#define nSTORED_DTC_REQ_INTERVAL 60000

#define nSIZE_OF_DTC_ARRAY 10
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
    void sendClearDTCs();
    void sendClearStoredDTCs();
    void sendGetStoredDTCs();

    typedef enum {
        NOTENGAGED          = 0,
        ENGAGED             = 1,
        ERROR               = 2,
        NOTAVAILABLE        = 3
    } AP_POLARISCAN_AWD_STATUS;

    typedef struct tDTC {
        uint8_t  index;
        uint32_t u32SPN;
        uint8_t  u8FMI;
        uint8_t  u8OC;
        uint16_t u16LightStatus;
    } AP_POLARISCAN_DTC;

    // Called from AP_RPM when using PolarisCAN type
    uint32_t get_current_rpm();

    // Called from AP_ICEEngine
    MAV_ICE_TRANSMISSION_GEAR_STATE get_current_gear();
    float get_coolant_temp();
    float get_fuel_level();
    AP_POLARISCAN_AWD_STATUS get_FWD_status();
    AP_POLARISCAN_AWD_STATUS get_ADC_status();
    float get_engine_hours();
    AP_POLARISCAN_DTC get_engine_dtc();
    AP_POLARISCAN_DTC get_cluster_dtc();
    AP_POLARISCAN_DTC get_steering_dtc();
    float get_odometer();
    float get_tripDistance();


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

    // loop function to contain all the cyclic routines for this module
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

    // Semaphore to protect access to message data structures
    HAL_Semaphore _data_sem;

    // Structure to store information from the EEC1 CAN Message
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

    struct DD_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint8_t  u8WshrFluidLvl;
        uint8_t  u8FuelLvl;
        uint8_t  u8FuelFltrDiffPress;
        uint8_t  u8EngOilFltrDiffPress;
        uint16_t u16CrgoAmbTemp;        
    } _DD_data;

    struct TRANS1_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint8_t  u8gear;
    } _TRANS1_data;

    struct FWD_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint8_t  u8FWDActStatus;
    } _FWD_data;

    enum SwStatus {
        SW_NEUTRAL       = 0x0,
        SW_LEFT          = 0x1,
        SW_RIGHT         = 0x4
    } SwStatus;

    struct ADC_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint8_t  u8ADCActStatus;
        uint8_t  u84WDSwStatus;
    } _ADC_data;

    struct ENGHRS_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        float    fTotalEngineHours;
        float    fTotalEngineRevolutions;
    } _ENGHRS_data;

    struct STRDTC_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint32_t u32SPN;
        uint8_t  u8FMI;
        uint8_t  u8OC;
        uint16_t u16WarningLightStatus;
    } _STRDTC_data;

    struct DDDTC_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint32_t u32SPN;
        uint8_t  u8FMI;
        uint8_t  u8OC;
        uint16_t u16WarningLightStatus;
    } _DDDTC_data;

    struct ENGDTC_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        uint32_t u32SPN;
        uint8_t  u8FMI;
        uint8_t  u8OC;
        uint16_t u16WarningLightStatus;
    } _ENGDTC_data;

    struct VDHR_data_t {
        uint32_t u32lastRecvFrameTime;
        bool     boMessageTimeout;
        float    fOdometer;
        float    fTripDistance;
    } _VDHR_data;

    struct tDTC STORED_DTC_ARRAY[nSIZE_OF_DTC_ARRAY];

    // structure for sending a Request command
    union req_cmd_t {
        struct PACKED {
            uint8_t data1;
            uint8_t data2;
            uint8_t data3;
        };
        uint8_t data[3];
    };

    // CAN message frame for Requesting Engine hours from J1939
    AP_HAL::CANFrame req_eng_hours_frame;
    AP_HAL::CANFrame req_stored_dtcs_frame;
    AP_HAL::CANFrame clear_dtcs_frame;
};
