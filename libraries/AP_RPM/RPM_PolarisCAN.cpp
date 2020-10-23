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
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_PolarisCAN/AP_PolarisCAN.h>
#include "RPM_PolarisCAN.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/* 
   open the sensor in constructor
*/
AP_RPM_PolarisCAN::AP_RPM_PolarisCAN(AP_RPM &_ap_rpm, uint8_t _instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(_ap_rpm, _instance, _state)
{
    instance = _instance;
}

void AP_RPM_PolarisCAN::update(void)
{
    uint8_t can_num_drivers = AP::can().get_num_drivers();

    for (uint8_t i = 0; i < can_num_drivers; i++) {
        //gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"AP_RPM candrv# %d, drvtype: %d",can_num_drivers, AP::can().get_driver_type(i));
        if (AP::can().get_driver_type(i) == AP_CANManager::Driver_Type_PolarisCAN) {
                AP_PolarisCAN *ap_plcan = AP_PolarisCAN::get_PolarisCAN(i);
                if (ap_plcan == nullptr) {
                    continue;
                }
                state.rate_rpm = (float)ap_plcan->get_current_rpm();
                state.signal_quality = 0.5f;
                state.last_reading_ms = AP_HAL::millis();
        }
    }
}