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

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

class AP_UserCustom {
public:
    AP_UserCustom();
    ~AP_UserCustom() { };

    // remove copy constructor for any class that has a singleton
    AP_UserCustom(const AP_UserCustom &other) = delete;
    AP_UserCustom &operator=(const AP_UserCustom&) = delete;

    static const struct AP_Param::GroupInfo        var_info[];

    static AP_UserCustom *get_singleton() { return _singleton; }

    // update - should be called at at least 10hz but usually 50Hz
    void update();

    // indicate whether this module is enabled or not
    bool enabled() const { return _enabled; }

    bool arming_check(bool report);

    MAV_RESULT handle_user_message(const mavlink_command_long_t &packet);

    int32_t get_lidar_M8_status() const { return lidar_M8_status; }
    bool lidar_M8_status_Startup() const { return get_lidar_M8_status() == MAV_M8_STARTUP; }
    bool lidar_M8_status_Running() const { return get_lidar_M8_status() == MAV_M8_RUNNING; }


    // public parameters
    AP_Int32    test_int1;
    AP_Int32    test_int2;
    AP_Int32    test_int3;
    AP_Float    test_float1;
    AP_Float    test_float2;
    AP_Float    test_float3;

private:
    // initialise this module
    bool init();

    static AP_UserCustom *_singleton;
    bool is_initialized;

    uint32_t arming_retry_ms;
    uint32_t arming_initial_fail_ms;

    // private parameters
    AP_Int8     _enabled;               //  module enable/disable

    int32_t lidar_M8_status;
    AP_Int32 arming_check_Lidar;
    AP_Int32 UI[4];
};

namespace AP {
    AP_UserCustom *usercustom();
};
