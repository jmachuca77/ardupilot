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

/*
  control of internal combustion engines (starter, ignition and choke)
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

class AP_ICEngine {
public:
    // constructor
    AP_ICEngine();

    /* Do not allow copies */
    AP_ICEngine(const AP_ICEngine &other) = delete;
    AP_ICEngine &operator=(const AP_ICEngine&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // update engine state. Should be called at 10Hz or more
    void update(void) { if (enable) { enable = 0; } }


    void init(bool value) { }
    void set_is_in_auto_mode(bool modeIsAnyAutoNav) { }
    void auto_mode_change_or_new_guided_point_event() {}
    void set_current_throttle(const float throttle) { }
    static AP_ICEngine *get_singleton() { return _singleton; }
    void set_is_waiting_in_auto(bool value) { }
    bool brake_override(float &brake_percent, const float desired_speed, const bool speed_is_valid, const float speed) { return false; }
    bool throttle_override(float &percent) { return false; }
    bool engine_control(float start_control, float cold_start, float height_delay, float gear_state_f) { return false; }
    bool handle_message(const mavlink_command_long_t &packt) { return false; }
    bool handle_set_ice_transmission_state(const mavlink_command_long_t &packet) { return false; }
    bool enabled() const { return false; }
    bool is_changing_gears() { return false; }
    bool has_gears() { return false; }
    bool gear_is_park() { return false; }
    bool gear_is_forward() { return false; }
    bool gear_is_reverse() { return false; }
    bool gear_is_neutral() { return false; }
    bool is_waiting_in_auto() { return false; }
    bool gear_is_inhibiting_locomotion() { return false; }
    float get_idle_throttle() { return 0; }
    void update_idle_governor(int8_t min_throttle) { }
    void mode_change_or_new_autoNav_point_event(bool modeIsAnyAutoNav) { }




private:
    static AP_ICEngine *_singleton;

    // enable library
    AP_Int8 enable;
};


namespace AP {
    AP_ICEngine *ice();
};