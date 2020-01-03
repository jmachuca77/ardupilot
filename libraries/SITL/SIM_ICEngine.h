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
  simple internal combustion motor simulation class
*/

#pragma once

#include <AP_Param/AP_Param.h>
#include "SITL_Input.h"

namespace SITL {

class ICEngine {
public:
    ICEngine() {
        AP_Param::setup_object_defaults(this, var_info);
    };

    const float slew_rate = 100; // percent-per-second

    static const struct AP_Param::GroupInfo var_info[];

    // update motor state and update throttle
    void update(const struct sitl_input &input, float &throttle_demand);

private:

    AP_Int8 rpm_fail;

    float last_output;
    uint64_t start_time_us;
    uint64_t last_update_us;
    union state {
        struct {
            bool choke:1;
            bool ignition:1;
            bool starter:1;
        };
        uint8_t value;
    } state, last_state;
    bool overheat:1;
};
}
