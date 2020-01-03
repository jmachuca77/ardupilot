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
  simple internal combustion engine simulator class
*/

#include "SIM_ICEngine.h"
#include <AP_ICEngine/AP_ICEngine.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <SITL/SITL.h>

using namespace SITL;


// table of user settable parameters
const AP_Param::GroupInfo ICEngine::var_info[] = {

    // SIM_ICE_   <--8 chars
    // @Param: RPM_FAIL
    // @DisplayName: Engine RPM failure
    // @Description: Engine RPM failure. When 1 the RPM is zero.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RPM_FAIL", 2, ICEngine, rpm_fail, 0),

    AP_GROUPEND
};

/*
  update engine state, returning power output from 0 to 1
 */
void ICEngine::update(const struct sitl_input &input, float &throttle_demand)
{
    AP_ICEngine *ice = AP::ice();
    const bool has_gears = (ice != nullptr) && ice->has_gears();

    // simulate engine RPM failure of 0rpm or a range of idle of 1000 and max of 7000
    AP::sitl()->state.rpm[0] = (rpm_fail==0 ? 1000+(throttle_demand*6000) : 0);

    const bool have_starter = SRV_Channels::function_assigned(SRV_Channel::k_starter);

    if (!have_starter) {
        // always on
        last_output = throttle_demand;
        return;
    }

    const bool have_ignition = SRV_Channels::function_assigned(SRV_Channel::k_ignition);
    const bool have_choke = SRV_Channels::function_assigned(SRV_Channel::k_choke);

    uint16_t pwmValue;
    state.ignition = (have_ignition && SRV_Channels::get_output_pwm(SRV_Channel::k_ignition, pwmValue)) ? pwmValue>1700:true;
    state.choke = (have_choke && SRV_Channels::get_output_pwm(SRV_Channel::k_choke, pwmValue)) ? pwmValue>1700:false;
    state.starter = (have_starter && SRV_Channels::get_output_pwm(SRV_Channel::k_starter, pwmValue)) ? pwmValue>1700:false;

    const uint64_t now = AP_HAL::micros64();
    const float dt = (now - last_update_us) * 1.0e-6f;
    const float max_change = slew_rate * 0.01f * dt;

    if (state.value != last_state.value) {
        printf("choke:%u starter:%u ignition:%u\n",
               (unsigned)state.choke,
               (unsigned)state.starter,
               (unsigned)state.ignition);
    }
    
    if (have_ignition && !state.ignition) {
        // engine is off
        if (!state.starter) {
            goto engine_off;
        }
        // give 10% when on starter alone without ignition
        last_update_us = now;
        if (!has_gears) {
            throttle_demand = 0.1;
        }
        goto output;
    }
    if (have_choke && state.choke && now - start_time_us > 1000*1000UL) {
        // engine is choked, only run for 1s
        goto engine_off;
    }
    if (last_output <= 0 && !state.starter) {
        // not started
        goto engine_off;
    }
    if (start_time_us == 0 && state.starter) {
        if (throttle_demand > 0.2) {
            printf("too much throttle to start: %.2f\n", throttle_demand);
        } else {
            // start the motor
            if (start_time_us == 0) {
                printf("Engine started\n");
            }
            start_time_us = now;
        }
    }
    if (start_time_us != 0 && state.starter) {
        uint32_t starter_time_us = (now - start_time_us);
        if (starter_time_us > 3000*1000UL && !overheat) {
            overheat = true;
            printf("Starter overheat\n");            
        }
    } else {
        overheat = false;
    }

output:
    if (has_gears) {
        if (ice->gear_is_park() || ice->gear_is_neutral()) {
            throttle_demand = 0;
        }

    } else if (start_time_us != 0 && throttle_demand < 0.01) {
        // even idling it gives some thrust
        throttle_demand = 0.01;
    }

    last_output = constrain_float(throttle_demand, last_output-max_change, last_output+max_change);
    last_output = constrain_float(last_output, 0, 1);
    
    goto done;

engine_off:
    if (start_time_us != 0) {
        printf("Engine stopped\n");
    }
    start_time_us = 0;
    last_output = 0;

done:
    last_update_us = now;
    last_state = state;
    throttle_demand = last_output;
    return;
}
