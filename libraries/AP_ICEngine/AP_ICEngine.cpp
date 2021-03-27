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


#include "AP_ICEngine.h"
#include <SRV_Channel/SRV_Channel.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Notify/AP_Notify.h>
#include "AP_ICEngine.h"
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_CANManager/AP_CANManager.h>


extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_Rover)
    #define AP_ICENGINE_TEMP_TOO_HOT_THROTTLE_REDUCTION_FACTOR_DEFAULT  0.25f
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    #define AP_ICENGINE_TEMP_TOO_HOT_THROTTLE_REDUCTION_FACTOR_DEFAULT  0.75f
#else
    #define AP_ICENGINE_TEMP_TOO_HOT_THROTTLE_REDUCTION_FACTOR_DEFAULT  1 // no reduction
#endif

#define AP_ICENGINE_TEMPERATURE_INVALID     -999
#define AP_ICENGINE_FUEL_LEVEL_INVALID      -1

#define ICE_GEAR_STATE_PWM_INVALID 0

const AP_Param::GroupInfo AP_ICEngine::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable ICEngine control
    // @Description: This enables internal combustion engine control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_ICEngine, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: START_CHAN
    // @DisplayName: Input channel for engine start
    // @Description: This is an RC input channel for requesting engine start. Engine will try to start when channel is at or above 1700. Engine will stop when channel is at or below 1300. Between 1301 and 1699 the engine will not change state unless a MAVLink command or mission item commands a state change, or the vehicle is disamed.
    // @User: Standard
    // @Values: 0:None,1:Chan1,2:Chan2,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    AP_GROUPINFO("START_CHAN", 1, AP_ICEngine, start_chan, 0),

    // @Param: STARTER_TIME
    // @DisplayName: Time to run starter
    // @Description: This is the number of seconds to run the starter when trying to start the engine
    // @User: Standard
    // @Units: s
    // @Range: 0.1 5
    AP_GROUPINFO("STARTER_TIME", 2, AP_ICEngine, starter_time, 3),

    // @Param: START_DELAY
    // @DisplayName: Time to wait between starts
    // @Description: Delay between start attempts
    // @User: Standard
    // @Units: s
    // @Range: 1 10
    AP_GROUPINFO("START_DELAY", 3, AP_ICEngine, starter_delay, 2),

    // @Param: RPM_THRESH
    // @DisplayName: RPM threshold
    // @Description: This is the measured RPM above which the engine is considered to be running
    // @User: Standard
    // @Range: 100 100000
    AP_GROUPINFO("RPM_THRESH", 4, AP_ICEngine, rpm_threshold_running, 100),

    // DEPRICATED   5   PWM_IGN_ON, use SERVOx_MAX
    // DEPRICATED   6   PWM_IGN_OFF, use SERVOx_MIN
    // DEPRICATED   7   PWM_STRT_ON, use SERVOx_MAX
    // DEPRICATED   8   PWM_STRT_OFF, use SERVOx_MIN

    // @Param: RPM_CHAN
    // @DisplayName: RPM instance channel to use
    // @Description: This is which of the RPM instances to use for detecting the RPM of the engine
    // @User: Standard
    // @Values: 0:None,1:RPM1,2:RPM2
    AP_GROUPINFO("RPM_CHAN",  9, AP_ICEngine, rpm_instance, 0),

    // @Param: START_PCT
    // @DisplayName: Throttle percentage for engine start
    // @Description: This is the percentage throttle output for engine start
    // @User: Standard
    // @Range: 0 100
    AP_GROUPINFO("START_PCT", 10, AP_ICEngine, start_percent, 5),

    // @Param: IDLE_PCT
    // @DisplayName: Throttle percentage for engine idle
    // @Description: This is the minimum percentage throttle output while running, this includes being disarmed, but not safe
    // @User: Standard
    // @Range: 0 100
    AP_GROUPINFO("IDLE_PCT", 11, AP_ICEngine, idle_percent, 0),

    // @Param: IDLE_RPM
    // @DisplayName: RPM Setpoint for Idle Governor
    // @Description: This configures the RPM that will be commanded by the idle governor. Set to -1 to disable
    // @User: Advanced
    AP_GROUPINFO("IDLE_RPM", 12, AP_ICEngine, idle_rpm, -1),

    // @Param: IDLE_DB
    // @DisplayName: Deadband for Idle Governor
    // @Description: This configures the deadband that is tolerated before adjusting the idle setpoint
    AP_GROUPINFO("IDLE_DB", 13, AP_ICEngine, idle_db, 50),

    // @Param: IDLE_SLEW
    // @DisplayName: Slew Rate for idle control
    // @Description: This configures the slewrate used to adjust the idle setpoint in percentage points per second
    AP_GROUPINFO("IDLE_SLEW", 14, AP_ICEngine, idle_slew, 1),

    // @Param: OPTIONS
    // @DisplayName: ICE options
    // @Description: Options for ICE control
    // @Bitmask: 0:DisableIgnitionRCFailsafe
    AP_GROUPINFO("OPTIONS", 15, AP_ICEngine, options, AP_ICENGINE_OPTIONS_MASK_DEFAULT),

    // @Param: PWM_PARK_D
    // @DisplayName: Gear PWM for Park Down
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in PARK when decreasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_STEP",  16, AP_ICEngine, gear.pwm_step, 1),

    // @Param: RESTART_CNT
    // @DisplayName: Restart attempts allowed
    // @Description: Limit auto-restart attempts to this value. Use -1 to allow unlimited restarts, 0 for no re-starts or higher for that many restart attempts.
    // @Range: -1 100
    // @User: Advanced
    AP_GROUPINFO("RESTART_CNT",  27, AP_ICEngine, resarts_allowed, -1),

    // @Param: PWR_UP_WAIT
    // @DisplayName: Time to wait after applying acceessory
    // @Description: Time to wait after applying acceessory before applying starter.
    // @Units: s
    // @Increment: 1
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("PWR_UP_WAIT", 28, AP_ICEngine, power_up_time, 0),

    // @Param: OUT_EN_PIN
    // @DisplayName: Output Enable Pin
    // @Description: Master Output Enable Pin. Useful to completely disable system during bootup if you have systems that are sensitive to PWM signals during boot. This is helpful to inhibit unintended startups if your output signals are set as reversed
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Advanced
    AP_GROUPINFO("OUT_EN_PIN",  29, AP_ICEngine, master_output_enable_pin, -1),

    // @Param: FUEL_OFFSET
    // @DisplayName: Fuel Level Offset
    // @Description: This makes up for a lack of voltage offset in the battery monitor which only has scaling.
    // @User: Advanced
    AP_GROUPINFO("FUEL_OFFSET",  30, AP_ICEngine, fuel.offset, 0),

    // @Param: RPM_THRESH2
    // @DisplayName: RPM threshold 2 starting
    // @Description: This is the measured RPM above which the engine is considered to be successfully started and the remaining starter time (ICE_STARTER_TIME) will be skipped. Use 0 to diable and always start for the full STARTER_TIME duration
    // @User: Standard
    // @Range: 0 100000
    AP_GROUPINFO("RPM_THRESH2", 31, AP_ICEngine, rpm_threshold_starting, 0),

    // @Param: TEMP_PIN
    // @DisplayName: Temperature analog feedback pin
    // @Description: Temperature analog feedback pin. This is used to sample the engine temperature.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Advanced
    AP_GROUPINFO("TEMP_PIN", 32, AP_ICEngine, temperature.pin, -1),

    // @Param: TEMP_SCALER
    // @DisplayName: Temperature scaler
    // @Description: Temperature scaler to apply to analog input to convert voltage to degrees C
    // @User: Advanced
    AP_GROUPINFO("TEMP_SCALER", 33, AP_ICEngine, temperature.scaler, 1),

    // @Param: TEMP_MAX
    // @DisplayName: Temperature overheat
    // @Description: Temperature limit that is considered overheating. When above this temperature the starting and throttle will be limited/inhibited. Use 0 to disable.
    // @User: Advanced
    // @Units: degC
    AP_GROUPINFO("TEMP_MAX", 34, AP_ICEngine, temperature.max, 105),

    // @Param: TEMP_MIN
    // @DisplayName: Temperature minimum
    // @Description: Temperature minimum that is considered too cold to run the engine. While under this temp the throttle will be inhibited. Use 0 to disable.
    // @User: Advanced
    // @Units: degC
    AP_GROUPINFO("TEMP_MIN", 35, AP_ICEngine, temperature.min, 10),

    // @Param: TEMP_RMETRIC
    // @DisplayName: Temperature is Ratiometric
    // @Description: This parameter sets whether an analog temperature is ratiometric. Most analog analog sensors are ratiometric, meaning that their output voltage is influenced by the supply voltage.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("TEMP_RMETRIC",36, AP_ICEngine, temperature.ratiometric, 1),

    // @Param: TEMP_OFFSET
    // @DisplayName: Temperature voltage offset
    // @Description: Offset in volts for analog sensor.
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("TEMP_OFFSET", 37, AP_ICEngine, temperature.offset, 0),

    // @Param: TEMP_FUNC
    // @DisplayName: Temperature sensor function
    // @Description: Control over what function is used to calculate temperature. For a linear function, the temp is (voltage-offset)*scaling. For a inverted function the temp is (offset-voltage)*scaling. For a hyperbolic function the temp is scaling/(voltage-offset).
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Standard
    AP_GROUPINFO("TEMP_FUNC", 38, AP_ICEngine, temperature.function, 0),

    // @Param: TEMP_HOT_THR
    // @DisplayName: Temperature overheat throttle behavior
    // @Description: Throttle reduction factor during an overheat. Smaller
    // @User: Advanced
    // @Range: 0 1
    AP_GROUPINFO("TEMP_HOT_THR", 39, AP_ICEngine, temperature.too_hot_throttle_reduction_factor, AP_ICENGINE_TEMP_TOO_HOT_THROTTLE_REDUCTION_FACTOR_DEFAULT),

    // @Param: PWM_PARK_U
    // @DisplayName: Gear PWM for Park Up
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in PARK when increasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_PARK_U",  40, AP_ICEngine, gear.pwm_park_up, 1000),

    // @Param: PWM_PARK_D
    // @DisplayName: Gear PWM for Park Down
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in PARK when decreasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_PARK_D",  41, AP_ICEngine, gear.pwm_park_down, 1000),

    // @Param: PWM_REV_U
    // @DisplayName: Gear PWM for Reverse Up
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in REVERSE when increasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_REV_U",  42, AP_ICEngine, gear.pwm_reverse_up, 1200),

    // @Param: PWM_REV_D
    // @DisplayName: Gear PWM for Reverse Down
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in REVERSE when decreasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_REV_D",  43, AP_ICEngine, gear.pwm_reverse_down, 1200),

    // @Param: PWM_NTRL_U
    // @DisplayName: Gear PWM for Neutral Up
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in NEUTRAL when increasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_NTRL_U",  44, AP_ICEngine, gear.pwm_neutral_up, 1295),

    // @Param: PWM_NTRL_D
    // @DisplayName: Gear PWM for Neutral Down
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in NEUTRAL when decreasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_NTRL_D",  45, AP_ICEngine, gear.pwm_neutral_down, 1295),

    // @Param: PWM_FWD1_U
    // @DisplayName: Gear PWM for Forward 1 Up
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in FORWARD1 when increasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_FWD1_U",  46, AP_ICEngine, gear.pwm_forward1_up, 1425),

    // @Param: PWM_FWD1_D
    // @DisplayName: Gear PWM for Forward 1 Down
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in FORWARD1 when decreasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_FWD1_D",  47, AP_ICEngine, gear.pwm_forward1_down, 1425),

    // @Param: PWM_FWD2_U
    // @DisplayName: Gear PWM for Forward 2 Up
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in FORWARD2 when increasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_FWD2_U",  48, AP_ICEngine, gear.pwm_forward2_up, 1600),

    // @Param: PWM_FWD2_D
    // @DisplayName: Gear PWM for Forward 2 Down
    // @Description: This is the output PWM value sent to the gear servo channel when the vehicle transmission is in FORWARD2 when decreasing the PWM
    // @User: Advanced
    AP_GROUPINFO("PWM_FWD2_D",  49, AP_ICEngine, gear.pwm_forward2_down, 1600),

    // @Param: GEAR_STOP
    // @DisplayName: Gear change stop vehicle time
    // @Description: Gear change duration to inhibit throttle while waiting for vehicle to stop moving before changing physical gear
    // @User: Advanced
    AP_GROUPINFO("GEAR_STOP",  50, AP_ICEngine, gear.pending.stop_duration, 0),

    // @Param: GEAR_DUR
    // @DisplayName: Gear change duration
    // @Description: Gear change duration to inhibit throttle while physically changing the gear. This is the time it takes to change one gear-distance. Actual duration is this param multiplied by how many gears it has to change.
    // @User: Advanced
    AP_GROUPINFO("GEAR_DUR",  51, AP_ICEngine, gear.pending.change_duration_per_posiiton, 1.5f),

    // @Param: CHRG_TIME
    // @DisplayName: Self Charge Duration
    // @Description: When the selected battery is below its threshold voltage, the engine will turn on so that the alternator has a chance to charge itself. This is the duration that the motor will stay on to charge before shutting off. This feature will only work while in PARK.
    // @Units: s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CHRG_TIME",  52, AP_ICEngine, recharge.duration_seconds, (5 * 60)),

    // @Param: CHRG_VOLT
    // @DisplayName: Self Charge Voltage Threshold
    // @Description: The low-battery voltage threshold that triggers the engine to turn on to self-charge. Use -1 to disable self-charge. This feature will only work while in PARK.
    // @Units: V
    // @Increment: 0.1
    // @Range: -1 100
    // @User: Advanced
    AP_GROUPINFO("CHRG_VOLT",  53, AP_ICEngine, recharge.voltage_threshold, -1),

    // @Param: CHRG_BATT
    // @DisplayName: Self Charge Battery Select
    // @Description: Selects which battery instances to use for ICE_CHRG_VOLT. This feature will only work while in PARK.
    // @Values: 0:Primary,1:BATT,2:BATT2,3:BATT3,4:BATT4,5:BATT5,6:BATT6,7:BATT7,8:BATT8,9:BATT9
    // @User: Advanced
    AP_GROUPINFO("CHRG_BATT",  54, AP_ICEngine, recharge.battery_instance, 0),

    // @Param: CHRG_THR
    // @DisplayName: Self Charge Battery Throttle
    // @Description: Idle throttle while charging
    // @Range: 0 100
    // @Units: %
    // @User: Advanced
    AP_GROUPINFO("CHRG_THR",  55, AP_ICEngine, recharge.throttle, 0),

    // @Param: NEUTRAL_BRAK
    // @DisplayName: Apply full brake when in Neutral
    // @Description: Apply full brake when in Neutral. This applies a brake override while in Neutral gear
    // @Values: -1:No override,0:Brake is 0%,1:Brake is 100%
    // @User: Advanced
    AP_GROUPINFO("NEUTRAL_BRAK",  56, AP_ICEngine, neutral_brake_enable, 0),

    // @Param: IGN_PIN
    // @DisplayName: Ignition Servo or Relay pin select
    // @Description: Ignition Servo or Relay pin select
    // @Values: 0:Set by SERVOx_FUNCTION,1:Relay1,2:Relay2,3:Relay3,4:Relay4,5:Relay5,6:Relay6,7:Relay7,8:Relay8,9:Relay9,10:Relay10,11:Relay11,12:Relay12,13:Relay13,14:Relay14
    // @User: Advanced
    AP_GROUPINFO("IGN_PIN",  57, AP_ICEngine, ignition_pin, 0),

    // @Param: START_PIN
    // @DisplayName: Starter Servo or Relay pin select
    // @Description: Starter Servo or Relay pin select
    // @Values: 0:Set by SERVOx_FUNCTION,1:Relay1,2:Relay2,3:Relay3,4:Relay4,5:Relay5,6:Relay6,7:Relay7,8:Relay8,9:Relay9,10:Relay10,11:Relay11,12:Relay12,13:Relay13,14:Relay14
    // @User: Advanced
    AP_GROUPINFO("START_PIN",  58, AP_ICEngine, starter_pin, 0),

    // @Param: FUEL_SRC
    // @DisplayName: Fuel Level Source
    // @Description: This is used to select if the fuel level is read by analog sensor or by CAN bus.
    // @Values: 0:Disabled,1:Analog,2:PolarisCAN
    // @User: Advanced
    AP_GROUPINFO("FUEL_SRC", 59, AP_ICEngine, fuel.source, 1),

    // @Param: GEAR_SRC
    // @DisplayName: Gear Status Source
    // @Description: This is used to select if the gear status is fedback from pwm values or by CAN bus.
    // @Values: 0:Disabled,1:PWM,2:PolarisCAN
    // @User: Advanced
    AP_GROUPINFO("GEAR_SRC", 60, AP_ICEngine, gear.source, 1),

    // @Param: TEMP_SRC
    // @DisplayName: Coolant Temperature Source
    // @Description: This is used to select if the Coolant Temperature is read by analog sensor or by CAN bus.
    // @Values: 0:Disabled,1:Analog,2:PolarisCAN
    // @User: Advanced
    AP_GROUPINFO("TEMP_SRC", 61, AP_ICEngine, temperature.source, 1),

    AP_GROUPEND
};


// constructor
AP_ICEngine::AP_ICEngine()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ICEngine must be singleton");
    }
    _singleton = this;
}

/*
 Initialize ICE outputs.
*/
void AP_ICEngine::init(const bool inhibit_outputs)
{
    if (master_output_enable_pin >= 0) {
        hal.gpio->pinMode(master_output_enable_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(master_output_enable_pin, inhibit_outputs);
    }
    set_output_channels();

    RC_Channel *c = rc().channel(start_chan-1);
    if (c != nullptr) {
        const uint16_t boot_up_value = c->get_radio_trim();
        c->set_override(boot_up_value, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, AP_HAL::millis());
        c->set_radio_in(boot_up_value);
        set_ignition_state(convertPwmToIgnitionState(boot_up_value));
    } else {
        set_ignition_state(ICE_IGNITION_OFF);
    }

    gear.pending.cancel();

    recharge.set_state(Recharge::ICE_RECHARGE_STATE_OFF);
}

AP_ICEngine::ice_ignition_state_t AP_ICEngine::convertPwmToIgnitionState(const uint16_t pwm)
{
    // low = off
    // mid = accessory/run only
    // high accessory/run + allow auto starting

    if (pwm <= 1300) {
        return ICE_IGNITION_OFF;
    } else if (pwm >= 1700) {
        return ICE_IGNITION_START_RUN;
    } else {
        return ICE_IGNITION_ACCESSORY;
    }
}
/*
  update engine state
 */
void AP_ICEngine::update(void)
{
    if (!enabled()) {
        state = ICE_OFF;
        if (run_once) {
            run_once = false;
            init(true);
        }
        return;
    }

    if (!run_once) {
        run_once = true;
        init(false);
    }

    update_temperature();
    update_fuel();

    determine_state();

    update_gear();

    set_output_channels();

    update_self_charging();

    send_status();
}

void AP_ICEngine::update_self_charging()
{

    if ((gear.state != MAV_ICE_TRANSMISSION_GEAR_STATE_PARK) || gear.pending.is_active()) {
        if (recharge.state == Recharge::ICE_RECHARGE_STATE_CHARGING) {
            gcs().send_text(MAV_SEVERITY_INFO, "%sOff", recharge.msg);
            // uncontrolled finish
            if (AP::arming().is_armed() && (AP::arming().get_armed_method() == AP_Arming::Method::ICE_RECHARGE)) {
                AP::arming().disarm(AP_Arming::Method::ICE_RECHARGE);
            }
        }
        recharge.set_state(Recharge::ICE_RECHARGE_STATE_OFF);
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const float battery_voltage = recharge.get_smoothed_battery_voltage();

    switch (recharge.state) {
        case Recharge::ICE_RECHARGE_STATE_OFF:
            if (recharge.battery_instance >= 0 && recharge.battery_instance <= AP_BATT_MONITOR_MAX_INSTANCES &&
                recharge.duration_seconds > 0 &&
                recharge.voltage_threshold > 0 &&
                battery_voltage > recharge.minimum_voltage)
            {
                // params all look OK.
                recharge.set_state(Recharge::ICE_RECHARGE_STATE_CHECKING_BATTERY);
            }
            break;

        case Recharge::ICE_RECHARGE_STATE_CHECKING_BATTERY:
            if (battery_voltage <= recharge.minimum_voltage ||
                    battery_voltage >= recharge.voltage_threshold ||
                    startControlSelect == ICE_IGNITION_START_RUN) {
                // not ready to charge
                recharge.set_state(Recharge::ICE_RECHARGE_STATE_SNOOZING);

            } else if (!recharge.timer_ms) {
                recharge.timer_ms = now_ms;

            } else if (now_ms - recharge.timer_ms > 3000) {
                // 3 second debounce
                recharge.set_state(Recharge::ICE_RECHARGE_STATE_CHARGING_PENDING);
            }
            break;

        case Recharge::ICE_RECHARGE_STATE_CHARGING_PENDING:
            if (startControlSelect == ICE_IGNITION_START_RUN) {
                recharge.set_state(Recharge::ICE_RECHARGE_STATE_SNOOZING);

            } else if (!recharge.timer_ms) {
                recharge.timer_ms = now_ms;

            } else if (now_ms - recharge.timer_ms > 20000) {
                recharge.set_state(Recharge::ICE_RECHARGE_STATE_CHARGING);

            } else if (!recharge.notify_ms || (now_ms - recharge.notify_ms > 5*1000)) {
                recharge.notify_ms = now_ms;
                gcs().send_text(MAV_SEVERITY_INFO, "%s%.2fV, Pending", recharge.msg, battery_voltage);
            }
            break;

        case Recharge::ICE_RECHARGE_STATE_CHARGING:
            if (recharge.timer_ms == 0) {
                recharge.timer_ms = now_ms;

                if (!AP::arming().is_armed()) {
                    // override arming checks. This is dangerous so make sure we disarm no-matter-what
                    // when we stop charging, whether from it finishing or user interrupts
                    AP::arming().arm(AP_Arming::Method::ICE_RECHARGE, false);
                }

                gcs().send_text(MAV_SEVERITY_INFO, "%s%.2fV, Start", recharge.msg, battery_voltage);
                engine_control(2, 0, 0, 0); // start the engine

            } else if (now_ms - recharge.timer_ms >= ((uint32_t)recharge.duration_seconds * 1000)) {
                // controlled finish
                if (AP::arming().is_armed() && (AP::arming().get_armed_method() == AP_Arming::Method::ICE_RECHARGE)) {
                    AP::arming().disarm(AP_Arming::Method::ICE_RECHARGE);
                }

                gcs().send_text(MAV_SEVERITY_INFO, "%s%.2fV, Done", recharge.msg, battery_voltage);
                engine_control(0, 0, 0, 0); // stop the engine
                recharge.set_state(Recharge::ICE_RECHARGE_STATE_SNOOZING);

            } else if (!recharge.notify_ms || (now_ms - recharge.notify_ms > 10*1000)) {
                recharge.notify_ms = now_ms;

                const uint32_t elapsed_seconds = (now_ms - recharge.timer_ms) * 0.001f;
                const uint32_t seconds_remaining = (uint32_t)recharge.duration_seconds - elapsed_seconds;
                if (seconds_remaining % 10) {
                    // nudge the notify timer by 1 second so we start getting even seconds with a 0 tens digits
                    recharge.notify_ms -= 1000;
                }

                gcs().send_text(MAV_SEVERITY_INFO, "%s%.2fV, Active %um %us left", recharge.msg, battery_voltage, (unsigned)(seconds_remaining/60), (unsigned)(seconds_remaining%60));
            }
            break;

        case Recharge::ICE_RECHARGE_STATE_SNOOZING:
            if (recharge.timer_ms == 0) {
                recharge.timer_ms = now_ms;

            } else if (now_ms - recharge.timer_ms > recharge.snooze_duration_ms) {
                recharge.set_state(Recharge::ICE_RECHARGE_STATE_OFF);
            }
        break;
    }
}

void AP_ICEngine::Recharge::pending_abort() {
    if (state == ICE_RECHARGE_STATE_CHARGING_PENDING) {
        set_state(ICE_RECHARGE_STATE_SNOOZING);
        gcs().send_text(MAV_SEVERITY_INFO, "%sPending Abort", msg);
    }
}

void AP_ICEngine::Recharge::set_state(Recharge_State next_state) {
//    ICE_RECHARGE_STATE_OFF,
//    ICE_RECHARGE_STATE_CHECKING_BATTERY,
//    ICE_RECHARGE_STATE_CHARGING_PENDING,
//    ICE_RECHARGE_STATE_CHARGING,
//    ICE_RECHARGE_STATE_SNOOZING,
#if 0
    if (state != next_state) {
        gcs().send_text(MAV_SEVERITY_INFO, "%d Recharge state change from %d to %d", AP_HAL::millis(), state, next_state);
    }
#endif
    state = next_state;
    timer_ms = 0;
    notify_ms = 0;
}

float AP_ICEngine::Recharge::get_smoothed_battery_voltage()
{
    float voltage = -1;
    if (battery_instance == 0) {
        if (AP::battery().healthy()) {
            voltage = AP::battery().voltage();
        }
    } else if (battery_instance >= 1 && battery_instance <= AP_BATT_MONITOR_MAX_INSTANCES) {
        if (AP::battery().healthy(battery_instance-1)) {
            voltage = AP::battery().voltage(battery_instance-1);
        }
    }

    if (voltage < minimum_voltage) {
        return -1;
    } else if (battery_voltage_last < minimum_voltage) {
        battery_voltage_last = voltage;
    }

    const float diff = fabsf(battery_voltage_last - voltage);
    const float LPFcoef = diff < 0.5f ? 0.95f : 0.8f; // go faster if its a large difference
    battery_voltage_last = LPFcoef * battery_voltage_last + (1.0f-LPFcoef)*voltage;

    return battery_voltage_last;
}

AP_ICEngine::enGearOrder_t AP_ICEngine::convertGearStateToGearOrder(MAV_ICE_TRANSMISSION_GEAR_STATE gearState) {
    AP_ICEngine::enGearOrder_t gearOrder;
                    
    switch (gearState) {
        case MAV_ICE_TRANSMISSION_GEAR_STATE_PARK: /* Park. | */
            gearOrder = ICE_GEAR_ORDER_PARK;
            break;
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE: /* Reverse for single gear systems or Variable Transmissions. | */
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_1: /* Reverse 1. Implies multiple gears exist. | */
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_2:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_3:
            gearOrder = ICE_GEAR_ORDER_REVERSE;
            break;
        case MAV_ICE_TRANSMISSION_GEAR_STATE_NEUTRAL: /* Neutral. Engine is physically disconnected. | */
            gearOrder = ICE_GEAR_ORDER_NEUTRAL;
            break;
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD: /* Forward for single gear systems or Variable Transmissions. | */
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1: /* First gear. Implies multiple gears exist. | */
            gearOrder = ICE_GEAR_ORDER_LOWGEAR;
            break;
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_2: /* Second gear. | */
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_3:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_4:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_5:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_6:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_7:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_8:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_9:
            gearOrder = ICE_GEAR_ORDER_HIGHGEAR;
            break;
        case MAV_ICE_TRANSMISSION_GEAR_STATE_PWM_VALUE:
        default:
            gearOrder = ICE_GEAR_ORDER_PARK;
            break;
    }

    return gearOrder;
}

void AP_ICEngine::determine_state()
{
    RC_Channel *c = rc().channel(start_chan-1);
    if (c != nullptr) {
        // check for 2 or 3 position switch:
        set_ignition_state(convertPwmToIgnitionState(c->get_radio_in()));
    }

    const uint32_t now_ms = AP_HAL::millis();

    const bool is_soft_armed = hal.util->get_soft_armed();
    const bool arming_OK_to_ign          = is_soft_armed || (!is_soft_armed && !(options & AP_ICENGINE_OPTIONS_MASK_ARMING_REQUIRED_IGNITION));
    const bool arming_OK_to_start_or_run = is_soft_armed || (!is_soft_armed && !(options & AP_ICENGINE_OPTIONS_MASK_ARMING_REQUIRED_START));
    const bool system_should_be_off = (startControlSelect == ICE_IGNITION_OFF) || !arming_OK_to_ign || ((options & uint16_t(Options::DISABLE_IGNITION_RC_FAILSAFE)) && AP_Notify::flags.failsafe_radio);

    if (system_should_be_off) {
        if (state != ICE_OFF) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine stopped");
        }
        state = ICE_OFF;
    }

    float current_rpm = -1;
    AP_RPM *rpm = AP::rpm();
    if (rpm_instance > 0 && rpm != nullptr) {
        rpm->get_rpm(rpm_instance-1, current_rpm);
    }

    // switch on current state to work out new state
    switch (state) {
    default:
        state = ICE_OFF;
        break;

    case ICE_OFF:
        starting_attempts = 0;
        if (!system_should_be_off && (startControlSelect != ICE_IGNITION_OFF)) {
            state = ICE_START_DELAY;
        }
        break;

#if !APM_BUILD_TYPE(APM_BUILD_Rover)
    case ICE_START_HEIGHT_DELAY: {
        // NOTE, this state can only be achieved via MAVLink command so the starter RCin is not checked
        Vector3f pos;
        if (!AP::ahrs().get_relative_position_NED_origin(pos)) {
            break;
        }

        if (height_pending || !is_soft_armed) {
            // reset initial height while disarmed or when forced
            height_pending = false;
            initial_height = -pos.z;
        } else if ((-pos.z) >= initial_height + height_required) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine starting height reached %.1f",
                                                (double)(-pos.z - initial_height));
            state = ICE_STARTING;
        }
        break;
    }
#endif

    case ICE_START_DELAY_NO_IGNITION:
        // this state is usually skipped, it's only used when ICE_RUNNING fails
        // and we want to stop the motor but not reset starting_attempts. This will
        // force the ignition off if it's actually still on because in ICE_DELAY_START
        // the ignition is on which would keep the engine running and then we're run
        // the starter with a running engine for a fraction of a moment.
        if (force_staying_in_DELAY_NO_IGNITION_duration_ms > 0 && now_ms - state_change_timestamp_ms < force_staying_in_DELAY_NO_IGNITION_duration_ms) {
            break;
        }
        force_staying_in_DELAY_NO_IGNITION_duration_ms = 0;
        break;

    case ICE_START_DELAY:
        if ((startControlSelect != ICE_IGNITION_START_RUN) || !arming_OK_to_start_or_run) {
            // nothing to do, linger in this state forever
            break;
        }
        if (resarts_allowed >= 0 && resarts_allowed < starting_attempts) {
            // we were running, auto-restarts are blocked. Linger in this state forever until ICE_OFF clears this state
            break;
        }

        if (power_up_time > 0) {
            if (engine_power_up_wait_ms == 0) {
                gcs().send_text(MAV_SEVERITY_INFO, "Engine waiting for %ds", (int)power_up_time);
                engine_power_up_wait_ms = now_ms;
                // linger in the current state
                break;
            } else if (now_ms - engine_power_up_wait_ms < (uint32_t)power_up_time*1000) {
                // linger in the current state
                break;
            }
        }

        if (!delay_starter_on_first_since_boot_timer_ms) {
            delay_starter_on_first_since_boot_timer_ms = now_ms;
        }

        if (now_ms - delay_starter_on_first_since_boot_timer_ms < 1000) {
            // to reduce power surges, delay a run (act like start/ignition) for an extra second but only on the first time.
            break;
        } else if (starter_delay <= 0) {
            state = ICE_STARTING;
        } else if (!starter_last_run_ms || now_ms - starter_last_run_ms >= starter_delay*1000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine starting for up to %.1fs", (double)starter_delay);
            state = ICE_STARTING;
        }
        break;

    case ICE_STARTING:
        engine_power_up_wait_ms = 0;
        if (starter_start_time_ms == 0) {
            starting_attempts++;
            starter_start_time_ms = now_ms;
        }
        starter_last_run_ms = now_ms;

        if (!arming_OK_to_start_or_run) {
            // user abort
            gcs().send_text(MAV_SEVERITY_INFO, "Engine stopped");
            state = ICE_START_DELAY;
        } else if (rpm_threshold_starting > 0 && current_rpm >= rpm_threshold_starting) {
            // RPM_THRESH2 exceeded, we know we're running
            // check RPM to see if we've started or if we'ved tried fo rlong enought. If so, skip to running
            gcs().send_text(MAV_SEVERITY_INFO, "Engine running! Detected %d rpm", (int)current_rpm);
            state = ICE_RUNNING;
            update_gear();
        } else if (now_ms - starter_start_time_ms >= starter_time*1000) {
            // STARTER_TIME expired
            if (rpm_threshold_starting <= 0) {
                // without an rpm sensor we have to assume we're successful
                gcs().send_text(MAV_SEVERITY_INFO, "Engine running! (No rpm feedback)");
                state = ICE_RUNNING;
                update_gear();
            } else if (current_rpm < 0) {
                // we're expecting an rpm but never saw it, lets sanity check it
                gcs().send_text(MAV_SEVERITY_INFO, "Engine start failed. Check rpm configuration");
                state = ICE_OFF;
            } else if (current_rpm < rpm_threshold_starting) {
                // not running, start has failed.
                gcs().send_text(MAV_SEVERITY_INFO, "Engine start failed. Detected %d rpm", (int)current_rpm);
                state = ICE_START_DELAY;
            }
        }
        break;

    case ICE_RUNNING:
        engine_power_up_wait_ms = 0;
        if (!is_soft_armed && get_idle_throttle() <= 0 && !(options & AP_ICENGINE_OPTIONS_MASK_KEEP_RUNNING_WHEN_DISARMED)) {
            // turn off when disarmed unless we need to idle or if we think it's OK to keep running while disarmed
            state = ICE_OFF;
            gcs().send_text(MAV_SEVERITY_INFO, "Engine stopped, disarmed");
            break;
        }

        // switch position can be either acc or acc_start while in this state

        if (rpm_threshold_running > 0 && current_rpm >= 0 && current_rpm < rpm_threshold_running) {
            // we're expecting an rpm, have a valid rpm, and the rpm is too low.
            // engine has stopped when it should be running

            if (!running_rpm_fail_timer_ms) {
                running_rpm_fail_timer_ms = now_ms;
            }

            if ((options & AP_ICENGINE_OPTIONS_MASK_RPM_FAIL_HAS_TIMER) && (now_ms - running_rpm_fail_timer_ms <= 500)) {
                // do nothing, just ignore the rpm for now
                break;
            } else if (options & AP_ICENGINE_OPTIONS_MASK_RUNNING_FAIL_FORCE_STOP_MOTOR) {
                // in the case of a noisy rpm signal, ensure we actually turn off the ignition
                state = ICE_START_DELAY_NO_IGNITION;
                force_staying_in_DELAY_NO_IGNITION_duration_ms = 3000;
            } else {
                state = ICE_START_DELAY;
            }

            if (state != ICE_RUNNING) {
                gcs().send_text(MAV_SEVERITY_INFO, "Engine died while running: %d rpm", (int)current_rpm);
            }
        } else {
            running_rpm_fail_timer_ms = 0;
        }

        break;
    } // switch

    if (state != ICE_STARTING) {
        starter_start_time_ms = 0;
    }

    if (state_prev != state) {
        state_change_timestamp_ms = now_ms;
    }
    state_prev = state;
}

void AP_ICEngine::set_output_channels()
{
    if ((!SRV_Channels::function_assigned(SRV_Channel::k_engine_gear) || (gear.source <= 0)) && (gear.source != 2)) {
        // if we don't have a gear then set it to a known invalid state
        gear.pwm_active = ICE_GEAR_STATE_PWM_INVALID;
        gear.state = MAV_ICE_TRANSMISSION_GEAR_STATE_UNKNOWN;
    } else if (gear.state == MAV_ICE_TRANSMISSION_GEAR_STATE_UNKNOWN) {
        // on boot or in an unknown state, set gear to trim and find out what that value is and set to that state
        SRV_Channels::set_output_to_trim(SRV_Channel::k_engine_gear);
        SRV_Channels::get_output_pwm(SRV_Channel::k_engine_gear, gear.pwm_active);
        // only set gear state if we are not using CAN for gear status.
        if (gear.source == 1) {  
            gear.state = convertPwmToGearState(gear.pwm_active);
        }
    } else {
        // normal operation, set the output
        if (!(options & AP_ICENGINE_OPTIOMS_MASK_GEAR_CTRL_TYPE)) {
            SRV_Channels::set_output_pwm(SRV_Channel::k_engine_gear, gear.pwm_active);
        } else {
            if(gear.state != gear.pending.state) {
                // get-travel direction.
                // increase current PWM in that direction
                AP_ICEngine::enGearOrder_t currentGearOrder = convertGearStateToGearOrder(gear.state);
                AP_ICEngine::enGearOrder_t targetGearOrder = convertGearStateToGearOrder(gear.pending.state);
                
                uint16_t pwmvalue; 
                SRV_Channels::get_output_pwm(SRV_Channel::k_engine_gear, pwmvalue);
                if (currentGearOrder < targetGearOrder) {
                    //moveup
                    SRV_Channels::get_output_pwm(SRV_Channel::k_engine_gear, gear.pwm_actual);
                    gear.pwm_actual += gear.pwm_step;
                    SRV_Channels::set_output_pwm(SRV_Channel::k_engine_gear, gear.pwm_actual);
                    SRV_Channels::constrain_pwm(SRV_Channel::k_engine_gear);
                } else if (currentGearOrder > targetGearOrder) {
                    //movedown
                    SRV_Channels::get_output_pwm(SRV_Channel::k_engine_gear, gear.pwm_actual);
                    gear.pwm_actual -= gear.pwm_step;
                    SRV_Channels::set_output_pwm(SRV_Channel::k_engine_gear, gear.pwm_actual);
                    SRV_Channels::constrain_pwm(SRV_Channel::k_engine_gear);
                }
                // static uint32_t sendtext_timeref;
                // gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"AP_ICE currgear: %d, tgtGear: %d, pwm: %d, gearpwm: %d, gear.state: %s, gear.pending.state: %s", currentGearOrder, targetGearOrder, pwmvalue, gear.pwm_actual, get_gear_name(gear.state), get_gear_name(gear.pending.state));
            } 
        }

        
    }

    if (gear.pending.is_active() && state != ICE_OFF) {
        // if we're actively changing gears, don't change our ignition state unless we're trying to turn off
        return;
    }

    switch (state) {
    case ICE_OFF:
    case ICE_START_DELAY_NO_IGNITION:
        set_srv_or_relay(ignition_pin, SRV_Channel::k_ignition, -1);
        set_srv_or_relay(starter_pin, SRV_Channel::k_starter, -1);
        break;

    case ICE_START_HEIGHT_DELAY:
    case ICE_START_DELAY:
        set_srv_or_relay(ignition_pin, SRV_Channel::k_ignition, 100);
        set_srv_or_relay(starter_pin, SRV_Channel::k_starter, 0);
        break;

    case ICE_STARTING:
        set_srv_or_relay(ignition_pin, SRV_Channel::k_ignition, 100);
        set_srv_or_relay(starter_pin, SRV_Channel::k_starter, 100);
        break;

    case ICE_RUNNING:
        set_srv_or_relay(ignition_pin, SRV_Channel::k_ignition, 100);
        set_srv_or_relay(starter_pin, SRV_Channel::k_starter, 0);
        break;
    } // switch
}

// set servo_function or relay to desired value. Use -1 to set to trim/default, otherwise relay is boolean and servos are percent
void AP_ICEngine::set_srv_or_relay(int8_t pin, SRV_Channel::Aux_servo_function_t srv_func, int8_t desired_value)
{
    if (pin <= 0) {
        if (desired_value < 0) {
            const SRV_Channel *srv_chan = SRV_Channels::get_channel_for(srv_func);
            if (srv_chan != nullptr) {
                // trim value dictates off state
                SRV_Channels::set_output_pwm(srv_func, srv_chan->get_trim());
            }
        } else {
            SRV_Channels::set_output_scaled(srv_func, (desired_value ? 100 : 0));
        }

    } else if (pin <= AP_RELAY_NUM_RELAYS) {
        AP_Relay *relay = AP::relay();
        if (relay != nullptr) {
            const uint8_t pin_index = pin - 1;
            if (desired_value < 0) {
                relay->set_to_default(pin_index);
            } else {
                relay->set(pin_index, desired_value);
            }
        }
    }
}

/*
check for brake override. This allows the ICE controller to force
the brake when starting the engine
*/
bool AP_ICEngine::brake_override(float &brake_percent, const float desired_speed, const bool speed_is_valid, const float speed)
{
    const float brake_percent_start = brake_percent;

    if (!enabled()) {
        return false;
    }


    switch (gear.state) {
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_1:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_2:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_3:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_2:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_3:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_4:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_5:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_6:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_7:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_8:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_9:
            if (!hal.util->get_soft_armed()) {
                // disarmed
                brake_percent = 100;
            } else if (is_equal(desired_speed, 0.0f) && speed_is_valid && fabsf(speed) < 0.1f) {
                // we want speed=0 and we are about speed=0
                brake_percent = 100;
            }
            break;

        case MAV_ICE_TRANSMISSION_GEAR_STATE_NEUTRAL:
            if (neutral_brake_enable == 0) {
                brake_percent = 0;
            } else if (neutral_brake_enable == 1) {
                brake_percent = 100;
            } else {
                // no action, let the existing value pass-thru unchanged
            }
            break;

        case MAV_ICE_TRANSMISSION_GEAR_STATE_PARK:
            brake_percent = 0;
            break;

        case MAV_ICE_TRANSMISSION_GEAR_STATE_UNKNOWN:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_PWM_VALUE:
        default:
            // unhandled, no brake management
            break;
    }

    if (gear.pending.is_active()) {
        brake_percent = 100;
    }
//    if (state == ICE_STARTING || state == ICE_START_DELAY) {
//        // when starting, apply full brake
//        brake_percent = 100;
//    }

    return !is_equal(brake_percent, brake_percent_start);
}

void AP_ICEngine::update_gear()
{
    const uint32_t now_ms = AP_HAL::millis();

    // sanity check params
    if (is_negative(gear.pending.stop_duration)) {
        gear.pending.stop_duration.set_and_save(0);
    }
    if (is_negative(gear.pending.change_duration_per_posiiton)) {
        gear.pending.change_duration_per_posiiton.set_and_save(2);
    }

    // Update gear state from CAN Bus values
    if (gear.source == 2) {
        uint8_t can_num_drivers = AP::can().get_num_drivers();
        for (uint8_t i = 0; i < can_num_drivers; i++) {
            if (AP::can().get_driver_type(i) == AP_CANManager::Driver_Type_PolarisCAN) {
                    AP_PolarisCAN *ap_plcan = AP_PolarisCAN::get_PolarisCAN(i);
                    if (ap_plcan == nullptr) {
                        continue;
                    }
                    gear.state = ap_plcan->get_current_gear();
            }
        }
    }

    // delay the gear change for user-defined duration. This helps ensure the vehicle is stopped before we attempt to change gears
    if (gear.pending.stop_vehicle_start_ms > 0) {
        if (now_ms - gear.pending.stop_vehicle_start_ms >= gear.pending.stop_duration*1000) {

            gear.pending.change_physical_gear_start_ms = now_ms;
            gear.pending.stop_vehicle_start_ms = 0;

            // we've waited to stop the vehicle, now set the gear and wait again for it to physically change
            gear.pwm_active = gear.pending.pwm;

            // only set gear state if we are not using CAN for gear status.
            if (gear.source == 1) {
                gear.state = gear.pending.state;
            }
            
            force_send_status = true;
        }

    } else if (gear.pending.change_physical_gear_start_ms > 0) {
        if (now_ms - gear.pending.change_physical_gear_start_ms >= gear.pending.change_duration_total_ms) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine gear is now %s", get_gear_name(gear.state));
            gear.pending.change_physical_gear_start_ms = 0;
            force_send_status = true;
        }

    } else if (auto_mode_active &&
            state == ICE_RUNNING &&
            (options & AP_ICENGINE_OPTIONS_MASK_AUTO_SETS_GEAR_FORWARD) &&
            !is_waiting_in_auto() &&
            !gear.is_forward() &&
            !gear.pending.is_active())
    {
        if (gear.auto_change_debounce == 0) {
            gear.auto_change_debounce = now_ms;
        } else if (now_ms - gear.auto_change_debounce >= 300) {
            set_ice_transmission_state(MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1, 0);
        }

    } else {
        gear.auto_change_debounce = 0;
    }
}
/*
  check for throttle override. This allows the ICE controller to force
  the correct starting throttle when starting the engine and maintain idle when disarmed or out of temp range
 */
bool AP_ICEngine::throttle_override(float &percentage)
{
    if (!enabled()) {
        return false;
    }

    const float idle_percent_local = get_idle_throttle();
    const float percentage_old = percentage;
    bool use_idle_percent = false;

    if (state == ICE_RUNNING &&
        idle_percent_local > 0 &&
        idle_percent_local < 100 &&
        (int16_t)idle_percent_local > SRV_Channels::get_output_scaled(SRV_Channel::k_throttle))
    {
        use_idle_percent = true;
    }  else if (state == ICE_STARTING ||
            state == ICE_START_DELAY ||
            too_cold() ||
            gear.pending.is_active() ||
            gear.auto_change_debounce) {
        use_idle_percent = true;
    } else if (too_hot()) {
        percentage *= constrain_float(temperature.too_hot_throttle_reduction_factor,0,1);
    }

    if (use_idle_percent) {
        // some of the above logic may have set it to zero but other logic says we're in a state that zero may kill the engine so use idle instead
        percentage = idle_percent_local;
    }

    if (is_equal(percentage_old, percentage)) {
        // no change on throttle
        return false;
    }

//    gcs().send_text(MAV_SEVERITY_INFO, "%d ICE throttle override %d to %d", AP_HAL::millis(), (int)percentage_old, (int)percentage);

    return true;
}


/*
  handle DO_ENGINE_CONTROL messages via MAVLink or mission
*/
bool AP_ICEngine::engine_control(float start_control, float cold_start, float height_delay, float gear_state_f)
{
    const MAV_ICE_TRANSMISSION_GEAR_STATE gear_state = (MAV_ICE_TRANSMISSION_GEAR_STATE)(int32_t)gear_state_f;

    if (options & AP_ICENGINE_OPTIONS_MASK_BLOCK_EXTERNAL_STARTER_CMDS) {
        gcs().send_text(MAV_SEVERITY_INFO, "Engine: external starter commands are blocked");
        return false;
    }

    if (!(auto_mode_active && (options & AP_ICENGINE_OPTIONS_MASK_AUTO_ALWAYS_AUTOSTART))) {
        // Allow RC input to block engine control commands if we're not in any autoNav mode and options flag says the autonav always autostarts
        RC_Channel *c = rc().channel(start_chan-1);
        if ((c != nullptr) && convertPwmToIgnitionState(c->get_radio_in()) == ICE_IGNITION_OFF) {
            gcs().send_text(MAV_SEVERITY_INFO, "Engine: start control disabled");
            return false;
        }
    }

#if !APM_BUILD_TYPE(APM_BUILD_Rover)
    if (height_delay > 0) {
        height_pending = true;
        initial_height = 0;
        height_required = height_delay;
        state = ICE_START_HEIGHT_DELAY;
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff height set to %.1fm", (double)height_delay);
    }
#endif

    if (is_equal(start_control, 0.0f)) {
        set_ignition_state(ICE_IGNITION_OFF);

    } else if (is_equal(start_control, 1.0f)) {
        set_ignition_state(ICE_IGNITION_ACCESSORY);

    } else if (is_equal(start_control, 2.0f)) {
        set_ignition_state(ICE_IGNITION_START_RUN);

    } else if (is_equal(start_control, 13.0f)) {
        recharge.pending_abort();
    }

    if (gear_state > 0 &&
            gear_state != MAV_ICE_TRANSMISSION_GEAR_STATE_UNKNOWN &&
            gear_state != MAV_ICE_TRANSMISSION_GEAR_STATE_PWM_VALUE &&
            gear_state_f < MAV_ICE_TRANSMISSION_GEAR_STATE_ENUM_END)
    {
        if (set_ice_transmission_state(gear_state, 0)) {
            force_send_status = true;
        }
    }

    return true;
}

const char* AP_ICEngine::get_ignition_state_name(ice_ignition_state_t state_name)
{
    switch (state_name) {
    case ICE_IGNITION_OFF:          return "OFF";
    case ICE_IGNITION_ACCESSORY:    return "ACCESSORY";
    case ICE_IGNITION_START_RUN:    return "START_RUN";
    default: return "UNKNOWN";
    }
}

bool AP_ICEngine::set_ignition_state(ice_ignition_state_t state_new)
{
    if (state_new == startControlSelect) {
        return false;
    }

    startControlSelect = state_new;
    gcs().send_text(MAV_SEVERITY_INFO, "Engine Control set to %s", get_ignition_state_name(state_new));
    force_send_status = true;

    return true;
}

bool AP_ICEngine::handle_message(const mavlink_command_long_t &packet)
{
    switch (packet.command) {
    case MAV_CMD_ICE_SET_TRANSMISSION_STATE:
        return handle_set_ice_transmission_state(packet);

    case MAV_CMD_ICE_TRANSMISSION_STATE:
    case MAV_CMD_ICE_FUEL_LEVEL:
    case MAV_CMD_ICE_COOLANT_TEMP:
        // unhandled, this is an outbound packet only
        return false;
    } // switch

    // unhandled
    return false;
}

int16_t AP_ICEngine::constrain_pwm_with_direction(const int16_t initial, const int16_t desired, const int16_t pwm_going_down, const int16_t pwm_going_up)
{
    if (initial == desired) {
        return initial;

    } else if (initial > desired) {
        return pwm_going_down;

    } else {
        return pwm_going_up;
    }
}

bool AP_ICEngine::handle_set_ice_transmission_state(const mavlink_command_long_t &packet)
{
//    //const uint8_t index = packet->param1; // unused
//    const MAV_ICE_TRANSMISSION_GEAR_STATE gearState = (MAV_ICE_TRANSMISSION_GEAR_STATE)packet.param2;
//    const uint16_t pwm_value = packet.param3;
//
//    if (set_ice_transmission_state(gearState, pwm_value)) {
//        brakeReleaseAllowedIn_Neutral_and_Disarmed = !is_zero(packet.param4);
//        return true;
//    }
    return false;
}

const char* AP_ICEngine::get_gear_name(const MAV_ICE_TRANSMISSION_GEAR_STATE gearState)
{
    switch (gearState) {
    case MAV_ICE_TRANSMISSION_GEAR_STATE_PARK: /* Park. | */
        return "Park";

    case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE: /* Reverse for single gear systems or Variable Transmissions. | */
    case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_1: /* Reverse 1. Implies multiple gears exist. | */
    case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_2:
    case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_3:
        return "Reverse";

    case MAV_ICE_TRANSMISSION_GEAR_STATE_NEUTRAL: /* Neutral. Engine is physically disconnected. | */
        return "Neutral";

    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD: /* Forward for single gear systems or Variable Transmissions. | */
    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1: /* First gear. Implies multiple gears exist. | */
        return "Forward";

    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_2: /* Second gear. | */
    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_3:
    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_4:
    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_5:
    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_6:
    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_7:
    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_8:
    case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_9:
        return "Forward High";

    case MAV_ICE_TRANSMISSION_GEAR_STATE_PWM_VALUE:
    default:
        return "Unknown";
    }
}

bool AP_ICEngine::set_ice_transmission_state(MAV_ICE_TRANSMISSION_GEAR_STATE gearState, const uint16_t pwm_value)
{
    uint16_t pendingPwm = 0;

    switch (gearState) {
        case MAV_ICE_TRANSMISSION_GEAR_STATE_PARK: /* Park. | */
            pendingPwm = constrain_pwm_with_direction(gear.pwm_active, (gear.pwm_park_down+gear.pwm_park_up)/2, gear.pwm_park_down, gear.pwm_park_up);
            break;

        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE: /* Reverse for single gear systems or Variable Transmissions. | */
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_1: /* Reverse 1. Implies multiple gears exist. | */
            gearState = MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_1;
            pendingPwm = constrain_pwm_with_direction(gear.pwm_active, (gear.pwm_reverse_down+gear.pwm_reverse_up)/2, gear.pwm_reverse_down, gear.pwm_reverse_up);
            break;

        case MAV_ICE_TRANSMISSION_GEAR_STATE_NEUTRAL: /* Neutral. Engine is physically disconnected. | */
            pendingPwm = constrain_pwm_with_direction(gear.pwm_active, (gear.pwm_neutral_down+gear.pwm_neutral_up)/2, gear.pwm_neutral_down, gear.pwm_neutral_up);
            break;

        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD: /* Forward for single gear systems or Variable Transmissions. | */
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1: /* First gear. Implies multiple gears exist. | */
            gearState = MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1;
            pendingPwm = constrain_pwm_with_direction(gear.pwm_active, (gear.pwm_forward1_down+gear.pwm_forward1_up)/2, gear.pwm_forward1_down, gear.pwm_forward1_up);
            break;

        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_2: /* Second gear. | */
            pendingPwm = constrain_pwm_with_direction(gear.pwm_active, (gear.pwm_forward2_down+gear.pwm_forward2_up)/2, gear.pwm_forward2_down, gear.pwm_forward2_up);
            break;

        case MAV_ICE_TRANSMISSION_GEAR_STATE_PWM_VALUE:
            // use as-is
            pendingPwm = pwm_value;
            break;

        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_3:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_4:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_5:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_6:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_7:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_8:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_9:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_2:
        case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_3:
        default:
            // unhandled
            return false;
    }

    gear.auto_change_debounce = 0;

    if (gearState != MAV_ICE_TRANSMISSION_GEAR_STATE_PWM_VALUE && (gear.state == gearState || (gear.pending.is_active() && gear.pending.state == gearState))) {
        // always handle PWM,
        // don't handle if (requested == current)
        // don't handle if it's changing and (requested == pending)
        return true;
    }

    gear.pending.state = gearState;
    gear.pending.pwm = pendingPwm;
    uint32_t total_steps;

    if (!gear.pending.is_active()) {
        // normal scenario
        const int8_t gear_pos_current = Gear_t::get_position(gear.state);
        const int8_t gear_pos_pending = Gear_t::get_position(gear.pending.state);
        total_steps = MAX(1,abs(gear_pos_current - gear_pos_pending));

    } else {
        // this is where we're trying to change to a new gear, while already in the middle of
        // changing to a different gear. Hard to know where we are, so let's be suuuper conservative
        total_steps = Gear_t::get_position_max();
    }

    gear.pending.change_duration_total_ms = gear.pending.change_duration_per_posiiton * 1000 * total_steps;
    gear.pending.stop_vehicle_start_ms = AP_HAL::millis();
    force_send_status = true;

    gcs().send_text(MAV_SEVERITY_INFO, "Engine gear change: %s to %s in %.1fs",
            get_gear_name(gear.state), get_gear_name(gear.pending.state),
            (double)(gear.pending.change_duration_total_ms*0.001f));

    return true;
}

/*
  Update low throttle limit to ensure steady idle for IC Engines
  return a new min_throttle value
*/
void AP_ICEngine::update_idle_governor(int8_t &min_throttle)
{
    if (!enable) {
        return;
    }
    const int8_t min_throttle_base = min_throttle;

    // Initialize idle point to min_throttle on the first run
    static bool idle_point_initialized = false;
    if (!idle_point_initialized) {
        idle_governor_integrator = min_throttle;
        idle_point_initialized = true;
    }
    AP_RPM *ap_rpm = AP::rpm();
    if (!ap_rpm || rpm_instance == 0 || !ap_rpm->healthy(rpm_instance-1)) {
        return;
    }

    // Check to make sure we have an enabled IC Engine, EFI Instance and that the idle governor is enabled
    if (get_state() != AP_ICEngine::ICE_RUNNING || idle_rpm < 0) {
        return;
    }

    // get current RPM feedback
    float rpmv;

    // Double Check to make sure engine is really running
    if (!ap_rpm->get_rpm(rpm_instance-1, rpmv) || rpmv < 1) {
        // Reset idle point to the default value when the engine is stopped
        idle_governor_integrator = min_throttle;
        return;
    }

    // Override
    min_throttle = roundf(idle_governor_integrator);

    // Caclulate Error in system
    int32_t error = idle_rpm - rpmv;

    bool underspeed = error > 0;

    // Don't adjust idle point when we're within the deadband
    if (abs(error) < idle_db) {
        return;
    }

    // Don't adjust idle point if the commanded throttle is above the
    // current idle throttle setpoint and the RPM is above the idle
    // RPM setpoint (Normal flight)
    if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > min_throttle && !underspeed) {
        return;
    }

    // Calculate the change per loop to acheieve the desired slew rate of 1 percent per second
    static const float idle_setpoint_step = idle_slew * AP::scheduler().get_loop_period_s();

    // Update Integrator 
    if (underspeed) {
        idle_governor_integrator += idle_setpoint_step;
    } else {
        idle_governor_integrator -= idle_setpoint_step;
    }

    idle_governor_integrator = constrain_float(idle_governor_integrator, min_throttle_base, 40.0f);

    min_throttle = roundf(idle_governor_integrator);
}

int8_t AP_ICEngine::Gear_t::get_position(const MAV_ICE_TRANSMISSION_GEAR_STATE gearState)
{
     switch (gearState) {
         case MAV_ICE_TRANSMISSION_GEAR_STATE_PARK: /* Park. | */
             return 1;

         case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE: /* Reverse for single gear systems or Variable Transmissions. | */
         case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_1: /* Reverse 1. Implies multiple gears exist. | */
         case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_2:
         case MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_3:
             return 2;

         case MAV_ICE_TRANSMISSION_GEAR_STATE_NEUTRAL: /* Neutral. Engine is physically disconnected. | */
             return 3;

         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD: /* Forward for single gear systems or Variable Transmissions. | */
         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1: /* First gear. Implies multiple gears exist. | */
             return 4;

         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_2: /* Second gear. | */
         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_3:
         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_4:
         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_5:
         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_6:
         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_7:
         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_8:
         case MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_9:
             return 5;

         case MAV_ICE_TRANSMISSION_GEAR_STATE_PWM_VALUE:
         default:
             return 0;
     }
 }

void AP_ICEngine::update_fuel()
{
    #define AP_ICENGINE_FUEL_LEVEL_BATTERY_INSTANCE 1
    // Fuel Level disabled
    if (fuel.source <= 0) {
        fuel.value = AP_ICENGINE_FUEL_LEVEL_INVALID;
        fuel.last_sample_ms = 0;
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // Analog Fuel Level readings
    if (fuel.source == 1) {
        if (!AP::battery().healthy(AP_ICENGINE_FUEL_LEVEL_BATTERY_INSTANCE)) {
            fuel.value = AP_ICENGINE_FUEL_LEVEL_INVALID;
            return;
        }

        const float new_value = AP::battery().capacity_remaining_pct(AP_ICENGINE_FUEL_LEVEL_BATTERY_INSTANCE);

        if (fuel.last_sample_ms == 0 || (fuel.last_sample_ms - now_ms > 5000)) {
            // jump to it immediately on first or stale
            fuel.value = new_value;
        }
        // Low Pass filter, very slow
        fuel.value = 0.1f*fuel.value + 0.9f*new_value;
        fuel.last_sample_ms = now_ms;
    }

    // Polaris CAN Bus Fuel Level reading
    if (fuel.source == 2) {
        uint8_t can_num_drivers = AP::can().get_num_drivers();
        for (uint8_t i = 0; i < can_num_drivers; i++) {
            if (AP::can().get_driver_type(i) == AP_CANManager::Driver_Type_PolarisCAN) {
                    AP_PolarisCAN *ap_plcan = AP_PolarisCAN::get_PolarisCAN(i);
                    if (ap_plcan == nullptr) {
                        continue;
                    }
                    fuel.value = (float)ap_plcan->get_fuel_level();
                    fuel.last_sample_ms = now_ms;
            }
        }
    }
}

void AP_ICEngine::update_temperature()
{
    if (temperature.analog_source == nullptr) {
        temperature.analog_source = hal.analogin->channel(temperature.pin);
        return;
    }

    if ((temperature.pin <= 0) && (temperature.source != 2)){
        // disabled
        temperature.value = 0;
        temperature.last_sample_ms = 0;
        return;
    }

    // CAN source selected
    if (temperature.source == 2) {
        const uint32_t now_ms = AP_HAL::millis();
        uint8_t can_num_drivers = AP::can().get_num_drivers();

        for (uint8_t i = 0; i < can_num_drivers; i++) {
            if (AP::can().get_driver_type(i) == AP_CANManager::Driver_Type_PolarisCAN) {
                    AP_PolarisCAN *ap_plcan = AP_PolarisCAN::get_PolarisCAN(i);
                    if (ap_plcan == nullptr) {
                        continue;
                    }
                    temperature.value = (float)ap_plcan->get_coolant_temp();
                    temperature.last_sample_ms = now_ms;
            }
        }
        return;
    }

    temperature.analog_source->set_pin(temperature.pin);

    float v, new_temp_value = 0;
    if (temperature.ratiometric) {
        v = temperature.analog_source->voltage_average_ratiometric();
    } else {
        v = temperature.analog_source->voltage_average();
    }

    switch ((AP_ICEngine::Temperature_Function)temperature.function.get()) {
    case Temperature_Function::FUNCTION_LINEAR:
        new_temp_value = (v - temperature.offset) * temperature.scaler;
        break;

    case Temperature_Function::FUNCTION_INVERTED:
        new_temp_value = (temperature.offset - v) * temperature.scaler;
        break;

    case Temperature_Function::FUNCTION_HYPERBOLA:
        if (is_zero(v - temperature.offset)) {
            // do not average in an invalid sample
            return;
        }
        new_temp_value = temperature.scaler / (v - temperature.offset);
        break;

    default:
        // do not average in an invalid sample
        return;
    }

    if (!isinf(new_temp_value)) {
        const uint32_t now_ms = AP_HAL::millis();
        if (temperature.last_sample_ms == 0 || (temperature.last_sample_ms - now_ms > 5000)) {
            // jump to it immediately on first or stale
            temperature.value = new_temp_value;
        }
        // Low Pass filter, very slow
        temperature.value = 0.1f*temperature.value + 0.9f*new_temp_value;
        temperature.last_sample_ms = now_ms;
    }
}

bool AP_ICEngine::get_temperature(float& value) const
{
    if (!temperature.is_healthy()) {
        return false;
    }
    value = temperature.value;
    return true;
}

void AP_ICEngine::send_status()
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool force = force_send_status;
    force_send_status = false;

    bool temp_sent = false, fuel_sent = false, gear_sent = false, awd_sent = false, enghrs_sent = false, engdtc_sent = false, clstrdtc_sent = false, strdtc_sent = false;

    const uint8_t chan_mask = GCS_MAVLINK::active_channel_mask();
    for (uint8_t chan=0; chan<MAVLINK_COMM_NUM_BUFFERS; chan++) {
        if ((chan_mask & (1U<<chan)) == 0) {
            // not active
            continue;
        }
        
        const bool send_temp = force || (now_ms - temperature.last_send_ms >= 1000);
        if (send_temp && HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, COMMAND_LONG)) {

            temp_sent = true;
            const float current_temp = temperature.is_healthy() ? temperature.value : AP_ICENGINE_TEMPERATURE_INVALID;

            mavlink_msg_command_long_send(
                    (mavlink_channel_t)chan, 0, 0,
                    MAV_CMD_ICE_COOLANT_TEMP,
                    0, // confirmation is unused
                    0, // index
                    current_temp,
                    temperature.max,    // too hot
                    temperature.min,    // too cold
                    0,0,0);
            // static uint32_t sendtext_timeref;
            // gcs().send_text_rate_limited(MAV_SEVERITY_INFO, 500, sendtext_timeref,"AP_ICE Temp: %0.2f", current_temp);
        }

        

        uint16_t current_gear_pwm = ICE_GEAR_STATE_PWM_INVALID;

        bool hasGear = false;
        if (gear.source == 1) {
            hasGear = SRV_Channels::get_output_pwm(SRV_Channel::k_engine_gear, current_gear_pwm);
        }
        if (gear.source == 2) {
            hasGear = true;
        }
        
        const bool send_gear = force || (now_ms - gear.last_send_ms >= 1000);
        if (hasGear && send_gear && HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, COMMAND_LONG)) {

            gear_sent = true;

            mavlink_msg_command_long_send(
                    (mavlink_channel_t)chan, 0, 0,
                    MAV_CMD_ICE_TRANSMISSION_STATE,
                    0, // confirmation is unused
                    0,                              // param1 = index
                    gear.pending.is_active() ? gear.pending.state : gear.state, // param2 = if pending, show what start we'll end up in
                    current_gear_pwm,               // param3
                    startControlSelect,             // param4
                    gear.pending.is_active(),       // param5
                    0,                              // param6 UNUSED
                    0);                             // param7 UNUSED
        }

        const bool send_fuel = force || (now_ms - fuel.last_send_ms >= 1000);
        if (send_fuel && HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, COMMAND_LONG)) {

            fuel_sent = true;
            float current_fuel = AP_ICENGINE_FUEL_LEVEL_INVALID;
            if (fuel.source == 1) {
                current_fuel = AP::battery().healthy() ? fuel.value : AP_ICENGINE_FUEL_LEVEL_INVALID;
            }

            if (fuel.source ==2) {
                current_fuel = fuel.value;
            }

            mavlink_msg_command_long_send(
                    (mavlink_channel_t)chan, 0, 0,
                    MAV_CMD_ICE_FUEL_LEVEL,
                    0, // confirmation is unused
                    0, // index
                    MAV_ICE_FUEL_TYPE_GASOLINE,
                    MAV_ICE_FUEL_LEVEL_UNITS_PERCENT,
                    100, // max
                    current_fuel,
                    0,0);
        }

        uint8_t can_num_drivers = AP::can().get_num_drivers();
        for (uint8_t i = 0; i < can_num_drivers; i++) {
            if (AP::can().get_driver_type(i) == AP_CANManager::Driver_Type_PolarisCAN) {
                    AP_PolarisCAN *ap_plcan = AP_PolarisCAN::get_PolarisCAN(i);
                    if (ap_plcan == nullptr) {
                        continue;
                    }
                    can_data.awd.fwd_status = ap_plcan->get_FWD_status();
                    can_data.awd.adc_status = ap_plcan->get_ADC_status();
                    can_data.engine_hours.engine_hours = ap_plcan->get_engine_hours();
                    can_data.odometer.fOdometer = ap_plcan->get_odometer();
                    can_data.odometer.fTripDistance = ap_plcan->get_tripDistance();
                    can_data.engine_dtc.dtcdata = ap_plcan->get_engine_dtc();
                    can_data.cluster_dtc.dtcdata = ap_plcan->get_cluster_dtc();
                    can_data.steering_dtc.dtcdata = ap_plcan->get_steering_dtc();
            }
        }

        const bool send_AWD = force || (now_ms - can_data.awd.last_send_ms >= 1000);
        if (send_AWD && HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, COMMAND_LONG)) {

            awd_sent = true;
            mavlink_msg_command_long_send(
                    (mavlink_channel_t)chan, 0, 0,
                    MAV_CMD_ICE_4WD_STATE,
                    0, // confirmation is unused
                    can_data.awd.fwd_status, // index
                    can_data.awd.adc_status,
                    0, 0, 0, 0,0);
        }

        const bool send_enghrs = force || (now_ms - can_data.engine_hours.last_send_ms >= 1000);
        if (send_enghrs && HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, COMMAND_LONG)) {

            enghrs_sent = true;
            mavlink_msg_command_long_send(
                    (mavlink_channel_t)chan, 0, 0,
                    MAV_CMD_ICE_ENGINE_HOURS,
                    0, // confirmation is unused
                    can_data.engine_hours.engine_hours, // index
                    can_data.odometer.fOdometer, // Odometer Place holder
                    can_data.odometer.fTripDistance, // Trip distance Place holder
                    0, 0, 0,0);
        }

        
        const bool send_engdtc = force || (now_ms - can_data.engine_dtc.last_send_ms >= 1000);
        if (send_engdtc && HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, COMMAND_LONG)) {
            engdtc_sent = true;
            mavlink_msg_command_long_send(
                    (mavlink_channel_t)chan, 0, 0,
                    MAV_CMD_ICE_ENGINE_DTC,
                    0, // confirmation is unused
                    can_data.engine_dtc.dtcdata.index, // index
                    can_data.engine_dtc.dtcdata.u16LightStatus, // Check Engine Light Status
                    can_data.engine_dtc.dtcdata.u32SPN, // Trip distance Place holder
                    can_data.engine_dtc.dtcdata.u8FMI,
                    can_data.engine_dtc.dtcdata.u8OC,
                    0, 0);
        }

        const bool send_clusterdtc = force || (now_ms - can_data.cluster_dtc.last_send_ms >= 1000);
        if (send_clusterdtc && HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, COMMAND_LONG)) {
            clstrdtc_sent = true;
            mavlink_msg_command_long_send(
                    (mavlink_channel_t)chan, 0, 0,
                    MAV_CMD_ICE_CLUSTER_DTC,
                    0, // confirmation is unused
                    can_data.cluster_dtc.dtcdata.index, // index
                    can_data.cluster_dtc.dtcdata.u16LightStatus, // Check Engine Light Status
                    can_data.cluster_dtc.dtcdata.u32SPN, // Trip distance Place holder
                    can_data.cluster_dtc.dtcdata.u8FMI,
                    can_data.cluster_dtc.dtcdata.u8OC,
                    0, 0);
        }

        const bool send_strdtc = force || (now_ms - can_data.steering_dtc.last_send_ms >= 1000);
        if (send_strdtc && HAVE_PAYLOAD_SPACE((mavlink_channel_t)chan, COMMAND_LONG)) {
            strdtc_sent = true;
            mavlink_msg_command_long_send(
                    (mavlink_channel_t)chan, 0, 0,
                    MAV_CMD_ICE_STEERING_DTC,
                    0, // confirmation is unused
                    can_data.steering_dtc.dtcdata.index, // index
                    can_data.steering_dtc.dtcdata.u16LightStatus, // Check Engine Light Status
                    can_data.steering_dtc.dtcdata.u32SPN, // Trip distance Place holder
                    can_data.steering_dtc.dtcdata.u8FMI,
                    can_data.steering_dtc.dtcdata.u8OC,
                    0, 0);
        }
    } // for


    if (temp_sent) {
        temperature.last_send_ms = now_ms;
    }
    if (gear_sent) {
        gear.last_send_ms = now_ms;
    }
    if (fuel_sent) {
        fuel.last_send_ms = now_ms;
    }
    if (awd_sent) {
        can_data.awd.last_send_ms = now_ms;
    }
    if (enghrs_sent) {
        can_data.engine_hours.last_send_ms = now_ms;
    }
    if (engdtc_sent) {
        can_data.engine_dtc.last_send_ms = now_ms;
    }
    if (clstrdtc_sent) {
        can_data.cluster_dtc.last_send_ms = now_ms;
    }
    if (strdtc_sent) {
        can_data.steering_dtc.last_send_ms = now_ms;
    }
}


MAV_ICE_TRANSMISSION_GEAR_STATE AP_ICEngine::convertPwmToGearState(const uint16_t pwm)
{
    const uint16_t margin = 20;

    if (gear.pwm_forward2_down <= gear.pwm_forward2_up && pwm <= gear.pwm_forward2_up + margin && pwm >= gear.pwm_forward2_down - margin) {
        return MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_2;
    } else if (gear.pwm_forward2_down > gear.pwm_forward2_up && pwm <= gear.pwm_forward2_down + margin && pwm >= gear.pwm_forward2_up - margin) {
        return MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_2;
    }

    else if (gear.pwm_forward1_down <= gear.pwm_forward1_up && pwm <= gear.pwm_forward1_up + margin && pwm >= gear.pwm_forward1_down - margin) {
        return MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1;
    } else if (gear.pwm_forward1_down > gear.pwm_forward1_up && pwm <= gear.pwm_forward1_down + margin && pwm >= gear.pwm_forward1_up - margin) {
        return MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1;
    }


    else if (gear.pwm_neutral_down <= gear.pwm_neutral_up && pwm <= gear.pwm_neutral_up + margin && pwm >= gear.pwm_neutral_down - margin) {
        return MAV_ICE_TRANSMISSION_GEAR_STATE_NEUTRAL;
    } else if (gear.pwm_neutral_down > gear.pwm_neutral_up && pwm <= gear.pwm_neutral_down + margin && pwm >= gear.pwm_neutral_up - margin) {
        return MAV_ICE_TRANSMISSION_GEAR_STATE_NEUTRAL;
    }

    else if (gear.pwm_reverse_down <= gear.pwm_reverse_up && pwm <= gear.pwm_reverse_up + margin && pwm >= gear.pwm_reverse_down - margin) {
        return MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_1;
    } else if (gear.pwm_reverse_down > gear.pwm_reverse_up && pwm <= gear.pwm_reverse_down + margin && pwm >= gear.pwm_reverse_up - margin) {
        return MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_1;
    }

    else {
        return MAV_ICE_TRANSMISSION_GEAR_STATE_PARK;
    }
}

float AP_ICEngine::get_idle_throttle()
{
    if (!enabled()) {
        return 0;
    }

    float idle = idle_percent;
    if (recharge.elapsed_time() > 0) {
        idle = MAX(idle, recharge.throttle);
    }
    return constrain_float(idle, 0, 100);
}

void AP_ICEngine::mode_change_or_new_autoNav_point_event(bool modeIsAnyAutoNav)
{
    auto_mode_active = modeIsAnyAutoNav;
    if (auto_mode_active && options & AP_ICENGINE_OPTIONS_MASK_AUTO_ALWAYS_AUTOSTART) {
        set_ignition_state(ICE_IGNITION_START_RUN);
    }
}

// singleton instance. Should only ever be set in the constructor.
AP_ICEngine *AP_ICEngine::_singleton;
namespace AP {
    AP_ICEngine *ice() {
        return AP_ICEngine::get_singleton();
    }
}
