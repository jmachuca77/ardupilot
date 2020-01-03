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

// 0000 0000 0000 0101 == 6
// 0000 0000 0010 1101 == 45
// 0000 0000 1010 1101 == 173
// 0000 0000 1010 1100 == 172

#define AP_ICENGINE_OPTIONS_MASK_RESERVED1                      (1<<0) // no
#define AP_ICENGINE_OPTIONS_MASK_ARMING_REQUIRED_START          (1<<1) // no
#define AP_ICENGINE_OPTIONS_MASK_KEEP_RUNNING_WHEN_DISARMED     (1<<2) // yes
#define AP_ICENGINE_OPTIONS_MASK_AUTO_ALWAYS_AUTOSTART          (1<<3) // yes

#define AP_ICENGINE_OPTIONS_MASK_BLOCK_EXTERNAL_STARTER_CMDS    (1<<4) // no NOTE: This blocks both external mavlink msgs and "internal" auto mission cmds
#define AP_ICENGINE_OPTIONS_MASK_AUTO_SETS_GEAR_FORWARD         (1<<5) // yes
#define AP_ICENGINE_OPTIONS_MASK_RUNNING_FAIL_FORCE_STOP_MOTOR  (1<<6) // no
#define AP_ICENGINE_OPTIONS_MASK_RPM_FAIL_HAS_TIMER             (1<<7) // yes

#define AP_ICENGINE_OPTIONS_MASK_ARMING_REQUIRED_IGNITION       (1<<8) // no


#define AP_ICENGINE_OPTIONS_MASK_DEFAULT                        ( AP_ICENGINE_OPTIONS_MASK_KEEP_RUNNING_WHEN_DISARMED       \
                                                                | AP_ICENGINE_OPTIONS_MASK_AUTO_ALWAYS_AUTOSTART            \
                                                                | AP_ICENGINE_OPTIONS_MASK_AUTO_SETS_GEAR_FORWARD           \
                                                                | AP_ICENGINE_OPTIONS_MASK_RPM_FAIL_HAS_TIMER               \
                                                                )


class AP_ICEngine {
public:
    // constructor
    AP_ICEngine();

    /* Do not allow copies */
    AP_ICEngine(const AP_ICEngine &other) = delete;
    AP_ICEngine &operator=(const AP_ICEngine&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // update engine state. Should be called at 10Hz or more
    void update(void);

    void init(const bool force_outut);

    // check for  override
    bool throttle_override(float &percent);

    // check for brake override
    bool brake_override(float &brake_percent, const float desired_speed, const bool speed_is_valid, const float speed);

    enum ICE_State {
        ICE_OFF=0,
        ICE_START_HEIGHT_DELAY=1,
        ICE_START_DELAY_NO_IGNITION=2,
        ICE_START_DELAY=3,
        ICE_STARTING=4,
        ICE_RUNNING=5
    };

    typedef enum {
        ICE_IGNITION_OFF,
        ICE_IGNITION_ACCESSORY,
        ICE_IGNITION_START_RUN
    } ice_ignition_state_t;

    ice_ignition_state_t startControlSelect;
    const char* get_ignition_state_name(ice_ignition_state_t state);
    const char* get_ignition_state_name() { return get_ignition_state_name(startControlSelect); }
    bool set_ignition_state(ice_ignition_state_t state_new);
    ice_ignition_state_t get_ignition_state() { return startControlSelect; }

    static ice_ignition_state_t convertPwmToIgnitionState(const uint16_t pwm);

    void mode_change_or_new_autoNav_point_event(bool modeIsAnyAutoNav);

    // get current engine control state
    ICE_State get_state(void) const { return state; }

    // handle DO_ENGINE_CONTROL messages via MAVLink or mission
    bool engine_control(float start_control, float cold_start, float height_delay, float gear_state_f);

    // update min throttle for idle governor
    void update_idle_governor(int8_t &min_throttle);
    
    bool handle_message(const mavlink_command_long_t &packt);
    bool handle_set_ice_transmission_state(const mavlink_command_long_t &packet);
    bool set_ice_transmission_state(MAV_ICE_TRANSMISSION_GEAR_STATE gearState, const uint16_t pwm_value);
    static int16_t constrain_pwm_with_direction(const int16_t initial, const int16_t desired, const int16_t pwm_going_down, const int16_t pwm_going_up);

    // Engine temperature status
    bool get_temperature(float& value) const;
    bool too_hot() const { return temperature.is_healthy() && temperature.too_hot(); }
    bool too_cold() const { return temperature.is_healthy() && temperature.too_cold(); }
    void send_status();

    void set_current_throttle(const float throttle) { current_throttle_percent = throttle; }

    bool enabled() const { return (enable != 0); }
    int8_t get_channel_starter() const { return start_chan; }

    static AP_ICEngine *get_singleton() { return _singleton; }

    MAV_ICE_TRANSMISSION_GEAR_STATE get_transmission_gear_state() const { return gear.state; }

    bool is_changing_gears() { return has_gears() && gear.pending.is_active(); }
    bool has_gears() { return gear.is_configured(); }
    bool gear_is_park() { return has_gears() && gear.is_park(); }
    bool gear_is_forward() { return has_gears() && gear.is_forward(); }
    bool gear_is_reverse() { return has_gears() && gear.is_reverse(); }
    bool gear_is_neutral() { return has_gears() && gear.is_neutral(); }
    void set_is_waiting_in_auto(bool value) { vehicle_is_waiting_in_auto = value; }
    bool is_waiting_in_auto() { return vehicle_is_waiting_in_auto && auto_mode_active; }
    bool gear_is_inhibiting_locomotion() { return enabled() && has_gears() && (gear.pending.is_active() || gear.is_park() || gear.is_neutral()); }
    float get_idle_throttle();

private:
    static AP_ICEngine *_singleton;

    enum ICE_State state;
    enum ICE_State state_prev;
    uint32_t state_change_timestamp_ms;
    uint32_t force_staying_in_DELAY_NO_IGNITION_duration_ms;

    struct Gear_t {
        bool is_forward() { return Gear_t::is_forward(state); }
        bool is_reverse() { return Gear_t::is_reverse(state); }
        bool is_neutral() { return Gear_t::is_neutral(state); }
        bool is_park()    { return Gear_t::is_park(state); }
        bool is_configured() { return (state != MAV_ICE_TRANSMISSION_GEAR_STATE_UNKNOWN); }

        static bool is_forward(const MAV_ICE_TRANSMISSION_GEAR_STATE gearState) {
            return (gearState == MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD ||
                (gearState >= MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_1 && gearState <= MAV_ICE_TRANSMISSION_GEAR_STATE_FORWARD_9));
        }
        static bool is_reverse(const MAV_ICE_TRANSMISSION_GEAR_STATE gearState) {
            return (gearState == MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE ||
                (gearState >= MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_1 && gearState <= MAV_ICE_TRANSMISSION_GEAR_STATE_REVERSE_3));
        }
        static bool is_neutral(const MAV_ICE_TRANSMISSION_GEAR_STATE gearState) { return (gearState == MAV_ICE_TRANSMISSION_GEAR_STATE_NEUTRAL); }
        static bool is_park(const MAV_ICE_TRANSMISSION_GEAR_STATE gearState) { return (gearState == MAV_ICE_TRANSMISSION_GEAR_STATE_PARK); }

        static int8_t get_position(const MAV_ICE_TRANSMISSION_GEAR_STATE gearState);
        static int8_t get_position_max() { return 6; }

        struct pending_t {
            void cancel()       { stop_vehicle_start_ms = 0; change_physical_gear_start_ms = 0; }
            bool is_active()    { return (stop_vehicle_start_ms > 0 || change_physical_gear_start_ms > 0); }
            bool is_forward()   { return Gear_t::is_forward(state); }
            bool is_reverse()   { return Gear_t::is_reverse(state); }
            bool is_neutral()   { return Gear_t::is_neutral(state); }
            bool is_park()      { return Gear_t::is_park(state); }

            uint16_t pwm;
            enum MAV_ICE_TRANSMISSION_GEAR_STATE state = MAV_ICE_TRANSMISSION_GEAR_STATE_UNKNOWN;

            // start time of when pending changed, waiting while throttle is zero until "wait_to_stop" expires
            uint32_t stop_vehicle_start_ms;
            // duration in seconds to inhibit throttle while we wait for vehicle to stop before we start changing to pwm_active
            AP_Float stop_duration;

            uint32_t change_duration_total_ms;

            // start time of when pending ended and pwm_active was just set, inhibit throttle while waiting for gear to physically change until "wait_to_change_physical" expires
            uint32_t change_physical_gear_start_ms;
            // duration in seconds to inhibit throttle while gear is changing from pending.pwm to pwm_active
            AP_Float change_duration_per_posiiton;
        } pending;

        enum MAV_ICE_TRANSMISSION_GEAR_STATE state = MAV_ICE_TRANSMISSION_GEAR_STATE_UNKNOWN;
        uint16_t pwm_active;

        uint32_t auto_change_debounce;

        uint32_t last_send_ms;
        AP_Int16 pwm_park_up;
        AP_Int16 pwm_park_down;
        AP_Int16 pwm_reverse_up;
        AP_Int16 pwm_reverse_down;
        AP_Int16 pwm_neutral_up;
        AP_Int16 pwm_neutral_down;
        AP_Int16 pwm_forward1_up;
        AP_Int16 pwm_forward1_down;
        AP_Int16 pwm_forward2_up;
        AP_Int16 pwm_forward2_down;
    } gear;

    MAV_ICE_TRANSMISSION_GEAR_STATE convertPwmToGearState(const uint16_t pwm);

    void update_fuel();
    struct {
        AP_Float offset;
        float value;
        uint32_t last_sample_ms;
        uint32_t last_send_ms;
    } fuel;

    // engine temperature for feedback
    struct {
        AP_Int8 pin;
        int8_t pin_prev; // check for changes at runtime
        AP_Float scaler;
        AP_Int16 min;
        AP_Int16 max;
        AP_Int8 ratiometric;
        AP_Float offset;
        AP_Int8 function;
        AP_Float too_hot_throttle_reduction_factor;

        AP_HAL::AnalogSource *source;
        float value;
        uint32_t last_sample_ms;
        uint32_t last_send_ms;

        bool is_healthy() const { return (pin > 0 && last_sample_ms && (AP_HAL::millis() - last_sample_ms < 1000)); }
        bool too_hot() const {  return max != 0 && (min < max) && (value > max); } // note, min == max will return false.
        bool too_cold() const { return min != 0 && (min < max) && (value < min); } // note, min == max will return false.
    } temperature;

    enum Temperature_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    struct Recharge {
        public:
        enum Recharge_State {
            ICE_RECHARGE_STATE_OFF,
            ICE_RECHARGE_STATE_CHECKING_BATTERY,
            ICE_RECHARGE_STATE_CHARGING_PENDING,
            ICE_RECHARGE_STATE_CHARGING,
            ICE_RECHARGE_STATE_SNOOZING,
        };

        const uint32_t snooze_duration_ms = 1 * 60 * 1000; // 1 minute
        const float minimum_voltage = 3.0f;
        const char *msg = "Engine Self-Charge: ";

        uint32_t timer_ms;
        uint32_t notify_ms;
        float battery_voltage_last;
        AP_Float voltage_threshold;
        AP_Int32 duration_seconds;
        AP_Int8 battery_instance;
        AP_Float throttle;
        Recharge_State state;

        uint32_t elapsed_time() const { return is_active() ? (AP_HAL::millis() - timer_ms) : 0; }
        bool is_active() const { return (state == ICE_RECHARGE_STATE_CHARGING); }
        float get_smoothed_battery_voltage();
        void set_state(Recharge_State next_state);
        void pending_abort();
    };
    Recharge recharge;

    void update_self_charging();

    void update_temperature();

    void update_gear();
    const char* get_gear_name(const MAV_ICE_TRANSMISSION_GEAR_STATE gearState);

    void set_output_channels();

    void set_srv_or_relay(int8_t pin, SRV_Channel::Aux_servo_function_t srv_func, int8_t desired_value);

    void determine_state();

    void set_outputs_off() const;

    AP_Int8 resarts_allowed;

    // enable library
    AP_Int8 enable;

    // channel for pilot to command engine start, 0 for none
    AP_Int8 start_chan;

    // which RPM instance to use
    AP_Int8 rpm_instance;
    
    // time to run starter for (seconds)
    AP_Float starter_time;

    // delay between start attempts (seconds)
    AP_Float starter_delay;
    
    // RPM above which engine is considered to be running
    AP_Int32 rpm_threshold_running;
    
    // RPM above which engine is considered to be running and remaining starting time should be skipped
    AP_Int32 rpm_threshold_starting;

    // time when we started the starter
    uint32_t starter_start_time_ms;

    // time when we last ran the starter
    uint32_t starter_last_run_ms;

    // throttle percentage for engine start
    AP_Int8 start_percent;

    // throttle percentage for engine idle
    AP_Int8 idle_percent;

    // Idle Controller RPM setpoint
    AP_Int16 idle_rpm;

    // Idle Controller RPM deadband
    AP_Int16 idle_db;

    // Idle Controller Slew Rate
    AP_Float idle_slew;
    
    AP_Int8 neutral_brake_enable;

    AP_Int8 ignition_pin;
    AP_Int8 starter_pin;

    // Time to wait after applying acceessory before applying starter
    AP_Int16 power_up_time;
    uint32_t engine_power_up_wait_ms;

#if !APM_BUILD_TYPE(APM_BUILD_APMrover2)
    // height when we enter ICE_START_HEIGHT_DELAY
    float initial_height;

    // height change required to start engine
    float height_required;

    // we are waiting for valid height data
    bool height_pending:1;
#endif

    // idle governor
    float idle_governor_integrator;

    enum class Options : uint16_t {
        DISABLE_IGNITION_RC_FAILSAFE=(1U<<0),
    };
    AP_Int16 options;

    // timestamp for periodic gcs msg regarding throttle_override
    uint32_t throttle_overrde_msg_last_ms;

    // store the previous throttle
    int8_t throttle_prev;

    // as reported from motor library from -100 to 100%
    float current_throttle_percent;

    // keep track of how many times we attempted to start. This will get conmpared to resarts_allowed
    uint8_t starting_attempts;

    // to know if we're running for the first time
    bool run_once;

    // force sending status over malvink, bypassing timers. This allows for a snappy response when something gets updated
    bool force_send_status;

    AP_Int8 master_output_enable_pin;

    // if option is set and rpm sensor says we're not running, this is the timer to inhibit any action
    uint32_t running_rpm_fail_timer_ms;

    bool auto_mode_active;

    bool vehicle_is_waiting_in_auto;

    uint32_t delay_starter_on_first_since_boot_timer_ms;
};


namespace AP {
    AP_ICEngine *ice();
};
