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

#include <AP_HAL/AP_HAL.h>
#include <AP_RPM/AP_RPM.h>

class AP_ICEngine {
public:
    // constructor
    AP_ICEngine(const AP_RPM &_rpm);

    /* Do not allow copies */
    AP_ICEngine(const AP_ICEngine &other) = delete;
    AP_ICEngine &operator=(const AP_ICEngine&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // update engine state. Should be called at 10Hz or more
    void update(void);

    // check for throttle override
    bool throttle_override(uint8_t &percent);

    enum ICE_State {
        ICE_OFF=0,
        ICE_START_HEIGHT_DELAY=1,
        ICE_START_DELAY=2,
        ICE_STARTING=3,
        ICE_RUNNING=4
    };

    // get current engine control state
    ICE_State get_state(void) const { return state; }

    // handle DO_ENGINE_CONTROL messages via MAVLink or mission
    bool engine_control(float start_control, float cold_start, float height_delay);
    
    // Engine temperature status
    bool get_temperature(float& value) const;
    bool too_hot() const { return temperature.too_hot(); }
    bool too_cold() const { return temperature.too_cold(); }
    void send_temp();

    static AP_ICEngine *get_singleton() { return _singleton; }

private:
    static AP_ICEngine *_singleton;

    const AP_RPM &rpm;

    enum ICE_State state;

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

        AP_HAL::AnalogSource *source;
        float value;
        uint32_t last_sample_ms;
        uint32_t last_send_ms;

        bool is_healthy() const { return (pin > 0 && last_sample_ms && (AP_HAL::millis() - last_sample_ms < 1000)); }
        bool too_hot() const { return this->is_healthy() && (value > max); }
        bool too_cold() const { return this->is_healthy() && (value < min); }
    } temperature;

    enum Temperature_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    void update_temperature();

    void set_output_channels();

    void determine_state();


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
    
    // pwm values 
    AP_Int16 pwm_ignition_on;
    AP_Int16 pwm_ignition_off;
    AP_Int16 pwm_starter_on;
    AP_Int16 pwm_starter_off;
    
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

    // Ignition state during starting. On or off.
    AP_Int8 ignition_during_start;

    // height when we enter ICE_START_HEIGHT_DELAY
    float initial_height;

    // height change required to start engine
    float height_required;

    // we are waiting for valid height data
    bool height_pending:1;

    // timestamp for periodic gcs msg regarding throttle_override
    uint32_t throttle_overrde_msg_last_ms;

};


namespace AP {
    AP_ICEngine *ice();
};
