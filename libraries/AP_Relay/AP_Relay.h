/*
 * AP_Relay.h
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

/// @file	AP_Relay.h
/// @brief	APM relay control class
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_HAL/I2CDevice.h>

#define AP_RELAY_NUM_RELAYS 14

#define AP_RELAY_EXTENRAL_PIN_FIRST            101
#define AP_RELAY_EXTENRAL_PIN_LAST             108

/// @class	AP_Relay
/// @brief	Class to manage the ArduPilot relay
class AP_Relay {
public:
    AP_Relay();

    /* Do not allow copies */
    AP_Relay(const AP_Relay &other) = delete;
    AP_Relay &operator=(const AP_Relay&) = delete;

    // setup the relay pin
    void        init();

    // activate the relay
    void        on(const uint8_t instance) { set(instance, true); }

    // de-activate the relay
    void        off(const uint8_t instance) { set(instance, false); }

    // set the relay to either on or off state
    void        set(const uint8_t instance, bool value);

    // set all relays all at once via mask
    void        set_all(const uint32_t mask);

    // see if the relay is enabled
    bool        enabled(const uint8_t instance) { return instance < AP_RELAY_NUM_RELAYS && _pin[instance] != -1; }

//    // disable the relay
//    void        disable(uint8_t relay) { if (relay < AP_RELAY_NUM_RELAYS) { _pin[relay] = -1; } }
//
//    // see if the relay is inverted
//    bool        inverted(uint8_t relay) { return relay < AP_RELAY_NUM_RELAYS && _inverted[relay] != 0; }
//
//    // invert the relay
//    void        invert(uint8_t relay, bool is_inverted) { if (relay < AP_RELAY_NUM_RELAYS) { _inverted[relay] = is_inverted; } }
//
    // toggle the relay status
    void        toggle(const uint8_t instance);

    // set index to default (bootup state)
    void        set_to_default(const uint8_t index);

    // set all to default (bootup state)
    void        set_to_default();

    static AP_Relay *get_singleton(void) {return singleton; }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_Relay *singleton;

    AP_Int8 _pin[AP_RELAY_NUM_RELAYS];
    AP_Int8 _inverted[AP_RELAY_NUM_RELAYS];
    AP_Int8 _default;

    // External relay types
    typedef enum {
        EXTERNAL_TYPE_NONE,
        EXTERNAL_TYPE_MCP23008_I2C,
    } external_relay_type_t;

    // External devices
    struct External_Device {
        uint32_t    output_bitfield;
        uint32_t    output_bitfield_last = 0xFFFFFFFF; // the upper bits can not be set pragmatically so this ensures an initial mismatch-forced write after init
        uint32_t    write_fail_count;
        uint32_t    write_fail_count_total;
        uint32_t    write_fail_last_ms;
        uint32_t    write_fail_snooze_ms;
        uint8_t     instance;
        AP_Int8     param_type;
        AP_Int8     param_address;
        AP_Int8     param_bus;
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
        bool        init_hw_MCP23008_I2C();
        bool        probe_MCP23008_I2C(const int8_t bus, const uint8_t address);

        // external periodic callback
        void        timer(void);
    } _external;

    void        init_hw();
    uint8_t     _external_instance_count;

    // returns true if selected instance is configured as an external device
    bool        is_external_pin(uint8_t instance) { return (instance < AP_RELAY_NUM_RELAYS) && (_pin[instance] >= AP_RELAY_EXTENRAL_PIN_FIRST) && (_pin[instance] <= AP_RELAY_EXTENRAL_PIN_LAST); }


};

namespace AP {
    AP_Relay *relay();
};
