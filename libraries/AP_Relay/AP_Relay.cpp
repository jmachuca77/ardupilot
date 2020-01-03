/*
 * AP_Relay.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Relay.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
  #define RELAY1_PIN_DEFAULT 13

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
    #define RELAY1_PIN_DEFAULT 57
    #define RELAY2_PIN_DEFAULT 49
    #define RELAY3_PIN_DEFAULT 116
    #define RELAY4_PIN_DEFAULT 113
  #endif
#endif

#ifndef RELAY1_PIN_DEFAULT
  #define RELAY1_PIN_DEFAULT -1
#endif

#ifndef RELAY2_PIN_DEFAULT
  #define RELAY2_PIN_DEFAULT -1
#endif

#ifndef RELAY3_PIN_DEFAULT
  #define RELAY3_PIN_DEFAULT -1
#endif

#ifndef RELAY4_PIN_DEFAULT
  #define RELAY4_PIN_DEFAULT -1
#endif

#ifndef RELAY5_PIN_DEFAULT
  #define RELAY5_PIN_DEFAULT -1
#endif

#ifndef RELAY6_PIN_DEFAULT
  #define RELAY6_PIN_DEFAULT -1
#endif

#ifndef RELAY7_PIN_DEFAULT
  #define RELAY7_PIN_DEFAULT -1
#endif

#ifndef RELAY8_PIN_DEFAULT
  #define RELAY8_PIN_DEFAULT -1
#endif

#ifndef RELAY9_PIN_DEFAULT
  #define RELAY9_PIN_DEFAULT -1
#endif

#ifndef RELAY10_PIN_DEFAULT
  #define RELAY10_PIN_DEFAULT -1
#endif

#ifndef RELAY11_PIN_DEFAULT
  #define RELAY11_PIN_DEFAULT -1
#endif

#ifndef RELAY12_PIN_DEFAULT
  #define RELAY12_PIN_DEFAULT -1
#endif

#ifndef RELAY13_PIN_DEFAULT
  #define RELAY13_PIN_DEFAULT -1
#endif

#ifndef RELAY14_PIN_DEFAULT
  #define RELAY14_PIN_DEFAULT -1
#endif


// MCP28003 address and registers
#define MCP23008_I2C_BUS                        1
#define MCP23008_I2C_ADDRESS                    0x20

#define MCP23008_I2C_REGISTER_IODIR             0x00    // 0 is output, 1 is input
#define MCP23008_I2C_REGISTER_IPOL              0x01
#define MCP23008_I2C_REGISTER_GPINTEN           0x02
#define MCP23008_I2C_REGISTER_DEFVAL            0x03
#define MCP23008_I2C_REGISTER_INTCON            0x04
#define MCP23008_I2C_REGISTER_IOCON             0x05
#define MCP23008_I2C_REGISTER_GPPU              0x06
#define MCP23008_I2C_REGISTER_INTF              0x07
#define MCP23008_I2C_REGISTER_INTCAP            0x08
#define MCP23008_I2C_REGISTER_GPIO              0x09    // gpio pin state is here
#define MCP23008_I2C_REGISTER_OLAT              0x0A


const AP_Param::GroupInfo AP_Relay::var_info[] = {
    // @Param: PIN
    // @DisplayName: First Relay Pin
    // @Description: Digital pin number for first relay control. This is the pin used for camera control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN",  0, AP_Relay, _pin[0], RELAY1_PIN_DEFAULT),

    // @Param: PIN2
    // @DisplayName: Second Relay Pin
    // @Description: Digital pin number for 2nd relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN2",  1, AP_Relay, _pin[1], RELAY2_PIN_DEFAULT),

    // @Param: PIN3
    // @DisplayName: Third Relay Pin
    // @Description: Digital pin number for 3rd relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN3",  2, AP_Relay, _pin[2], RELAY3_PIN_DEFAULT),

    // @Param: PIN4
    // @DisplayName: Fourth Relay Pin
    // @Description: Digital pin number for 4th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN4",  3, AP_Relay, _pin[3], RELAY4_PIN_DEFAULT),

    // @Param: DEFAULT
    // @DisplayName: Default relay state
    // @Description: The state of the relay on boot.
    // @User: Standard
    // @Values: 0:Off,1:On,2:NoChange
    AP_GROUPINFO("DEFAULT",  4, AP_Relay, _default, 0),

    // @Param: PIN5
    // @DisplayName: Fifth Relay Pin
    // @Description: Digital pin number for 5th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN5",  5, AP_Relay, _pin[4], RELAY5_PIN_DEFAULT),

    // @Param: PIN6
    // @DisplayName: Sixth Relay Pin
    // @Description: Digital pin number for 6th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN6",  6, AP_Relay, _pin[5], RELAY6_PIN_DEFAULT),

    // @Param: PIN1_INV
    // @DisplayName: First Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN1_INV",  7, AP_Relay, _inverted[0], 0),

    // @Param: PIN2_INV
    // @DisplayName: Second Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN2_INV",  8, AP_Relay, _inverted[1], 0),

    // @Param: PIN3_INV
    // @DisplayName: Third Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN3_INV",  9, AP_Relay, _inverted[2], 0),

    // @Param: PIN4_INV
    // @DisplayName: Fourth Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN4_INV",  10, AP_Relay, _inverted[3], 0),

    // @Param: PIN5_INV
    // @DisplayName: Fifth Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN5_INV",  11, AP_Relay, _inverted[4], 0),

    // @Param: PIN6_INV
    // @DisplayName: Sixth Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN6_INV",  12, AP_Relay, _inverted[5], 0),

#if AP_RELAY_NUM_RELAYS >= 7
    // @Param: PIN7
    // @DisplayName: Seventh Relay Pin
    // @Description: Digital pin number for 7th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN7", 13, AP_Relay, _pin[6], RELAY7_PIN_DEFAULT),

    // @Param: PIN7_INV
    // @DisplayName: Seventh Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN7_INV", 14, AP_Relay, _inverted[6], 0),
#endif
#if AP_RELAY_NUM_RELAYS >= 8
    // @Param: PIN8
    // @DisplayName: Eighth Relay Pin
    // @Description: Digital pin number for 8th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN8", 15, AP_Relay, _pin[7], RELAY8_PIN_DEFAULT),

    // @Param: PIN8_INV
    // @DisplayName: Eigth Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN8_INV", 16, AP_Relay, _inverted[7], 0),
#endif

#if AP_RELAY_NUM_RELAYS >= 9
    // @Param: PIN9
    // @DisplayName: Ninth Relay Pin
    // @Description: Digital pin number for 9th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN9", 17, AP_Relay, _pin[8], RELAY9_PIN_DEFAULT),

    // @Param: PIN9_INV
    // @DisplayName: Ninth Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN9_INV", 18, AP_Relay, _inverted[8], 0),
#endif

#if AP_RELAY_NUM_RELAYS >= 10
    // @Param: PIN10
    // @DisplayName: Tenth Relay Pin
    // @Description: Digital pin number for 10th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN10", 19, AP_Relay, _pin[9], RELAY10_PIN_DEFAULT),

    // @Param: PIN10_INV
    // @DisplayName: Tenth Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN10_INV", 20, AP_Relay, _inverted[9], 0),
#endif

#if AP_RELAY_NUM_RELAYS >= 11
    // @Param: PIN11
    // @DisplayName: Eleventh Relay Pin
    // @Description: Digital pin number for 11th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN11", 21, AP_Relay, _pin[10], RELAY11_PIN_DEFAULT),

    // @Param: PIN11_INV
    // @DisplayName: Eleventh Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN11_INV", 22, AP_Relay, _inverted[10], 0),
#endif

#if AP_RELAY_NUM_RELAYS >= 12
    // @Param: PIN12
    // @DisplayName: Twelfth Relay Pin
    // @Description: Digital pin number for 12th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN12", 23, AP_Relay, _pin[11], RELAY12_PIN_DEFAULT),

    // @Param: PIN12_INV
    // @DisplayName: Twelfth Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN12_INV", 24, AP_Relay, _inverted[11], 0),
#endif

#if AP_RELAY_NUM_RELAYS >= 13
    // @Param: PIN13
    // @DisplayName: Thirteenth Relay Pin
    // @Description: Digital pin number for 13th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN13", 25, AP_Relay, _pin[12], RELAY13_PIN_DEFAULT),

    // @Param: PIN13_INV
    // @DisplayName: Thirteenth Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN13_INV", 26, AP_Relay, _inverted[12], 0),
#endif

#if AP_RELAY_NUM_RELAYS >= 14
    // @Param: PIN14
    // @DisplayName: Fourteenth Relay Pin
    // @Description: Digital pin number for 14th relay control.
    // @User: Standard
    // @Values: -1:Disabled,49:BB Blue GP0 pin 4,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6,57:BB Blue GP0 pin 3,113:BB Blue GP0 pin 6,116:BB Blue GP0 pin 5,101:ExternalPin1,102:ExternalPin2,103:ExternalPin3,104:ExternalPin4,105:ExternalPin5,106:ExternalPin6,107:ExternalPin7,108:ExternalPin8
    AP_GROUPINFO("PIN14", 27, AP_Relay, _pin[13], RELAY14_PIN_DEFAULT),

    // @Param: PIN14_INV
    // @DisplayName: Fourteenth Relay Pin Inverted
    // @Description: Digital pin inverted
    // @User: Standard
    // @Values: 0:Normal,1:Inverted
    AP_GROUPINFO("PIN14_INV", 28, AP_Relay, _inverted[13], 0),
#endif

    // @Param: EXT_TYPE
    // @DisplayName: External Relay Device Type
    // @Description: Type of attached external Relay device
    // @User: Standard
    // @Values: 0:None,1:MCP23008_I2C
    AP_GROUPINFO("EXT_TYPE", 29, AP_Relay, _external.param_type, 0),

    // @Param: EXT_ADDR
    // @DisplayName: External Relay Device Address
    // @Description: External Relay Device Address. Typically used for I2C. Use 0 to use the default for the selected EXT_TYPE
    // @User: Standard
    AP_GROUPINFO("EXT_ADDR", 30, AP_Relay, _external.param_address, 0),

    // @Param: EXT_BUS
    // @DisplayName: External Relay Device Bus
    // @Description: This selects the bus number for looking for an I2C relay device. When set to -1 it will probe all
    // @Values: -1:ProbeAll,0:Bus0,1:Bus1,2:Bus2
    AP_GROUPINFO("EXT_BUS", 31, AP_Relay, _external.param_bus, -1),


    AP_GROUPEND
};

AP_Relay *AP_Relay::singleton;

extern const AP_HAL::HAL& hal;

AP_Relay::AP_Relay(void)
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("AP_Relay must be singleton");
    }
#endif
    singleton = this;
}


void AP_Relay::init()
{
    set_to_default();
    init_hw();
}

void AP_Relay::set_to_default()
{
    for (uint8_t instance=0; instance<AP_RELAY_NUM_RELAYS; instance++) {
        set_to_default(instance);
    }
}

void AP_Relay::set_to_default(const uint8_t instance)
{
    if (_default != 0 && _default != 1) {
        return;
    }
    set(instance, _default);
}

void AP_Relay::set(const uint8_t instance, const bool value)
{
    if (!enabled(instance)) {
        return;
    }

    const bool ison = _inverted[instance] ? !value : value;

    if (is_external_pin(instance)) {
        const uint8_t external_index = (_pin[instance] - AP_RELAY_EXTENRAL_PIN_FIRST);
        if (ison) {
            _external.output_bitfield |= (1 << external_index);
        } else {
            _external.output_bitfield &= ~(1 << external_index);
        }
    } else {
        hal.gpio->pinMode(_pin[instance], HAL_GPIO_OUTPUT);
        hal.gpio->write(_pin[instance], ison);
    }
}

void AP_Relay::toggle(const uint8_t instance)
{
    if (!enabled(instance)) {
        return;
    }

    bool ison;
    if (is_external_pin(instance)) {
        const uint8_t external_index = (_pin[instance] - AP_RELAY_EXTENRAL_PIN_FIRST);
        ison = (_external.output_bitfield & (1 << external_index)) != 0;
    } else {
        ison = hal.gpio->read(_pin[instance]);
    }

    // *NOTE* this inversion check is not intuitive. It is required
    // because set() will invert so inverting here will give the
    // correctly toggled value result
    ison = _inverted[instance] ? !ison : ison;

    set(instance, !ison);
}

void AP_Relay::set_all(const uint32_t mask)
{
    for (uint32_t instance=0; instance<AP_RELAY_NUM_RELAYS; instance++) {
        const bool ison = ((1 << instance) & mask) != 0;
        set(instance, ison);
    }

}

void AP_Relay::init_hw()
{
    switch (_external.param_type) {
        case EXTERNAL_TYPE_MCP23008_I2C:
            if (_external.init_hw_MCP23008_I2C()) {
                _external.instance = _external_instance_count;
                _external_instance_count++;
            }
            break;

        case EXTERNAL_TYPE_NONE:
          default:
            break;
     }
}

bool AP_Relay::External_Device::init_hw_MCP23008_I2C()
{
    const uint8_t address = (param_address == 0) ? MCP23008_I2C_ADDRESS : param_address;

    if (param_bus >= 0) {
        return probe_MCP23008_I2C(param_bus, address);
    }

    for (int8_t bus=0; bus<2; bus++) {
        if (probe_MCP23008_I2C(bus, address)) {
            return true;
        }
    }
    return false;
}

bool AP_Relay::External_Device::probe_MCP23008_I2C(const int8_t bus, const uint8_t address)
{
    //gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Relay: probe_MCP23008_I2C bus:%d addr:%d", bus, address);
    dev = hal.i2c_mgr->get_device(bus, address);

    if (!dev) {
        //gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Relay: probe_MCP23008_I2C = null");
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_retries(5);

    // all 8 pins as outputs and normal polarity
    hal.scheduler->delay(10);
    if (!dev->write_register(MCP23008_I2C_REGISTER_IODIR, 0)) {
        //gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Relay: probe IODIR write fail");
        return false;
    }

    hal.scheduler->delay(10);
    if (!dev->write_register(MCP23008_I2C_REGISTER_IPOL, 0)) {
        //gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Relay: probe IPOL write fail");
        return false;
    }

    // keep this module at 5 retries because it does not writ4e very often
    //_dev->set_retries(1);

    dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&AP_Relay::External_Device::timer, void));

    //gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Relay: MCP23008_I2C found bus:%d addr:%d", bus, address);
    return true;
}


void AP_Relay::External_Device::timer()
{
    const uint32_t desired_output = output_bitfield;

    if (desired_output == output_bitfield_last) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    if (write_fail_snooze_ms != 0) {
        if (now_ms - write_fail_snooze_ms < 60000) {
            return;
        }
        write_fail_snooze_ms = 0;
    }

    if (dev->write_register(MCP23008_I2C_REGISTER_GPIO, desired_output)) {
        //gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Relay: changed from 0x%02x to 0x%02x", _desired_output_bitfield_last, desired_output);
        output_bitfield_last = desired_output;

    } else {
        // if we get ten failures within 10 seconds, snooze for 1 minute so we don't spam the bus for a bad device
        write_fail_count_total++;

        //gcs().send_text(MAV_SEVERITY_DEBUG, "%d AP_Relay: write failed", now_ms);

        if (write_fail_last_ms != 0 && now_ms - write_fail_last_ms >= 10000) {
            // previous failure was >10s ago. consider this the first
            write_fail_count = 0;
        }

        write_fail_count++;
        write_fail_last_ms = now_ms;

        if (write_fail_count >= 10) {
            //gcs().send_text(MAV_SEVERITY_DEBUG, "%d AP_Relay: too many i2c write failures, snoozing", now_ms);
            write_fail_snooze_ms = now_ms;
            write_fail_count = 0;
        }
    }
}



namespace AP {

AP_Relay *relay() {
    return AP_Relay::get_singleton();
}

}
