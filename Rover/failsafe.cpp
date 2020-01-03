/*
  failsafe support
  Andrew Tridgell, December 2011
 */

#include "Rover.h"

#include <stdio.h>

/*
  our failsafe strategy is to detect main loop lockup and disarm.
 */

/*
  this failsafe_check function is called from the core timer interrupt
  at 1kHz.
 */
void Rover::failsafe_check()
{
    static uint16_t last_ticks;
    static uint32_t last_timestamp;
    const uint32_t tnow = AP_HAL::micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        // the main loop is running, all is OK
        last_ticks = ticks;
        last_timestamp = tnow;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. disarm the motors
        // To-Do: log error
        if (arming.is_armed()) {
            // disarm motors
            arming.disarm(AP_Arming::Method::CPUFAILSAFE);
        }
    }

    // If our PID.I is at 95% of our PID.IMAX then something is wrong.
    if (g2.fs_pid_i_steering > 0 && arming.is_armed()) {
        const float i = g2.attitude_control.get_steering_rate_pid().get_i();
        const float imax = g2.attitude_control.get_steering_rate_pid().imax();
        if (i/imax > 0.95f) {
            // our PID.I is at 95% of our PID.IMAX. Something is wrong.
            gcs().send_text(MAV_SEVERITY_CRITICAL, "FAILSAFE: Steering PID Integrator out of bounds");
            arming.disarm();
        }
    }

    if (g2.fs_pid_i_throttle > 0 && arming.is_armed()) {
        const float i = g2.attitude_control.get_throttle_speed_pid().get_i();
        const float imax = g2.attitude_control.get_throttle_speed_pid().imax();
        if (i/imax > 0.95f) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "FAILSAFE: Throttle PID Integrator out of bounds");
            arming.disarm();
        }
    }

}

const char* Rover::failsafe_to_string(const uint8_t failsafe_type)
{
    switch (failsafe_type) {
    case FAILSAFE_EVENT_THROTTLE:
        return "Throttle";
    case FAILSAFE_EVENT_GCS:
        return "GCS";
    case (FAILSAFE_EVENT_THROTTLE | FAILSAFE_EVENT_GCS):
        return "Throttle and GCS";
    }
    return "Unknown Type";
}

/*
  called to set/unset a failsafe event.
 */
void Rover::failsafe_trigger(uint8_t failsafe_type, const char* type_str, bool on)
{
    uint8_t old_bits = failsafe.bits;
    if (on) {
        failsafe.bits |= failsafe_type;
    } else {
        failsafe.bits &= ~failsafe_type;
    }
    if (old_bits == 0 && failsafe.bits != 0) {
        // a failsafe event has started
        failsafe.start_time = millis();
    }
    if (failsafe.triggered != 0 && failsafe.bits == 0) {
        // a failsafe event has ended
        gcs().send_text(MAV_SEVERITY_INFO, "%s Failsafe Cleared", type_str);
    }

    failsafe.triggered &= failsafe.bits;

    if ((failsafe.triggered == 0) &&
        (failsafe.bits != 0) &&
        (millis() - failsafe.start_time > g.fs_timeout * 1000) &&
        (control_mode != &mode_rtl) &&
        (control_mode != &mode_manual) &&
        ((control_mode != &mode_hold || (g2.fs_options & (uint32_t)Failsafe_Options::Failsafe_Option_Active_In_Hold)))) {
        failsafe.triggered = failsafe.bits;
        gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe trigger: %s", failsafe_to_string(failsafe.triggered));

        // clear rc overrides
        RC_Channels::clear_overrides();

        if ((control_mode == &mode_auto) &&
            ((failsafe_type == FAILSAFE_EVENT_THROTTLE && g.fs_throttle_enabled == FS_THR_ENABLED_CONTINUE_MISSION) ||
             (failsafe_type == FAILSAFE_EVENT_GCS && g.fs_gcs_enabled == FS_GCS_ENABLED_CONTINUE_MISSION))) {
            // continue with mission in auto mode
        } else {
            switch (g.fs_action) {
            case Failsafe_Action_None:
                break;
            case Failsafe_Action_RTL:
                if (!set_mode(mode_rtl, ModeReason::FAILSAFE)) {
                    set_mode(mode_manual, ModeReason::FAILSAFE);
                }
                break;
            case Failsafe_Action_Hold:
                set_mode(mode_manual, ModeReason::FAILSAFE);
                break;
            case Failsafe_Action_SmartRTL:
                if (!set_mode(mode_smartrtl, ModeReason::FAILSAFE)) {
                    if (!set_mode(mode_rtl, ModeReason::FAILSAFE)) {
                        set_mode(mode_manual, ModeReason::FAILSAFE);
                    }
                }
                break;
            case Failsafe_Action_SmartRTL_Hold:
                if (!set_mode(mode_smartrtl, ModeReason::FAILSAFE)) {
                    set_mode(mode_manual, ModeReason::FAILSAFE);
                }
                break;
            }
        }
    }
}

void Rover::handle_battery_failsafe(const char* type_str, const int8_t action)
{
        switch ((Failsafe_Action)action) {
            case Failsafe_Action_None:
                break;
            case Failsafe_Action_SmartRTL:
                if (set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case Failsafe_Action_RTL:
                if (set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case Failsafe_Action_Hold:
                set_mode(mode_manual, ModeReason::BATTERY_FAILSAFE);
                break;
            case Failsafe_Action_SmartRTL_Hold:
                if (!set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    set_mode(mode_manual, ModeReason::BATTERY_FAILSAFE);
                }
                break;
            case Failsafe_Action_Terminate:
#if ADVANCED_FAILSAFE == ENABLED
                char battery_type_str[17];
                snprintf(battery_type_str, 17, "%s battery", type_str);
                g2.afs.gcs_terminate(true, battery_type_str);
#else
                arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
#endif // ADVANCED_FAILSAFE == ENABLED
                break;
        }
}

#if ADVANCED_FAILSAFE == ENABLED
/*
   check for AFS failsafe check
 */
void Rover::afs_fs_check(void)
{
    // perform AFS failsafe checks
    g2.afs.check(failsafe.last_heartbeat_ms, g2.fence.get_breaches() != 0, failsafe.last_valid_rc_ms);
}
#endif
