#include "Rover.h"

// Code to detect a crash or block
static const uint32_t CRASH_CHECK_TRIGGER_MS = 2000;   // 2 seconds blocked indicates a crash
static const float CRASH_CHECK_THROTTLE_MIN = 5.0f;  // vehicle must have a throttle greater that 5% to be considered crashed
static const float CRASH_CHECK_VEL_MIN = 0.08f;      // vehicle must have a velocity under 0.08 m/s or rad/s to be considered crashed

// crash_check - disarms motors if a crash or block has been detected
// crashes are detected by the vehicle being static (no speed) for more than CRASH_CHECK_TRIGGER_MS and motor are running
// called at 10Hz
void Rover::crash_check()
{
    bool crashed = false; //stores crash state

    // return immediately if disarmed, crash checking is disabled or vehicle is Hold, Manual or Acro mode(if it is not a balance bot)
    if (!arming.is_armed() || g.fs_crash_check == FS_CRASH_DISABLE || ((!control_mode->is_autopilot_mode()) && (!is_balancebot()))) {
        crash_last_ms = 0;
        return;
    }

    // Crashed if pitch/roll > crash_angle
    if (g2.crash_angle_pitch > 0 && (fabsf(ahrs.pitch) > radians(g2.crash_angle_pitch))) {
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Pitch angle %.1f exceeded %d", degrees(fabsf(ahrs.pitch)), (int)g2.crash_angle_pitch);
        crashed = true;
    } else if (g2.crash_angle_roll > 0 && (fabsf(ahrs.roll) > radians(g2.crash_angle_roll))) {
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Roll angle %.1f exceeded %d", degrees(fabsf(ahrs.roll)), (int)g2.crash_angle_roll);
        crashed = true;
    }

    // TODO : Check if min vel can be calculated
    // min_vel = ( CRASH_CHECK_THROTTLE_MIN * g.speed_cruise) / g.throttle_cruise;

    if (!is_balancebot()) {
        const bool is_not_crashing = (ahrs.groundspeed() >= CRASH_CHECK_VEL_MIN) ||         // is moving
                            (fabsf(ahrs.get_gyro().z) >= CRASH_CHECK_VEL_MIN) ||            // is turning
                            (fabsf(g2.motors.get_throttle()) <= (g2.ice_control.get_idle_throttle() + CRASH_CHECK_THROTTLE_MIN)) || // has throttle
                            (g2.ice_control.enabled() && g2.ice_control.gear_is_inhibiting_locomotion()) ||
                            g2.avoid.get_is_slowing_vehicle() ||
                            control_mode->is_waiting();                                     // is intentionally not moving;

        if (!crashed && is_not_crashing) {
            crash_last_ms = 0;
            return;
        }

        const uint32_t now_ms = AP_HAL::millis();

        // we may be crashing
        if (crash_last_ms == 0) {
            crash_last_ms = now_ms;
        } else if (now_ms - crash_last_ms >= CRASH_CHECK_TRIGGER_MS) {
            // failed check for crashing after a few seconds
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Expected to be moving");
            crashed = true;
        }
    }

    if (crashed) {
        AP::logger().Write_Error(LogErrorSubsystem::CRASH_CHECK,
                                 LogErrorCode::CRASH_CHECK_CRASH);

        if (is_balancebot()) {
            // send message to gcs
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Disarming");
            arming.disarm(AP_Arming::Method::CRASH);
        } else {
            // change mode to hold and disarm
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Going to HOLD");
            set_mode(mode_hold, ModeReason::CRASH_FAILSAFE);
            if (g.fs_crash_check == FS_CRASH_HOLD_AND_DISARM) {
                arming.disarm(AP_Arming::Method::CRASH);
            }
        }
    }
}
