#include "mode.h"
#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_lateral, brake = 0;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);

    } else if (desired_throttle <= 0) {
        // with no throttle, add in a little brake just to have something
        brake = rover.g2.attitude_control.get_brake_manual_pct();
    }

    // set sailboat sails
    float desired_mainsail;
    float desired_wingsail;
    g2.sailboat.get_pilot_desired_mainsail(desired_mainsail, desired_wingsail);
    g2.motors.set_mainsail(desired_mainsail);
    g2.motors.set_wingsail(desired_wingsail);

#if 0
    if (is_positive(g2.manual_speed_limit)) {
        const uint32_t now_ms = AP_HAL::millis();
        float speed_acc;
        if (AP::gps().status() < AP_GPS::GPS_OK_FIX_3D ||
            AP::gps().get_hdop() > 5 ||
            !AP::gps().speed_accuracy(speed_acc))
        {
            // with bad GPS, limit throttle to fixed percent via param
            desired_throttle = MIN(desired_throttle, g2.manual_throttle_percent_max);

        } else {
            if (now_ms - throttle_max_timer_ms >= 200) {
                throttle_max_timer_ms = now_ms; // slow it down to 5Hz

                if (AP::ahrs().groundspeed() > g2.manual_speed_limit) {
                   // if speed is too fast, deduce max throttle
                   if (throttle_max < 2) {
                       throttle_max = 0;
                   } else {
                       throttle_max *= constrain_float(g2.manual_throttle_max_rate_shrink, 0.01f, 1.0f);
                       throttle_max = constrain_float(throttle_max, 0, 100);
                   }

                } else if (desired_throttle >= throttle_max && hal.util->get_soft_armed()) {
                    // if we're too slow and the throttle is at max throttle for a couple seconds, raise max throttle
                    if (throttle_max < 2) {
                        throttle_max = 10;
                    } else if (now_ms - throttle_max_timer_grow_ms > 3000) {
                        throttle_max *= constrain_float(g2.manual_throttle_max_rate_grow, 1.0f, 10.0f);
                        throttle_max = constrain_float(throttle_max, 0, 100);
                    }

                } else {
                    throttle_max_timer_grow_ms = now_ms;
                }
            }
            desired_throttle = MIN(desired_throttle, throttle_max);
        }
    }
#endif

    // copy RC scaled inputs to outputs
    rover.set_throttle(desired_throttle);
    rover.set_brake(brake * attitude_control.get_brake_gain());
    g2.motors.set_steering(desired_steering, false);
    g2.motors.set_lateral(desired_lateral);
}
