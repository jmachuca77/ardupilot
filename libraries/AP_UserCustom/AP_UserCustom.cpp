#include "AP_UserCustom.h"

#if !HAL_MINIMIZE_FEATURES && 1

#include <AP_Proximity/AP_Proximity.h>
#include "AP_Arming/AP_Arming.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_UserCustom::var_info[] = {
        // @Param: ENABLE
        // @DisplayName: UserCustom Enable/Disable
        // @Description: UserCustom enable/disable
        // @User: Standard
        // @Values: 0:Disabled, 1:Enabled
        AP_GROUPINFO_FLAGS("ENABLE", 0, AP_UserCustom, _enabled, 1, AP_PARAM_FLAG_ENABLE),

        // @Param: TEST_INT1
        // @DisplayName: User Custom Integer 1
        // @Description: User Custom Integer 1
        // @User: Standard
        AP_GROUPINFO("TEST_INT1", 1, AP_UserCustom, test_int1, 0),

        // @Param: TEST_INT2
        // @DisplayName: User Custom Integer 2
        // @Description: User Custom Integer 3
        // @User: Standard
        AP_GROUPINFO("TEST_INT2", 2, AP_UserCustom, test_int2, 0),

        // @Param: TEST_INT3
        // @DisplayName: User Custom Integer 4
        // @Description: User Custom Integer 4
        // @User: Standard
        AP_GROUPINFO("TEST_INT3", 3, AP_UserCustom, test_int3, 0),

        // @Param: TEST_FLT1
        // @DisplayName: User Custom Float 1
        // @Description: User Custom Float 1
        // @User: Standard
        AP_GROUPINFO("TEST_FLT1", 4, AP_UserCustom, test_float1, 0),

        // @Param: TEST_FLT2
        // @DisplayName: User Custom Float 2
        // @Description: User Custom Float 2
        // @User: Standard
        AP_GROUPINFO("TEST_FLT2", 5, AP_UserCustom, test_float2, 0),

        // @Param: TEST_FLT3
        // @DisplayName: User Custom Float 3
        // @Description: User Custom Float 3
        // @User: Standard
        AP_GROUPINFO("TEST_FLT3", 6, AP_UserCustom, test_float3, 0),

        // @Param: ARMCHK_LDR
        // @DisplayName: Arming check for LiDAR health
        // @Description: Arming check for LiDAR health
        // @User: Standard
        AP_GROUPINFO("ARMCHK_LDR", 7, AP_UserCustom, arming_check_Lidar, 1),

        // @Param: UI1
        // @Param: UI2
        // @Param: UI3
        // @Param: UI4
        AP_GROUPINFO("UI1", 8, AP_UserCustom, UI[0], 0),
        AP_GROUPINFO("UI2", 9, AP_UserCustom, UI[1], 0),
        AP_GROUPINFO("UI3", 10, AP_UserCustom, UI[2], 0),
        AP_GROUPINFO("UI4", 11, AP_UserCustom, UI[3], 0),
    AP_GROUPEND
};

AP_UserCustom::AP_UserCustom() {
    // constructor
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many AP_UserCustom");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// return true on successful init
bool AP_UserCustom::init()
{
    // return immediately if not enabled
    if (!enabled()) {
        return false;
    }

    // TODO: add custom init work

    return true;
}

// periodic callback that runs at 50Hz
void AP_UserCustom::update()
{
    // return immediately if not enabled
    if (!enabled()) {
        is_initialized = false;
        return;
    } else if (!is_initialized) {
        is_initialized = init();
        // return to give a tiny amount of time between init and update in case it failed or extra time is needed after init
        return;
    }


    if (arming_check_Lidar == 1) {
        if (!arming_initial_fail_ms || AP::arming().is_armed()) {
            arming_initial_fail_ms = 0;
            return;
        }

        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - arming_retry_ms > 1000) {
            arming_retry_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_INFO, "%d Arming auto-retry, quiet", (unsigned)now_ms);
            if (AP::arming().pre_arm_checks(false) || (now_ms - arming_initial_fail_ms >= 45000)) {
                // if we pass the checks or timeout, attempt one final arm attempt
                gcs().send_text(MAV_SEVERITY_INFO, "%d Arming auto-retry, loud and final", (unsigned)now_ms);
                AP::arming().arm(AP_Arming::Method::USER_CUSTOM, true);
                arming_initial_fail_ms = 0; // clear this after the check to disable the timer in case we fail again
            }
        }
    }
}

// return true if checks are OK and we allow arming.
// return false if checks fail and we want to block arming. If "report" is true we can optionally send a message to inform the user and/or external system(s) of the reason
bool AP_UserCustom::arming_check(bool report)
{
    if (!enabled()) {
        return true;
    }

    bool checks_passed = true;

    if (arming_check_Lidar == 1) {
        AP_Proximity *proximity = AP::proximity();

        if (proximity != nullptr && (proximity->get_type(0) == AP_Proximity::Type::MAV)) {
            checks_passed = proximity->healthy() && (lidar_M8_status == MAV_M8_RUNNING);
        }
        if (!checks_passed && report) {
            // report is true only when an actual arm check happens. Otherwise other checks are always passively happening quietly
            if (!arming_initial_fail_ms) {
                arming_initial_fail_ms = AP_HAL::millis();

                const char* msg = (lidar_M8_status == MAV_M8_STARTUP) ? "LiDAR is Starting Up" : "LiDAR Error";
                gcs().send_text(MAV_SEVERITY_INFO, "Arming Failure: %s", msg);
            }
        }
    }

    return checks_passed;
}

// handle inbound MAVLink messages
MAV_RESULT AP_UserCustom::handle_user_message(const mavlink_command_long_t &packet)
{
    if (!enabled()) {
        return MAV_RESULT_FAILED;
    }

    switch (packet.command) {
    case MAV_CMD_WAYPOINT_USER_1:
    case MAV_CMD_WAYPOINT_USER_2:
    case MAV_CMD_WAYPOINT_USER_3:
    case MAV_CMD_WAYPOINT_USER_4:
    case MAV_CMD_WAYPOINT_USER_5:
    case MAV_CMD_SPATIAL_USER_1:
    case MAV_CMD_SPATIAL_USER_2:
    case MAV_CMD_SPATIAL_USER_3:
    case MAV_CMD_SPATIAL_USER_4:
    case MAV_CMD_SPATIAL_USER_5:
//    case MAV_CMD_USER_1:
    case MAV_CMD_USER_2:
    case MAV_CMD_USER_3:
    case MAV_CMD_USER_4:
    case MAV_CMD_USER_5:
        // TODO: add custom MAVLink handling work. Change to true for entries that get handled
        return MAV_RESULT_FAILED;

    case MAV_CMD_USER_1:
        lidar_M8_status = (int32_t)packet.param1;
        return MAV_RESULT_ACCEPTED;

    default:
        return MAV_RESULT_FAILED;
    }
}
#else
AP_UserCustom::AP_UserCustom() { }
void AP_UserCustom::update() { }
bool AP_UserCustom::arming_check(bool report) { return true; }  // do not inhibit arming
MAV_RESULT AP_UserCustom::handle_user_message(const mavlink_command_long_t &packet) { return MAV_RESULT_FAILED; } // not handled
#endif // !HAL_MINIMIZE_FEATURES

// singleton instance
AP_UserCustom *AP_UserCustom::_singleton;
namespace AP {
AP_UserCustom *usercustom() {
    return AP_UserCustom::get_singleton();
}};

