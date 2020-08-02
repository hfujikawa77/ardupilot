#include "Copter.h"

#if MODE_QUBE_ENABLED == ENABLED

/*
 * Init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// qube_init - initialise rtl controller
bool ModeQube::init(bool ignore_checks)
{
    if (!ignore_checks) {
        if (!AP::ahrs().home_is_set()) {
            return false;
        }
    }
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();
    _state = QUBE_Starting;
    _state_complete = true; // see run() method below
    terrain_following_allowed = !copter.failsafe.terrain;
    return true;
}

// re-start RTL with terrain following disabled
void ModeQube::restart_without_terrain()
{
    AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RESTARTED_RTL);
    terrain_following_allowed = false;
    _state = QUBE_Starting;
    _state_complete = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Restarting RTL - Terrain data missing");
}

ModeQube::QUBEAltType ModeQube::get_alt_type() const
{
    // sanity check parameter
    if (g.qube_alt_type < 0 || g.qube_alt_type > (int)QUBEAltType::QUBE_ALTTYPE_TERRAIN) {
        return QUBEAltType::QUBE_ALTTYPE_RELATIVE;
    }
    return (QUBEAltType)g.qube_alt_type.get();
}

// qube_run - runs the return-to-launch controller
// should be called at 100hz or more
void ModeQube::run(bool disarm_on_land)
{
    if (!motors->armed()) {
        return;
    }

    // check if we need to move to next state
    if (_state_complete) {
        switch (_state) {
        case QUBE_Starting:
            build_path();
            qube_start();
            break;
        case QUBE_InitialQube:
            climb_start();
            break;
        case QUBE_InitialClimb:
            return_start();
            break;
        case QUBE_ReturnHome:
            loiterathome_start();
            break;
        case QUBE_LoiterAtHome:
            if (qube_path.land || copter.failsafe.radio) {
                land_start();
            }else{
                descent_start();
            }
            break;
        case QUBE_FinalDescent:
            // do nothing
            break;
        case QUBE_Land:
            // do nothing - qube_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (_state) {

    case QUBE_Starting:
        // should not be reached:
        _state = QUBE_InitialClimb;
        FALLTHROUGH;

    case QUBE_InitialQube:
        qube_run();
        break;

    case QUBE_InitialClimb:
        climb_return_run();
        break;

    case QUBE_ReturnHome:
        climb_return_run();
        break;

    case QUBE_LoiterAtHome:
        loiterathome_run();
        break;

    case QUBE_FinalDescent:
        descent_run();
        break;

    case QUBE_Land:
        land_run(disarm_on_land);
        break;
    }
}

// qube_climb_start - initialise climb to RTL altitude
void ModeQube::climb_start()
{
    _state = QUBE_InitialClimb;
    _state_complete = false;

    // QUBE_SPEED == 0 means use WPNAV_SPEED
    if (g.qube_speed_cms != 0) {
        wp_nav->set_speed_xy(g.qube_speed_cms);
    }

    // set the destination
    if (!wp_nav->set_wp_destination(qube_path.climb_target)) {
        // this should not happen because qube_build_path will have checked terrain data was available
        gcs().send_text(MAV_SEVERITY_CRITICAL,"RTL: unexpected error setting climb target");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        copter.set_mode(Mode::Number::LAND, ModeReason::TERRAIN_FAILSAFE);
        return;
    }
    wp_nav->set_fast_waypoint(true);

    // hold current yaw during initial climb
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// qube_qube_start - initialise qube
void ModeQube::qube_start()
{
    _state = QUBE_InitialQube;
    _state_complete = false;

    // qube_SPEED == 0 means use WPNAV_SPEED
    if (g.qube_speed_cms != 0) {
        wp_nav->set_speed_xy(g.qube_speed_cms);
    }

    // set the destination
    if (!wp_nav->set_wp_destination(qube_path.qube_targets[qube_path.qube_index])) {
        // this should not happen because qube_build_path will have checked terrain data was available
        gcs().send_text(MAV_SEVERITY_CRITICAL,"RTL: unexpected error setting climb target");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        copter.set_mode(Mode::Number::LAND, ModeReason::TERRAIN_FAILSAFE);
        return;
    }
    wp_nav->set_fast_waypoint(true);

    // hold current yaw during initial climb
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// qube_return_start - initialise return to home
void ModeQube::return_start()
{
    _state = QUBE_ReturnHome;
    _state_complete = false;

    if (!wp_nav->set_wp_destination(qube_path.return_target)) {
        // failure must be caused by missing terrain data, restart RTL
        restart_without_terrain();
    }

    // initialise yaw to point home (maybe)
    auto_yaw.set_mode_to_default(true);
}

// qube_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by qube_run at 100hz or more
void ModeQube::climb_return_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(),true);
    }

    // check if we've completed this stage of RTL
    _state_complete = wp_nav->reached_wp_destination();
}

// qube_qube_run
//      called by qube_run at 100hz or more
void ModeQube::qube_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(),true);
    }

    // check if we've completed this stage of RTL
    if (wp_nav->reached_wp_destination()) {
        if (qube_path.qube_index == sizeof(qube_path.qube_targets)/sizeof(qube_path.qube_targets[0]) -1 ) { // reach last index
            _state_complete = true;
            gcs().send_text(MAV_SEVERITY_DEBUG, "QUBE: reach last target");
        } else {
            qube_path.qube_index++;
            gcs().send_text(MAV_SEVERITY_DEBUG, "QUBE: next target[%d]", qube_path.qube_index);
            qube_start();
        }
    }
}

// qube_loiterathome_start - initialise return to home
void ModeQube::loiterathome_start()
{
    _state = QUBE_LoiterAtHome;
    _state_complete = false;
    _loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if(auto_yaw.default_mode(true) != AUTO_YAW_HOLD) {
        auto_yaw.set_mode(AUTO_YAW_RESETTOARMEDYAW);
    } else {
        auto_yaw.set_mode(AUTO_YAW_HOLD);
    }
}

// qube_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by qube_run at 100hz or more
void ModeQube::loiterathome_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(),true);
    }

    // check if we've completed this stage of RTL
    if ((millis() - _loiter_start_time) >= (uint32_t)g.qube_loiter_time.get()) {
        if (auto_yaw.mode() == AUTO_YAW_RESETTOARMEDYAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            if (abs(wrap_180_cd(ahrs.yaw_sensor-copter.initial_armed_bearing)) <= 200) {
                _state_complete = true;
            }
        } else {
            // we have loitered long enough
            _state_complete = true;
        }
    }
}

// qube_descent_start - initialise descent to final alt
void ModeQube::descent_start()
{
    _state = QUBE_FinalDescent;
    _state_complete = false;

    // Set wp navigation target to above home
    loiter_nav->init_target(wp_nav->get_wp_destination());

    // initialise altitude target to stopping point
    pos_control->set_target_to_stopping_point_z();

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
}

// qube_descent_run - implements the final descent to the qube_ALT
//      called by qube_run at 100hz or more
void ModeQube::descent_run()
{
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw_rate = 0.0f;

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // process pilot's input
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!copter.set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

            // record if pilot has overridden roll or pitch
            if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // process roll, pitch inputs
    loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);

    // run loiter controller
    loiter_nav->update();

    // call z-axis position controller
    pos_control->set_alt_target_with_slew(qube_path.descent_target.alt, G_Dt);
    pos_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

    // check if we've reached within 20cm of final altitude
    _state_complete = labs(qube_path.descent_target.alt - copter.current_loc.alt) < 20;
}

// qube_loiterathome_start - initialise controllers to loiter over home
void ModeQube::land_start()
{
    _state = QUBE_Land;
    _state_complete = false;

    // Set wp navigation target to above home
    loiter_nav->init_target(wp_nav->get_wp_destination());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
}

bool ModeQube::is_landing() const
{
    return _state == QUBE_Land;
}

// qube_returnhome_run - return home
//      called by qube_run at 100hz or more
void ModeQube::land_run(bool disarm_on_land)
{
    // check if we've completed this stage of RTL
    _state_complete = copter.ap.land_complete;

    // disarm when the landing detector says we've landed
    if (disarm_on_land && copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    land_run_horizontal_control();
    land_run_vertical_control();
}

void ModeQube::build_path()
{
    // origin point is our stopping point
    Vector3f stopping_point, qube_point;
    pos_control->get_stopping_point_xy(stopping_point);
    pos_control->get_stopping_point_z(stopping_point);
    qube_path.origin_point = Location(stopping_point);
    qube_path.origin_point.change_alt_frame(Location::AltFrame::ABOVE_HOME);

    // compute return target
    compute_return_target();

    // climb target is above our origin point at the return altitude
    qube_path.climb_target = Location(qube_path.origin_point.lat, qube_path.origin_point.lng, qube_path.return_target.alt, qube_path.return_target.get_alt_frame());

    // descent target is below return target at qube_alt_final
    qube_path.descent_target = Location(qube_path.return_target.lat, qube_path.return_target.lng, g.qube_alt_final, Location::AltFrame::ABOVE_HOME);

    // set land flag
    qube_path.land = g.qube_alt_final <= 0;

    qube_path.qube_index = 0;

    const float qube_side_length = 100 * 100; // 立方体の1辺の長さ(100m)

    if (qube_path.origin_point.get_vector_from_origin_NEU(qube_point)) {
        qube_point.x += qube_side_length;
        qube_path.qube_targets[0] = Location(qube_point);
        qube_point.y += qube_side_length;
        qube_path.qube_targets[1] = Location(qube_point);
        qube_point.x -= qube_side_length;
        qube_path.qube_targets[2] = Location(qube_point);
        qube_point.y -= qube_side_length;
        qube_path.qube_targets[3] = Location(qube_point);
        qube_point.z += qube_side_length;
        qube_path.qube_targets[4] = Location(qube_point);
        qube_point.x += qube_side_length;
        qube_path.qube_targets[5] = Location(qube_point);
        qube_point.y += qube_side_length;
        qube_path.qube_targets[6] = Location(qube_point);
        qube_point.x -= qube_side_length;
        qube_path.qube_targets[7] = Location(qube_point);
        qube_point.y -= qube_side_length;
        qube_path.qube_targets[8] = Location(qube_point);
        qube_point.z -= qube_side_length;
        qube_path.qube_targets[9] = Location(qube_point);
    } else {
        //AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "QUBE: get_vector_from_origin_NEU failed...");
    }
}

// compute the return target - home or rally point
//   return target's altitude is updated to a higher altitude that the vehicle can safely return at (frame may also be set)
void ModeQube::compute_return_target()
{
    // set return target to nearest rally point or home position (Note: alt is absolute)
#if AC_RALLY == ENABLED
    qube_path.return_target = copter.rally.calc_best_rally_or_home_location(copter.current_loc, ahrs.get_home().alt);
#else
    qube_path.return_target = ahrs.get_home();
#endif

    // curr_alt is current altitude above home or above terrain depending upon use_terrain
    int32_t curr_alt = copter.current_loc.alt;

    // determine altitude type of return journey (alt-above-home, alt-above-terrain using range finder or alt-above-terrain using terrain database)
    ReturnTargetAltType alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE;
    if (terrain_following_allowed && (get_alt_type() == QUBEAltType::QUBE_ALTTYPE_TERRAIN)) {
        // convert qube_ALT_TYPE and WPNAV_RFNG_USE parameters to ReturnTargetAltType
        switch (wp_nav->get_terrain_source()) {
        case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE;
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::QUBE_MISSING_RNGFND);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RANGEFINDER;
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_TERRAINDATABASE;
            break;
        }
    }

    // set curr_alt and return_target.alt from range finder
    if (alt_type == ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RANGEFINDER) {
        if (copter.get_rangefinder_height_interpolated_cm(curr_alt)) {
            // set return_target.alt
            qube_path.return_target.set_alt_cm(MAX(curr_alt + MAX(0, g.qube_climb_min), MAX(g.qube_altitude, QUBE_ALT_MIN)), Location::AltFrame::ABOVE_TERRAIN);
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: rangefinder unhealthy, using alt-above-home");
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::QUBE_MISSING_RNGFND);
        }
    }

    // set curr_alt and return_target.alt from terrain database
    if (alt_type == ReturnTargetAltType::RETURN_TARGET_ALTTYPE_TERRAINDATABASE) {
        // set curr_alt to current altitude above terrain
        // convert return_target.alt from an abs (above MSL) to altitude above terrain
        //   Note: the return_target may be a rally point with the alt set above the terrain alt (like the top of a building)
        int32_t curr_terr_alt;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt) &&
            qube_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
            curr_alt = curr_terr_alt;
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE;
            AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
        }
    }

    // for the default case we must convert return-target alt (which is an absolute alt) to alt-above-home
    if (alt_type == ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE) {
        if (!qube_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
            // this should never happen but just in case
            qube_path.return_target.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
            gcs().send_text(MAV_SEVERITY_WARNING, "RTL: unexpected error calculating target alt");
        }
    }

    // set new target altitude to return target altitude
    // Note: this is alt-above-home or terrain-alt depending upon qube_alt_type
    // Note: ignore negative altitudes which could happen if user enters negative altitude for rally point or terrain is higher at rally point compared to home
    int32_t target_alt = MAX(qube_path.return_target.alt, 0);

    // increase target to maximum of current altitude + climb_min and rtl altitude
    target_alt = MAX(target_alt, curr_alt + MAX(0, g.qube_climb_min));
    target_alt = MAX(target_alt, MAX(g.qube_altitude, QUBE_ALT_MIN));

    // reduce climb if close to return target
    float qube_return_dist_cm = qube_path.return_target.get_distance(qube_path.origin_point) * 100.0f;
    // don't allow really shallow slopes
    if (g.qube_cone_slope >= QUBE_MIN_CONE_SLOPE) {
        target_alt = MAX(curr_alt, MIN(target_alt, MAX(qube_return_dist_cm*g.qube_cone_slope, curr_alt+QUBE_ABS_MIN_CLIMB)));
    }

    // set returned target alt to new target_alt (don't change altitude type)
    qube_path.return_target.set_alt_cm(target_alt, (alt_type == ReturnTargetAltType::RETURN_TARGET_ALTTYPE_RELATIVE) ? Location::AltFrame::ABOVE_HOME : Location::AltFrame::ABOVE_TERRAIN);

#if AC_FENCE == ENABLED
    // ensure not above fence altitude if alt fence is enabled
    // Note: because the qube_path.climb_target's altitude is simply copied from the return_target's altitude,
    //       if terrain altitudes are being used, the code below which reduces the return_target's altitude can lead to
    //       the vehicle not climbing at all as RTL begins.  This can be overly conservative and it might be better
    //       to apply the fence alt limit independently on the origin_point and return_target
    if ((copter.fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        // get return target as alt-above-home so it can be compared to fence's alt
        if (qube_path.return_target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt)) {
            float fence_alt = copter.fence.get_safe_alt_max()*100.0f;
            if (target_alt > fence_alt) {
                // reduce target alt to the fence alt
                qube_path.return_target.alt -= (target_alt - fence_alt);
            }
        }
    }
#endif

    // ensure we do not descend
    qube_path.return_target.alt = MAX(qube_path.return_target.alt, curr_alt);
}

bool ModeQube::get_wp(Location& destination)
{
    // provide target in states which use wp_nav
    switch (_state) {
    case QUBE_Starting:
    case QUBE_InitialQube:
    case QUBE_InitialClimb:
    case QUBE_ReturnHome:
    case QUBE_LoiterAtHome:
    case QUBE_FinalDescent:
        return wp_nav->get_oa_wp_destination(destination);
    case QUBE_Land:
        return false;
    }

    // we should never get here but just in case
    return false;
}

uint32_t ModeQube::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeQube::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

#endif
