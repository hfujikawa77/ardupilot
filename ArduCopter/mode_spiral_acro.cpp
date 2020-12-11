#include "Copter.h"

#if MODE_SPIRALACRO_ENABLED == ENABLED
/*
 * Init and run calls for spiral acro flight mode
 */

bool ModeSpiralAcro::init(bool ignore_checks)
{
    // Staus set Wait Arming
    _state = SPIRALACRO_RunWaitArming;

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeSpiralAcro::run()
{
    //Spiral acro stat machine
    switch( state() ){
        case SPIRALACRO_RunWaitArming:
            waitarming_run();
            break;
        case SPIRALACRO_RunTakeoff:
            takeoff_run();
            break;
        case SPIRALACRO_RunSpiral:
            spiral_run();
            break;
        case SPIRALACRO_RunAcro:
            acro_run();
            break;
        case SPIRALACRO_RunRtl:
            rtl_run();
            break;
        default:
            rtl_start();
            break;
    }
}

// takeoff_start
void ModeSpiralAcro::takeoff_start()
{
    //inisialize takeoff status
    _state = SPIRALACRO_RunTakeoff;

    copter.set_auto_armed(true);

    wp_nav->wp_and_spline_init();

    float takeoff_alt_cm = TakeOffAltcm;

    // initialise wpnav destination
    Location target_loc = copter.current_loc;
    Location::AltFrame frame = Location::AltFrame::ABOVE_HOME;
    if (wp_nav->rangefinder_used_and_healthy() &&
            wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
            takeoff_alt_cm < copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
        // can't takeoff downwards
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return;
        }
        frame = Location::AltFrame::ABOVE_TERRAIN;
    }
    target_loc.set_alt_cm(takeoff_alt_cm, frame);

    // set waypoint controller target
    if (!wp_nav->set_wp_destination(target_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();

}

// takeoff_start
void ModeSpiralAcro::spiral_start()
{
    //inisialize spiral status
    pilot_yaw_override = false;

    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    copter.circle_nav->set_radius( 100 );
    copter.circle_nav->init();

    _state = SPIRALACRO_RunSpiral;
}

void ModeSpiralAcro::acro_start()
{
    //inisialize acro status
    _state = SPIRALACRO_RunAcro;
    _state_submode = Submode_MoveAboveHP;

    wp_nav->wp_and_spline_init();

    Location above_hp_loc = ahrs.get_home();
    above_hp_loc.set_alt_cm(SpiralAltcm, Location::AltFrame::ABOVE_TERRAIN);

    wp_nav->set_speed_xy(100);

    if (!wp_nav->set_wp_destination(above_hp_loc)) {
        // failure to set destination can only be because of missing terrain data
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

}

void ModeSpiralAcro::rtl_start()
{
    // call regular rtl flight mode initialisation and ask it to ignore checks
    copter.mode_rtl.init(true);

    _state = SPIRALACRO_RunRtl;
}

// waitarming_run - in SPIRAL ACRO flight mode
//      called by auto_run at 100hz or more
void ModeSpiralAcro::waitarming_run()
{    

    // if not armed exit immediately    
    if (!motors->armed()){
        return;
    }

    // start takeoff automatically
    takeoff_start();

}

// takeoff_run - takeoff in SPIRAL ACRO flight mode
//      called by auto_run at 100hz or more
void ModeSpiralAcro::takeoff_run()
{
    auto_takeoff_run();

    // alt >= threshold(TakeOffAltcm)
    if (wp_nav->reached_wp_destination()) {
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();

        // move spiral status
        spiral_start();
    }
}

// _spiral_run - spiral in SPIRAL ACRO flight mode
//      called by auto_run at 100hz or more
void ModeSpiralAcro::spiral_run()
{
    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // Calc spiral radius and climb rate based on atltude
    int32_t height_cm = get_alt_above_ground_cm();
    float spiral_climb_rate = 150-height_cm/100;
    float spiral_radius = 1000-((height_cm-1000)/10);
    if(spiral_radius < 100){
        spiral_radius = 100;
    }

    // pilot changes to circle rate and radius
    // skip if in radio failsafe
    if (!copter.failsafe.radio && copter.circle_nav->pilot_control_enabled()) {
        // update the circle controller's radius target based on pilot pitch stick inputs
        const float radius_current = copter.circle_nav->get_radius();           // circle controller's radius target, which begins as the circle_radius parameter
        const float radius_new = MAX(spiral_radius,0);                          // new radius target

        if (!is_equal(radius_current, radius_new)) {
            copter.circle_nav->set_radius(radius_new);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run circle controller
    copter.failsafe_terrain_set_status(copter.circle_nav->update());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(copter.circle_nav->get_roll(),
                                                       copter.circle_nav->get_pitch(),
                                                       copter.circle_nav->get_yaw(), true);

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate(spiral_climb_rate, G_Dt, false);
    pos_control->update_z_controller();

    // alt >= threshold(SpiralAltcm)
    if(get_alt_above_ground_cm() >= SpiralAltcm){
        //acro start 
        acro_start();
    }
}

// _acro_run - acro in SPIRAL ACRO flight mode
//      called by auto_run at 100hz or more

#define FLIP_THR_INC        0.20f   // throttle increase during FlipState::Start stage (under 45deg lean angle)
#define FLIP_THR_DEC        0.24f   // throttle decrease during FlipState::Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROTATION_RATE  40000   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define FLIP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define FLIP_RECOVERY_ANGLE 500     // consider successful recovery when roll is back within 5 degrees of original

#define FLIP_ROLL_RIGHT      1      // used to set flip_dir
#define FLIP_ROLL_LEFT      -1      // used to set flip_dir

#define FLIP_PITCH_BACK      1      // used to set flip_dir
#define FLIP_PITCH_FORWARD  -1      // used to set flip_dir

void ModeSpiralAcro::acro_run()
{
    // get pilot's desired throttle
    float throttle_out = get_pilot_desired_throttle();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get corrected angle based on direction and axis of rotation
    // we flip the sign of flip_angle to minimize the code repetition
    int32_t flip_angle;
    flip_angle = ahrs.roll_sensor * FLIP_ROLL_RIGHT;

    switch(_state_submode){
        case Submode_MoveAboveHP:
            copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
            pos_control->update_z_controller();
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), channel_yaw->get_control_in());

            if (copter.wp_nav->reached_wp_destination()) {

                // log start of flip
                AP::logger().Write_Event(LogEvent::FLIP_START);

                // capture current attitude which will be used during the FlipState::Recovery stage
                const float angle_max = copter.aparm.angle_max;
                orig_attitude.x = constrain_float(ahrs.roll_sensor, -angle_max, angle_max);
                orig_attitude.y = constrain_float(ahrs.pitch_sensor, -angle_max, angle_max);
                orig_attitude.z = ahrs.yaw_sensor;

                // set next mode
                _state_submode = Submode_FlipStart;
            }
            break;
        case Submode_FlipStart:
            // under 45 degrees request 400deg/sec roll or pitch
            attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * FLIP_ROLL_RIGHT, FLIP_ROTATION_RATE * FLIP_PITCH_BACK, 0.0);

            // increase throttle
            throttle_out += FLIP_THR_INC;

            // beyond 45deg lean angle move to next stage
            if (flip_angle >= 4500) {
                _state_submode = Submode_FlipRoll;
            }
            break;
        case Submode_FlipRoll:
            // between 45deg ~ -90deg request 400deg/sec roll
            attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * FLIP_ROLL_RIGHT, 0.0, 0.0);
            // decrease throttle
            throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

            // beyond -90deg move on to recovery
            if ((flip_angle < 4500) && (flip_angle > -9000)) {
                _state_submode = Submode_FlipRecover;
            }
            break;
        case Submode_FlipRecover:
            // use originally captured earth-frame angle targets to recover
            attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

            // increase throttle to gain any lost altitude
            throttle_out += FLIP_THR_INC;

            float recovery_angle;
            // we are rolling
            recovery_angle = fabsf(orig_attitude.x - (float)ahrs.roll_sensor);

            // check for successful recovery
            if (fabsf(recovery_angle) <= FLIP_RECOVERY_ANGLE) {
                _state_submode = Submode_AcroEnd;                
            }
            break;
        default:
            //rtl start 
            rtl_start();
            break;
    }
}

// _rtl_run - rtl in SPIRAL ACRO flight mode
//      called by auto_run at 100hz or more
void ModeSpiralAcro::rtl_run()
{
    // call regular rtl flight mode run function
    copter.mode_rtl.run(false);

    // if disarmed finish set initial status    
    if (!motors->armed()){
        // Staus set Wait Arming
        _state = SPIRALACRO_RunWaitArming;
    }
}

uint32_t ModeSpiralAcro::wp_distance() const
{
    if( _state == SPIRALACRO_RunSpiral){
        return copter.circle_nav->get_distance_to_target();
    }else{
        return wp_nav->get_wp_distance_to_destination();
    }
}

int32_t ModeSpiralAcro::wp_bearing() const
{
    if( _state == SPIRALACRO_RunSpiral){
        return copter.circle_nav->get_bearing_to_target();
    }else{
        return wp_nav->get_wp_bearing_to_destination();
    }
}

bool ModeSpiralAcro::allows_arming(bool from_gcs) const
{
    // always allow arming from the ground station
    if (from_gcs) {
        return true;
    }

    // optionally allow arming from the transmitter
    return (copter.g2.spiralacro_options & (uint32_t)Options::AllowArmingFromTX) != 0;
};

#endif