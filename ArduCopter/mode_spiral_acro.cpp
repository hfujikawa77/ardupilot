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

    _state = SPIRALACRO_RunSpiral;
}

void ModeSpiralAcro::acro_start()
{
    //inisialize acro status


    _state = SPIRALACRO_RunAcro;
}

void ModeSpiralAcro::rtl_start()
{
    //inisialize rtl status


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
    // set arm automatically
    copter.set_auto_armed(true);

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
    // alt >= threshold(SpiralAltcm)

        //acro start 
        acro_start();

}

// _acro_run - acro in SPIRAL ACRO flight mode
//      called by auto_run at 100hz or more
void ModeSpiralAcro::acro_run()
{
    
    // finish member's acrobat fright
        //rtl start 
        rtl_start();
}

// _rtl_run - rtl in SPIRAL ACRO flight mode
//      called by auto_run at 100hz or more
void ModeSpiralAcro::rtl_run()
{

}

uint32_t ModeSpiralAcro::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeSpiralAcro::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
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