#include "Copter.h"

#if MODE_SPIRALACRO_ENABLED == ENABLED
/*
 * Init and run calls for spiral acro flight mode
 */

bool ModeSpiralAcro::init(bool ignore_checks)
{
    //Arming(Takeoff) start 
    takeoff_start();

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeSpiralAcro::run()
{
    //Spiral acro stat machine
    switch( state() ){
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


// takeoff_run - takeoff in SPIRAL ACRO flight mode
//      called by auto_run at 100hz or more
void ModeSpiralAcro::takeoff_run()
{
    // alt >= threshold(TakeOffAltcm)
        //spirac start 
        spiral_start();
 
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

#endif