#ifndef CONTROL_LOOP_HPP_
#define CONTROL_LOOP_HPP_
/**
 * @copyright This information is proprietary to Eaton.  All rights reserved.
 *
 * @file
 *
 * Class for configuring a Control Loop
 */
// Library header files
#include <math.h>
#include <stdlib.h>

//#include "converter_board.h"
#include "driverlib.h"
#include "converter_settings.h"

#include "DCL.h"

extern volatile float  PI_KP;
extern volatile float  PI_KI;


typedef float (*PI_Ctl_Func)( void );
static DCL_PI TI_Controller_Settings = { PI_KP, PI_KI, 0, PI_OUT_UP_SAT, PI_OUT_LOW_SAT, 0, PI_INTG_MAX, PI_INTG_MIN  };



/**
 * Control Loop class.
 */
class Control_Loop
{

public:
    Control_Loop(volatile float vOutRef, float dc_min, float dc_max)
    : m_vOutRef( vOutRef ),
      m_duty_min( dc_min ),
      m_duty_max( dc_max ),
      m_PISettings( TI_Controller_Settings )
      {
      }

public:
    float PI_Control( void ){return m_CtlFunc();}
    void setPIFunc( PI_Ctl_Func func ) { m_CtlFunc = func; }
    void setPIFuncParameters(DCL_PI &piInfo);
    float get_vOutRef(){ return m_vOutRef; }
    DCL_PI &get_PI_Settings() {return m_PISettings;}


private:
    /** Private storage for the outputs of the processing */
    float          m_current_command;
    float          m_current_command_duty;          // The Duty Cycle provided to the PWM
    float          m_vOutRef;
    float          m_duty_min;          // The minimum Duty Cycle (we scale 0 - 1.0)
    float          m_duty_max;          // The maximum Duty Cycle (we scale 0 - 1.0)
    PI_Ctl_Func    m_CtlFunc;           // The Control feedback function
    // PI control related data
    // Possibly not needed...May be within the generated code for the
#if 0
    float           m_Kp;       // Proportional gain
    float           m_Ki;       // Integral gain
    float           m_i10;      // I storage
    float           m_Umax;     // Upper control saturation limit
    float           m_Umin;     // Lower control saturation limit
    float           m_i6;       // Saturation storage
    float           m_Imax;     // Upper integrator saturation limit
    float           m_Imin;     // Lower integrator saturation limit
#endif
    DCL_PI &m_PISettings;

};


#endif /* CONTROL_LOOP_HPP_ */
