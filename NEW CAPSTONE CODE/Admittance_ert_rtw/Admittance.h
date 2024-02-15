//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Admittance.h
//
// Code generated for Simulink model 'Admittance'.
//
// Model version                  : 1.65
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Wed Jan 24 15:37:53 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_Admittance_h_
#define RTW_HEADER_Admittance_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include <cstring>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
struct ODE3_IntgData {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
};

#endif

// Class declaration for model Admittance
class Admittance final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW {
    real_T PostionGenerator;           // '<S1>/Postion Generator'
    real_T DampingGain2;               // '<S1>/Damping Gain2'
    real_T Integrator;                 // '<S1>/Integrator'
    real_T Add1;                       // '<S1>/Add1'
    real_T Tnet;                       // '<S2>/Sum1'
    real_T wmotorvelocity;             // '<S2>/Mechanical Admittance'
    real_T vw;                         // '<S2>/Sum'
    int32_T clockTickCounter;          // '<S1>/Postion Generator'
    int32_T clockTickCounter_c;        // '<S1>/Velocity Generator'
    int_T Integrator_IWORK;            // '<S2>/Integrator'
  };

  // Continuous states (default storage)
  struct X {
    real_T Integrator_CSTATE;          // '<S1>/Integrator'
    real_T Integrator1_CSTATE;         // '<S1>/Integrator1'
    real_T ElectricalAdmittance_CSTATE;// '<S2>/Electrical Admittance'
    real_T Integrator_CSTATE_n;        // '<S2>/Integrator'
    real_T MechanicalAdmittance_CSTATE;// '<S2>/Mechanical Admittance'
  };

  // State derivatives (default storage)
  struct XDot {
    real_T Integrator_CSTATE;          // '<S1>/Integrator'
    real_T Integrator1_CSTATE;         // '<S1>/Integrator1'
    real_T ElectricalAdmittance_CSTATE;// '<S2>/Electrical Admittance'
    real_T Integrator_CSTATE_n;        // '<S2>/Integrator'
    real_T MechanicalAdmittance_CSTATE;// '<S2>/Mechanical Admittance'
  };

  // State disabled
  struct XDis {
    boolean_T Integrator_CSTATE;       // '<S1>/Integrator'
    boolean_T Integrator1_CSTATE;      // '<S1>/Integrator1'
    boolean_T ElectricalAdmittance_CSTATE;// '<S2>/Electrical Admittance'
    boolean_T Integrator_CSTATE_n;     // '<S2>/Integrator'
    boolean_T MechanicalAdmittance_CSTATE;// '<S2>/Mechanical Admittance'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    real_T DesiredAcceleration;        // '<Root>/Desired Acceleration'
    real_T DesiredVelocity;            // '<Root>/Desired Velocity'
  };

  // Real-time Model Data Structure
  struct RT_MODEL {
    const char_T *errorStatus;
    RTWSolverInfo solverInfo;
    X *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    XDis *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T CTOutputIncnstWithState;
    real_T odeY[5];
    real_T odeF[3][5];
    ODE3_IntgData intgData;

    //
    //  Sizes:
    //  The following substructure contains sizes information
    //  for many of the model attributes such as inputs, outputs,
    //  dwork, sample times, etc.

    struct {
      int_T numContStates;
      int_T numPeriodicContStates;
      int_T numSampTimes;
    } Sizes;

    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      uint32_T clockTick0;
      time_T stepSize0;
      uint32_T clockTick1;
      boolean_T firstInitCondFlag;
      time_T tStart;
      SimTimeStep simTimeStep;
      boolean_T stopRequestedFlag;
      time_T *t;
      time_T tArray[2];
    } Timing;
  };

  // Copy Constructor
  Admittance(Admittance const&) = delete;

  // Assignment Operator
  Admittance& operator= (Admittance const&) & = delete;

  // Move Constructor
  Admittance(Admittance &&) = delete;

  // Move Assignment Operator
  Admittance& operator= (Admittance &&) = delete;

  // Real-Time Model get method
  Admittance::RT_MODEL * getRTM();

  // External outputs
  ExtY rtY;

  // Root outports get method
  const ExtY &getExternalOutputs() const;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  Admittance();

  // Destructor
  ~Admittance();

  // private data and function members
 private:
  // Block states
  DW rtDW;

  // Block continuous states
  X rtX;

  // Block Continuous state disabled vector
  XDis rtXDis;

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void Admittance_derivatives();

  // Real-Time Model
  RT_MODEL rtM;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Control Torque Scope' : Unused code path elimination
//  Block '<Root>/Net Torque Scope' : Unused code path elimination
//  Block '<Root>/Postition Scope' : Unused code path elimination
//  Block '<Root>/Velocity Scope' : Unused code path elimination
//  Block '<S1>/Inverse Inertia Gain' : Eliminated nontunable gain of 1
//  Block '<S1>/Stiffness Gain ' : Eliminated nontunable gain of 1
//  Block '<S1>/Stifness Gain' : Eliminated nontunable gain of 1
//  Block '<Root>/Kp' : Eliminated nontunable gain of 1


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'Admittance'
//  '<S1>'   : 'Admittance/Admittance Controller'
//  '<S2>'   : 'Admittance/Motor Plant'

#endif                                 // RTW_HEADER_Admittance_h_

//
// File trailer for generated code.
//
// [EOF]
//
