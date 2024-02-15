//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Admittance.cpp
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
#include "Admittance.h"
#include "rtwtypes.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmSetFirstInitCond
#define rtmSetFirstInitCond(rtm, val)  ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
#define rtmIsFirstInitCond(rtm)        ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

// private model entry point functions
extern void Admittance_derivatives();

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
void Admittance::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3]{
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3]{
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t { rtsiGetT(si) };

  time_T tnew { rtsiGetSolverStopTime(si) };

  time_T h { rtsiGetStepSize(si) };

  real_T *x { rtsiGetContStates(si) };

  ODE3_IntgData *id { static_cast<ODE3_IntgData *>(rtsiGetSolverData(si)) };

  real_T *y { id->y };

  real_T *f0 { id->f[0] };

  real_T *f1 { id->f[1] };

  real_T *f2 { id->f[2] };

  real_T hB[3];
  int_T i;
  int_T nXc { 5 };

  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  Admittance_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  Admittance_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  Admittance_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

// Model step function
void Admittance::step()
{
  real_T u0;
  if (rtmIsMajorTimeStep((&rtM))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&rtM)->solverInfo,(((&rtM)->Timing.clockTick0+1)*
      (&rtM)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&rtM))) {
    (&rtM)->Timing.t[0] = rtsiGetT(&(&rtM)->solverInfo);
  }

  if (rtmIsMajorTimeStep((&rtM))) {
    // DiscretePulseGenerator: '<S1>/Postion Generator'
    rtDW.PostionGenerator = ((rtDW.clockTickCounter < 1) &&
      (rtDW.clockTickCounter >= 0));

    // DiscretePulseGenerator: '<S1>/Postion Generator'
    if (rtDW.clockTickCounter >= 19) {
      rtDW.clockTickCounter = 0;
    } else {
      rtDW.clockTickCounter++;
    }

    // DiscretePulseGenerator: '<S1>/Velocity Generator'
    if (rtDW.clockTickCounter_c >= 19) {
      rtDW.clockTickCounter_c = 0;
    } else {
      rtDW.clockTickCounter_c++;
    }

    // End of DiscretePulseGenerator: '<S1>/Velocity Generator'

    // Gain: '<S1>/Damping Gain2'
    rtDW.DampingGain2 = 0.0;
  }

  // Integrator: '<S1>/Integrator'
  rtDW.Integrator = rtX.Integrator_CSTATE;

  // Sum: '<S1>/Add1' incorporates:
  //   Gain: '<S1>/Damping Gain'
  //   Integrator: '<S1>/Integrator1'
  //   Sum: '<S1>/Add'

  rtDW.Add1 = ((rtDW.PostionGenerator + rtDW.DampingGain2) - 0.0 *
               rtDW.Integrator) - rtX.Integrator1_CSTATE;

  // Outport: '<Root>/Desired Acceleration'
  rtY.DesiredAcceleration = rtDW.Add1;

  // Sum: '<S2>/Sum1' incorporates:
  //   Gain: '<S2>/Gain'
  //   TransferFcn: '<S2>/Electrical Admittance'

  rtDW.Tnet = 1724.1379310344828 * rtX.ElectricalAdmittance_CSTATE * 0.011;

  // Integrator: '<S2>/Integrator'
  if (rtDW.Integrator_IWORK != 0) {
    rtX.Integrator_CSTATE_n = 0.0;
  }

  // TransferFcn: '<S2>/Mechanical Admittance'
  rtDW.wmotorvelocity = 1.0E+6 * rtX.MechanicalAdmittance_CSTATE;

  // Sum: '<Root>/Sum' incorporates:
  //   Integrator: '<S1>/Integrator1'
  //   Integrator: '<S2>/Integrator'

  u0 = rtX.Integrator1_CSTATE - rtX.Integrator_CSTATE_n;

  // Saturate: '<Root>/Saturation'
  if (u0 > 12.0) {
    u0 = 12.0;
  } else if (u0 < -12.0) {
    u0 = -12.0;
  }

  // Sum: '<S2>/Sum' incorporates:
  //   Gain: '<S2>/Gain1'
  //   Saturate: '<Root>/Saturation'

  rtDW.vw = u0 - 0.011 * rtDW.wmotorvelocity;

  // Outport: '<Root>/Desired Velocity'
  rtY.DesiredVelocity = rtDW.Integrator;
  if (rtmIsMajorTimeStep((&rtM))) {
    // Update for Integrator: '<S2>/Integrator'
    rtDW.Integrator_IWORK = 0;
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep((&rtM))) {
    rt_ertODEUpdateContinuousStates(&(&rtM)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&rtM)->Timing.clockTick0;
    (&rtM)->Timing.t[0] = rtsiGetSolverStopTime(&(&rtM)->solverInfo);

    {
      // Update absolute timer for sample time: [0.5s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.5, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&rtM)->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void Admittance::Admittance_derivatives()
{
  Admittance::XDot *_rtXdot;
  _rtXdot = ((XDot *) (&rtM)->derivs);

  // Derivatives for Integrator: '<S1>/Integrator'
  _rtXdot->Integrator_CSTATE = rtDW.Add1;

  // Derivatives for Integrator: '<S1>/Integrator1'
  _rtXdot->Integrator1_CSTATE = rtDW.Integrator;

  // Derivatives for TransferFcn: '<S2>/Electrical Admittance'
  _rtXdot->ElectricalAdmittance_CSTATE = -2017.2413793103447 *
    rtX.ElectricalAdmittance_CSTATE;
  _rtXdot->ElectricalAdmittance_CSTATE += rtDW.vw;

  // Derivatives for Integrator: '<S2>/Integrator'
  _rtXdot->Integrator_CSTATE_n = rtDW.wmotorvelocity;

  // Derivatives for TransferFcn: '<S2>/Mechanical Admittance'
  _rtXdot->MechanicalAdmittance_CSTATE = 0.0 - rtX.MechanicalAdmittance_CSTATE;
  _rtXdot->MechanicalAdmittance_CSTATE += rtDW.Tnet;
}

// Model initialize function
void Admittance::initialize()
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&rtM)->solverInfo, &(&rtM)->Timing.simTimeStep);
    rtsiSetTPtr(&(&rtM)->solverInfo, &rtmGetTPtr((&rtM)));
    rtsiSetStepSizePtr(&(&rtM)->solverInfo, &(&rtM)->Timing.stepSize0);
    rtsiSetdXPtr(&(&rtM)->solverInfo, &(&rtM)->derivs);
    rtsiSetContStatesPtr(&(&rtM)->solverInfo, (real_T **) &(&rtM)->contStates);
    rtsiSetNumContStatesPtr(&(&rtM)->solverInfo, &(&rtM)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&(&rtM)->solverInfo, (boolean_T**) &(&rtM)
      ->contStateDisabled);
    rtsiSetErrorStatusPtr(&(&rtM)->solverInfo, (&rtmGetErrorStatus((&rtM))));
    rtsiSetRTModelPtr(&(&rtM)->solverInfo, (&rtM));
  }

  rtsiSetSimTimeStep(&(&rtM)->solverInfo, MAJOR_TIME_STEP);
  (&rtM)->intgData.y = (&rtM)->odeY;
  (&rtM)->intgData.f[0] = (&rtM)->odeF[0];
  (&rtM)->intgData.f[1] = (&rtM)->odeF[1];
  (&rtM)->intgData.f[2] = (&rtM)->odeF[2];
  (&rtM)->contStates = ((X *) &rtX);
  (&rtM)->contStateDisabled = ((XDis *) &rtXDis);
  (&rtM)->Timing.tStart = (0.0);
  rtsiSetSolverData(&(&rtM)->solverInfo, static_cast<void *>(&(&rtM)->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&(&rtM)->solverInfo, false);
  rtsiSetSolverName(&(&rtM)->solverInfo,"ode3");
  rtmSetTPtr((&rtM), &(&rtM)->Timing.tArray[0]);
  (&rtM)->Timing.stepSize0 = 0.5;
  rtmSetFirstInitCond((&rtM), 1);

  // InitializeConditions for Integrator: '<S1>/Integrator'
  rtX.Integrator_CSTATE = 0.0;

  // InitializeConditions for Integrator: '<S1>/Integrator1'
  rtX.Integrator1_CSTATE = 0.0;

  // InitializeConditions for TransferFcn: '<S2>/Electrical Admittance'
  rtX.ElectricalAdmittance_CSTATE = 0.0;

  // InitializeConditions for Integrator: '<S2>/Integrator'
  if (rtmIsFirstInitCond((&rtM))) {
    rtX.Integrator_CSTATE_n = 0.0;
  }

  rtDW.Integrator_IWORK = 1;

  // End of InitializeConditions for Integrator: '<S2>/Integrator'

  // InitializeConditions for TransferFcn: '<S2>/Mechanical Admittance'
  rtX.MechanicalAdmittance_CSTATE = 0.0;

  // set "at time zero" to false
  if (rtmIsFirstInitCond((&rtM))) {
    rtmSetFirstInitCond((&rtM), 0);
  }
}

// Root outports get method
const Admittance::ExtY &Admittance::getExternalOutputs() const
{
  return rtY;
}

// Constructor
Admittance::Admittance() :
  rtY(),
  rtDW(),
  rtX(),
  rtXDis(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
Admittance::~Admittance() = default;

// Real-Time Model get method
Admittance::RT_MODEL * Admittance::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
