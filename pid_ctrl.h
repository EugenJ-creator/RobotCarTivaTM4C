/*
 * @file pid_ctrl.h
 * @author stfwi
 * @license (what you want it to be)
 * @date 2005-09-07
 * ---
 * 2013-07-03 Removed D prefilter, added more documentation
 */
#ifndef __PID_CTRL_H__
#define __PID_CTRL_H__

 #include <stdint.h>
/**
 * You should define pidctl_n_t before you include this file.
 * This way you can decide which number data type you like to
 * use for the controller.
 */
#ifndef pidctl_n_t
#define pidctl_n_t float
#endif
 
 const float KP = 0.1;
 const float KI = 0.04;   // 0.02 was ok
 const float KD  = 0.08;
 #define OFFSET  750
 #define I_SATURATION  700
 #define SATURATION  1599
 
/**
 * The structure used to configure and run the closed loop
 * controller.
 */
typedef struct {
  pidctl_n_t Kp;            /* Proportional constant                */
  pidctl_n_t Ki;            /* Integral constant                    */
  pidctl_n_t Kd;            /* Differential constant                */
  pidctl_n_t offset;        /* Output offset                        */
  pidctl_n_t saturation;    /* Output saturation                    */
  pidctl_n_t i_saturation;  /* Saturation of the intrgrator         */
  pidctl_n_t i;             /* Buffer of the intrgrator             */
  pidctl_n_t e_last;        /* Last error for the differentiator    */
} pidctl_t;                 /* (filter for D removed)               */
 
 
/**
 * Resets the current values (integrator and last error).
 * It does not change the configuration.
 */
#define pidctl_reset(PID, E) {  					\
		(PID).Kp = KP;              					\
		(PID).Ki = KI;               					\
		(PID).Kd = KD;               					\
		(PID).Kd = KD;               					\
		(PID).offset = OFFSET;         				\
		(PID).saturation = SATURATION;    		\
		(PID).i_saturation = I_SATURATION;    \
    (PID).e_last = (E);         					\
}
 
/**
 * Calculates the new output value (saved in O) dependent on the
 * actual error (E) and the controller configuration/state (PID).
 */
#define pidctl(PID, E, O) {                     \
  (PID).i += (PID).Ki * (E);                    \
  if((PID).i > (PID).i_saturation) {            \
    (PID).i = (PID).i_saturation;               \
  } else if((PID).i < -(PID).i_saturation) {    \
    (PID).i = -(PID).i_saturation;              \
  }                                             \
  (O) = (E) - (PID).e_last;                     \
  /* stfwi: removed D input filter  */          \
  (O) *= (PID).Kd;                              \
  (O) += (PID).Kp * (E);                        \
  (O) += (PID).i;                               \
  (O) += (PID).offset;                          \
  if((O) > (PID).saturation) {                  \
    (O) = (PID).saturation;                     \
  } else if((O) < -(PID).saturation) {          \
    (O) = -(PID).saturation;                    \
  }                                             \
    (PID).e_last = (E);                           \
}
 

#endif





///*
// * @file pid_ctrl.h
// * @author stfwi
// * @license (what you want it to be)
// * @date 2005-09-07
// * ---
// * 2013-07-03 Removed D prefilter, added more documentation
// */
//#ifndef __PID_CTRL_H__
//#define __PID_CTRL_H__




///**
// * You should define pidctl_n_t before you include this file.
// * This way you can decide which number data type you like to
// * use for the controller.
// */
//#ifndef pidctl_n_t
//#define pidctl_n_t float
//#endif


///**
// * The structure used to configure and run the closed loop
// * controller.
// */
//typedef struct {
//  pidctl_n_t Kp;            /* Proportional constant                */
//  pidctl_n_t Ki;            /* Integral constant                    */
//  pidctl_n_t Kd;            /* Differential constant                */
//  pidctl_n_t offset;        /* Output offset                        */
//  pidctl_n_t saturation;    /* Output saturation                    */
//  pidctl_n_t i_saturation;  /* Saturation of the intrgrator         */
//  pidctl_n_t i;             /* Buffer of the intrgrator             */
//  pidctl_n_t e_last;        /* Last error for the differentiator    */
//} pidctl_t; 



//void pidctl_reset(pidctl_t PID, pidctl_n_t E); 



///**
// * Calculates the new output value (saved in O) dependent on the
// * actual error (E) and the controller configuration/state (PID).
// */
//void pidctl(pidctl_t PID, pidctl_n_t E , pidctl_n_t O);

//#endif

