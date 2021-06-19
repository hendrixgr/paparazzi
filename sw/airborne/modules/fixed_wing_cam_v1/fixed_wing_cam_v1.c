/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 * Modified and expanded to show the coordinates of where the camera is looking at, locking function
 * and other useful functions by Chris Efstathiou 5-April-2013 AD ~4.5 billion years after creation.
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *   file my_cam_v4_0.c
 *   Pan/Tilt camera library
 *
 */

#include <math.h>
#include "fixed_wing_cam_v1.h"
#include "subsystems/navigation/common_nav.h"
#include "firmwares/fixedwing/nav.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "state.h"
#include "modules/multi/traffic_info.h"
#include "subsystems/gps.h"
#include "math/pprz_geodetic_float.h"
#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/imu.h"

#ifndef CONCAT1
#define CONCAT1(a, b)           (a##b)
#endif
#ifndef CONCAT
#define CONCAT(a, b)            CONCAT1(a, b)
#endif

#define MIN_PPRZ_CAM ((int16_t)(MAX_PPRZ * 0.05))
#define DELTA_ALPHA 0.2


#if defined(CAM_PAN_RADIO_INPUT_CHANNEL) && CAM_PAN_RADIO_INPUT_CHANNEL != -1
#define CAM_PAN_RADIO_CH    CONCAT(RADIO_, CAM_PAN_RADIO_INPUT_CHANNEL)
#endif

#if defined(CAM_TILT_RADIO_INPUT_CHANNEL) && CAM_TILT_RADIO_INPUT_CHANNEL != -1
#define CAM_TILT_RADIO_CH   CONCAT(RADIO_, CAM_TILT_RADIO_INPUT_CHANNEL)
#endif

#if defined(CAM_ZOOM_RADIO_INPUT_CHANNEL) && CAM_ZOOM_RADIO_INPUT_CHANNEL != -1
#define CAM_ZOOM_RADIO_CH   CONCAT(RADIO_, CAM_ZOOM_RADIO_INPUT_CHANNEL)
#endif

#if defined(CAM_LOCK_RADIO_INPUT_CHANNEL) && CAM_LOCK_RADIO_INPUT_CHANNEL != -1 && !defined(RADIO_CONTROL_TYPE_DATALINK)
#define CAM_LOCK_RADIO_CH   CONCAT(RADIO_, CAM_LOCK_RADIO_INPUT_CHANNEL)
#endif

// CAMERA CONTROL OUTPUT SECTION
#if defined(CAM_PAN_OUTPUT_COMMAND) && CAM_PAN_OUTPUT_COMMAND != -1
#define CAM_PAN_COMMAND     CONCAT(COMMAND_, CAM_PAN_OUTPUT_COMMAND)
#else
#define CAM_PAN_NOT_AVAILABLE 1
#endif

#if defined(CAM_TILT_OUTPUT_COMMAND) && CAM_TILT_OUTPUT_COMMAND != -1
#define CAM_TILT_COMMAND    CONCAT(COMMAND_, CAM_TILT_OUTPUT_COMMAND)
#else
#warning Cannot control CAMERA TILT
#endif

#if defined(CAM_ZOOM_OUTPUT_COMMAND) && CAM_ZOOM_OUTPUT_COMMAND != -1
#define CAM_ZOOM_COMMAND    CONCAT(COMMAND_, CAM_ZOOM_OUTPUT_COMMAND)
#endif

// ERROR CHECKING
#if !defined(CAM_PAN_COMMAND) && !defined(CAM_TILT_COMMAND)
#error NO WAY TO CONTROL CAMERA PAN AND TILT
#endif




/**********************************************************************************************/
/******************************* VARIABLE DECLARATION *****************************************/
/**********************************************************************************************/

typedef struct {
  float fx;
  float fy;
  float fz;
} VECTOR;

typedef struct {
  float fx1; float fx2; float fx3;
  float fy1; float fy2; float fy3;
  float fz1; float fz2; float fz3;
} MATRIX;

bool cam_lock = 0;
bool wgs84_dms = false;
bool camera_zoom_lock = 0;

uint8_t cam_target_wp;
uint8_t cam_mode;

long   cam_pan_lp_sum = 0;
long   cam_tilt_lp_sum = 0;
int    cam_pan_filtered = 0;
int    cam_tilt_filtered = 0;

float mag_heading = 0;

#if defined(USE_JOYSTICK_FOR_CAMERA) && !defined(RADIO_CONTROL_TYPE_DATALINK)
int8_t joystick_cam_pan_val = 0;
int8_t joystick_cam_tilt_val = 0;
int8_t joystick_cam_zoom_val = 0;
#endif

int16_t camera_zoom_pprz_val = 0;

uint16_t cam_point_distance_from_home;
float cam_point_lon,  cam_point_lat;

float cam_pan_c = 0;
float cam_tilt_c = 0;
float cam_phi_c;
float cam_theta_c;
float cam_target_x, cam_target_y, cam_target_alt;


/**********************************************************************************************/
/******************************** FUNCTION PROTOTYPES *****************************************/
/**********************************************************************************************/
static void cam_angles(void);

static inline void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC);

static inline void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC);

static inline void wgs84_to_wgs84dms(void);

static void point_cam(float fObjectEast, float fObjectNorth, float fAltitude);

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"
#endif

/**********************************************************************************************/
/******************************* FUNCTION DEFINITIONS *****************************************/
/**********************************************************************************************/

#if DOWNLINK
static void send_cam_point(struct transport_tx *trans, struct link_device *dev)
{

  pprz_msg_send_CAM_POINT(trans, dev, AC_ID, &cam_point_distance_from_home, &cam_point_lat, &cam_point_lon);

  return;
}

static void send_cam(struct transport_tx *trans, struct link_device *dev)
{

  int16_t phi_angle = DegOfRad(cam_phi_c);
  int16_t theta_angle = DegOfRad(cam_theta_c);
  int16_t camlock = cam_lock;
#if defined(USE_JOYSTICK_FOR_CAMERA) && !defined(RADIO_CONTROL_TYPE_DATALINK)
  int16_t camzoom = joystick_cam_zoom_val;
#else
  int16_t camzoom = camera_zoom_pprz_val;
#endif
  pprz_msg_send_CAM(trans, dev, AC_ID, &phi_angle, &theta_angle, &camlock, &camzoom);

  return;
}

#endif



// Initialization of the camera module
void fixed_wing_cam_init(void)
{
  cam_mode = CAM_MODE_OFF;
#if defined(USE_JOYSTICK_FOR_CAMERA) && !defined(RADIO_CONTROL_TYPE_DATALINK)
  joystick_cam_pan_val = 0;
  joystick_cam_tilt_val = 0;
  joystick_cam_zoom_val = 0;
#endif
  camera_zoom_pprz_val = 0;

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CAM, send_cam);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CAM_POINT, send_cam_point);
#endif

  return;
}

// Periodic function called with a frequency defined in the module .xml file
void fixed_wing_cam_periodic(void)
{

  if (autopilot.mode == AP_MODE_AUTO2) {
    switch (cam_mode) {
      case CAM_MODE_OFF:
        cam_pan_c = RadOfDeg(CAM_PAN0);
        cam_tilt_c = RadOfDeg(CAM_TILT0);
        cam_angles();
        break;
      case CAM_MODE_NADIR:
        cam_pan_c = 0;
        cam_tilt_c = 0;
        cam_angles();
        break;
      case CAM_MODE_ANGLES:
        cam_angles();
        break;
      case CAM_MODE_WP_HOME:
        if (cam_target_wp < nb_waypoint) {
          point_cam(waypoints[WP_HOME].x, waypoints[WP_HOME].y, ground_alt);
          cam_angles();
        }
        break;
      case CAM_MODE_WP_TARGET:
        if (cam_target_wp < nb_waypoint) {
#if defined(WP_CAM_POINT)
          point_cam(waypoints[cam_target_wp].x, waypoints[cam_target_wp].y, waypoints[WP_CAM_POINT].a);
#else
          point_cam(waypoints[cam_target_wp].x, waypoints[cam_target_wp].y, ground_alt);
#endif
          cam_angles();
        }
        break;
      // In this mode the angles come from the pan and tilt radio channels.
      // The "TARGET" waypoint coordinates are not used the camera servos are almost
      //directly driven. I could pass the radio values directly to the servos but this
      // way i can also use this mode for calibration.
      // 0 as an argument means that this value is not used or internally calculated.
      case CAM_MODE_MANUAL:
        point_cam(0, 0, 0);
        cam_angles();
        break;
      // In this mode the pan/tilt radio channels are used to calculate the coordinates
      // of a target and then the camera tracks this target. For simplicity everything
      // radio channels. It also shows the coordinates of where the camera points to and
      // can also lock on to a target.
      // 0 as an argument means that this value is not used or internally calculated.
      case CAM_MODE_TRACK:
#if defined(WP_CAM_POINT)
        point_cam(0, 0, waypoints[WP_CAM_POINT].a);
#else
        point_cam(0, 0, ground_alt);
#endif
        cam_angles();
        break;
      default:
        break;
    }

  } else {
    if (gps.fix == GPS_FIX_3D && cam_mode == CAM_MODE_WP_HOME) {
      cam_mode = CAM_MODE_WP_HOME;
      point_cam(waypoints[WP_HOME].x, waypoints[WP_HOME].y, ground_alt);
      cam_angles();

    } else {
      cam_point_lon = 0;
      cam_point_lat = 0;
      cam_point_distance_from_home = 0;
      //Position the camera for straight view.
      cam_tilt_c = RadOfDeg(CAM_TILT_POSITION_FOR_FPV);
      cam_pan_c = RadOfDeg(CAM_PAN_POSITION_FOR_FPV);
      cam_angles();
    }

  }


  return;
}



/** Computes the servo values from cam_pan_c and cam_tilt_c */
static void cam_angles(void)
{

  float cam_pan = 0;
  float cam_tilt = 0;

  if (cam_pan_c > RadOfDeg(CAM_PAN_MAX)) {
    cam_pan_c = RadOfDeg(CAM_PAN_MAX);
  } else {
    if (cam_pan_c < RadOfDeg(CAM_PAN_MIN)) {
      cam_pan_c = RadOfDeg(CAM_PAN_MIN);
    }
  }

  if (cam_tilt_c > RadOfDeg(CAM_TILT_MAX)) {
    cam_tilt_c = RadOfDeg(CAM_TILT_MAX);
  } else {
    if (cam_tilt_c < RadOfDeg(CAM_TILT_MIN)) {
      cam_tilt_c = RadOfDeg(CAM_TILT_MIN);
    }
  }

#ifdef CAM_PAN_NEUTRAL
  float pan_diff = cam_pan_c - RadOfDeg(CAM_PAN_NEUTRAL);
  if (pan_diff > 0) {
    cam_pan = (float)MAX_PPRZ * (pan_diff / (float)(RadOfDeg(CAM_PAN_MAX - CAM_PAN_NEUTRAL)));
  } else {
    cam_pan = (float)MIN_PPRZ * (pan_diff / (float)(RadOfDeg(CAM_PAN_MIN - CAM_PAN_NEUTRAL)));
  }
#else
  cam_pan = ((float)RadOfDeg(cam_pan_c - CAM_PAN_MIN)) * ((float)MAX_PPRZ / (float)RadOfDeg(CAM_PAN_MAX - CAM_PAN_MIN));
#endif

#ifdef CAM_TILT_NEUTRAL
  float tilt_diff = cam_tilt_c - RadOfDeg(CAM_TILT_NEUTRAL);
  if (tilt_diff > 0) {
    cam_tilt = (float)MAX_PPRZ * (tilt_diff / (float)(RadOfDeg(CAM_TILT_MAX - CAM_TILT_NEUTRAL)));
  } else {
    cam_tilt = (float)MIN_PPRZ * (tilt_diff / (float)(RadOfDeg(CAM_TILT_MIN - CAM_TILT_NEUTRAL)));
  }
#else
  cam_tilt = ((float)RadOfDeg(cam_tilt_c - CAM_TILT_MIN))  * ((float)MAX_PPRZ / (float)RadOfDeg(
               CAM_TILT_MAX - CAM_TILT_MIN));
#endif

  // Load the servo pwm variables.
  cam_pan = TRIM_PPRZ(cam_pan);
  cam_tilt = TRIM_PPRZ(cam_tilt);
#ifdef CAM_PAN_COMMAND
  ap_state->commands[CAM_PAN_COMMAND] = cam_pan;
#endif
#ifdef CAM_TILT_COMMAND
  ap_state->commands[CAM_TILT_COMMAND] = cam_tilt;
#endif
  // Load the message variables (camera angles "CAM").
  cam_phi_c = cam_pan_c;
  cam_theta_c = cam_tilt_c;

  return;
}

/*******************************************************************
; function name:   vSubtractVectors
; description:     subtracts two vectors a = b - c
; parameters:
;*******************************************************************/
static inline void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC)
{
  svA->fx = svB.fx - svC.fx;
  svA->fy = svB.fy - svC.fy;
  svA->fz = svB.fz - svC.fz;
}

/*******************************************************************
; function name:   vMultiplyMatrixByVector
; description:     multiplies matrix by vector svA = smB * svC
; parameters:
;*******************************************************************/
static inline void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC)
{
  svA->fx = smB.fx1 * svC.fx  +  smB.fx2 * svC.fy  +  smB.fx3 * svC.fz;
  svA->fy = smB.fy1 * svC.fx  +  smB.fy2 * svC.fy  +  smB.fy3 * svC.fz;
  svA->fz = smB.fz1 * svC.fx  +  smB.fz2 * svC.fy  +  smB.fz3 * svC.fz;
}

static inline void wgs84_to_wgs84dms(void)
{
  // Conversion of decimal degrees to degree, minutes, seconds.
  float lat_long_buf = 0;
  float mem1 = 0;
  uint8_t minutes = 0;
  float seconds = 0;

  mem1 =  cam_point_lon - (int16_t)cam_point_lon;
  mem1 *= 60.;
  minutes = (uint8_t)mem1;
  mem1 =  mem1 - (int16_t)mem1;
  mem1 *= 60.;
  seconds = mem1;
  lat_long_buf = ((int16_t)cam_point_lon) * 10000;
  lat_long_buf += minutes * 100;
  lat_long_buf += seconds;
  cam_point_lon = (float)(lat_long_buf / 10000.);

  mem1 =  cam_point_lat - (int16_t)cam_point_lat;
  mem1 *= 60.;
  minutes = (uint8_t)mem1;
  mem1 =  mem1 - (uint8_t)mem1;
  mem1 *= 60.;
  seconds = mem1;
  lat_long_buf = ((int16_t)cam_point_lat) * 10000;
  lat_long_buf += minutes * 100;
  lat_long_buf += seconds;
  cam_point_lat = (float)(lat_long_buf / 10000.);

  return;
}

/*******************************************************************
; function name:   point_cam
; description:     Transforms ground coordinate system into
;                  plane's coordinate system via three rotations
;                  and determines positions of camera servos.
; parameters:      fPlaneNorth, fPlaneEast, fPlaneAltitude  plane's
;                           position with respect to ground
;                           in m (actually the units do not matter as
;                           long as they are the same as for the object's
;                           position)
;                  fRollAngle  level=0; right wing down = positive values
;                  fPitchAngle level=0; nose up = positive values
;                           plane's pitch and roll angles
;                           with respect to ground in radians
;                  fYawAngle   north=0; right= positive values in radians
;                           plane's yaw angle with respect to north
;                  fObjectNorth, fObjectEast, fAltitude object's
;                           position with respect to ground
;                           in m (actually the units do not matter as
;                           long as they are the same for the plane's
;                           position)
;                  fPan, fTilt angles for camera servos in radians,
;                           pan is turn/left-right and tilt is down-up
;                           in reference to the camera picture
;
; IN THIS IMPLEMENTATION ONLY "fObjectNorth", "fObjectEast", "fAltitude" ARE USED
; AS ALL THE OTHERS ARE TAKEN FROM THE STATE INTERFACE DIRECTLY
; ALSO THIS FUNCTION TAKES AN INPUT FROM THE RC RADIO OR A JOYSTICK AND CALCULATES
; A CAMERA POINT IN RELATION TO THE AIRPLANE'S BODY.
;
; camera mount:    The way the camera is mounted is given through a
;                  define POINT_CAM_a_[_b] where a gives the mount
;                  angle within the aircraft and b the angle when
;                  viewing the direction of the first servo.
;
;SERVOS                                FOR BEST RESULTS:
;                  USE PROGRAMMABLE SERVOS LIKE THE HYPERION ATLAS SO YOU CAN EASILY
;                  CALIBRATE THE CAMERA ANGLES.
;                  FIRST PROGRAM THE SERVOS FOR THE WIDEST ARM DEFLECTION AND THEN
;                  PROGRAM THE SERVOS CENTER VALUE LIKE IN THE EXAMPLE BELOW AFTER YOU
;                  CENTER THEM MECHANICALLY THE BEST YOU CAN!!!
;                  EXAMPLE 1: The servo needs a pulse of 1480us in order to be centered.
;                  CENTER VALUE = 1500 + (1500-1480) = 1520us
;                  EXAMPLE 2: The servo needs a pulse of 1520us in order to be centered.
;                  CENTER VALUE = 1500 + (1500-1520) = 1480us
;                  OF COURSE YOU CAN ALTERNATIVELY USE THE MEASURED SERVO CENTER VALUES (1520 or 1480)
;                  AS THE NEUTRAL VALUES IN THE AIRFRAME FILE BUT I FOUND THAT THIS DOES NOT
;                  GIVE THE BEST RESULTS.
;                  NEXT WITH A SERVO TESTER FIND THE SERVO PULSE WIDTH REQUIRED FOR THE MAX
;                  AND MIN CAMERA ANGLES AND USE THEM AS THE SERVO LIMITS IN THE AIRFRAME FILE.
;                  THE SERVO's NEUTRAL PULSE WIDTH IN THE AIRFRAME FILE SHOULD BE ALWAYS 1500us
;            <servo name="CAM_PAN"   no="6" min="970"   neutral="1500"   max="2075"/>
;                  <servo name="CAM_TILT"   no="7" min="1040"   neutral="1500"   max="1900"/>
;                  FINALLY TEST FOR CORRECT CAMERA ANGLES WITH THE "CAM_MODE_ANGLES" MODE
;                  YOU ARE DONE!!!
;
;*******************************************************************/
static void point_cam(float fObjectEast, float fObjectNorth, float fAltitude)
{
  static VECTOR svPlanePosition,
         svObjectPosition,
         svObjectPositionForPlane,
         svObjectPositionForPlane2;

  static MATRIX smRotation;

  float heading_radians = 0;
#if defined(CAM_PAN_COMMAND)
  int16_t pan_servo_pprz_val = 0;
  float  pan_srv_normalized = 0;
#endif

  static uint8_t cnt1 = 0;
  static float cam_phi = RadOfDeg(CAM_PAN_POSITION_FOR_FPV);
  static float cam_theta = RadOfDeg(CAM_TILT_POSITION_FOR_FPV);
  static float cam_pan_max = RadOfDeg(CAM_PAN_MAX);
  static float cam_pan_min = RadOfDeg(CAM_PAN_MIN);
  static float cam_tilt_max = RadOfDeg(CAM_TILT_MAX);
  static float cam_tilt_min = RadOfDeg(CAM_TILT_MIN);

  int16_t tilt_servo_pprz_val = 0;

  float   dist_x = 0;
  float   dist_y = 0;
  float   cam_point_x;
  float   cam_point_y;
  static float  memory_x, memory_y, memory_z = 0;

  float  tilt_srv_normalized = 0;

  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct EnuCoor_f *pos = stateGetPositionEnu_f();

  /*####################################################################################################*/
  /*########################### CAMERA MODULE DEPENDANT CONFIGURATION ##################################*/
  /*####################################################################################################*/
  /************************************************************************************************/
#if USE_JOYSTICK_FOR_CAMERA && !defined(RADIO_CONTROL_TYPE_DATALINK)
  /************************************************************************************************/

#if defined(CAM_ZOOM_COMMAND)
#pragma message "CAMERA ZOOM is controlled from JOYSTICK and a SERVO CHANNEL"
  ap_state->commands[COMMAND_CAM_ZOOM] = joystick_cam_zoom_val * (MAX_PPRZ / 128);
#else
#pragma message "NO ZOOM CONTROL FROM THE CAMERA MODULE"
#endif
#pragma message "CAMERA LOCK is controlled from JOYSTICK "

  /************************************************************************************************/
#else //#if defined(USE_JOYSTICK_FOR_CAMERA) && !defined(RADIO_CONTROL_TYPE_DATALINK)
  /************************************************************************************************/

#if defined(CAM_ZOOM_RADIO_CH) && defined(CAM_ZOOM_COMMAND)
#pragma message "Camera Zoom is controlled from a RC RADIO CHANNEL and a SERVO CHANNEL"
  ap_state->commands[CAM_ZOOM_COMMAND] = (*fbw_state).channels[CAM_ZOOM_RADIO_CH];
#elif defined(CAM_ZOOM_COMMAND)
#pragma message "Camera Zoom is controlled from a VARIABLE and a SERVO CHANNEL"
  ap_state->commands[CAM_ZOOM_COMMAND] = camera_zoom_pprz_val = 0;
#elif defined(USE_CHDK) && USE_CHDK == 1
#pragma message "Camera Zoom is controlled by the CHDK module"
#else
#pragma message "NO ZOOM CONTROL FROM THE CAMERA MODULE"
#endif // #if defined(CAM_ZOOM_RADIO_CH)

#if defined(CAM_LOCK_RADIO_CH)
#pragma message "CAMERA LOCK is controlled from a RC RADIO CHANNEL"

 if ((float)(*fbw_state).channels[CAM_LOCK_RADIO_CH] > MIN_PPRZ / 2) { cam_lock = true; }else{ cam_lock = false; }

#else
#pragma message "CAMERA LOCK is controlled from datalink "
#endif

  /************************************************************************************************/
#endif //#if defined(USE_JOYSTICK_FOR_CAMERA)
  /************************************************************************************************/


  /*####################################################################################################*/
  /*#################################  PAN AND TILT INPUT CONTROL  #####################################*/
  /*####################################################################################################*/

//Pan & Tilt input and camera target calculation only active in MANUAL & STABILIZED mode.
  if (cam_mode == CAM_MODE_TRACK || cam_mode == CAM_MODE_MANUAL) {

    /*----------------------------------------------------------------------------------------------------*/
// When the variable "cam_lock" is set then the last calculated position is used as the target waypoint.
#if USE_JOYSTICK_FOR_CAMERA && !defined(RADIO_CONTROL_TYPE_DATALINK)
#pragma message "PAN & TILT is controlled from JOYSTICK"
    pan_servo_pprz_val = joystick_cam_pan_val * (MAX_PPRZ / 128) * CAM_PAN_DIRECTION;
    tilt_servo_pprz_val = joystick_cam_tilt_val * (MAX_PPRZ / 128) * CAM_TILT_DIRECTION;
#elif !defined(CAM_PAN_COMMAND)
#pragma message "TILT ONLY AVAILABLE, controlled from an RC RADIO CHANNEL"
    tilt_servo_pprz_val = (*fbw_state).channels[CAM_TILT_RADIO_CH] * CAM_TILT_DIRECTION;
#else
#pragma message "PAN & TILT is controlled from RC RADIO CHANNELS"
    pan_servo_pprz_val = (*fbw_state).channels[CAM_PAN_RADIO_CH] * CAM_PAN_DIRECTION;
    tilt_servo_pprz_val = (*fbw_state).channels[CAM_TILT_RADIO_CH] * CAM_TILT_DIRECTION;
#endif // #if USE_JOYSTICK_FOR_CAMERA && !defined(RADIO_CONTROL_TYPE_DATALINK)

#if defined(CAM_PAN_COMMAND)
    if (cam_lock == false) {
      if (pan_servo_pprz_val > (MAX_PPRZ / 32) || pan_servo_pprz_val < (MIN_PPRZ / 32)) {
        pan_srv_normalized = (float)pan_servo_pprz_val / (float)(MAX_PPRZ);
        cam_phi += pan_srv_normalized * 0.02; // 3 degrees x 20 = 60/s max.
      }
    }
#endif

#if defined(CAM_TILT_COMMAND)
    if (cam_lock == false) {
      if (tilt_servo_pprz_val > (MAX_PPRZ / 32) || tilt_servo_pprz_val < (MIN_PPRZ / 32)) {
        tilt_srv_normalized = (float)tilt_servo_pprz_val / (float)MAX_PPRZ;
        cam_theta += tilt_srv_normalized * 0.02;
      }

    }
#endif

    // Limit pan and tilt angles to maximum allowed (defined in the airframe file)
    if (cam_phi >= cam_pan_max) { cam_phi = cam_pan_max; }
    if (cam_phi <= cam_pan_min) { cam_phi = cam_pan_min; }
    if (cam_theta >= cam_tilt_max) { cam_theta = cam_tilt_max; }
    if (cam_theta <= cam_tilt_min) { cam_theta = cam_tilt_min; }

    /*####################################################################################################*/
    /*################################  END OF PAN-TILT INPUT CONTROL  ###################################*/
    /*####################################################################################################*/

    /*####################################################################################################*/
    /*##############################  START OF CAMERA TARGET CALCULATION  ################################*/
    /*####################################################################################################*/
    // If the camera is locked use the buffer instead of the current coordinates and angles.
    // This is done in order to have trimming capability while the camera is still locked.
    // The below code is executed only when the camera is unlocked or trim is requested.

// FOR TESTING (FIXED ANGLES) UNCOMMENT THE TWO BELOW LINES
#if defined(SITL) 
    cam_phi = RadOfDeg(CAM_SIM_PAN_ANGLE); //  -X IS TO THE LEFT AND +X IS TO THE RIGHT
    cam_theta = RadOfDeg(CAM_SIM_TILT_ANGLE);  // 0 IS STRAIGHT DOWN 90 IS STRAIGHT IN FRONT -X is to the back.
#endif
#if !defined(CAM_PAN_COMMAND)
    cam_phi = 0;
#endif
    if (cam_lock == false) {
      svPlanePosition.fx = pos->x;
      svPlanePosition.fy = pos->y;
      svPlanePosition.fz = pos->z;
      if (cam_mode == CAM_MODE_MANUAL) {
        svObjectPositionForPlane2.fx = svPlanePosition.fx;
        dist_x = tanf(cam_theta + att->theta) * (svPlanePosition.fz - fAltitude);
        svObjectPositionForPlane2.fx += dist_x;
        dist_y = (tanf(att->phi) * (svPlanePosition.fz - fAltitude)) * cosf(cam_theta + att->theta);
        svObjectPositionForPlane2.fy = svPlanePosition.fy + dist_y;

      } else {
        svObjectPositionForPlane2.fx = svPlanePosition.fx;
        dist_x = tan(cam_theta) * (svPlanePosition.fz - fAltitude);
        svObjectPositionForPlane2.fx += dist_x;
        svObjectPositionForPlane2.fy = svPlanePosition.fy;
      }

      svObjectPositionForPlane2.fz = fAltitude;

      // distance between plane and camera projection when the camera is at 0 degrees azimuth and the plane goes to the East (0 degrees)
      vSubtractVectors(&svObjectPositionForPlane, svObjectPositionForPlane2, svPlanePosition);

#if defined(POINT_CAM_YAW_PITCH) || defined(POINT_CAM_ROLL)
      //"stateGetHorizontalSpeedDir_f()" ιs in radians 180 to -180. if you need 0 to 360 then use the "gps_course" variable.
      //The YAW angle comes from the GPS where 0 = North and the camera is already facing EAST at 90 degrees (to the right wing tip)
      //so when the aircraft flies at 0 degrees (to the NORTH), the camera looks to 0 degrees in Standard Mathematical notation
      // THUS THE CONVERSION IS ALREADY DONE.
      heading_radians = *stateGetHorizontalSpeedDir_f(); 
//    camera pan angle correction, using a counter clockwise rotation
      smRotation.fx1 = cos(cam_phi - heading_radians);
      smRotation.fx2 = -sin(cam_phi - heading_radians);
      smRotation.fx3 = 0.;
      smRotation.fy1 = -smRotation.fx2;
      smRotation.fy2 = smRotation.fx1;
      smRotation.fy3 = 0.;
      smRotation.fz1 = 0.;
      smRotation.fz2 = 0.;
      smRotation.fz3 = 1.;

#elif defined(POINT_CAM_YAW_PITCH_NOSE) || defined(POINT_CAM_PITCH_ROLL) || defined(POINT_CAM_PITCH)
      //"estimator_hspeed_dir" ιs in radians 180 to -180. if you need 0 to 360 then use the "gps_course" variable.
      //The YAW angle comes from the GPS
      //Convert the gps heading (radians) to standard mathematical notation.
      heading_radians = RadOfDeg(90) - stateGetHorizontalSpeedDir_f();
#if defined(POINT_CAM_PITCH_ROLL)
      cam_phi = 0;
#endif
      // camera pan angle correction, using a counter clockwise rotation
#if CAM_LEFT_IS_POSITIVE //FIXME do  need this?
      smRotation.fx1 = cos(heading_radians + cam_phi);
      smRotation.fx2 = -sin(heading_radians + cam_phi);
#else
      smRotation.fx1 = cos(heading_radians - cam_phi);
      smRotation.fx2 = -sin(heading_radians - cam_phi);
#endif
      smRotation.fx3 = 0.;
      smRotation.fy1 = -smRotation.fx2;
      smRotation.fy2 = smRotation.fx1;
      smRotation.fy3 = 0.;
      smRotation.fz1 = 0.;
      smRotation.fz2 = 0.;
      smRotation.fz3 = 1.;

#endif

      vMultiplyMatrixByVector(&svObjectPositionForPlane2, smRotation, svObjectPositionForPlane);

      fObjectEast = (pos->x + svObjectPositionForPlane2.fx) ;
      fObjectNorth = (pos->y + svObjectPositionForPlane2.fy) ;
      memory_x = fObjectEast;
      memory_y = fObjectNorth;
      memory_z = fAltitude;
      cam_point_x = fObjectEast;
      cam_point_y = fObjectNorth;

      // The below code is cpu intensive so i run it slower
      if (cnt1 >= (FIXED_WING_CAM_PERIODIC_FREQ / 5)) {
        struct UtmCoor_f utm;
        utm.east = nav_utm_east0 + fObjectEast;
        utm.north = nav_utm_north0 + fObjectNorth;
        utm.zone = gps.utm_pos.zone;
        struct LlaCoor_f lla;
        lla_of_utm_f(&lla, &utm);
        cam_point_lon = lla.lon * (180 / M_PI);
        cam_point_lat = lla.lat * (180 / M_PI);
        if (wgs84_dms == true) { wgs84_to_wgs84dms(); }
        cam_point_distance_from_home = (uint16_t)((sqrt((cam_point_x * cam_point_x) + (cam_point_y * cam_point_y))) / 10);
        cnt1 = 0;

      } else { cnt1++; }

    } else {
      fObjectEast = memory_x;
      fObjectNorth = memory_y;
      fAltitude = memory_z;
    }



    /*###################################################################################################*/
    /*############################  END OF CAMERA TARGET CALCULATION  ###################################*/
    /*###################################################################################################*/

  } else {
    //THE BELOW CODE IS ONLY EXECUTED IN CAM_MODE_WP_TARGET OR CAM_MODE_XY_TARGET
    // The below code calculates distance of target from home in meters.
    if (cnt1 >= (FIXED_WING_CAM_PERIODIC_FREQ / 5)) { // latlong_of_utm() is cpu intensive so i run it slower
      cam_point_distance_from_home = (uint16_t) fabs(((uint16_t)((sqrt((fObjectNorth * fObjectNorth) +
                                     (fObjectEast * fObjectEast))) / 10)) - ((uint16_t)((sqrt((pos->y * pos->y) + (pos->x * pos->x))) / 10)));
      struct UtmCoor_f utm;
      utm.east = nav_utm_east0 + fObjectEast;
      utm.north = nav_utm_north0 + fObjectNorth;
      utm.zone = gps.utm_pos.zone;
      struct LlaCoor_f lla;
      lla_of_utm_f(&lla, &utm);
      cam_point_lon = lla.lon * (180 / M_PI);
      cam_point_lat = lla.lat * (180 / M_PI);
      if (wgs84_dms == true) { wgs84_to_wgs84dms(); }
      cnt1 = 0;

    } else { cnt1++; }
  }
#if defined(WP_CAM_POINT)
  waypoints[WP_CAM_POINT].x = fObjectEast;
  waypoints[WP_CAM_POINT].y = fObjectNorth;
  //waypoints[WP_CAM_POINT].a = fAltitude;
#endif
  if (cam_mode == CAM_MODE_MANUAL && cam_lock == false) { cam_pan_c = cam_phi; cam_tilt_c = cam_theta; return; }

  /*####################################################################################################*/
  /*#######################  CAMERA PAN & TILT SERVO ANGLE CALCULATION  ################################*/
  /*####################################################################################################*/

  /*
  By swapping coordinates (fx=fPlaneNorth, fy=fPlaneEast) we make the the circle angle go from 0 (0 is to the top of the circle)
  to 360 degrees or from 0 radians to 2 PI radians in a clockwise rotation. This way the GPS reported angle can be directly
  applied to the rotation matrices (in radians).
  In standard mathematical notation 0 is to the right (East) of the circle, -90 is to the bottom, +-180 is to the left
  and +90 is to the top (counterclockwise rotation).
  When reading back the actual rotated coordinates sv_cam_projection.fx has the y coordinate and sv_cam_projection.fy has the x
  represented on a circle in standard mathematical notation.
  */
// Input airplane attitude and target, airplane coordinates. output camera pan and tilt angles in radians.
  svPlanePosition.fx = pos->y;
  svPlanePosition.fy = pos->x;
  svPlanePosition.fz = pos->z;

  svObjectPosition.fx = fObjectNorth;
  svObjectPosition.fy = fObjectEast;
  svObjectPosition.fz = fAltitude;

  // distance between plane and object
  vSubtractVectors(&svObjectPositionForPlane, svObjectPosition, svPlanePosition);

  // yaw, clockwise rotation
  //The YAW angle comes from the GPS.
#if USE_MAG_FOR_CAMERA && !defined(SITL)
  //The YAW angle comes from the magnetometer so it follows better the fuselage movement.
  smRotation.fx1 = cos(mag_heading);
  smRotation.fx2 = sin(mag_heading);
#else
  //The YAW angle comes from the GPS.
  smRotation.fx1 = cos(stateGetHorizontalSpeedDir_f());
  smRotation.fx2 = sin(stateGetHorizontalSpeedDir_f());
#endif
  smRotation.fx3 = 0.;
  smRotation.fy1 = -smRotation.fx2;
  smRotation.fy2 = smRotation.fx1;
  smRotation.fy3 = 0.;
  smRotation.fz1 = 0.;
  smRotation.fz2 = 0.;
  smRotation.fz3 = 1.;

  vMultiplyMatrixByVector(&svObjectPositionForPlane2, smRotation, svObjectPositionForPlane);

  // pitch
  smRotation.fx1 = cos(att->theta);
  smRotation.fx2 = 0.;
  smRotation.fx3 = sin(att->theta);
  smRotation.fy1 = 0.;
  smRotation.fy2 = 1.;
  smRotation.fy3 = 0.;
  smRotation.fz1 = -smRotation.fx3;
  smRotation.fz2 = 0.;
  smRotation.fz3 = smRotation.fx1;

  vMultiplyMatrixByVector(&svObjectPositionForPlane, smRotation, svObjectPositionForPlane2);

  // roll
  smRotation.fx1 = 1.;
  smRotation.fx2 = 0.;
  smRotation.fx3 = 0.;
  smRotation.fy1 = 0.;
  smRotation.fy2 = cos(att->phi);
  smRotation.fy3 = -sin(att->phi);
  smRotation.fz1 = 0.;
  smRotation.fz2 = -smRotation.fy3;
  smRotation.fz3 = smRotation.fy2;

  vMultiplyMatrixByVector(&svObjectPositionForPlane2, smRotation, svObjectPositionForPlane);



#if defined(POINT_CAM_PITCH)

  /*
   * This is for one axis pitch camera mechanisms. The pitch servo neutral
   * makes the camera look down, 90° is to the front and -90° is to the
   * back. The pitch value is given through the tilt parameter.
   * The camera picture is upright when looking in flight direction.
   *
   * tilt servo, looking from left:
   *
   *     plane front <-------------- plane back
   *                      / I \
   *                     /  I  \
   *                   45°  I  -45°
   *                        0°
   *
   * (should be hyperbolic, we use lines to make it better, the plane rolls
   *  away from the object while flying towards it!)
   *
   */

  //fTilt =   0 -> camera looks down,  90 -> camera looks forward, -90 -> camera looks backward

  //we roll away anyways
  //cam_tilt_c = (float)(atan2( svObjectPositionForPlane2.fx,
  //                            sqrt(svObjectPositionForPlane2.fy * svObjectPositionForPlane2.fy
  //                            + svObjectPositionForPlane2.fz * svObjectPositionForPlane2.fz )
  //                      ));
  cam_tilt_c = (float)(atan2(svObjectPositionForPlane2.fx, -svObjectPositionForPlane2.fz));

  /* fPan is deactivated
  */
  cam_pan_c = 0;

#elif defined(POINT_CAM_ROLL)

  /*
   * This is for single axis roll camera mechanisms. The tilt servo neutral
   * makes the camera look down, -90° is to the right and 90° is to the
   * left.
   * The camera picture is upright when looking to the right.
   *
   *
   * tilt servo, looking from behind:
   *
   *     plane left --------------- plane right
   *                     / I \
   *                    /  I  \
   *                  45°  I  -45°
   *                       0°
   *
   */
#if 1  // have to check if it helps
  cam_tilt_c = (float)(atan2(svObjectPositionForPlane2.fy,
                             sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                                  + svObjectPositionForPlane2.fz * svObjectPositionForPlane2.fz)
                            ));
#else
  cam_tilt_c = (float)(atan2(svObjectPositionForPlane2.fy, -svObjectPositionForPlane2.fz));
#endif

  /* fPan is deactivated
  */
  cam_pan_c = 0;

#elif defined(POINT_CAM_YAW_PITCH_NOSE)


  /*
                   -45   0   45
                      \  |  /
                       \ | /
                        \|/
                        ###
              TOP VIEW  ###
           _____________###_____________
          |                             |
  left tip|_____________________________|right wing tip
                        ###
                        ###
                        ###
                        ###
                   _____###____
                  |______|______|
                         |
  */

// Pan servo is fixed to the plane.
#if CAM_PAN_MODE == 360
#pragma message "PAN mode is 360 degrees"

  cam_pan_c = (float)(atan2(svObjectPositionForPlane2.fy, svObjectPositionForPlane2.fx));

  cam_tilt_c = (float)(atan2(sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                                  + svObjectPositionForPlane2.fy * svObjectPositionForPlane2.fy),
                             -svObjectPositionForPlane2.fz
                            ));


#else // #if CAM_PAN_MODE == 360
#pragma message "PAN mode is 180 degrees"

  cam_pan_c = (float)(atan2(svObjectPositionForPlane2.fy, svObjectPositionForPlane2.fx));

  cam_tilt_c = (float)(atan2(sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                                  + svObjectPositionForPlane2.fy * svObjectPositionForPlane2.fy),
                             -svObjectPositionForPlane2.fz
                            ));

  if (cam_pan_c > cam_pan_max) {
    cam_pan_c -= RadOfDeg(180); // 3.14 Radians = 180 degrees
    cam_tilt_c = -(cam_tilt_c);

  } else if (cam_pan_c < cam_pan_min) {
    cam_pan_c += RadOfDeg(180); // 3.14 Radians = 180 degrees
    cam_tilt_c = -(cam_tilt_c);
  }

#endif // #if CAM_PAN_MODE == 360 #else ....
  if (cam_lock == true) { cam_phi = cam_pan_c; cam_theta = cam_tilt_c;  }

#elif defined(POINT_CAM_YAW_PITCH)

  /*
   * This is for two axes pan/tilt camera mechanisms. The default is to
   * circle clockwise so view is right. The pan servo neutral makes
   * the camera look to the right with 0° given, -90° is to the back and
   * 90° is to the front. The tilt servo neutral makes the camera look
   * down with 0° given, 90° is to the right and -90° is to the left (all
   * values are used in radian in the software). If the camera looks to
   * the right side of the plane, the picture is upright. It is upside
   * down when looking to the left. That is corrected with the MPEG
   * decoding software on the laptop by mirroring. The pan servo is fixed
   * in the plane and the tilt servo is moved by the pan servo and moves
   * the camera.
   *
   *
   * pan servo, tilt set to 90°, looking from top:
   *
   *   plane front
   *
   *       ^
   *       I
   *       I  45°
   *       I /
   *       I/
   *       I------- 0°
   *       I\
   *       I \
   *       I  -45°
   *       I
   *
   *   plane back
   *
   *
   * tilt servo, pan set to 0°, looking from back:
   *
   *     plane left --------------- plane right
   *                     / I \
   *                    /  I  \
   *                 -45°  I   45°
   *                       0°
   *
   */

  /* fPan =   0  -> camera looks along the wing
             90  -> camera looks in flight direction
            -90  -> camera looks backwards
  */
  /* fixed to the plane*/
  cam_pan_c = (float)(atan2(svObjectPositionForPlane2.fx, fabs(svObjectPositionForPlane2.fy)));
  /* fTilt =   0  -> camera looks down
              90  -> camera looks into right hemisphere
             -90  -> camera looks into left hemispere
     actually the camera always looks more or less downwards, but never upwards
  */
  cam_tilt_c = (float)(atan2(sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                                  + svObjectPositionForPlane2.fy * svObjectPositionForPlane2.fy),
                             -svObjectPositionForPlane2.fz
                            ));
  if (svObjectPositionForPlane2.fy < 0) {
    cam_pan_c = -cam_pan_c;
    cam_tilt_c = -cam_tilt_c;
  }

  if (cam_lock == true) { cam_phi = cam_pan_c; cam_theta = cam_tilt_c;  }

#elif defined(POINT_CAM_PITCH_ROLL)

  /*
   * This is for another two axes camera mechanisms. The tilt servo is fixed to
   * the fuselage and moves the pan servo.
   *
   * tilt servo, looking from left:
   *
   *    plane front <--------------- plane back
   *                      / I \
   *                     /  I  \
   *                   45°  I  -45°
   *                        0°
   *
   *
   * pan servo, looking from back:
   *
   *     plane left --------------- plane right
   *                     / I \
   *                    /  I  \
   *                  45°  I  -45°
   *                       0°
   *
   */

  cam_tilt_c = (float)(atan2(svObjectPositionForPlane2.fx, -svObjectPositionForPlane2.fz));

  cam_pan_c  = (float)(atan2(-svObjectPositionForPlane2.fy,
                             sqrt(svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx
                                  + svObjectPositionForPlane2.fz * svObjectPositionForPlane2.fz)
                            ));

  if (cam_lock == true) { cam_phi = cam_pan_c; cam_theta = cam_tilt_c;  }

#else
#error at least one CAM_POINT_* camera mount has to be defined!
#endif

  return;
}



