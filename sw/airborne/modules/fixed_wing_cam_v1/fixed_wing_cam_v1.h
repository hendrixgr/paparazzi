/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 * Modified and expanded to show the coordinates of where the camera is looking at, locking function
 * and other useful functions by Chris Efstathiou 15-November-2012 AD ~4 billion years after creation.
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
 *   file my_cam_v3_3.h
 *   Pan/Tilt camera library
 *
 */

#ifndef FIXED_WING_CAM_V1_H
#define FIXED_WING_CAM_V1_H

#include <inttypes.h>
#include "inter_mcu.h"

#ifndef CAM
#define CAM
#endif
#ifndef MOBILE_CAM
#define MOBILE_CAM
#endif
#ifndef POINT_CAM
#define POINT_CAM
#endif
#ifndef USE_JOYSTICK_FOR_CAMERA
#define USE_JOYSTICK_FOR_CAMERA 0
#endif
#if !defined(POINT_CAM_YAW_PITCH_NOSE) && !defined(POINT_CAM_YAW_PITCH) && !defined(POINT_CAM_PITCH_ROLL) && !defined(POINT_CAM_PITCH) && !defined(POINT_CAM_ROLL)
#define POINT_CAM_YAW_PITCH_NOSE
#endif
#if !defined(SHOW_CAM_COORDINATES)
#define SHOW_CAM_COORDINATES
#endif
// -X IS TO THE LEFT AND +X IS TO THE RIGHT, 0 IS STRAIGHT DOWN 90, IS STRAIGHT IN FRONT and -X is to the back.
#if !defined(CAM_SIM_PAN_ANGLE)
#define CAM_SIM_PAN_ANGLE 45
#endif
#if !defined(CAM_SIM_TILT_ANGLE)
#define CAM_SIM_TILT_ANGLE 45
#endif

#if defined(CAM_PAN_MODE) && CAM_PAN_MODE == 360 
#define CAM_PAN_POSITION_FOR_FPV  0
#define CAM_PAN_DIRECTION         1
#if !defined(CAM_PAN_MAX) && !defined(CAM_PAN_NEUTRAL)  && !defined(CAM_PAN_MIN) 
#define CAM_PAN_MAX               180 
#define CAM_PAN_NEUTRAL           0
#define CAM_PAN_MIN               -180
#endif
#define CAM_PAN0                  180  // When camera is retracted in the fuselage 

#define CAM_TILT_POSITION_FOR_FPV 90
#define CAM_TILT_DIRECTION        1
#if !defined(CAM_TILT_MAX) && !defined(CAM_TILT_NEUTRAL)  && !defined(CAM_TILT_MIN) 
#define CAM_TILT_MAX              90 
#define CAM_TILT_NEUTRAL          45
#define CAM_TILT_MIN              0
#endif
#define CAM_TILT0                 0 // When camera is retracted in the fuselage 

#else

#ifndef CAM_PAN_MODE
#define CAM_PAN_MODE              180
#endif
#define CAM_PAN_POSITION_FOR_FPV  0
#define CAM_PAN_DIRECTION         1
#if !defined(CAM_PAN_MAX) && !defined(CAM_PAN_NEUTRAL)  && !defined(CAM_PAN_MIN) 
#define CAM_PAN_MAX               90 
#define CAM_PAN_NEUTRAL           0
#define CAM_PAN_MIN               -90
#endif
#define CAM_PAN0                  0  // When camera is retracted in the fuselage 

#define CAM_TILT_POSITION_FOR_FPV 90
#define CAM_TILT_DIRECTION        1
#if !defined(CAM_TILT_MAX) && !defined(CAM_TILT_NEUTRAL)  && !defined(CAM_TILT_MIN)
#define CAM_TILT_MAX              90 
#define CAM_TILT_NEUTRAL          0
#define CAM_TILT_MIN              -90
#endif
#define CAM_TILT0                 -90 // When camera is retracted in the fuselage 

#endif

#ifndef CAM_USE_ANALOG_MOVEMENT
#define CAM_USE_ANALOG_MOVEMENT    0
#endif

/*
#ifndef CAM_PAN_MODE
#define CAM_PAN_MODE               180
#endif
#ifndef CAM_PAN_MAX
#define CAM_PAN_MAX                90
#endif
#ifndef CAM_PAN_NEUTRAL
#define CAM_PAN_NEUTRAL            0
#endif
#ifndef CAM_PAN_MIN
#define CAM_PAN_MIN                -90
#ifndef CAM_PAN0
#define CAM_PAN0                   0
#endif
#endif
#ifndef CAM_TILT_MAX
#define CAM_TILT_MAX               90
#endif
#ifndef CAM_TILT_NEUTRAL
#define CAM_TILT_NEUTRAL           0
#endif
#ifndef CAM_TILT_MIN
#define CAM_TILT_MIN               -90
#endif
#ifndef CAM_TILT0
#define CAM_TILT0                  90
#endif

#ifndef CAM_TILT_DIRECTION
#define CAM_TILT_DIRECTION         1
#endif
#ifndef CAM_PAN_DIRECTION
#define CAM_PAN_DIRECTION          1
#endif
#ifndef CAM_TILT_POSITION_FOR_FPV
#define CAM_TILT_POSITION_FOR_FPV  90
#endif
#ifndef CAM_PAN_POSITION_FOR_FPV
#define CAM_PAN_POSITION_FOR_FPV   0
#endif
#ifndef CAM_PAN_TILT_SAMPLES
#define CAM_PAN_TILT_SAMPLES       8
#endif
*/

#define CAM_MODE_OFF    0   /* Park the camera*/
#define CAM_MODE_NADIR    1   /* Look straight down */
#define CAM_MODE_ANGLES   2   /* Input: servo angles */
#define CAM_MODE_WP_HOME  3   /* Point camera to HOME */
#define CAM_MODE_WP_TARGET  4   /* Input: waypoint no */
#define CAM_MODE_MANUAL   5   // Move the camera from the stick, no calculations.
#define CAM_MODE_TRACK 6   // Camera is tracking coordinates calculated from the stick.

//extern uint8_t pprz_mode_joystick;

extern uint8_t cam_mode;
//extern uint8_t cam_lock;

extern float cam_phi_c, cam_theta_c;

extern float cam_pan_c, cam_tilt_c;
/* pan (move left and right), tilt (move up and down) */
/** Radians, for CAM_MODE_ANGLES mode */

extern float cam_target_x, cam_target_y;
/** For CAM_MODE_XY_TARGET mode */

extern uint8_t cam_target_wp;
/** For CAM_MODE_WP_TARGET mode */

#if defined(SHOW_CAM_COORDINATES)
extern uint16_t cam_point_distance_from_home;
extern float cam_point_lon,  cam_point_lat;
extern bool wgs84_dms;
extern bool cam_lock; // Locks to the coordinates where the cam was looking when the variable was set.
#endif
#if defined(USE_JOYSTICK_FOR_CAMERA) && !defined(RADIO_CONTROL_TYPE_DATALINK)
extern int8_t joystick_cam_pan_val;
extern int8_t joystick_cam_tilt_val;
extern int8_t joystick_cam_zoom_val;
#endif

extern int16_t camera_zoom_pprz_val;

extern float magnetic_compass_heading;

void fixed_wing_cam_periodic(void);
void fixed_wing_cam_init(void);

extern int8_t pan_trim;
extern int8_t tilt_trim;

//#if !defined(USE_CHDK)
//extern uint8_t chdk_command;
//#endif


#endif // CAM_H
