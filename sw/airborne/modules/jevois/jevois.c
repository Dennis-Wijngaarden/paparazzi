/*
 * Copyright (C) Dennis van Wijngaarden and Arne Imbrechts
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/jevois/jevois.c"
 * @author Dennis van Wijngaarden and Arne Imbrechts
 * Bebop jeVois test code
 */

#include "modules/jevois/jevois.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "math/pprz_trig_int.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"

#include "state.h"
#include "generated/flight_plan.h"

#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"

#include "subsystems/datalink/downlink.h"

// Main jevois structure
static struct jevois_t jevois = {
		.device = (&((JEVOIS_PORT).device)),
		.msg_available = false
};

// Georeference structure
struct georeference_jevois_t geo;

/* Focal length and field of view of the camera sensor */
int32_t focal_length;
int32_t FOV = 65; //field of view [degrees] specific jevois parameter


// Set message buffer
static char jevois_msg_buf[128] __attribute__ ((aligned)); // Create a 128 bytes buffer
static struct jevois_raw_data jevois_data;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
static void send_jevois_data(struct transport_tx *trans, struct link_device *dev)
{
	pprz_msg_send_JEVOIS_DATA(trans, dev, AC_ID, &jevois_data.x, &jevois_data.y);
}
#endif

/*Initialisation function for georeference */
void georeference_init(void)
{
  INT32_VECT3_ZERO(geo.target_i);
  INT32_VECT3_ZERO(geo.target_l);

  VECT3_ASSIGN(geo.x_t, 0, 0, 0);

  focal_length = cam.w/(2*tan(FOV*(M_PI/(2*180)))); //set to constant value for demoaruco module of jevois in pixels
  printf("%d", focal_length); //effe om te checken of de focal length correct is
}

void jevois_init()
{
	pprz_transport_init(&jevois.transport);
	//uart_periph_set_baudrate(&uart2, B115200);
#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_JEVOIS_DATA, send_jevois_data);
#endif

	/* georeference_init */
	georeference_init();
}

void jevois_parse_msg()
{
#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_JEVOIS_DATA, send_jevois_data);
#endif
}

/* Georeference_project will set a waypoint on a detected target in a camera frame by also taking into account the rotation matrix between camera and body frame */
void georeference_project(struct camera_frame_jevois_t *tar, int wp)
{
  // Target direction in camera frame: Zero is looking down in body frames
  // Pixel with x (width) value 0 projects to the left (body-Y-axis)
  // and y = 0 (height) to the top (body-X-axis)
  VECT3_ASSIGN(geo.target_i,
               ((tar->h / 2) - tar->py),
               (tar->px - (tar->w / 2)),
               (tar->f)
              );
  INT32_VECT3_LSHIFT(geo.target_i, geo.target_i, 4)

  // Camera <-> Body
  // Looking down in body frame
  // Bebop has 180deg z rotation but jevois has not so no minus before first two 1's, this rotation can be eliminated if it works as cam and body frame are equal in case of jevois
  struct Int32RMat body_to_cam_rmat;
  INT32_MAT33_ZERO(body_to_cam_rmat);
  MAT33_ELMT(body_to_cam_rmat, 0, 0) = 1 << INT32_TRIG_FRAC;
  MAT33_ELMT(body_to_cam_rmat, 1, 1) = 1 << INT32_TRIG_FRAC;
  MAT33_ELMT(body_to_cam_rmat, 2, 2) = 1 << INT32_TRIG_FRAC;

  struct Int32Vect3 target_b;
  int32_rmat_transp_vmult(&target_b, &body_to_cam_rmat, &geo.target_i);

  // Body <-> LTP
  struct Int32RMat *ltp_to_body_rmat = stateGetNedToBodyRMat_i();
  int32_rmat_transp_vmult(&geo.target_l, ltp_to_body_rmat, &target_b);

  // target_l is now a scale-less [pix<<POS_FRAC] vector in LTP from the drone to the target
  // Divide by z-component to normalize the projection vector
  int32_t zi = geo.target_l.z;
  if (zi <= 0)
  {
    // Pointing up or horizontal -> no ground projection
    return;
  }

  // Multiply with height above ground
  struct NedCoor_i *pos = stateGetPositionNed_i();
  int32_t zb = pos->z;
  geo.target_l.x *= zb;
  geo.target_l.y *= zb;

  // Divide by z-component
  geo.target_l.x /= zi;
  geo.target_l.y /= zi;
  geo.target_l.z = zb;

  // NED
  geo.x_t.x = pos->x - geo.target_l.x;
  geo.x_t.y = pos->y - geo.target_l.y;
  geo.x_t.z = 0;

  // ENU
  if (wp > 0) {
    waypoint_set_xy_i(wp, geo.x_t.y, geo.x_t.x);
    waypoint_set_alt_i(wp, geo.x_t.z);

    int32_t h = -geo.x_t.z;
    uint8_t wp_id = wp;
    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(geo.x_t.y),
                                   &(geo.x_t.x), &(h));

  }
}

void georeference_run(void)
{
	// Camera structure
	struct camera_frame_jevois_t cam = {
			.w = 320,
			.h = 240,
			.f = focal_length,
			//TODO put px and py in here as they are updated
	};

	georeference_project(&cam, WP_tar);
}

void jevois_event()
{

	static uint8_t i = 0;
	static uint8_t j = 0;

	while (uart_char_available(&uart2)) {
		uint8_t ch;
		ch = uart_getch(&uart2);
		//printf("%d\n",ch);
		if (ch != 10) {
			jevois_msg_buf[i] = ch;
			i++;
		}
		else {
			char *tok = strtok(jevois_msg_buf, " \r\n");
			//printf("%s\n",jevois_msg_buf);
			while (tok) {
				if (j == 2) {
					jevois_data.x = atoi(tok);
					//printf("%s\n",tok);
				}
				if (j == 3) {
					jevois_data.y = atoi(tok);
				}
				tok = strtok(0, " \r\n");
				j++;
			}
			jevois_parse_msg();
			i = 0;
			j = 0;
		}
	}
}
