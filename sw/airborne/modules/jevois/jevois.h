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
 * @file "modules/jevois/jevois.h"
 * @author Dennis van Wijngaarden and Arne Imbrechts
 * Bebop jeVois test code
 */

#ifndef JEVOIS_H
#define JEVOIS_H
#define DELIMITTER 0x20
#define BUFFER_LENGTH

#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"

/* georeference module */
extern void georeference_init(void);
extern void georeference_run(void);

/* Camera frame structure */
struct camera_frame_jevois_t {
	int32_t w;     ///< Frame width [px]
	int32_t h;     ///< Frame height [px]
	int32_t f;     ///< Camera Focal length in [px]
	int32_t px;    ///< Target pixel coordinate (left = 0)
	int32_t py;    ///< Target pixel coordinate (top = 0)
};

/* Georeference structure */
struct georeference_jevois_t {
  struct Int32Vect3 target_i;   ///< Target in pixels, with z being the focal length in pixels, x=up,y=right,out
  struct Int32Vect3 target_l;   ///< Target in meters, relative to the drone in LTP frame

  struct Int32Vect3 x_t;        ///< Target coordinates NED
};

void georeference_project(struct camera_frame_jevois_t *tar, int wp);

/* Main jevois structure */
struct jevois_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  bool msg_available;                 ///< If we received a message
};

/* Data structure */
static struct jevois_raw_data {
	int16_t x;
	int16_t y;
};

extern void jevois_init(void);
extern void jevois_event(void);

#endif
