/*
 * Copyright (C) Dennis van Wijngaarden
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
 * @file "modules/jevois/jevois_comm.h"
 * @author Dennis van Wijngaarden
 * Standard communication protocol for jevois camera
 */

#ifndef JEVOIS_COMM_H
#define JEVOIS_COMM_H

extern void jevois_comm_init();
extern void jevois_comm_event();

/* Main jevois_comm structure */
struct jevois_comm_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  bool msg_available;                 ///< If we received a message
};

// Data structure
static struct jevois_comm_data {
	int16_t x;
	int16_t y;
};

// Data Command structure
static struct jevois_comm_command {
	int16_t alt;
};

#endif

