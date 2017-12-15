/*
 * Copyright (C) D.C. van Wijngaarden
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
 * @file "modules/alpha_esc/alpha_esc.h"
 * @author D.C. van Wijngaarden
 * Converts telemtry data from t motor alpha esc to the autopilot (and groundstation)
 */

#ifndef ALPHA_ESC_H
#define ALPHA_ESC_H

#include "std.h"
#include "generated/airframe.h"
#include "pprzlink/pprz_transport.h"
// extern void alpha_esc_init();
// extern void alpha_esc_event();

/* Main alpha esc strcuture */
struct alpha_esc_t {
  struct link_device *device;           ///< The device which is uses for communication
  struct pprz_transport transport;      ///< The transport layer (PPRZ)
  bool msg_available;                 ///< If we received a message
};

#endif

