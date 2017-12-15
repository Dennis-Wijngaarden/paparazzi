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
 * @file "modules/alpha_esc/alpha_esc.c"
 * @author D.C. van Wijngaarden
 * Converts telemtry data from t motor alpha esc to the autopilot (and groundstation)
 */

#include "modules/alpha_esc/alpha_esc.h"

#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"

// void alpha_esc_init() {}
// void alpha_esc_event() {}


