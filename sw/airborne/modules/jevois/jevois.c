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

#include "math/pprz_trig_int.h"
#include "math/pprz_algebra.h"
#include "math/pprz_algebra_int.h"

#include "state.h"
#include "generated/flight_plan.h"

#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"

// Main jevois structure
static struct jevois_t jevois = {
		.device = (&((JEVOIS_PORT).device)),
		.msg_available = false
};

// Georeference structure
struct georeference_jevois_t geo;

// Camera structure
struct camera_frame_jevois_t tar = {
		.w = 320,
		.h = 240,
		.f = 20//TODO
};

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

void jevois_init()
{
	pprz_transport_init(&jevois.transport);
	//uart_periph_set_baudrate(&uart2, B115200);
#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_JEVOIS_DATA, send_jevois_data);
#endif
}

void jevois_parse_msg()
{
#if PERIODIC_TELEMETRY
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_JEVOIS_DATA, send_jevois_data);
#endif
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
