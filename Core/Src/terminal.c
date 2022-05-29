/*
 * terminal.c
 *
 *  Created on: Feb 9, 2022
 *      Author: rangg
 */

#include "terminal.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include "stdarg.h"

extern int mode;
extern int time;
extern int flag;

void command_printf(const char *format, ...) {
	va_list arg;
	va_start(arg, format);
	int len;
	unsigned char print_buffer[255];
	//char *buf;
	len = vsnprintf(print_buffer, 254, format, arg);
	va_end(arg);
	if (len > 0) {
		CDC_Transmit_FS((unsigned char*) print_buffer,
				(len < 254) ? len + 1 : 255);
	}
	//memset(print_buffer, 0, len);
}

void terminal_proses(uint8_t *str) {
	enum {
		kMaxArgs = 64
	};
	int argc = 0;
	char *argv[kMaxArgs];

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}
	if (argc == 0) {

		command_printf("tidak ada perintah\n");
	}

	else if (strcmp(argv[0], "param") == 0) {
		command_printf("Mode %d\n", mode);
		command_printf("Waktu %d\n\n", time);
	}

	else if (strcmp(argv[0], "off") == 0) {
		flag = 0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		command_printf("OFF\n");
	}

	else if (strcmp(argv[0], "on") == 0) {
		flag = 1;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		command_printf("ON\n");
	}

	else if (strcmp(argv[0], "setMode") == 0) {
		if (argc == 2) {
			sscanf(argv[1], "%d", &mode);
		}
		if (mode == 1) {
			command_printf("Mode DOL\n");
		} else if (mode == 2) {
			command_printf("Mode Soft Starting\n");
		}

	}

	else if (strcmp(argv[0], "setWaktu") == 0) {
		if (argc == 2) {
			sscanf(argv[1], "%d", &time);
			command_printf("Waktu Softstart = %d detik\n", time);
		}
	}
}

