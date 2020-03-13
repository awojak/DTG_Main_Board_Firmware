/*
 * usbd_cdc_vcp.h
 *
 *  Created on: 13.03.2020
 *      Author: Adrian
 */

#ifndef DRIVERS_USBD_CDC_VCP_H_
#define DRIVERS_USBD_CDC_VCP_H_

#include "main.h"
#include "stdbool.h"
#include "serial.h"

typedef struct {
    serialPort_t port;

    // Buffer used during bulk writes.
    uint8_t txBuf[20];
    uint8_t txAt;
    // Set if the port is in bulk write mode and can buffer.
    bool buffering;
} vcpPort_t;

void usbVcpInitHardware(void);
serialPort_t *usbVcpOpen(void);
//struct serialPort_s;
//uint32_t usbVcpGetBaudRate(struct serialPort_s *instance);

#endif /* DRIVERS_USBD_CDC_VCP_H_ */
