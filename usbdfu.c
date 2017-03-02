/*
 * This file is part of the IRMP_STM32 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2017 Joerg Riechardt
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>

#define APP_ADDRESS	0x08002000
#define MAX_ADDRESS	0x08040000

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21
#define CMD_ERASE	0x41

/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[1024];

static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

static struct {
	uint8_t buf[sizeof(usbd_control_buffer)];
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog;

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0xDF11,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE, /* Device Firmware Upgrade */
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,

	/* The ST Microelectronics DfuSe application needs this string.
	 * The format isn't documented... */
	.iInterface = 4,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"DFU Demo",
	"DEMO",
	/* This string is used by ST Microelectronics' DfuSe utility. */
	"@Internal Flash   /0x08000000/8*001Ka,56*001Kg",
};

static uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout)
{
	switch (usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		usbdfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = 100;
		return DFU_STATUS_OK;
	case STATE_DFU_MANIFEST_SYNC:
		/* Device will reset when read is complete */
		usbdfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;
	default:
		return DFU_STATUS_OK;
	}
}

static void strobePin(uint32_t bank, uint16_t pin, uint8_t count, uint32_t rate) {
    gpio_clear(bank, pin);

    uint32_t c;
    while (count-- > 0) {
        for (c = rate; c > 0; c--) {
            asm volatile("nop");
        }
        gpio_set(bank, pin);
        for (c = rate; c > 0; c--) {
            asm volatile("nop");
        }
        gpio_clear(bank, pin);
    }
}

static void usbdfu_getstatus_complete(usbd_device *device,
					  struct usb_setup_data *req)
{
	int i;
	(void)req;

	switch (usbdfu_state) {
	case STATE_DFU_DNBUSY:
		flash_unlock();
		if (prog.blocknum == 0) {
			if ((*(uint32_t*)(prog.buf+1) < APP_ADDRESS) ||
				(*(uint32_t*)(prog.buf+1) >= MAX_ADDRESS)) {
				flash_lock();
				usbd_ep_stall_set(device, 0, 1);
				return;
			}
			switch (prog.buf[0]) {
			case CMD_ERASE:
				flash_erase_page(*(uint32_t*)(prog.buf+1));
			case CMD_SETADDR:
				prog.addr = *(uint32_t*)(prog.buf+1);
			}
		} else {
			uint32_t baseaddr = prog.addr +
				((prog.blocknum - 2) *
					dfu_function.wTransferSize);
			for (i = 0; i < prog.len; i += 2)
				flash_program_half_word(baseaddr + i,
						*(uint16_t*)(prog.buf+i));
			strobePin(GPIOB, GPIO12, 2, 0x10000);
		}
		flash_lock();

		/* We jump straight to dfuDNLOAD-IDLE,
		 * skipping dfuDNLOAD-SYNC
		 */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;
	case STATE_DFU_MANIFEST:
		/* USB device must detach, we just reset... */
		scb_reset_system();
		return; /* Will never return. */
	default:
		return;
	}
}

static int usbdfu_control_request(usbd_device *device,
				  struct usb_setup_data *req, uint8_t **buf,
				  uint16_t *len,
				  void (**complete)(usbd_device *device,
						struct usb_setup_data *req))
{
	(void)device;

	if ((req->bmRequestType & 0x7F) != 0x21)
		return 0; /* Only accept class request */

	switch (req->bRequest) {
	case DFU_DNLOAD:
		if ((len == NULL) || (*len == 0)) {
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
			return 1;
		} else {
			/* Copy download data for use on GET_STATUS */
			prog.blocknum = req->wValue;
			prog.len = *len;
			memcpy(prog.buf, *buf, *len);
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
			return 1;
		}
	case DFU_CLRSTATUS:
		/* Clear error and return to dfuIDLE */
		if (usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_ABORT:
		/* Abort returns to dfuIDLE state */
		usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_UPLOAD:
		/* Upload not supported for now */
		return 0;
	case DFU_GETSTATUS: {
		uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */

		(*buf)[0] = usbdfu_getstatus(&bwPollTimeout);
		(*buf)[1] = bwPollTimeout & 0xFF;
		(*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
		(*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
		(*buf)[4] = usbdfu_state;
		(*buf)[5] = 0; /* iString not used here */
		*len = 6;

		*complete = usbdfu_getstatus_complete;

		return 1;
		}
	case DFU_GETSTATE:
		/* Return state with no state transision */
		*buf[0] = usbdfu_state;
		*len = 1;
		return 1;
	}

	return 0;
}

static bool dfuUploadStarted(void) {
    return (usbdfu_state == STATE_DFU_DNBUSY) ? 1 : 0;
}

static bool dfuUploadDone(void)
{
    return (usbdfu_state == STATE_DFU_MANIFEST) ? 1 : 0;
}

static bool checkUserCode(uint32_t usrAddr) {
    uint32_t sp = *(volatile uint32_t *) usrAddr;

    if ((sp & 0x2FFE0000) == 0x20000000) {
        return (1);
    } else {
        return (0);
    }
}

static void jump_to_app_if_valid(void)
{
    /* Boot the application if it's valid */
    if(checkUserCode(APP_ADDRESS)) {
	/* Set vector table base address */
	SCB_VTOR = APP_ADDRESS & 0x3FFFF;
	/* Initialise master stack pointer */
	asm volatile ("msr msp, %0"::"g"
	    (*(volatile uint32_t*)APP_ADDRESS));
	    /* Jump to application */
	    (*(void(**)())(APP_ADDRESS + 4))();
    }
}

int main(void)
{
	/* If we did just download new code, reset the register and jump */
	if (RCC_CSR & RCC_CSR_SFTRSTF) { /* software reset */
		RCC_CSR = (RCC_CSR & RCC_CSR_RMVF); /* Clear the reset flags */
		jump_to_app_if_valid();
	}

	rcc_clock_setup_in_hse_25mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_OTGFS);

	gpio_set(GPIOB, GPIO12);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

	usbd_device *usbd_dev = usbd_init(&stm32f107_usb_driver, &dev, &config, usb_strings, 4, usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_control_callback(usbd_dev,
		      USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
		      USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		      usbdfu_control_request);

	bool no_user_jump = !checkUserCode(APP_ADDRESS);
  
	int delay_count = 0;
	
	while ((delay_count++ < 1) || no_user_jump) {
	    strobePin(GPIOB, GPIO12, 1, 0x100000);
	    for (int i=0; i<150000; i++) {
		usbd_poll(usbd_dev);
		if(dfuUploadStarted()) {
		    while (!dfuUploadDone()) {
			usbd_poll(usbd_dev);
		    }
		}
	    }
	}
	scb_reset_system();
}
