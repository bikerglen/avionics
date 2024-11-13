/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"
#include "pico/unique_id.h"

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x4247, // vendor id - my poached vendor id
    .idProduct          = 0x0023, // product id - USB Gear and Flaps Indicators
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// HID Report Descriptor
// a single OUT report with report ID of 1
// first byte is accept mask:
//   0x80 to accept channel 0
//   0x40 to accept channel 1
//   0x20 to accept channel 2
//   0x10 to accept channel 3
// bytes 2 to 5 are data for channels 0 to 3
//   0 turns indicator power off
//   valid range from 32 to 224
//   values below 32 are set 32
//   values above 224 are set to 224
//--------------------------------------------------------------------+

uint8_t const desc_hid_report[] =
{
	0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
	0x09, 0x01,        // Usage (0x01)
	0xA1, 0x01,        // Collection (Application)

	0x85, 0x01,        //   Report ID (1)
	0x95, 0x05,        //   Report Count (5)
	0x75, 0x08,        //   Report Size (8)
	0x26, 0xFF, 0x00,  //   Logical Maximum (255)
	0x15, 0x00,        //   Logical Minimum (0)
	0x09, 0x01,        //   Usage (0x01)
	0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

	0xC0              // End Collection
};

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf)
{
  (void) itf;
  return desc_hid_report;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
  ITF_NUM_HID,
  ITF_NUM_TOTAL
};

#define  CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

#define EPNUM_HID   0x01

uint8_t const desc_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // Interface number, string index, protocol, report descriptor len, EP In & Out address, size & polling interval
  TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, 0x80 | EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, 10)
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr [] =
{
  (const char[]) { 0x09, 0x04 },    // 0: is supported language is English (0x0409)
  "bikerglen.com",                  // 1: Manufacturer
  "USB Gear and Flaps Indicator",   // 2: Product
  "123456",                         // 3: Serial, not used, returns pico unique board id string
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	(void) langid;

	uint8_t chr_count;

	if (index == 0) {
		memcpy (&_desc_str[1], string_desc_arr[0], 2);
		chr_count = 1;
	} else {

		const char *str;
		char tmp[32];

		if (index == 3) {
			pico_get_unique_board_id_string (tmp, 32);
			str = tmp;
		} else {
			if (!(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) {
				return NULL;
			}
			str = string_desc_arr[index];
		}

		// Cap at max char
		chr_count = strlen (str);
		if (chr_count > 31) chr_count = 31;

		// Convert ASCII string into UTF-16
		for (uint8_t i = 0; i < chr_count; i++) {
			_desc_str[1+i] = str[i];
		}
	}

	// first byte is length (including header), second byte is string type
	_desc_str[0] =(TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

	return _desc_str;
}
