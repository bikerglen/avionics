//---------------------------------------------------------------------------------------------
// notes
// 
// report id (0x01)
// select mask:
//    0x80: nose gear
//    0x40: right wing gear
//    0x20: flaps
//    0x10: left wing gear
// four bytes of data from 32 to 224 for a level or 0 to power down channel:
//    'd32 gear up to 'd224 gear down
//    'd32 flaps up to 'd224 flaps down
//


//---------------------------------------------------------------------------------------------
// includes
//

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "tusb.h"


//---------------------------------------------------------------------------------------------
// defines
//

// usb states
enum {
	USB_SUSPENDED = 0,		// 1 blink
	USB_NOT_MOUNTED = 1,    // 2 blinks
	USB_CONFIGURED = 2      // 3 blinks
};

// cli buffer length
#define CMD_MAXLEN 72


//---------------------------------------------------------------------------------------------
// typedefs
//


//---------------------------------------------------------------------------------------------
// prototypes
//

void InitGpioToOff (uint pin);

bool repeating_timer_callback (struct repeating_timer *t);

void InitCommand (void);
void GetCommand (void);

void dacWrite2 (uint8_t select, uint8_t a, uint8_t b);
void DacWriteLevels (uint8_t select_mask, uint8_t level);


//---------------------------------------------------------------------------------------------
// globals
//

const uint LED_PIN = 25;

const uint SPI0_SCK_PIN  = 2;
const uint SPI0_MOSI_PIN = 3;
const uint SPI0_MISO_PIN = 4;
const uint SPI0_CS0n_PIN = 5;   // nose gear
const uint SPI0_CS1n_PIN = 6;   // right gear

const uint SPI1_SCK_PIN  = 14;
const uint SPI1_MOSI_PIN = 15;
const uint SPI1_MISO_PIN = 12;
const uint SPI1_CS0n_PIN = 13;  // flaps
const uint SPI1_CS1n_PIN = 11;  // left gear

volatile bool flag100 = false;

uint8_t usbState = USB_NOT_MOUNTED;

static char cmd_buffer[CMD_MAXLEN];
static uint8_t cmd_length = 0;
static uint8_t cmd_state = 0;


//---------------------------------------------------------------------------------------------
// main
//

int main ()
{
	// local system variables
    struct repeating_timer timer100Hz;
	uint16_t ledTimer = 0;

	// initialize stdio
    stdio_uart_init_full (uart0, 115200, 0, 1);
	
	// initialize led to off
	InitGpioToOff (LED_PIN);

	// initialize TinyUSB
	tusb_init ();

    // initialize spi 0
    gpio_init    (SPI0_CS0n_PIN);
    gpio_set_dir (SPI0_CS0n_PIN, GPIO_OUT);
    gpio_put     (SPI0_CS0n_PIN, 1);
    gpio_init    (SPI0_CS1n_PIN);
    gpio_set_dir (SPI0_CS1n_PIN, GPIO_OUT);
    gpio_put     (SPI0_CS1n_PIN, 1);
    spi_init (spi0, 100000);
    spi_set_format (spi0, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function (SPI0_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI0_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI0_MOSI_PIN, GPIO_FUNC_SPI);

    // initialize spi 1
    gpio_init    (SPI1_CS0n_PIN);
    gpio_set_dir (SPI1_CS0n_PIN, GPIO_OUT);
    gpio_put     (SPI1_CS0n_PIN, 1);
    gpio_init    (SPI1_CS1n_PIN);
    gpio_set_dir (SPI1_CS1n_PIN, GPIO_OUT);
    gpio_put     (SPI1_CS1n_PIN, 1);
    spi_init (spi1, 100000);
    spi_set_format (spi1, 8, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function (SPI1_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI1_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI1_MOSI_PIN, GPIO_FUNC_SPI);

    // slow down IOs
    gpio_set_slew_rate (SPI0_SCK_PIN,  GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate (SPI0_MOSI_PIN, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate (SPI0_CS0n_PIN, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate (SPI0_CS1n_PIN, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength (SPI0_SCK_PIN,  GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength (SPI0_MOSI_PIN, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength (SPI0_CS0n_PIN, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength (SPI0_CS1n_PIN, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate (SPI1_SCK_PIN,  GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate (SPI1_MOSI_PIN, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate (SPI1_CS0n_PIN, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate (SPI1_CS1n_PIN, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength (SPI1_SCK_PIN,  GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength (SPI1_MOSI_PIN, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength (SPI1_CS0n_PIN, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength (SPI1_CS1n_PIN, GPIO_DRIVE_STRENGTH_2MA);

    // initialize dac
    dacWrite2 (0x0F, 0x00, 0x00); // write to buffer and dac b
    dacWrite2 (0x0F, 0x80, 0x00); // xfer buffer to dac b and write to dac a
    dacWrite2 (0x0F, 0x90, 0x01); // fast mode, powered up, use 1.024V internal ref voltage
    dacWrite2 (0x0F, 0x00, 0x00); // write to buffer and dac b
    dacWrite2 (0x0F, 0x80, 0x00); // xfer buffer to dac b and write to dac a

	// hello world
	printf ("\n\nHello, world!\n");

    // set up command processor
    InitCommand ();

	// create timer
    add_repeating_timer_ms (-10, repeating_timer_callback, NULL, &timer100Hz);

	// main loop
	while (1) {
	
		//----------------------------------------
		// run fast tasks
		//----------------------------------------

		// tiny usb tasks
		tud_task();

        // run get command state machine to get a line of input (non-blocking)
        GetCommand ();

        // once a line of input is received, process it
        if (cmd_state == 2) {
			// TODO
			cmd_state = 0;
		}

		
		//----------------------------------------
		// run 100Hz tasks
		//----------------------------------------

		if (flag100) {
			flag100 = false;

			if (ledTimer == 0) {												// first blink
				gpio_put (LED_PIN, 1);
			} else if (ledTimer == 2*7) {
				gpio_put (LED_PIN, 0);
			} else if ((ledTimer == 2*14) && (usbState >= USB_NOT_MOUNTED)) {	// second blink
				gpio_put (LED_PIN, 1);
			} else if (ledTimer == 2*21) {
				gpio_put (LED_PIN, 0);
			} else if ((ledTimer == 2*28) && (usbState >= USB_CONFIGURED)) {	// third blink
				gpio_put (LED_PIN, 1);
			} else if (ledTimer == 2*35) {
				gpio_put (LED_PIN, 0);
			}
            
			// increment led timer counter, 1.5 second period
			if (++ledTimer >= 2*75) {
				ledTimer = 0;
			}

		}
	}

	// not really
	return 0;
}


//---------------------------------------------------------------------------------------------
// InitGpioToOff
//

void InitGpioToOff (uint pin)
{
    gpio_init (pin);
    gpio_set_dir (pin, GPIO_OUT);
	gpio_put (pin, 0);
}


//---------------------------------------------------------------------------------------------
// Loop100Hz -- This is called at interrupt time so just set a flag then let the
//              main loop run the tasks.
//

bool repeating_timer_callback (struct repeating_timer *t)
{
	flag100 = true;

	return true;
}


//---------------------------------------------------------------------------------------------
// set usb state via TinyUSB callbacks
//

void tud_mount_cb (void)
{
	usbState = USB_CONFIGURED;
}

void tud_umount_cb (void)
{
	usbState = USB_NOT_MOUNTED;
}

void tud_suspend_cb (bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	usbState = USB_SUSPENDED;
}

void tud_resume_cb (void)
{
	usbState = USB_CONFIGURED;
}


//---------------------------------------------------------------------------------------------
// TinyUSB HID Get Report callback -- not used
//

uint16_t tud_hid_get_report_cb (uint8_t itf, uint8_t report_id, hid_report_type_t report_type, 
			uint8_t* buffer, uint16_t reqlen)
{
	(void) itf;
	(void) report_id;
	(void) report_type;
	(void) buffer;
	(void) reqlen;

	return 0;
}


//---------------------------------------------------------------------------------------------
// TinyUSB HID Set Report callback
//

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
	// this example doesn't use itf and report_type
	(void) itf;
	(void) report_type;

	for (int i = 0; i < bufsize; i++) {
		printf ("%02x ", buffer[i]);
	}
	printf ("\n");

	// check that we were called from hidd_xfer_cb in lib/tinyusb/src/class/hid/hid_device.c
	if (report_id == 0) {
		// check exactly six bytes in buffer
		if (bufsize == 6) {
			// check report id in first byte
			if (buffer[0] == 0x01) {
				if (buffer[1] & 0x80) {                // check mask
					DacWriteLevels (0x01, buffer[2]);  // limit and send levels to dac a and B
				}
				if (buffer[1] & 0x40) {                // check mask
					DacWriteLevels (0x02, buffer[3]);  // limit and send levels to dac a and B
				}
				if (buffer[1] & 0x20) {                // check mask
					DacWriteLevels (0x04, buffer[4]);  // limit and send levels to dac a and B
				}
				if (buffer[1] & 0x10) {                // check mask
					DacWriteLevels (0x08, buffer[5]);  // limit and send levels to dac a and B
				}
			}
		}
	}
}



//---------------------------------------------------------------------------------------------
// InitCommand
//

void InitCommand (void)
{
    cmd_state == 0;
    cmd_length = 0;
}


//---------------------------------------------------------------------------------------------
// GetCommand
//

void GetCommand (void)
{
    int ch;

    if (cmd_state == 0) {
        cmd_length = 0;
        cmd_buffer[cmd_length] = 0;
        printf ("CMD> ");
        cmd_state++;
    } else if (cmd_state == 1) {
        // get character
        ch = getchar_timeout_us (0);

        // process character
        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == 0x0d) {                           // return
                // carriage return and linefeed
                putchar (0x0d);
                putchar (0x0a);
                cmd_state++;
            } else if ((ch == 127) || (ch == 0x08)) {   // backspace
                if (cmd_length > 0) {
                    putchar (0x08);
                    putchar (' ');
                    putchar (0x08);
                    cmd_buffer[--cmd_length] = 0;
                }
            } else if (ch == 0x15) {                    // ctrl-u is rub out
                while (cmd_length > 0) {
                    putchar (0x08);
                    putchar (' ');
                    putchar (0x08);
                    cmd_buffer[--cmd_length] = 0;
                }
            } else if (ch >= 0x20 && ch <= 0x7e) {      // printable characters
                if (cmd_length < (CMD_MAXLEN - 1)) {
                    putchar (ch);
                    cmd_buffer[cmd_length++] = ch;
                    cmd_buffer[cmd_length] = 0;
                }
            }
        }
    }
}


//---------------------------------------------------------------------------------------------
// dacWrite2
//
// write two bytes to up to four DACs at a time
//

void dacWrite2 (uint8_t select, uint8_t a, uint8_t b)
{
    // printf ("%02x %02x\n", a, b);

    // CS low
    asm volatile ("nop \n nop \n nop");
    if (select & 1) {
        gpio_put (SPI0_CS0n_PIN, 0);
    }
    if (select & 2) {
        gpio_put (SPI0_CS1n_PIN, 0);
    }
    if (select & 4) {
        gpio_put (SPI1_CS0n_PIN, 0);
    }
    if (select & 8) {
        gpio_put (SPI1_CS1n_PIN, 0);
    }
    asm volatile ("nop \n nop \n nop");

    // transfer data
    if (select & 3) {
        spi_write_blocking (spi0, &a, 1);
        spi_write_blocking (spi0, &b, 1);
    }
    if (select & 12) {
        spi_write_blocking (spi1, &a, 1);
        spi_write_blocking (spi1, &b, 1);
    }

    // CS high
    asm volatile ("nop \n nop \n nop");
    gpio_put (SPI0_CS0n_PIN, 1);
    gpio_put (SPI0_CS1n_PIN, 1);
    gpio_put (SPI1_CS0n_PIN, 1);
    gpio_put (SPI1_CS1n_PIN, 1);
    asm volatile ("nop \n nop \n nop");

    // enforce a small delay between writes
    asm volatile ("nop \n nop \n nop");
    asm volatile ("nop \n nop \n nop");
    asm volatile ("nop \n nop \n nop");
    asm volatile ("nop \n nop \n nop");
    asm volatile ("nop \n nop \n nop");
}


//---------------------------------------------------------------------------------------------
// DacWriteLevels
//
// limit levels and calculate dac a and dac b values
// write levels to the selected DAC(s)
//

void DacWriteLevels (uint8_t select_mask, uint8_t level)
{
	uint8_t dacA, dacB;

	// limit levels and convert to DAC values
	if (level == 0) {
		dacA = 0;
		dacB = 0;
	} else {
		// if (level <  16) { level =  16; }
		// if (level > 248) { level = 248; }
		dacA = level;
		dacB = 256 - dacA;
	}

	// write dacA and dacB
	dacWrite2 (select_mask, 0x00 | ((dacB >> 4) & 0x0f),   // write buff and dac B
					        0x00 | ((dacB << 4) & 0xf0));
	dacWrite2 (select_mask, 0x80 | ((dacA >> 4) & 0x0f),   // write A, mv buf to B
					        0x00 | ((dacA << 4) & 0xf0));
}
