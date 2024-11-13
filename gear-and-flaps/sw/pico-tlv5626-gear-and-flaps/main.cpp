//---------------------------------------------------------------------------------------------
// notes
//
//
// <nose>,<right>,<left>,<flaps>

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


//---------------------------------------------------------------------------------------------
// defines
//

#define CMD_MAXLEN 72


//---------------------------------------------------------------------------------------------
// typedefs
//


//---------------------------------------------------------------------------------------------
// prototypes
//

bool repeating_timer_callback (struct repeating_timer *t);

void InitCommand (void);
void GetCommand (void);

void dacWrite2 (uint8_t select, uint8_t a, uint8_t b);


//---------------------------------------------------------------------------------------------
// globals
//

const uint LED_PIN = 25;

const uint SPI0_SCK_PIN  = 2;
const uint SPI0_MOSI_PIN = 3;
const uint SPI0_MISO_PIN = 4;
const uint SPI0_CS0n_PIN = 5;	// nose gear
const uint SPI0_CS1n_PIN = 6;	// right gear

const uint SPI1_SCK_PIN  = 14;
const uint SPI1_MOSI_PIN = 15;
const uint SPI1_MISO_PIN = 12;
const uint SPI1_CS0n_PIN = 13;	// flaps
const uint SPI1_CS1n_PIN = 11;	// left gear

volatile bool flag100 = false;

static char cmd_buffer[CMD_MAXLEN];
static uint8_t cmd_length = 0;
static uint8_t cmd_state = 0;


//---------------------------------------------------------------------------------------------
// main
//

int main ()
{
    // local system variables
    struct repeating_timer timer;
    uint8_t ledTimer;
	uint8_t dacState = 0;
	uint8_t dacA = 0, dacB = 255;

	// initialize stdio
    stdio_uart_init_full (uart0, 115200, 0, 1);
	
	// initialize led to off
    gpio_init (LED_PIN);
    gpio_set_dir (LED_PIN, GPIO_OUT);
	gpio_put (LED_PIN, 0);

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
	dacWrite2 (0x0F, 0x90, 0x02); // fast mode, powered up, use 2.048V internal ref voltage
	dacWrite2 (0x0F, 0x00, 0x00); // write to buffer and dac b
	dacWrite2 (0x0F, 0x80, 0x00); // xfer buffer to dac b and write to dac a

	// hello world
	printf ("Hello, world!\n");

    // set up command processor
    InitCommand ();

    // set up 10 ms / 100 Hz repeating timer
    add_repeating_timer_ms (-10, repeating_timer_callback, NULL, &timer);

	// main loop
	while (1) {

        //----------------------------------------
        // fast tasks
        //----------------------------------------

        // run get command state machine to get a line of input (non-blocking)
        GetCommand ();

        // once a line of input is received, process it
        if (cmd_state == 2) {
            printf ("processing command %s with length %d\n", cmd_buffer, cmd_length);

            int index = 0;
            char *buffptr = strtok (cmd_buffer, ",");
            while (buffptr != NULL) {

                int16_t a = atoi (buffptr);
				if (a == 0) {
					dacA = 0;
					dacB = 0;
				} else {
					if (a < 32) { a = 32; }
					if (a > 224) { a = 224; }
					dacA = a;
					dacB = 255 - dacA;
				}

                switch (index++) {

                    case 0:
						dacWrite2 (0x01, 0x00 | ((dacB >> 4) & 0x0f),	// write buff and dac B
                                         0x00 | ((dacB << 4) & 0xf0));
						dacWrite2 (0x01, 0x80 | ((dacA >> 4) & 0x0f),	// write A, mv buf to B
                                         0x00 | ((dacA << 4) & 0xf0));
                        break;

                    case 1:
						dacWrite2 (0x02, 0x00 | ((dacB >> 4) & 0x0f),	// write buff and dac B
                                         0x00 | ((dacB << 4) & 0xf0));
						dacWrite2 (0x02, 0x80 | ((dacA >> 4) & 0x0f),	// write A, mv buf to B
                                         0x00 | ((dacA << 4) & 0xf0));
                        break;

                    case 2:
						dacWrite2 (0x08, 0x00 | ((dacB >> 4) & 0x0f),	// write buff and dac B
                                         0x00 | ((dacB << 4) & 0xf0));
						dacWrite2 (0x08, 0x80 | ((dacA >> 4) & 0x0f),	// write A, mv buf to B
                                         0x00 | ((dacA << 4) & 0xf0));
                        break;

                    case 3:
						dacWrite2 (0x04, 0x00 | ((dacB >> 4) & 0x0f),	// write buff and dac B
                                         0x00 | ((dacB << 4) & 0xf0));
						dacWrite2 (0x04, 0x80 | ((dacA >> 4) & 0x0f),	// write A, mv buf to B
                                         0x00 | ((dacA << 4) & 0xf0));
                        break;
                }
                buffptr = strtok (NULL, ",");
            }

            cmd_state = 0;
        }


        //----------------------------------------
        // 100 Hz tasks
        //----------------------------------------

        if (flag100) {
            flag100 = false;

            // blihk led
            if (ledTimer == 0) {
                // led on
				gpio_put (LED_PIN, 1);
            } else if (ledTimer == 25) {
                // led off
				gpio_put (LED_PIN, 0);
            }

            // increment led timer counter, 1 second period
            if (++ledTimer >= 100) {
                ledTimer = 0;
            }

/*
			dacWrite2 (0x10 | ((dacB >> 4) & 0x0f),	// write buff and dac B
					   0x00 | ((dacB << 4) & 0xf0));
			dacWrite2 (0x80 | ((dacA >> 4) & 0x0f),	// write dac A, mv buff to B
					   0x00 | ((dacA << 4) & 0xf0));
*/
			if (dacA == 255) {
				dacState = 1;
			}
			if (dacA == 0) {
				dacState = 0;
			}
			if (dacState == 0) {
				dacA++;
				dacB--;
			} else {
				dacA--;
				dacB++;
			}

        }
	}

	// not really
	return 0;
}


bool repeating_timer_callback (struct repeating_timer *t)
{
    flag100 = true;

    return true;
}


void InitCommand (void)
{
    cmd_state == 0;
    cmd_length = 0;
}


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
            } else if (ch == 0x08) {                    // backspace
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
