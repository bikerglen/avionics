//---------------------------------------------------------------------------------------------
// notes
//
// make && openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program sin400.elf verify reset ; init ; reset halt ; rp2040.core1 arp_reset assert 0 ; rp2040.core0 arp_reset assert 0 ; exit"
//

//---------------------------------------------------------------------------------------------
// includes
//

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/adc.h"


//---------------------------------------------------------------------------------------------
// defines
//

#define CMD_MAXLEN 72


//---------------------------------------------------------------------------------------------
// typedefs
//


//---------------------------------------------------------------------------------------------
// prototypes - core 0
//

bool repeating_timer_callback_200Hz (struct repeating_timer *t);

void InitCommand (void);
void GetCommand (void);


//---------------------------------------------------------------------------------------------
// prototypes - core 1
//

void core1_entry (void);
bool repeating_timer_callback_40kHz (struct repeating_timer *t);
void dacWrite16 (spi_inst_t *spi, uint cs_pin, uint16_t a);


//---------------------------------------------------------------------------------------------
// globals
//

const uint LED_PIN = 25;

const uint SPI0_SCK_PIN  = 2;
const uint SPI0_MOSI_PIN = 3;
const uint SPI0_MISO_PIN = 4;
const uint SPI0_CS0n_PIN = 5;
const uint SPI0_CS1n_PIN = 6;

const uint SPI1_MISO_PIN = 12;
const uint SPI1_CS0n_PIN = 13;
const uint SPI1_SCK_PIN  = 14;
const uint SPI1_MOSI_PIN = 15;

volatile bool flag100 = false;

static char cmd_buffer[CMD_MAXLEN];
static uint8_t cmd_length = 0;
static uint8_t cmd_state = 0;

static volatile uint8_t sin_phase = 0;
static volatile uint8_t dac0B = 0; 		// SPI 0, CS 0
static volatile uint8_t dac1B = 0; 		// SPI 0, CS 1
static volatile uint8_t dac2B = 0; 		// SPI 1, CS 0
static volatile float scaleDac0 = 0.0;
static volatile float scaleDac1 = 0.0;
static volatile float scaleDac2 = -1.0;

// critical section for communicating between the two cores
critical_section_t scale_critsec;

// sine lookup table
// a=sin((0:99)*2*pi/100);
// b=round(a*127);
// min(b) ans = -127
// max(b) ans =  127
// b(1)   ans =  0
static const int8_t sine[100] = {
     0,    8,   16,   24,   32,   39,   47,   54,   61,   68,   
    75,   81,   87,   93,   98,  103,  107,  111,  115,  118,  
   121,  123,  125,  126,  127,  127,  127,  126,  125,  123,  
   121,  118,  115,  111,  107,  103,   98,   93,   87,   81,   
    75,   68,   61,   54,   47,   39,   32,   24,   16,    8,    
     0,   -8,  -16,  -24,  -32,  -39,  -47,  -54,  -61,  -68,  
   -75,  -81,  -87,  -93,  -98, -103, -107, -111, -115, -118, 
  -121, -123, -125, -126, -127, -127, -127, -126, -125, -123, 
  -121, -118, -115, -111, -107, -103,  -98,  -93,  -87,  -81,  
   -75,  -68,  -61,  -54,  -47,  -39,  -32,  -24,  -16,   -8
};

/*
static const int8_t sine[100] = {
   127,  127,  127,  127,  127,  127,  127,  127,  127,  127,   
   127,  127,  127,  127,  127,  127,  127,  127,  127,  127,   
   127,  127,  127,  127,  127,  127,  127,  127,  127,  127,   
   127,  127,  127,  127,  127,  127,  127,  127,  127,  127,   
   127,  127,  127,  127,  127,  127,  127,  127,  127,  127,   
  -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,   
  -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,   
  -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,   
  -127, -127, -127, -127, -127, -127, -127, -127, -127, -127,   
  -127, -127, -127, -127, -127, -127, -127, -127, -127, -127
};
*/


//---------------------------------------------------------------------------------------------
// main
//

int main ()
{
    // local system variables
    struct repeating_timer timer_200Hz;
	uint8_t ledTimer;

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
    spi_init (spi0, 8000000);
    spi_set_format (spi0, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function (SPI0_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI0_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI0_MOSI_PIN, GPIO_FUNC_SPI);

	// initialize spi 1
    gpio_init    (SPI1_CS0n_PIN);
    gpio_set_dir (SPI1_CS0n_PIN, GPIO_OUT);
	gpio_put     (SPI1_CS0n_PIN, 1);
    spi_init (spi1, 8000000);
    spi_set_format (spi1, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function (SPI1_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI1_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI1_MOSI_PIN, GPIO_FUNC_SPI);

	// initialize dacs on spi 0
	dacWrite16 (spi0, SPI0_CS0n_PIN, 0x3800); // write 0x800 to DAC 0 A
	dacWrite16 (spi0, SPI0_CS0n_PIN, 0xB000); // write 0x000 to DAC 0 B
	dacWrite16 (spi0, SPI0_CS1n_PIN, 0x3800); // write 0x800 to DAC 1 A
	dacWrite16 (spi0, SPI0_CS1n_PIN, 0xB000); // write 0x000 to DAC 1 B
	dac0B = 0;
	dac1B = 0;

	// initialize dacs on spi 1
	dacWrite16 (spi1, SPI1_CS0n_PIN, 0x3800); // write 0x800 to DAC 2 A
	dacWrite16 (spi1, SPI1_CS0n_PIN, 0xB000); // write 0x000 to DAC 2 B
	dac2B = 0;

	// hello world
	printf ("Hello, world!\n");

    // set up command processor
    InitCommand ();

    // set up 5 ms / 200 Hz repeating timer on core 0
    add_repeating_timer_ms (-5, repeating_timer_callback_200Hz, NULL, &timer_200Hz);

	// initialize critical section
	critical_section_init (&scale_critsec);

	// start core 1 tasks
	multicore_launch_core1 (core1_entry);

	float target = 0, theta = 0, newScale0 = 0, newScale1 = 0;

	// main loop
	while (1) {


        //----------------------------------------
        // fast tasks
        //----------------------------------------

        // run get command state machine to get a line of input (non-blocking)
        GetCommand ();

        // once a line of input is received, process it
        if (cmd_state == 2) {
            int index = 0;
            char *buffptr = strtok (cmd_buffer, ",");
            while (buffptr != NULL) {

                switch (index++) {

                    case 0:
						target = fmod (atof (buffptr), 360.0);
						newScale0 =  sin ((target + 120)*M_PI/180.0); // s3 / blue
						newScale1 = -sin ((target + 240)*M_PI/180.0); // s1 / yellow
						printf ("               YL-BU  BU-BK   BK-YL\n");
						printf ("target: %6.0f %6.2f %6.2f %6.2f\n", 
							target,                // target angle
							newScale1 - newScale0, // target s1-s3
                            newScale0 - 0,         // target s3-s2
                            0 - newScale1);		   // target s2-s1
                        break;

                    case 1:
                        printf ("nothing happens (1).\n");
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

			// calculate next theta based on current theta and target set through CLI

			// immediate move
			// theta = target;

			// gradual move
			float diff = fmod ((target - theta + 180), 360) - 180;
			diff = diff < -180 ? diff + 360 : diff;
			if (fabs(diff) == 180) { // always move CW for 180 degree difference
				theta++;
			} else if (diff < 0) {   // move CCW
				theta--;
			} else if (diff > 0) {   // move CW
				theta++;
			} else {          		 // no move needed
				// nothing 
			}

			// move to theta
			newScale0 =  sin ((theta + 120)*M_PI/180.0); // s3 / blue
			newScale1 = -sin ((theta + 240)*M_PI/180.0); // s1 / yellow
			critical_section_enter_blocking (&scale_critsec);
			scaleDac0 = newScale0;
			scaleDac1 = newScale1;
			critical_section_exit (&scale_critsec);

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
        }
	}

	// not really
	return 0;
}


bool repeating_timer_callback_200Hz (struct repeating_timer *t)
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
            } else if ((ch == 0x08) || (ch == 0x7F)) {  // backspace or delete
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


//=============================================================================================
// core 1 tasks -- keep the sine waves going
//

void core1_entry (void)
{
    // local system variables
	alarm_pool_t *core1_alarm_pool;
    struct repeating_timer timer_40kHz;
	
	// create new alarm pool
    core1_alarm_pool = alarm_pool_create (2, 16);

	// run 40 kHz timer interrupt on core 1
    alarm_pool_add_repeating_timer_us (core1_alarm_pool, 
		-25, repeating_timer_callback_40kHz, NULL, &timer_40kHz);

	// nothing else to do on core 1
	while (1) {
		tight_loop_contents ();
	}
}


bool repeating_timer_callback_40kHz (struct repeating_timer *t)
{
	uint16_t a;

	a = 0xB000 | ((uint16_t)dac0B << 4);
	gpio_put (SPI0_CS0n_PIN, 0);
	spi_write16_blocking (spi0, &a, 1);
	gpio_put (SPI0_CS0n_PIN, 1);

	a = 0xB000 | ((uint16_t)dac1B << 4);
	gpio_put (SPI0_CS1n_PIN, 0);
	spi_write16_blocking (spi0, &a, 1);
	gpio_put (SPI0_CS1n_PIN, 1);

	a = 0xB000 | ((uint16_t)dac2B << 4);
	gpio_put (SPI1_CS0n_PIN, 0);
	spi_write16_blocking (spi1, &a, 1);
	gpio_put (SPI1_CS0n_PIN, 1);

	if (++sin_phase >= 100) {
		sin_phase = 0;
	}

	critical_section_enter_blocking (&scale_critsec);
	dac0B = 128+scaleDac0*sine[sin_phase];
	dac1B = 128+scaleDac1*sine[sin_phase];
	dac2B = 128+scaleDac2*sine[sin_phase];
	critical_section_exit (&scale_critsec);

	return true;
}


void dacWrite16 (spi_inst_t *spi, uint cs_pin, uint16_t a)
{
	// CS low
    asm volatile ("nop \n nop \n nop");
	gpio_put (cs_pin, 0);
    asm volatile ("nop \n nop \n nop");

	// transfer data
	spi_write16_blocking (spi, &a, 1);

	// CS high
    asm volatile ("nop \n nop \n nop");
	gpio_put (cs_pin, 1);
    asm volatile ("nop \n nop \n nop");
}
