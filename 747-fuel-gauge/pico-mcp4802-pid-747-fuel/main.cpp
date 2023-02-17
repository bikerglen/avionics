//---------------------------------------------------------------------------------------------
// notes
//
// make && openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program fuel747.elf verify reset ; init ; reset halt ; rp2040.core1 arp_reset assert 0 ; rp2040.core0 arp_reset assert 0 ; exit"
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

#include "pwl.h"


//---------------------------------------------------------------------------------------------
// defines
//

#define CMD_MAXLEN 72

// #define SCALING_USER_MIN  ( 0.00)
// #define SCALING_USER_MAX  (34.10)
// #define SCALING_ADC_MIN   (  149)
// #define SCALING_ADC_MAX   ( 4089)

#define Ts (1.0/100.0)

#define KP (1.0/ 48.0) // (1.0/ 64.0)
#define KD (1.0/ 96.0) // (1.0/256.0)
#define KI (1.0/ 32.0) // (1.0/ 64.0)

#define Imax (128.0)
#define alpha (0.9)


//---------------------------------------------------------------------------------------------
// typedefs
//


//---------------------------------------------------------------------------------------------
// prototypes - core 0
//

bool repeating_timer_callback_100Hz (struct repeating_timer *t);

void InitCommand (void);
void GetCommand (void);

int16_t FilterPosition (int16_t in);


//---------------------------------------------------------------------------------------------
// prototypes - core 1
//

void core1_entry (void);
bool repeating_timer_callback_40kHz (struct repeating_timer *t);
void dacWrite16 (uint cs_pin, uint16_t a);


//---------------------------------------------------------------------------------------------
// globals
//

const uint LED_PIN = 25;

const uint SPI_CS2n_PIN = 10;
const uint SPI_CS1n_PIN = 11;
const uint SPI_MISO_PIN = 12;
const uint SPI_CS0n_PIN = 13;
const uint SPI_SCK_PIN  = 14;
const uint SPI_MOSI_PIN = 15;

#define SPI_IF spi1

volatile bool flag100 = false;

static char cmd_buffer[CMD_MAXLEN];
static uint8_t cmd_length = 0;
static uint8_t cmd_state = 0;

static volatile uint8_t sin_phase = 0;
static volatile uint8_t dac0B = 0;
static volatile uint8_t dac1B = 0;
static volatile float scale = 0.0;

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


//---------------------------------------------------------------------------------------------
// main
//

int main ()
{
    // local system variables
    struct repeating_timer timer_100Hz;
	uint8_t ledTimer;

	// pid state variables
	float   sumError = 0;
	int16_t lastError = 0;
	float   deltaError = 0;
	float	previousFilterEstimate = 0;

	// pid transient variables
	int16_t target = pwl_interp (0.0);
	uint16_t adc_result;
	int16_t position;
	int16_t error;
	float   currentFilterEstimate; 
	float   pTerm, iTerm, dTerm;

	// adc avg read command variables
	int i, sum;

	// initialize stdio
    stdio_uart_init_full (uart0, 115200, 0, 1);
	
	// initialize led to off
    gpio_init (LED_PIN);
    gpio_set_dir (LED_PIN, GPIO_OUT);
	gpio_put (LED_PIN, 0);

	// initialize spi
    gpio_init    (SPI_CS0n_PIN);
    gpio_set_dir (SPI_CS0n_PIN, GPIO_OUT);
	gpio_put     (SPI_CS0n_PIN, 1);
    gpio_init    (SPI_CS1n_PIN);
    gpio_set_dir (SPI_CS1n_PIN, GPIO_OUT);
	gpio_put     (SPI_CS1n_PIN, 1);
    spi_init (SPI_IF, 8000000);
    spi_set_format (SPI_IF, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function (SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function (SPI_MOSI_PIN, GPIO_FUNC_SPI);

	// initialize dac
	dacWrite16 (SPI_CS0n_PIN, 0x3800); // write 0x800 to DAC 0 A
	dacWrite16 (SPI_CS0n_PIN, 0xB000); // write 0x000 to DAC 0 B
	dacWrite16 (SPI_CS1n_PIN, 0x3800); // write 0x800 to DAC 1 A
	dacWrite16 (SPI_CS1n_PIN, 0xB000); // write 0x000 to DAC 1 B
	dac0B = 0;
	dac1B = 0;

	// initialize adc
	adc_init ();
	adc_gpio_init (28);
	adc_select_input (2);

	// hello world
	printf ("Hello, world!\n");

    // set up command processor
    InitCommand ();

    // set up 10 ms / 100 Hz repeating timer on core 0
    add_repeating_timer_ms (-10, repeating_timer_callback_100Hz, NULL, &timer_100Hz);

	// initialize critical section
	critical_section_init (&scale_critsec);

	// start core 1 tasks
	multicore_launch_core1 (core1_entry);

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
						if (!strcmp (buffptr, "a")) {
							sum = 0;
							for (i = 0; i < 1024; i++) {
								sum += adc_read ();
							}
							sum = round (sum / 1024.0);
							printf ("avg: %d\n", sum);
							printf ("scale: %6.3f p: %6.3f, i: %6.3f, d: %6.3f\n", scale, pTerm, iTerm, dTerm);
							break;
						}
						target = pwl_interp (atof (buffptr));
						target = (target > 4095) ? 4095 : target;
						target = (target <    0) ?    0 : target;
						printf ("target = %d\n", target);
                        break;

                    case 1:
                        printf ("nothing happens.\n");
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


			//----------------------------------------
			// run pid loop
			//----------------------------------------

			// get adc reading -- nope
			// adc_result = adc_read ();
			// position = adc_result;

			// get lots of adc readings
			sum = 0;
			for (i = 0; i < 512; i++) {
				sum += adc_read ();
			}
			position = round (sum / 512.0);

			// filter position
			// position = FilterPosition (position);

			// calculate error
			error = target - position;

			// calculate P term
			pTerm = KP * error;

			// calculate I term
			sumError += error * Ts;
			if (sumError > Imax*Ts) sumError = Imax*Ts;
			if (sumError <= -Imax*Ts) sumError = -Imax*Ts;
			iTerm = KI * sumError;

			// calculate D term
			deltaError = error - lastError;
			lastError = error;
			currentFilterEstimate = (alpha*previousFilterEstimate) + (1-alpha)*deltaError;
			previousFilterEstimate = currentFilterEstimate;
			dTerm = KD * currentFilterEstimate / Ts;

			// add terms together
			float newScale = pTerm + iTerm + dTerm;

			// saturate result
			if (newScale > 1.0) newScale = 1.0;
			if (newScale < -1.0) newScale = -1.0;

			// update speed and direction for core 1 ISR
			critical_section_enter_blocking (&scale_critsec);
			scale = newScale;
			critical_section_exit (&scale_critsec);
        }
	}

	// not really
	return 0;
}


bool repeating_timer_callback_100Hz (struct repeating_timer *t)
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


//
// y(n) = b0x(n) + b1x(n–1) + b2x(n–2) – a1y(n–1) – a2y(n–2)
// 

/*
// [b,a]=butter_synth(5,10,100)
#define b0 ( 0.0012826)
#define b1 ( 0.0064129)
#define b2 ( 0.0128258)
#define b3 ( 0.0128258)
#define b4 ( 0.0064129)
#define b5 ( 0.0012826)
#define a1 (-2.97542)
#define a2 ( 3.80602)
#define a3 (-2.54525)
#define a4 ( 0.88113)
#define a5 (-0.12543)
*/

// [b,a]=butter_synth(2,10,100)
#define b0 (0.067455)
#define b1 (0.134911)
#define b2 (0.067455)
#define a1 (-1.14298)
#define a2 ( 0.41280)

int16_t FilterPosition (int16_t x0)
{
	int16_t y0;
	static float x1, x2;
	static float y1, y2;

	y0 = b0*x0 + b1*x1 + b2*x2
               - a1*y1 - a2*y2;

	x2 = x1;
	x1 = x0;
	y2 = y1;
	y1 = y0;

	return y0;

/*
	int16_t y0;
	static float x1, x2, x3, x4, x5;
	static float y1, y2, y3, y4, y5;

	y0 = b0*x0 + b1*x1 + b2*x2 + b3*x3 + b4*x4 + b5*x5
               - a1*y1 - a2*y2 - a3*y3 - a4*y4 - a5*y5;

	x5 = x4;
	x4 = x3;
	x3 = x2;
	x2 = x1;
	x1 = x0;

	y5 = y4;
	y4 = y3;
	y3 = y2;
	y2 = y1;
	y1 = y0;

	// printf ("%d %d\n", x0, y0);

	return y0;
*/
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
	gpio_put (SPI_CS0n_PIN, 0);
	spi_write16_blocking (SPI_IF, &a, 1);
	gpio_put (SPI_CS0n_PIN, 1);

	a = 0xB000 | ((uint16_t)dac1B << 4);
	gpio_put (SPI_CS1n_PIN, 0);
	spi_write16_blocking (SPI_IF, &a, 1);
	gpio_put (SPI_CS1n_PIN, 1);

	if (++sin_phase >= 100) {
		sin_phase = 0;
	}

	critical_section_enter_blocking (&scale_critsec);
	dac0B = 128+sine[sin_phase];
	dac1B = 128+scale*sine[sin_phase];
	critical_section_exit (&scale_critsec);

	return true;
}


void dacWrite16 (uint cs_pin, uint16_t a)
{
	// CS low
    asm volatile ("nop \n nop \n nop");
	gpio_put (cs_pin, 0);
    asm volatile ("nop \n nop \n nop");

	// transfer data
	spi_write16_blocking (SPI_IF, &a, 1);

	// CS high
    asm volatile ("nop \n nop \n nop");
	gpio_put (cs_pin, 1);
    asm volatile ("nop \n nop \n nop");
}
