/********************************************************
 Name          : main.c
 Author        : Jordan Murray
 Copyright     : Not really
 Description   : EVK1100 template
 **********************************************************/
/*
 * GTIcontrol.c
 *
 *  Created on: Jan 8, 2013
 *
 */

#include "board.h"
#include "gpio.h"
#include "pm.h"
#include "adc.h"
#include "dip204.h"
#include "spi.h"
#include "pwm.h"
#include "delay.h"
#include "intc.h"
#include "rtc.h"
#include "math.h"
#include "flashc.h"
#include "compiler.h"
#include "macb.h"

#define FOSCO						12000000
#define RTC_FREQ					16000
#define RTC_PRESCALAR				0
#define PWM_PERIOD					256
#define PWM_SCALE					220
#define PERIOD_TOLERANCE			15
#define DISPLAY_INTERVAL    		16000
#define PACKET_CHECK_INTERVAL       200
#define MEASUREMENT_INTERVAL 		500
#define CALC_INTERVAL				800
#define INITIAL_TICK				0

//ADC
#define VOLTAGE_OUTPUT_CHANNEL		2
#define VOLTAGE_OUTPUT_FUNCTION		AVR32_ADC_AD_2_FUNCTION
#define VOLTAGE_OUTPUT_PIN			AVR32_ADC_AD_2_PIN

#define CURRENT_OUTPUT_CHANNEL		7
#define CURRENT_OUTPUT_FUNCTION		AVR32_ADC_AD_7_FUNCTION
#define CURRENT_OUTPUT_PIN			AVR32_ADC_AD_7_PIN

#define EDGE_DETECTION_PIN          AVR32_PIN_PX20   //actually PX20
#define CURRENT_EDGE_DETECTION_PIN	AVR32_PIN_PX08   // (pin 33) actually PX11  --changed the code on 11/24/2015 to compensate for broken pin on test micro
#define VOLTAGE_EDGE_DETECTION_PIN	AVR32_PIN_PX06
#define CONNECTION_PIN				AVR32_PIN_PX26

#define LOW_SIDE_PIN				AVR32_PIN_PX14

//PWM
#define HIGH_FREQ_PIN               AVR32_PIN_PA05
#define HIGH_FREQ_FUNCTION			1
#define HIGH_FREQ_CHANNEL_ID		4


#define GRID_FOLLOWING				0
#define STANDALONE					1

#define COMM_DEBUG                  0

//NETWORKING MACROS

const unsigned short GRID_PROTOCOL_ID  =         0xa3cf;

#define GRID_PROTOCOL_SCOPE_INVERTER                0x01

//defines values corresponding to various TYPE CODES
const unsigned short GRID_PROTOCOL_MISDIRECTED_MESSAGE 		=	0x0000;
const unsigned short GRID_PROTOCOL_UNIMPLEMENTED_PROTOCOL	=	0x0001;
const unsigned short GRID_PROTOCOL_UNIMPLEMENTED_FUNCTION	=	0x0002;
const unsigned short GRID_PROTOCOL_BAD_UDP_CHECKSUM			=	0x0003;
const unsigned short GRID_PROTOCOL_ACK						=	0x0004;

const unsigned short GRID_PROTOCOL_PHASE_ADJUST				=	0x1000;
const unsigned short GRID_PROTOCOL_AMP_ADJUST				=	0x1001;
const unsigned short GRID_PROTOCOL_REAL_POWER_ADJUST		= 	0x1002;
const unsigned short GRID_PROTOCOL_REACTIVE_POWER_ADJUST	=	0x1003;
const unsigned short GRID_PROTOCOL_FREQUENCY_ADJUST			=	0x1004;

const unsigned short GRID_PROTOCOL_MODE_GRID_FOLLOWING		=	0x2000;
const unsigned short GRID_PROTOCOL_MODE_STANDALONE			=	0x2001;
const unsigned short GRID_PROTOCOL_DISCONNECT				=	0x2002;
const unsigned short GRID_PROTOCOL_CONNECT					=	0x2003;
const unsigned short GRID_PROTOCOL_LOCK_SCREEN				=	0x2004;
const unsigned short GRID_PROTOCOL_UNLOCK_SCREEN			=	0x2005;
const unsigned short GRID_PROTOCOL_SCREEN_1					=	0x2006;
const unsigned short GRID_PROTOCOL_SCREEN_2					=	0x2007;
const unsigned short GRID_PROTOCOL_SCREEN_3					=	0x2008;
const unsigned short GRID_PROTOCOL_SCREEN_4					=	0x2009;


const unsigned short GRID_PROTOCOL_REQUEST_PHASE			=	0x3000;
const unsigned short GRID_PROTOCOL_REQUEST_AMP				=	0x3001;
const unsigned short GRID_PROTOCOL_REQUEST_MODE				=	0x3002;
const unsigned short GRID_PROTOCOL_REQUEST_PWM_PERIOD		=	0x3003;
const unsigned short GRID_PROTOCOL_REQUEST_NSAMPLES			=	0x3004;
const unsigned short GRID_PROTOCOL_REQUEST_OUTPUT_VOLTAGE	=	0x3005;
const unsigned short GRID_PROTOCOL_REQUEST_OUTPUT_CURRENT	=	0x3006;
const unsigned short GRID_PROTOCOL_REQUEST_REAL_POWER		=	0x3007;
const unsigned short GRID_PROTOCOL_REQUEST_REACTIVE_POWER	=	0x3008;


const unsigned short GRID_PROTOCOL_RESPONSE_PHASE			=   0x4000;
const unsigned short GRID_PROTOCOL_RESPONSE_AMP				=   0x4001;
const unsigned short GRID_PROTOCOL_RESPONSE_MODE			=   0x4002;
const unsigned short GRID_PROTOCOL_RESPONSE_PWM_PERIOD		=	0x4003;
const unsigned short GRID_PROTOCOL_RESPONSE_NSAMPLES		= 	0x4004;
const unsigned short GRID_PROTOCOL_RESPONSE_OUTPUT_VOLTAGE	=	0x4005;
const unsigned short GRID_PROTOCOL_RESPONSE_OUTPUT_CURRENT	=	0x4006;
const unsigned short GRID_PROTOCOL_RESPONSE_REAL_POWER		=	0x4007;
const unsigned short GRID_PROTOCOL_RESPONSE_REACTIVE_POWER	=	0x4008;

//defines locations of various fields in the grid protocol frame
const unsigned short GRID_PROTOCOL_SCOPE_FIELD                =   44;
const unsigned short GRID_PROTOCOL_MSG_COMPS_FIELD           =    45;
const unsigned short GRID_PROTOCOL_DATA_FIELD               =     50;    //not typically used
const unsigned short GRID_PROTOCOL_SEQNUM_FIELD            =      46;
const unsigned short GRID_PROTOCOL_VALUE_FIELD           =        50;
const unsigned short GRID_PROTOCOL_TYPE_FIELD           =         48;
const unsigned short GRID_PROTOCOL_ID_FIELD           =           42;



//values for switch statements


#define UNIMPLEMENTED_PROTOCOL						1
#define UNIMPLEMENTED_FUNCTION						2
#define BAD_UDP_CHECKSUM							3
#define ACK											4
#define PHASE_ADJUST								5
#define AMP_ADJUST									6
#define REAL_POWER_ADJUST							7
#define REACTIVE_POWER_ADJUST						8
#define FREQUENCY_ADJUST							9
#define MODE_GRID_FOLLOWING							10
#define MODE_STANDALONE								11
#define DISCONNECT									12
#define CONNECT										13
#define LOCK_SCREEN									14
#define UNLOCK_SCREEN								15
#define SCREEN1										16
#define SCREEN2										17
#define SCREEN3										18
#define SCREEN4										19
#define REQUEST_PHASE								20
#define REQUEST_AMP									21
#define REQUEST_MODE								22
#define REQUEST_PWM_PERIOD							23
#define REQUEST_NSAMPLES							24
#define RESPONSE_PHASE								25
#define RESPONSE_AMP								26
#define RESPONSE_MODE								27
#define RESPONSE_PWM_PERIOD							28
#define RESPONSE_NSAMPLES							29
#define REQUEST_OUTPUT_VOLTAGE						30
#define REQUEST_OUTPUT_CURRENT						31
#define REQUEST_REAL_POWER							32
#define REQUEST_REACTIVE_POWER						33
#define RESPONSE_OUTPUT_VOLTAGE						34
#define RESPONSE_OUTPUT_CURRENT						35
#define RESPONSE_REAL_POWER							36
#define RESPONSE_REACTIVE_POWER						37
#define MISDIRECTED_MESSAGE							38

/*! initial sequence number for ICMP request and reply */
#define SEQ_NUM_START 0x2546

//declaration of PWM variables
pwm_opt_t HIGH_FREQ_opt;
avr32_pwm_channel_t HIGH_FREQ_channel = { .ccnt = 0 };  // One channel config.
unsigned int HIGH_FREQ_id;
volatile avr32_pm_t *pm = &AVR32_PM;        // power manager address

volatile avr32_adc_t *adc=&AVR32_ADC;  		//ADC IP registers address//**************************************************************
//VARIABLE DECLARATIONS
//**************************************************************


// sine data
int sine_data[400];
int comp_sine_data[400];

//general
volatile U32 rtc_tick = INITIAL_TICK;
volatile U32 last_crossing_time=INITIAL_TICK;
U32 last_rising_edge_time=INITIAL_TICK;
U32 rising_edge_time=INITIAL_TICK;
U32 voltage_rising_edge_time = INITIAL_TICK;
U32 current_rising_edge_time = INITIAL_TICK;
U32 last_update_tick=INITIAL_TICK;
U32 last_measurement_tick = INITIAL_TICK;
U32 last_calc_tick = INITIAL_TICK;
U32 last_check_tick = INITIAL_TICK;
U32 timer=INITIAL_TICK;
U32 start=INITIAL_TICK;


int sine_wave_index=0;
int data_length=128;
int trigger_status=0;
int last_trigger_status=0;
int phase_samples=0;
int last_phase_samples=0;
int half_period_samples=133;
int period_samples=266;
int temp_period_samples=266;
int last_period_samples=266;
int relative_phase;
int debug_mode=1;
int phase_adjust=0;
int false_period=0;
int cycle_half=0;
int low_side_adjust=0;
double pwm_amp_scale_factor=.62;
int resample_flag=0;
int resample_req=0;
int phase_count=0;
int data_id;
int send_data_flag = 0;
int waiting_for_current = 1;
int pwm_real_adj = 0;
int rtc_int_freq = 16000;
int set_low_side_pin_flag = 0;
int clr_low_side_pin_flag = 0;

unsigned short GP_seqnum = 0;
unsigned short return_udp = 30000;




//HMI associated variables
int op_mode= 0;
int temp_op_mode=0;
int option_ID=0;
int num_options=6;
int set_values=0;
int temp_period=266;
int temp_phase=0;
int phase_inc=5;
int period_inc=2;
int i;
int last_case = 0;
int display_counter = 0;
int display_index = 0;
int display_locked = 0;
int display_selection = 1;
int recd_packet_count = 0;
int sent_packet_count = 0;
int connected = 0;
char packet_summary[14] = "just chillin' ";

double peak_iout = 0;
double peak_vout = 0;
double pow_reac_target = 0;
double pow_real_target = 0;
double pow_angle = 0;
double pf = 0;
double pow_apparent = 0;
double pow_real = 0;
double pow_reactive = 0;
double output_voltage_amp = 0;
double output_current_amp = 0;


//debug variables
int vedge_count = 0;
int iedge_count = 0;
int refedge_count = 0;
int goodrefedge_count = 0;
int disable_checksum_validation = 1;

// networking stuff
/*! buffer for sending packets */
unsigned char data[ETHERNET_CONF_TX_BUFFER_SIZE];
/* The IP and Ethernet addresses are read from the header files. */
unsigned char hwaddr[ 6 ] = { ETHERNET_CONF_ETHADDR0,ETHERNET_CONF_ETHADDR1,ETHERNET_CONF_ETHADDR2,ETHERNET_CONF_ETHADDR3,ETHERNET_CONF_ETHADDR4,ETHERNET_CONF_ETHADDR5 };

/*! sequence number for ICMP request and reply */
unsigned short seqnum = SEQ_NUM_START;

/*! buffer for sending packets */
const unsigned char local_ipaddr[4] = {ETHERNET_CONF_IPADDR0,ETHERNET_CONF_IPADDR1,ETHERNET_CONF_IPADDR2,ETHERNET_CONF_IPADDR3};
const unsigned char local_ethaddr[6] = {ETHERNET_CONF_ETHADDR0,ETHERNET_CONF_ETHADDR1,ETHERNET_CONF_ETHADDR2,ETHERNET_CONF_ETHADDR3,ETHERNET_CONF_ETHADDR4,ETHERNET_CONF_ETHADDR5};

/*! define the ARP global frame */
const unsigned char ARP_FRAME[42] = {
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
ETHERNET_CONF_ETHADDR0,ETHERNET_CONF_ETHADDR1,ETHERNET_CONF_ETHADDR2,ETHERNET_CONF_ETHADDR3,ETHERNET_CONF_ETHADDR4,ETHERNET_CONF_ETHADDR5,
0x08,0x06,0x00,0x01,0x08,0x00,0x06,0x04,0x00,0x01,
ETHERNET_CONF_ETHADDR0,ETHERNET_CONF_ETHADDR1,ETHERNET_CONF_ETHADDR2,ETHERNET_CONF_ETHADDR3,ETHERNET_CONF_ETHADDR4,ETHERNET_CONF_ETHADDR5,
ETHERNET_CONF_IPADDR0,ETHERNET_CONF_IPADDR1,ETHERNET_CONF_IPADDR2,ETHERNET_CONF_IPADDR3,
0x00,0x00,0x00,0x00,0x00,0x00,
ETHERNET_CONF_GATEWAY_ADDR0,ETHERNET_CONF_GATEWAY_ADDR1,ETHERNET_CONF_GATEWAY_ADDR2,ETHERNET_CONF_GATEWAY_ADDR3
};


//

int rtc_run=0;								//indicates that the rtc interrupt has run


int target_period_samples=266;
double target_phase=0;
int target_phase_samples=0;


float display_freq=0;
float display_phase=0;


//function declarations
void display_update(int mode);
int resample(int n_samples);

//RTC interrupt routine - runs at about 16kHz or 266 times per cycle/grid reference interrupt
__attribute__((__interrupt__))
static void RTC_irq(void)
{
	//rtc_tick is the base unit of time
    rtc_tick++;

	//if the phase measurement interrupt has determined adjustment is unnecessary, advance one sample
    //if we need to catch up, advance two
    //if we are too far ahead, repeat a sample
    if(phase_adjust == 0)
    {
    	sine_wave_index++;
    }
    else if(phase_adjust == 1)
    {
    	sine_wave_index = sine_wave_index + 2; 								//skip a sample
    	phase_count--;														//one less sample to skip
    }
    else if(phase_adjust == -1)
    {
    	//repeat a sample, but only on every third sample to minimize harmonics
    	//don't repeat in the beginning of the wave to avoid repeated zeros
    	if((sine_wave_index < 10) | (sine_wave_index%3))
    	{
    		sine_wave_index++;
    	}
    	else
    	{
    		phase_count--;													//one less sample to repeat
    	}
    }
    //if the counter reaches zero, we don't need to adjust the phase anymore
    if(phase_count == 0)
    {
    	phase_adjust = 0;
    }


    //if we advance beyond the length of the sine data, loop back to the beginning
    if(sine_wave_index >= data_length)
    {
    	sine_wave_index = 0;												//reset sine index
    	//toggle cycle half each time we run out of data
    	if(cycle_half == 0)
    	{
        	HIGH_FREQ_channel.cupd = comp_sine_data[sine_wave_index];
            pwm_sync_update_channel(HIGH_FREQ_CHANNEL_ID, &HIGH_FREQ_channel);
    		cycle_half = 1;
    		gpio_set_gpio_pin(LOW_SIDE_PIN);
    		//set_low_side_pin_flag = 1;
    	}
    	else
    	{

        	HIGH_FREQ_channel.cupd= sine_data[sine_wave_index];
            pwm_sync_update_channel(HIGH_FREQ_CHANNEL_ID, &HIGH_FREQ_channel);
            cycle_half = 0;
    		last_crossing_time = rtc_tick;									//defined as beginning for comparison to reference
    		gpio_clr_gpio_pin(LOW_SIDE_PIN);								//sets sine polarity
    		//clr_low_side_pin_flag = 1;
	        //loads appropriate data to update register
    	}
    }

    // the polarity of the high frequency switching signal depends on the state of the low frequency switches
    // using complimentary data accounts for this
    if(cycle_half == 1)
    {
    	HIGH_FREQ_channel.cupd = comp_sine_data[sine_wave_index];
    }
    else
    {
    	HIGH_FREQ_channel.cupd= sine_data[sine_wave_index];
    }

    //update duty cycle register from .cupd
    pwm_sync_update_channel(HIGH_FREQ_CHANNEL_ID, &HIGH_FREQ_channel);
    rtc_clear_interrupt(&AVR32_RTC);

}



__attribute__((__interrupt__))
static void rising_edge_interrupt_handler(void)
{
	//this interrupt is triggered by any of the gpio interrupts
	//was the grid reference interrupt triggered?
	if(gpio_get_pin_interrupt_flag(EDGE_DETECTION_PIN))
	{
		//in grid following mode attempt to follow a reference
		if(op_mode == GRID_FOLLOWING)
		{
			//record the time of the rising edge
			rising_edge_time=rtc_tick;

			//keeps track of interrupts for debugging
			refedge_count++;

			//calculate possible period in samples
			temp_period_samples=rising_edge_time-last_rising_edge_time;

			//sometimes the gpio rising edge interrupt triggers erroneously
			//this tends to happen earlier in the cycle, so it's easy to rule out false triggers
			//the following tests the period value for accuracy and then uses the last reasonable value if the test fails
			//the phase is then corrected based on the difference between the bad phase measurement and the last good one
			if(( fabs(temp_period_samples-266))>20)
			{
				false_period=temp_period_samples;
				//update phase based on historical mismatch between actual frequency and data frequency
				phase_samples=(int) last_phase_samples*false_period/period_samples;

			}
			else
			{
				if(debug_mode)
				{
					goodrefedge_count++;
				}

				//if the period passes the test, update the period using a weighted average
				period_samples=(int) (.8*period_samples+.2*temp_period_samples);
				//figure out how much further out of phase we are than we mean to be
				phase_samples=(rising_edge_time-last_crossing_time-target_phase_samples)%period_samples;

				//adjust phase up or down?
				//if the difference is greater than half the period, adjust the other way
				if(phase_samples<(period_samples/2))
				{
					phase_adjust = -1;

				}
				else if(phase_samples>=(period_samples/2))
				{
					phase_adjust=1;
					phase_samples=period_samples-phase_samples;
				}
			}

			last_rising_edge_time=rising_edge_time;
			last_phase_samples=phase_samples;
			phase_count=fabs(phase_samples);								//sets up phase counter for main loop
		}
		else if(op_mode == STANDALONE)
		{
			period_samples=target_period_samples;							//no need to try to adjust to track the reference
		}

		gpio_clear_pin_interrupt_flag(EDGE_DETECTION_PIN);
	}
	//run if the output voltage interrupt was triggered
	else if(gpio_get_pin_interrupt_flag(VOLTAGE_EDGE_DETECTION_PIN))
	{
	    voltage_rising_edge_time = rtc_tick;
	    waiting_for_current = 1;

	    if(debug_mode)
	    {
	    	vedge_count++;
	    }

		gpio_clear_pin_interrupt_flag(VOLTAGE_EDGE_DETECTION_PIN);
	}
	//run if the output current interrupt is triggered
	else if(gpio_get_pin_interrupt_flag(CURRENT_EDGE_DETECTION_PIN))
	{
		current_rising_edge_time = rtc_tick;
	    if(waiting_for_current)
	    {
	        current_rising_edge_time = rtc_tick;
	        relative_phase = voltage_rising_edge_time-current_rising_edge_time;
	        waiting_for_current = 0;
	    }

	    if(debug_mode)
	    {
	    	iedge_count++;
	    }
		gpio_clear_pin_interrupt_flag(CURRENT_EDGE_DETECTION_PIN);
	}
}

//calculates udp checksum
unsigned short in_cksum(unsigned short *addr, int len)
{
    int nleft, sum;
    unsigned short *w;
    union
    {
        unsigned short us;
        unsigned char  uc[2];
    } last;
    unsigned short answer;

    nleft = len;
    sum = 0;
    w = addr;

    /*
     * Algorithm is simple, using a 32 bit accumulator (sum) :
     * add sequential 16 bit words to it, and at the end, fold back all the
     * carry bits from the top 16 bits into the lower 16 bits.
     */
    while (nleft > 1)
    {

        sum += *w++;
        nleft -= 2;
    }

    // mop up an odd byte, if necessary
    if (nleft == 1)
    {
        last.uc[0] = *(unsigned char *)w;
        last.uc[1] = 0;
        sum += last.us;
    }

    // add back carry outs from top 16 bits to low 16 bits
    sum = (sum >> 16) + (sum & 0xffff);     // add hi 16 to low 16
    sum += (sum >> 16);                     // add carry
    answer = ~sum;                          // truncate to 16 bits
    return(answer);
}

void output_measurement(void)
{
    adc_start(&AVR32_ADC);
    //Vin = I*150/1613*1.04   * 3.26
    peak_iout = 33.53*((double) adc_get_value(adc,CURRENT_OUTPUT_CHANNEL))/1024;
    //Vin = V*14/115*.76      * 3.26
    peak_vout = 35.16*((double) adc_get_value(adc,VOLTAGE_OUTPUT_CHANNEL))/1024;
    output_voltage_amp = .707*peak_vout;
    output_current_amp = .707*peak_iout;

}

//display_update
/*brief: writes information to the LCD according to the argument passed
to save time, after a new screen has been written, subsequent
updates in the same display mode will only write variable fields */
void display_update(int mode)
{
    switch(mode)
    {
        //update measured variables
    case 1:
    	// don't waste time rewriting stuff that's already on the screen
    	if(last_case == 1)
    	{
    		dip204_set_cursor_position(7,1);
    		dip204_printf_string("%4.2f", pwm_amp_scale_factor);
    		dip204_set_cursor_position(7,2);
    		dip204_printf_string("%3d",period_samples);
    		dip204_set_cursor_position(18,2);
    		dip204_printf_string("%3d",phase_samples);
    		dip204_set_cursor_position(4,3);
    		dip204_printf_string("%4.2f",output_voltage_amp);
    		dip204_set_cursor_position(14,3);
    		dip204_printf_string("%4.2f",output_current_amp);
    		dip204_set_cursor_position(4,4);
    		dip204_printf_string("%4.1f",pow_real);
    		dip204_set_cursor_position(14,4);
    		dip204_printf_string("%4.1f",pow_reactive);
    	}

    	else
    	{
    		dip204_set_cursor_position(1,1);
    		dip204_printf_string("DUTY: %4.2f          ", pwm_amp_scale_factor);
    		dip204_set_cursor_position(1,2);
    		dip204_printf_string("FREQ: %3d ", period_samples);
    		dip204_set_cursor_position(11,2);
    		dip204_printf_string("PHASE: %3d", phase_samples);
    		dip204_set_cursor_position(1,3);
    		dip204_printf_string("V: %4.2f V ",output_voltage_amp);
    		dip204_set_cursor_position(11,3);
    		dip204_printf_string("I: %4.2f A ",output_current_amp);
    		dip204_set_cursor_position(1,4);
    		dip204_printf_string("P: %4.1f W ",pow_real);
    		dip204_set_cursor_position(11,4);
    		dip204_printf_string("Q: %4.1fvar",pow_reactive);
    		last_case = 1;
    	}
        break;
    case 2:
    	if(last_case == 2)
    	{
    		dip204_set_cursor_position(5,3);
    		dip204_printf_string("%5d",recd_packet_count);
    		dip204_set_cursor_position(16,3);
    		dip204_printf_string("%5d",sent_packet_count);
    		dip204_set_cursor_position(1,4);
    		dip204_printf_string("stat: %s",packet_summary);
    	}
    	else
    	{
    		dip204_set_cursor_position(1,1);
    		dip204_printf_string("%d.%d.%d.%d         ",local_ipaddr[0],local_ipaddr[1],local_ipaddr[2],local_ipaddr[3]);
    		dip204_set_cursor_position(1,2);
    		dip204_printf_string("%02x-%02x-%02x-%02x-%02x-%02x   ",local_ethaddr[0],local_ethaddr[1],local_ethaddr[2],local_ethaddr[03],local_ethaddr[4],local_ethaddr[5]);
    		dip204_set_cursor_position(1,3);
    		dip204_printf_string("rec:%5d sent:%5d",recd_packet_count,sent_packet_count);
    		dip204_set_cursor_position(1,4);
    		dip204_printf_string("stat: %s",packet_summary);
    		last_case = 2;
    	}

        break;
    case 3:
    	if(last_case == 3)
    	{
    		dip204_set_cursor_position(1,1);
    		dip204_printf_string("%5d %5d %5d",rising_edge_time,voltage_rising_edge_time,current_rising_edge_time);
    		dip204_set_cursor_position(1,2);
    		if(connected == 1)
    		{
    			dip204_write_string("   ");
    		}
    		else if(connected == 0)
    		{
    			dip204_write_string("DIS");
    		}
    		dip204_set_cursor_position(10,3);
    		dip204_printf_string("%5d",(int)rtc_tick/rtc_int_freq);
    		dip204_set_cursor_position(1,4);
    		if(op_mode == GRID_FOLLOWING)
    		{
    			dip204_write_string("FOLLOW");
    		}
    		else if(op_mode == STANDALONE)
    		{
    			dip204_write_string("SET   ");
    		}
    	}
    	else
    	{
    		dip204_set_cursor_position(1,1);
    		dip204_printf_string("%5d %5d %5d  ",rising_edge_time,voltage_rising_edge_time,current_rising_edge_time);
    		dip204_set_cursor_position(1,2);
    		if(connected == 1)
    		{
    			dip204_write_string("   CONNECTED        ");
    		}
    		else if(connected == 0)
    		{
    			dip204_write_string("DISCONNECTED        ");
    		}
    		dip204_set_cursor_position(1,3);
    		dip204_printf_string("up time: %5d      ",(int)rtc_tick/rtc_int_freq);
    		dip204_set_cursor_position(1,4);
    		if(op_mode == GRID_FOLLOWING)
    		{
    			dip204_write_string("FOLLOW              ");
    		}
    		else if(op_mode == STANDALONE)
    		{
    			dip204_write_string("SET                 ");
    		}
    		last_case = 3;
    	}
    	break;
    default:
        dip204_set_cursor_position(1,4);
        dip204_write_string("GTI 11/14/2015");
        break;

    }

}

//resamples the sine wave for different frequencies
int resample(int n_samples)
{

    for(i=0; i<n_samples; i++)
    {

        sine_data[i]=(int) ceil(PWM_PERIOD*(1-pwm_amp_scale_factor*sin(M_PI*i/(n_samples))));
        comp_sine_data[i] = (int) floor(PWM_PERIOD*(pwm_amp_scale_factor*sin(M_PI*i/(n_samples))));

    }

    return(n_samples);
}

void disconnect_output()
{
	//disconnect inverter
	gpio_clr_gpio_pin(CONNECTION_PIN);
	connected = 0;
}

void connect_output()
{
	//connect inverter
	gpio_set_gpio_pin(CONNECTION_PIN);
	connected = 1;
}

void phase_amp_calc()
{
    int lead_lag = 0;


    double pow_reac_adj = 0;
    double pow_real_adj = 0;

    if(relative_phase>=0)
    {
        lead_lag = 1;
    }
    else if(relative_phase<0)
    {
        lead_lag = -1;
    }

    pow_angle = 2*M_PI*relative_phase/period_samples;
    pf = cos(pow_angle);
    pf = pf * lead_lag;

    pow_apparent = .5*peak_vout*peak_iout;
    pow_real = pow_apparent*fabs(pf);
    pow_reactive = pow_apparent*sin(pow_angle);

    /*
    pow_reac_adj = pow_reac_target-pow_reactive;
    pow_real_adj = pow_real_target-pow_real;

    target_phase = target_phase*(1+.05*pow_reac_adj);
    target_phase_samples=(int) (period_samples*target_phase/360);
    pwm_amp_scale_factor = pwm_amp_scale_factor*(1+.05*pow_real_adj);
	*/
}

//handles data from macb receive buffer
void packet_processing(macb_packet_t *pkt)
{
    unsigned char ipaddr[4];
    short response_list[4] = {0xff, 0xff, 0xff, 0xff};

    //packet is icmp ping request
    if (pkt->data[23] == 0x01 )
    {
    	//update network stats
    	recd_packet_count++;

        //check IP address
        if (memcmp(&pkt->data[30], local_ipaddr, 4) == 0)
        {
            //is it an echo request?
            if(pkt->data[34] == 0x08 )
            {
                icmp_echo_response(pkt);
				snprintf(packet_summary,14,"echoreq @%5d",(int)rtc_tick/rtc_int_freq);
            }
            else
            {
                //not an implemented ICMP function
            	snprintf(packet_summary,14,"unkicmp@%6d",(int)rtc_tick/rtc_int_freq);
            }
        }

    }
    //packet is a udp packet on the proper port (33000)
    else if(pkt->data[23] == 0x11  && (pkt->data[36] == 0x80 && pkt->data[37] == 0xe8 ))
    {
        double temp;
        unsigned short message_components;
        unsigned short n = 0;
        unsigned short rec_checksum = 0;
        unsigned short response_list[4] = {0xffff, 0xffff, 0xffff, 0xffff};
        unsigned short rl_index = 0;
        unsigned short ACK_added = 0;
        unsigned short udp_checksum;
        unsigned short GPID =0;
        unsigned short udp_length = pkt->data[38] << 8 | pkt->data[39];
        unsigned short pseudo_length = udp_length + 12;
        unsigned char pseudo_header[pseudo_length];
        unsigned short switch_var = 0;
        //double temp = 0;

    	//update network stats
    	recd_packet_count++;

        GPID = pkt->data[GRID_PROTOCOL_ID_FIELD] << 8 | pkt->data[GRID_PROTOCOL_ID_FIELD + 1];

        //our grid protocol
        if(GPID == GRID_PROTOCOL_ID)
        {
            //is this message meant for an inverter?
            if(pkt->data[GRID_PROTOCOL_SCOPE_FIELD] == GRID_PROTOCOL_SCOPE_INVERTER)
            {
				rec_checksum = pkt->data[40] << 8 | pkt->data[41];

            	//zero checksum
            	pkt->data[40] = 0x00;
            	pkt->data[41] = 0x00;

                //construct pseudo_header and compute UDP checksum
                //copy source and destination addresses
                memcpy(pseudo_header, pkt->data+26, 8);
                //zero byte
                pseudo_header[8] = 0x00;
                //protocol field
                pseudo_header[9] = 0x11;
                //UDP length
                pseudo_header[10] = MSB(udp_length);
                pseudo_header[11] = LSB(udp_length);
                //source and destination ports
                memcpy(pseudo_header+12, pkt->data+34, 4);
                //UDP length again
                pseudo_header[16] = MSB(udp_length);
                pseudo_header[17] = LSB(udp_length);
                //zero checksum bytes
                pseudo_header[18] = 0x00;
                pseudo_header[19] = 0x00;
                //UDP payload
                memcpy(pseudo_header + 20, pkt->data + GRID_PROTOCOL_ID_FIELD, udp_length-8);

                return_udp =  pkt->data[34] << 8 | pkt->data[35];

                //calculate UDP checksum
                udp_checksum = in_cksum((unsigned short *) pseudo_header, pseudo_length);

                //good checksum if checksum validation is enabled
                if(udp_checksum == rec_checksum || disable_checksum_validation == 1)
                {
                    message_components = pkt->data[GRID_PROTOCOL_MSG_COMPS_FIELD];

                    //as long as there are more components to the message, keep adding to the list of response components
                    while(message_components > 0)
                    {
                    	switch_var = pkt->data[GRID_PROTOCOL_TYPE_FIELD + n*10] << 8 | pkt->data[GRID_PROTOCOL_TYPE_FIELD + n*10 + 1];

                        if(switch_var == GRID_PROTOCOL_PHASE_ADJUST)
                        {
                            memcpy(&temp,pkt->data+GRID_PROTOCOL_VALUE_FIELD + n*10, 8);
                            target_phase = temp;

                            snprintf(packet_summary,14,"adjphase@%5d",(int) rtc_tick/rtc_int_freq);

							if(!ACK_added)
							{
								response_list[rl_index] = ACK;
								rl_index++;
								ACK_added = 1;
							}
                        }
                        else if(switch_var == GRID_PROTOCOL_AMP_ADJUST)
                        {
                            memcpy(&temp,pkt->data+GRID_PROTOCOL_VALUE_FIELD + n*10, 8);
                            pwm_amp_scale_factor= temp;
                            resample(half_period_samples);

                            snprintf(packet_summary,14,"adjamp  @%5d",(int)rtc_tick/rtc_int_freq);
							if(!ACK_added)
							{
								response_list[rl_index] = ACK;
								rl_index++;
								ACK_added = 1;
							}
                        }
                        else if(switch_var == GRID_PROTOCOL_REAL_POWER_ADJUST)
                        {
                        	snprintf(packet_summary,14,"adjreal @%5d",(int)rtc_tick/rtc_int_freq);
                        	if(!ACK_added)
                        	{
                        		response_list[rl_index] = ACK;
                        		rl_index++;
                        		ACK_added = 1;
                        	}
                        }
                        else if(switch_var == GRID_PROTOCOL_REACTIVE_POWER_ADJUST)
                        {
                        	snprintf(packet_summary,14,"adjreac @%5d",(int)rtc_tick/rtc_int_freq);
                        	if(!ACK_added)
                        	{
                        	    response_list[rl_index] = ACK;
                        	    rl_index++;
                        	    ACK_added = 1;
                        	}
                        }
                        else if(switch_var == GRID_PROTOCOL_FREQUENCY_ADJUST)
                        {
                        	snprintf(packet_summary,14,"adjfreq @%5d",(int)rtc_tick/rtc_int_freq);
                        	if(!ACK_added)
                        	{
                        	    response_list[rl_index] = ACK;
                        	    rl_index++;
                        	    ACK_added = 1;
                        	}
                        }
                        else if(switch_var == GRID_PROTOCOL_MODE_STANDALONE)
                        {
                            op_mode = STANDALONE;

                            snprintf(packet_summary,14,"refset  @%5d",(int)rtc_tick/rtc_int_freq);
							if(!ACK_added)
							{
								response_list[rl_index] = ACK;
								rl_index++;
								ACK_added = 1;
							}
                        }
                        else if(switch_var == GRID_PROTOCOL_MODE_GRID_FOLLOWING)
                        {
                            op_mode = GRID_FOLLOWING;

                            snprintf(packet_summary,14,"grid @%5d",(int)rtc_tick/rtc_int_freq);

							if(!ACK_added)
							{
								response_list[rl_index] = ACK;
								rl_index++;
								ACK_added = 1;
							}
                        }
                        else if(switch_var == GRID_PROTOCOL_DISCONNECT)
                        {
                        	disconnect_output();

                            snprintf(packet_summary,14,"disconn@%5d",(int)rtc_tick/rtc_int_freq);

							if(!ACK_added)
							{
								response_list[rl_index] = ACK;
								rl_index++;
								ACK_added = 1;
							}
                        }
                        else if(switch_var == GRID_PROTOCOL_CONNECT)
                        {
							connect_output();

                            snprintf(packet_summary,14,"connect@%5d",(int)rtc_tick/rtc_int_freq);

							if(!ACK_added)
							{
								response_list[rl_index] = ACK;
								rl_index++;
								ACK_added = 1;
							}
                        }
                        else if(switch_var == GRID_PROTOCOL_LOCK_SCREEN)
                        {
                        	display_locked = 1;
                        	snprintf(packet_summary,14,"lockdisp@%5d",(int)rtc_tick/rtc_int_freq);
							if(!ACK_added)
							{
								response_list[rl_index] = ACK;
								rl_index++;
								ACK_added = 1;
							}
                        }
                        else if(switch_var == GRID_PROTOCOL_UNLOCK_SCREEN)
                        {
                        	display_locked = 0;
                        	snprintf(packet_summary,14,"unlkdisp@%5d",(int)rtc_tick/rtc_int_freq);
                        	if(!ACK_added)
                        	{
                        		response_list[rl_index] = ACK;
                        		rl_index++;
                        		ACK_added = 1;
                        	}
                        }
                        else if(switch_var == GRID_PROTOCOL_SCREEN_1)
                        {
                        	display_selection = 1;
                        	snprintf(packet_summary,14,"screen1 @%5d",(int)rtc_tick/rtc_int_freq);
                        	if(!ACK_added)
                        	{
                        		response_list[rl_index] = ACK;
                        		rl_index++;
                        		ACK_added = 1;
                        	}
                        }
                        else if(switch_var == GRID_PROTOCOL_SCREEN_2)
                        {
                        	display_selection = 2;
                        	snprintf(packet_summary,14,"screen2 @%5d",(int)rtc_tick/rtc_int_freq);
                        	if(!ACK_added)
                        	{
                        		response_list[rl_index] = ACK;
                        		rl_index++;
                        		ACK_added = 1;
                        	}
                        }
                        else if(switch_var == GRID_PROTOCOL_SCREEN_3)
                        {
                        	display_selection = 3;
                        	snprintf(packet_summary,14,"screen3 @%5d",(int)rtc_tick/rtc_int_freq);
                        	if(!ACK_added)
                        	{
                        		response_list[rl_index] = ACK;
                        		rl_index++;
                        		ACK_added = 1;
                        	}
                        }
                        else if(switch_var == GRID_PROTOCOL_REQUEST_PHASE)
                        {
                        	snprintf(packet_summary,14,"getphase@%5d",(int)rtc_tick/rtc_int_freq);
                            response_list[rl_index] = RESPONSE_PHASE;
                            rl_index++;
                        }
                        else if(switch_var == GRID_PROTOCOL_REQUEST_AMP)
                        {
                        	snprintf(packet_summary,14,"getamp  @%5d",(int)rtc_tick/rtc_int_freq);
                            response_list[rl_index] = RESPONSE_AMP;
                            rl_index++;
                        }
                        else if(switch_var ==  GRID_PROTOCOL_REQUEST_MODE)
                        {
                        	snprintf(packet_summary,14,"getmode @%5d",(int)rtc_tick/rtc_int_freq);
                            response_list[rl_index] = RESPONSE_MODE;
                            rl_index++;
						}
						else if(switch_var == GRID_PROTOCOL_REQUEST_PWM_PERIOD)
						{
                        	snprintf(packet_summary,14,"getpwm  @%5d",(int)rtc_tick/rtc_int_freq);
                            response_list[rl_index] = RESPONSE_PWM_PERIOD;
                            rl_index++;
						}
						else if(switch_var ==GRID_PROTOCOL_REQUEST_NSAMPLES)
						{
                        	snprintf(packet_summary,14,"getsampl@%5d",(int)rtc_tick/rtc_int_freq);
                        	response_list[rl_index] = RESPONSE_NSAMPLES;
                            rl_index++;
						}
						else if (switch_var == GRID_PROTOCOL_REQUEST_OUTPUT_VOLTAGE)
						{
                        	snprintf(packet_summary,14,"getsampl@%5d",(int)rtc_tick/rtc_int_freq);
                        	response_list[rl_index] = RESPONSE_OUTPUT_VOLTAGE;
                        	rl_index++;
						}
						else if(switch_var == GRID_PROTOCOL_REQUEST_OUTPUT_CURRENT)
						{
                        	snprintf(packet_summary,14,"getsampl@%5d",(int)rtc_tick/rtc_int_freq);
                        	response_list[rl_index] = RESPONSE_OUTPUT_CURRENT;
							rl_index++;
						}
						else if(switch_var == GRID_PROTOCOL_REQUEST_REAL_POWER)
						{
                        	snprintf(packet_summary,14,"getsampl@%5d",(int)rtc_tick/rtc_int_freq);
                        	response_list[rl_index] = RESPONSE_REAL_POWER;
							rl_index++;
						}
						else if(switch_var == GRID_PROTOCOL_REQUEST_REACTIVE_POWER)
						{
                        	snprintf(packet_summary,14,"getsampl@%5d",(int)rtc_tick/rtc_int_freq);
							response_list[rl_index] = RESPONSE_REACTIVE_POWER;
							rl_index++;
						}
						else
						{
                        	snprintf(packet_summary,14,"unimp@%5d",(int)rtc_tick/rtc_int_freq);
                            response_list[rl_index] = UNIMPLEMENTED_FUNCTION;
							rl_index++;
                        }

                        message_components--;
                        n++;
                    }
                    //the number of components that the response will have
					pkt->data[GRID_PROTOCOL_MSG_COMPS_FIELD]= LSB(rl_index);

                    send_udp_packet(pkt, response_list);
                }
                else
                {
                    //bad checksum
                	snprintf(packet_summary,14,"badcksum@%5d",(int)rtc_tick/rtc_int_freq);
					response_list[0] = BAD_UDP_CHECKSUM;
                    send_udp_packet(pkt, response_list);
                }
            }
            else
            {
                //improper scope, packet may have been sent to inverter by mistake
            	//maybe IP address device mappings won't be properly maintained
            	response_list[0] = MISDIRECTED_MESSAGE;
                send_udp_packet(pkt, response_list);

                snprintf(packet_summary,14,"notinv  @%5d",(int)rtc_tick/rtc_int_freq);
            }
        }
        //if this isn't a grid protocol packet
        else
        {
        	if(COMM_DEBUG)
        	{
        		dip204_write_string("-!ID");
        	}
        }
    }
    // if it is a ARP frame, answer it
    else if ((pkt->data[12] == 0x08) && (pkt->data[13] == 0x06))
    {
    	//update network stats
    	recd_packet_count++;

        // ARP request
        if ((pkt->data[20] == 0x00) && (pkt->data[21] == 0x01))
        {

          // Reply only if the ARP request is destined to our IP address.
          if(memcmp(pkt->data+38, local_ipaddr, 4))
          {
             return;
          }

          // swap sender & receiver address
          memmove(pkt->data, pkt->data+6, 6);
          memcpy(pkt->data+6, hwaddr, 6);
          // ARP Response
          pkt->data[21] = 0x02;
          // swap sender & receiver parameters
          memcpy(ipaddr, pkt->data+38, 4);
          memmove(pkt->data+32, pkt->data+22, 10);
          memcpy(pkt->data+22, hwaddr, 6);
          memcpy(pkt->data+28, ipaddr, 4);
          // send response, nothing else to send
          lMACBSend(&AVR32_MACB, pkt->data, pkt->len, TRUE);
          //update networking stats
          sent_packet_count++;
        }

      }
    //unimplemented protocols - there was too much irrelevant traffic and too much time was wasted on responses this is commented now
    else
    {
    	//snprintf(packet_summary,14,"badproto@%5d",(int)rtc_tick/rtc_int_freq);
    }
}

void icmp_echo_response(macb_packet_t *pkt)
{
    unsigned char ipaddr[4];
    unsigned short chksum;

    // swap sender & receiver HW address
    memmove(pkt->data, pkt->data+6, 6);
    memcpy(pkt->data+6, hwaddr, 6);
    // swap sender & receiver IP address
    memcpy(ipaddr, pkt->data+26, 4);
    memmove(pkt->data+26, pkt->data+30, 4);
    memcpy(pkt->data+30, ipaddr, 4);
    // set seq num
    pkt->data[18] = MSB(seqnum);
    pkt->data[19] = LSB(seqnum);
    if (++seqnum >= 0xFF00) seqnum = SEQ_NUM_START;
    // reset checksum before computation
    pkt->data[24] = 0;
    pkt->data[25] = 0;
    // compute IP checksum
    chksum = in_cksum((unsigned short *)&pkt->data[14], 20);
    // set IP checksum
    pkt->data[24] = MSB(chksum);
    pkt->data[25] = LSB(chksum);
    // set reply bit
    pkt->data[34] = 0;
    // reset checksum before computation
    pkt->data[36] = 0;
    pkt->data[37] = 0;
    // compute ICMP checksum
    chksum = in_cksum((unsigned short *)&pkt->data[34], (pkt->len - 34));
    // set ICMP checksum
    pkt->data[36] = MSB(chksum);
    pkt->data[37] = LSB(chksum);
    // send request, nothing else to send
    lMACBSend(&AVR32_MACB, pkt->data, pkt->len, TRUE);
    //update network stats
    sent_packet_count++;

}

//sends a response to a valid packet with components given by response list
void send_udp_packet(macb_packet_t *pkt, short *response_list)
{
	double temp;
    unsigned char ipaddr[4];
    unsigned short chksum;
    unsigned short udp_checksum;
    unsigned short udp_length;
    unsigned short rl_index = 0;


    // swap sender & receiver HW address
    memmove(pkt->data, pkt->data+6, 6);
    memcpy(pkt->data+6, hwaddr, 6);
    // swap sender & receiver IP address
    memcpy(ipaddr, pkt->data+26, 4);
    memmove(pkt->data+26, pkt->data+30, 4);
    memcpy(pkt->data+30, ipaddr, 4);
    // reset checksum before computation
    pkt->data[24] = 0;
    pkt->data[25] = 0;
    // compute IP checksum
    chksum = in_cksum((unsigned short *)&pkt->data[14], 20);
    // set IP checksum
    pkt->data[24] = MSB(chksum);
    pkt->data[25] = LSB(chksum);
    // source port number
    pkt->data[34] = 0x80;
    pkt->data[35] = 0xe8;
    //receiver port number
    pkt->data[36] = MSB(return_udp);
    pkt->data[37] = LSB(return_udp);

    //change scope to indicate response to control
    pkt->data[GRID_PROTOCOL_SCOPE_FIELD] = 0x00;

while((response_list[rl_index] != 0xffff) && rl_index < 5)
{
    switch(response_list[rl_index])
    {
    case UNIMPLEMENTED_PROTOCOL:
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 ] = MSB(GRID_PROTOCOL_UNIMPLEMENTED_PROTOCOL);
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB(GRID_PROTOCOL_UNIMPLEMENTED_PROTOCOL);
        break;

    case ACK:
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB( GRID_PROTOCOL_ACK);
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB(GRID_PROTOCOL_ACK);
        break;

    case BAD_UDP_CHECKSUM:
        pkt->data[GRID_PROTOCOL_TYPE_FIELD] = MSB(GRID_PROTOCOL_BAD_UDP_CHECKSUM);
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + 1] = LSB( GRID_PROTOCOL_BAD_UDP_CHECKSUM);
        break;

    case RESPONSE_AMP:
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_RESPONSE_AMP);
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB( GRID_PROTOCOL_RESPONSE_AMP);
        memcpy(pkt->data + GRID_PROTOCOL_DATA_FIELD + rl_index*10, &pwm_amp_scale_factor, 8);
        break;

    case RESPONSE_PHASE:
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_RESPONSE_PHASE);
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB( GRID_PROTOCOL_RESPONSE_PHASE);
        memcpy(pkt->data + GRID_PROTOCOL_DATA_FIELD + rl_index*10, &target_phase, 8);
    	break;

    case RESPONSE_MODE:
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_RESPONSE_MODE);
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB( GRID_PROTOCOL_RESPONSE_MODE);
        temp = (double) op_mode;
        memcpy(pkt->data + GRID_PROTOCOL_DATA_FIELD + rl_index*10, &temp, 8);
    	break;

    case RESPONSE_PWM_PERIOD:
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_RESPONSE_PWM_PERIOD);
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB( GRID_PROTOCOL_RESPONSE_PWM_PERIOD);
        temp = (double) PWM_PERIOD;
        memcpy(pkt->data + GRID_PROTOCOL_DATA_FIELD + rl_index*10, &temp, 8);
    	break;

    case RESPONSE_NSAMPLES:
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_RESPONSE_NSAMPLES);
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB(GRID_PROTOCOL_RESPONSE_NSAMPLES);
    	temp = (double) period_samples;
        memcpy(pkt->data + GRID_PROTOCOL_DATA_FIELD + rl_index*10, &temp, 8);
        break;
    case RESPONSE_OUTPUT_VOLTAGE:
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_RESPONSE_OUTPUT_VOLTAGE);
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB(GRID_PROTOCOL_RESPONSE_OUTPUT_VOLTAGE);
    	temp = output_voltage_amp;
    	memcpy(pkt->data + GRID_PROTOCOL_DATA_FIELD + rl_index*10, &temp, 8);
    	break;
    case RESPONSE_OUTPUT_CURRENT:
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_RESPONSE_OUTPUT_CURRENT);
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB(GRID_PROTOCOL_RESPONSE_OUTPUT_CURRENT);
    	temp = output_current_amp;
    	memcpy(pkt->data + GRID_PROTOCOL_DATA_FIELD + rl_index*10, &temp, 8);
    	break;
    case RESPONSE_REAL_POWER:
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_RESPONSE_REAL_POWER);
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB(GRID_PROTOCOL_RESPONSE_REAL_POWER);
    	temp = pow_real;
    	memcpy(pkt->data + GRID_PROTOCOL_DATA_FIELD + rl_index*10, &temp, 8);
    	break;
    case RESPONSE_REACTIVE_POWER:
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_RESPONSE_REACTIVE_POWER);
    	pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB(GRID_PROTOCOL_RESPONSE_REACTIVE_POWER);
    	temp = pow_reactive;
    	memcpy(pkt->data + GRID_PROTOCOL_DATA_FIELD + rl_index*10, &temp, 8);
    	break;
    case MISDIRECTED_MESSAGE:
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_MISDIRECTED_MESSAGE);
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB(GRID_PROTOCOL_MISDIRECTED_MESSAGE);
    default:
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10] = MSB(GRID_PROTOCOL_UNIMPLEMENTED_FUNCTION);
        pkt->data[GRID_PROTOCOL_TYPE_FIELD + rl_index*10 + 1] = LSB(GRID_PROTOCOL_UNIMPLEMENTED_FUNCTION);
        break;
    }
    rl_index++;
}
unsigned short gp_length = 6 + 10*(0xffff & pkt->data[GRID_PROTOCOL_MSG_COMPS_FIELD]);
unsigned short pseudo_length = 20 + gp_length;
unsigned char pseudo_header[pseudo_length];

    udp_length = 8 + gp_length;

    // UDP length
    pkt->data[38] = MSB(udp_length);
    pkt->data[39] = LSB(udp_length);

    //copy source and destination addresses
    memcpy(pseudo_header, pkt->data+26, 8);
    //zero byte
    pseudo_header[8] = 0x00;
    //protocol field
    pseudo_header[9] = 0x11;
    //UDP length
    pseudo_header[10] = MSB(udp_length);
    pseudo_header[11] = LSB(udp_length);

    //source and destination ports
    memcpy(pseudo_header+12, pkt->data+34, 4);
    //UDP length again
    pseudo_header[16] = MSB(udp_length);
    pseudo_header[17] = LSB(udp_length);
    //zero checksum bytes
    pseudo_header[18] = 0x00;
    pseudo_header[19] = 0x00;
    //UDP payload
    memcpy(pseudo_header + 20, pkt->data + GRID_PROTOCOL_ID_FIELD, gp_length);

    //calculate UDP checksum
    udp_checksum = in_cksum((unsigned short *) pseudo_header, pseudo_length);

    if(COMM_DEBUG)
    {
    	dip204_set_cursor_position(1,4);
    	dip204_printf_string("%c", pseudo_header[2]);
    }

    //udp checksum
    pkt->data[40] = MSB(udp_checksum);
    pkt->data[41] = LSB(udp_checksum);


    pkt->len = udp_length + 34;


    lMACBSend(&AVR32_MACB, pkt->data, pkt->len, TRUE);
    //update networking stats
    sent_packet_count++;

}

//MAIN
int main(void)
{
    //clock settings
    pm_switch_to_osc0(pm,FOSC0,OSC0_STARTUP);         //   12 MHZ

    pm_pll_setup(pm,0,9,1,0,20);

    pm_pll_set_option(pm,0,1,1,0);

    pm_pll_enable(pm, 0);

    pm_wait_for_pll0_locked(pm);

    pm_cksel(pm,1,0,0,0,0,0);

    flashc_set_wait_state(1);

    pm_switch_to_clock(pm, AVR32_PM_MCSEL_PLL0);

    //GPIO initialization
    gpio_enable_pin_glitch_filter(EDGE_DETECTION_PIN);
    gpio_enable_pin_glitch_filter(VOLTAGE_EDGE_DETECTION_PIN);
    gpio_enable_pin_glitch_filter(CURRENT_EDGE_DETECTION_PIN);
    //gpio_enable_module_pin(AVR32_PIN_PA20, 2);
    //gpio_enable_module_pin(AVR32_PIN_PB23,0);
    gpio_enable_gpio_pin(LOW_SIDE_PIN);
    gpio_enable_gpio_pin(EDGE_DETECTION_PIN);
    gpio_enable_gpio_pin(VOLTAGE_EDGE_DETECTION_PIN);
    gpio_enable_gpio_pin(CURRENT_EDGE_DETECTION_PIN);

    //power manager settings
    volatile avr32_pm_t* pm = &AVR32_PM;

    //PWM initialization
    //******************************************************
    pwm_opt_t HIGH_FREQ_opt;

    avr32_pwm_channel_t HIGH_FREQ_channel = { .ccnt = 0 };

    unsigned int HIGH_FREQ_id;


    HIGH_FREQ_id=HIGH_FREQ_CHANNEL_ID;



    gpio_enable_module_pin(HIGH_FREQ_PIN,HIGH_FREQ_FUNCTION);

    // PWM controller configuration.
    HIGH_FREQ_opt.diva = AVR32_PWM_DIVA_CLK_OFF;
    HIGH_FREQ_opt.divb = AVR32_PWM_DIVB_CLK_OFF;
    HIGH_FREQ_opt.prea = AVR32_PWM_PREA_MCK;
    HIGH_FREQ_opt.preb = AVR32_PWM_PREB_MCK;



    HIGH_FREQ_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED;       // Channel mode.
    HIGH_FREQ_channel.CMR.cpol = PWM_POLARITY_LOW;            // Channel polarity.
    HIGH_FREQ_channel.CMR.cpd = PWM_UPDATE_DUTY;
    HIGH_FREQ_channel.CMR.cpre = 0x00000000;
    HIGH_FREQ_channel.cdty = 200;   // Channel duty cycle, should be < CPRD.
    HIGH_FREQ_channel.cprd = PWM_PERIOD;
    //HIGH_FREQ_channel.cupd = 200;


    pwm_init(&HIGH_FREQ_opt);

    pwm_channel_init(HIGH_FREQ_id, &HIGH_FREQ_channel);

    pwm_start_channels((1 << HIGH_FREQ_id));



    //adc initialization
    static const gpio_map_t ADC_GPIO_MAP =
    {
        { CURRENT_OUTPUT_PIN, CURRENT_OUTPUT_FUNCTION },
        { VOLTAGE_OUTPUT_PIN, VOLTAGE_OUTPUT_FUNCTION }
    };

    volatile avr32_adc_t *adc = &AVR32_ADC;

    gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));

    AVR32_ADC.MR.prescal = 20;
    adc_configure(adc);
    adc_enable(adc, VOLTAGE_OUTPUT_CHANNEL);
    adc_enable(adc, CURRENT_OUTPUT_CHANNEL);



    //SPI and DIP204 initialization
    static const gpio_map_t DIP204_SPI_GPIO_MAP =
    {
        { DIP204_SPI_SCK_PIN, DIP204_SPI_SCK_FUNCTION }, // SPI Clock.
        { DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION }, // MISO.
        { DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION }, // MOSI.
        { DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION } // Chip Select NPCS.
    };
    spi_options_t spiOptions = // SPI options for the LCD DIP204
    {
        .reg = DIP204_SPI_NPCS,
        .baudrate = 4000000,
        .bits = 8,
        .spck_delay = 0,
        .trans_delay = 0,
        .stay_act = 1,
        .spi_mode = 0,
        .modfdis = 1
    }; // mode fault

    gpio_enable_module(DIP204_SPI_GPIO_MAP, sizeof(DIP204_SPI_GPIO_MAP) /
                       sizeof(DIP204_SPI_GPIO_MAP[0])); // Assign I/O's to SPI
    spi_initMaster(DIP204_SPI, &spiOptions); // Initialize as master
    spi_selectionMode(DIP204_SPI, 0, 0, 0); // Set selection mode
    spi_enable(DIP204_SPI); // Enable SPI
    spi_setupChipReg(DIP204_SPI, &spiOptions, FOSCO); // setup chip registers
    delay_init(FOSCO); // initialize delay driver
    dip204_init(backlight_PWM, TRUE); // initialize LCD

    //interrupt controller initialization

    Disable_global_interrupt(); // Disable all interrupts
    INTC_init_interrupts(); // Initialize interrupt vectors
    INTC_register_interrupt(&RTC_irq, AVR32_RTC_IRQ, AVR32_INTC_INT1); // Register RTC
    //INTC_register_interrupt(&pushbutton_interrupt_handler, AVR32_GPIO_IRQ_0+(GPIO_PUSH_BUTTON_0/8), AVR32_INTC_INT1);        //IRQ_0 base address with offset for pin 88, PX_16
    //INTC_register_interrupt(&pushbutton_interrupt_handler, AVR32_GPIO_IRQ_0+(GPIO_PUSH_BUTTON_1/8), AVR32_INTC_INT1);
    //INTC_register_interrupt(&pushbutton_interrupt_handler, AVR32_GPIO_IRQ_0+(GPIO_PUSH_BUTTON_2/8), AVR32_INTC_INT1);
    INTC_register_interrupt(&rising_edge_interrupt_handler, AVR32_GPIO_IRQ_0+(EDGE_DETECTION_PIN/8), AVR32_INTC_INT1);
    INTC_register_interrupt(&rising_edge_interrupt_handler, AVR32_GPIO_IRQ_0+(VOLTAGE_EDGE_DETECTION_PIN/8), AVR32_INTC_INT1);
    INTC_register_interrupt(&rising_edge_interrupt_handler, AVR32_GPIO_IRQ_0+(CURRENT_EDGE_DETECTION_PIN/8), AVR32_INTC_INT1);


    Enable_global_interrupt();

    //gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_0, GPIO_RISING_EDGE);
    //gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_1, GPIO_RISING_EDGE);
    //gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_2, GPIO_RISING_EDGE);
    gpio_enable_pin_interrupt(EDGE_DETECTION_PIN, GPIO_RISING_EDGE);
    gpio_enable_pin_interrupt(VOLTAGE_EDGE_DETECTION_PIN, GPIO_RISING_EDGE);
    gpio_enable_pin_interrupt(CURRENT_EDGE_DETECTION_PIN, GPIO_RISING_EDGE);


    //rtc initialization
    rtc_init(&AVR32_RTC, RTC_OSC_32KHZ, RTC_PRESCALAR);
    rtc_set_top_value(&AVR32_RTC, 0);
    rtc_enable_interrupt(&AVR32_RTC);
    rtc_enable(&AVR32_RTC);
    //main_int_freq=16000/((double) rtc_get_top_value(&AVR32_RTC));               //calculates rtc interrupt frequency

    //generate initial sine wave lookup table
    data_length=resample(half_period_samples);

    //display initialization
    dip204_set_cursor_position(1,1);
    dip204_write_string("Oi gov!");

    //macb initialization
    unsigned long len = 0;
    macb_packet_t recvd_pkt;
    static const gpio_map_t MACB_GPIO_MAP =
    {
        {AVR32_MACB_MDC_0_PIN,    AVR32_MACB_MDC_0_FUNCTION   },
        {AVR32_MACB_MDIO_0_PIN,   AVR32_MACB_MDIO_0_FUNCTION  },
        {AVR32_MACB_RXD_0_PIN,    AVR32_MACB_RXD_0_FUNCTION   },
        {AVR32_MACB_TXD_0_PIN,    AVR32_MACB_TXD_0_FUNCTION   },
        {AVR32_MACB_RXD_1_PIN,    AVR32_MACB_RXD_1_FUNCTION   },
        {AVR32_MACB_TXD_1_PIN,    AVR32_MACB_TXD_1_FUNCTION   },
        {AVR32_MACB_TX_EN_0_PIN,  AVR32_MACB_TX_EN_0_FUNCTION },
        {AVR32_MACB_RX_ER_0_PIN,  AVR32_MACB_RX_ER_0_FUNCTION },
        {AVR32_MACB_RX_DV_0_PIN,  AVR32_MACB_RX_DV_0_FUNCTION },
        {AVR32_MACB_TX_CLK_0_PIN, AVR32_MACB_TX_CLK_0_FUNCTION}
    };

    // Assign GPIO to MACB
    gpio_enable_module(MACB_GPIO_MAP, sizeof(MACB_GPIO_MAP) / sizeof(MACB_GPIO_MAP[0]));

    //DEBUG**DEBUG**DEBUG - make sure inverter is connected to load for testing purposes
    connect_output();


    // initialize MACB & Phy Layers
    if (xMACBInit(&AVR32_MACB) == FALSE )
    {
        //gpio_clr_gpio_pin(LED0_GPIO);
        //while(1);
    }
    gpio_clr_gpio_pin(LED1_GPIO);

    //set initial state of low freq output
    target_phase_samples=(int) (period_samples*target_phase/360);


    //MAIN LOOP
    while(1)
    {
        start=rtc_tick;

        if(fabs(period_samples-2*data_length)>PERIOD_TOLERANCE)
        {
            //deal with frequency mismatch
            //resample_req = 1;
        }

        //check for incoming packets
        if((rtc_tick-last_check_tick) > PACKET_CHECK_INTERVAL)
        {

        	last_check_tick = rtc_tick;
        	len = ulMACBInputLength();
        	if(len !=0)
        	{
        		//let the driver know we are going to read a new packet
                vMACBRead(NULL, 0, 0);
                //read enough bytes to fill this buffer
                vMACBRead( data, 128, len );
                recvd_pkt.data = data;
                recvd_pkt.len = len;
                packet_processing(&recvd_pkt);
            }
        }


        if(resample_flag)
        {
        	half_period_samples= (int) .5*period_samples;

            data_length=resample(half_period_samples);

            resample_flag=0;
        }


        //update display
        if((rtc_tick-last_update_tick)>DISPLAY_INTERVAL)
        {
        	last_update_tick=rtc_tick;
			if(display_locked)
			{
				display_update(display_selection);
			}
			else
			{

				switch(display_counter)
				{
				case 1:
					display_index = 1;
					break;
				case 10:
					display_index = 2;
					break;
				case 16:
					display_index = 3;
					break;
				case 20:
					display_counter = 0;
					break;
				default:
					break;
				}
				display_counter++;
		        display_update(display_index);
			}
        }

        //measure output voltage
        if(((rtc_tick-last_measurement_tick)>MEASUREMENT_INTERVAL))
        {
            output_measurement();
            last_measurement_tick = rtc_tick;
        }

        if(((rtc_tick-last_calc_tick)>CALC_INTERVAL))
        {
            phase_amp_calc();
            last_calc_tick = rtc_tick;
        }


        if(!(rtc_tick%8000))
        {
        //calculate desired phase offset in samples from degrees
        target_phase_samples=(int) (period_samples*target_phase/360);
        }


        timer=rtc_tick-start;
    }

}
