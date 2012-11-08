/*
 * Copyright (c) 2001, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Adam Dunkels.
 * 4. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: main.c,v 1.16 2006/06/11 21:55:03 adam Exp $
 *
 */

// *********************************************************************
// Define the IP address to be used for the MCU running the TCP/IP stack
// *********************************************************************
#define MYIP_1	192
#define MYIP_2	168
#define MYIP_3	2
#define MYIP_4	200


#include "LPC17xx.h"

#include "timer.h"
#include "uip.h"
#include "uip_arp.h"
#include "tapdev.h"
#include "type.h"
#include "uart.h"
#include "leds.h"
#include "clock-arch.h"
#include "logger.h"
#include "version.h"

#include <cr_section_macros.h>
#include <NXP/crp.h>
// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

#if defined ( _RDB1768_)
// Used to decide if "First connection message" is printed to LCD
int firstconnection = 0;
#endif


#define BUF ((struct uip_eth_hdr *)&uip_buf[0])


#define S0_INPUT0 (1 << 10)
#define S0_INPUT1 (1 << 11)
#define S0_INPUT2 (1 << 12)


//#define TAKT 370
#define TAKT 352 // entspricht ca 369us

/*--------------------------- uip_log ---------------------------------*/

void uip_log(char *m)
{
  //printf("uIP log message: %s\n", m);
}


/* wait */
uint32_t wait_ticks(uint32_t last_value, uint32_t ticks) {
	uint32_t ticks_now = clock_time();
	if (ticks_now >= last_value) {
		return (ticks_now - last_value) >= ticks;
	}
	else {
		// check for timer overflow
		return (UINT32_MAX - last_value + ticks_now) >= ticks;
	}
}

void delay_10ms(uint8_t ticks) {
	int currentms = clock_time() ;
	// wait 1s
	while(!(wait_ticks(currentms, ticks)));
}


void delayMs(uint8_t timer_num, uint32_t delayInMs)
{
  if ( timer_num == 0 )
  {
	LPC_TIM0->TCR = 0x02;		/* reset timer */
	//LPC_TIM0->PR  = 0x00;		/* set prescaler to zero */
	//LPC_TIM0->PR  = 100000;		/* set prescaler to zero */
	LPC_TIM0->PR  = 25000;		/* set prescaler to zero */
	LPC_TIM0->MR0 = delayInMs ;
	LPC_TIM0->IR  = 0xff;		/* reset all interrrupts */
	LPC_TIM0->MCR = 0x04;		/* stop timer on match */
	LPC_TIM0->TCR = 0x01;		/* start timer */

	/* wait until delay time has elapsed */
	while (LPC_TIM0->TCR & 0x01);
  }
  else if ( timer_num == 1 )
  {
	LPC_TIM1->TCR = 0x02;		/* reset timer */
	LPC_TIM1->PR  = 0x00;		/* set prescaler to zero */
	LPC_TIM1->MR0 = delayInMs * (9000000 / 1000-1);
	LPC_TIM1->IR  = 0xff;		/* reset all interrrupts */
	LPC_TIM1->MCR = 0x04;		/* stop timer on match */
	LPC_TIM1->TCR = 0x01;		/* start timer */

	/* wait until delay time has elapsed */
	while (LPC_TIM1->TCR & 0x01);
  }
  return;
}


void delayUs(uint8_t timer_num, uint32_t delayInUs)
{
  if ( timer_num == 0 )
  {
	LPC_TIM0->TCR = 0x02;		/* reset timer */
	//LPC_TIM0->PR  = 0x00;		/* set prescaler to zero */
	//LPC_TIM0->PR  = 100000;		/* set prescaler to zero */
	LPC_TIM0->PR  = 25;		/* set prescaler to zero */
	LPC_TIM0->MR0 = delayInUs ;
	LPC_TIM0->IR  = 0xff;		/* reset all interrrupts */
	LPC_TIM0->MCR = 0x04;		/* stop timer on match */
	LPC_TIM0->TCR = 0x01;		/* start timer */

	/* wait until delay time has elapsed */
	while (LPC_TIM0->TCR & 0x01);
  }
  else if ( timer_num == 1 )
  {
	LPC_TIM1->TCR = 0x02;		/* reset timer */
	LPC_TIM1->PR  = 25;		/* set prescaler to zero */
	LPC_TIM1->MR0 = delayInUs;
	LPC_TIM1->IR  = 0xff;		/* reset all interrrupts */
	LPC_TIM1->MCR = 0x04;		/* stop timer on match */
	LPC_TIM1->TCR = 0x01;		/* start timer */

	/* wait until delay time has elapsed */
	while (LPC_TIM1->TCR & 0x01);
  }
  return;
}

void send_sync(void) {
	led_on(1);
	delayUs(0, TAKT);
	led_off(1);
	delayUs(0, TAKT*31);
}

void send_0(void) {
	led_on(1);
	delayUs(0, TAKT);
	led_off(1);
	delayUs(0, TAKT*3);
	led_on(1);
	delayUs(0, TAKT);
	led_off(1);
	delayUs(0, TAKT*3);
}

void send_1(void) {
	led_on(1);
	delayUs(0, TAKT*3);
	led_off(1);
	delayUs(0, TAKT);
	led_on(1);
	delayUs(0, TAKT*3);
	led_off(1);
	delayUs(0, TAKT);
}

void send_F(void) {
	led_on(1);
	delayUs(0, TAKT);
	led_off(1);
	delayUs(0, TAKT*3);
	led_on(1);
	delayUs(0, TAKT*3);
	led_off(1);
	delayUs(0, TAKT);
}

void send_rc_helper(char* data){
	led_on(0);
	while(*data) {
		if (*data == '0') {
			send_0();
		}
		else if (*data == '1') {
			send_1();
		}
		else if ((*data == 'F') || (*data == 'f')) {
			send_F();
		}
		else {
			break;
		}
		data++;
	}
	send_sync();
	led_off(0);
}

void send_rc(char* data) {
	int i=0;
	while(i++ < 5) {
		send_rc_helper(data);
	}
}
void send_test(void) {
	//FF000F0FFFF0
	led_on(0);

	send_F();
	send_F();
	send_0();
	send_0();
	send_0();

	send_F();
	send_F();
	send_F();
	send_0();
	send_F();

	send_0();
	send_F();

	send_sync();

	led_off(0);
}


void send_test2(void) {
	send_rc_helper("FF000F0FFF0F");
}

void send_test3(void) {
	send_rc("FF000F0FFF0F");
}
/*--------------------------- main ---------------------------------*/

char ipstring [20];

extern volatile uint32_t UART2Count;
extern volatile uint8_t UART2Buffer[BUFSIZE];

int main(void)
{
	unsigned int i;
	uip_ipaddr_t ipaddr;	/* local IP address */
	struct timer periodic_timer, arp_timer;

	// Code Red - if CMSIS is being used, then SystemInit() routine
	// will be called by startup code rather than in application's main()
#ifndef __USE_CMSIS
	// system init
	SystemInit();                                      /* setup core clocks */
#endif

	// clock init
	clock_init();
		// two timers for tcp/ip
	timer_set(&periodic_timer, CLOCK_SECOND / 2); /* 0.5s */
	timer_set(&arp_timer, CLOCK_SECOND * 10);	/* 10s */

	// led init
	led_init();
	led2_on();
	//led_all_on();
	delay_10ms(100);
	led_all_off();



	// sensor init
	led_on(0);


	UARTInit(2, 9600);	/* baud rate setting */

	/*
	UARTSendCRLF(2);
	UARTSendCRLF(2);
	UARTSendStringln(2, "UART2 online ...");

	UARTSendString(2, PRODUCT_NAME);
	UARTSendString(2, " v");
	UARTSendNumber(2, VERSION_MAJOR);
	UARTSendString(2, ".");
	UARTSendNumber(2, VERSION_MINOR);
	UARTSendString(2, " BUILD ID ");
	UARTSendStringln(2, VERSION_BUILD_ID);
	UARTSendCRLF(2);

	logger_logStringln("log online ...");

	UARTSendString(2, "wait 500ms ...");
	delay_10ms(50);
	UARTSendStringln(2, " done");
*/

	/*
	// ethernet init
	UARTSendString(2, "init ethernet ...");
	tapdev_init();
	UARTSendStringln(2, " done");

	led_on(4);
	UARTSendString(2, "init TCP/IP stack ...");
	// Initialize the uIP TCP/IP stack.
	uip_init();
	UARTSendStringln(2, " done");

	uip_ipaddr(ipaddr, MYIP_1,MYIP_2,MYIP_3,MYIP_4);
	uip_sethostaddr(ipaddr);
	uip_ipaddr(ipaddr, MYIP_1,MYIP_2,MYIP_3,1);
	uip_setdraddr(ipaddr);
	uip_ipaddr(ipaddr, 255,255,255,0);
	uip_setnetmask(ipaddr);

	led_on(5);
	UARTSendString(2, "init httpd ...");
	// Initialize the HTTP server, listen to port 80.
	httpd_init();
	UARTSendStringln(2, " done");
	*/

	delay_10ms(100);

	UARTSendStringln(2, "entering main loop ...");
	led2_off();
	led_all_off();
	delayMs(0,100);
	led_on(0);
	delayMs(0,100);
	led_off(0);
	delayMs(0,100);
	led_on(0);
	delayMs(0,100);
	led_off(0);

	delayMs(0,1000);

	/*
	send_test();
	send_test();
	send_test();
	send_test();
	send_test();
	*/

	/*
	send_test2();
	send_test2();
	send_test2();
	send_test2();
	send_test2();
	 */
	//send_test3();

	uint32_t s0_msticks = 0;
	uint8_t s0_active = 0;
	uint32_t s0_state = 0;
	uint32_t s0_oldState = 0;
	uint32_t s0_newState = 0;



	while(1)
	{

		s0_state = ~LPC_GPIO2->FIOPIN;
		if (s0_state & (S0_INPUT0)) {
			led_on(0);
			if (~s0_oldState & S0_INPUT0) {
				s0_oldState |= S0_INPUT0;
				led2_off();
				send_rc("FF000F0FFFF0");
			}
			else {
				s0_oldState &= ~S0_INPUT0;
				led2_on();
				send_rc("FF000F0FFF0F");
			}
			delayMs(0,500);
		}
		else if (s0_state & (S0_INPUT1)) {
			led_on(0);
			if (~s0_oldState & S0_INPUT1) {
				s0_oldState |= S0_INPUT1;
				led2_off();
				send_rc("FF000FF0FFF0");
			}
			else {
				s0_oldState &= ~S0_INPUT1;
				led2_on();
				send_rc("FF000FF0FF0F");
			}
			delayMs(0,500);
		}
		else if (s0_state & (S0_INPUT2)) {
			led_on(0);
			if (~s0_oldState & S0_INPUT2) {
				s0_oldState |= S0_INPUT2;
				led2_off();
				send_rc("FF000FFF0FF0");
			}
			else {
				s0_oldState &= ~S0_INPUT2;
				led2_on();
				send_rc("FF000FFF0F0F");
			}
			delayMs(0,500);
		}
		else {
			led_off(0);

		}

		/* process logger */
		if (logger_dataAvailable() && UARTTXReady(2)) {
			uint8_t data = logger_read();
			UARTSendByte(2,data);
		}


		/* process S0 input */
		/* DEBOUNCING 1/2 */
		/*
		if (s0_active == 0) {
			s0_newState = ~LPC_GPIO2->FIOPIN & (S0_INPUT0 | S0_INPUT1 | S0_INPUT2 );
			if (s0_oldState != s0_newState) {
				s0_active = 1;
				s0_msticks = clock_time();
			}
		}
		*/

		/* DEBOUNCING 2/2 */
		/* wait about 200ms */
		/*
		if (s0_active == 1 && wait_ticks(s0_msticks, 20)) {
			s0_state = ~LPC_GPIO2->FIOPIN & (S0_INPUT0 | S0_INPUT1 | S0_INPUT2 );
			if (s0_state == s0_newState) {

				// falling edge
				if ((s0_newState & S0_INPUT0) > 0) {
					led_on(0);
				}

				if ((s0_newState & S0_INPUT1) > 0) {

				}

			}
			s0_oldState = s0_state;
			s0_active = 2;
		}
		*/

		/*
		 * WAIT 1 second until  next trigger event
		 */
		/*
		if (s0_active == 2 && wait_ticks(s0_msticks, 100)) {
			s0_active = 0;
		}
		*/


 	    /* receive packet and put in uip_buf */
		uip_len = tapdev_read(uip_buf);
    	if(uip_len > 0)		/* received packet */
    	{ 
      		if(BUF->type == htons(UIP_ETHTYPE_IP))	/* IP packet */
      		{
	      		uip_arp_ipin();	
	      		uip_input();
	      		/* If the above function invocation resulted in data that
	         		should be sent out on the network, the global variable
	         		uip_len is set to a value > 0. */
 
	      		if(uip_len > 0)
        		{
#if defined ( _RDB1768_)
	      			if (firstconnection == 0) {
	      				firstconnection = 1;
	      			}
#endif
	      			uip_arp_out();
	        		tapdev_send(uip_buf,uip_len);
	      		}
      		}
	      	else if(BUF->type == htons(UIP_ETHTYPE_ARP))	/*ARP packet */
	      	{
	        	uip_arp_arpin();
		      	/* If the above function invocation resulted in data that
		         	should be sent out on the network, the global variable
		         	uip_len is set to a value > 0. */
		      	if(uip_len > 0)
	        	{
		        	tapdev_send(uip_buf,uip_len);	/* ARP ack*/
		      	}
	      	}
    	}
    	else if(timer_expired(&periodic_timer))	/* no packet but periodic_timer time out (0.5s)*/
    	{
      		timer_reset(&periodic_timer);
	  
      		for(i = 0; i < UIP_CONNS; i++)
      		{
      			uip_periodic(i);
		        /* If the above function invocation resulted in data that
		           should be sent out on the network, the global variable
		           uip_len is set to a value > 0. */
		        if(uip_len > 0)
		        {
		          uip_arp_out();
		          tapdev_send(uip_buf,uip_len);
		        }
      		}
	     	/* Call the ARP timer function every 10 seconds. */
			if(timer_expired(&arp_timer))
			{
				timer_reset(&arp_timer);
				uip_arp_timer();
			}
    	}


	}
}
