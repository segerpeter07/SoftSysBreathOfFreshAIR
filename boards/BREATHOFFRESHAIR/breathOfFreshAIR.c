/*----- Includes -----*/
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"
#include "log_uart.h"

/*********** MARCO POLOS *************/

// Message Objects
#define MOB_BMS_VOLTAGE 0
#define MOB_MC_VOLTAGE 1
#define MOB_SEND_STATE 2
#define MOB_SEND_SHUTDOWN_STATUS 3

// gFlag flag bit names
#define BUS_VOLTAGE_RECEIVED 0

// Globals
volatile uint8_t gFlag = 0x00;
volatile uint16_t motorControllerVoltage = 0xffff;

/************ ISRs ************/
ISR(CAN_INT_vect) {
		CANPAGE = (MOB_MC_VOLTAGE << MOBNB0);
	  if (bit_is_set(CANSTMOB,RXOK)) {
	      msgMC[0] = CANMSG;
	      msgMC[1] = CANMSG;

	      motorControllerVoltage = msgMC[0] | (msgMC[1]<<8);

	      CANSTMOB = 0x00;
        gFlag |= _BV(BUS_VOLTAGE_RECEIVED);
	  }
}

/********** TiMeRs ************/
void initTimer0(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);  // Every 1024 cycles, OCR0A increments
    OCR0A = 0xff; //dec 39  // until 0xff, 255, which then calls for
                            // the TIMER0_COMPA_vect interrupt
			    // currently running at 100Hz
}

void initTimer1(void) {
		// Normal operation so no need to set TCCR1A
		TCCR1B |= _BV(CS11); // prescaler set to 8
		// 4MHz CPU, prescaler 8, 16-bit timer overflow -> (4000000/8)/(2^16-1) =  7.63 Hz
		TIMSK1 = 0x01; // enable interrupt on overflow
}

void resetTimer1(void) {
		cli();
		TCNT1H = 0x00; // write timer count to 0, high byte must be written first per datasheet
		TCNT1L = 0x00;
		sei();
		timer1OverflowCount = 0x00;
}

/************ STATE FUNCS **************/

/*
Runs state independent checks and returns fault code or 0 for OK status.
Checks AIR+/- weld/stuck open. Checks IMD/BMS power stage implausibility.

return: uint8_t fault code or 0 for OK status if all checks passed
*/
uint8_t state_independent_checks (void) {
  #TODO
  // check AIR+
  // check AIR-
  // check IMD
  // check BMS
}

/*
Checks if precharge is stuck open.

return: 0 if OK status, 1 if fault
*/
uint8_t check_precharge_stuck_open (void) {
  #TODO
}

/*
Checks if precharge is stuck closed.

return: 0 if OK status, 1 if fault
*/
uint8_t check_precharge_stuck_closed (void) {
  #TODO
}

/*
Checks if discharge is stuck open.

return: 0 if OK status, 1 if fault
*/
uint8_t check_discharge_stuck_open (void) {
  #TODO
}

/*
Somehow gets CAN message information, sums pack voltage and returns it

return: uint8_t voltage of pack (maybe x10 or something idfk)
*/
uint8_t get_pack_voltage (void) {
  #TODO
  // maybe use one MOB and iterate through listening to each message
}

/*
Listen for bus voltage from motor controller

return: uint8_t bus voltage (I think from motor controller it's x10)
*/
uint16_t* get_bus_voltage (void) {
  CAN_wait_on_receive(MOB_MC_VOLTAGE,
	                          CAN_ID_MC_VOLTAGE,
	                          CAN_LEN_MC_VOLTAGE,
	                          CAN_MSK_SINGLE);
  while (bit_is_clear(gFlag, BUS_VOLTAGE_RECEIVED)) {
    // do nothing, waiting for message to be received
  }
  return *motorControllerVoltage;
}

/*
Sends shutdown node status CAN message.
*/
void send_shutdown_CAN_msg (void) {
  #TODO
}

/*
Sends state status CAN message.
*/
void send_state_CAN_msg (void) {
  #TODO
}

/********** MAIN ***********/

int main (void) {
  #TODO
}