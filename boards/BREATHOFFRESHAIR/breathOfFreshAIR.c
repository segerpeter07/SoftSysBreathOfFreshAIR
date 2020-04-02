/*----- Includes -----*/
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"
#include "log_uart.h"

/*********** MARCO POLOS *************/

/**************** Outputs **************/
#define PRECHARGE_CTRL			PB2
#define PRECHARGE_PORT		PORTB
#define DDR_PRECHARGE			 DDRB
#define AIRMINUS_LSD			  PC6
#define AIRMINUS_PORT			PORTC
#define DDR_AIRMINUS			 DDRC

/***************** Inputs **************/
#define PIN_AIRPLUS_AUX_CONTACTS				 PC4 // PCINT12
#define INREG_AIRPLUS_AUX_CONTACTS			PINC // INREG -> input register
#define PIN_AIRMINUS_AUX_CONTACTS			   PC5 // PCINT13
#define INREG_AIRPMINUS_AUX_CONTACTS	 	PINC

#define PIN_BMS_STATUS		  	PC0 // PCINT8
#define INREG_BMS_STATUS	   PINC
#define PIN_IMD_STATUS		  	PD0 // PCINT16
#define INREG_IMD_STATUS	   PIND

#define PIN_TSMS_SENSE_n				PB3 // PCINT3
#define INREG_TSMS_SENSE			 PINB
#define PIN_IMD_SENSE_n					PB4 // PCINT4
#define INREG_IMD_SENSE			 	 PINB
#define PIN_TSCONN_SENSE_n			PB5 // PCINT5
#define INREG_TSCONN_SENSE		 PINB
#define PIN_HVDCONN_SENSE_n			PB6 // PCINT6
#define INREG_HVDCONN_SENSE	 	 PINB
#define PIN_HVD_SENSE_n					PB7 // PCINT7
#define INREG_HVD_SENSE			 	 PINB
#define PIN_BMS_SENSE_n					PC7 // PCINT9
#define INREG_BMS_SENSE			 	 PINC

// Message Objects
#define MOB_BMS_VOLTAGE 0
#define MOB_MC_VOLTAGE 1
#define MOB_SEND_STATE 2
#define MOB_SEND_SHUTDOWN_STATUS 3

// STATES
#define LV_ON_DEENERGIZED 0
#define PRECHARGE_STUCK_CLOSED_CHECK 1
#define PRECHARGE_STUCK_OPEN_CHECK 2
#define PRECHARGE 3
#define ENERGIZED 4
#define DISCHARGE_STUCK_OPEN_CHECK 5
#define DISCHARGE 6
#define PANIC 7

// gFlag flag bit names
#define UPDATE_STATUS 0
#define BUS_VOLTAGE_RECEIVED 1
#define BMS_MESSAGE_RECEIVED 2
#define GENERAL_TIMER_COMPLETE 3
#define AIRPLUS_CLOSED 4
#define AIRMINUS_CLOSED 5

// shutdownSenseFlag flag bit names
#define TSMS_SENSE 0
#define IMD_SENSE 1
#define TSCONN_SENSE 2
#define HVDCONN_SENSE 3
#define HVD_SENSE 4
#define BMS_SENSE 5
#define IMD_STATUS 6
#define BMS_STATUS 7

// PANIC transitions
// iN THE STYLE OF mATLAB
#define FAILED_PRECHARGE_STUCK_CLOSED_CHECK 1
#define FAILED_PRECHARGE_STUCK_OPEN_CHECK 2
#define PRECHARGE_TIMEOUT 3
#define FAILED_DISCHARGE_STUCK_OPEN_CHECK 4
#define DISCHARGE_TIMEOUT 5
#define FAILED_STATE_INDEPENDENT_CHECKS 6


// Globals
volatile uint8_t gFlag = 0x00;
volatile uint8_t shutdownSenseFlag = 0x00;
volatile uint16_t bus_voltage = 0xffff;
volatile uint16_t pack_voltage = 0xffff;
volatile uint16_t timer1CompareMatchCount = 0x00;
uint16_t timer1CompareMatchCountThreshold = 0x00;
const uint8_t number_of_bms_can_messages = 24;
const uint16_t bms_can_messages[][] = {
  {CAN_ID_BMS_VOLTAGES_1_1, CAN_LEN_BMS_VOLTAGES_1_1},
  {CAN_ID_BMS_VOLTAGES_1_2, CAN_LEN_BMS_VOLTAGES_1_2},
  {CAN_ID_BMS_VOLTAGES_1_3, CAN_LEN_BMS_VOLTAGES_1_3},
  {CAN_ID_BMS_VOLTAGES_1_4, CAN_LEN_BMS_VOLTAGES_1_4},
  {CAN_ID_BMS_VOLTAGES_2_1, CAN_LEN_BMS_VOLTAGES_2_1},
  {CAN_ID_BMS_VOLTAGES_2_2, CAN_LEN_BMS_VOLTAGES_2_2},
  {CAN_ID_BMS_VOLTAGES_2_3, CAN_LEN_BMS_VOLTAGES_2_3},
  {CAN_ID_BMS_VOLTAGES_2_4, CAN_LEN_BMS_VOLTAGES_2_4},
  {CAN_ID_BMS_VOLTAGES_3_1, CAN_LEN_BMS_VOLTAGES_3_1},
  {CAN_ID_BMS_VOLTAGES_3_2, CAN_LEN_BMS_VOLTAGES_3_2},
  {CAN_ID_BMS_VOLTAGES_3_3, CAN_LEN_BMS_VOLTAGES_3_3},
  {CAN_ID_BMS_VOLTAGES_3_4, CAN_LEN_BMS_VOLTAGES_3_4},
  {CAN_ID_BMS_VOLTAGES_4_1, CAN_LEN_BMS_VOLTAGES_4_1},
  {CAN_ID_BMS_VOLTAGES_4_2, CAN_LEN_BMS_VOLTAGES_4_2},
  {CAN_ID_BMS_VOLTAGES_4_3, CAN_LEN_BMS_VOLTAGES_4_3},
  {CAN_ID_BMS_VOLTAGES_4_4, CAN_LEN_BMS_VOLTAGES_4_4},
  {CAN_ID_BMS_VOLTAGES_5_1, CAN_LEN_BMS_VOLTAGES_5_1},
  {CAN_ID_BMS_VOLTAGES_5_2, CAN_LEN_BMS_VOLTAGES_5_2},
  {CAN_ID_BMS_VOLTAGES_5_3, CAN_LEN_BMS_VOLTAGES_5_3},
  {CAN_ID_BMS_VOLTAGES_5_4, CAN_LEN_BMS_VOLTAGES_5_4},
  {CAN_ID_BMS_VOLTAGES_6_1, CAN_LEN_BMS_VOLTAGES_6_1},
  {CAN_ID_BMS_VOLTAGES_6_2, CAN_LEN_BMS_VOLTAGES_6_2},
  {CAN_ID_BMS_VOLTAGES_6_3, CAN_LEN_BMS_VOLTAGES_6_3},
  {CAN_ID_BMS_VOLTAGES_6_4, CAN_LEN_BMS_VOLTAGES_6_4}
}

/************ ISRs ************/
ISR(PCINT0_vect) { // PCINT0-7 -> TSMS_SENSE, IMD_SENSE, TSCONN_SENSE, HVDCONN_SENSE, HVD_SENSE
    if(bit_is_clear(INREG_TSMS_SENSE, PIN_TSMS_SENSE_n)){
		 shutdownSenseFlag |= _BV(TSMS_SENSE);
		} else {
		 shutdownSenseFlag &= ~_BV(TSMS_SENSE);
		}
		
		if(bit_is_clear(INREG_IMD_SENSE, PIN_IMD_SENSE_n)){
		 shutdownSenseFlag |= _BV(IMD_SENSE);
		} else {
		 shutdownSenseFlag &= ~_BV(IMD_SENSE);
		}
		
		if(bit_is_clear(INREG_TSCONN_SENSE, PIN_TSCONN_SENSE_n)){
		 shutdownSenseFlag |= _BV(TSCONN_SENSE);
		} else {
		 shutdownSenseFlag &= ~_BV(TSCONN_SENSE);
		}
		
		if(bit_is_clear(INREG_HVDCONN_SENSE, PIN_HVDCONN_SENSE_n)){
		 shutdownSenseFlag |= _BV(HVDCONN_SENSE);
		} else {
		 shutdownSenseFlag &= ~_BV(HVDCONN_SENSE);
		}
		
		if(bit_is_clear(INREG_HVD_SENSE, PIN_HVD_SENSE_n)){
		 shutdownSenseFlag |= _BV(HVD_SENSE);
		} else {
		 shutdownSenseFlag &= ~_BV(HVD_SENSE);
		}
}

ISR(PCINT1_vect) { // PCINT8-15 -> BMS_STATUS, BMS_SENSE, AIRPLUS_AUX, AIRMINUS_AUX
		if(bit_is_set(INREG_BMS_STATUS, PIN_BMS_STATUS)){
		 shutdownSenseFlag |= _BV(BMS_STATUS);
		} else {
		 shutdownSenseFlag &= ~_BV(BMS_STATUS);
		}
		
		if(bit_is_clear(INREG_BMS_SENSE, PIN_BMS_SENSE_n)){
		 shutdownSenseFlag |= _BV(BMS_SENSE);
		} else {
		 shutdownSenseFlag &= ~_BV(BMS_SENSE);
		}
		
		if(bit_is_set(INREG_AIRPLUS_AUX_CONTACTS, PIN_AIRPLUS_AUX_CONTACTS)){
		 gFlag |= _BV(AIRPLUS_CLOSED);
		} else {
		 gFlag &= ~_BV(AIRPLUS_CLOSED);
		}
		
		if(bit_is_set(INREG_AIRMINUS_AUX_CONTACTS, PIN_AIRMINUS_AUX_CONTACTS)){
		 gFlag |= _BV(AIRMINUS_CLOSED);
		} else {
		 gFlag &= ~_BV(AIRMINUS_CLOSED);
		}
}

ISR(PCINT2_vect) { // PCINT16 -> IMD_SENSE
		if(bit_is_clear(INREG_IMD_SENSE, PIN_IMD_SENSE_n)){
		 shutdownSenseFlag |= _BV(IMD_SENSE);
		} else {
		 shutdownSenseFlag &= ~_BV(IMD_SENSE);
		}
}

ISR(CAN_INT_vect) {
		CANPAGE = (MOB_MC_VOLTAGE << MOBNB0);
	  if (bit_is_set(CANSTMOB,RXOK)) {
	      uint8_t low_byte = CANMSG;
	      uint8_t high_byte = CANMSG;

	      bus_voltage = low_byte | (high_byte<<8);

	      CANSTMOB = 0x00;
        gFlag |= _BV(BUS_VOLTAGE_RECEIVED);
	  }
    
    CANPAGE = (MOB_BMS_VOLTAGE << MOBNB0);
	  if (bit_is_set(CANSTMOB,RXOK)) {
	      uint8_t cell_1_high_byte = CANMSG;
	      uint8_t cell_1_low_byte = CANMSG;
        uint8_t cell_2_high_byte = CANMSG;
	      uint8_t cell_2_low_byte = CANMSG;
        uint8_t cell_3_high_byte = CANMSG;
	      uint8_t cell_3_low_byte = CANMSG;
        uint8_t cell_4_high_byte = CANMSG;
	      uint8_t cell_4_low_byte = CANMSG;

	      pack_voltage += (cell_1_low_byte | (cell_1_high_byte<<8)) >> 13;
        pack_voltage += (cell_2_low_byte | (cell_2_high_byte<<8)) >> 13;
        pack_voltage += (cell_3_low_byte | (cell_3_high_byte<<8)) >> 13;
        pack_voltage += (cell_4_low_byte | (cell_4_high_byte<<8)) >> 13;

	      CANSTMOB = 0x00;
        gFlag |= _BV(BMS_MESSAGE_RECEIVED);
	  }
}

ISR(TIMER0_COMPA_vect) {
    /*
    Timer/Counter0 compare match A
    If the clock frequency is 4MHz then this is called 16 times per second
    MATH: (4MHz/1024)/255 = ~16
    */

		gFlag |= _BV(UPDATE_STATUS);
}

ISR(TIMER1_OVF_vect) {
		timer1OverflowCount++;
    if (timer1OverflowCount = timer1OverflowCountThreshold) {
      gFlag |= _BV(GENERAL_TIMER_COMPLETE);
    }
    #TODO
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
		TCCR1B |= _BV(CS10); // no prescaler set
    uint16_t output_compare_match = 4000;
    OCR1AL = output_compare_match & 0xFF;
    OCR1AH = (output_compare_match >> 8);
		// 4MHz CPU, prescaler 1, 16-bit timer overflow -> (4000000/8)/(2^16-1) =  7.63 Hz
		TIMSK1 = 0x01; // enable interrupt on overflow
    #TODO
}

void resetTimer1(void) {
		cli();
		TCNT1H = 0x00; // write timer count to 0, high byte must be written first per datasheet
		TCNT1L = 0x00;
		sei();
		timer1CompareMatchCount = 0x00;
    gFlag &= ~_BV(GENERAL_TIMER_COMPLETE);
}

/*
Calculates time threshold and resets timer 1
*/
void startTimer(uint16_t milliseconds) {
  timer1CompareMatchCountThreshold = milliseconds;
  resetTimer1();
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
Somehow gets CAN message information, sums pack voltage and returns it

return: uint8_t voltage of pack (maybe x10 or something idfk)
*/
uint8_t* get_pack_voltage (void) {
  pack_voltage = 0;
  for (int i; i < number_of_bms_can_messages; i++) {
    gFlag &= ~_BV(BMS_VOLTAGE_RECEIVED);
    CAN_wait_on_receive(MOB_BMS_VOLTAGE,
  	                          bms_can_messages[i][0], // id
  	                          bms_can_messages[i][1], // length
  	                          CAN_MSK_SINGLE);
    while (bit_is_clear(gFlag, BMS_VOLTAGE_RECEIVED)) {
      // do nothing, waiting for message to be received
    }
  }
  return *pack_voltage;
}

/*
Listen for bus voltage from motor controller

return: uint16_t bus voltage (I think from motor controller it's x10) #TODO
*/
uint16_t* get_bus_voltage (void) {
	gFlag &= ~_BV(BUS_VOLTAGE_RECEIVED);
  CAN_wait_on_receive(MOB_MC_VOLTAGE,
	                          CAN_ID_MC_VOLTAGE,
	                          CAN_LEN_MC_VOLTAGE,
	                          CAN_MSK_SINGLE);
  while (bit_is_clear(gFlag, BUS_VOLTAGE_RECEIVED)) {
    // do nothing, waiting for message to be received
  }
  return *bus_voltage;
}

/*
Does precharge. #TODO better dick string

return: 0 if OK status, 1 if fault, -1 if cancelled
*/
int8_t do_precharge (void) {
  uint16_t *pack_voltage = get_pack_voltage();
  startTimer(2000);
	if (bit_is_clear(shutdownSenseFlag, TSMS_SENSE)) {
		return -1;
	} else if (*get_bus_voltage()*10 >= pack_voltage*9) { // compare bus voltage to 90% of pack voltage
		return 0;
	} else if (bit_is_set(gFlag, GENERAL_TIMER_COMPLETE)) {
		return 1;
	}
}

/*
Does discharge. #TODO better dick string

return: 0 if OK status, 1 if fault
*/
uint8_t do_discharge (void) {
  #TODO
}

/*
Checks if precharge is stuck open.

return: 0 if OK status, 1 if fault, -1 if cancelled
*/
int8_t check_precharge_stuck_open (void) {
  startTimer(100);
	if (bit_is_clear(shutdownSenseFlag, TSMS_SENSE)) {
		return -1;
	} else if (*get_bus_voltage() > 0) {
		return 0;
	} else if (bit_is_set(gFlag, GENERAL_TIMER_COMPLETE)) {
		return 1;
	}
}

/*
Checks if precharge is stuck closed.

return: 0 if OK status, 1 if fault, -1 if cancelled
*/
int8_t check_precharge_stuck_closed (void) {
	startTimer(1000);
	if (bit_is_clear(shutdownSenseFlag, TSMS_SENSE)) {
		return -1;
	} else if (*get_bus_voltage() > 0) {
		return 1;
	} else if (bit_is_set(gFlag, GENERAL_TIMER_COMPLETE)) {
		return 0;
	}
}

/*
Checks if discharge is stuck open.

return: 0 if OK status, 1 if fault, -1 if bus voltage already 0
*/
int8_t check_discharge_stuck_open (void) {
  startTimer(100);
  uint16_t *bus_voltage = get_bus_voltage();
	if (*bus_voltage == 0) {
		return -1;
	} else if (*get_bus_voltage() < *bus_voltage) { #TODO // deal with bus voltage pointer issue
		return 0;
	} else if (bit_is_set(gFlag, GENERAL_TIMER_COMPLETE)) {
		return 1;
	}
}

/*
Sends shutdown node status CAN message.
*/
void send_shutdown_CAN_msg (void) {
  uint8_t byte0 = if(bit_is_set(shutdownSenseFlag, TSMS_SENSE)*0xFF;
  uint8_t byte1 = if(bit_is_set(shutdownSenseFlag, IMD_SENSE)*0xFF;
  uint8_t byte2 = if(bit_is_set(shutdownSenseFlag, TSCONN_SENSE)*0xFF;
  uint8_t byte3 = if(bit_is_set(shutdownSenseFlag, HVDCONN_SENSE)*0xFF;
  uint8_t byte4 = if(bit_is_set(shutdownSenseFlag, HVD_SENSE)*0xFF;
  uint8_t byte5 = if(bit_is_set(shutdownSenseFlag, BMS_SENSE)*0xFF;
  uint8_t byte6 = if(bit_is_set(shutdownSenseFlag, IMD_STATUS)*0xFF;
  uint8_t byte7 = if(bit_is_set(shutdownSenseFlag, BMS_STATUS)*0xFF;
  uint8_t msg[] = {byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7};
  CAN_transmit(MOB_SEND_SHUTDOWN_STATUS,
                CAN_ID_AIR_CONTROL_SENSE,
                CAN_LEN_AIR_CONTROL_SENSE,
                msg);
}

/*
Sends state status CAN message.
*/
void send_state_CAN_msg (uint8_t error_code, uint8_t state, uint8_t state_independent_check_response) {
  uint8_t byte3 = if(bit_is_set(gFlag, AIRPLUS_CLOSED)*0xFF;
  uint8_t byte4 = if(bit_is_set(gFlag, AIRMINUS_CLOSED)*0xFF;
  uint8_t msg[] = {error_code, state, state_independent_check_response, byte3, byte4};
  CAN_transmit(MOB_SEND_STATE,
                CAN_ID_AIR_CONTROL_CRITICAL,
                CAN_LEN_AIR_CONTROL_CRITICAL,
                msg);
}

/*********** SOME RANDOM INIT CRAP TO REVIEW LATER **************/ #TODO
void setOutputs(void) {
		//Sets these pins at outputs
		DDR_PRECHARGE |= _BV(PRECHARGE_CTRL);
		DDR_AIRMINUS |= _BV(AIRMINUS_LSD);
}

void readAllInputs(void) {
		PCINT0_vect();
		PCINT1_vect();
		PCINT2_vect();
}

void enableInterrupts(void) {
	PCICR |= _BV(PCIE0) | _BV(PCIE1) | _BV(PCIE2);
	PCMSK0 |= _BV(PCINT3) | _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6) | _BV(PCINT7);
	PCMSK1 |= _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT12) | _BV(PCINT13);
	PCMSK2 |= _BV(PCINT16);
}

/********** MAIN ***********/

int main (void) {
	uint8_t state = LV_ON_DEENERGIZED;
	uint8_t state_independent_check_response;
	uint8_t panic_transition = 0x00;

	initTimer0();
	initTimer1();
	CAN_init(CAN_ENABLED);
	// LOG_init(); if we add an UART
	setOutputs();
	sei(); // globally enable interrupts
	enableInterrupts();
	readAllInputs();
	
	while(1) {
		if(bit_is_set(gFlag, UPDATE_STATUS)){
			
			gFlag &= ~_BV(UPDATE_STATUS);
			#TODO // consider always checking for PANIC address from all other boards and adding that as a PANIC transition
			state_independent_check_response = state_independent_checks();
			if (state_independent_check_response != 0) {
				state = PANIC;
        panic_transition = FAILED_STATE_INDEPENDENT_CHECKS;
			}
			switch(state) {
				case LV_ON_DEENERGIZED:
					// if voltage present after TSMS
					if (bit_is_set(shutdownSenseFlag, TSMS_SENSE)) {
						state = PRECHARGE_STUCK_CLOSED_CHECK;
					}
					break;
				case PRECHARGE_STUCK_CLOSED_CHECK:
					switch (check_precharge_stuck_closed()) {
						case -1: // cancelled
							state = LV_ON_DEENERGIZED;
							break;
						case 0: // status OK
							state = PRECHARGE_STUCK_OPEN_CHECK;
							break;
						case 1: // fault
							state = PANIC;
							panic_transition = FAILED_PRECHARGE_STUCK_CLOSED_CHECK;
							break;
					}
					break;
				case PRECHARGE_STUCK_OPEN_CHECK:
          PRECHARGE_PORT |= _BV(PRECHARGE_CTRL); // close precharge relay
					switch (check_precharge_stuck_open()) {
						case -1: // cancelled
							state = DISCHARGE_STUCK_OPEN_CHECK;
              PRECHARGE_PORT &= ~_BV(PRECHARGE_CTRL); // open precharge relay
							break;
						case 0: // status OK
							state = PRECHARGE;
							break;
						case 1: // fault
							state = PANIC;
							panic_transition = FAILED_PRECHARGE_STUCK_OPEN_CHECK;
							break;
					}
					break;
				case PRECHARGE:
					switch (do_precharge()) {
						case -1: // cancelled
							state = DISCHARGE_STUCK_OPEN_CHECK;
              PRECHARGE_PORT &= ~_BV(PRECHARGE_CTRL); // open precharge relay
							break;
						case 0: // status OK
							AIRMINUS_PORT |= _BV(AIRMINUS_LSD); // close AIR-
							PRECHARGE_PORT &= ~_BV(PRECHARGE_CTRL); // open precharge relay
							state = ENERGIZED;
							break;
						case 1: // fault
							state = PANIC;
							panic_transition = PRECHARGE_TIMEOUT;
							break;
					}
					break;
				case ENERGIZED:
					// if no voltage present after TSMS
					if (bit_is_clear(shutdownSenseFlag, TSMS_SENSE)) {
						AIRMINUS_PORT &= ~_BV(AIRMINUS_LSD); // open AIR-
						state = DISCHARGE_STUCK_OPEN_CHECK;
					}
					break;
				case DISCHARGE_STUCK_OPEN_CHECK:
					switch (check_discharge_stuck_open()) {
						case -1: // bus voltage already 0
							state = LV_ON_DEENERGIZED;
							break;
						case 0: // status OK
							state = DISCHARGE;
							break;
						case 1: // fault
							state = PANIC;
							panic_transition = FAILED_DISCHARGE_STUCK_OPEN_CHECK;
							break;
					}
					break;
				case DISCHARGE:
					switch (do_discharge()) {
						case 0: // status OK
							state = LV_ON_DEENERGIZED;
							break;
						case 1: // fault
							state = PANIC;
							panic_transition = DISCHARGE_TIMEOUT;
							break;
					}
					break;
				case PANIC:
					// open AIR+ and precharge relay
					AIRMINUS_PORT &= ~_BV(AIRMINUS_LSD);
					PRECHARGE_PORT &= ~_BV(PRECHARGE_CTRL);
					break;
			}
			send_shutdown_CAN_msg();
      send_state_CAN_msg(panic_transition, state, state_independent_check_response);
		}
	}
}