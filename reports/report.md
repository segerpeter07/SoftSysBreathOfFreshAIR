# BreathOfFreshAIR Project Report
[Lucky Jordan](https://github.com/ljordan51) & [Peter Seger](https://github.com/segerpeter07)

## Project Goals
This project is inspiried by our work and experience on Olin Electric Motorsports (Formula) over the past 3 years working on electric vehicle firmware design and implementation. 

The Accumulator Isolation Relay (AIR) control board detects when the the tractive system master switch (TSMS) is closed and then completes the precharge sequence by closing the precharge relay until the TS bus reaches 90% of the accumulator voltage and then opening the precharge relay and closing the low side AIR. During this process, it detects a number of faults including failures of the AIRs and other safety system relays and opens the low side AIR in the case of a fault. 

Both Lucky and I have been on Formula for the past 3 years and have written firmware for the past 3 vehicles. The AIR firmware is some of the most important code for the car and is also some of worst understood code in our entire codebase. We want to change that. 

Not only do we want to re-write the firmware to adhere more strictly to correct C coding structure, we want to make it significantly easier to understand to future team members. 

Since we already have working AIR code from previous years, our project's lower bound would be to just make sure we don't break that. Beyond that, we feel confident in re-working the code to work better for our car's efficiency and our team's longevity. 

## Learning Goals
### Lucky
I want to improve my understanding of how to structure larger blocks of code that manage complex state in C. I particularly want to focus on creating robust, readable code to the end of reducing the likelihood that important variables are edited in places where they shouldn't be or at least increasing the likelihood that those kinds of mistakes are easy to detect when debugging.

### Peter
Similarly to Lucky, I want to learn more about state management in a complex system. In addition, I want to learn more about building robust, error-resistent code that is highly re-useable. Another thing that I would love to learn, however might be outside the scope of this project, is firmware verification systems that would allow us (and others) to test/verify our firmware before it is loaded to our boards. This would be awesome to learn because it would have a big impact on our team.

## Resources
We didn't use that many outside resources to complete this project. During the designing phase, we explored alternatives to Finite State Machines (FSM) and collaborated with Dr. Riccardo Pucella during this exploration phase. While we ended up determining that a FSM was the best for our application after all, he did suggest the value of designing a solid state diagram that would help us plan our architecture better. 

## Outcomes
We were able to  successfully re-design and write a new set of firmware for the AIR control board. Giving ourselves adequate time to actually design and architect the new firmware was a really enjoyable experience. In the past, the firmware for our team has been thrown together at the last minute and often under immense presssure, so having the time and space to actually think through decisions was a welcome change. The actually code-writing process was done as partner-programming style which allowed both Lucky and Peter the ability to ask questions and really think through each logical pattern or decision. 

Our new firmware (found [here](../boards/BREATHOFFRESHAIR/breathOfFreshAir.c)), is significantly less complicated and easier to read with succinct docstrings for each function and a multiple, well named helper functions. 

Read our code [here](../boards/BREATHOFFRESHAIR/breathOfFreshAir.c).

### State Diagram
![State Diagram](./State_Diagram2.png)
Larger diagram can be found [here](./State_Diagram2.png).

The state diagram was a very important step for our project. Since this board manages quite a complex set of states and potential transitions, displaying this easily for team members to understand what is going on was very important. Explicitly writing all the states and transitions also allowed us to notice the repeated state and transitions that exist in the system. This would allow us to design a more efficient FSM without redundant nodes and transitions leading to more efficient firmware.

## Design Decision Example
One design limitation we reached was with resepect to the CAN message object limits that exist on out Atmega16m1 microprocessor. This specific model only contains 6 CAN message objects however we actually need more than 6 to store all the data we might be reading from CAN. However, we don't need to have access to all 6 at all times, meaning we can, at times, share message objects. This does mean, though, that we need to keep track of when we can share message objects and when we cannot. 

For example, here are the data we need to read from or sent to the CAN bus and the number of message addresses that are associated with each of those data:
| Data        | Addresses
| ------------- |:-------------:
| State msg      | 1 
| Shutdown msg      | 1      
| Bus Voltage | 1
| Pack Voltage | 24

As you can see, if we dedicated 1 message object to each address, we would need much more than 6 total message objects to send/receive all these messages. Fortunately for us, we also don't need to be able to send/receive the messages at all times, which means we can intelligently share some of the message objects.

For this, we needed to be able to signal when a certain message object was in use so we could iterate through assigning all the addresses for measuring pack voltage to one message object. 

```c
ISR(CAN_INT_vect) {
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

    // combine bytes and bit shift to right by 13 to get voltage x10
    pack_voltage += (cell_1_low_byte | (cell_1_high_byte<<8)) >> 13;
    pack_voltage += (cell_2_low_byte | (cell_2_high_byte<<8)) >> 13;
    pack_voltage += (cell_3_low_byte | (cell_3_high_byte<<8)) >> 13;
    pack_voltage += (cell_4_low_byte | (cell_4_high_byte<<8)) >> 13;

    CANSTMOB = 0x00;
    gFlag |= _BV(BMS_MESSAGE_RECEIVED);
  }
}
```
As this interrupt shows, when it receives one of the pack voltage messages, it sets the `gFlag` high in the `BMS_MESSAGE_RECEIVED` position. Then, in our `get_pack_voltage` function, we can check to see when the messasge object is free to receive another message and iterate through all the messages we need to read using that flag.
```c
uint16_t get_pack_voltage (void) {
  pack_voltage = 0;
  for (int i; i < number_of_bms_can_messages; i++) {
    gFlag &= ~_BV(BMS_MESSAGE_RECEIVED);
    CAN_wait_on_receive(MOB_BMS_VOLTAGE,
  	                          bms_can_messages[i][0], // id
  	                          bms_can_messages[i][1], // length
  	                          CAN_MSK_SINGLE);
    while (bit_is_clear(gFlag, BMS_MESSAGE_RECEIVED)) {
      // do nothing, waiting for message to be received
    }
  }
  return pack_voltage;
}
```
This simple flag setting architecture allowed us to more efficiently use our message objects and makes it much more clear when they are available for new message addresses. 

## Reflection
We are both quite happy with the outcome of our project since we feel that we have had a positive impact on the team and are leaving behind more useable, readable, and robust code. We both wanted to gain a better understanding of managing state across a complex program in C and feel that we have learned quite a bit about this. While our main style of architecture hasn't changed much (still a FSM), we now place more value in the actual planning and architecting of these types of complex systems. We especially value the process of actually creating a state diagram to help better understand the system, reduce work and complexity, and increase transparency about what our code actually does to others. These are all valuable skills and we are happy to have learned and applied them to this project.

[Trello Board](https://trello.com/b/P1uwdoRG/breath-of-fresh-air) & [Github](https://github.com/segerpeter07/SoftSysBreathOfFreshAIR)

