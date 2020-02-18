# SoftSysBreathOfFreshAIR

## Project Goals
This project is inspiried by our work and experience on Olin Electric Motorsports (Formula) over the past 3 years working on electric vehicle firmware design and implementation. 

The Accumulator Isolation Relay (AIR) control board detects when the the tractive system master switch (TSMS) is closed and then completes the precharge sequence by closing the precharge relay until the TS bus reaches 90% of the accumulator voltage and then opening the precharge relay and closing the high side AIR. During this process, it detects a number of faults including failures of the AIRs and other safety system relays and opens the high side AIR in the case of a fault. 

Both Lucky and I have been on Formula for the past 3 years and have written firmware for the past 3 vehicles. The AIR firmware is some of the most important code for the car and is also some of worst understood code in our entire codebase. We want to change that. 

Not only do we want to re-write the firmware to adhere more strictly to correct C coding structure, we want to make it significantly easier to understand to future team members. 

Since we already have working AIR code from previous years, our project's lower bound would be to just make sure we don't break that. Beyond that, we feel confident in re-working the code to work better for our car's efficiency and our team's longevity. 

## Learning Goals
### Lucky
I want to improve my understanding of how to structure larger blocks of code that manage complex state in C. I particularly want to focus on creating robust, readable code to the end of reducing the likelihood that important variables are edited in places where they shouldn't be or at least increasing the likelihood that those kinds of mistakes are easy to detect when debugging.

### Peter
Similarly to Lucky, I want to learn more about state management in a complex system. In addition, I want to learn more about building robust, error-resistent code that is highly re-useable. Another thing that I would love to learn, however might be outside the scope of this project, is firmware verification systems that would allow us (and others) to test/verify our firmware before it is loaded to our boards. This would be awesome to learn because it would have a big impact on our team.


## Getting Started
To test our firmware changes, we will need a benchtop test which Lucky will be preparing with off the shelf relays and switches. Additionally, we already have custom hardware (PCBs) that we can use to load our firmware onto.

We already have the code from the past 4 years of AIR boards that we can look at and use as inspiriation, so one of our first steps will be to read through and really understand the code logic well.

## Plan of Action
1) Lucky will establish a benchtop test environment where we can test the functionality of our firmware.
2) We will both thoroughly read the previous firmware and take notes on the logic of each section/function.
3) We will plan a high level architecture layout for the firmware.
4) We will implement each function defined in our architecture plan.
5) Testing throughout development to ensure full functionality is met.

