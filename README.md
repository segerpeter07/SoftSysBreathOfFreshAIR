# Breath Of Fresh AIR
[Lucky Jordan](https://github.com/ljordan51) & [Peter Seger](https://github.com/segerpeter07)

## Project Propsal
Can be found [here](./reports/proposal.md)

## Final Report
Can be found [here](./reports/report.md)

## Compiling and Running
These steps are necessary to get the necessary packages and tools needed for the [Olin Electric Motorsports](https://github.com/olin-electric-motorsports) build chain.

### Getting Libraries
First run the setup.sh script to get all the defualt AVR libraries.
```
$ sudo bash setup.sh
```
Then you need to add an additional AVR library for the specific MCU we use (ATmega16m1)
```
$ bash getDir.sh
```

### Compiling and Flashing Boards
To flash your C code onto an ATmega, you need to compile that code into a certain type that the ATmega can read.
```
$ python3 make.py
```
Then just enter what board you want to flash (BREATHOFFRESHAIR) and select if you want to flash it or just compile it. Select to just compile it.
```
$ Flash (y/n) or Set Fuses(fuses):fuses
  n
```
Voila! Happy programming