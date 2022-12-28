# ESP32-VCU
A VCU design to run control processes in an EV conversion.

Like i showed i developed a V1 of ESP32 VCU design. I shuffled outputs a bit so i cleared the dubious pins. I tested it and it looks good.
My goal is to develop the code for this board which would be able to run Chademo process a BMS or use it as VCU with option for wifi interface.

I managed to get the CAN bus code working!
I have the I/Os working as well, though not in the way as i intended. I cant get the opto coupler chip working from 12V inputs. I will simply replace that chip by 4 resistors and reverse the sensing. This will pull the signal lines to GND and effect signal change in ESP32 chip.

There would be one output on this VCU dedicated to HMI BMS warning - a buzzer. 

Also since there is a single CAN and it needs to run at 500kbaud i can only use it for one CAN speed.... So i intend to use this VCU to actually control the charger and other systems like AC or heater. I really like Outlander way of single CAN telegram that works as a heartbeat. Remove that and all processes cease. So in my instance i intend to use a simple pulldown pin to command 0x285 CAN telegram to stop transmitting. That will stop chargers as well as running gear, but keep contactors alive.
Or i can do it the FAILSAFE way; to command the heartbeat by that active BMS signal. If you remove the signal everything fails safe.... Havent decided yet.

I have already implemented PP and CP signalling. Board also has PWM duty sensing pin which can be used to correct power as required from the EVSE.

This version of VCU actually could also be used to run Leaf or Outlander inverter. It has the analog pins and correct inputs and outputs as well. So practicaly to run the car, L2 charger, BMS and chademo one would need like 3 or 4 of theese VCUs.
