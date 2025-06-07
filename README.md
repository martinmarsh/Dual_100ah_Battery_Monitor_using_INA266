Work in progress of changing to use  INA226_M class based on
a modified library for the  INA226 Current and Power
Sensor Module originally Written by Wolfgang (Wolle) Ewald
 
This code is intended to run on an arduino UNO 3 connected to two
using two INA226 chips each connected to an external 100A 75mv shunts.

This is part of a project to construct a 2400 KW 13v power bank using
two 100 Ahr LIFePO4 batteries and a 3 KW pure sin wave inverter for
240v loads.

You are welcome to copy the code and use it for any purpose entirely at
your own risk.  A power bank of this size can deliver a dangerous amount
of power even if the batteries BMS is preventing overdischarge. Also
incorrect wiring could result in damage to the batteries and a hard to
control and life threatening fire.

It is recommeded/cautioned by battery manufacturers to only connect in parrallel
batteries from the same make, model and batch or purchased at same time.
It is not just sufficient to rely on them having the same chemistry. This
is because different cell contructions and BMS designs alter the internal
resitance and other parmeters may not be identical.  In extreme cases
mismatched batteries may cause overloading, over discharge and undetected
BMS protection.  Relying on the BMS for protection on a regular basis is
a safety concern as this is not an intended use of the BMS.

Of course it should be possible to use different batteries providing
they are matched, the currents from each battery carefuly monitored,
loads and enviromental factors kept within limits. For example never loading
the batteries beyond the current limit of one battery and not assuming
that twice the single max load can be applied to the pair.  This is not just
a theorical issue; for example I tested 2 makes of seemingly identical
batteries fully charged and left connected in parrallel to ensure balanced.
When a 88A load was applied and one battery supplied 54A and the other at 34A.
When the load was removed a balancing current flowed from the one discharged
at 34A to the other battery.  The issue was probably due to differences in
BMS of only a few tens of milli ohms resistance.  Heavy duty wiring ensured
connections were in the order of milli-ohms and could not have caused the
imbalance.

This code can be used to check the battery balance and to monitor capacity
of each battery as it is charged and discharged.  Currently the code does
not work a display and prints out the various parameters. This is because on
the pack I built I have meters connected to the shunts also used by the INA266
chips and I have an external USB monitoring port.  The meters monitor
current load but do not monitor capacity during charge and discharge.
However, the voltage monitor on one meters has a bar graph so I have
connected the votage monitor pin to the UNO 3's PWM pin so that it can display
the calculated percentage discharge of both batteries.

