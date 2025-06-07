#include "battery.h"

bool  Battery::setUpINA266(const int address){
    ina226 = INA226_M(address);
    if(!ina226.init()){
        return 0;
    }
    ina226.setConversionTime(INA226_CONV_TIME_4156);
    ina226.setAverage(INA226_AVERAGE_512);
    ina226.setShuntValues(shunt_v_drop, shunt_max_amps);
    return 1;
}

void Battery::restoreCharge(double new_charge){
    charge = new_charge;
    normaliseCharge();
    lastLoggedCharge = charge;
}

bool Battery::isLogRequired(){
    return lastLoggedCharge == charge;

}

void Battery::chargeLogged(){
    lastLoggedCharge = charge;
}

 void Battery::updateCharge(double lapsedHrs){
    current = ina226.getCurrent();
    if(current > 0){
        charge += ((double) current * chargeEfficiency * lapsedHrs);
    } else {
        charge += ((double) current * lapsedHrs);
    }
    normaliseCharge();
 }
        
bool Battery::logRequired(){
    return fabs(charge - lastLoggedCharge) > 0.49;
}

void Battery::normaliseCharge(){
   if (charge > maxCapacity){
        charge = maxCapacity;
   }
   if (charge < 0.1){
      charge = 0.1;
   }
}



