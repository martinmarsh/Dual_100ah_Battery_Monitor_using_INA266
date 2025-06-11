#include "battery.h"

bool  Battery::setUpINA266(const int address, const float busCorrFactor){
    ina226 = INA226_M(address);
    if(!ina226.init()){
        return 0;
    }
    ina226.busVoltageCorrFactor = busCorrFactor;
    ina226.setConversionTime(INA226_CONV_TIME_4156);
    ina226.setAverage(INA226_AVERAGE_512);
    ina226.setShuntValues(shunt_v_drop, shunt_max_amps);
    return 1;
}

void Battery::restoreCharge(double new_charge){
    int est_percent = 0;
    double est_charge = 0;
    
    charge = new_charge;
    lastLoggedCharge = charge;
    // When restoring correct charge based on Voltage as there may have been unmeasured discharge
    // assume measured is most accurate unless too great a difference 
    voltage = ina226.getBusVoltage();
    if (voltage > 13.45){
        est_percent = 100;
    } else if (voltage >= 13.37){
        est_percent = 90;
    } else if (voltage >= 13.35){
        est_percent = 80;
    } else if (voltage >= 13.33){
        est_percent = 70;
     } else if (voltage >= 13.28){
        est_percent = 60;
     } else if (voltage >= 13.23){
        est_percent = 50;
     } else if (voltage >= 13.22){
        est_percent = 40;
     } else if (voltage >= 13.21){
        est_percent = 30;
     } else if (voltage >= 13.06){
        est_percent = 20;
     } else if (voltage >= 12.85){
        est_percent = 10;
    }
    est_charge = maxCapacity * double(est_percent)/100.0;
    Serial.print("Restored Charge: ");
    Serial.println(charge); 

    Serial.print("Estimated Charge based on Votage: "); 
    Serial.print(voltage);
    Serial.print(" is ");
    Serial.println(est_charge);

    int measured_percent = charge/maxCapacity * 100.0;
    if (est_percent - measured_percent > 20){
        charge = est_charge;
        Serial.println("Upading charge based higher voltage"); 
    } else if (measured_percent - est_percent > 35.0){
        charge = est_charge;
        Serial.println("Upading charge based lower voltage");   
    }
 
    normaliseCharge();
  
}

bool Battery::isLogRequired(){
    return lastLoggedCharge != charge;

}

void Battery::chargeLogged(){
    lastLoggedCharge = charge;
}

 void Battery::updateCharge(double lapsedHrs){
    current = ina226.getCurrent();
    voltage = ina226.getBusVoltage();
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

float Battery::getVoltage(){
    return voltage;
}

float Battery::getCurrent(){
    return current;
}

