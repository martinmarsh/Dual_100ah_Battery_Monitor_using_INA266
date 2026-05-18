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

void Battery::restoreCharge(double new_charge, float voltage){
    int est_percent = 0;
    double est_charge = 0;
    
    charge = new_charge;
    lastLoggedCharge = charge;
    // When restoring correct charge based on Voltage as there may have been unmeasured discharge
    // assume measured is most accurate unless too great a difference 
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
    } else if (voltage >= 13.24){
        est_percent = 56;
    } else if (voltage >= 13.20){
        est_percent = 50;
    } else if (voltage >= 13.15){
        est_percent = 46;
    } else if (voltage >= 13.10){
        est_percent = 40;
    } else if (voltage >= 13.05){
        est_percent = 36;
    } else if (voltage >= 13.00){
        est_percent = 30;
    } else if (voltage >= 12.90){
        est_percent = 20;
    } else if (voltage >= 12.80){
        est_percent = 17;
    } else if (voltage >= 12.50){
        est_percent = 14;
    } else if (voltage >= 12.00){
        est_percent = 9;
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
    update();
    if(current > 0.1){
        charge += ((double) current * chargeEfficiency * lapsedHrs);
    } else if (current < -0.3){
        charge += ((double) current * lapsedHrs);
    } else {
        charge -= 0.03 * lapsedHrs;
    }
    normaliseCharge();
 }

 void Battery::update(){
    current = ina226.getCurrent();
    voltage = ina226.getBusVoltage();
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

