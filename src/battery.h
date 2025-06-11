#ifndef BATTERY_H_
#define BATTERY_H_

#include "Arduino.h"
#include <Wire.h>
#include "ina266_m.h"

constexpr float shunt_v_drop = 0.075;        // Shunt v drop at max current
constexpr float shunt_max_amps = 100.0;      // shunt max current
constexpr double maxCapacity = 95.0;     // battery capacity in amp hours:
constexpr float chargeEfficiency = .95;  // Lipo4 charge efficiency - amount of charge actually stored


class Battery
{
    public:   
        bool setUpINA266(const int address, const float busCorrFactor);
        void restoreCharge(double new_charge);
        bool isLogRequired();
        void chargeLogged();
        void updateCharge(double lapsedHrs);
        bool logRequired();
        void normaliseCharge();
        float getVoltage();
        float getCurrent();
 
        double charge; 

     protected:
        INA226_M ina226;
        double lastLoggedCharge;
        float current;
        float voltage;
 };
 

#endif
 