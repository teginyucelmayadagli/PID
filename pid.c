#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
//PID parametreleri
#define MIN_ACTUATOR_VALUE 200 //PID sonucunu gercekleyecek birimin alacagi min deger
#define MAX_ACTUATOR_VALUE 1000 //PID sonucunu gercekleyecek birimin alacagi max deger
#define MIN_SETPOINT_VALUE 0 //ayar noktasinin alacagi min deger
#define MAX_SETPOINT_VALUE 255 //ayar noktasinin alacagi max deger
#define MIN_FEEDBACK_VALUE 0 //geribeslemenin alacagi min deger
#define MAX_FEEDBACK_VALUE 255 //geribeslemenin alacagi max deger
#define PID_SCALE_LOWER_LIMIT -3.0 //PID sonucunun alacagi min deger. PID limitlerine gore bu deger degisebilir
#define PID_SCALE_UPPER_LIMIT +3.0 //PID sonucunun alacagi max deger. PID limitlerine gore bu deger degisebilir
#define SET_POINT 85 //ayar noktasi
 
void vPID(uint16 wInput); //her timer kesmesinde bu fonksiyonu cagirin. wInput degiskeni geribeslemeden aldiginiz degerdir.
static float fNormalizeFeedback(uint16 wValue);
static void vSetActuator(uint16 wToActuator); //PID sonucunu gerceklemek icin bu fonksiyonu cagirin
 
//Asagidaki fonksiyonlarda degisiklik yapmayin
static float fNormalizeSetPoint(uint16 wValue);
static float fNormalize(uint16 wValue, float fPIDLowerLimit, float fPIDUpperLimit, uint16 wInputMin, uint16 wInputMax);
static uint16 wPlantProcess(float fInput, float fPIDLowerLimit, float fPIDUpperLimit, uint16 wInputMin, uint16 wInputMax);
 
void vPID(uint16 wInput)
{
float fError;
static float fLastError = 0;
uint16 wToActuator;
 
float fSetPoint = fNormalizeSetPoint(SET_POINT);
float fInput = fNormalizeFeedback(wInput);
float fPIDOutput;
 
float fProportional;
float fDerivative ;
static float fIntegral;
//PID kazanc ve limitleri. Debug modunda belirlediginiz kazanc ve limit degerlerini,
//programin son halinde sabit degiskeler olarak degistirebilirsiniz.
static float fGainProportional = 1; //P kazanc
static float fLimitProportional = 1; //P limit
static float fGainIntegral = 0; //I kazanc
static float fLimitIntegral = 1; //I limit
static float fGainDifferential = 0; //D kazanc
static float fLimitDerivative = 1; //D limit
 
//Error
fError = fSetPoint - fInput;
 
//Proportional
fProportional = fError * fGainProportional;
if (fProportional > fLimitProportional) fProportional = fLimitProportional;
else if (fProportional < -fLimitProportional) fProportional = -fLimitProportional;
//Integral
fIntegral += fError * fGainIntegral;
if (fIntegral > fLimitIntegral) fIntegral = fLimitIntegral;
else if (fIntegral < -fLimitIntegral) fIntegral = -fLimitIntegral;
//Derivative
fDerivative = (fError - fLastError) * fGainDifferential; fLastError = fError;
if (fDerivative > fLimitDerivative ) fDerivative = fLimitDerivative ;
else if (fDerivative < -fLimitDerivative ) fDerivative = -fLimitDerivative ;
 
fPIDOutput = fProportional + fIntegral + fDerivative ;
 
if (fPIDOutput > PID_SCALE_UPPER_LIMIT) fPIDOutput = PID_SCALE_UPPER_LIMIT;
else if (fPIDOutput < PID_SCALE_LOWER_LIMIT) fPIDOutput = PID_SCALE_LOWER_LIMIT;
 
wToActuator = wPlantProcess(fPIDOutput, PID_SCALE_LOWER_LIMIT, PID_SCALE_UPPER_LIMIT, MIN_ACTUATOR_VALUE, MAX_ACTUATOR_VALUE);
 
vSetActuator(wToActuator);
 
//Calisma aninda PID degerlerini gozlemlemek icin asagidaki satiri acabilirsiniz
//printf("Set:%0.3f fError:%0.3f Act:%d P:%0.3f I:%0.3f D:%0.3f \n\r", fSetPoint, fError, wToActuator, fProportional, fIntegral, fDerivative);
 
}
 
//Bu fonksiyonu PID sonucunu gerceklemek icin kullanin
static void vSetActuator(uint16 wToActuator)
{
//bu fonksiyonu PID sonucu ile etkilemek istediginiz
//register i guncelleyin. mesela bir PWM modulunun darbe bosluk oranini
//ayarlayan bir register gibi
}
 
//Bu satirin altinda kalanlari degistirmeyin
static float fNormalize(uint16 wValue, float fPIDLowerLimit, float fPIDUpperLimit, uint16 wInputMin, uint16 wInputMax)
{
//function normalizes the input to PID limits
//y = ax + b
float a = (fPIDUpperLimit - fPIDLowerLimit)/(wInputMax - wInputMin);
float b = (fPIDUpperLimit + fPIDLowerLimit - a*(wInputMax + wInputMin))/2.0;
 
return a*wValue + b;
}
 
static float fNormalizeSetPoint(uint16 wValue)
{
return fNormalize(wValue, PID_SCALE_LOWER_LIMIT, PID_SCALE_UPPER_LIMIT, MIN_SETPOINT_VALUE, MAX_SETPOINT_VALUE);
}
 
static float fNormalizeFeedback(uint16 wValue)
{
return fNormalize(wValue, PID_SCALE_LOWER_LIMIT, PID_SCALE_UPPER_LIMIT, MIN_FEEDBACK_VALUE, MAX_FEEDBACK_VALUE);
}
 
static uint16 wPlantProcess(float fInput, float fPIDLowerLimit, float fPIDUpperLimit, uint16 wInputMin, uint16 wInputMax)
{
//function returns the real output
//x = (y - b)/a
float a = (fPIDUpperLimit - fPIDLowerLimit)/(wInputMax - wInputMin);
float b = (fPIDUpperLimit + fPIDLowerLimit - a*(wInputMax + wInputMin))/2.0;
 
return (uint16)((fInput - b)/a);
}