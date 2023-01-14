// Rev counter repair for a Toyota - Removed old board and coil then replaced with
// x27-168 GM stepper motor with HW-310 L293D motor driver board
// using sparkfun pro micro ATmega32U4 5v, 16hz 
// 02-07-22
// Guy Carpenters X25 library
#include <SwitecX25.h>

// standard X25.168 & X27.168 range 315 degrees at 1/3 degree steps
#define STEPS (315*3) //totaling 945 steps to complete 315 degrees of rotation

// For motors connected to digital pins 4,5,6,7 (on X25 & X27 pin 1-> pin4, pin 2-> pin5, pin3 -> pin6, pin4 -> pin7)
SwitecX25 motor1(STEPS,4,5,6,7);

// digital pin 2 is an interupt pin
int RPM_PIN = 2;   // RPM input from vehicle
int led_pin = 9;   // Output shift light
int reset_pin = 8;  //rest poniter button

// Variables
volatile byte half_revolutions = 0;
unsigned int rpm = 0;
unsigned long timeold = 0;
static int nextPos = 0;

void setup(void)  /////// runs once on start ////////
{  
  // run the motor to max rpm and back to zero
  motor1.setPosition(720); // tell the motor to goto full scale
  motor1.currentStep = 9;  // this will cause overdrive at zero to ensure zero position (might not be needed and could damage some stepper motors)
  while (motor1.currentStep < 720 )digitalWrite(led_pin, HIGH),motor1.update(),delay(3); // drive to 8000rpm delay slows needle movement
  motor1.setPosition(0); // tell the motor to return to the zero position
  while (motor1.currentStep > 10)digitalWrite(led_pin, HIGH),motor1.update(),delay(3); // call update until it returns to zero & LED on as led test
  while (motor1.currentStep > 0)digitalWrite(led_pin, HIGH),motor1.update(),delay(50); // slow movement for last 10 steps
  
  pinMode(led_pin, OUTPUT);             //D9
  pinMode(RPM_PIN, INPUT_PULLUP);       //D2
  pinMode(reset_pin, INPUT_PULLUP);     //D8
}                                                //////////////// end of setup /////////////

void loop(void)
{
calcRPM();    //goto subroutine to calculate rpm
  
  int rpmmap = map(rpm, 0, 8000, 0, 680);// int scaled_value = map(rpm, act min val, act max val, min scale, max scale); map(rpm, 0, 8000, 0, 680)
  nextPos = rpmmap;
  if(digitalRead(reset_pin) == LOW) {motor1.currentStep = 9,motor1.setPosition(0);
    while (motor1.currentStep > 0)digitalWrite(led_pin, HIGH),motor1.update(),delay(50);//nextPos = nextPos -1,;
  }
  motor1.setPosition(nextPos); // call needle to rotate to given potition
  // the motor only moves when you call update. It will pull the value from the last motor1.setPosition()
  motor1.update();
} ////////////end of void loop /////

void RPMPulse() {
  half_revolutions++;   //Count rpm +6v pulses from car ecu IG pin (make votlage divider to drop 6v to 3v)
}
void calcRPM()  {
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), RPMPulse, RISING);
  if ((digitalRead(RPM_PIN) == LOW)&&(rpm>5500)) digitalWrite(led_pin, HIGH);
  else digitalWrite(led_pin, LOW);
  if (half_revolutions >= 15) {                                     // Update RPM every 15 counts, increase this for better RPM resolution but slower update.
    detachInterrupt(digitalPinToInterrupt(RPM_PIN));                // Disable interrupt when calculating
    rpm = (60000 / (millis() - timeold) * half_revolutions)/2;      // 60000
    if (rpm<600) rpm = 0;                                           // if rpm under idle speed, zero rev counter.
    timeold = millis();
    half_revolutions = 0;
  }
}