#include <genieArduino.h> //interact between 4D touch pad and arduino, https://github.com/4dsystems/ViSi-Genie-Arduino-Library
#include <AccelStepper.h> //control stepper motor, https://github.com/waspinator/AccelStepper
#include <PID_v1.h> //PID control heater, https://github.com/br3ttb/Arduino-PID-Library/
#include <SPI.h>
#include "Adafruit_MAX31855.h" 
#define MAXCS   10
Adafruit_MAX31855 thermocouple(MAXCS);

//Keyboard variables
char keyvalue[10]; // array to hold keyboard values
char keyvalue2[10]; // array to hold keyboard values
int temp; //temporary event variable from keyboard
int counter = 0;
int counter2 = 0;

//Syringe/stepper variables
bool RunStepper = false;
String StepperStatus = "not running";
float Withdraw = -1;

double BDdiameters[] = {4.78, 8.66, 12.06, 14.5, 19.3, 21.7}; //BD Plastipak Diameters, 1ml, 3ml, 5ml, 10ml, 20ml, 30ml.
String BDnames[] = { " 1 ml", " 3 ml", " 5 ml", "10 ml", "20 ml", "30 ml" };
double diameter = BDdiameters[4]; //standard is 20ml.
String syrname = BDnames[4]; //standard
// values from http://www.harvardapparatus.com/media/harvard/pdf/Syringe%20Selection%20Guide.pdf

// Define a stepper and the pins it will use (2 - Step, 3 - Direction)
AccelStepper stepper(AccelStepper::DRIVER, 2, 3);
float FlowValue = 0;
float speedvalue = 0;

//Temperature variables
bool RunHeater = 0;
float TempSetPoint = 37; //setpoint temperature (default 37);
int tempPin = 0; //the analog pin the TMP36's Voltage-out pin is connected to
int heaterPin = A5; //the analog pin the TIP 120 transistor is connected to
int x;
unsigned int tempReading = 0;
unsigned long tempReadingTotal = 0;
double Setpoint, Input, Output, temperatureC;

//Define the Tuning Parameters
double Kp = 150, Ki = 1, Kd = 0;

//Specify the links and tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Genie genie;
#define RESETLINE 4  // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)
void setup()
{
  Serial.begin(200000);  // Serial0 @ 200000 (200K) Baud
  genie.Begin(Serial);   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
  genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events

  // Reset the Display (change D4 to D2 if you have original 4D Arduino Adaptor)
  // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
  // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
  // the 4D Arduino Adaptors invert this signal so must be Active High.
  pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 1);  // Reset the Display via D4
  delay(100);
  digitalWrite(RESETLINE, 0);  // unReset the Display via D4

  delay (3500); //let the display start up after the reset (This is important)

  // Set the brightness/Contrast of the Display - (Not needed but illustrates how)
  // Most Displays, 1 = Display ON, 0 = Display OFF. See below for exceptions and for DIABLO16 displays.
  // For uLCD-43, uLCD-220RD, uLCD-70DT, and uLCD-35DT, use 0-15 for Brightness Control, where 0 = Display OFF, though to 15 = Max Brightness ON.
  genie.WriteContrast(15);
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, TempSetPoint * 10); // display default temp set point
  myPID.SetMode(AUTOMATIC); //set up PID mode
  myPID.SetTunings(Kp, Ki, Kd);

  genie.WriteStr(2, StepperStatus);
  genie.WriteStr(3, syrname);
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);
  stepper.setSpeed(speedvalue*Withdraw);
}

void loop()
{
  static long waitPeriod = millis();  //initialize waitperiod to do temperature measurement and PID computation

  genie.DoEvents(); // This calls the library each loop to process the queued responses from the display

  if (RunStepper == true)
  {
    stepper.runSpeed();
  }
  if (RunStepper == false)
  {
    stepper.stop();
    stepper.setSpeed(speedvalue*Withdraw);
  }

  //read temperature value and write to LED Digits
  if (millis() >= waitPeriod)
  {
    //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, speedvalue);
//    tempReadingTotal = 0;
//    // Read 64 readings
//    for (x = 0; x < 64; x++)
//    {
//      tempReadingTotal += analogRead(tempPin);
//    }
    //Divide by 64
//    tempReading = tempReadingTotal / 64.0;
//    double voltage = tempReading * 5000.0 / 1024.0;
    temperatureC = thermocouple.readCelsius();
    //temperatureC = (voltage - 500.0) / 10.0;
    //write measured value to LED Digits and scope
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, temperatureC*10);
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, temperatureC*10);
        genie.WriteObject(GENIE_OBJ_SCOPE, 0x00, (temperatureC-20)*2);  //scope goes from 0 to 80. However, scale is from 20 to 60.
        genie.WriteObject(GENIE_OBJ_SCOPE, 0x00, (TempSetPoint-20)*2);

    //PID and heater control
    if (RunHeater == true)
    {
      Input = temperatureC;
      Setpoint = TempSetPoint;
      myPID.Compute();
      analogWrite(heaterPin, Output);
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, Output/255*100);
    }
    else
    {
      analogWrite(heaterPin, 0);
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, 0);
    }
    waitPeriod = millis() + 1000; // rerun this code after 5000 ms
  }
}

/////////////////////////////////////////////////////////////////////
//
// This is the user's event handler. It is called by genieDoEvents()
// when the following conditions are true
//
//		The link is in an IDLE state, and
//		There is an event to handle
//
// The event can be either a REPORT_EVENT frame sent asynchronously
// from the display or a REPORT_OBJ frame sent by the display in
// response to a READ_OBJ (genie.ReadObject) request.
//

void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event);

  //Filter Events from Keyboard0 (Index = 0) for setting Temperature
  //genie.EventIs usage: genie.EventIs(&Event,GENIE_REPORT_XX,GENIE_OBJ_XX,index of object).
  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_KEYBOARD, 0))
  {
    temp = genie.GetEventData(&Event);  // Receive the event data from the Keyboard
    if (temp >= 48 && temp <= 57 && counter <= 5)
    {
      keyvalue[counter] = temp;
      genie.WriteStr(1, keyvalue); //prints to String Object
      counter = counter + 1; //increment array
    }
    else if (temp == 110) // . key
    {
      keyvalue[counter] = '.';        // Receive the event data from the keyboard
      genie.WriteStr(1, keyvalue); //prints to String Object
      counter = counter + 1; //increment array
    }
    else if (temp == 8)      // backspace
    {
      counter--;
      keyvalue[counter] = " ";
      genie.WriteStr(1, keyvalue); //prints to String Object
    }
    else if (temp == 13) //enter key = 13
    {
      //genie.WriteStr(0,keyvalue);
      TempSetPoint = atof(keyvalue);
      genie.WriteObject(GENIE_OBJ_FORM, 0x00, 0); //switch back to form 0
      genie.WriteStr(2, StepperStatus);
      genie.WriteStr(3, syrname);
      for (int x = 0; x < counter ; x++)
      {
        keyvalue[x] = 0;
      }
      genie.WriteStr(1, keyvalue);
      counter = 0;
    }
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, TempSetPoint * 10);   // Write Keyboard0 value to to LED Digits (temperature set point)
  }

  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_WINBUTTON, 1)) //back to main WINBUTTON
  {
    genie.WriteObject(GENIE_OBJ_FORM, 0x00, 0); //switch back to form 0
    genie.WriteStr(2, StepperStatus);
    genie.WriteStr(3, syrname);
  }

  //Filter Events from Keyboard0 (Index = 0) for setting Flow Rate
  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_KEYBOARD, 1))
  {
    temp = genie.GetEventData(&Event);  // Receive the event data from the Keyboard
    if (temp >= 48 && temp <= 57 && counter2 <= 5)
    {
      keyvalue2[counter2] = temp;
      genie.WriteStr(0, keyvalue2); //prints to String Object
      counter2 = counter2 + 1; //increment array
    }
    else if (temp == 110) // . key
    {
      keyvalue2[counter2] = '.';        // Receive the event data from the keyboard
      genie.WriteStr(0, keyvalue2); //prints to String Object
      counter2 = counter2 + 1; //increment array
    }
    else if (temp == 8)      // backspace
    {
      counter2--;
      keyvalue2[counter2] = " ";
      genie.WriteStr(0, keyvalue2); //prints to String Object
    }
    else if (temp == 13) //enter key = 13
    {
      //genie.WriteStr(0,keyvalue2);
      FlowValue = atof(keyvalue2);
      genie.WriteObject(GENIE_OBJ_FORM, 0x00, 0); //switch back to form 0
      genie.WriteStr(2, StepperStatus);
      genie.WriteStr(3, syrname);
      for (int x = 0; x < counter2 ; x++)
      {
        keyvalue2[x] = 0;
      }
      genie.WriteStr(0, keyvalue2);
      counter2 = 0;
    }
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, FlowValue * 100);   // Write Keyboard1 value to to LED Digits (flow rate)
    //recalculate speedvalue from FlowValue and diameter
    speedvalue = FlowValue / (60 * 3.14159 * diameter * diameter * 0.001 / 4 * 0.8) * 3200;
    stepper.setSpeed(speedvalue*Withdraw);

  }
  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_WINBUTTON, 3)) //back to main WINBUTTON
  {
    genie.WriteObject(GENIE_OBJ_FORM, 0x00, 0); //switch back to form 0
    genie.WriteStr(2, StepperStatus);
    genie.WriteStr(3, syrname);
  }

  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_WINBUTTON, 5)) //back to main WINBUTTON
  {
    genie.WriteObject(GENIE_OBJ_FORM, 0x00, 0); //switch back to form 0
    genie.WriteStr(2, StepperStatus);
    genie.WriteStr(3, syrname);
  }

  //Filter Events from UserButton0 (Index = 0) for starting stepper motor
  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_USERBUTTON, 0))
  {
    RunStepper = genie.GetEventData(&Event);
    if (RunStepper == true)
    {
      StepperStatus = "  running  ";
      genie.WriteStr(2, StepperStatus);
    }
    else
    {
      StepperStatus = "not running";
      genie.WriteStr(2, StepperStatus);
    }
  }

  //Filter Events from 4DButton0 (Index = 0) for starting heater
  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_4DBUTTON, 0))
  {
    RunHeater = genie.GetEventData(&Event);
    genie.WriteObject(GENIE_OBJ_4DBUTTON, 2, RunHeater);
  }
  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_4DBUTTON, 2))
  {
    RunHeater = genie.GetEventData(&Event);
    genie.WriteObject(GENIE_OBJ_4DBUTTON, 0, RunHeater);
  }

  //Filter Events from 4DButton1 (Index = 1) for checking withdraw/infuse
  if (genie.EventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_4DBUTTON, 1))
  {
    Withdraw = genie.GetEventData(&Event); //is either 0 or 1
    Withdraw = 2*Withdraw-1;  //convert Withdraw to either -1 or +1
    stepper.setSpeed(speedvalue*Withdraw);
  }

  //Filter Events for choosing syringe type (Form 4, Winbutton 7-12)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)                // If the Reported Message was from a Winbutton
    {
      if (Event.reportObject.index >= 7 && Event.reportObject.index <= 12) // If WINBUTTON index between 7 and 12
      {
        diameter = BDdiameters[Event.reportObject.index - 7];
        syrname = BDnames[Event.reportObject.index - 7];
        genie.WriteObject(GENIE_OBJ_FORM, 0x00, 0); //switch back to form 0
        genie.WriteStr(2, StepperStatus);
        genie.WriteStr(3, syrname);
        //recalculate speedvalue from FlowValue and diameter
        // slope for M5 threaded rod: 0.8mm per revolution
        speedvalue = FlowValue / (60 * 3.14159 * diameter * diameter * 0.001/ 4 * 0.8) * 3200;
        stepper.setSpeed(speedvalue*Withdraw);
      }
    }
  }
}
