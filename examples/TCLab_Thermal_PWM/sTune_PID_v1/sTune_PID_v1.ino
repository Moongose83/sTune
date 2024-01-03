/********************************************************************
  sTune PID_v1 Example
  This sketch does on-the-fly tunning and PID Digital Output control
  of a thermal heater (TIP31C). Tunning parameters are quickly
  determined and applied during the temperature ramp-up to setpoint.
  Open the serial plotter to view the graphical results.
  *****************************************************************/
#include <sTune.h>
#include <PID_v2.h>

// pins
#define VOLTAGE_PIN A0
#define R1 10000  // 26000  //3900 //30000 //33000 //36000 //39000
#define R2 3300   // 10000 //1500 //10000 //11000 //12000 //13000
#define CORRECTION 0.4
#define ERROR_TRS 0.5
#define VOLTAGE_TARGET 14.4

// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 500;  // runPid interval = testTimeSec / samples
const uint16_t samples = 500;
const float inputSpan = 150;
const float outputSpan = 1000;
float outputStart = 0;
float outputStep = 300;
uint8_t debounce = 1;

// variables
double input, output, kp, ki, kd; // PID_v2
float Input, Output, Kp, Ki, Kd; // sTune

double setpoint = VOLTAGE_TARGET;
sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);
PID myPID(&input, &output, &setpoint, 0, 0, 0, P_ON_M, DIRECT);

void setup() {
  //analogReference(EXTERNAL); // 3.3V
  //pinMode(relayPin, OUTPUT);
  Serial.begin(9600);
  while (!Serial) delay(10);
  delay(3000);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(VOLTAGE_TARGET);
}

void loop() {
  tuner.softPwm(relayPin, Input, Output, Setpoint, outputSpan, debounce);

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      voltage = analogRead(VOLTAGE_PIN) * (5.0 / 1023.0) * ((R1 + R2) / R2);
      input = voltage - CORRECTION;
      tuner.plotter(Input, Output, Setpoint, 0.1f, 3); // output scale 0.1, plot every 3rd sample
      break;

    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.SetOutputLimits(0, outputSpan);
      myPID.SetSampleTime(outputSpan - 1);
      output = outputStep, kp = Kp, ki = Ki, kd = Kd;
      myPID.SetMode(AUTOMATIC); // the PID is turned on
      myPID.SetTunings(kp, ki, kd); // update PID with the new tunings
      break;

    case tuner.runPid: // active once per sample after tunings
      voltage = analogRead(VOLTAGE_PIN) * (5.0 / 1023.0) * ((R1 + R2) / R2);
      input = voltage - CORRECTION;
      myPID.Compute();
      Input = input, Output = output;
      tuner.plotter(Input, Output, Setpoint, 0.1f, 3);
      int powerPercentage = map(output, 0, 255, 0, 100);
      printPWM(powerPercentage);
      break;
  }
  
}

void printPWM(int pwm) { Serial.print("D0" + String(pwm)); }
