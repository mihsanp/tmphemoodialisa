#include <IntervalTimer.h>

//interrupt
volatile int counter = 0; // Variabel counter
//sensor pin
int sensor1 = A3;
int sensor2 = A2;
int sensor3 = A1;
int sensor4 = A0;
int inputPin = 18;//flow
//aktuator pin
//stepper
const int stepPin = 3; 
const int dirPin = 2; 
const unsigned long interval = 50; //65
//Pump
const int pwmPump_pin = 6;
const int RightPump_pin = 7;
//valve
int v1 = 22;
int v2 = 23;
int v3 = 21;
//valve
const int pwmVR= 10;
const int pwmVL = 9;
const int enVR = 11;
const int enVL = 8;

//data
//pressure
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;
float DialInPressure = 0.0;
float DialOutPressure = 0.0;
float ArteriPressure = 0.0;
float VenaPressure = 0.0;
// Define the filter variables
float cutoffFrequency = 1;  // Initial cutoff frequency in Hz
float RC = 1.0 / (2.0 * PI * cutoffFrequency);
float dt = 0.00833;  // Sample time in seconds
float alpha = dt / (RC + dt);

// Define the filtered sensor value
float filteredValue = 0.0;
float Pressure;

//flowsensor
volatile unsigned long previousPulseTime = 0;  // Time of previous pulse event
double pulseFrequency;
double flow;
float actual = 0.0;
char bp[10];
char currentchar;
const int movingAverageSize = 100 ;  // Adjust the size of the moving average window as needed
float flowValues[movingAverageSize];
int flowIndex = 0;
float flowSum = 0.0;
float averageFlow = 0.0;
unsigned long averageStartTime = 0;

//valve
float angle;
int pulsesPerRevolution = 833; 

//pengiriman data
const int sample_rate = 5;
int num1 = 0;
int num2 = 0;
int num3 =0;
int num4 =0;
int num5 = 0;
int num6 = 0;
int num7;
int num8;
int pump_statuts;
int valve_status;
int kalibrasi_status;
int stepper_status;
int flag;

double Kp = 0.92;   // Proportional gain
double Ki = 0.000399;   // Integral gain
double Kd = 0.98;   // Derivative gain

double integral = 0.0;        // Integral term
double previousError = 0.0;   // Previous error for derivative term
int setpoint = 500;
double error;

//PID pressure
// PID control parameters for Valve
double Kp_valve = 0.85;   // Proportional gain
double Ki_valve = 0.005;   // Integral gain
double Kd_valve = 0.21;   // Derivative gain
double integral_valve = 0.0;        // Integral term
double previousError_valve = 0.0;   // Previous error for derivative term
int setpoint_valve = -50; // set as per requirement
double error_valve;
double output_valve;

float tmp = 0.0;
float tmpset;
float tmpsetpoint;

float flowhanning;
const int hanningFilterSize = 21;
float hanningCoefficients[hanningFilterSize];


// Rest of the code remains unchanged


IntervalTimer timer;
IntervalTimer timer1;
IntervalTimer timer2;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(pwmPump_pin,OUTPUT);
  pinMode(RightPump_pin,OUTPUT);
  pinMode(v1, OUTPUT);
  pinMode(v2, OUTPUT);
  pinMode(v3, OUTPUT);
  pinMode(pwmVR, OUTPUT);
  pinMode(pwmVL, OUTPUT);
  pinMode(enVR, OUTPUT);
  pinMode(enVL, OUTPUT);
  pinMode(24, INPUT); // Channel A
  pinMode(12, INPUT); // Channel B
  digitalWrite(enVL, HIGH);
  digitalWrite(enVR, HIGH);
  attachInterrupt(digitalPinToInterrupt(24), A_Rise, RISING); // Attach interrupt
  analogReadResolution(12);
  analogWriteResolution(12);
  timer.begin(sendData, 1000000/sample_rate); //25Hz
  timer1.begin(updateSignal, 1000);
  timer2.begin(pump, 1000);
  // timer3.begin(valve, 1000);
  // timer4.begin(bloodpump, 1);
  digitalWrite(RightPump_pin, HIGH);
  pinMode(inputPin, INPUT);  // Set the input pin as INPUT_PULLUP
  attachInterrupt(digitalPinToInterrupt(inputPin), pulseInterrupt, FALLING); 
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; 
  averageStartTime = millis();
    for (int i = 0; i < hanningFilterSize; i++) {
    hanningCoefficients[i] = 0.5 * (1.0 - cos(2.0 * PI * i / (hanningFilterSize - 1)));
  }


}

void loop() {
  // put your main code here, to run repeatedly:
      // Calculate the moving average
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - averageStartTime;
  applyHanningFilter(flow);
  if (elapsedTime >= 500) {  // Adjust the time interval as needed (0.5 seconds in this case)
    averageStartTime = currentTime;

    flowSum -= flowValues[flowIndex];
    flowValues[flowIndex] = flow;
    flowSum += flowValues[flowIndex];

    flowIndex = (flowIndex + 1) % movingAverageSize;
    averageFlow = flowSum / movingAverageSize;
  }
  
  if (Serial.available()){
    int index = 0;
    while (Serial.available() && index < 9) { 
     currentchar = Serial.read();
      if (currentchar == '\n') {
        break; // Break if newline character is encountered
      }
      bp[index] = currentchar;
      index++;
    }
      bp[index] = '\0';
  }
  if (bp[0]=='a'){
    num3 = atoi(&bp[1]);
  }else 
  if(bp[0]=='b'){
    num4 = atoi(&bp[1]);
  }else
  if(bp[0]=='1'){
    num2 = 1;
  }else
  if(bp[0]=='0'){
    num2 = 0;
  }else
  if (bp[0]=='2'){
    num1 = 1;
  }else
  if (bp[0]=='3'){
    num1 = 0;
  }else
  if (bp[0] == 'n'){
    num8 = 1;
  } else 
  if(bp[0] == 's'){
    num8 = 0;
  }

  tmpset = num4*1.0;
  //aktual pump
  pump_statuts=1*num1;
  actual = num1*500;

  //bloodpump
  stepper_status = 1*num2;

  //valve
  valve_status = 1*num7;
  //kalibrasi valve

  kalibrasi_status = 1*num8;

  //
  // bloodpump();
  valve();
  

}

//interrupt
void A_Rise() {
  if(digitalRead(12)) {
    counter++; // Jika Channel B HIGH, motor bergerak ke depan
  }
  else {
    counter--; // Jika Channel B LOW, motor bergerak ke belakang
  }

}

void pulseInterrupt() {
  unsigned long currentMillis = ARM_DWT_CYCCNT;
  unsigned long pulsePeriod = currentMillis - previousPulseTime;
  pulseFrequency = 1.11 * 1000000000.0 / pulsePeriod; // Frequency in Hz (assuming pulsePeriod is in milliseconds)//1.19999
  flow = (pulseFrequency * 20.49362439);
  float flowh = flow;  
  previousPulseTime = currentMillis;

}

// const int hanningFilterSize = 5; // Adjust the filter size as needed
// float hanningCoefficients[hanningFilterSize] = {0.25, 0.5, 0.25, 0.5, 0.25}; // Hanning window coefficients



void applyHanningFilter(float newFlowValue) {
  static float hanningBuffer[hanningFilterSize];
  static int hanningIndex = 0;

  hanningBuffer[hanningIndex] = newFlowValue;
  hanningIndex = (hanningIndex + 1) % hanningFilterSize;

  float filteredValue = 0.0;
  for (int i = 0; i < hanningFilterSize; i++) {
    filteredValue += hanningBuffer[(hanningIndex + i) % hanningFilterSize] * hanningCoefficients[i];
  }

  flowhanning = filteredValue *0.1;
}


void sendData() {
  float flowmlToSend = averageFlow;
  float flowml = averageFlow;
  angle = (float)counter / pulsesPerRevolution * 360.0;
  byte data[28];  // Increase the size to accommodate the additional data
  memcpy(&data[0], &DialInPressure, sizeof(DialInPressure));
  memcpy(&data[4], &DialOutPressure, sizeof(DialOutPressure));
  memcpy(&data[8], &ArteriPressure, sizeof(ArteriPressure));
  memcpy(&data[12], &VenaPressure, sizeof(VenaPressure));
  memcpy(&data[16], &actual, sizeof(actual));
  memcpy(&data[20], &flowmlToSend, sizeof(flowmlToSend));
  memcpy(&data[24], &tmp, sizeof(tmp));
  Serial.write(data, sizeof(data));
}

void updateSignal(){
  sensorValue1 = analogRead(sensor1);
  sensorValue2 = analogRead(sensor2);
  sensorValue3 = analogRead(sensor3);
  sensorValue4 = analogRead(sensor4);
  float PressureDialIn = ((sensorValue1*(3.3/4096)-1.66)/0.0163934426)*7.50061683;
  float PressureDialOut = ((sensorValue2*(3.3/4096)-1.66)/0.0163934426)*7.50061683;
  float PressureArteri = ((sensorValue3*(3.3/4096)-1.66)/0.0163934426)*7.50061683;
  float PressureVena = ((sensorValue4*(3.3/4096)-1.66)/0.0163934426)*7.50061683;
  //fix
  DialInPressure= alpha * PressureDialIn + (1 - alpha) * DialInPressure;
  DialOutPressure= alpha * PressureDialOut + (1 - alpha) * DialOutPressure;
  ArteriPressure= alpha * PressureArteri + (1 - alpha) * ArteriPressure;
  VenaPressure= alpha * PressureVena + (1 - alpha) * VenaPressure;
  tmp=((ArteriPressure + VenaPressure)/2)-((DialInPressure+DialOutPressure)/2);

  tmpsetpoint = ((ArteriPressure+VenaPressure - DialInPressure - (2*tmpset)));
  bloodpump();
}

void pump(){
    if(pump_statuts==1){
      error = 500.0 - (double)averageFlow;

      // Perform PID calculations
      double output = Kp * error;
      integral += Ki * error;
      double derivative = Kd * (error - previousError)/0.1;
      output += integral + derivative;

      // Update previous error
      previousError = error;
      setpoint = num3;
      output = constrain(output, 0, 1500);
      analogWrite(pwmPump_pin, 830);
      digitalWrite(v2, HIGH);
      digitalWrite(v1, LOW);
      digitalWrite(v3, LOW);

  // digitalWrite(v2, LOW);
  // digitalWrite(v1, HIGH);
  // digitalWrite(v3, HIGH); 
  }else 
  if(pump_statuts==0){
      analogWrite(pwmPump_pin, 0);
      flow = 0.0;
      // flowhanning = 0;
      error = 0.0;
  }
}

void valve(){
  // if(kalibrasi_status==1){
  //   analogWrite(pwmVL,1500);
  //   analogWrite(pwmVR,0);
  // }else 
  // if(kalibrasi_status==0){
  //   analogWrite(pwmVL, 0);
  //   analogWrite(pwmVR, 0);
  // }else
if(kalibrasi_status == 1){
  pidValve();
  // Jika error_valve <= 0, pwmVL menyala dan pwmVR mati
  if(error_valve <= 0 || angle>=0){
    analogWrite(pwmVL, output_valve);
    analogWrite(pwmVR, 0);
    // Memeriksa jika angle mencapai 0 derajat

  }
  // Jika error_valve > 0, pwmVR menyala dan pwmVL mati
  if(error_valve > 0 || angle<=7*360){
    analogWrite(pwmVR, output_valve);
    analogWrite(pwmVL, 0);

    // Memeriksa jika angle mencapai 7 * 360 derajat

    }
  if(angle <= 0){
      // Menghentikan valve
      // output_valve = 0;
      analogWrite(pwmVL, 0);
      analogWrite(pwmVR, 0);
    }
  if(angle >= 7 * 360){
      // Menghentikan valve
      // output_valve = 0;
      analogWrite(pwmVL, 0);
      analogWrite(pwmVR, 0);
       }
}

else
  if(kalibrasi_status==0){
    if(angle>0){
      analogWrite(pwmVL, 1500);
      analogWrite(pwmVR, 0);
    }else {
      analogWrite(pwmVL, 0);
      analogWrite(pwmVR, 0);
    }

  }
    // PID controller for DialOutPressure

}
void pidValve(){
  error_valve = tmpsetpoint - DialOutPressure;
  output_valve = Kp_valve * error_valve;
  integral_valve += Ki_valve * error_valve;
  double derivative_valve = Kd_valve * (error_valve - previousError_valve)/0.2;
  output_valve += integral_valve + derivative_valve;
  previousError_valve = error_valve;

  if(output_valve>=2000){
    output_valve = 2000;
  }else
  if(output_valve<=0){
    output_valve = 0;
  }

}
void bloodpump(){
    // valve();
  if (stepper_status==1){
    digitalWrite(13, HIGH);
    digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(interval); 
    digitalWrite(stepPin,LOW); 
  }else
  if (stepper_status==0){
    digitalWrite(13, LOW);
  }
}