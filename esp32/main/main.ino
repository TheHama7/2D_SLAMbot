#include <Arduino.h>
#include"IMU.h"
// define GPIOs for IIC.
EulerAngles stAngles;
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

// --- --- --- Encoder Part --- --- ---

// Encoder A pin definition 
const uint16_t AENCA = 35;        // Encoder A input A_C2(B)
const uint16_t AENCB = 34;        // Encoder A input A_C1(A)

// Encoder B pin definition   
const uint16_t BENCB = 16;        // Encoder B input B_C2(B)
const uint16_t BENCA = 27;        // Encoder B input B_C1(A)

//To calculate the number of transitions between high and low levels of a Hall sensor of the encoder within a given "interval" time (in milliseconds)
//If you are using RISING edge detection for initializing interrupts, you'll specifically count the number of transitions from low to high level
volatile long B_wheel_pulse_count = 0;
volatile long A_wheel_pulse_count  = 0;

//To calculate the period for calculating speed, you need to specify the interval at which you want to compute the speed
int interval = 100;

//The current time  
long currentMillis = 0;

// The reduction ratio of the motor means that the motor speed and the output shaft speed of the reduced motor are different.
// For example, with the DCGM3865 motor, with a reduction ratio of 1:42, it means that for every 42 revolutions of the motor, the output shaft completes 1 revolution.
// For each revolution of the output shaft, the motor needs to complete more revolutions as the reduction ratio increases. Typically, higher reduction ratios result in greater torque.
// Take the DCGM3865 motor (reduction ratio: 1：42) as an example:
double reduction_ratio = 42;

//Number of encoder lines, one revolution of the motor, the number of high and low level changes of one Hall sensor of the encoder
int ppr_num = 11;

// When the output shaft completes one revolution, the number of transitions observed in a single Hall sensor
double shaft_ppr = reduction_ratio * ppr_num;

// The callback function of the interrupt function, refer to the attachInterrupt() function later
void IRAM_ATTR B_wheel_pulse() {
  if(digitalRead(BENCA)){
    B_wheel_pulse_count++;
  }
  else{
    B_wheel_pulse_count--;
  }
}

void IRAM_ATTR A_wheel_pulse() {
  if(digitalRead(AENCA)){
    A_wheel_pulse_count++;
  }
  else{
    A_wheel_pulse_count--;
  }
}
// --- --- --- --- --- --- --- --- ---

// --- --- --- Motor Part --- --- ---
// The following defines how to control the ESP32 pin of TB6612
// Motor A
const uint16_t PWMA = 25;         // Motor A PWM control  Orange
const uint16_t AIN2 = 17;         // Motor A input 2      Brown
const uint16_t AIN1 = 21;         // Motor A input 1      Green

// Motor B 
const uint16_t BIN1 = 22;        // Motor B input 1       Yellow
const uint16_t BIN2 = 23;        // Motor B input 2       Purple
const uint16_t PWMB = 26;        // Motor B PWM control   White

// PWM frequency of pins used for PWM outputs
int freq = 100000;

// Define PWM channel
int channel_A = 5;
int channel_B = 6;

// Defines PWM accuracy, when it is 8, the PWM value is 0-255 (2^8-1)
const uint16_t ANALOG_WRITE_BITS = 8;
// The maximum PWM value 
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;
// The minimum PWM, due to the poor low-speed characteristics of DC motors, may not reach the motor's rotation threshold
const uint16_t MIN_PWM = MAX_PWM/5;

// Motor A control 
void channel_A_Ctrl(float pwmInputA){
  // Round the pwmInput value to the nearest integer
  int pwmIntA = round(pwmInputA);

  // If pwmInput is 0, it stops rotating  
  if(pwmIntA == 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    return;
  }

  // Determine the direction of rotation by determining the positive or negative pwmInput value
  if(pwmIntA > 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    // constrain() function is for limiting the pwmIntA value between MIN_PWM and MAX_PWM  
    ledcWrite(channel_A, constrain(pwmIntA, MIN_PWM, MAX_PWM));
  }
  else{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(channel_A,-constrain(pwmIntA, -MAX_PWM, 0));
  }
}

// Motor B control 
void channel_B_Ctrl(float pwmInputB){
  int pwmIntB = round(pwmInputB);
  if(pwmIntB == 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    return;
  }

  if(pwmIntB > 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(channel_B, constrain(pwmIntB, 0, MAX_PWM));
  }
  else{
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(channel_B,-constrain(pwmIntB, -MAX_PWM, 0));
  }
}
// --- --- --- --- --- --- --- --- ---

// --- --- --- Closed-loop Control  --- --- ---
// PID Controller Parameters  
double Kp = 0.05;   // Scale factor
double Ki = 0.05;   // Integral coefficient
double Kd = 0;   // Differentiation factor

// Target rotation speed and the actual rotation speed  
double targetSpeed_A = 100.0;  // Target rotation speed (Adjustable on request)  
double actualSpeed_A = 0.0;   // The actual rotation speed 
double targetSpeed_B = 100.0;  // The target rotation speed (Adjustable on request) 
double actualSpeed_B = 0.0;   // The actual rotation speed 

// PID controller variables 
double previousError_A = 0.0;
double integral_A = 0.0;
double previousError_B = 0.0;
double integral_B = 0.0;

// --- READ MOTOR COMMANDS FROM SERIAL ---
// Format: MOTOR,<rpm_A>,<rpm_B>
void readSerialCommand() {
  if (Serial.available()) {
    line = Serial.readStringUntil('\n');
    line.trim();  // remove \r or whitespace

    Serial.print("Got command: ");
    Serial.println(line);

    if (line.startsWith("MOTOR")) {
      line.replace("MOTOR,", "");
      int comma = line.indexOf(',');

      if (comma > 0) {
        targetSpeed_A = line.substring(0, comma).toFloat();
        targetSpeed_B = line.substring(comma + 1).toFloat();

        Serial.print("Set target A = ");
        Serial.print(targetSpeed_A);
        Serial.print("  B = ");
        Serial.println(targetSpeed_B);
      }
    }
  }
}

float temp;
void setup() {
// Set up the working mode of the encoder's related pins 
    pinMode(BENCB , INPUT_PULLUP);
    pinMode(BENCA , INPUT_PULLUP);

    pinMode(AENCB , INPUT_PULLUP);
    pinMode(AENCA , INPUT_PULLUP);
    
    // Set the interrupt and the corresponding callback function to call the B_wheel_pulse function when BEBCB changes from low to high (RISING)
    attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
    // Set the interrupt and the corresponding callback function to call the A_wheel_pulse function when AEBCB changes from low to high (RISING) 
    attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);

    Serial.begin(115200);
    while(!Serial){}
  // Setting the operating mode of the ESP32 pin used to control the TB6612FNG  
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    // Setting the channel, frequency, and accuracy of the ESP32 pin used for PWM outputs  
    ledcSetup(channel_A, freq, ANALOG_WRITE_BITS);
    ledcAttachPin(PWMA, channel_A);

    ledcSetup(channel_B, freq, ANALOG_WRITE_BITS);
    ledcAttachPin(PWMB, channel_B);

    // The pin used to control rotation is placed at a low level, the motor stops rotating to avoid starting rotation immediately after initialization
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    imuInit();
}

void loop() {
    // --- READ IMU ---
  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

  float ax = stAccelRawData.X;
  float ay = stAccelRawData.Y;
  float az = stAccelRawData.Z;

  float gx = stGyroRawData.X;
  float gy = stGyroRawData.Y;
  float gz = stGyroRawData.Z;

   // --- ENCODER RPM ---
  actualSpeed_A = (A_wheel_pulse_count / shaft_ppr) * 60 * (1000.0/interval);
  actualSpeed_B = (B_wheel_pulse_count / shaft_ppr) * 60 * (1000.0/interval);

  A_wheel_pulse_count = 0;
  B_wheel_pulse_count = 0;

  // --- PID CALC ---
  double error_A = targetSpeed_A - actualSpeed_A;
  integral_A += error_A;
  double output_A = Kp*error_A + Ki*integral_A;

  double error_B = targetSpeed_B - actualSpeed_B;
  integral_B += error_B;
  double output_B = Kp*error_B + Ki*integral_B;

  output_A = constrain(output_A, -MAX_PWM, MAX_PWM);
  output_B = constrain(output_B, -MAX_PWM, MAX_PWM);

  channel_A_Ctrl(output_A);
  channel_B_Ctrl(output_B);

  // --- OUTPUT COMBINED PACKET ---
  Serial.printf("DATA,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f\n",
               ax, ay, az, gx, gy, gz, actualSpeed_A, actualSpeed_B);

  // --- READ MOTOR COMMANDS ---
  readSerialCommand();

  delay(interval);
  

}
