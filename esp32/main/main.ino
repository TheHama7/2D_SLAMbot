#include <Arduino.h>
#include "IMU.h"

// --- IMU Variables ---
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

// VOLATILE COUNTERS (Declaration that was missing the most)
volatile long B_wheel_pulse_count = 0;
volatile long A_wheel_pulse_count  = 0;

// Motor/Encoder constants
double reduction_ratio = 42;
int ppr_num = 11;
const double shaft_ppr = reduction_ratio * ppr_num; // Use 'const' since it's a fixed value

// The distance in mm traveled per one encoder tick
const float WHEEL_DIAMETER_MM = 85.0; // wheel diameter
const float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM; // C = pi*D
const float MM_PER_TICK = WHEEL_CIRCUMFERENCE_MM / shaft_ppr; 

// --- Robot Base Width (for heading calculation) ---
const float TRACK_WIDTH_MM = 150.0; // Example: Distance between the centers of the two wheels in mm

// Odometry Variables (for sending data)
float delta_distance_A_mm = 0.0;
float delta_distance_B_mm = 0.0;

// Encoder Interrupts (IRAM_ATTR ensures fast execution)
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
// --- --- --- End Encoder Part --- --- ---


// The following defines the ESP32 pins used to control the TB6612
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

// Define PWM accuracy, when it is 8, and PWM value is 0-255（2^8-1）
const uint16_t ANALOG_WRITE_BITS = 8;
// The maximum PWM value
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1; // 255
// The minimum PWM value, due to the poor low-speed characteristics of DC motors, may not reach the motor's rotation threshold.
const uint16_t MIN_PWM = MAX_PWM/5; // 51 (approx)


// --- Function Prototypes ---
void channel_A_Ctrl(float pwmInputA);
void channel_B_Ctrl(float pwmInputB);
void processSerialInput();
// ---------------------------

void setup(){
  // Initialize Serial Communication
  Serial.begin(115200); // Standard baud rate for ESP32
  Serial.println("Motor Control Initialized.");
  Serial.println("Enter motor speeds as 'A,B' (e.g., 255,-128). Max speed is 255.");
  Serial.println("------------------------------------------------------------------");

  // --- ENCODER SETUP (MISSING) ---
  pinMode(BENCB , INPUT_PULLUP);
  pinMode(BENCA , INPUT_PULLUP);
  pinMode(AENCB , INPUT_PULLUP);
  pinMode(AENCA , INPUT_PULLUP);
    
  // Attach interrupts for quadrature encoders
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);

  // --- MOTOR SETUP ---
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

  // The pin used to control rotation should be set to a low logic level to stop the motor from rotating, thereby avoiding immediate rotation upon initialization
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  
  // --- IMU SETUP (MISSING) ---
  imuInit(); 
}


// Motor A control  
void channel_A_Ctrl(float pwmInputA){
  // Round the pwmInput value to the nearest integer
  int pwmIntA = round(pwmInputA);
  
  if(pwmIntA == 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(channel_A, 0); // Ensure PWM is 0 when motor is stopped
    return;
  }

  // Determine the direction of rotation by determining the positive or negative pwmInput value 
  if(pwmIntA > 0){ // Forward
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    // constrain() function is for limiting the pwmIntA value between MIN_PWM and MAX_PWM
    ledcWrite(channel_A, constrain(pwmIntA, MIN_PWM, MAX_PWM));
  }
  else{ // Reverse
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    // The absolute value of pwmIntA is constrained between MIN_PWM and MAX_PWM
    ledcWrite(channel_A, constrain(abs(pwmIntA), MIN_PWM, MAX_PWM));
  }
}

// Motor B control  
void channel_B_Ctrl(float pwmInputB){
  int pwmIntB = round(pwmInputB);
  
  if(pwmIntB == 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(channel_B, 0); // Ensure PWM is 0 when motor is stopped
    return;
  }

  if(pwmIntB > 0){ // Forward
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(channel_B, constrain(pwmIntB, MIN_PWM, MAX_PWM));
  }
  else{ // Reverse
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(channel_B, constrain(abs(pwmIntB), MIN_PWM, MAX_PWM));
  }
}

// --- New Function for Serial Input Processing ---
void processSerialInput() {
  if (Serial.available()) {
    // Read the incoming string until a newline character
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any leading/trailing whitespace

    // Find the comma separator
    int commaIndex = input.indexOf(',');
    
    if (commaIndex != -1) {
      // Extract the speed for Motor A (everything before the comma)
      String motorA_str = input.substring(0, commaIndex);
      // Extract the speed for Motor B (everything after the comma)
      String motorB_str = input.substring(commaIndex + 1);

      // Convert the strings to floating point numbers
      float speedA = motorA_str.toFloat();
      float speedB = motorB_str.toFloat();

      // Check if conversion was successful (toFloat() returns 0.0 if invalid, but we need a better check)
      // A simple check is to see if the original string was not empty and not just whitespace
      if (motorA_str.length() > 0 && motorB_str.length() > 0) {
        // Control the motors
        channel_A_Ctrl(speedA);
        channel_B_Ctrl(speedB);

        Serial.print("Set Motor A Speed: ");
        Serial.print(speedA);
        Serial.print(" | Motor B Speed: ");
        Serial.println(speedB);
      } else {
        Serial.println("Invalid format. Please use 'A,B' (e.g., 255,-128).");
      }
    } else {
      Serial.println("Invalid input. Please separate motor speeds with a comma (',').");
    }
  }
}
// ------------------------------------------------

void loop(){
  // Check for serial input and process motor speeds
  processSerialInput();

  // --- IMU Data Reading ---
  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  
  // Define and assign IMU variables (MISSING)
  float ax = stAccelRawData.X;
  float ay = stAccelRawData.Y;
  float az = stAccelRawData.Z;
  float gx = stGyroRawData.X;
  float gy = stGyroRawData.Y;
  float gz = stGyroRawData.Z;

  // --- ENCODER ODOMETRY CALCULATION ---
  // Read the volatile counts into local variables 
  long current_ticks_A = A_wheel_pulse_count; // ERROR 1 FIXED: A_wheel_pulse_count is now declared
  long current_ticks_B = B_wheel_pulse_count; // ERROR 2 FIXED: B_wheel_pulse_count is now declared
  
  // Calculate distance traveled for this interval
  delta_distance_A_mm = (float)current_ticks_A * MM_PER_TICK;
  delta_distance_B_mm = (float)current_ticks_B * MM_PER_TICK;
  
  // Reset pulse count for the next interval
  A_wheel_pulse_count = 0;
  B_wheel_pulse_count = 0;

  // --- OUTPUT COMBINED PACKET ---
  // Format: ODOM,<delta_dist_A_mm>,<delta_dist_B_mm>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<yaw>
  Serial.printf("ODOM,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               delta_distance_A_mm, delta_distance_B_mm, 
               ax, ay, az, gx, gy, gz, stAngles.yaw); // ERROR 3 FIXED: Changed .Yaw to .yaw
               
  // You can keep a small delay here if you want to limit the rate of serial checks, 
  // but it's generally safe to run the check continuously.
  delay(10); 
}