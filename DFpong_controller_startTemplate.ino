/*
 * Dragon Ball stick IMU Controller for DF Pong Player Number: 4
   Based on imu_orientationData + DFpong_controller_startTemplate
   Hold the Nano 33 IoT and swing it up and down like a dragon-dance staff to control the paddle.
 */

#include <Arduino_LSM6DS3.h>
#include "SensorFusion.h"

#include <ArduinoBLE.h>          
#include "ble_functions.h"       
#include "buzzer_functions.h"    

SF fusion;

// ---------- IMU global variables ----------
float gx, gy, gz, ax, ay, az;
float pitch = 0.0f;
float roll  = 0.0f;
float yaw   = 0.0f;
float deltat = 0.0f;
unsigned long lastImuReadTime = 0;
unsigned int imuReadInterval = 10;  // 100Hz

// Calibration-related
float gyro_bias[3] = {0, 0, 0};
const int calibration_samples = 500;

// Record the neutral pose*****
float neutralPitch = 6.2f; //obtained from testing
float neutralRoll  = 0.0f;

// Dead zone & smoothing
const float tiltDeadband = 20.0f;   // angle range considered as the "zero state"
float pitchFiltered = 0.0f;
const float alpha = 0.05f;           // Exponential smoothing factor (0–1)

// ---------- DF Pong–related global variables ----------
int currentMovement = 0;            // 0=STOP, 1=UP, 2=DOWN   

const int DEVICE_NUMBER = 4;       

const char* deviceName = "DragonStick";

const int BUZZER_PIN = 11;          
const int LED_PIN = LED_BUILTIN;

// ---------- Function declarations ----------
void calibrateImu();
bool initializeImu();
void readImu();
void printImuValues();
void handleInput();                 // DF Pong 


// ---------- IMU calibration ----------
void calibrateImu() {
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  int valid_samples = 0;
  
  Serial.println("Keep the IMU still in NEUTRAL dragon-stick position..."); // change
  delay(2000);
  
  Serial.println("Calibrating...");
  
  while (valid_samples < calibration_samples) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;
      
      sum_ax += ax;
      sum_ay += ay;
      sum_az += az;
      
      valid_samples++;
      
      if (valid_samples % 100 == 0) {
        Serial.print("Progress: ");
        Serial.print((valid_samples * 100) / calibration_samples);
        Serial.println("%");
      }
      
      delay(2);
    }
  }
  
  // Gyroscope bias
  gyro_bias[0] = sum_gx / calibration_samples;
  gyro_bias[1] = sum_gy / calibration_samples;
  gyro_bias[2] = sum_gz / calibration_samples;
  
  // Average acceleration (used to compute the initial pose)
  float initial_ax = sum_ax / calibration_samples;
  float initial_ay = sum_ay / calibration_samples;
  float initial_az = sum_az / calibration_samples;
  
  // Initialize the filter
  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(0, 0, 0, initial_ax, initial_ay, initial_az, deltat);

  // Stop recording the neutral pose pitch/roll as the zero reference; set them directly instead.
  pitch = fusion.getPitch();
  roll  = fusion.getRoll();
  //neutralPitch = pitch;
  //neutralRoll  = roll;
  pitchFiltered = pitch;  
  
  Serial.println("Calibration complete!");
  Serial.println("Gyro bias values:");
  Serial.print("X: "); Serial.print(gyro_bias[0]);
  Serial.print(" Y: "); Serial.print(gyro_bias[1]);
  Serial.print(" Z: "); Serial.println(gyro_bias[2]);
  
  Serial.print("Neutral pitch: "); // 
  Serial.print(neutralPitch);
  Serial.print("  neutral roll: ");
  Serial.println(neutralRoll);
  
}

// ---------- Initialize the IMU ----------
bool initializeImu() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    return false;
  }
  
  calibrateImu();
  return true;
}

// ---------- Read IMU and update orientation ----------
void readImu() {
  unsigned long currentTime = millis();
  if (currentTime - lastImuReadTime >= imuReadInterval) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);
      
      // 去掉零偏
      gx -= gyro_bias[0];
      gy -= gyro_bias[1];
      gz -= gyro_bias[2];
      
      // deg/s → rad/s
      gx *= DEG_TO_RAD;
      gy *= DEG_TO_RAD;
      gz *= DEG_TO_RAD;
      
      deltat = fusion.deltatUpdate();
      fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
      
      pitch = fusion.getPitch();
      roll  = fusion.getRoll();
      yaw   = fusion.getYaw();

      // Simple first-order low-pass filter to reduce jitter
      pitchFiltered = (1.0f - alpha) * pitchFiltered + alpha * pitch;
    }
    
    lastImuReadTime = currentTime;
  }
}

// ---------- Print IMU values (for debugging only) ----------
void printImuValues() {
  Serial.print("Roll: ");
  Serial.print(roll, 1);
  Serial.print("\tPitch: ");
  Serial.print(pitch, 1);
  Serial.print("\tYaw: ");
  Serial.println(yaw, 1);
}


// ---------- DF Pong setup ----------
void setup() {
  Serial.begin(115200);
  //while (!Serial);

  delay(1000);                      
  Serial.println("=== DF Pong Dragon-Stick Controller Starting ==="); 
  
  if (!initializeImu()) {
    Serial.println("IMU initialization failed!");
    while (1);
  }
  Serial.println("IMU initialized and calibrated!");

  setupBLE(deviceName, DEVICE_NUMBER, LED_PIN);
  setupBuzzer(BUZZER_PIN);
}


// ---------- DF Pong loop ----------
void loop() {
  updateBLE(); 

   handleInput();              
  
  sendMovement(currentMovement);   
  
  updateBuzzer(currentMovement);  
}

// ---------- Core logic: determine 0/1/2 based on the IMU orientation ----------
void handleInput() {           
  // Update IMU orientation
  readImu();
  // Use pitch: decide based on its offset from the neutral value
  float relativeTilt = pitchFiltered - neutralPitch;

  // Dead zone check: treat small-angle motion as STOP
  if (relativeTilt > tiltDeadband) {
    currentMovement = 1;   // UP
  } else if (relativeTilt < -tiltDeadband) {
    currentMovement = 2;   // DOWN
  } else {
    currentMovement = 0;   // 0 
  }

  // Debug output to verify whether the thresholds are appropriate
  Serial.print("pitchFiltered: ");
  Serial.print(pitchFiltered, 1);
  Serial.print("  relativeTilt: ");
  Serial.print(relativeTilt, 1);
  Serial.print("  movement: ");
  Serial.println(currentMovement);
}
