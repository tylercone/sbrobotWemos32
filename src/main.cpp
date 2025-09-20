#include <Arduino.h>
#include "wifi/wifi_manager.h"
#include "control/input_controller.h"
#include "gyro/gyro.h"
#include "display/oled.h"
#include "self_balancing/balance.h"
#include "serial_iface/tlv.h"

OLED_Display oled;

float targetAngle = 87.0;

// Deadband for error to reduce noise
float deadBand = 2.0; // degrees

// Global TLV parser instance
TLVParser tlvParser;

void setup()
{
  Serial.begin(115200);
  // Initialize OLED display
  if (!oled.begin())
  {
    Serial.println("OLED initialization failed");
  }
  else
  {
    Serial.println("OLED initialized successfully");
    // Set brightness to a higher level (255 out of 255 for maximum visibility)
    oled.setBrightness(255);
    oled.clearDisplay();
    oled.displayText("Robot Initializing...", 0, 0, 1);
    oled.updateDisplay();
  }

  // Initialize the robot controller
  initController();
  initGyro();
  calibrateAll();

  // calibrateGyro(gyroOffsets);
  // calibrateAccel(accelOffsets);
  initBalance();
  setSpeed(60); // Set initial speed to 60%

  // initWiFi();
}

void loop()
{
  static uint32_t last_angle_print_time = 0;
  static uint32_t last_pid_print_time = 0;
  // handleWebServer();
  // Main code for the robot's balancing loop
  // GyroData gyro = readGyro(gyroOffsets);
  // AccelData accel = readAccel(accelOffsets);
  // Orientation orientation = readOrientation(gyro, accel);
  // Calculate current angle
  // float angle = calculateAngle(accel, gyro);
  // Serial.printf("Angle: %.2f\n", angle);
  // Non-blocking serial input - process each character immediately

  if(millis() - last_angle_print_time > 100) {
    last_angle_print_time = millis();
    float angle = calculateAngle();
    tlvParser.sendFloatTLV(TLV_TYPE_CURRENT_ANGLE, angle);
  }
  if(millis() - last_pid_print_time > 2000) {
    last_pid_print_time = millis();
    tlvParser.sendFloatTLV(TLV_TYPE_GET_TARGET_ANGLE, targetAngle);
    tlvParser.sendFloatTLV(TLV_TYPE_GET_PRINCIPAL, balancePID.kp);
    tlvParser.sendFloatTLV(TLV_TYPE_GET_INTEGRAL, balancePID.ki);
    tlvParser.sendFloatTLV(TLV_TYPE_GET_DERIVITIVE, balancePID.kd);
    tlvParser.sendFloatTLV(TLV_TYPE_GET_BASESPEED, balancePID.baseSpeed);
    tlvParser.sendFloatTLV(TLV_TYPE_GET_DEADBAND, deadBand);
  }

  // Check for incoming TLV data
  if (tlvParser.readTLVFromSerial()) {
    // Example: Process specific TLV types
    float* angle_value;
    if (tlvParser.getTLVAsFloat(TLV_TYPE_SET_ANGLE, angle_value)) {
      targetAngle = *angle_value;
    }

    float* p_value;
    if (tlvParser.getTLVAsFloat(TLV_TYPE_SET_PRINCIPAL, p_value)) {
      setPrincipal(*p_value);
    }

    float* i_value;
    if (tlvParser.getTLVAsFloat(TLV_TYPE_SET_INTEGRAL, i_value)) {
      setPrincipal(*i_value);
    }

    float* d_value;
    if (tlvParser.getTLVAsFloat(TLV_TYPE_SET_DERIVITIVE, d_value)) {
      setPrincipal(*d_value);
    }

    float* basespeed_value;
    if (tlvParser.getTLVAsFloat(TLV_TYPE_SET_BASESPEED, basespeed_value)) {
      setBaseSpeed(*basespeed_value);
    }

    float* deadband_value;
    if (tlvParser.getTLVAsFloat(TLV_TYPE_SET_DEADBAND, deadband_value)) {
      deadBand = *deadband_value
    }

    uint32_t* stop_value;
    if (tlvParser.getTLVAsInt(TLV_TYPE_STOP, stop_value)) {
      if (*stop_value > 0) {
        stopMovement();
	delay(1000); // Small delay to ensure stop command is processed
        calibrateAll();
        targetAngle = 87.0;
      }
    }


    tlvParser.reset();
  }

  // Balance the robot
  balanceRobot(targetAngle, deadBand);

  // Handle web server requests
  // handleWebServer();

  // Display gyro and accelerometer data on OLED using combined function
  // oled.displaySensorData(gyro, accel);

  // Print orientation less frequently to reduce serial lag
  // static int printCounter = 0;
  // printCounter++;
  // if (printCounter >= 20) {  // Print every 20 loops (~100ms at 5ms loop time)
  //   Serial.printf("Orientation - Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", orientation.pitch, orientation.roll, orientation.yaw);
  //   printCounter = 0;
  // }

  // Small delay for stability
  delay(5); // Reduced from 10ms for faster response
}
