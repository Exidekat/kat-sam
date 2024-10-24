/*
 * Overview:
 * This code shows you how easy it is to set up and run the CodeCell. You just needs to specify which sensors to enable by 
 * passing the appropriate macros to the `Init()` function and use the `Run()` function to handle the power and battery management.
 *
 **** Initialization (`Init` function):
 * 
 * To initialize the CodeCell, use the `myCodeCell.Init()` function with one or more of the predefined macros. Each macro
 * corresponds to a specific sensing function. Here are the available macros:
 * 
 * - `LIGHT`                          // Enables Light Sensing
 * - `MOTION_ACCELEROMETER`           // Enables Accelerometer Sensing
 * - `MOTION_GYRO`                    // Enables Gyroscope Sensing
 * - `MOTION_MAGNETOMETER`            // Enables Magnetometer Sensing
 * - `MOTION_LINEAR_ACC`              // Enables Linear Acceleration Sensing
 * - `MOTION_GRAVITY`                 // Enables Gravity Sensing
 * - `MOTION_ROTATION`                // Enables Rotation Sensing
 * - `MOTION_ROTATION_NO_MAG`         // Enables Rotation Sensing without Magnetometer
 * - `MOTION_STEP_COUNTER`            // Enables Walking Step Counter
 * - `MOTION_STATE`                   // Enables Motion State Detection
 * - `MOTION_TAP_DETECTOR`            // Enables Tap Detector
 * - `MOTION_ACTIVITY`                // Enables Motion Activity Recognition
 * 
 * Example Usage:
 * - `myCodeCell.Init(LIGHT);`                                      // Initializes Light Sensing
 * - `myCodeCell.Init(LIGHT + MOTION_ROTATION);`                    // Initializes Light Sensing and Angular Rotation Sensing
 * - `myCodeCell.Init(LIGHT + MOTION_ROTATION + MOTION_STATE);`     // Initializes Light Sensing, Angular Rotation Sensing, and State Detection
 * 
 * Note: You can combine multiple macros using the `+` operator to initialize multiple sensors.
 * 
 **** Running the Code (`Run` function):
 * 
 * Call the `myCodeCell.Run()` function in the `loop()` to manage battery and power. This function returns true every 100ms and
 * also handles the onboard LED to indicate power status. When the battery voltage falls below 3.3V, the LED will blink red 10 times 
 * and then go into Sleep Mode until the USB cable is connected for charging. While charging, the CodeCell will shut down the application, 
 * light the LED blue, and wait until the battery is fully charged. Once fully charged, it will start a breathing-light animation with a speed
 * corresponding to the proximity distance. The LED will shine green when powered by the battery and blue when powered via USB.
 * 
 **** Reading Sensor Data:
 * 
 * After initializing the sensors, you can use the following functions to read data from them:
 * 
 * Sensor Read Functions:
 * 
 * - `Light_ProximityRead()`                                            // Reads the proximity value from the light sensor
 * - `Light_WhiteRead()`                                                // Reads the white light intensity from the light sensor
 * - `Light_AmbientRead()`                                              // Reads the ambient light intensity from the light sensor
 * - `Motion_TapRead()`                                                  // Reads the number of taps detected (tap = 1, no tap = 0)
 * - `Motion_StateRead()`                                               // Reads the current state (On Table = 1, Stationary = 2, Stable = 3, Motion = 4)
 * - `Motion_ActivityRead()`                                            // Reads the current activity (Driving = 1, Cycling = 2, Walking = 3/6, Still = 4, Tilting = 5, Running = 7, Climbing Stairs = 8)
 * - `Motion_AccelerometerRead(float &x, float &y, float &z)`           // Reads acceleration data along the x, y, and z axes
 * - `Motion_GyroRead(float &x, float &y, float &z)`                    // Reads rotational velocity data along the x, y, and z axes
 * - `Motion_MagnetometerRead(float &x, float &y, float &z)`            // Reads magnetic field strength data along the x, y, and z axes
 * - `Motion_GravityRead(float &x, float &y, float &z)`                 // Reads gravity vector data along the x, y, and z axes
 * - `Motion_LinearAccRead(float &x, float &y, float &z)`               // Reads linear acceleration data along the x, y, and z axes
 * - `Motion_StepCounterRead(uint16_t &x)`                              // Reads the number of steps counted
 * - `Motion_RotationRead(float &roll, float &pitch, float &yaw)`       // Reads angular rotational data (roll, pitch, yaw)
 * - `Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw)`  // Reads angular rotational data without magnetometer
 *
 * Example Usage:
 * - `uint16_t proximity = myCodeCell.Light_ProximityRead();`
 * - `myCodeCell.Motion_AccelerometerRead(myX, myY, myZ);`
 * - `myCodeCell.Motion_RotationRead(myRoll, myPitch, myYaw);`
 * 
 * Note: You can use `myCodeCell.PrintSensors()`to prints the values of all enabled sensors on the Serial Monitor.
 * 
 */

// ESP32 Positioning System V4
// This version uses two ESP32s - one as a mobile robot and one as a stationary beacon to refine position estimation.

// ESP32 Positioning System V4
// This version uses two ESP32s - one as a mobile robot and one as a stationary beacon to refine position estimation.

/* === Mobile ESP32 Code === */
#include <CodeCell.h>
#include <WiFi.h>
#include <math.h>

// Replace these with your WiFi credentials
const char* ssid = "GORDAN FREEMAN";
const char* password = "crowbars";

const char* beaconIP = "192.168.4.3";  // Replace with actual IP of the beacon
const int beaconPort = 12345;

WiFiClient client;
CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("Local IPv4: ");
  Serial.println(WiFi.localIP());

  myCodeCell.Init(LIGHT + MOTION_ROTATION + MOTION_ACCELEROMETER + MOTION_GRAVITY + MOTION_GYRO + MOTION_MAGNETOMETER);
  myCodeCell.Run();
}

void loop() {
  unsigned long currentMillis = millis();
  static unsigned long lastSendMillis = 0;
  const long sendInterval = 500;  // Send data every 500 ms

  if (currentMillis - lastSendMillis >= sendInterval) {
    lastSendMillis = currentMillis;

    // Collect sensor data
    float ax, ay, az;
    myCodeCell.Motion_AccelerometerRead(ax, ay, az);
    float gx, gy, gz;
    myCodeCell.Motion_GravityRead(gx, gy, gz);
    float mx, my, mz;
    myCodeCell.Motion_MagnetometerRead(mx, my, mz);

    // Calculate orientation and other data
    float pitch_angle = atan2(gx, sqrt(gy * gy + gz * gz));
    float roll_angle = atan2(gy, gz);
    float theta = atan2(my, mx) * 180 / M_PI;

    // Debugging output for data being sent
    Serial.printf("Sending data to beacon: ax=%.2f, ay=%.2f, az=%.2f, gx=%.2f, gy=%.2f, gz=%.2f, mx=%.2f, my=%.2f, mz=%.2f\n", ax, ay, az, gx, gy, gz, mx, my, mz);

    // Send data to beacon with error detection
    if (client.connect(beaconIP, beaconPort)) {
      char buffer[150];
      snprintf(buffer, sizeof(buffer), "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, mx, my, mz);
      client.print(buffer);
      client.stop();
    } else {
      Serial.println("Failed to connect to beacon");
    }
  }
}
