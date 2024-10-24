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

#include <CodeCell.h>
#include <math.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */
  //SerialBT.begin("ESP32_BT");      // Bluetooth Serial communication (ESP32_BT is the device name)
  myCodeCell.Init(LIGHT + MOTION_ROTATION + MOTION_ACCELEROMETER + MOTION_GRAVITY + MOTION_GYRO + MOTION_MAGNETOMETER);
  myCodeCell.Run();
}

unsigned long previousMillis = 0;
unsigned long lastRunMillis = 0;
unsigned long lastPrintMillis = 0;
const long sensorInterval = 10; // 10ms interval for sensor reads
const long runInterval = 100;   // 100ms for myCodeCell.Run()
const long printInterval = 1000; // Print sensor data every 1000ms interval

int cycle = 0;
float gx, gy, gz;
float ax, ay, az;
float raw_ax, raw_ay, raw_az;
float mx, my, mz;
float roll, pitch, yaw;
float vx = 0, vy = 0, vz = 0;
float px = 0, py = 0, pz = 0;

float dampeningFactor = 0.50; // attempts to account for velocity error by assuming it tends towards 0
                              // 0.50 should halve every 100 ms
// adjust for sensorInterval
float dampeningFactorAdj = pow(dampeningFactor, float(sensorInterval) / 1000);
                             
char buffer[50];  // Buffer for formatted string

void loop() {
  unsigned long currentMillis = millis();

  // Call myCodeCell.Run() every 100ms for power and battery management
  if (currentMillis - lastRunMillis >= runInterval) {
    lastRunMillis = currentMillis;
    //myCodeCell.Run();
  }

  // Read sensors every 10ms
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    
    // Read accelerometer and gravity data
    myCodeCell.Motion_AccelerometerRead(raw_ax, raw_ay, raw_az); // Outputs linear acceleration in m/s/s
    myCodeCell.Motion_GravityRead(gx, gy, gz); // Outputs gravity vector of magnitude 9.8 m/s/s

    // Correct accelerometer data by removing the effect of gravity
    ax = -(raw_ax - gx);
    ay = -(raw_ay - gy);
    az = -(raw_az - gz);

    // Apply magnetometer data for correction and orientation adjustments
    myCodeCell.Motion_MagnetometerRead(mx, my, mz);
    myCodeCell.Motion_RotationRead(roll, pitch, yaw);

    // Apply a complementary filter to fuse accelerometer and magnetometer data for better orientation estimation
    float alpha = 0.98;
    float acc_roll = atan2(ay, az) * 180 / M_PI;
    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;
    roll = alpha * (roll + gx * sensorInterval / 1000) + (1 - alpha) * acc_roll;
    pitch = alpha * (pitch + gy * sensorInterval / 1000) + (1 - alpha) * acc_pitch;

    // Improve dampening with adaptive dampening based on acceleration magnitude
    float accelerationMagnitude = sqrt(ax * ax + ay * ay + az * az);
    float adaptiveDampeningFactor = dampeningFactor / (1 + accelerationMagnitude);
    float adaptiveDampeningFactorAdj = pow(adaptiveDampeningFactor, float(sensorInterval) / 1000);

    // Adjust velocity based on corrected acceleration
    vx = vx * adaptiveDampeningFactorAdj + ax * sensorInterval / 1000;
    vy = vy * adaptiveDampeningFactorAdj + ay * sensorInterval / 1000;
    vz = vz * adaptiveDampeningFactorAdj + az * sensorInterval / 1000;

    // Apply a threshold to velocity to prevent drift when stationary
    float velocityThreshold = 0.05;
    if (fabs(vx) < velocityThreshold) vx = 0;
    if (fabs(vy) < velocityThreshold) vy = 0;
    if (fabs(vz) < velocityThreshold) vz = 0;

    // Update position based on velocity
    px = px + vx * sensorInterval / 1000;
    py = py + vy * sensorInterval / 1000;
    pz = pz + vz * sensorInterval / 1000;
  }

  // Print data every second (or adjust to your preferred interval)
  if (currentMillis - lastPrintMillis >= printInterval) {
    lastPrintMillis = currentMillis;

    // Count cycles
    cycle++;

    // Print cycle count and sensor data
    Serial.print("Cycle: ");
    Serial.println(cycle);

    sprintf(buffer, "Gx: %6.2f, Gy: %6.2f, Gz: %6.2f", gx, gy, gz);
    Serial.println(buffer);

    sprintf(buffer, "Raw_Ax: %6.2f, Raw_Ay: %6.2f, Raw_Az: %6.2f", raw_ax, raw_ay, raw_az);
    Serial.println(buffer);

    sprintf(buffer, "Ax: %6.2f, Ay: %6.2f, Az: %6.2f", ax, ay, az);
    Serial.println(buffer);

    sprintf(buffer, "Vx: %6.2f, Vy: %6.2f, Vz: %6.2f", vx, vy, vz);
    Serial.println(buffer);

    sprintf(buffer, "Px: %6.2f, Py: %6.2f, Pz: %6.2f", px, py, pz);
    Serial.println(buffer);

    sprintf(buffer, "Roll: %6.2f, Pitch: %6.2f, Yaw: %6.2f", roll, pitch, yaw);
    Serial.println(buffer);
  }
}


