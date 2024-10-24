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

/*
 * V3 Roadmap Overview:
 * 1. **Global Reference Frame Conversion**: Use gravity vector to convert accelerometer data to a global reference frame.
 * 2. **Orientation Calculation**: Estimate the device's orientation angle 'theta' with respect to the global north using the magnetometer.
 * 3. **Fusion with Position Systems**: Integrate future triangulation data from GPS or beacons for improved accuracy and verification.
 * 4. **Data Filtering**: Implement filtering techniques (e.g., Kalman filter) to improve accuracy and reduce noise in position estimates.
 */

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
float theta = 0; // Angle with respect to global north

// Kalman Filter Variables
float kalman_px = 0, kalman_py = 0, kalman_pz = 0;
float kalman_vx = 0, kalman_vy = 0, kalman_vz = 0;
float kalman_ax = 0, kalman_ay = 0, kalman_az = 0;
float p_estimate = 1.0;
float process_noise = 0.01;
float measurement_noise = 0.1;
float kalman_gain;

char buffer[50];  // Buffer for formatted string

void loop() {
  unsigned long currentMillis = millis();

  // Call myCodeCell.Run() every 100ms for power and battery management
  if (currentMillis - lastRunMillis >= runInterval) {
    lastRunMillis = currentMillis;
    myCodeCell.Run();
  }

  // Read sensors every 10ms
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    
    // Read accelerometer and gravity data
    myCodeCell.Motion_AccelerometerRead(raw_ax, raw_ay, raw_az); // Outputs linear acceleration in m/s/s
    myCodeCell.Motion_GravityRead(gx, gy, gz); // Outputs gravity vector of magnitude 9.8 m/s/s

    // Convert accelerometer data to the global reference frame using gravity vector
    float acc_global_x = raw_ax - gx;
    float acc_global_y = raw_ay - gy;
    float acc_global_z = raw_az - gz;

    // Apply Kalman filter for acceleration
    kalman_gain = p_estimate / (p_estimate + measurement_noise);
    kalman_ax = kalman_ax + kalman_gain * (acc_global_x - kalman_ax);
    kalman_ay = kalman_ay + kalman_gain * (acc_global_y - kalman_ay);
    kalman_az = kalman_az + kalman_gain * (acc_global_z - kalman_az);
    p_estimate = (1 - kalman_gain) * p_estimate + process_noise;

    // Read magnetometer data for orientation
    myCodeCell.Motion_MagnetometerRead(mx, my, mz);
    theta = atan2(my, mx) * 180 / M_PI; // Calculate angle with respect to global north

    // Adjust velocity based on corrected acceleration in the global frame
    kalman_vx = kalman_vx * 0.9 + kalman_ax * sensorInterval / 1000;
    kalman_vy = kalman_vy * 0.9 + kalman_ay * sensorInterval / 1000;
    kalman_vz = kalman_vz * 0.9 + kalman_az * sensorInterval / 1000;

    // Update position based on velocity
    kalman_px += kalman_vx * sensorInterval / 1000;
    kalman_py += kalman_vy * sensorInterval / 1000;
    kalman_pz += kalman_vz * sensorInterval / 1000;
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

    sprintf(buffer, "Acc_Global_X: %6.2f, Acc_Global_Y: %6.2f, Acc_Global_Z: %6.2f", kalman_ax, kalman_ay, kalman_az);
    Serial.println(buffer);

    sprintf(buffer, "Vx: %6.2f, Vy: %6.2f, Vz: %6.2f", kalman_vx, kalman_vy, kalman_vz);
    Serial.println(buffer);

    sprintf(buffer, "Px: %6.2f, Py: %6.2f, Pz: %6.2f", kalman_px, kalman_py, kalman_pz);
    Serial.println(buffer);

    sprintf(buffer, "Theta: %6.2f", theta);
    Serial.println(buffer);
  }
}
