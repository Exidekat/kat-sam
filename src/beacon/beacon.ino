/* === Beacon ESP32 Code === */
#include <WiFi.h>

const char* ssid = "GORDAN FREEMAN";
const char* password = "crowbars";

WiFiServer server(12345);
float beaconPositionX = 0.0;  // Replace with actual coordinates of the beacon
float beaconPositionY = 0.0;

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
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    String data = client.readStringUntil('\n');
    Serial.println("Received data from robot: " + data);

    // Parse incoming data and validate it
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    int parsedItems = sscanf(data.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f", &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    if (parsedItems == 9) {
      // Data successfully parsed
      Serial.printf("Parsed data - ax: %.2f, ay: %.2f, az: %.2f, gx: %.2f, gy: %.2f, gz: %.2f, mx: %.2f, my: %.2f, mz: %.2f\n",
                    ax, ay, az, gx, gy, gz, mx, my, mz);

      // RSSI Measurement for Distance Estimation
      int32_t rssi = WiFi.RSSI();
      float estimatedDistance = pow(10, ((-69 - rssi) / (10 * 2)));  // Simple RSSI to distance estimation
      Serial.printf("Estimated Distance based on RSSI: %.2f meters\n", estimatedDistance);

      // TODO: Implement AoA measurement for further distance refinement
      // For now, we will just send the estimated distance back to the robot for triangulation

      client.print(estimatedDistance);
    } else {
      // Data parsing failed
      Serial.println("Error: Failed to parse incoming data correctly.");
    }
  }
} 