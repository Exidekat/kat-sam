#define GPS_RX_PIN 8 // Define RX pin
#define GPS_TX_PIN 9 // Define TX pin

HardwareSerial gpsSerial(1); // Use UART1

void setup() {
  Serial.begin(115200); // Initialize Serial for debugging
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // Initialize Serial1 for GPS
  Serial.println("Starting GPS communication using Hardware Serial...");
}

void loop() {
  if (gpsSerial.available()) {
    String gpsData = gpsSerial.readStringUntil('\n');
    
    // Check if the line contains useful GPS data and parse it
    if (gpsData.startsWith("$GPGGA")) {
      parseGPGGA(gpsData);
    } else if (gpsData.startsWith("$GPRMC")) {
      parseGPRMC(gpsData);
    } else if (gpsData.startsWith("$GPGSV")) {
      parseGPGSV(gpsData);
    }
  }
}

// Convert NMEA format to Degrees, Minutes, Seconds (DMS)
String convertToDMS(String coordinate, String direction) {
  if (coordinate.length() < 6) return "";

  // Extract degrees and minutes from the NMEA coordinate
  double raw = coordinate.toDouble();
  int degrees = int(raw / 100); // Extract degrees
  double minutes = raw - (degrees * 100); // Extract minutes

  // Convert to DMS format
  int seconds = int((minutes - int(minutes)) * 60);
  int intMinutes = int(minutes);

  // Construct the formatted string
  String dms = String(degrees) + "° " + String(intMinutes) + "' " + String(seconds) + "\" " + direction;
  return dms;
}

// Function to parse $GPGGA sentence
void parseGPGGA(String data) {
  // Split the sentence by commas
  int commaIndex = 0;
  String fields[15];
  for (int i = 0; i < 15; i++) {
    int nextComma = data.indexOf(',', commaIndex);
    if (nextComma == -1) break;
    fields[i] = data.substring(commaIndex, nextComma);
    commaIndex = nextComma + 1;
  }

  // Extract information
  String time = fields[1];
  String latitude = fields[2];
  String latitudeDir = fields[3];
  String longitude = fields[4];
  String longitudeDir = fields[5];
  String fixQuality = fields[6];
  String numSatellites = fields[7];
  String altitude = fields[9];

  // Convert coordinates to DMS format
  String latDMS = convertToDMS(latitude, latitudeDir);
  String lonDMS = convertToDMS(longitude, longitudeDir);

  Serial.println("\n[GPS GPGGA Data]");
  Serial.println("Time: " + time);
  Serial.println("Position: " + latDMS + ", " + lonDMS);
  Serial.println("Fix Quality: " + fixQuality);
  Serial.println("Number of Satellites: " + numSatellites);
  Serial.println("Altitude: " + altitude + " meters");
}

// Function to parse $GPRMC sentence
void parseGPRMC(String data) {
  // Split the sentence by commas
  int commaIndex = 0;
  String fields[12];
  for (int i = 0; i < 12; i++) {
    int nextComma = data.indexOf(',', commaIndex);
    if (nextComma == -1) break;
    fields[i] = data.substring(commaIndex, nextComma);
    commaIndex = nextComma + 1;
  }

  // Extract information
  String time = fields[1];
  String status = fields[2];
  String latitude = fields[3];
  String latitudeDir = fields[4];
  String longitude = fields[5];
  String longitudeDir = fields[6];
  String speed = fields[7];
  String date = fields[9];

  // Convert coordinates to DMS format
  String latDMS = convertToDMS(latitude, latitudeDir);
  String lonDMS = convertToDMS(longitude, longitudeDir);

  Serial.println("\n[GPS GPRMC Data]");
  Serial.println("Time: " + time);
  Serial.println("Status: " + String(status == "A" ? "Active" : "Void"));
  Serial.println("Position: " + latDMS + ", " + lonDMS);
  Serial.println("Speed: " + speed + " knots");
  Serial.println("Date: " + date);
}

// Function to parse $GPGSV sentence (Satellite information)
void parseGPGSV(String data) {
  Serial.println("\n[GPS GPGSV Data]");
  Serial.println("Raw Satellite Info: " + data);
}
