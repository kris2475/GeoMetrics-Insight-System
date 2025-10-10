// ESP32 Master: Multi-Sensor Logger & CAM Commander
// This sketch runs on the ESP32 Dev Board, which acts as the MASTER unit.
// It is responsible for:
// 1. Initializing and reading data from a u-blox NEO-6M GPS module.
// 2. Initializing and reading data from an ADXL345 accelerometer.
// 3. Initializing and reading RAW data from an HMC5883L magnetometer (GY-271).
// 4. Displaying real-time sensor data (GPS, Accelerometer, Magnetometer raw X,Y,Z) on an SSD1306 OLED.
// 5. Communicating with the SLAVE ESP32-CAM via Serial (UART1) to trigger image captures.
// 6. Sending gathered RAW sensor metadata to the SLAVE ESP32-CAM for logging alongside images.
// 7. Implementing a speed-adaptive, distance-based image capture logic to maintain data uniformity.

#include <Wire.h>             // Required for I2C communication (OLED, ADXL345, HMC5883L)
#include <Adafruit_GFX.h>     // Core graphics library for Adafruit displays
#include <Adafruit_SSD1306.h> // SSD1306 OLED driver library

#include <HardwareSerial.h>   // Required for ESP32's multiple serial ports (GPS, ESP32-CAM)
#include <TinyGPS++.h>        // For parsing GPS data from NMEA sentences

#include <Adafruit_Sensor.h>    // Required for Adafruit Unified Sensor Library
#include <Adafruit_HMC5883_U.h> // Specific library for HMC5883L (GY-271)
#include <Adafruit_ADXL345_U.h> // Specific library for ADXL345
#include <math.h>               // Required for math functions like atan2() and PI (though heading calc removed)

// --- OLED Display Configuration ---
#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 64       // OLED display height, in pixels
#define OLED_RESET -1          // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D    ///< Confirmed working I2C address for your OLED (0x3C or 0x3D)

// Declaration for an SSD1306 display connected to I2C (SDA=GPIO21, SCL=GPIO22 on most ESP32 Dev Kits)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- GPS Module Configuration ---
// Define the pins for the GPS module's serial communication (UART2)
#define GPS_RX_PIN 16 // Connect GPS TX to ESP32 Dev Board RX2 (GPIO16)
#define GPS_TX_PIN 17 // Connect GPS RX to ESP32 Dev Board TX2 (GPIO17)
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // The '2' refers to UART2 on the ESP32 Dev Board

// --- ESP32-CAM Communication (Slave) Configuration ---
// Define the pins for communication with the ESP32-CAM (UART1 on the Master ESP32 Dev Board)
#define CAM_TX_PIN 26 // ESP32 Dev Board TX1 (GPIO26) -> ESP32-CAM-MB RX0 (GPIO3)
#define CAM_RX_PIN 25 // ESP32 Dev Board RX1 (GPIO25) -> ESP32-CAM-MB TX0 (GPIO1)
HardwareSerial camSerial(1); // The '1' refers to UART1 on the ESP32 Dev Board

// --- Status LED Configuration (on Master ESP32 Dev Board) ---
#define STATUS_LED_PIN 2 // GPIO pin for the status LED (often onboard LED on ESP32 dev boards)

// --- Sensor Declarations ---
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // Magnetometer object
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(54321); // Accelerometer object

// --- Timing and Display Variables ---
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000; // Update display every 1 second

// Enum to manage which sensor data page is currently shown on the OLED.
enum DisplayMode {
  GPS_MODE,           // Display GPS data
  ACCEL_MODE,         // Display Accelerometer data
  MAG_MODE,           // Display Magnetometer data (now raw X,Y,Z)
  NUM_DISPLAY_MODES   // Helper to count the total number of display modes
};
DisplayMode currentDisplayMode = GPS_MODE; // Start with GPS data display
unsigned long lastModeChange = 0;        // Stores the last time the display mode was changed
const unsigned long MODE_CHANGE_INTERVAL = 5000; // Interval in milliseconds to change display mode (5 seconds)

// --- Image Capture Logic Variables ---
float totalDistanceTraveled = 0.0;     // Accumulates total distance since boot
float lastCaptureTotalDistance = 0.0;  // Distance at which the last image was captured
TinyGPSLocation last_valid_gps_location; // Stores the last valid GPS location for distance calculation

// Function Prototypes
void setup_oled();
void setup_gps();
void setup_i2c_sensors();
void setup_cam_serial();
void setup_status_led(); // New function for status LED setup
void updateDisplay();
void printSensorDataToSerial();
void capture_data_and_send_metadata();

void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging
  while (!Serial);

  Serial.println("ESP32 Master: Multi-Sensor Logger & CAM Commander");

  Wire.begin(); // Initialize the default I2C bus (SDA=GPIO21, SCL=GPIO22)

  setup_oled();         // Initialize OLED
  setup_gps();          // Initialize GPS
  setup_i2c_sensors();  // Initialize Accelerometer and Magnetometer
  setup_cam_serial();   // Initialize Serial communication with ESP32-CAM (Slave)
  setup_status_led();   // Initialize the status LED

  // Initial display update after all initializations
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Sensors & CAM");
  display.println("Ready!");
  display.display();
  delay(1000);
}

void loop() {
  // Process incoming GPS data from the GPS module
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // If a new valid GPS sentence is processed, update total distance traveled
      if (gps.location.isValid() && last_valid_gps_location.isValid()) {
        // Calculate distance between current and last valid location
        totalDistanceTraveled += gps.distanceBetween(
          gps.location.lat(), gps.location.lng(),
          last_valid_gps_location.lat(), last_valid_gps_location.lng()
        );
      }
      last_valid_gps_location = gps.location; // Update last valid location for next calculation
    }
  }

  unsigned long currentTime = millis();

  // --- Display Update Logic (OLED and Serial Monitor) ---
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = currentTime;
    updateDisplay();           // Refresh OLED
    printSensorDataToSerial(); // Print all data to Serial Monitor for detailed debugging
  }

  // --- Display Mode Cycling Logic (for OLED) ---
  if (currentTime - lastModeChange >= MODE_CHANGE_INTERVAL) {
    lastModeChange = currentTime;
    currentDisplayMode = static_cast<DisplayMode>((currentDisplayMode + 1) % NUM_DISPLAY_MODES);
    updateDisplay(); // Force immediate update on mode change for smoother transition
  }

  // --- Adaptive Distance-Based Image Capture Logic ---
  float captureDistance = 0.0; // Distance threshold for triggering a capture
  float currentSpeed = gps.speed.kmph(); // Current speed in km/h

  // Define capture distance based on current speed, as per document requirements
  if (currentSpeed < 5.0) { // Walking speed (< 5 km/h)
    captureDistance = 5.0;  // Capture every 5 meters
  } else if (currentSpeed >= 5.0 && currentSpeed < 50.0) { // City driving (5 - 50 km/h)
    captureDistance = 15.0; // Capture every 15 meters
  } else { // High speed driving (> 50 km/h)
    captureDistance = 50.0; // Capture every 50 meters
  }

  // Trigger capture if traveled distance exceeds threshold AND speed is above a minimum
  // to avoid false triggers from GPS drift when stationary.
  if (totalDistanceTraveled - lastCaptureTotalDistance >= captureDistance && currentSpeed > 0.5) {
    capture_data_and_send_metadata(); // Call function to trigger capture on Slave and send data
    lastCaptureTotalDistance = totalDistanceTraveled; // Reset for next capture cycle
  }

  // Basic timeout check for GPS data (to help debug if the GPS module is not sending anything)
  if (currentTime > 10000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS data detected. Check GPS module wiring and power.");
  }
}

// --- Initialization Functions ---

void setup_oled() {
  Serial.println("Initializing OLED display...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed. Check OLED wiring and I2C address!"));
    for (;;); // Halt program execution if OLED fails, as it's critical for user feedback
  }
  display.display(); // Show initial splash screen
  delay(2000);       // Display splash for 2 seconds
  display.clearDisplay(); // Clear display after splash
  display.setTextColor(SSD1306_WHITE); // Set text color
  display.setTextSize(1);              // Set smallest text size
  display.setCursor(0, 0);             // Set cursor to top-left
  display.println("OLED Init Done.");
  display.display();
  Serial.println("OLED initialized successfully.");
}

void setup_gps() {
  Serial.println("Initializing GPS module...");
  // Begin HardwareSerial for the GPS module with specified baud rate and pins.
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // Common GPS baud rate (9600 bps)
  Serial.println("GPS module initialized.");
}

void setup_i2c_sensors() {
  Serial.println("Initializing HMC5883L (GY-271) magnetometer...");
  // Initialize the magnetometer. If it fails, print an error but allow the program to continue.
  if (!mag.begin()) {
    Serial.println("Ooops, no HMC5883L detected ... Check wiring!");
  } else {
    Serial.println("HMC5883L initialized successfully.");
  }

  Serial.println("Initializing ADXL345 accelerometer...");
  // Initialize the accelerometer. If it fails, print an error but allow the program to continue.
  if (!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check wiring!");
  } else {
    Serial.println("ADXL345 initialized successfully.");
    accel.setRange(ADXL345_RANGE_16_G); // Set the range for ADXL345 to +/- 16g
  }
}

void setup_cam_serial() {
  Serial.println("Initializing Serial communication with ESP32-CAM (Slave)...");
  // Initialize UART1 on the Master ESP32 for communication with the Slave ESP32-CAM.
  camSerial.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN); // ESP32-CAM typically uses 115200 baud
  Serial.println("ESP32-CAM Serial initialized.");
}

void setup_status_led() {
  Serial.println("Initializing Status LED on Master...");
  pinMode(STATUS_LED_PIN, OUTPUT); // Configure the LED pin as an output
  digitalWrite(STATUS_LED_PIN, LOW); // Ensure the LED is off initially
  Serial.println("Status LED initialized.");
}

// --- Display Update Function ---
// Updates the OLED display content based on the currentDisplayMode.
void updateDisplay() {
  display.clearDisplay();           // Clear the entire display buffer for new content
  display.setTextColor(SSD1306_WHITE); // Set text color to white
  display.setTextSize(1);           // Set text size to 1 (smallest)
  display.setCursor(0, 0);          // Set cursor to top-left corner for each page

  switch (currentDisplayMode) {
    case GPS_MODE:
      display.println("-- GPS Data --");
      if (gps.location.isValid()) {
        display.print("Lat: "); display.println(gps.location.lat(), 5); // Latitude with 5 decimal places
        display.print("Lng: "); display.println(gps.location.lng(), 5); // Longitude with 5 decimal places
        if (gps.altitude.isValid()) {
          display.print("Alt: "); display.print(gps.altitude.meters(), 1); display.println(" m"); // Altitude in meters
        }
        if (gps.speed.isValid()) {
          display.print("Spd: "); display.print(gps.speed.kmph(), 1); display.println(" km/h"); // Speed in km/h
        }
        display.setCursor(0, 55); // Adjust cursor for satellites to fit on screen
        if (gps.satellites.isValid()) {
          display.print("Sats: "); display.println(gps.satellites.value()); // Number of satellites
        } else {
          display.println("Sats: --"); // Display if satellite data is not valid
        }
      } else {
        // Display "Searching for GPS" Status if no fix yet
        display.println("Searching for GPS...");
        display.println("Place outdoors.");
        display.println("Acquiring fix...");
        display.setCursor(0, 40); // Adjust cursor for raw data stats
        if (gps.charsProcessed() > 0) {
          display.print("Chars RX: "); display.println(gps.charsProcessed()); // Total characters received
          display.print("Fixes: "); display.println(gps.sentencesWithFix());   // Number of sentences with a fix
        } else {
          display.println("No NMEA data yet."); // If no NMEA data is being received at all
        }
      }
      break;

    case ACCEL_MODE:
      display.println("-- Accelerometer --");
      sensors_event_t accel_event; // Event object for accelerometer data
      accel.getEvent(&accel_event); // Read the latest accelerometer data

      // Display acceleration values in m/s^2, formatted to 2 decimal places
      display.print("X: "); display.print(accel_event.acceleration.x, 2); display.println(" m/s^2");
      display.print("Y: "); display.print(accel_event.acceleration.y, 2); display.println(" m/s^2");
      display.print("Z: "); display.print(accel_event.acceleration.z, 2); display.println(" m/s^2");
      break;

    case MAG_MODE:
      display.println("-- Magnetometer --");
      sensors_event_t mag_event; // Event object for magnetometer data
      mag.getEvent(&mag_event); // Read the latest magnetometer data

      // Display RAW magnetic field values in micro-Tesla (uT), formatted to 2 decimal places
      display.print("X: "); display.print(mag_event.magnetic.x, 2); display.println(" uT");
      display.print("Y: "); display.print(mag_event.magnetic.y, 2); display.println(" uT");
      display.print("Z: "); display.print(mag_event.magnetic.z, 2); display.println(" uT");
      // Magnetic north calculation removed as per request.
      break;
  }
  display.display(); // Push the content from the buffer to the actual OLED screen
}

// --- Serial Monitor Output Function ---
// Prints all available sensor data to the Serial Monitor for detailed debugging.
void printSensorDataToSerial() {
  Serial.println("--- Current Sensor Data ---");

  // GPS Data Section
  if (gps.location.isValid()) {
    Serial.print("GPS Lat: "); Serial.println(gps.location.lat(), 6);   // Latitude with 6 decimal places
    Serial.print("GPS Lng: "); Serial.println(gps.location.lng(), 6);   // Longitude with 6 decimal places
    if (gps.altitude.isValid()) {
      Serial.print("GPS Alt: "); Serial.print(gps.altitude.meters(), 2); Serial.println(" m"); // Altitude in meters
    }
    if (gps.speed.isValid()) {
      Serial.print("GPS Spd: "); Serial.print(gps.speed.kmph(), 2); Serial.println(" km/h");   // Speed in km/h
    }
    if (gps.date.isValid() && gps.time.isValid()) {
      Serial.print("GPS UTC: ");
      // Format time with leading zeros for consistency
      if (gps.time.hour() < 10) Serial.print("0"); Serial.print(gps.time.hour()); Serial.print(":");
      if (gps.time.minute() < 10) Serial.print("0"); Serial.print(gps.time.minute()); Serial.print(":");
      if (gps.time.second() < 10) Serial.print("0"); Serial.print(gps.time.second()); Serial.println();
    }
    if (gps.satellites.isValid()) {
      Serial.print("GPS Sats: "); Serial.println(gps.satellites.value()); // Number of satellites
    }
    if (gps.hdop.isValid()) {
      Serial.print("GPS HDOP: "); Serial.println(gps.hdop.value(), 2); // Horizontal Dilution of Precision (fix quality)
    }
  } else {
    Serial.println("GPS: Waiting for fix..."); // Message if GPS data is not yet valid
  }

  // Accelerometer Data Section
  sensors_event_t accel_event; // Event object for accelerometer data
  accel.getEvent(&accel_event); // Read accelerometer data
  Serial.print("Accel X: "); Serial.print(accel_event.acceleration.x, 3); Serial.println(" m/s^2");
  Serial.print("Accel Y: "); Serial.print(accel_event.acceleration.y, 3); Serial.println(" m/s^2");
  Serial.print("Accel Z: "); Serial.print(accel_event.acceleration.z, 3); Serial.println(" m/s^2");

  // Magnetometer Data Section (RAW)
  sensors_event_t mag_event; // Event object for magnetometer data
  mag.getEvent(&mag_event); // Read magnetometer data
  Serial.print("Mag X: "); Serial.print(mag_event.magnetic.x, 3); Serial.println(" uT");
  Serial.print("Mag Y: "); Serial.print(mag_event.magnetic.y, 3); Serial.println(" uT");
  Serial.print("Mag Z: "); Serial.print(mag_event.magnetic.z, 3); Serial.println(" uT");
  // Magnetic heading calculation removed as per request for raw data logging.

  Serial.println("------------------------------------"); // Separator for readability in Serial Monitor
}

// --- Image Capture and Metadata Sending Function ---
// This function sends a capture command to the Slave ESP32-CAM and then transmits
// the current sensor metadata for the Slave to save alongside the captured image.
void capture_data_and_send_metadata() {
  digitalWrite(STATUS_LED_PIN, HIGH); // Turn Status LED ON to indicate data transmission
  Serial.println("\nSending capture command to SLAVE ESP32-CAM...");
  camSerial.print('C'); // Send 'C' character as command to the Slave ESP32-CAM

  // Give the Slave ESP32-CAM a moment to prepare for capture
  delay(500);

  // Construct the metadata string with current sensor readings
  String sensorData = "";
  sensors_event_t accel_event, mag_event;
  accel.getEvent(&accel_event); // Get latest accelerometer data
  mag.getEvent(&mag_event);     // Get latest magnetometer data

  // Full timestamp format (YYYY-MM-DDTHH:MM:SSZ)
  String timestamp_utc = "";
  if (gps.date.isValid() && gps.time.isValid()) {
    timestamp_utc += String(gps.date.year());
    timestamp_utc += "-"; if (gps.date.month() < 10) timestamp_utc += "0"; timestamp_utc += String(gps.date.month());
    timestamp_utc += "-"; if (gps.date.day() < 10) timestamp_utc += "0"; timestamp_utc += String(gps.date.day());
    timestamp_utc += "T";
    if (gps.time.hour() < 10) timestamp_utc += "0"; timestamp_utc += String(gps.time.hour());
    timestamp_utc += ":"; if (gps.time.minute() < 10) timestamp_utc += "0"; timestamp_utc += String(gps.time.minute());
    timestamp_utc += ":"; if (gps.time.second() < 10) timestamp_utc += "0"; timestamp_utc += String(gps.time.second());
    timestamp_utc += "Z";
  } else {
    timestamp_utc += "N/A_TIMESTAMP"; // Placeholder if GPS time is not valid
  }

  // Format the metadata string as described in your document, but with RAW magnetic data:
  // "timestamp_utc,latitude,longitude,altitude_m,speed_kmh,mag_x,mag_y,mag_z,true_heading,pitch,roll,image_filename,mode"
  sensorData += timestamp_utc;
  sensorData += ","; sensorData += (gps.location.isValid() ? String(gps.location.lat(), 6) : "N/A");
  sensorData += ","; sensorData += (gps.location.isValid() ? String(gps.location.lng(), 6) : "N/A");
  sensorData += ","; sensorData += (gps.altitude.isValid() ? String(gps.altitude.meters(), 2) : "N/A");
  sensorData += ","; sensorData += (gps.speed.isValid() ? String(gps.speed.kmph(), 2) : "N/A");
  sensorData += ","; sensorData += String(mag_event.magnetic.x, 3); // Raw Magnetic X
  sensorData += ","; sensorData += String(mag_event.magnetic.y, 3); // Raw Magnetic Y
  sensorData += ","; sensorData += String(mag_event.magnetic.z, 3); // Raw Magnetic Z
  sensorData += ",N/A_TRUE_HDG"; // Placeholder for True Heading (requires declination correction)
  sensorData += ",N/A_PITCH";   // Placeholder for Pitch (requires sensor fusion or different calc for accel)
  sensorData += ",N/A_ROLL";    // Placeholder for Roll (requires sensor fusion or different calc for accel)
  sensorData += ",IMG_FILENAME.jpg"; // Placeholder for image filename (actual filename generated by slave)
  sensorData += ",VEHICLE_NORMAL"; // Placeholder for operational mode


  Serial.print("Sending sensor data to SLAVE ESP32-CAM: ");
  Serial.println(sensorData);
  camSerial.print(sensorData); // Send the constructed metadata string to the Slave
  camSerial.print('\n');       // Send a newline character to signal end of data packet

  // Wait for confirmation from the Slave ESP32-CAM
  String camResponse = "";
  unsigned long responseStartTime = millis();
  bool receivedDone = false;
  while (millis() - responseStartTime < 2000) { // Wait up to 2 seconds for response
    if (camSerial.available()) {
      char c = camSerial.read();
      camResponse += c;
      if (camResponse.indexOf("DONE") != -1) { // Look for "DONE" string from Slave
        Serial.println("SLAVE ESP32-CAM confirmed data saved.");
        receivedDone = true;
        break; // Exit loop upon successful confirmation
      }
    }
  }

  digitalWrite(STATUS_LED_PIN, LOW); // Turn Status LED OFF after transmission/timeout
  if (!receivedDone) {
    Serial.println("Timeout: No 'DONE' response from SLAVE ESP32-CAM.");
  }
}









