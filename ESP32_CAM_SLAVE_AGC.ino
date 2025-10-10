// ESP32-CAM Slave: Camera & Data Logger
// This sketch runs on the ESP32-CAM-MB board, acting as the SLAVE unit.
// It is responsible for:
// 1. Initializing the camera module with dynamic settings (AGC, AEC, AWB).
// 2. Initializing the integrated SD card.
// 3. Listening for a 'C' command from the MASTER ESP32 Dev Board via Serial0.
// 4. Upon receiving the 'C' command, capturing a JPEG image.
// 5. Receiving sensor metadata sent by the MASTER ESP32 Dev Board.
// 6. Saving the captured image to the microSD card with a unique timestamp-based filename.
// 7. Appending the received sensor metadata as a new row in a single 'data.csv' file.
// 8. Sending a "DONE" confirmation back to the MASTER upon completion.

// --- Library Inclusions ---
// The main library for interacting with the ESP32 camera module.
#include "esp_camera.h"
// The standard File System library for ESP32.
#include "FS.h"
// The specific library for the ESP32-CAM's integrated SD card reader, using the SD_MMC protocol.
#include "SD_MMC.h"

// --- Pin Definitions ---
// These pin definitions are specific to the AI-Thinker ESP32-CAM module.
// They map the camera's connections to the correct GPIO pins on the ESP32.
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1 // The camera's reset pin is not typically connected externally.
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM     26 // Serial Camera Control Bus (SCCB) Data pin (like I2C SDA).
#define SIOC_GPIO_NUM     27 // Serial Camera Control Bus (SCCB) Clock pin (like I2C SCL).
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --- Function Prototypes ---
// Declaring functions before the main `setup()` and `loop()` helps organize the code.
void setup_camera();
void setup_sd_card();
void setup_data_file();
void capture_image_and_save_data();

// --- Setup Function ---
// This function runs once when the ESP32-CAM board is powered on or reset.
void setup() {
  // Initialize Serial0, the default serial port used for communication with the Master.
  Serial.begin(115200);
  while (!Serial); // Wait for the serial port to connect.

  Serial.println("ESP32-CAM SLAVE Ready");

  // Call the initialization functions.
  setup_camera();
  setup_sd_card();
  setup_data_file(); // New function to set up the data.csv file.
}

// --- Loop Function ---
// This function runs repeatedly after `setup()` completes. It checks for a command.
void loop() {
  // Check if the Master ESP32 Dev Board has sent any data.
  if (Serial.available()) {
    char command = Serial.read(); // Read the first character from the serial buffer.
    // If the command is 'C' (for Capture), start the capture process.
    if (command == 'C') {
      Serial.println("Capture command received from MASTER!");
      capture_image_and_save_data();
    }
  }
}

// --- Initialization Functions ---

// This function sets up the camera module with all necessary configurations.
void setup_camera() {
  camera_config_t config;
  // Set the LEDC peripheral to control the camera's XCLK signal.
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  // Assign GPIO pins to their corresponding camera functions.
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // Set the clock frequency for the camera.
  config.xclk_freq_hz = 20000000;
  // Configure the output format to JPEG.
  config.pixel_format = PIXFORMAT_JPEG;

  // Set the image resolution to SVGA (800x600 pixels).
  config.frame_size = FRAMESIZE_SVGA;

  // Set the JPEG quality from 0 (highest quality) to 63 (lowest quality).
  config.jpeg_quality = 10;
  // The number of frame buffers to allocate.
  config.fb_count = 1;

  // Initialize the camera with the defined configuration.
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  Serial.println("Camera initialized.");

  // Get a pointer to the sensor object to access its settings.
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    // Enable Automatic Gain Control (AGC), Automatic Exposure Control (AEC),
    // and Automatic White Balance (AWB). This is crucial for dynamic
    // lighting conditions, such as those experienced in a car.
    s->set_gain_ctrl(s, 1); // 1 = Enable AGC (Automatic ISO adjustment)
    s->set_exposure_ctrl(s, 1); // 1 = Enable AEC (Automatic shutter speed adjustment)
    s->set_whitebal(s, 1); // 1 = Enable AWB (Automatic color correction)

    // Ensure manual settings for brightness, contrast, and saturation
    // are at their default values (0) or are not set manually, to allow
    // the automatic controls to function properly.
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
  }
}

// This function sets up the integrated microSD card.
void setup_sd_card() {
  // Begin communication with the SD card using the SD_MMC protocol.
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed! Check card insertion.");
    return;
  }
  Serial.println("SD card initialized.");
}

// New function to handle the data file setup.
void setup_data_file() {
  const char* path = "/data.csv";
  // Check if the file already exists on the SD card.
  if (!SD_MMC.exists(path)) {
    Serial.println("data.csv not found. Creating a new file and writing header...");
    File dataFile = SD_MMC.open(path, FILE_WRITE);
    if (dataFile) {
      // Write the header row to match the structure of your example CSV.
      dataFile.println("timestamp_utc,latitude,longitude,altitude_m,speed_kmh,mag_x,mag_y,mag_z,accel_x,accel_y,accel_z,image_filename_timestamp,mode");
      dataFile.close();
      Serial.println("Header written to data.csv.");
    } else {
      Serial.println("Failed to create data.csv!");
    }
  } else {
    Serial.println("data.csv already exists. New data will be appended.");
  }
}

// --- Image Capture and Data Saving Function ---

// This function handles the entire process of capturing an image, receiving data, and saving both.
void capture_image_and_save_data() {
  // --- Step 1: Receive Metadata from Master ---
  Serial.println("Waiting for sensor data from MASTER...");
  String sensorData = "";
  // Wait for up to 2 seconds to receive the data string.
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') { // Check for a newline character, indicating the end of the data.
        break;
      }
      sensorData += c; // Append the character to the data string.
    }
  }

  // Handle case where no data is received.
  if (sensorData.length() == 0) {
    Serial.println("No sensor data received. Cannot save.");
    Serial.println("DONE_NO_DATA"); // Send a special DONE message to prevent Master timeout.
    return;
  }

  // --- Step 2: Parse Filename and save data to CSV ---
  // The first part of the metadata string is the UTC timestamp.
  int commaIndex = sensorData.indexOf(',');
  String timestamp_utc = "";
  if (commaIndex != -1) {
    timestamp_utc = sensorData.substring(0, commaIndex);
  } else {
    // If the timestamp is not found, use a default string.
    timestamp_utc = "NO_TIMESTAMP";
  }

  // Sanitize the timestamp string to be a valid filename
  // "2025-09-02T16:46:00Z" -> "2025_09_02_16_46_00"
  String timestamp_filename = timestamp_utc;
  timestamp_filename.replace('-', '_');
  timestamp_filename.replace('T', '_');
  timestamp_filename.replace(':', '_');
  timestamp_filename.replace('Z', ' ');
  timestamp_filename.trim();

  // Create the full image filename
  char image_path[60];
  sprintf(image_path, "/IMG_%s.jpg", timestamp_filename.c_str());

  // --- Step 3: Capture Image ---
  Serial.println("Capturing image...");
  // Get a pointer to the camera's frame buffer, which holds the image data.
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed!");
    // Send a "DONE" response even if capture fails, to prevent Master timeout.
    Serial.println("DONE_CAM_FAIL");
    return;
  }

  // --- Step 4: Save Image to SD Card ---
  // Open a file on the SD card in write mode. This creates a new file or overwrites an existing one.
  File imageFile = SD_MMC.open(image_path, FILE_WRITE);
  if (imageFile) {
    imageFile.write(fb->buf, fb->len); // Write the image buffer to the file.
    imageFile.close();                  // Close the file to save the data.
    Serial.printf("Image saved as %s\n", image_path);
  } else {
    Serial.println("Failed to open image file for writing!");
  }
  esp_camera_fb_return(fb); // Return the frame buffer to the camera system for reuse.

  // --- Step 5: Append Metadata to data.csv ---
  // Open data.csv in APPEND mode, which adds new data to the end of the file.
  File dataFile = SD_MMC.open("/data.csv", FILE_APPEND);
  if (dataFile) {
    // Append the received metadata string directly to the CSV file.
    // The master sketch is responsible for the correct formatting of the row.
    dataFile.println(sensorData);
    dataFile.close();
    Serial.println("Sensor data appended to data.csv.");
  } else {
    Serial.println("Failed to open data.csv for appending!");
  }

  // --- Step 6: Send Confirmation to Master ---
  Serial.println("DONE"); // Signal completion back to the Master ESP32 Dev Board.
}








