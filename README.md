# GeoMetrics Insight System

## 🗺️ Project Overview
The **GeoMetrics Insight System** is a low-cost, dual-use (vehicle-mounted and handheld) data logging solution designed to capture high-quality, spatially uniform datasets. It employs a resilient **ESP32 Master/Slave architecture** to decouple sensor reading/command from image capture/storage, ensuring reliable operation.

The primary goal is to generate **research-grade datasets** for Machine Learning (ML) and Deep Learning (DL) applications, environmental monitoring, or advanced spatial analysis, by addressing common data collection problems like **temporal redundancy** and **lack of orientation context**.

---

## ✨ Key Features
- **Speed-Adaptive Capture**: Images are captured based on distance traveled (not time), using smart logic to ensure uniform spatial coverage, whether the system is moving at high speed (vehicle) or low speed (walking).
- **Decoupled Architecture**:
  - **Master ESP32**: Manages high-precision sensor reading (GPS, 9-DoF IMU), calculates distance, and commands the Slave unit.  
  - **Slave ESP32-CAM**: Focuses solely on reliable, high-speed image capture and SD card logging.
- **Integrated AGC**: The camera configuration is optimized to use Automatic Gain Control (AGC), Automatic White Balance (AWB), and Automatic Exposure Control (AEC) to guarantee consistent image quality across rapidly changing lighting environments.
- **Geospatial and Orientation Data**: Every captured image is logged with precise GPS data (latitude, longitude, altitude, speed) and raw 9-DoF sensor readings (Accelerometer and Magnetometer X, Y, Z) to enable post-processing for Tilt-Compensated Orientation (Pitch, Roll, Yaw).

---

## 🛠️ Hardware Requirements

### Unit Breakdown

#### MASTER
| Component        | Details & Purpose |
|------------------|-------------------|
| ESP32 Dev Board  | Controls logic, reads sensors, communicates with Slave. |
| u-blox NEO-6M    | High-precision GPS module for location and speed data. |
| ADXL345          | 3-Axis Accelerometer (IMU data). |
| HMC5883L         | 3-Axis Magnetometer (IMU data). |
| SSD1306          | 128x64 I2C OLED display for real-time status. |

#### SLAVE
| Component        | Details & Purpose |
|------------------|-------------------|
| ESP32-CAM-MB     | Dedicated image capture and local SD card storage. |
| MicroSD Card     | Used for bulk storage of JPEG images and the combined `data.csv` log file. |

---

## ⚙️ Software Architecture
The two microcontrollers communicate via a dedicated **HardwareSerial (UART1)** connection.

### 1. Master Logic (`ESP32_DEV_MASTER.ino`)
- **Distance Calculation**: Calculates distance moved since the last capture using TinyGPS++ distance formulas.  
- **Capture Trigger**: If the distance exceeds the required threshold (which can be speed/mode adaptive), the Master sends the `'C'` (Capture) command to the Slave.  
- **Data Transmission**: Compiles a comma-separated string of all sensor readings and transmits the full metadata string to the Slave.  
- **Confirmation**: Waits for a `DONE` or `DONE_CAM_FAIL` response from the Slave before resetting the distance counter and continuing the loop.  

### 2. Slave Logic (`ESP32_CAM_SLAVE_AGC.ino`)
- **Listen**: Waits for the `'C'` command.  
- **Capture**: Immediately upon receiving `'C'`, it captures a high-quality JPEG image.  
- **Metadata Wait**: Waits for the subsequent full metadata string from the Master.  
- **Logging**:
  - Saves the image using a timestamp-based filename.  
  - Appends the received metadata string as a new row in the primary log file: `/data.csv`.  
- **Acknowledge**: Sends the `DONE` message back to the Master.  

---

## 🚀 Post-Processing Note
The logged raw accelerometer and magnetometer data must be **post-processed** using a sensor fusion algorithm (e.g., **AHRS/Mahony Filter**) to calculate the true **Pitch, Roll, and Tilt-Compensated Yaw** (True North heading corrected for magnetic declination).  

This step is done **offline** to minimize processing load and power draw on the Master ESP32.

