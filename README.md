# ESP32 RC Boat Web Controller 

This project is an **ESP32-powered remote control boat** that can be operated through a **web-based user interface**.  
It features motor control, servo steering, and a real-time **2S Li-ion battery monitor** with charging state detection.

---

##  Features
- **ESP32 Access Point (AP) Mode** – Works without internet.
- **Web UI Control**
  - Vertical throttle slider (forward/reverse).
  - Horizontal steering slider (left/right).
  - Both reset to center (0%) when released.
- **Motor Control** – MX1508 motor driver with PWM forward/reverse.
- **Servo Rudder**
  - Controlled via ESP32Servo library.
  - Always returns to center when released.
- **Battery Monitoring**
  - Calibrated voltage reading via voltage divider.
  - Displayed in volts and percentage (%SoC).
  - Critical cutoff at 10% to protect Li-ion cells.
  - Detects charging (voltage > 9V → “Charging” indicator, green icon).
- **Serial Logging**
  - Every command logged as:  
    ```
    THR: +45%  STR: -20%
    ```
- **Failsafe**
  - Motor automatically shuts down on low battery.

---

# Hardware
- ESP32 Dev Module  
- MX1508 motor driver  
- SG90 servo motor  
- 2 × 18650 cells (2S with BMS protection)  

---

# Usage
1. Flash the provided code to ESP32.  
2. Connect to Wi-Fi SSID: `TEKNE_AP` (default password: `12345678`).  
3. Open the ESP32 web interface in your browser.  
4. Control throttle and steering with the sliders.  
5. Monitor battery voltage, percentage, and charging status in real-time.  

---

# To Do
- Add wiring diagram.  
- Add photos of the assembled boat.  
- Improve UI design for mobile responsiveness.  

---

# License
This project is open-source under the MIT License.
