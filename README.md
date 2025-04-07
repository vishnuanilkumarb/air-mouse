# ESP32 Air Mouse with MPU6050 & BleMouse

Turn your ESP32 into a Wireless Air Mouse!
Move the cursor by moving your hand vertically and horizontally, click with button.

---

##  Features

 * **Motion Control:** Move the mouse by moving your hand horizontally and vertically (MPU6050).  
 * **Left Click:** Press the first button (GPIO 2).  
 * **Right Click:** Press the second button (GPIO 4).  
 * **Pause Click:** Press the third button to help with readjusting the mouse in air(GPIO 18).
 * **Sensitivity Click** Press the fourth Button to adjust the values and keep pressing to iterate from the Min Value to the Max Value by a step value of 50 (GPIO 5).
 * **Wireless Bluetooth HID:** No PC script needed—connects as a real Bluetooth mouse, can use with a power bank or another portable power source.  

---

##  Hardware Required

| Component       | Quantity |
|----------------|----------|
| ESP32 Board    | 1        |
| MPU6050 IMU Sensor | 1    |
| Push Buttons   |    4     |
| Resistors (10kΩ) |   4    |
| Breadboard & Jumper Wires | As needed |
| Battery (For wireless use) | 1 |

---

##  Pin Configuration

| Component  | ESP32 GPIO Pin |
|------------|---------------|
| MPU6050 VCC | 3.3V |
| MPU6050 GND | GND  |
| MPU6050 SDA | GPIO 22 |
| MPU6050 SCL | GPIO 21 |
| Left Click  | GPIO 2  |
| Right Click | GPIO 4  |
| Sen Click   | GPIO 5  |
| Pause Click | GPIO 18 |

---

##  Setup & Installation

###  Install Arduino Libraries

- **BLEMouse** (`BLEMouse`)

###  Upload Code to ESP32

- Select DOIT ESP32 DEVKIT V1 board in Arduino IDE.
- Connect ESP32 via USB and upload the code.

###  Connect Bluetooth on PC/Phone

- Go to Bluetooth Settings → Add a new device.
- Select **"Air Mouse"** from the list.
- Pair and start using the air mouse!

---

##  Code Explanation

## 1️⃣ Reads motion data from MPU6050

- Communicates with MPU6050 using I2C protocol.
- Reads 14 bytes from register `0x3B` including gyro and accel data.
- Extracts and calibrates **gyroscope X and Z values**.

```cpp
Wire.beginTransmission(IMU_ADDR);
Wire.write(0x3B); // Starting register to read from
Wire.endTransmission(false);
Wire.requestFrom(IMU_ADDR, 14);

gyroX = (Wire.read() << 8 | Wire.read()) - gyroX_offset;
// ...
gyroZ = (Wire.read() << 8 | Wire.read()) - gyroZ_offset;
```
## 2️⃣ Converts gyroscope values into mouse movements

- The gyroscope readings from the MPU6050 are scaled using a `Sensitivity` factor to control how fast the cursor moves.
- The movement values are inverted (`* -1`) for natural cursor direction.
- A **dead zone** is used to eliminate tiny unwanted movements when the device is still.
- Movements are clamped between -127 and 127, which is the range for HID mouse reports.

```cpp
int8_t dx = constrain(gyroX / Sensitivity * -1, -127, 127);
int8_t dy = constrain(gyroZ / Sensitivity * -1, -127, 127);

if (abs(dx) < deadZone) dx = 0;
if (abs(dy) < deadZone) dy = 0;
```
## 3️⃣ Detects Button Presses for Left/Right Click

- Two physical buttons are connected to digital pins on the ESP32:
  - `buttonLeft` triggers a **left mouse click**
  - `buttonRight` triggers a **right mouse click**
- When a button is pressed (`LOW` state), the corresponding bit is set in the `btns` variable:
  - `0x01` for left click
  - `0x02` for right click
- A third button (`buttonSen`) is used to adjust mouse sensitivity:
  - Each press increases sensitivity by 50
  - When it exceeds `maxSen`, it wraps around to `minSen`
  - Includes a debounce delay to prevent rapid toggling

### Example Code Snippet:

```cpp
uint8_t btns = 0;

// Detect button press for left click
if (digitalRead(buttonLeft) == LOW) {
  btns |= 0x01;
}

// Detect button press for right click
if (digitalRead(buttonRight) == LOW) {
  btns |= 0x02;
}

// Handle sensitivity adjustment with debounce
if (digitalRead(buttonSen) == LOW && lastButtonSen == HIGH &&
    millis() - lastDebounceTime > debounceDelay) {
  Sensitivity += 50;
  if (Sensitivity > maxSen) Sensitivity = minSen;
  Serial.print("New Sensitivity: ");
  Serial.println(Sensitivity);
  lastDebounceTime = millis();
}
lastButtonSen = digitalRead(buttonSen);
```
## 4️⃣ Sends HID Mouse Commands Over Bluetooth

- The ESP32 uses the `ArduinoBLE` library to act as a **Bluetooth Low Energy (BLE) HID device**.
- It creates a BLE HID service with a **mouse report characteristic**.
- Mouse movement and button state are sent by writing to this characteristic as a **4-byte HID report**:
  - Byte 0: Button state (`0x01` = left click, `0x02` = right click, etc.)
  - Byte 1: Horizontal movement (X)
  - Byte 2: Vertical movement (Y)
  - Byte 3: Scroll (Z)

### Sending Mouse Reports:

```cpp
uint8_t report[4] = {btns, (int8_t)gyroZ, (int8_t)gyroX, 0};  // HID mouse report
mouseInput.writeValue(report, sizeof(report));               // Send to BLE host
```


---

  ## <u>Troubleshooting</u>

**<u>Bluetooth not connecting?</u>**  
* Restart ESP32 and try pairing again / press the reset button.  
* Remove **"ESP32 AirMouse"** from Bluetooth devices and re-add.  

**<u>Mouse movement too fast/slow?</u>**  
* Press the fourth button which is the sensitivity for the mouse.  
* Check the serial for the sensitivity of the mouse.

**<u>Button not working?</u>**  
* Check wiring.



