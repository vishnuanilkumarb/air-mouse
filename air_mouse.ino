#include <Wire.h>
#include <BleMouse.h>

uint8_t i2cData[14];
int16_t gyroX, gyroZ;
int16_t gyroX_offset = 0, gyroZ_offset = 0;

int Sensitivity = 600;
int maxSen = 1200;
int minSen = 400;
int delayi = 20;
int deadZone = 3;

const int buttonLeft = 2;
const int buttonRight = 4;
const int buttonSen = 5;
const int buttonPause = 18; // New pause button

bool lastButtonLeftState = HIGH;
bool lastButtonRightState = HIGH;
bool lastButtonSensitivityState = HIGH;
bool lastPauseButtonState = HIGH;
unsigned long lastPausePressTime = 0;
bool xFlipInverted = true; 

bool isPaused = false;
unsigned long pauseUntil = 0;

BleMouse bleMouse("Air Mouse");
const uint8_t IMUAddress = 0x68;
const uint16_t I2C_TIMEOUT = 1000;

// ---------------------- I2C Helper Functions ----------------------
uint8_t i2cWrite(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  return Wire.endTransmission(sendStop);
}

uint8_t i2cWrite2(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop);
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t* data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false)) return 1;
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available()) data[i] = Wire.read();
      else return 2;
    }
  }
  return 0;
}

// ---------------------- Gyro Calibration ----------------------
void calibrateGyro() {
  int numSamples = 500;
  long sumX = 0, sumZ = 0;

  Serial.println("Calibrating... Keep sensor still!");
  for (int i = 0; i < numSamples; i++) {
    while (i2cRead(0x3B, i2cData, 14));
    int16_t rawX = ((i2cData[8] << 8) | i2cData[9]);
    int16_t rawZ = ((i2cData[12] << 8) | i2cData[13]);
    sumX += rawX;
    sumZ += rawZ;
    delay(2);
  }
  gyroX_offset = sumX / numSamples;
  gyroZ_offset = sumZ / numSamples;

  Serial.print("GyroX Offset: "); Serial.println(gyroX_offset);
  Serial.print("GyroZ Offset: "); Serial.println(gyroZ_offset);
}

// ---------------------- Button Handlers ----------------------
void handleMouseClick(int button, bool &lastState, int mouseButton) {
  bool currentState = digitalRead(button);
  if (currentState == LOW && lastState == HIGH) {
    if (bleMouse.isConnected()) {
      bleMouse.click(mouseButton);
      Serial.print("Clicked button: ");
      Serial.println(mouseButton);
    }
    delay(150); // debounce
  }
  lastState = currentState;
}

void handleSensitivityAdjustment() {
  bool currentState = digitalRead(buttonSen);
  if (currentState == LOW && lastButtonSensitivityState == HIGH) {
    Sensitivity += 50;
    if (Sensitivity > maxSen) {
      Sensitivity = minSen;
    }
    Serial.print("New Sensitivity: ");
    Serial.println(Sensitivity);
    delay(200); // debounce
  }
  lastButtonSensitivityState = currentState;
}

void handlePauseButton() {
  bool currentState = digitalRead(buttonPause);
  unsigned long currentTime = millis();

  if (currentState == LOW && lastPauseButtonState == HIGH) {
    if (currentTime - lastPausePressTime < 500) {
      // Double press detected → toggle X flip
      xFlipInverted = !xFlipInverted;
      Serial.print("X-axis flip toggled. Now: ");
      Serial.println(xFlipInverted ? "INVERTED" : "NORMAL");
    } else {
      // Single press → pause mouse for 2 seconds
      isPaused = true;
      pauseUntil = currentTime + 2000;
      Serial.println("Mouse paused for 2 seconds...");
    }

    lastPausePressTime = currentTime;
  }

  if (isPaused && currentTime > pauseUntil) {
    isPaused = false;
    Serial.println("Mouse resumed.");
  }

  lastPauseButtonState = currentState;
}

// ---------------------- Setup ----------------------
void setup() {
  Wire.begin();
  Serial.begin(115200);
  bleMouse.begin();

  pinMode(buttonLeft, INPUT_PULLUP);
  pinMode(buttonRight, INPUT_PULLUP);
  pinMode(buttonSen, INPUT_PULLUP);
  pinMode(buttonPause, INPUT_PULLUP);

  // Initialize MPU6050
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[3] = 0x00;
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite2(0x6B, 0x01, true));
  while (i2cRead(0x75, i2cData, 1));

  delay(100);

  // Perform Calibration
  calibrateGyro();

  // Optional button action at startup
  if (digitalRead(buttonLeft) == LOW) bleMouse.click(MOUSE_LEFT);
  if (digitalRead(buttonRight) == LOW) bleMouse.click(MOUSE_RIGHT);
  if (digitalRead(buttonSen) == LOW) {
    Sensitivity += 400;
    if (Sensitivity > maxSen) Sensitivity = minSen;
    Serial.print("New Sensitivity: ");
    Serial.println(Sensitivity);
    delay(200);
  }
}

// ---------------------- Loop ----------------------
void loop() {
  // Handle pause
  handlePauseButton();

  // Read IMU if not paused
  if (!isPaused) {
    while (i2cRead(0x3B, i2cData, 14));
    gyroX = ((i2cData[8] << 8) | i2cData[9]) - gyroX_offset;
    gyroZ = ((i2cData[12] << 8) | i2cData[13]) - gyroZ_offset;
    
    gyroX = gyroX / Sensitivity * (xFlipInverted ? -1 : 1);
    gyroZ = gyroZ / Sensitivity * 1;

    if (abs(gyroX) < deadZone) gyroX = 0;
    if (abs(gyroZ) < deadZone) gyroZ = 0;

    if (bleMouse.isConnected()) {
      Serial.print("GyroX: ");
      Serial.print(gyroX);
      Serial.print("  GyroZ: ");
      Serial.println(gyroZ);
      bleMouse.move(gyroX, gyroZ);
    }
  }

  // Handle buttons
  handleMouseClick(buttonLeft, lastButtonLeftState, MOUSE_LEFT);
  handleMouseClick(buttonRight, lastButtonRightState, MOUSE_RIGHT);
  handleSensitivityAdjustment();

  delay(delayi);
}
