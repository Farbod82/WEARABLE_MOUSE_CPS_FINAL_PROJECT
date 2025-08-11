#include <Wire.h>
#include <Arduino.h>
#include <BleMouse.h>
#include <mabutrace.h>
#include <WiFi.h> 

#define MPU6050_ADDR 0x68
#define MPU6050_ADDR2 0x69
#define ADXL345_ADDR 0x53
#define FRAME_DURATION 1250

const float ACCEL_THRESHOLD = 40000;                
const unsigned long DOUBLE_CLICK_WINDOW = 300; 
const unsigned long DEBOUNCE_TIME = 50;   
unsigned long lastClickTime = 0;
bool clickDetected = false;

const double accelDeadzone    = 0.03;  
const double linearSens       = 40.0; 
const float alpha             = 0.2;   
const float smoothDeadzone    = 0.015;
const float stationaryThresh  = 0.1;  
const float gyroThresh        = 5.0;

unsigned long lastMicros;

float ax_smooth = 0.0;
float ay_smooth = 0.0;

double ax_off, ay_off, az_off;
double gx_off, gy_off, gz_off;
double mx_off, my_off, mz_off;
double ax_off2, ay_off2, az_off2;

unsigned long long frameStartTime = 0;
int currentFrame = 0;
BleMouse bleMouse("I2C BLE Mouse", "ESP32", 100);

void readAccelMouse(double &ax, double &ay, double &az);
void readAccel(double &ax, double &ay, double &az);
void readGyro(double &gx, double &gy, double &gz);
void readMag(double &mx, double &my, double &mz);
void getClick(double ax, double ay, double az);
void calibrateAll();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  bleMouse.begin();
    WiFi.mode(WIFI_STA);

    WiFi.begin("farbod", "farbod1234");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    mabutrace_init();
    mabutrace_start_server(81); 

    Serial.print("MabuTrace server started. Go to http://");
    Serial.print(WiFi.localIP());
    Serial.println(":81/ to capture a trace.");

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
   Wire.beginTransmission(MPU6050_ADDR2);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission();

  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x2D); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x31); Wire.write(0x00); Wire.endTransmission();


  calibrateAll();
  lastMicros = micros();

  	while (!bleMouse.isConnected()) {
		delay(100);
    }

    frameStartTime = micros();

}


void log_data(double ax, double ay, int dx, int dy) {
    Serial.print("ax: ");
    Serial.print(ax, 4);
    Serial.print(" ay: ");
    Serial.print(ay, 4);
    Serial.print(F(" ax_smooth: "));
    Serial.print(ax_smooth, 4);
    Serial.print(" ay_smooth: ");
    Serial.print(ay_smooth, 4);
    Serial.print(" dx: ");
    Serial.print(dx);
    Serial.print(" dy: ");
    Serial.print(dy);
}

void sensor_fusion(double &ax, double &ay, double az, double gx, double gy, double gz, int &dx, int &dy) {
    static float roll_est = 0, pitch_est = 0;
    static unsigned long lastTime = micros();
    const float alpha_cf = 0.98;
    const float sensitivity = 40;
    const float deadzone = 0.2;

    gx = gx / 131.0 - gx_off;
    gy = gy / 131.0 - gy_off;

    float roll_acc = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    roll_est = alpha_cf * (roll_est + gy * dt) + (1 - alpha_cf) * roll_acc;
    pitch_est = alpha_cf * (pitch_est + gx * dt) + (1 - alpha_cf) * pitch_acc;

    dx = (fabs(pitch_est) > deadzone) ? pitch_est * sensitivity : 0;
    dy = (fabs(roll_est) > deadzone) ? roll_est * sensitivity : 0;
}

void apply_calibration_to_accel(double &ax, double &ay, double &az)
{
     TRACE_SCOPE("calibrate", COLOR_LIGHT_GRAY);
    {
    ax -= ax_off;
    ay -= ay_off;
    az -= az_off;
    }
}

void apply_calibration_to_gyro(double &gx, double &gy, double &gz)
{
   TRACE_SCOPE("calibrate", COLOR_LIGHT_GRAY);
    {
  gx = gx / 131.0 - gx_off;
    gy = gy / 131.0 - gy_off;
    gz = gz / 131.0 - gz_off;
    }
}
void apply_calibration(double &ax, double &ay, double &az, double &gx, double &gy, double &gz)
{
    TRACE_SCOPE("calibrate", COLOR_LIGHT_GRAY);
    {
    ax -= ax_off;
    ay -= ay_off;
    az -= az_off;
    gx = gx / 131.0 - gx_off;
    gy = gy / 131.0 - gy_off;
    gz = gz / 131.0 - gz_off;
    }
}

void getClick(double ax, double ay, double az) {
    TRACE_SCOPE("click", COLOR_GREEN);
    {
    float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    static unsigned long lastTriggerTime = 0;
    static unsigned long lastClickTime = 0;
    static bool clickDetected = false;
    static bool possibleDoubleClick = false;
    unsigned long currentTime = millis();
    if (accelMagnitude > ACCEL_THRESHOLD && (currentTime - lastTriggerTime > DEBOUNCE_TIME)) {
        if (!clickDetected) {
            clickDetected = true;
            lastClickTime = currentTime;
            possibleDoubleClick = true;
        } else if (possibleDoubleClick && (currentTime - lastClickTime <= DOUBLE_CLICK_WINDOW)) {
            Serial.println("Double Click Detected!");
            bleMouse.click(MOUSE_LEFT);
            delay(100);
            bleMouse.click(MOUSE_LEFT);
            clickDetected = false;
            possibleDoubleClick = false;
        }
        lastTriggerTime = currentTime;
    }

    if (clickDetected && (currentTime - lastClickTime > DOUBLE_CLICK_WINDOW)) {
        Serial.println("Single Click Detected!");
        bleMouse.click(MOUSE_LEFT);
        clickDetected = false;
        possibleDoubleClick = false;
    }
    }
}

void readAccel(double &ax, double &ay, double &az) {
  TRACE_SCOPE("accel", COLOR_BLACK);
  {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x32);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("Accel I2C error");
    return;
  }
  if (Wire.requestFrom(ADXL345_ADDR, 6, true) != 6) {
    Serial.println("Accel read error");
    return;
  }
  int16_t x = Wire.read() | (Wire.read() << 8);
  int16_t y = Wire.read() | (Wire.read() << 8);
  int16_t z = Wire.read() | (Wire.read() << 8);
  ax = x * 0.0039; ay = y * 0.0039; az = z * 0.0039;
  }
}

void readGyro(double &gx, double &gy, double &gz) {
  TRACE_SCOPE("gyro", COLOR_DARK_ORANGE);
  {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("Gyro I2C error");
    return;
  }
  if (Wire.requestFrom(MPU6050_ADDR, 6, true) != 6) {
    Serial.println("Gyro read error");
    return;
  }
  gx = (int16_t)(Wire.read() << 8 | Wire.read());
  gy = (int16_t)(Wire.read() << 8 | Wire.read());
  gz = (int16_t)(Wire.read() << 8 | Wire.read());
  }
}

void readAccelMouse(double &ax, double &ay, double &az) {
  TRACE_SCOPE("accel click", COLOR_GREEN);
  {
  Wire.beginTransmission(MPU6050_ADDR2);
  Wire.write(0x3B); 
  if (Wire.endTransmission(false) != 0) {
    Serial.println("Accel I2C error");
    return;
  }
  if (Wire.requestFrom(MPU6050_ADDR2, 6, true) != 6) {
    Serial.println("Accel read error");
    return;
  }
  ax = (int16_t)(Wire.read() << 8 | Wire.read());
  ay = (int16_t)(Wire.read() << 8 | Wire.read());
  az = (int16_t)(Wire.read() << 8 | Wire.read());
  }
}

void calibrateAll() {
  const int N = 500;
  double Sax = 0, Say = 0, Saz = 0;
  double Sgx = 0, Sgy = 0, Sgz = 0;
  double Smx = 0, Smy = 0, Smz = 0;
  
  double ax_prev = 0, ay_prev = 0, az_prev = 0;
  bool stable = true;

  for (int i = 0; i < N; i++) {
    double ax, ay, az, gx, gy, gz;
    readAccel(ax, ay, az);
    readGyro(gx, gy, gz);
    Sax += ax; Say += ay; Saz += az;
    Sgx += gx; Sgy += gy; Sgz += gz;

    if (i > 0) {
      if (fabs(ax - ax_prev) > 0.05 || fabs(ay - ay_prev) > 0.05 || 
          fabs(az - az_prev) > 0.05) {
        stable = false;
      }
    }
    ax_prev = ax; ay_prev = ay; az_prev = az;
    delay(5);
  }

  if (!stable) {
    Serial.println("Calibration failed: Device not stable");
  }

  ax_off = Sax / N; ay_off = Say / N; az_off = Saz / N - 1.0;
  gx_off = Sgx / N / 131.0; gy_off = Sgy / N / 131.0; gz_off = Sgz / N / 131.0;
}

void move(int dx, int dy){
  TRACE_SCOPE("move", COLOR_LIGHT_GREEN);
  {
    bleMouse.move(-dy, dx);
  }
}

void loop() {
  	double ax, ay, az, gx, gy, gz,ax2,ay2,az2;
	  int dx,dy;
    frameStartTime = micros();

    switch (currentFrame) {
    case 0:
        readAccel(ax,ay,az);
        apply_calibration_to_accel(ax, ay, az);
        sensor_fusion(ax, ay, az, gx, gy, gz, dx, dy);
        break;
    case 1:
        readGyro(gx, gy, gz);
        break;
    case 2:
        readAccelMouse(ax2, ay2, az2);
        apply_calibration_to_gyro(gx,gy, gz);
        sensor_fusion(ax, ay, az, gx, gy, gz, dx, dy);
        break;
    case 3:
        getClick(ax2, ay2, az2);
        break;
    case 4:
        move(-dy, dx);
        apply_calibration(ax, ay, az, gx, gy, gz);
        sensor_fusion(ax, ay, az, gx, gy, gz, dx, dy);
        break;
    case 5:
        readAccel(ax, ay, az);
        break;
    case 6:
        readGyro(gx, gy, gz);
        apply_calibration(ax, ay, az, gx, gy,gz);
        sensor_fusion(ax, ay, az, gx, gy, gz, dx, dy);
        break;
    case 7:
        readAccelMouse(ax2, ay2, az2);
        break;
    }
    while (micros() - frameStartTime < FRAME_DURATION){
        
    }
    currentFrame = (currentFrame + 1) % 8;
}
