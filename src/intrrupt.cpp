#include <Wire.h>
#include <Arduino.h>
#include <BleMouse.h>

#define MPU6050_ADDR      0x68
#define MPU6050_ADDR2     0x69
#define ADXL345_ADDR      0x53

#define ADXL345_INT_PIN   25
#define MPU6050_INT_PIN   27
#define MPU6050_INT_PIN2  26

enum EventType {
    NO_EVENT,
    READ_ADXL345,
    READ_MPU6050_GYRO,
    READ_MPU6050_MOUSE_ACCEL,
    PROCESS_MOVEMENT,
    PROCESS_CLICK
};

#define QUEUE_SIZE 64
volatile EventType eventQueue[QUEUE_SIZE];
volatile int queueHead = 0;
volatile int queueTail = 0;

hw_timer_t *moveTimer = NULL;
hw_timer_t *accelTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
const int MOVE_INTERVAL_MS = 10;
const int ACCEL_POLL_INTERVAL_MS = 10;

const float CLICK_ACCEL_THRESHOLD = 40000;
const unsigned long DOUBLE_CLICK_WINDOW = 300;
const unsigned long DEBOUNCE_TIME = 50;

const double accelDeadzone = 0.03;
const double linearSens = 40.0;
const float alpha = 0.2;
const float smoothDeadzone = 0.015;
const float stationaryThresh = 0.1;
const float gyroThresh = 5.0;

volatile double ax = 0.0, ay = 0.0, az = 0.0;
volatile double gx = 0.0, gy = 0.0, gz = 0.0;
volatile double ax2 = 0.0, ay2 = 0.0, az2 = 0.0;
int dx = 0, dy = 0;

float ax_smooth = 0.0;
float ay_smooth = 0.0;

double ax_off, ay_off, az_off;
double gx_off, gy_off, gz_off;

BleMouse bleMouse("I2C BLE Mouse", "ESP32", 100);

void readAccel(double &ax_p, double &ay_p, double &az_p);
void readGyro(double &gx_p, double &gy_p, double &gz_p);
void readAccelMouse(double &ax_p, double &ay_p, double &az_p);
void getClick();
void calibrateAll();
void sensor_fusion(int &dx_p, int &dy_p);

void IRAM_ATTR enqueue(EventType event) {
    int nextTail = (queueTail + 1) % QUEUE_SIZE;
    if (nextTail != queueHead) {
        eventQueue[queueTail] = event;
        queueTail = nextTail;
    }
}

EventType dequeue() {
    if (queueHead == queueTail) {
        return NO_EVENT;
    }
    EventType event = eventQueue[queueHead];
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    return event;
}

void IRAM_ATTR onMoveTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    enqueue(PROCESS_MOVEMENT);
    portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR onAdxlPoll() {
    portENTER_CRITICAL_ISR(&timerMux);
    enqueue(READ_ADXL345);
    portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR onMpuGyroDataReady() {
    enqueue(READ_MPU6050_GYRO);
}

void IRAM_ATTR onMpuMouseAccelDataReady() {
    enqueue(READ_MPU6050_MOUSE_ACCEL);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    bleMouse.begin();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); Wire.write(0);
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x19); Wire.write(9);
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x38); Wire.write(0x01);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR2);
    Wire.write(0x6B); Wire.write(0);
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU6050_ADDR2);
    Wire.write(0x1A); Wire.write(0x03);
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU6050_ADDR2);
    Wire.write(0x19); Wire.write(3);
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU6050_ADDR2);
    Wire.write(0x38); Wire.write(0x01);
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU6050_ADDR2);
    Wire.write(0x37); Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x2D); Wire.write(0x08);
    Wire.endTransmission(true);
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x31); Wire.write(0x00);
    Wire.endTransmission(true);
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x2C); Wire.write(0x0B);
    Wire.endTransmission(true);
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x2E); Wire.write(0x00);
    Wire.endTransmission(true);

    calibrateAll();

    pinMode(MPU6050_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU6050_INT_PIN), onMpuGyroDataReady, RISING);
    pinMode(MPU6050_INT_PIN2, INPUT);
    attachInterrupt(digitalPinToInterrupt(MPU6050_INT_PIN2), onMpuMouseAccelDataReady, RISING);

    moveTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(moveTimer, &onMoveTimer, true);
    timerAlarmWrite(moveTimer, MOVE_INTERVAL_MS * 1000, true);
    timerAlarmEnable(moveTimer);

    accelTimer = timerBegin(1, 80, true);
    timerAttachInterrupt(accelTimer, &onAdxlPoll, true);
    timerAlarmWrite(accelTimer, ACCEL_POLL_INTERVAL_MS * 1000, true);
    timerAlarmEnable(accelTimer);

    while (!bleMouse.isConnected()) {
        delay(100);
    }
}

void loop() {
    EventType event = dequeue();
    if (event != NO_EVENT) {
        switch (event) {
            case READ_ADXL345: {
                double temp_ax, temp_ay, temp_az;
                readAccel(temp_ax, temp_ay, temp_az);
                ax = temp_ax - ax_off;
                ay = temp_ay - ay_off;
                az = temp_az - az_off;
                break;
            }
            case READ_MPU6050_GYRO: {
                double temp_gx, temp_gy, temp_gz;
                readGyro(temp_gx, temp_gy, temp_gz);
                gx = (temp_gx / 131.0) - gx_off;
                gy = (temp_gy / 131.0) - gy_off;
                gz = (temp_gz / 131.0) - gz_off;
                break;
            }
            case READ_MPU6050_MOUSE_ACCEL: {
                double temp_ax2, temp_ay2, temp_az2;
                readAccelMouse(temp_ax2, temp_ay2, temp_az2);
                ax2 = temp_ax2;
                ay2 = temp_ay2;
                az2 = temp_az2;
                enqueue(PROCESS_CLICK);
                break;
            }
            case PROCESS_MOVEMENT:
                sensor_fusion(dx, dy);
                if (dx != 0 || dy != 0) {
                    bleMouse.move(-dy, dx);
                }
                break;
            case PROCESS_CLICK:
                getClick();
                break;
            case NO_EVENT:
                break;
        }
    }
}

void sensor_fusion(int &dx_p, int &dy_p) {
    double temp_ax = ax;
    double temp_ay = ay;
    double temp_az = az;
    double temp_gx = gx;
    double temp_gy = gy;
    double temp_gz = gz;

    float accelMag = sqrt(temp_ax * temp_ax + temp_ay * temp_ay + temp_az * temp_az);
    bool isStationary = (fabs(accelMag - 1.0) < stationaryThresh &&
                         fabs(temp_gx) < gyroThresh &&
                         fabs(temp_gy) < gyroThresh &&
                         fabs(temp_gz) < gyroThresh);

    if (isStationary) {
        ax_smooth = 0.0;
        ay_smooth = 0.0;
    } else {
        if (fabs(temp_ax) < accelDeadzone) temp_ax = 0;
        if (fabs(temp_ay) < accelDeadzone) temp_ay = 0;
        ax_smooth = alpha * temp_ax + (1.0 - alpha) * ax_smooth;
        ay_smooth = alpha * temp_ay + (1.0 - alpha) * ay_smooth;
    }

    if (fabs(ax_smooth) < smoothDeadzone) ax_smooth = 0;
    if (fabs(ay_smooth) < smoothDeadzone) ay_smooth = 0;

    dx_p = ax_smooth * linearSens;
    dy_p = ay_smooth * linearSens;
}

void getClick() {
    double temp_ax2 = ax2;
    double temp_ay2 = ay2;
    double temp_az2 = az2;

    float accelMagnitude = sqrt(temp_ax2 * temp_ax2 + temp_ay2 * temp_ay2 + temp_az2 * temp_az2);
    static unsigned long lastTriggerTime = 0;
    static unsigned long firstClickTime = 0;
    static bool waitingForSecondClick = false;
    unsigned long currentTime = millis();

    if (accelMagnitude > CLICK_ACCEL_THRESHOLD && (currentTime - lastTriggerTime > DEBOUNCE_TIME)) {
        lastTriggerTime = currentTime;
        if (!waitingForSecondClick) {
            waitingForSecondClick = true;
            firstClickTime = currentTime;
        } else {
            if (currentTime - firstClickTime <= DOUBLE_CLICK_WINDOW) {
                bleMouse.click(MOUSE_LEFT);
                delay(50);
                bleMouse.click(MOUSE_LEFT);
                waitingForSecondClick = false;
            }
        }
    }
    if (waitingForSecondClick && (currentTime - firstClickTime > DOUBLE_CLICK_WINDOW)) {
        bleMouse.click(MOUSE_LEFT);
        waitingForSecondClick = false;
    }
}

void readAccel(double &ax_p, double &ay_p, double &az_p) {
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x32);
    if (Wire.endTransmission(false) != 0) return;
    if (Wire.requestFrom(ADXL345_ADDR, 6, true) != 6) return;
    int16_t x = Wire.read() | (Wire.read() << 8);
    int16_t y = Wire.read() | (Wire.read() << 8);
    int16_t z = Wire.read() | (Wire.read() << 8);
    ax_p = x * 0.0039; ay_p = y * 0.0039; az_p = z * 0.0039;
}

void readGyro(double &gx_p, double &gy_p, double &gz_p) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    if (Wire.endTransmission(false) != 0) return;
    if (Wire.requestFrom(MPU6050_ADDR, 6, true) != 6) return;
    gx_p = (int16_t)(Wire.read() << 8 | Wire.read());
    gy_p = (int16_t)(Wire.read() << 8 | Wire.read());
    gz_p = (int16_t)(Wire.read() << 8 | Wire.read());
}

void readAccelMouse(double &ax_p, double &ay_p, double &az_p) {
    Wire.beginTransmission(MPU6050_ADDR2);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return;
    if (Wire.requestFrom(MPU6050_ADDR2, 6, true) != 6) return;
    ax_p = (int16_t)(Wire.read() << 8 | Wire.read());
    ay_p = (int16_t)(Wire.read() << 8 | Wire.read());
    az_p = (int16_t)(Wire.read() << 8 | Wire.read());
}

void calibrateAll() {
    const int N = 500;
    double Sax = 0, Say = 0, Saz = 0;
    double Sgx = 0, Sgy = 0, Sgz = 0;
    for (int i = 0; i < N; i++) {
        double ax_c, ay_c, az_c, gx_c, gy_c, gz_c;
        readAccel(ax_c, ay_c, az_c);
        readGyro(gx_c, gy_c, gz_c);
        Sax += ax_c; Say += ay_c; Saz += az_c;
        Sgx += gx_c; Sgy += gy_c; Sgz += gz_c;
        delay(5);
    }
    ax_off = Sax / N;
    ay_off = Say / N;
    az_off = Saz / N - 1.0;
    gx_off = Sgx / N / 131.0;
    gy_off = Sgy / N / 131.0;
    gz_off = Sgz / N / 131.0;
}
