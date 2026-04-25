#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MCP4725.h>

#define SDA_1 33
#define SCL_1 32

#define eBrake 15
#define directionR 4
#define directionL 16
#define weapon 22

int gear = 1;

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

Adafruit_MCP4725 right;
Adafruit_MCP4725 left;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

constexpr int kSteerAxisMin = -512;
constexpr int kSteerAxisMax = 512;
constexpr int kSteerDeadzone = 24;
constexpr int kSteerScale = 512;

constexpr uint16_t kButtonEBrake = 0x0004;
constexpr uint16_t kButtonGearUp = 0x0020;
constexpr uint16_t kButtonGearDown = 0x0010;
constexpr int kGearMin = 1;
constexpr int kGearMax = 3;
constexpr unsigned long kGearDebounceMs = 200;

void setEBrakeGround(bool active) {
    static bool lastActive = false;
    if (active == lastActive) {
        return;
    }
    lastActive = active;

    // Open-drain mode: LOW grounds the line, HIGH releases it (floating).
    digitalWrite(eBrake, active ? LOW : HIGH);
}

void setDirectionGround(bool leftActive, bool rightActive) {
    static bool lastLeftActive = false;
    static bool lastRightActive = false;
    if (leftActive == lastLeftActive && rightActive == lastRightActive) {
        return;
    }
    lastLeftActive = leftActive;
    lastRightActive = rightActive;

    // Open-drain mode: LOW grounds the line, HIGH releases it (floating).
    digitalWrite(directionL, leftActive ? LOW : HIGH);
    digitalWrite(directionR, rightActive ? LOW : HIGH);
}

int processSteeringInput(int rawSteer, int controllerIndex) {
    // Keep a separate filter state per controller to avoid cross-controller mixing.
    static int filteredSteer[BP32_MAX_GAMEPADS] = {0};
    const int idx = constrain(controllerIndex, 0, BP32_MAX_GAMEPADS - 1);

    // Light filtering smooths jitter from noisy analog sticks.
    filteredSteer[idx] = (filteredSteer[idx] * 3 + rawSteer) / 4;

    int centered = constrain(filteredSteer[idx], kSteerAxisMin, kSteerAxisMax);

    if (abs(centered) <= kSteerDeadzone) {
        return 0;
    }

    if (centered > 0) {
        centered -= kSteerDeadzone;
    } else {
        centered += kSteerDeadzone;
    }

    // Cubic response keeps fine control near center and stronger turn at edges.
    const long cubic = (long)centered * centered * centered;
    return (int)(cubic / (kSteerAxisMax * kSteerAxisMax));
}

void updateGear(uint16_t buttons, int controllerIndex) {
    static unsigned long lastChangeMs[BP32_MAX_GAMEPADS] = {0};
    static bool lastUpPressed[BP32_MAX_GAMEPADS] = {false};
    static bool lastDownPressed[BP32_MAX_GAMEPADS] = {false};

    const int idx = constrain(controllerIndex, 0, BP32_MAX_GAMEPADS - 1);
    const bool upPressed = (buttons & kButtonGearUp) != 0;
    const bool downPressed = (buttons & kButtonGearDown) != 0;
    const unsigned long nowMs = millis();
    const bool debounceElapsed = (nowMs - lastChangeMs[idx]) >= kGearDebounceMs;

    if (upPressed && !lastUpPressed[idx] && debounceElapsed && gear < kGearMax) {
        gear++;
        lastChangeMs[idx] = nowMs;
    } else if (downPressed && !lastDownPressed[idx] && debounceElapsed && gear > kGearMin) {
        gear--;
        lastChangeMs[idx] = nowMs;
    }

    lastUpPressed[idx] = upPressed;
    lastDownPressed[idx] = downPressed;
}

// ================= Controller callbacks =================

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Controller connected: %d\n", i);
            myControllers[i] = ctl;
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Controller disconnected: %d\n", i);
            myControllers[i] = nullptr;
            return;
        }
    }
}


// ================= Gamepad processing =================

void processGamepad(ControllerPtr ctl) {
    const uint16_t buttons = ctl->buttons();
    const int throttleValue = ctl->throttle();
    const int brakeValue = ctl->brake();
    const int steerValue = processSteeringInput(ctl->axisX(), ctl->index());
    const bool eBrakePressed = (buttons == kButtonEBrake);

    setEBrakeGround(eBrakePressed);

    int driveValue = 0;

    // Throttle has priority if both throttle and brake are pressed.
    if (throttleValue > 0) {
        // Forward: left direction pin grounded, right direction pin disconnected.
        setDirectionGround(true, false);
        driveValue = throttleValue;
    } else if (brakeValue > 0) {
        // Reverse: right direction pin grounded, left direction pin disconnected.
        setDirectionGround(false, true);
        driveValue = brakeValue;
    } else {
        // No direction selected: disconnect both pins.
        setDirectionGround(false, false);
        driveValue = 0;
    }

    const int baseDrive = driveValue * gear;
    const int turnDelta = (steerValue * baseDrive) / kSteerScale;

    left.setVoltage(constrain(baseDrive - turnDelta, 0, 4095), false);
    right.setVoltage(constrain(baseDrive + turnDelta, 0, 4095), false);


    updateGear(buttons, ctl->index());
    


    // Serial.printf(
    //     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    //     "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    //     ctl->index(),        // Controller Index
    //     ctl->dpad(),         // D-pad
    //     ctl->buttons(),      // bitmask of pressed buttons
    //     ctl->axisX(),        // (-511 - 512) left X Axis
    //     ctl->axisY(),        // (-511 - 512) left Y axis
    //     ctl->axisRX(),       // (-511 - 512) right X axis
    //     ctl->axisRY(),       // (-511 - 512) right Y axis
    //     ctl->brake(),        // (0 - 1023): brake button
    //     ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    //     ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    //     ctl->gyroX(),        // Gyro X
    //     ctl->gyroY(),        // Gyro Y
    //     ctl->gyroZ(),        // Gyro Z
    //     ctl->accelX(),       // Accelerometer X
    //     ctl->accelY(),       // Accelerometer Y
    //     ctl->accelZ()        // Accelerometer Z
    // );



}

void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            if (ctl->isGamepad()) {
                processGamepad(ctl);
            }
        }
    }
}

// ================= Setup =================

void setup() {


Serial.begin(115200);

BP32.setup(&onConnectedController, &onDisconnectedController);
BP32.forgetBluetoothKeys();
BP32.enableVirtualDevice(false);

pinMode(eBrake, OUTPUT_OPEN_DRAIN);
pinMode(directionL, OUTPUT_OPEN_DRAIN);
pinMode(directionR, OUTPUT_OPEN_DRAIN);
pinMode(weapon, OUTPUT);

setEBrakeGround(false);
setDirectionGround(false, false);

I2Cone.begin(SDA_1, SCL_1, 100000);
//I2Ctwo.begin(SDA_2, SCL_2, 100000);

bool status = left.begin(0x60, &I2Cone);
bool status1 = right.begin(0x61, &I2Cone);

if (!status) Serial.println("Left DAC not found!");
if (!status1) Serial.println("Right DAC not found!");

left.setVoltage(0, false);
right.setVoltage(0, false);


}

// ================= Loop =================

void loop() {


bool dataUpdated = BP32.update();

if (dataUpdated) {
    processControllers();
}


delay(10);


}
