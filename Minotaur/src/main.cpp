#include <Arduino.h>
#include <Bluepad32.h>
#include <bt/uni_bt_allowlist.h>
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

struct AllowedControllerMac {
    bd_addr_t address;
    bool enabled;
};

// Replace the MAC address below with the controller(s) you want to allow.
static const AllowedControllerMac kAllowedControllerMacs[] = {
    {{0xE4, 0x17, 0xD8, 0x34, 0xA2, 0x10}, true},
};

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

void configureControllerMacWhitelist() {
    bool hasEnabledEntries = false;

    uni_bt_allowlist_remove_all();

    for (const auto& entry : kAllowedControllerMacs) {
        if (!entry.enabled) {
            continue;
        }

        bd_addr_t address;
        memcpy(address, entry.address, sizeof(address));
        uni_bt_allowlist_add_addr(address);
        hasEnabledEntries = true;
    }

    uni_bt_allowlist_set_enabled(hasEnabledEntries);

    if (!hasEnabledEntries) {
        Serial.println("Bluetooth allowlist is configured, but no MAC addresses are enabled yet.");
    }
}

void setEBrakeGround(bool active) {
    static bool lastActive = false;
    if (active == lastActive) {
        return;
    }
    lastActive = active;

    // Standard output mode: actively drive LOW/HIGH.
    digitalWrite(eBrake, active ? LOW : HIGH);
}

void setDriveDirection(bool reverse) {
    static bool lastReverse = false;
    if (reverse == lastReverse) {
        return;
    }
    lastReverse = reverse;

    digitalWrite(directionL, reverse ? HIGH : LOW);
    digitalWrite(directionR, reverse ? LOW : HIGH);
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
    const bool bothThrottleAndBrake = (throttleValue > 0) && (brakeValue > 0);
    const bool eBrakePressed = ((buttons & kButtonEBrake) != 0) || bothThrottleAndBrake;

    setEBrakeGround(eBrakePressed);

    if (bothThrottleAndBrake) {
        left.setVoltage(0, false);
        right.setVoltage(0, false);
    } else {
        const bool reverse = (brakeValue > 0) && (throttleValue == 0);
        const int driveValue = (throttleValue > 0) ? throttleValue : brakeValue;

        if (driveValue > 0) {
        setDriveDirection(reverse);

        const int baseDrive = driveValue * gear;
        const int turnDelta = (-steerValue * baseDrive) / kSteerScale;

        left.setVoltage(constrain(baseDrive - turnDelta, 0, 4095), false);
        right.setVoltage(constrain(baseDrive + turnDelta, 0, 4095), false);
        } else {
        left.setVoltage(0, false);
        right.setVoltage(0, false);
        }
    }


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
configureControllerMacWhitelist();
BP32.forgetBluetoothKeys();
BP32.enableVirtualDevice(false);

pinMode(eBrake, OUTPUT);
pinMode(directionL, OUTPUT);
pinMode(directionR, OUTPUT);
pinMode(weapon, OUTPUT);
digitalWrite(directionL, HIGH);
digitalWrite(directionR, LOW);

setEBrakeGround(false);
setDriveDirection(false);

I2Cone.begin(SDA_1, SCL_1, 100000);

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
