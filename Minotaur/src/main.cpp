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

int processSteeringInput(int rawSteer) {
    // Light filtering smooths jitter from noisy analog sticks.
    static int filteredSteer = 0;
    filteredSteer = (filteredSteer * 3 + rawSteer) / 4;

    int centered = constrain(filteredSteer, kSteerAxisMin, kSteerAxisMax);

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
    const int throttleValue = ctl->throttle();
    const int brakeValue = ctl->brake();
    const int steerValue = processSteeringInput(ctl->axisX());

    int driveValue = 0;

    // Throttle has priority if both throttle and brake are pressed.
    if (throttleValue > 0) {
        digitalWrite(directionL, LOW);
        digitalWrite(directionR, HIGH);
        driveValue = throttleValue;
    } else if (brakeValue > 0) {
        digitalWrite(directionL, HIGH);
        digitalWrite(directionR, LOW);
        driveValue = brakeValue;
    } else {
        driveValue = 0;
    }

    const int baseDrive = driveValue * gear;
    const int turnDelta = (steerValue * baseDrive) / 512;

    left.setVoltage(constrain(baseDrive - turnDelta, 0, 4095), false);
    right.setVoltage(constrain(baseDrive + turnDelta, 0, 4095), false);


    if (ctl->buttons() == 0x0020 && gear < 3) {
        gear++;
        delay(200); // Debounce delay
    } else if (ctl->buttons() == 0x0010 && gear > 1) {
        gear--;
        delay(200); // Debounce delay
    }
    


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

pinMode(eBrake, OUTPUT);
pinMode(directionL, OUTPUT);
pinMode(directionR, OUTPUT);
pinMode(weapon, OUTPUT);

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
