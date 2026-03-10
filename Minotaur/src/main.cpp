#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MCP4725.h>

#define SDA_1 26
#define SCL_1 27

#define SDA_2 33
#define SCL_2 32

#define eBrake 15
#define direction 12
#define weapon 22
#define gear 1

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

Adafruit_MCP4725 right;
Adafruit_MCP4725 left;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

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
    
    left.setVoltage(constrain(ctl->throttle() * gear - ctl->axisX(), 0, 4095), false);
    right.setVoltage(constrain(ctl->throttle() * gear + ctl->axisX(), 0, 4095), false);


    if (ctl->a()) {
        digitalWrite(weapon, HIGH);
    } else {
        digitalWrite(weapon, LOW);
    }

    if (ctl->button_shoulder_l && gear < 4) {
        gear++;
    } else if (ctl->button_shoulder_r && gear > 1) {
        gear--;
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
pinMode(direction, OUTPUT);
pinMode(weapon, OUTPUT);

I2Cone.begin(SDA_1, SCL_1, 100000);
I2Ctwo.begin(SDA_2, SCL_2, 100000);

bool status = left.begin(0x60, &I2Cone);
bool status1 = right.begin(0x61, &I2Ctwo);

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
