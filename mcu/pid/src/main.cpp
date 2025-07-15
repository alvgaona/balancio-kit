#include "Wire.h"
#include "config.h"
#include "dc_motor.h"
#include "imu.h"
#include "pid.h"
#include "timer.h"

volatile bool controlFlag = false;

float currentPitch = 0.0;
float targetPitch = 0.0;
const float pitchLimit = 0.5;

float currentYaw = 0.0;
float targetYaw = 0.0;

int timeControl;

bool prevState = false;
bool currState = false;

IMU *imu;
Motors *motors;
PID *yawControl;
PID *pitchControl;

float pwm;
float yawOutput;
float inputPoint[2];

bool enabled = false;

void setup() {
  // UART PC connection
  Serial.begin(115200);
  // Motor initialization
  motors = new Motors();
  motors->init();

  // IMU init
  imu = new IMU();
  imu->setup();

  // Timer init. (ISR at 1/LOOP_PERIOD Hz)
  timerInit();

  // Controllers init
  pitchControl = new PID(KP_PITCH, KI_PITCH, KD_PITCH, 5.0);
  yawControl = new PID(KP_YAW, KI_YAW, KD_YAW, 5.0);

  // Angle initialization
  currentPitch = -getAccelPitch();
  timeControl = micros();

  pinMode(PIN_LEDR, OUTPUT);
  pinMode(PIN_LEDG, OUTPUT);
  pinMode(PIN_LEDB, OUTPUT);
  pinMode(PIN_ENABLE_BUTTON, INPUT_PULLUP);
}

// ISR at 1/LOOP_PERIOD Hz
void IRAM_ATTR onTime() { controlFlag = true; }

void loop() {
  if (!controlFlag) {
    return;
  }
  controlFlag = false;

  if (digitalRead(PIN_ENABLE_BUTTON) == 0) {
    enabled = !enabled;
    delay(200);
  }
  if (!enabled) {
    stopMotor();
    digitalWrite(PIN_LEDR, LOW);
    return;
  }

  // Get pitch and yaw angles.
  currentPitch = imu->updatePitch(currentPitch);
  currentYaw = imu->updateYaw(currentYaw);

  if (currentPitch > pitchLimit || currentPitch < -pitchLimit) {
    currState = false;
    stopMotor();
    targetYaw = currentYaw;

    // Reset PID
    pitchControl->reset(0.0);
    yawControl->reset(targetYaw);
    digitalWrite(PIN_LEDR, HIGH);
  } else {
    currState = true;

    pwm = pitchControl->update(currentPitch, targetPitch);
    yawOutput = yawControl->update(currentYaw, targetYaw);

    motors->leftMotor(pwm + int(yawOutput));  // -255 to 255
    motors->rightMotor(pwm - int(yawOutput)); // -255 to 255

    digitalWrite(PIN_LEDR, LOW);
  }
  if (true) {
    // Print relevant data.
    Serial.print("  targetPitch: ");
    Serial.print(targetPitch);
    Serial.print("  targetYaw: ");
    Serial.print(targetYaw);
    Serial.print(" pitch: ");
    Serial.print(currentPitch);
    Serial.print(" yaw: ");
    Serial.print(currentYaw);
    Serial.print(" Loop time: ");
    Serial.println(micros() - timeControl);

    timeControl = micros();
  }

  if (prevState != currState) {
    stoppedCommand(currState);
  }
  prevState = currState;
}
