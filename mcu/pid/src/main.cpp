#include "config.h"
#include "dc_motor.h"
#include "imu.h"
#include "pid.h"
#include "state.h"
#include "timer.h"

int timeControl;

State *state;
Imu *imu;
Motors *motors;
PID *yawControl;
PID *pitchControl;

bool enabled = false;

struct Target {
  float pitch = 0.0;
  float yaw = 0.5;

  void reset() {
    pitch = 0.0;
    yaw = 0.5;
  }
};

Target target;

void setup() {
  // UART PC connection
  Serial.begin(115200);

  motors = new Motors();
  motors->init();

  imu = new Imu();
  imu->setup();

  state = new State();
  state->pitch = -imu->measurePitch();

  timerInit();

  pitchControl = new PID(KP_PITCH, KI_PITCH, KD_PITCH, 5.0);
  yawControl = new PID(KP_YAW, KI_YAW, KD_YAW, 5.0);

  // Angle initialization
  timeControl = micros();

  pinMode(PIN_LEDR, OUTPUT);
  pinMode(PIN_LEDG, OUTPUT);
  pinMode(PIN_LEDB, OUTPUT);
  pinMode(PIN_ENABLE_BUTTON, INPUT_PULLUP);
}

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
    motors->stop();
    digitalWrite(PIN_LEDR, LOW);
    return;
  }

  state->pitch = imu->updatePitch(state->pitch);
  state->yaw = imu->updateYaw(state->yaw);

  if (!state->isPitchWithinRange()) {
    motors->stop();
    target.yaw = state->yaw;

    // Reset PID
    pitchControl->reset(0.0);
    yawControl->reset(state->yaw);
    digitalWrite(PIN_LEDR, HIGH);
  } else {
    const float pwm = pitchControl->update(state->pitch, target.pitch);
    const float yawOutput = yawControl->update(state->yaw, target.yaw);

    motors->leftMotor(pwm + int(yawOutput));  // -255 to 255
    motors->rightMotor(pwm - int(yawOutput)); // -255 to 255

    digitalWrite(PIN_LEDR, LOW);
  }

  Serial.print("  targetPitch: ");
  Serial.print(target.pitch);
  Serial.print("  targetYaw: ");
  Serial.print(target.yaw);
  Serial.print(" pitch: ");
  Serial.print(state->pitch);
  Serial.print(" yaw: ");
  Serial.print(state->yaw);
  Serial.print(" Loop time: ");
  Serial.println(micros() - timeControl);

  timeControl = micros();
}
