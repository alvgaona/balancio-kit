#pragma once

class Motors {
public:
  Motors();
  ~Motors();

  void init();
  void stop();

  /**
   * Set the left motor speed in foward direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [0, 255]
   */
  void fwdLeftMotor(int pwm);

  /**
   * Set the right motor speed in foward direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [0, 255]
   */
  void fwdRightMotor(int pwm);

  /**
   * Set the left motor speed in backward direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [0, 255]
   */
  void bwdLeftMotor(int pwm);

  /**
   * Set the right motor speed in backward direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [0, 255]
   */
  void bwdRightMotor(int pwm);

  /**
   * Set the left motor speed and direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [-255, 255].
   *                     Positive (+) values --> Foward
   *                     Negative (-) values --> Backward
   */
  void leftMotor(int pwm);

  /**
   * Set the right motor speed and direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [-255, 255].
   *                     Positive (+) values --> Foward
   *                     Negative (-) values --> Backward
   */
  void rightMotor(int pwm);

private:
  int pwmFrequency = 200;
};
