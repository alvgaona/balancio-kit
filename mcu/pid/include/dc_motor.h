#pragma once

class Motors {
public:
  Motors();
  ~Motors();

  void init() const;
  static void stop();

  /**
   * Set the left motor speed in foward direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [0, 255]
   */
  static void fwdLeftMotor(int pwm);

  /**
   * Set the right motor speed in foward direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [0, 255]
   */
  static void fwdRightMotor(int pwm);

  /**
   * Set the left motor speed in backward direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [0, 255]
   */
  static void bwdLeftMotor(int pwm);

  /**
   * Set the right motor speed in backward direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [0, 255]
   */
  static void bwdRightMotor(int pwm);

  /**
   * Set the left motor speed and direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [-255, 255].
   *                     Positive (+) values --> Foward
   *                     Negative (-) values --> Backward
   */
  static void leftMotor(int pwm);

  /**
   * Set the right motor speed and direction.
   *
   * @param  {int} pwm : Motor PWM. Values in range [-255, 255].
   *                     Positive (+) values --> Foward
   *                     Negative (-) values --> Backward
   */
  static void rightMotor(int pwm);

private:
  int pwmFrequency = 200;
};
