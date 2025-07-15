#pragma once

class State {
public:
  State();
  ~State();

  float pitch() const { return pitch_; };
  float yaw() const { return yaw_; };

private:
  float pitch_;
  float yaw_;
};
