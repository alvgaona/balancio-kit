#pragma once

class State {
public:
  State();
  ~State();

  float pitch;
  float yaw;

  bool isPitchWithinRange() const {
    return pitch >= -pitchLimit_ && pitch <= pitchLimit_;
  }

private:
  float pitchLimit_;
};
