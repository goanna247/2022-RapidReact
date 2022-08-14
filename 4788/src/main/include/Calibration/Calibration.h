#pragma once

class CalibrationMode {
 public:
  virtual void OnUpdate(double dt) = 0;
};