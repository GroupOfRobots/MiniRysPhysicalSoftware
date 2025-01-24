#pragma once

#include <chrono>

namespace pid_regulator
{

class PIDRegulator
{
public:
  PIDRegulator(float t, float k, float ti, float td);
  float getK() const;
  float getTi() const;
  float getTd() const;
  void setK(float newK);
  void setTi(float newTi);
  void setTd(float newTd);
  float pid(float y, float y_zad);
  float pid_aw(float y, float y_zad, float Tv, float max);
  void clear();

private:
  float K;
  float Ti;
  float Td;
  float T;
  float e_past = 0.0f;
  float u_past = 0.0f;
  float calcUdUp(float e) const;
};

}  // namespace pid_regulator
