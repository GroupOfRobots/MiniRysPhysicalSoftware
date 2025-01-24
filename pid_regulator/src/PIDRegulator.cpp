#include <pid_regulator/PIDRegulator.hpp>

#include <algorithm>

namespace pid_regulator
{

PIDRegulator::PIDRegulator(float t, float k = 0.0f, float ti = 10000000.0f, float td = 0.0f)
: K(k), Ti(ti), Td(td), T(t)
{
}

float PIDRegulator::pid(float y, float y_zad)
{
  float e = y_zad - y;
  float ui = this->u_past + this->K / this->Ti * this->T * (this->e_past + e) / 2;
  float u = calcUdUp(e) + ui;
  this->u_past = ui;
  this->e_past = e;
  return u;
}

float PIDRegulator::pid_aw(float y, float y_zad, float Tv, float max)
{
  float uw = 0.0f;
  if (this->u_past > max) {
    uw = max;
  } else if (this->u_past < -max) {
    uw = -max;
  } else {
    uw = this->u_past;
  }

  float e = y_zad - y;
  // float up = this->K*e;
  float ui = this->u_past + this->K / this->Ti * this->T * (this->e_past + e) / 2 +
             this->T / Tv * (uw - this->u_past);
  // float ud = this->K*this->Td*(e-this->e_past)/this->T;
  float u = calcUdUp(e) + ui;
  this->u_past = ui;
  this->e_past = e;
  return u;
}

float PIDRegulator::calcUdUp(float e) const
{
  float up = this->K * e;
  float ud = this->K * this->Td * (e - this->e_past) / this->T;
  return up + ud;
}

void PIDRegulator::clear()
{
  this->e_past = 0.0f;
  this->u_past = 0.0f;
}

float PIDRegulator::getK() const
{
  return this->K;
}

float PIDRegulator::getTi() const
{
  return this->Ti;
}

float PIDRegulator::getTd() const
{
  return this->Td;
}

void PIDRegulator::setK(float newK)
{
  this->K = newK;
}

void PIDRegulator::setTi(float newTi)
{
  this->Ti = newTi;
}

void PIDRegulator::setTd(float newTd)
{
  this->Td = newTd;
}

}  // namespace pid_regulator
