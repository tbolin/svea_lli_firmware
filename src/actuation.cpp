#include "actuation.h"

namespace actuate{

act_t Actuation::steering() const{
  return act_values_[STEER_IX];
}

act_t Actuation::velocity() const{
  return act_values_[VEL_IX];
}

act_t Actuation::gear() const{
  return act_values_[GEAR_IX];
}

act_t Actuation::f_diff() const{
  return act_values_[F_DIFF_IX];
}

act_t Actuation::r_diff() const{
  return act_values_[R_DIFF_IX];
}

void Actuation::steering(act_t value){
  act_values_[STEER_IX] = value;
}

void Actuation::velocity(act_t value){
  act_values_[VEL_IX] = value;
}

void Actuation::gear(act_t value){
  act_values_[GEAR_IX] = value;
}

void Actuation::f_diff(act_t value){
  act_values_[F_DIFF_IX] = value;
}

void Actuation::r_diff(act_t value){
  act_values_[R_DIFF_IX] = value;
}


} //namespace actuate