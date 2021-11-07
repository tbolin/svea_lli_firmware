#ifndef SVEA_ACTUATION
#define SVEA_ACTUATION

#include <cstdint>
#include <array>

namespace actuate{

typedef int8_t act_t;
/*
 * @defgroup ActuationConstants Actuation constants
 */
 /*@{*/

static constexpr act_t ACTUATION_MIN      = -127;    //!< Minimum actuation value
static constexpr act_t ACTUATION_NEUTRAL  = 0;       //!< Neutral actuation value
static constexpr act_t ACTUATION_MAX      = 127;     //!< Maximum actuation value
static constexpr act_t ACTUATION_PREVIOUS = -128;    //!< Actuation valuethat will just use the orevious value

/*@}*/


class Actuation {

public:
  static constexpr size_t STEER_IX = 0;
  static constexpr size_t VEL_IX = 1;
  static constexpr size_t GEAR_IX = 2;
  static constexpr size_t F_DIFF_IX = 3;
  static constexpr size_t R_DIFF_IX = 4;
  static constexpr size_t NUMEL = 5;
  static constexpr std::array<act_t, NUMEL> DEFAULT = {
    ACTUATION_PREVIOUS,
    ACTUATION_PREVIOUS,
    ACTUATION_PREVIOUS,
    ACTUATION_PREVIOUS,
    ACTUATION_PREVIOUS
  };

  constexpr Actuation(): act_values_(DEFAULT) {}
  constexpr Actuation(const std::array<act_t, NUMEL>& a) : act_values_(a){}
  constexpr Actuation(act_t steering,
            act_t velocity,
            act_t gear,
            act_t f_diff,
            act_t r_diff) : act_values_({steering, velocity, gear, f_diff, r_diff}) {}
  Actuation(const Actuation& other) : act_values_(other.act_values_) {}

  act_t steering() const;
  act_t velocity() const;
  act_t gear() const;
  act_t f_diff() const;
  act_t r_diff() const;

  void steering(act_t value);
  void velocity(act_t value);
  void gear(act_t value);
  void f_diff(act_t value);
  void r_diff(act_t value);

private:
  std::array<act_t, NUMEL> act_values_;

public:

  act_t& at(size_t ix){
    return act_values_.at(ix);
  }

  act_t at(size_t ix) const{
    return act_values_.at(ix);
  }

  act_t& operator [] (size_t ix) {
    return act_values_[ix];
  }

  act_t operator [] (size_t ix) const {
    return act_values_[ix];
  }

  static const Actuation initial(){
    return Actuation(
        ACTUATION_NEUTRAL,
        ACTUATION_NEUTRAL,
        ACTUATION_MIN,
        ACTUATION_MIN,
        ACTUATION_MIN);
  }
  static constexpr Actuation emergency();

};

} //namespace actuate
#endif //SVEA_ACTUATION