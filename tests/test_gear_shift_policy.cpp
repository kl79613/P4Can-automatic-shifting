#include <cstdint>
#include <iostream>

#include "gear_shift_policy_pure.h"

int main() {
  unsigned checks = 0;
  const bool juniper = true, other = false, legacy = true, notLegacy = false;
  using gearshift::Protocol;

  if (!gearShiftVehiclePairOnly(juniper, Protocol::V13V14) ||
      !gearShiftVehiclePairOnly(juniper, Protocol::Auto) ||
      gearShiftVehiclePairOnly(juniper, Protocol::V12) ||
      gearShiftVehiclePairOnly(other, Protocol::Auto)) return 1;
  ++checks;

  if (!gearShiftParkSteeringCalibrated(juniper, notLegacy, Protocol::V13V14) ||
      !gearShiftParkSteeringCalibrated(juniper, notLegacy, Protocol::Auto) ||
      !gearShiftParkSteeringCalibrated(juniper, notLegacy, Protocol::V12) ||
      !gearShiftParkSteeringCalibrated(other, legacy, Protocol::V13V14) ||
      gearShiftParkSteeringCalibrated(other, notLegacy, Protocol::V13V14)) return 2;
  ++checks;

  if (gearShiftEffectiveProtocol(juniper, Protocol::Auto) != Protocol::V13V14 ||
      gearShiftEffectiveProtocol(juniper, Protocol::V12) != Protocol::V13V14 ||
      gearShiftEffectiveProtocol(juniper, Protocol::V13V14) != Protocol::V13V14 ||
      gearShiftEffectiveProtocol(other, Protocol::Auto) != Protocol::Auto ||
      gearShiftEffectiveProtocol(other, Protocol::V12) != Protocol::V12 ||
      gearShiftEffectiveProtocol(other, Protocol::V13V14) != Protocol::V13V14)
    return 3;
  ++checks;

  std::cout << "PASS: " << checks << " Juniper V13/V14 gear-source policy checks\n";
  return 0;
}
