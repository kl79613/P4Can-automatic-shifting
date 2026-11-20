#pragma once

#include "gear_shift_pure.h"

// Juniper V13/V14 uses only the vehicle pair or driver steering selection.
// Its AUTO mode must not silently fall back to the V12 distance inference.
inline bool gearShiftVehiclePairOnly(bool juniper,
                                     gearshift::Protocol protocol) {
  return protocol == gearshift::Protocol::V13V14 ||
      (juniper && protocol == gearshift::Protocol::Auto);
}

// Juniper hardware must use the V13/V14 0x678 encoder. AUTO must not select
// V12 merely because a 0x229 template happened to be observed first.
inline gearshift::Protocol gearShiftEffectiveProtocol(
    bool juniper, gearshift::Protocol protocol) {
  return juniper ? gearshift::Protocol::V13V14 : protocol;
}

// Legacy carries the 2021 Model Y field calibration. Juniper uses the same
// requested sign mapping for V13/V14: negative relative motion is left, left
// requests D, and right requests R.
inline bool gearShiftParkSteeringCalibrated(bool juniper, bool modelYLegacy,
                                            gearshift::Protocol protocol) {
  return protocol == gearshift::Protocol::V12 || modelYLegacy ||
      (juniper && protocol != gearshift::Protocol::V12);
}
