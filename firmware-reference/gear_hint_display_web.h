#pragma once

// GET-only, read-only serializer. No settings, preview state or CAN output.
static String gearHintDisplayToJson() {
  gearhint::DisplayHintObserver copy;
  gearhintv12live::PrearmResult prearm;
  gearshift::Config shiftCfg;
  uint32_t now;
  portENTER_CRITICAL(&gearHintDisplayMux);
  copy = gearHintDisplayObserver;
  now = static_cast<uint32_t>(millis());
  portENTER_CRITICAL(&gearShiftMux);
  prearm = gearShiftPrearmSnapshot(now);
  shiftCfg = gearShiftConfig;
  portEXIT_CRITICAL(&gearShiftMux);
  portEXIT_CRITICAL(&gearHintDisplayMux);
  const bool supported = activeCanAIsBody() && activeCanBIsChassis();
  const bool available = !vehicleProfileSetupMode && !vehicleProfileNvsError &&
      !canSubsystemBusy && p4CanOnline[P4_CHASSIS] && p4CanOnline[P4_PARTY] &&
      p4CanState[P4_CHASSIS] == TWAI_STATE_RUNNING &&
      p4CanState[P4_PARTY] == TWAI_STATE_RUNNING;
  const gearhint::HintResult result = prearm.speedCut ? gearhint::HintResult{} : copy.result(now, available && supported);
  String json;
  json.reserve(1000);
  JsonWriterArduino jw(json);
  jw.boolean("ok", true);
  jw.boolean("displayOnly", true);
  jw.boolean("controlPermission", false);
  jw.boolean("brakeRequired", true);
  jw.string("brakeRule", "DRIVER_STATE_2");
  jw.boolean("brakeApplyRequired", false);
  jw.string("brakeState", copy.brakeState(now));
  jw.i32("driverBrakeRaw", copy.brake.seen && copy.brake.dlc == 8 ? (copy.brake.data[3] >> 5) & 3 : -1);
  jw.i32("brakeApplyRaw", copy.brake.seen && copy.brake.dlc == 8 ? copy.brake.data[3] >> 7 : -1);
  jw.boolean("protocolVerified", false);
  jw.boolean("profileSupported", supported);
  jw.string("observationScope", "P_D_R_FIELD_MATCH");
  jw.boolean("nonParkInterpretationExperimental", true);
  jw.string("directionField", "BYTE1_BITS4_6");
  jw.string("suggestion", gearhint::hintName(result.hint));
  jw.string("reason", supported ? gearhint::reasonName(result.reason) : "PROFILE_UNSUPPORTED");
  jw.u32("remainingMs", result.remainingMs);
  jw.i32("gearRaw", copy.gear.seen ? copy.gear.data[2] >> 5 : -1);
  jw.i32("requestRaw", copy.advice.seen ? copy.advice.data[3] & 3 : -1);
  jw.i32("directionRaw", copy.direction.seen ? gearhint::directionCode(copy.direction.data) : -1);
  jw.u32("parkSession", copy.parkSession);
  jw.u32("gearSession", copy.gearSession);
  jw.u32("brakeSession", copy.brakeSession);
  jw.beginObject("agesMs");
  if (copy.gear.seen) jw.u32("gear", gearhint::ageMs(copy.gear, now));
  else jw.raw("gear", "null");
  if (copy.advice.seen) jw.u32("advice", gearhint::ageMs(copy.advice, now));
  else jw.raw("advice", "null");
  if (copy.direction.seen) jw.u32("direction", gearhint::ageMs(copy.direction, now));
  else jw.raw("direction", "null");
  if (copy.brake.seen) jw.u32("brake", gearhint::ageMs(copy.brake, now));
  else jw.raw("brake", "null");
  jw.endObject();
  jw.beginObject("raw");
  jw.string("gear118", canFrameHexArduino(copy.gear.data, copy.gear.dlc));
  jw.string("advice255", canFrameHexArduino(copy.advice.data, copy.advice.dlc));
  jw.string("direction12B", canFrameHexArduino(copy.direction.data, copy.direction.dlc));
  jw.string("brake145", canFrameHexArduino(copy.brake.data, copy.brake.dlc));
  jw.endObject();
  jw.beginObject("prearm");
  jw.boolean("applied",shiftCfg.protocol!=gearshift::Protocol::V12);
  jw.boolean("ready",prearm.ready); jw.boolean("speedCut",prearm.speedCut);
  jw.string("candidate",gearhintv12live::candidateName(prearm.candidate));
  jw.string("source",gearhintv12live::sourceName(prearm.source));
  jw.boolean("sameGearIgnored",prearm.sameGearIgnored);jw.boolean("radarAvailable",prearm.radarAvailable);
  jw.boolean("radarSteeringRequired",prearm.radarSteeringRequired);jw.boolean("radarPriority",prearm.radarPriority);
  jw.string("reason",gearhintv12live::reasonName(prearm.reason));
  jw.string("rule",prearm.rule); jw.u32("stableForMs",prearm.stableForMs);
  jw.i32("baselineWheelDeciDeg",prearm.baselineWheelDeci);
  jw.i32("wheelDeltaDeciDeg",prearm.wheelDeltaDeci); jw.u32("brakeEpoch",prearm.brakeEpoch);
  jw.u32("remainingMs",prearm.remainingMs); jw.i32("wheelAngleDeciDeg",prearm.wheelAngleDeci);
  jw.endObject();
  jw.finish();
  return json;
}

static void httpGearHintDisplayStatus() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", gearHintDisplayToJson());
}

// X179's USB connector is a UART0 bridge, independent of Hardware CDC.
// Loop-only, one bounded snapshot per second; no RX commands or CAN writes.
static void gearHintUsbDebugTick() {
#if defined(P4CAN_BOARD_X179)
  static bool started = false;
  if (!started) {
    Serial0.setTxBufferSize(4096);
    Serial0.begin(115200, SERIAL_8N1, 38, 37);
    started = true;
  }
  String line = "P4GEAR " + gearHintDisplayToJson() + "\n";
  if (line.length() <= 2048 && Serial0.availableForWrite() >= static_cast<int>(line.length()))
    Serial0.write(reinterpret_cast<const uint8_t *>(line.c_str()), line.length());
#endif
}

static String gearHintV12RawFrameJson(const gearhintv12::Frame &f) {
  String out;
  out.reserve(160);
  JsonWriterArduino j(out);
  j.u32("sequence", f.sequence);
  j.u32("observedMs", f.observedMs);
  j.u32("bus", f.bus);
  j.u32("id", gearhintv12::kIds[f.idIndex]);
  j.u32("dlc", f.dlc);
  j.string("data", canFrameHexArduino(f.data, f.dlc));
  j.finish();
  return out;
}

static bool gearHintV12QueryU32(const char *key, uint32_t &value) {
  const String s = server.arg(key);
  if (!s.length() || s.length() > 10) return false;
  uint64_t n = 0;
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    n = n * 10 + static_cast<uint8_t>(s[i] - '0');
  }
  if (n > UINT32_MAX) return false;
  value = static_cast<uint32_t>(n); return true;
}

// Synchronous HTTP server: one static snapshot avoids a large task-stack copy.
// The receive lock is released before allocating/serializing the JSON response.
static void httpGearHintV12RawStatus() {
  static gearhintv12::RawObserver view;
  static const uint32_t streamId = esp_random();
  uint32_t now;
  portENTER_CRITICAL(&gearHintDisplayMux);
  view = gearHintV12RawObserver;
  now = static_cast<uint32_t>(millis());
  portEXIT_CRITICAL(&gearHintDisplayMux);
  uint32_t requestedStream = 0, requestedEpoch = 0, after = 0;
  const bool cursorValid = gearHintV12QueryU32("streamId", requestedStream) &&
      gearHintV12QueryU32("epoch", requestedEpoch) && gearHintV12QueryU32("after", after) &&
      requestedStream == streamId && requestedEpoch == view.epoch;
  const auto page = view.page(cursorValid, after);
  String out;
  out.reserve(30000);
  JsonWriterArduino j(out);
  j.boolean("ok", true);
  j.string("schema", "p4can.v12.raw-observation.v1");
  j.string("source", "LIVE_CAN_RAW");
  j.boolean("displayOnly", true);
  j.boolean("decoderVerified", false);
  j.boolean("geometryVerified", false);
  j.boolean("vehicleDecisionImplemented", false);
  j.string("suggestion", "NONE");
  j.string("decision", "INSUFFICIENT");
  j.string("reason", "INPUT_DEFINITION_UNVERIFIED");
  j.u32("nowMs", now); j.u32("streamId", streamId); j.u32("epoch", view.epoch);
  j.u32("sequence", view.sequence);
  j.u32("historyCapacity", gearhintv12::kHistoryCapacity);
  j.u32("captureRevision", 4);
  j.string("decisionEndpoint", "/api/gear-hint/v12/decision");
  j.u32("pageLimit", gearhintv12::kPageLimit);
  j.boolean("hasMore", page.hasMore);
  j.u32("historyOverwritten", view.overwritten);
  String buses = "[";
  for (uint8_t b = 0; b < gearhintv12::kBusCount; ++b) {
    if (b) buses += ',';
    String row;
    JsonWriterArduino r(row);
    r.u32("bus", b);
    r.boolean("online", p4CanOnline[b]);
    r.boolean("running", p4CanState[b] == TWAI_STATE_RUNNING);
    r.finish(); buses += row;
  }
  buses += ']'; j.raw("buses", buses);
  String latest = "[";
  bool first = true;
  for (uint8_t b = 0; b < gearhintv12::kBusCount; ++b) {
    for (uint8_t i = 0; i < gearhintv12::kIdCount; ++i) {
      // MUX1 reception must not replace or refresh a MUX0 buckle observation.
      const bool mux0 = gearhintv12::kIds[i] == 0x3C2 && view.buckleMux0[b].count;
      const auto &s = mux0 ? view.buckleMux0[b] : view.latest[b][i];
      if (!s.count) continue;
      if (!first) latest += ',';
      first = false;
      String row = gearHintV12RawFrameJson(s.frame);
      row.remove(row.length() - 1);
      JsonWriterArduino r(row, true);
      r.u32("count", s.count);
      r.u32("ageMs", now - s.frame.observedMs);
      const bool nonChassisSdiId = b != 1 &&
          (gearhintv12::kIds[i] == 0x20E || gearhintv12::kIds[i] == 0x22E);
      r.string("candidateName", nonChassisSdiId ? "UNCLASSIFIED_SAME_ID" : gearhintv12::candidateName(i));
      r.boolean("referenceDlcMatches", s.frame.dlc == gearhintv12::kReferenceDlcs[i]);
      r.boolean("mux0Only", mux0);
      const int gear = gearhintv12::referenceGearRaw(s.frame);
      const int brake = gearhintv12::referenceDriverBrakeRaw(s.frame);
      const int buckle = gearhintv12::referenceBuckleRaw(s.frame);
      if (gear >= 0) r.i32("referenceGearRaw", gear);
      if (brake >= 0) {
        r.i32("referenceDriverBrakeRaw", brake);
        r.i32("referenceBrakeApplyRaw", gearhintv12::referenceBrakeApplyRaw(s.frame));
      }
      if (buckle >= 0) r.i32("referenceBuckleRaw", buckle);
      if (b != 1 && (gearhintv12::kIds[i] == 0x20E || gearhintv12::kIds[i] == 0x22E))
        r.string("candidateNameNote", "NON_CHASSIS_ID_ONLY");
      uint16_t raw[6];
      if (gearhintv12::referenceSdiRaw(s.frame, raw)) {
        String values = "[";
        for (uint8_t probe = 0; probe < 6; ++probe) {
          if (probe) values += ',';
          values += raw[probe];
        }
        values += ']'; r.raw("referenceSdiRaw", values);
      }
      r.string("semanticValidity", "UNVERIFIED");
      r.finish(); latest += row;
    }
  }
  latest += ']'; j.raw("latest", latest);
  String events = "[";
  for (uint16_t i = 0; i < page.count; ++i) {
    if (i) events += ',';
    events += gearHintV12RawFrameJson(view.ordered(page.start + i));
  }
  events += ']'; j.raw("events", events);
  j.finish();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

// A GET computes only a display projection. Query options never mutate device
// settings, RX state, confirmation counters, NVS or any original feature.
static void httpGearHintV12DecisionStatus() {
  uint32_t aux=0,belt=1,maxKph=8,hardCut=10,turn=0,steerFallback=0,parkSteer=0,cooldown=1000,pDeg=30,drDeg=60,obstacleDeg=30,obstacleCm=100;
  const auto option=[](const char *key,uint32_t &value,uint32_t lo,uint32_t hi) {
    return !server.hasArg(key)||(gearHintV12QueryU32(key,value)&&value>=lo&&value<=hi);
  };
  if(!option("auxBus",aux,0,2)||!option("requireSeatbelt",belt,0,1)||!option("maxKph",maxKph,0,8)||
      !option("hardCutKph",hardCut,1,30)||hardCut<maxKph||!option("turnConfirm",turn,0,1)||!option("steeringFallback",steerFallback,0,1)||!option("parkSteeringFallback",parkSteer,0,1)||
      !option("cooldownMs",cooldown,0,10000)||
      !option("pDeg",pDeg,1,720)||!option("drDeg",drDeg,1,720)||!option("obstacleDeg",obstacleDeg,1,720)||
      !option("obstacleCm",obstacleCm,1,499)) {
    server.send(400,"application/json","{\"ok\":false,\"reason\":\"INVALID_DISPLAY_OPTIONS\"}");return;
  }
  gearhintv12live::Config config;
  config.requireSeatbelt=belt!=0;config.maxSpeedKph=static_cast<uint8_t>(maxKph);config.hardCutSpeedKph=static_cast<uint8_t>(hardCut);config.requireSteering=turn!=0;config.steeringFallback=steerFallback!=0;config.parkSteeringFallback=parkSteer!=0;
  config.cooldownMs=cooldown;config.pConfirmDeg=pDeg;config.drConfirmDeg=drDeg;config.obstacleDeg=obstacleDeg;
  config.obstacleThresholdCm=static_cast<uint16_t>(obstacleCm);
  const bool supported=activeCanAIsBody()&&activeCanBIsChassis();
  const bool available=supported&&!vehicleProfileSetupMode&&!vehicleProfileNvsError&&!canSubsystemBusy&&
      p4CanOnline[P4_CHASSIS]&&p4CanOnline[P4_BODY]&&p4CanOnline[aux]&&
      p4CanState[P4_CHASSIS]==TWAI_STATE_RUNNING&&p4CanState[P4_BODY]==TWAI_STATE_RUNNING&&
      p4CanState[aux]==TWAI_STATE_RUNNING;
  gearhintv12live::Inputs in;
  gearhintv12live::Result result;
  gearhintv12live::PrearmResult prearm;
  uint32_t now;
  portENTER_CRITICAL(&gearHintDisplayMux);
  portENTER_CRITICAL(&gearShiftMux);
  now=static_cast<uint32_t>(millis());
  in=gearHintV12LiveObserver.inputs(aux,now,false);
  if(!gearhintv12live::hardSpeedCut(in,now,config.hardCutSpeedKph)) in=gearHintV12LiveObserver.inputs(aux,now);
  result=gearHintV12LiveObserver.result(aux,now,available,config);
  prearm=gearShiftPrearmSnapshot(now);
  const auto &saved=gearShiftConfig;
  const bool prearmApplied=(saved.protocol!=gearshift::Protocol::V13V14)&&saved.speedBus==aux&&
      saved.requireSeatbelt==config.requireSeatbelt&&saved.requireSteering==config.requireSteering&&
      saved.steeringFallback==config.steeringFallback&&saved.parkSteeringFallback==config.parkSteeringFallback&&
      saved.maxSpeedKph==maxKph&&saved.hardCutSpeedKph==hardCut&&saved.cooldownMs==cooldown&&
      saved.parkConfirmDeg==pDeg&&saved.driveReverseConfirmDeg==drDeg&&
      saved.obstacleConfirmDeg==obstacleDeg&&saved.obstacleThresholdCm==obstacleCm;
  portEXIT_CRITICAL(&gearShiftMux);
  portEXIT_CRITICAL(&gearHintDisplayMux);
  String out;out.reserve(6200);JsonWriterArduino j(out);
  j.boolean("ok",true);j.string("schema","p4can.v12.distance-rules.v3");
  j.string("source","LIVE_CAN_DISTANCE_RULES");j.string("mode","SIMPLE_DISTANCE_EXPERIMENT");
  j.boolean("displayOnly",true);j.boolean("vehicleDecisionImplemented",true);
  j.boolean("protocolVerified",false);j.boolean("geometryVerified",false);
  j.boolean("sweptPathComputed",false);j.boolean("ttcComputed",false);
  j.boolean("radarHealthVerified",false);j.boolean("brakePressureVerified",false);
  j.boolean("controlPermission",false);j.boolean("profileSupported",supported);
  j.u32("auxBus",aux);j.u32("bodyStateBus",0);j.boolean("wheelConfirmation",config.requireSteering);
  j.boolean("steeringRequired",config.requireSteering);j.boolean("occupancyRequired",false);
  j.beginObject("config");j.boolean("requireSeatbelt",config.requireSeatbelt);j.u32("maxKph",maxKph);j.u32("hardCutKph",hardCut);
  j.boolean("turnConfirm",config.requireSteering);j.boolean("steeringFallback",config.steeringFallback);j.boolean("parkSteeringFallback",config.parkSteeringFallback);j.u32("cooldownMs",cooldown);
  j.u32("pDeg",pDeg);j.u32("drDeg",drDeg);j.u32("obstacleDeg",obstacleDeg);j.u32("obstacleCm",obstacleCm);j.endObject();
  j.u32("nowMs",now);j.u32("freshnessMs",gearhintv12live::kFreshMs);
  j.string("candidate",gearhintv12live::candidateName(result.candidate));
  j.string("reason",gearhintv12live::reasonName(result.reason));
  j.string("blockedInput",result.blockedSignal>=0?gearhintv12live::signalName(result.blockedSignal):"");
  j.string("rule",result.rule);j.u32("remainingMs",result.remainingMs);
  j.u32("zeroForMs",in.zeroForMs);j.u32("stableForMs",result.stableForMs);j.u32("pairs",result.pairs);
  j.beginObject("prearm");j.boolean("applied",prearmApplied);j.boolean("ready",prearm.ready);j.boolean("speedCut",prearm.speedCut);
  j.string("candidate",gearhintv12live::candidateName(prearm.candidate));
  j.string("source",gearhintv12live::sourceName(prearm.source));
  j.boolean("sameGearIgnored",prearm.sameGearIgnored);j.boolean("radarAvailable",prearm.radarAvailable);
  j.boolean("radarSteeringRequired",prearm.radarSteeringRequired);j.boolean("radarPriority",prearm.radarPriority);
  j.string("reason",gearhintv12live::reasonName(prearm.reason));j.string("rule",prearm.rule);
  j.u32("stableForMs",prearm.stableForMs);j.i32("baselineWheelDeciDeg",prearm.baselineWheelDeci);
  j.i32("wheelDeltaDeciDeg",prearm.wheelDeltaDeci);j.u32("brakeEpoch",prearm.brakeEpoch);j.u32("remainingMs",prearm.remainingMs);j.i32("wheelAngleDeciDeg",prearm.wheelAngleDeci);j.endObject();
  j.u32("cooldownRemainingMs",result.cooldownRemainingMs);
  j.i32("wheelDeltaDeciDeg",result.wheelDeltaDeci);j.i32("confirmationDeciDeg",result.confirmationDeci);
  j.beginObject("values");
  j.i32("gearRaw",in.gear);j.i32("driverBrakeRaw",in.driverBrake);j.i32("brakeApplyRaw",in.brakeApply);
  j.i32("acceleratorRaw",in.accelRaw);j.i32("speedCentiKph",in.speedCentiKph);
  j.i32("wheelAngleDeciDeg",in.wheelAngleDeci);j.i32("buckleRaw",in.buckle);j.i32("occupancyRaw",in.occupancy);
  j.i32("leftFrontLatch",in.leftFront);j.i32("leftRearLatch",in.leftRear);
  j.i32("rightFrontLatch",in.rightFront);j.i32("rightRearLatch",in.rightRear);j.i32("trunkLatch",in.trunk);
  j.endObject();
  j.beginObject("scans");
  for(uint8_t side=0;side<2;++side) {
    const auto &s=side?in.rear:in.front;
    j.beginObject(side?"rear":"front");j.boolean("valid",s.valid);
    j.i32("minMeasuredCm",s.minCm);j.u32("noObjectCount",s.noObjectCount);
    j.u32("measuredCount",s.measuredCount);j.boolean("nearDetection",s.near);
    String values="[";for(uint8_t i=0;i<6;++i){if(i)values+=',';values+=s.raw[i];}values+=']';
    j.raw("raw",values);j.endObject();
  }
  j.endObject();j.beginObject("inputs");
  for(uint8_t s=0;s<gearhintv12live::SignalCount;++s) {
    const auto &t=in.tracks[s];j.beginObject(gearhintv12live::signalName(s));
    j.boolean("required",gearhintv12live::required(s,in.gear,config));
    j.u32("id",gearhintv12live::kSignalIds[s]);
    j.u32("bus",s<=gearhintv12live::Rear?1:(s==gearhintv12live::Speed||s==gearhintv12live::Steering?aux:0));
    j.boolean("seen",t.seen);j.boolean("valid",t.valid);
    j.boolean("fresh",t.seen&&now-t.frame.observedMs<gearhintv12live::kFreshMs);
    if(t.seen) j.u32("ageMs",now-t.frame.observedMs);else j.raw("ageMs","null");
    j.boolean("checksumChecked",gearhintv12live::checksumPosition(s)>=0);
    j.u32("generation",t.generation);
    j.string("invalidReason",t.valid?"":gearhintv12live::reasonName(t.invalidReason));
    j.string("data",t.seen?canFrameHexArduino(t.frame.data,t.frame.dlc):String(""));
    j.endObject();
  }
  j.endObject();j.finish();server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",out);
}
