# P4CAN Auto Shift — V12 / V13-V14 Extract

**中文** | [English](#english)

## 这是什么

这是从 P4CAN Universal 固件中提取的 **V12 与 V13/V14 自动换挡判断与执行源码包**，包含候选判断、刹车授权、方向盘相对角度确认、请求模板编码、安全门禁、状态接口参考和主机回归测试。

本包分成两层：

- `src/`：便携核心，只依赖 C++ 标准库，可在普通电脑上编译测试；
- `firmware-reference/`：原 P4CAN 固件接入参考，依赖主工程里的 Arduino/ESP-IDF、WebServer、NVS、三路 CAN 状态和车型档案符号。

本包不是完整整车固件，也不包含 P4CAN 的其他车控功能。

## 原作者来源

本项目提取和修改自 06066060606060 的 P4CAN Universal Unlock 固件：

- 原作者仓库：<https://github.com/06066060606060/P4can-universal-unlock>

## 状态与测试声明

- 当前代码主要由 AI 生成，尚未完成独立第三方安全审查或车辆安全认证。
- 目前仅在 **2021 款带雷达 Model Y** 上做过实车测试，主要覆盖 V12 前后雷达路径。
- **焕新 Model Y（Juniper）尚未适配完成**：部分刹车、配对和方向盘逻辑只是实验性实现，不应视为已完成车型适配。
- **Model 3 尚未适配完成**，也没有完成对应的实车验证。
- 未列入实测范围的车型一律视为未验证；不要在这些车型上开启实际换挡执行。

## 安全边界

这份代码来自实验性协议观察和逆向资料，不是安全认证软件。自动换挡可能导致车辆意外移动、财产损失、人身伤亡或违反当地法规。发布或实车使用前，必须自行完成法规检查、风险评估、失效模式分析、封闭场地测试和安全退避方案。本项目按 MIT 许可“原样”提供，不提供任何担保。

## 目录结构

```text
src/
  gear_shift_pure.h              执行状态机、安全门禁、配置序列化、0x229/0x678 编码
  gear_hint_v12_raw_pure.h       V12 原始帧与信号定义
  gear_hint_v12_live_pure.h      V12 距离规则、刹车后方向盘状态机、D/R 雷达确认
  gear_hint_display_pure.h       V13/V14 0x255 + 0x12B 配对观察器

firmware-reference/
  gear_hint_display_runtime.h    CAN 接收旁路与 observer 接入
  gear_shift_runtime.h           10ms 执行服务、候选优先级、NVS 配置、CAN 发送
  gear_shift_web.h               /api/gear-hint/shift/status 与 /config API
  gear_hint_display_web.h        V12/V13-V14 状态与调试 API
  text_format_arduino.h          字符串辅助依赖
  json_writer_arduino.h          JSON 输出依赖
  fixed_point_pure.h             数值/文本依赖
  text_format_pure.h             字符串依赖

tests/
  test_gear_shift_control.cpp    编码、刹车互锁、一次刹车一次请求测试
  test_v12_live_decision.cpp     V12 距离、雷达/手动方向、相对角度测试
  test_gear_hint_display.cpp     V13/V14 配对、过期、刹车和计数器测试

tools/run-host-tests.sh          用本机 C++17 编译器运行测试
docs/                            原中文设计说明
LICENSE                          MIT 许可证
```

## 当前逻辑摘要

### 候选优先级

1. V13/V14 原车配对候选有效时优先，踩刹车边沿立即执行。
2. D/R 雷达候选有效时：
   - D/R 雷达方向确认关闭，踩刹车立即执行；
   - 开启时，踩刹车记录方向盘基准角，任一方向相对变化达到 `obstacleDeg` 后立即执行。
3. 无原车配对和雷达候选时，进入手动 D/R 方向盘逻辑：踩刹车记录基准角，任一方向相对变化达到 `drDeg` 后切相反挡。
4. P 档手动方向盘需要车型左右符号标定。当前 2021 Model Y 按负相对角=左打处理；Model Y Juniper 尚未标定，P 档手动执行拒绝。

### V12 雷达规则

- P 档：前方更近 → R；后方更近 → D；两端空旷 → 默认 D。
- D 档：前方阈值内有障碍 → R，否则保持 D。
- R 档：后方阈值内有障碍 → D，否则保持 R。
- 默认障碍阈值 60cm。

### V13/V14 配对规则

- `0x255` 建议码 `1` + `0x12B` 方向码 `3` → D。
- `0x255` 建议码 `2` + `0x12B` 方向码 `2` → R。
- 其他组合不执行。
- Model Y Juniper 的配对字段约 1Hz，执行预备层保留 1500ms，避免正常报文间隔造成候选闪烁。

### 刹车与一次动作限制

- 普通候选必须先预备；踩下刹车只做最终授权。
- 手动方向盘与 D/R 雷达方向确认例外：踩刹车边沿记录基准角，刹车保持期间达到相对角度立即执行。
- 每次物理“松开→踩下”最多发送一次请求。
- 刹车保持期间新出现的普通候选不执行。
- Model Y Juniper 使用 `driverBrakeRaw=2`，apply 位仅诊断；其他档案仍要求 `driverBrake=2 && apply=1`。

### 最终安全门禁

实际发送前还必须满足：执行开关开启、三路 CAN RUNNING、挡位为 P/D/R、目标挡不同、车速低于允许值、未超过彻底切断值、加速踏板为 0、车门和尾门关闭、安全带可选门禁通过、请求模板新鲜且样本足够、冷却完成。发送后 1500ms 内等待实际挡位确认，同脚刹车不重试。

## 运行主机测试

macOS / Linux：

```sh
cd P4CAN-AutoShift-V12-V13V14
./tools/run-host-tests.sh
```

预期输出包含：

```text
PASS: ... gear-shift encoder/template/interlock/one-shot checks
PASS: ... simplified live-rule checks
PASS: ... brake-interlock checks plus V13/V14 ...
```

Windows 可使用 MSYS2、WSL，或用任意 C++17 编译器编译 `tests/*.cpp`，并把 `src/` 加入 include 路径。

## 移植要点

1. 在 CAN 接收路径调用 `gearHintDisplayObserveRaw()`，送入 BODY/CHASSIS/PARTY 原始帧。
2. 每 10ms 调用一次 `gearShiftServiceTick()`。
3. 提供 `p4CanSendFrame()` 与三路 CAN 状态。
4. 将 `gearShiftConfigLoad()`、`gearShiftConfigSave()` 接到 NVS 或 Flash。
5. 注册 `GET /api/gear-hint/shift/status` 和 `POST /api/gear-hint/shift/config`。
6. 接入车型档案，决定刹车 apply 规则和 P 档方向盘符号。
7. 未完成失败安全测试前，不要接入实车。

## 许可证

MIT License，见 [LICENSE](LICENSE)。发布前请确认你拥有所有拟发布代码的权利；如需其他许可证，请先替换本文件。

## 支持作者

欢迎请作者喝杯咖啡，助力后续更新～

<img src="docs/alipay-qr.jpg" alt="支付宝收款码" width="360">

---

# English

## What this is

This package extracts the **V12 and V13/V14 automatic gear-shift decision and execution source** from the P4CAN Universal firmware. It includes candidate selection, brake authorization, relative steering confirmation, request-template encoding, final safety interlocks, reference status APIs, and host regression tests.

The package has two layers:

- `src/`: a portable core that depends only on the C++ standard library and can be built and tested on a computer;
- `firmware-reference/`: integration excerpts from the original P4CAN firmware, which expect parent-project Arduino/ESP-IDF, WebServer, NVS, three-bus CAN state, and vehicle-profile symbols.

This is not a complete vehicle firmware and does not include unrelated P4CAN features.

## Upstream author and source

This package was extracted and modified from 06066060606060's P4CAN Universal Unlock firmware:

- Upstream repository: <https://github.com/06066060606060/P4can-universal-unlock>

## Status and testing disclosure

- The current code is primarily AI-generated and has not completed independent third-party safety review or vehicle safety certification.
- Real-vehicle testing has only been performed on a **2021 radar-equipped Model Y**, mainly covering the V12 front/rear parking-sensor path.
- **Model Y Juniper is not fully adapted yet**: some brake, pairing, and steering logic is experimental and must not be treated as a completed vehicle adaptation.
- **Model 3 is not fully adapted yet** and has not completed corresponding real-vehicle validation.
- Vehicles not listed as tested are unverified. Do not enable actual gear-shift execution on unverified vehicles.

## Safety boundary

This code comes from experimental protocol observation and reverse-engineering. It is not safety-certified. Automatic gear shifting can cause unintended vehicle motion, property damage, injury, or legal violations. Before publishing or using it on a vehicle, perform regulatory review, risk assessment, failure-mode analysis, closed-course testing, and fail-safe planning. The project is provided as-is under the MIT License, without warranty.

## Repository layout

```text
src/
  gear_shift_pure.h              Execution state machine, interlocks, config codec, 0x229/0x678 encoder
  gear_hint_v12_raw_pure.h       V12 raw frame and signal definitions
  gear_hint_v12_live_pure.h      V12 distance rules, brake-first steering state machines, D/R radar confirmation
  gear_hint_display_pure.h       V13/V14 0x255 + 0x12B pairing observer

firmware-reference/
  gear_hint_display_runtime.h    CAN bypass observer integration
  gear_shift_runtime.h           10ms service, candidate priority, NVS config, CAN submission
  gear_shift_web.h               /api/gear-hint/shift/status and /config API
  gear_hint_display_web.h        V12/V13-V14 status and debug API
  text_format_arduino.h          String helper dependency
  json_writer_arduino.h          JSON writer dependency
  fixed_point_pure.h             Numeric/text dependency
  text_format_pure.h             String dependency

tests/
  test_gear_shift_control.cpp    Encoding, brake interlock, one-shot checks
  test_v12_live_decision.cpp     V12 distance, radar/manual steering, relative-angle checks
  test_gear_hint_display.cpp     V13/V14 pairing, expiry, brake, counter checks

tools/run-host-tests.sh          Run tests with a local C++17 compiler
docs/                            Original Chinese design notes
LICENSE                          MIT license
```

## Behavior summary

### Candidate priority

1. A valid V13/V14 vehicle-pair candidate has priority and executes on the brake press edge.
2. For a valid D/R radar candidate:
   - D/R radar steering confirmation off: execute immediately on the brake edge;
   - on: capture a steering baseline on the edge and execute after relative motion reaches `obstacleDeg`.
3. With no vehicle-pair or radar candidate, manual D/R steering captures a baseline on the brake edge and switches to the opposite gear after relative motion reaches `drDeg`.
4. P manual steering requires model-specific left/right sign calibration. The 2021 Model Y maps negative relative motion to left; Model Y Juniper is not calibrated, so P manual execution fails closed.

### V12 radar rules

- P: front closer → R; rear closer → D; both clear → D by default.
- D: a front obstacle within threshold → R; otherwise stay in D.
- R: a rear obstacle within threshold → D; otherwise stay in R.
- Default obstacle threshold: 60cm.

### V13/V14 pairing rules

- `0x255` request `1` plus `0x12B` direction `3` → D.
- `0x255` request `2` plus `0x12B` direction `2` → R.
- Other combinations are rejected.
- Juniper pair fields arrive at about 1Hz, so the execution pre-arm retains them for 1500ms to avoid flicker during normal frame gaps.

### Brake and one-action rule

- Normal candidates must be pre-armed; the brake edge is only final authorization.
- Manual steering and D/R radar confirmation are exceptions: the edge captures a steering baseline, and relative motion during the same held pedal triggers immediately.
- Each physical release-to-press transition may send at most one request.
- A normal candidate first appearing while the brake is already held is not executed.
- Model Y Juniper accepts `driverBrakeRaw=2` and treats apply as diagnostic; other profiles require `driverBrake=2 && apply=1`.

### Final interlocks

Before transmission, all of the following must pass: execution enabled, all three CAN buses running, current gear P/D/R, target differs from current gear, speed below the allowed limit, speed below the hard cutoff, accelerator at zero, doors and trunk closed, optional seatbelt gate passed, fresh request template with enough samples, and cooldown elapsed. After sending, the code waits up to 1500ms for actual-gear confirmation and never retries during the same brake press.

## Run host tests

macOS / Linux:

```sh
cd P4CAN-AutoShift-V12-V13V14
./tools/run-host-tests.sh
```

Expected output includes:

```text
PASS: ... gear-shift encoder/template/interlock/one-shot checks
PASS: ... simplified live-rule checks
PASS: ... brake-interlock checks plus V13/V14 ...
```

On Windows, use MSYS2, WSL, or any C++17 compiler to build `tests/*.cpp` with `src/` on the include path.

## Porting notes

1. Call `gearHintDisplayObserveRaw()` from the CAN receive path for BODY/CHASSIS/PARTY frames.
2. Call `gearShiftServiceTick()` every 10ms.
3. Provide `p4CanSendFrame()` and three-bus CAN state.
4. Connect `gearShiftConfigLoad()` and `gearShiftConfigSave()` to NVS or flash.
5. Register `GET /api/gear-hint/shift/status` and `POST /api/gear-hint/shift/config`.
6. Integrate the vehicle-profile layer to select brake-apply behavior and P steering sign calibration.
7. Do not connect to a real vehicle before fail-safe testing.

## License

MIT License; see [LICENSE](LICENSE). Confirm that you own or may publish all included code. Replace the license first if a different policy applies.
