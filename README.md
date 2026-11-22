# Interactive AMR Cockpit Simulator

[![Node.js](https://img.shields.io/badge/Node.js-%3E%3D24.0.0-339933.svg)](https://nodejs.org/)
[![Pure ESM](https://img.shields.io/badge/JavaScript-Zero--Dependency%20ESM-F7DF1E.svg)](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Guide/Modules)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)

> Bilingual portfolio repository (**English** & **Tiếng Việt**) — Zero-dependency HTML5 / CSS3 / ESM JS cockpit & 2D LiDAR/SLAM simulator for a Differential-Drive 2-Wheel Autonomous Mobile Robot (AMR), calibrated with experimental hardware, latency, motor-tracking, and SLAM datasets.

---

## English Documentation

### Overview

`amr-cockpit-simulator` is a zero-dependency, offline-capable HTML5 / CSS3 / native ESM simulator for a 2-wheel differential-drive AMR cockpit. It models:

1. **Connection & Launch**: Configurable host/port (`127.0.0.1:8080/8081`), `[1, 65535]` validation, 1-click **Enter Cockpit**, **Launch Offline Simulator**, and **Connect (Live Probe)**.
2. **3-Step Preflight**: `API Control`, `LiDAR & Map`, `Camera Stream` (`0/3 -> 3/3`).
3. **3-Column Landscape Cockpit**:
   - **Left**: 2D LiDAR/SLAM radar canvas (`0.5/1.0/1.5/2.0 m` rings, dashed `0.40 m` safety boundary, click-to-place/remove obstacles) + 8-direction hold-to-drive D-pad (`pointerdown`/`pointerup` + `WASD`/Arrows/`QEZC`/`Space`).
   - **Center**: `640×480` simulated MJPEG tactical HUD canvas + `2×2` telemetry readout + 8-zone cm strip + live REST event log.
   - **Right**: `ARM`/`DISARM`/`CLEAR_FAULT` FSM card, `40–135 RPM` (`±10`) speed stepper, `EMERGENCY STOP`, 4 arena presets (`Clear`, `Front Obstacle`, `Corridor`, `Dynamic Moving`), and interactive obstacle sliders.
   - **Drawers**: **Settings, PID & Kinematics Reference** (armed-guarded) and **Mission & Telemetry Inspector**.

### Architecture, Kinematics & Hardware Reference

- **Pure ESM Core (`src/core/`)**: Zero DOM/browser globals (`kinematics.js`, `lidar.js`, `safety.js`, `scenarios.js`, `simulator.js`), fully testable under Node's built-in test runner.
- **Kinematics & Safety Contract**:
  - Wheel radius $R = 0.0366\text{ m}$, conversion $K = \frac{2\pi R}{60} \approx 0.003833\text{ m/s/RPM}$ ($40\text{ RPM} \approx 0.153\text{ m/s}$), speed range `40–135 RPM` (`±10` steps).
  - Physical bench chassis track width $L = 0.1875\text{ m}$ ($B = 0.09375\text{ m}$); simulator engine uses `HALF_WHEELBASE_M = 0.096 m` and fixed `120×120` grid (`0.05 m/cell`) to preserve existing unit & E2E parity contracts.
  - Angular rate $\omega = \min(1.80, 0.60 \cdot |v| / B)$, diagonal factor $1/\sqrt{2} \approx 0.707$, active 8-zone `< 0.40 m` (`< 40.0 cm`) safety interlock, `150 ms` hold repeat, and `1500 ms` telemetry watchdog.
- **Onboard Hardware Stack Reference (Sanitized)**:
  - **Raspberry Pi 4 (`ROS 2 Jazzy`)**: `rplidar_ros2`, `slam_toolbox`, `tf2 static_transform_publisher`; REST `:8080`, MJPEG `:8081` (`640×480`), Auth Proxy `:8090`.
  - **ESP32 (`CH340`, `115200` baud)**: `MKE-M18` I2C motor driver (`GPIO21` SDA, `GPIO22` SCL, `0x42`), Left Encoder (`GPIO18/19`, `937` PPR), Right Encoder (`GPIO25/26`, `937` PPR).
  - **RPLiDAR C1**: `10.0 Hz`, `666–682` pts/rev, polling timers `150 / 250 / 400 / 500 / 1000 ms`.

### Calibrated Sample Datasets (`data/`)

| Fixture | Contents | Calibrated Benchmarks |
|---|---|---|
| `data/hardware_stack_reference.json` | Pi 4 / ROS 2 / ESP32 / timer / FSM spec | `R=0.0366 m`, `L=0.1875 m` (`B_sim=0.096 m`), `150/250/400/500/1000 ms` |
| `data/telemetry_latency_n40.json` | `n = 40` proxy telemetry latency array | Min `26.4`, Median `36.0`, Mean `40.9`, **p95 `60.3 ms` (`< 150 ms`)**, Max `163.9 ms` |
| `data/motor_tracking_benchmarks.json` | 6 motor tracking & interlock scenarios | Unarmed `0/0`; `40` Fwd `46.09/47.38`; `80` Turn `74.26/43.53`; `40` Rev `-38.41/-38.41`; `135` Max `133.16/138.28` (`< 3%`); Lock `0.17 m -> 0/0` |
| `data/slam_grid_snapshot.json` | `101×99` SLAM grid + `674`-pt LiDAR scan | `0.05 m/cell` (`9999` cells), `age_s=0.018`, nearest `0.62 m @ 268°`, pose `(0.114, -2.120, 124.6°)`, `dist=7.88 m`, `cov=3.7%` |

### Installation, Usage & Automated Tests

Requires **Node.js `>= 24.0.0`** (zero npm external dependencies):

```bash
# Serve locally (ESM requires HTTP, not file://)
npm run serve

# Run full unit, calibration dataset, privacy, and headless Edge/Chrome CDP E2E tests
npm test
```

---

## Tài liệu Tiếng Việt

### Tổng quan

`amr-cockpit-simulator` là trình mô phỏng buồng lái và vật lý thời gian thực hoàn toàn offline (HTML5 / CSS3 / ESM JavaScript thuần, không phụ thuộc thư viện ngoài) cho Robot tự hành vi sai 2 bánh (AMR), bao gồm mô phỏng động học, quét tia LiDAR 360°, bản đồ chiếm dụng SLAM `0.05 m/ô`, màn hình HUD `640×480` và khóa an toàn chủ động 8 vùng `< 40 cm`.

### Kiến trúc, Động học & Thông số phần cứng

- **Lõi ESM thuần (`src/core/`)**: Tách biệt hoàn toàn khỏi DOM (`kinematics.js`, `lidar.js`, `safety.js`, `scenarios.js`, `simulator.js`), kiểm thử trực tiếp trên Node.js.
- **Thông số phần cứng & ROS 2 Jazzy tham chiếu**:
  - **Raspberry Pi 4 (`ROS 2 Jazzy`)**: `rplidar_ros2`, `slam_toolbox`, `tf2 static_transform_publisher`; REST `:8080`, MJPEG `:8081` (`640×480`), Auth Proxy `:8090`.
  - **ESP32 (`CH340`, `115200` baud)**: Mạch công suất `MKE-M18` I2C (`GPIO21` SDA, `GPIO22` SCL, `0x42`), Encoder trái (`GPIO18/19`, `937` PPR), Encoder phải (`GPIO25/26`, `937` PPR), **RPLiDAR C1** (`10.0 Hz`, `666–682` điểm/vòng).
  - **Động học & An toàn**: $R = 0.0366\text{ m}$, bề rộng cơ sở vật lý $L = 0.1875\text{ m}$ ($B = 0.09375\text{ m}$; trình mô phỏng giữ `HALF_WHEELBASE_M = 0.096 m` và lưới `120×120` để bảo toàn hợp đồng kiểm thử), dải `40–135 RPM`, khóa an toàn `< 0.40 m`, watchdog `1500 ms` và chu kỳ `150/250/400/500/1000 ms`.

### Bộ dữ liệu mẫu & Chuẩn đối chiếu (`data/`)

- `data/hardware_stack_reference.json`: Tham chiếu phần cứng Pi 4, ESP32, cổng dịch vụ và máy trạng thái `ARM`/`STOP`.
- `data/telemetry_latency_n40.json`: `40` mẫu độ trễ `/api/telemetry` qua proxy (`Min 26.4`, `Median 36.0`, `Mean 40.9`, `p95 60.3 ms < 150 ms`, `Max 163.9 ms`).
- `data/motor_tracking_benchmarks.json`: `6` kịch bản bám tốc độ động cơ (`40–135 RPM`, sai số ở `135 RPM < 3%`, khóa vật cản `0.17 m -> 0/0 RPM`).
- `data/slam_grid_snapshot.json`: Bản đồ SLAM `101×99` (`0.05 m/ô`, `9999` ô, `age_s = 0.018 s`, `674` tia, vật cản gần nhất `0.62 m @ 268°`, vị trí `X=0.114 m, Y=-2.120 m, Heading=124.6°, Distance=7.88 m, coverage=3.7%`).

### Cài đặt & Kiểm thử tự động

Yêu cầu **Node.js `>= 24.0.0`**:

```bash
# Chạy máy chủ HTTP nội bộ để mở giao diện trình duyệt
npm run serve

# Chạy toàn bộ kiểm thử đơn vị, bộ dữ liệu hiệu chuẩn, bảo mật và E2E trình duyệt CDP
npm test
```
