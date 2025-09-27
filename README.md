# spi_bridge (ROS 2 Jazzy)

Minimal ROS 2 package that bridges a Raspberry Pi 5 SPI master to a Nucleo F446RE SPI slave with a double-transfer protocol.

It keeps your original Python SPI loop structure (headers, CRC table, verify function, double-transfer timing) and wraps it in a ROS 2 node:
- Subscribes to `/cmd_vel` and sends the first two floats (forward, turn) to the Nucleo.
- Publishes feedback (Nucleo’s measured forward/turn), IMU raw data, magnetometer, and a diagnostics array.

## Wiring (Pi J8 → Nucleo-F446RE)

| Raspberry Pi 5 Pin | Function              | Nucleo F446RE Pin |
| ------------------ | --------------------- | ----------------- |
| 5V (PIN 2)         | Optional Power Supply | E5V               |
| GND (PIN 6)        | First GND             | GND below E5V     |
|                    |                       |                   |
| GPIO10 (Pin 19)    | MOSI                  | PC_3              |
| GND    (Pin 20)    | Second GND            | GND below AVDD    |
| GPIO9  (Pin 21)    | MISO                  | PC_2              |
| GPIO11 (Pin 23)    | SCLK                  | PB_10             |
| GPIO8  (Pin 24)    | CS (CE0)              | PB_12             |

It is important to connect two GNDs (pins 6 and 20) to ensure a stable reference.

## Topics

- **Subscribes:**
  - `/cmd_vel` (`geometry_msgs/Twist`)
- **Publishes:**
  - `/robot/feedback` (`geometry_msgs/TwistStamped`) — forward & turn measured by the Nucleo
  - `/imu/data_raw` (`sensor_msgs/Imu`) — gyro (rad/s) & accel (m/s²)
  - `/imu/mag` (`sensor_msgs/MagneticField`) — magnetic field (Tesla)
  - `/spi/diag` (`diagnostic_msgs/DiagnosticArray`) — timing & counters

## Install dependencies

### Ubuntu on Raspberry Pi (Debian/Ubuntu-based, apt packages available)

```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions ros-jazzy-teleop-twist-keyboard python3-spidev
```

### Raspberry Pi OS (Bookworm) — build `teleop_twist_keyboard` from source

The `ros-jazzy-teleop-twist-keyboard` apt package is usually **not available** on Raspberry Pi OS.
Build it from source alongside this package:

```bash
# Create or reuse a workspace for Pi OS
mkdir -p ~/spi_ws/src
cd ~/spi_ws/src

# Clone teleop_twist_keyboard (ROS 2 version)
git clone https://github.com/ros2/teleop_twist_keyboard.git

# Also clone this spi_bridge repo here if not already
# git clone https://github.com/pichim/spi_bridge.git

# Ensure spidev is installed
sudo apt update && sudo apt install -y python3-spidev

# Build the workspace
cd ~/spi_ws
colcon build --symlink-install
source install/setup.bash
```

## Enable SPI (if not already)

Edit `/boot/firmware/config.txt` and ensure:

```ini
dtparam=spi=on
```

Reboot if you changed it:

```bash
sudo reboot
```

## Permissions

### Ubuntu

Make sure your user can open `/dev/spidev0.0` without sudo:

```bash
ls -l /dev/spidev*
sudo usermod -aG dialout $USER
newgrp dialout
```

### Raspberry Pi OS

```bash
ls -l /dev/spidev*
sudo usermod -aG spi $USER
newgrp spi
```

## Build

### Ubuntu (workspace: `~/ros2_ws`)

```bash
source /opt/ros/jazzy/setup.bash

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Raspberry Pi OS (workspace: `~/spi_ws`)

If you built ROS 2 from source, remember to source your base install **before** building/running this workspace.

```bash
source ~/ros2_jazzy/install/local_setup.bash

cd ~/spi_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

### Ubuntu on Raspberry Pi (workspace: `~/ros2_ws`)

```bash
# Terminal 1: SPI bridge
source ~/ros2_ws/install/setup.bash
ros2 run spi_bridge spi_bridge_node

# Terminal 2: keyboard teleop (keep this terminal focused while driving)
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Raspberry Pi OS (workspace: `~/spi_ws`)

```bash
# Terminal 1: SPI bridge
source ~/spi_ws/install/setup.bash
ros2 run spi_bridge spi_bridge_node

# Terminal 2: keyboard teleop (keep this terminal focused while driving)
source ~/spi_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Monitor data

```bash
ros2 topic echo /robot/feedback
ros2 topic echo /imu/data_raw
ros2 topic echo /imu/mag
ros2 topic echo /spi/diag
```

## Parameters (see config/params.yaml)

- `spi_bus`, `spi_dev`, `spi_max_speed_hz` — selects `/dev/spidev<bus>.<dev>` and SPI clock.
- `main_task_period_us` — loop period in microseconds (default 10000 ⇒ 100 Hz).
- `arm_gap_us` — busy-wait gap between ARM and PUBLISH transfers (default 100 µs).
- `cmd_timeout_ms` — deadman timeout; if `/cmd_vel` is silent this long, send zeros.
- `max_forward`, `max_turn` — safety clamps for incoming `/cmd_vel`.
- `imu_frame_id`, `base_frame_id` — frame names for published messages.
- `gyro_scale`, `accel_scale`, `mag_scale` — optional scaling to convert Nucleo units to ROS units.

Example: if Nucleo sends degrees/s, set
```yaml
gyro_scale: 0.01745329252  # to publish rad/s
```

## Additional commands

### Change `spi_max_speed_hz` at runtime and send a test command

```bash
# Send a simple forward command at 50 Hz
ros2 topic pub -r 50 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Then, in another terminal, attempt to set the SPI speed parameter:

```bash
ros2 param set /spi_bridge spi_max_speed_hz 12000000
```

> **Note:** The node currently does **not** re-apply changed parameters at runtime (no parameter callback).
> The value will update on the parameter server, but the SPI device speed will only change after you **restart**
> the node with a new YAML/launch configuration.

## Notes

- Protocol matches the original script:
  - ARM-ONLY transfer (`0x56` + zeros) so the Nucleo re-arms its TX.
  - PUBLISH transfer (`0x55` + payload) sends commands and reads fresh data.
- On CRC/header mismatch, the node increments `fail_count` (see `/spi/diag`) and keeps timing.
- A simple deadman zeros commands if `/cmd_vel` is silent for `cmd_timeout_ms`.
- You can tweak period/gap/limits in `config/params.yaml` without touching code.

## TODO

- [ ] Add a runtime parameter to control per-cycle log verbosity (currently gated by `VERBOSE_CYCLE_LOG` in code).
- [ ] Verify and document sensor units (gyro = rad/s, accel = m/s², mag = Tesla) and set `gyro_scale`, `accel_scale`, `mag_scale` accordingly.
- [ ] Optional: publish a fused orientation on `/imu/data` with covariances per REP-145.
- [ ] Optional: add joystick teleop and a separate launch.
- [ ] Optional: diagnostics improvements (e.g., reasons for CRC failures, SPI timing histograms).
