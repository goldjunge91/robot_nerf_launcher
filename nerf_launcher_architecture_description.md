# 🎯 Nerf Launcher System - Implementierungs-Anleitung

> **Hardware:** Arduino Pro Micro (ATmega32U4)  
> **Software:** ROS2 Humble, Custom Serial Protocol, C++ Bridge Node  
> **Location:** `/src/robot_nerf_launcher`

## � Hardware-Setup

### Arduino Pro Micro
- **MCU:** ATmega32U4, 5V/16MHz
- **RAM:** 2.5KB | **Flash:** 32KB
- **I/O:** 18 Digital, 5 PWM
- **USB:** Native CDC (kein FTDI-Chip)

### Pin-Belegung
```
Pin 9  (PWM)    → Pitch Servo Signal
Pin 10 (PWM)    → Motor Driver Enable/PWM
Pin 16 (Digital) → Motor Driver Direction
Pin 14 (Digital) → Trigger Servo/Solenoid
A0 (Analog)     → Current Sense (optional)
```

### Externe Hardware
- **Servo:** Standard 5V Servo (SG90 oder ähnlich)
- **Motor Driver:** TB6612 oder L298N
- **DC Motors:** 2x für Dart-Beschleunigung
- **Power:** 5V/2A für Arduino + Servos, separate 7-12V für Motoren

## 🎮 Xbox Controller Mappings

**Bestehende Fahrzeug-Steuerung (bleibt unverändert):**
- Axis 0 (Left X) → Strafe links/rechts
- Axis 1 (Left Y) → Vorwärts/Rückwärts  
- Axis 2 (Right X) → Rotation
- Button 5 (RB) → Turbo Mode

**Nerf Launcher Steuerung:**
- Axis 4 (Right Y) → Pitch Control (-30° bis +45°)
- Button 0 (A) → Arm/Disarm Toggle
- Axis 5 (RT) → Fire (bei >0.9 + armed)
- Button 1 (B) → Emergency Disarm
- Button 3 (Y) → Motor Start/Stop

## 🏗️ System-Architektur

```
Xbox Controller (joy_node)
         │
         ├─→ teleop_twist_joy → /cmd_vel → Mecanum Drive
         │
         └─→ nerf_launcher_joy_node → /launcher/cmd_* 
                                            │
                                            ▼
                                  nerf_launcher_bridge_node
                                            │
                                   Serial /dev/ttyACM0
                                            │
                                    Arduino Pro Micro
                                      ├─→ Pitch Servo
                                      └─→ Motor Driver
```

## 📡 Serial Protocol (115200 Baud, ASCII, `\n` terminated)

### ROS2 → Arduino Commands
```
ARM\n                    - Arme Launcher
DISARM\n                 - Disarme Launcher
PITCH:<angle>\n          - Setze Pitch (-30 bis +45 Grad)
FIRE\n                   - Feuere Dart
MOTOR:<speed>\n          - Motor Speed (0-255)
ESTOP\n                  - Emergency Stop
HEARTBEAT\n              - Watchdog Reset
```

### Arduino → ROS2 Feedback
```
STATUS:<DISARMED|ARMED|FIRING|ERROR>\n
PITCH:<angle>\n          - Aktueller Winkel
READY:<0|1>\n            - Bereit zum Feuern
ERROR:<code>:<msg>\n     - Fehler
ACK:<command>\n          - Command bestätigt
HEARTBEAT\n              - Alive Signal (alle 200ms)
INIT\n                   - Boot complete
```

## 🔌 ROS2 Interface

### Topics
```yaml
# Published by Bridge Node
/launcher/state          (std_msgs/msg/String)    # DISARMED|ARMED|FIRING|ERROR
/launcher/pitch          (std_msgs/msg/Float32)   # Current angle in degrees
/launcher/ready          (std_msgs/msg/Bool)      # Ready to fire

# Subscribed by Bridge Node
/launcher/cmd_pitch      (std_msgs/msg/Float32)   # Target pitch (-30 to +45)
/launcher/cmd_motor      (std_msgs/msg/Float32)   # Motor speed (0.0 to 1.0)
```

### Services
```yaml
/launcher/arm            (std_srvs/srv/Trigger)   # Arm launcher
/launcher/disarm         (std_srvs/srv/Trigger)   # Disarm launcher
/launcher/fire           (std_srvs/srv/Trigger)   # Fire dart
/launcher/emergency_stop (std_srvs/srv/Trigger)   # Emergency stop
```

### Bridge Node Parameters
```yaml
serial_port: "/dev/ttyACM0"
baudrate: 115200
timeout_ms: 1000
watchdog_timeout_ms: 500
pitch_min: -30.0
pitch_max: 45.0
```

## 🛡️ State Machine & Safety

### States
```
DISARMED (default) → ARM → ARMED → FIRE → FIRING → DISARMED
                      ↑                              │
                      └──────────────────────────────┘
                      Auto-Disarm: 30s timeout, watchdog fail, emergency stop
```

### Safety Rules
1. **Watchdog:** Arduino erwarten Heartbeat alle 500ms → sonst Emergency Disarm
2. **Arming:** Default DISARMED, expliziter ARM command nötig
3. **Firing:** Nur wenn `ARMED && motor_speed > threshold && !cooldown_active`
4. **Cooldown:** 500ms zwischen Schüssen
5. **Limits:** Pitch -30° bis +45° (clamped in software)

## 📦 Package-Struktur

```
robot_nerf_launcher/
├── CMakeLists.txt
├── package.xml
├── firmware/nerf_launcher/
│   ├── nerf_launcher.ino
│   ├── StateMachine.cpp/h
│   ├── SerialProtocol.cpp/h
│   └── README.md
├── config/
│   ├── launcher_params.yaml
│   └── joy_launcher.yaml
├── launch/
│   ├── launcher_bridge.launch.py
│   └── launcher_full.launch.py
├── include/robot_nerf_launcher/
│   ├── launcher_bridge_node.hpp
│   └── launcher_joy_node.hpp
└── src/
    ├── launcher_bridge_node.cpp
    └── launcher_joy_node.cpp
```

### package.xml Dependencies
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>std_srvs</depend>
<depend>sensor_msgs</depend>
<depend>serial</depend>
```

## 🚀 Implementation Steps

### 1. Package Setup
```bash
cd /home/pi/workspace/ros2_dev_ws/my_steel-robot_ws/src/robot_nerf_launcher
# Erstelle package.xml und CMakeLists.txt
# Erstelle Verzeichnisse: firmware/nerf_launcher, config, launch, include, src
```

### 2. Arduino Firmware
```bash
cd firmware/nerf_launcher
# Schreibe nerf_launcher.ino
# Implementiere Serial Protocol Parser
# Implementiere State Machine
# Servo Control (Pitch)
# Motor Control (PWM + Direction)
# Flash auf Arduino Pro Micro
```

### 3. ROS2 Bridge Node
```bash
# src/launcher_bridge_node.cpp schreiben
# Serial Port Handler (async)
# Protocol Parser
# Topics: state, pitch, ready (publish)
# Topics: cmd_pitch, cmd_motor (subscribe)
# Services: arm, disarm, fire, emergency_stop
```

### 4. Joy Control Node
```bash
# src/launcher_joy_node.cpp schreiben
# Subscribe /joy
# Button mapping (A=arm, B=disarm, Y=motor, RT=fire)
# Axis mapping (Right Y = pitch)
# Publish zu /launcher/cmd_*
```

### 5. Build & Test
```bash
cd /home/pi/workspace/ros2_dev_ws/my_steel-robot_ws
./build.sh

# Terminal 1: Bridge Node
ros2 launch robot_nerf_launcher launcher_bridge.launch.py

# Terminal 2: Test Topics
ros2 topic echo /launcher/state
ros2 topic pub /launcher/cmd_pitch std_msgs/msg/Float32 "data: 15.0"

# Terminal 3: Test Services
ros2 service call /launcher/arm std_srvs/srv/Trigger
ros2 service call /launcher/fire std_srvs/srv/Trigger
```