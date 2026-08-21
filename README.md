# ESP32 二轴头追云台（Head Tracking Gimbal）

手里拿着 MPU6050 姿态传感器，**手怎么动，两个 SG90 舵机组成的云台就怎么转**。零外网依赖（纯 Arduino 内置库实现），编译烧录全程离线。

![硬件组成](docs/接线图.html)

## ✨ 功能特点

- **随手而动**：手部姿态 500Hz 实时解算（Mahony 互补滤波），云台零延迟跟随
- **小动作大响应**：动作增益可调（默认俯仰 1.33 倍：手抬 30° → 云台抬 40°）
- **防抖**：目标角度死区 + 指数平滑 + 限速，静止不抖、不刺啦
- **方向可调**：程序预留方向开关，装反了改一行即可
- **零外网依赖**：MPU6050 驱动、姿态解算、舵机 PWM 全部用 ESP32 内置库手写，无需下载任何第三方库

## 🛠 硬件清单

| 器件 | 数量 |
|---|---|
| ESP32-CH340C 开发板 | 1 |
| MPU6050 六轴姿态传感器 | 1 |
| SG90 舵机 | 2 |
| 面包板 + 电源模块（MB102） | 1 |
| 5V 电源（测试可用 USB 供电） | 1 |
| 杜邦线 | 若干 |

## 🔌 接线

完整接线图见 [docs/接线图.html](docs/接线图.html)（浏览器打开）。

速查表：

| 器件 | 引脚 | 接到 ESP32 |
|---|---|---|
| MPU6050 | VCC | 3V3 |
| MPU6050 | GND | GND |
| MPU6050 | SCL | GPIO22 |
| MPU6050 | SDA | GPIO21 |
| 舵机1（俯仰/上臂） | 棕=GND / 红=5V / 橙=信号 | 信号 → GPIO13 |
| 舵机2（偏航/底座） | 棕=GND / 红=5V / 橙=信号 | 信号 → GPIO14 |

> ⚠️ 要点：MPU6050 只能接 3.3V；舵机接 5V，两舵机电源并联；**舵机 GND 必须与 ESP32 的 GND 共地**；测试用 USB 供电，堵转/重启时给舵机单独接 5V 2A 电源。

## 🚀 烧录

### 方案 A：直接刷固件（推荐，无需装编译环境）

本仓库 `firmware/` 目录已提供编译好的固件。需要 Python 3：

```bash
pip install esptool -i https://pypi.tuna.tsinghua.edu.cn/simple
cd firmware
python -m esptool --chip esp32 --port COM3 --baud 921600 write_flash -z 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
```

（`COM3` 换成你的端口号）

### 方案 B：源码编译（PlatformIO）

```bash
# 安装 PlatformIO 后，在项目根目录执行：
platformio run -t upload --upload-port COM3
```

## 🎮 使用

1. 上电后等 2 秒，舵机自动转到 90° 中位
2. 拿起 MPU6050 上下点头 → 舵机1 跟随；左右转手 → 舵机2 跟随
3. 串口监视器 115200 可看实时角度（可选）

## ⚙️ 参数调节

所有手感参数都在 `src/main.cpp` 顶部，改完重新烧录即可：

| 想调什么 | 改哪里 |
|---|---|
| 舵机方向反了 | `INV_SERVO1` / `INV_SERVO2` 改为 `true` |
| 动作放大倍数 | `MAP_GAIN_S1` / `MAP_GAIN_S2`（1.0=1:1，1.33=手30°云台40°） |
| 转动速度 | `MAX_SPEED_DEG_S1` / `S2`（越大越快） |
| 防抖灵敏度 | `SERVO_DEADBAND`（1.0 = 默认，调大到 2~3 更抗抖） |
| 跟随平滑度 | `SMOOTH_GAIN`（越小越平滑越迟钝） |

## ❓ 常见问题

- **舵机刺啦响**：供电不足最常见。给舵机单独接 5V 2A 电源（GND 与 ESP32 共地）；或调大 `SERVO_DEADBAND`
- **云台不动**：查接线、查共地、查 COM 口；MPU6050 必须接 3.3V
- **方向反**：见参数表，改一行重烧

## 📁 仓库结构

```
├── README.md          # 本文档
├── platformio.ini     # PlatformIO 工程配置
├── src/
│   └── main.cpp       # 全部源码（含详细中文注释）
├── docs/
│   └── 接线图.html     # 接线图
└── firmware/          # 编译好的固件（可直接烧录）
    ├── firmware.bin
    ├── bootloader.bin
    ├── partitions.bin
    └── boot_app0.bin
```

## 📜 技术说明

- 姿态解算：Mahony 互补滤波（Kp=2.0, Ki=0.05），四元数转欧拉角，500Hz 更新
- 舵机控制：ESP32 LEDC 硬件 PWM，50Hz，0.5ms~2.5ms 脉宽对应 0~180°
- 防抖链：输入死区(2°) → 增益映射 → 目标死区(1°) → 指数平滑(0.15) → 限速(3°/周期)

---

MIT License
