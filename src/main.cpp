/**
 * ============================================================
 *  ESP32 二轴头追云台
 * ------------------------------------------------------------
 *  手里拿着 MPU6050 姿态传感器，手腕怎么动，
 *  两个 SG90 舵机组成的云台就跟着怎么转。
 *
 *  硬件接线：
 *    MPU6050 : VCC -> ESP32 3.3V
 *              GND -> ESP32 GND
 *              SCL -> ESP32 GPIO22
 *              SDA -> ESP32 GPIO21
 *    舵机1(俯仰/上臂): 信号 -> GPIO13, 电源 -> 5V, GND -> GND
 *    舵机2(偏航/底座): 信号 -> GPIO14, 电源 -> 5V, GND -> GND
 *
 *  姿态解算：Mahony 互补滤波（纯内置库实现，无需联网下载任何库）
 *  舵机控制：ESP32 硬件 LEDC 输出 50Hz PWM
 *
 *  调试：串口 115200 输出 pitch/roll/yaw 与舵机角度
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>

// ================== 引脚定义 ==================
#define PIN_SDA        21   // I2C 数据线（接 MPU6050 SDA）
#define PIN_SCL        22   // I2C 时钟线（接 MPU6050 SCL）
#define PIN_SERVO1     13   // 舵机1：俯仰（上臂，管抬头低头）
#define PIN_SERVO2     14   // 舵机2：偏航（底座，管左右转向）

// ================== MPU6050 寄存器 ==================
#define MPU_ADDR       0x68
#define REG_PWR_MGMT_1 0x6B   // 电源管理（唤醒用）
#define REG_SMPLRT_DIV 0x19   // 采样率分频
#define REG_CONFIG     0x1A   // 低通滤波
#define REG_GYRO_CFG   0x1B   // 陀螺仪量程
#define REG_ACCEL_CFG  0x1C   // 加速度计量程
#define REG_ACCEL_XH   0x3B   // 加速度数据起始地址
#define REG_GYRO_XH    0x43   // 陀螺仪数据起始地址

#define GYRO_SCALE     (500.0f / 32768.0f)   // 陀螺仪 ±500°/s 下每个 LSB 代表多少度/秒
#define ACCEL_SCALE    (4.0f / 32768.0f)     // 加速度计 ±4g 下每个 LSB 代表多少 g
#define DEG2RAD        (PI / 180.0f)
#define RAD2DEG        (180.0f / PI)

// ================== 舵机参数（SG90） ==================
#define SERVO_FREQ     50      // SG90 舵机刷新频率 50Hz（周期 20ms）
#define SERVO_US_MIN   500     // 0°   对应的脉宽（微秒）
#define SERVO_US_MAX   2500    // 180° 对应的脉宽（微秒）
#define SERVO_CENTER   90      // 上电默认角度

// ================== 姿态->舵机 映射配置 ==================
// 角度数组下标: 0=pitch(点头)  1=roll(歪头)  2=yaw(左右转)
// 如果发现"手左右转但云台不动/转错方向"，改下面 4 行即可，不用动其它代码
static int   SRC_SERVO1 = 0;   // 舵机1(俯仰) 跟随哪个姿态角: 0=pitch
static int   SRC_SERVO2 = 2;   // 舵机2(偏航) 跟随哪个姿态角: 2=yaw
static bool  INV_SERVO1 = true;    // 舵机1 方向反了改成 true（已开启：芯片下倾→云台上抬改为跟随）
static bool  INV_SERVO2 = false;   // 舵机2 方向反了改成 true

// 姿态角到舵机角的映射（增益放大）
// 原理：舵机角 = 90°(中位) + 姿态角 × 增益
// 增益 1.0 = 手转 1° 云台转 1°（1:1）；1.67 = 手转 30° 云台转 50°（小动作大响应）
#define MAP_GAIN_S1    1.33f   // 舵机1(俯仰) 增益：手抬 30° → 云台抬 40°
#define MAP_GAIN_S2    1.0f    // 舵机2(偏航) 增益：1.0 保持 1:1（想放大改成 1.67 即可）
#define DEADZONE_DEG     2.0f   // 输入死区：传感器角度靠近 0° 时不响应
#define SMOOTH_GAIN      0.15f  // 平滑系数（0~1，越小越平滑越迟钝；0.15=跟手且顺滑）
#define MAX_SPEED_DEG_S1 3.0f   // 舵机1(俯仰) 限速：值越大转得越快（3.0≈舵机全速，跟手）
#define MAX_SPEED_DEG_S2 3.0f   // 舵机2(偏航) 限速：跟手模式，随手而动
#define SERVO_DEADBAND   1.0f   // 防抖核心：目标角度与当前角度差小于 1° 就完全不动（既防抖又跟手）

// ================== Mahony 滤波参数 ==================
static const float Kp = 2.0f;   // 比例增益：加速度计纠正姿态的强度
static const float Ki = 0.05f;  // 积分增益：消除陀螺仪漂移
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // 四元数
static float iFBx = 0, iFBy = 0, iFBz = 0;                // 积分反馈

// 姿态角（度）
static float angPitch = 0, angRoll = 0, angYaw = 0;
// 当前输出的舵机角度
static float servo1Now = SERVO_CENTER, servo2Now = SERVO_CENTER;

// LEDC 通道
static int chServo1, chServo2;

// ================== 工具函数 ==================

// 写 MPU6050 寄存器
static void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// 读 MPU6050 寄存器
static uint8_t mpuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.read();
}

// MPU6050 初始化（唤醒 + 配置量程和采样率）
static bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    return false;   // I2C 上没发现 MPU6050
  }

  mpuWriteReg(REG_PWR_MGMT_1, 0x00);   // 退出睡眠，用内部 8MHz 时钟
  delay(50);
  mpuWriteReg(REG_SMPLRT_DIV, 0x01);   // 采样率 = 1kHz/(1+1) = 500Hz
  mpuWriteReg(REG_CONFIG, 0x02);       // DLPF 92Hz（滤掉高频噪声）
  mpuWriteReg(REG_GYRO_CFG, 0x08);     // 陀螺仪 ±500°/s
  mpuWriteReg(REG_ACCEL_CFG, 0x08);    // 加速度计 ±4g
  delay(100);
  return true;
}

// 读取原始数据（加速度 3 轴 + 陀螺仪 3 轴）
static void mpuReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                       int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XH);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  *ax = (int16_t)((Wire.read() << 8) | Wire.read());
  *ay = (int16_t)((Wire.read() << 8) | Wire.read());
  *az = (int16_t)((Wire.read() << 8) | Wire.read());
  // 跳过温度 2 字节
  Wire.read(); Wire.read();
  *gx = (int16_t)((Wire.read() << 8) | Wire.read());
  *gy = (int16_t)((Wire.read() << 8) | Wire.read());
  *gz = (int16_t)((Wire.read() << 8) | Wire.read());
}

// ================== Mahony 互补滤波 ==================
static void mahonyUpdate(float gx, float gy, float gz,   // 度/秒
                         float ax, float ay, float az,   // g
                         float dt) {
  float norm, halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx *= DEG2RAD; gy *= DEG2RAD; gz *= DEG2RAD;   // 转弧度/秒

  // 由四元数算出"期望的重力方向"（向量叉积用的中间量）
  halfvx = q1 * q3 - q0 * q2;
  halfvy = q0 * q1 + q2 * q3;
  halfvz = q0 * q0 - 0.5f + q3 * q3;

  // 加速度计测出的实际重力方向，归一化成长度为 1 的向量
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return;
  norm = 1.0f / norm;
  ax *= norm; ay *= norm; az *= norm;

  // 两个方向的叉积 = 姿态误差
  halfex = (ay * halfvz - az * halfvy);
  halfey = (az * halfvx - ax * halfvz);
  halfez = (ax * halfvy - ay * halfvx);

  // 积分项：慢慢修正陀螺仪漂移
  if (Ki > 0.0f) {
    iFBx += Ki * halfex * dt;
    iFBy += Ki * halfey * dt;
    iFBz += Ki * halfez * dt;
    gx += iFBx; gy += iFBy; gz += iFBz;
  } else {
    iFBx = iFBy = iFBz = 0;
  }

  // 比例项：立即把姿态拉向加速度计判断的方向
  gx += Kp * halfex;
  gy += Kp * halfey;
  gz += Kp * halfez;

  // 四元数积分（一阶龙格库塔），把角速度变成姿态角增量
  qa = q0; qb = q1; qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz) * 0.5f * dt;
  q1 += ( qa * gx + qc * gz - q3 * gy) * 0.5f * dt;
  q2 += ( qa * gy - qb * gz + q3 * gx) * 0.5f * dt;
  q3 += ( qa * gz + qb * gy - qc * gx) * 0.5f * dt;

  // 归一化四元数，防止数值漂移
  norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  norm = 1.0f / norm;
  q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}

// 四元数 -> 欧拉角（度）
static void quat2Euler() {
  // 防止 asinf 参数超出 [-1,1] 导致 NaN
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;

  angPitch = asinf(sinp) * RAD2DEG;                                   // 俯仰
  angRoll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                    1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;      // 横滚
  angYaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;      // 偏航
}

// ================== 舵机 PWM ==================
// 微秒脉宽 -> LEDC 占空比（16bit）
static uint32_t us2duty(float us) {
  return (uint32_t)(us * 65535.0f / 1000000.0f / (1.0f / SERVO_FREQ));
}

// 舵机角度 -> 脉宽微秒（线性）
static float deg2us(float deg) {
  return SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN) * deg / 180.0f;
}

// 写舵机角度
static void servoWrite(int ch, float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  ledcWrite(ch, us2duty(deg2us(deg)));
}

// ================== 角度映射与平滑 ==================
// 姿态角 -> 目标舵机角（带死区 + 增益放大 + 行程限位）
static float angleToServo(float in, bool invert, float gain) {
  if (invert) in = -in;
  // 死区：靠近中间时不做微小响应
  if (fabsf(in) < DEADZONE_DEG) in = 0.0f;
  float out = 90.0f + in * gain;   // 中心 90°，按增益放大
  if (out < 0.0f) out = 0.0f;      // 限位：不能超过舵机物理行程
  if (out > 180.0f) out = 180.0f;
  return out;
}

// 指数平滑 + 限速 + 目标死区，输出最终舵机角度
// maxSpeed: 该舵机的最大转速（度/周期）
static float smoothServo(float target, float now, float maxSpeed) {
  // 目标没变化超过死区 → 完全不动（这是消除"刺啦刺啦"的关键）
  if (fabsf(target - now) < SERVO_DEADBAND) return now;
  float next = now + (target - now) * SMOOTH_GAIN;   // 一阶低通
  float delta = next - now;
  if (delta >  maxSpeed) delta =  maxSpeed;   // 限速
  if (delta < -maxSpeed) delta = -maxSpeed;
  return now + delta;
}

// ================== 主程序 ==================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n==== ESP32 二轴头追云台 ===="));

  Wire.begin(PIN_SDA, PIN_SCL);        // 指定 I2C 引脚
  Wire.setClock(400000);               // 400kHz 快速模式

  if (!mpuInit()) {
    Serial.println(F("[错误] 没找到 MPU6050！请检查 SDA/SCL 接线和 3.3V 供电。"));
    while (1) { delay(1000); }         // 死等，方便排查
  }
  Serial.println(F("[OK] MPU6050 初始化成功 (500Hz)"));

  // 舵机 PWM：50Hz，16bit 分辨率
  chServo1 = 0; ledcSetup(chServo1, SERVO_FREQ, 16); ledcAttachPin(PIN_SERVO1, chServo1);
  chServo2 = 1; ledcSetup(chServo2, SERVO_FREQ, 16); ledcAttachPin(PIN_SERVO2, chServo2);
  servoWrite(chServo1, SERVO_CENTER);
  servoWrite(chServo2, SERVO_CENTER);
  Serial.println(F("[OK] 舵机已归中 (90°)"));
  Serial.println(F("请把 MPU6050 放平，等待 2 秒让姿态收敛..."));

  // 给滤波和舵机 2 秒稳定时间
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    int16_t ax, ay, az, gx, gy, gz;
    mpuReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
    uint32_t now = micros();
    static uint32_t last = now;
    float dt = (now - last) / 1000000.0f;
    last = now;
    if (dt <= 0.0f || dt > 0.01f) dt = 0.002f;   // 兜底
    mahonyUpdate(gx * GYRO_SCALE, gy * GYRO_SCALE, gz * GYRO_SCALE,
                 ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE, dt);
  }
  quat2Euler();
  Serial.println(F("[OK] 就绪！现在转动 MPU6050 试试，云台会跟随你的手。"));
}

void loop() {
  // 1) 读传感器
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(&ax, &ay, &az, &gx, &gy, &gz);

  // 2) 姿态解算（dt 用真实时间，保持稳定）
  uint32_t now = micros();
  static uint32_t last = now;
  float dt = (now - last) / 1000000.0f;
  last = now;
  if (dt <= 0.0f || dt > 0.01f) dt = 0.002f;
  mahonyUpdate(gx * GYRO_SCALE, gy * GYRO_SCALE, gz * GYRO_SCALE,
               ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE, dt);
  quat2Euler();

  // 3) 姿态角 -> 舵机目标角 -> 平滑输出
  float ang[3] = { angPitch, angRoll, angYaw };
  float t1 = angleToServo(ang[SRC_SERVO1], INV_SERVO1, MAP_GAIN_S1);
  float t2 = angleToServo(ang[SRC_SERVO2], INV_SERVO2, MAP_GAIN_S2);
  servo1Now = smoothServo(t1, servo1Now, MAX_SPEED_DEG_S1);
  servo2Now = smoothServo(t2, servo2Now, MAX_SPEED_DEG_S2);
  // 钳到合法范围（0~180°），防止显示超限值误导
  servo1Now = constrain(servo1Now, 0.0f, 180.0f);
  servo2Now = constrain(servo2Now, 0.0f, 180.0f);
  servoWrite(chServo1, servo1Now);
  servoWrite(chServo2, servo2Now);

  // 4) 串口调试输出（约每 50ms 打印一次）
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 50) {
    lastPrint = millis();
    Serial.printf("pitch=%6.1f  roll=%6.1f  yaw=%6.1f  |  舵机1=%5.1f  舵机2=%5.1f\n",
                  angPitch, angRoll, angYaw, servo1Now, servo2Now);
  }
}
