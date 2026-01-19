#include <Wire.h>
#include <MPU6050.h>

extern "C" {
  #include "driver/twai.h"
}

#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

#define I2C_SDA 10
#define I2C_SCL 9

// ===================== CAN IDs =====================
static const uint32_t CAN_ID_REQ  = 0x100;  
static const uint32_t CAN_ID_RSP1 = 0x101;  
static const uint32_t CAN_ID_RSP2 = 0x102;  
#define CAN_MODE TWAI_MODE_NORMAL
// #define CAN_MODE TWAI_MODE_NO_ACK   
// ===================== IMU =====================
MPU6050 imu;          
bool imu_ok = false;

// ===================== Helpers =====================
static inline void put_be16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)((v >> 8) & 0xFF);
  p[1] = (uint8_t)(v & 0xFF);
}

void i2cScan() {
  Serial.println("I2C scan...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  (no I2C devices found)");
  Serial.println("I2C scan done.");
}

bool imu_init() {
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  i2cScan();
  imu.initialize();
  delay(10);
  return imu.testConnection();
}

bool can_init() {
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_GPIO,
                                (gpio_num_t)CAN_RX_GPIO,
                                CAN_MODE);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  int install_ret = (int)twai_driver_install(&g, &t, &f);
  int start_ret   = (int)twai_start();

  Serial.printf("TWAI install=%d start=%d\n", install_ret, start_ret);

  twai_clear_transmit_queue();
  twai_clear_receive_queue();

  return (install_ret == 0 && start_ret == 0);
}

bool send_frame(uint32_t id, const uint8_t *data, uint8_t dlc) {
  twai_message_t m = {};
  m.extd = 0;
  m.rtr  = 0;
  m.identifier = id;
  m.data_length_code = dlc;
  for (int i = 0; i < dlc; i++) m.data[i] = data[i];

  esp_err_t e = twai_transmit(&m, pdMS_TO_TICKS(50));
  return (e == ESP_OK);
}

void send_imu_reply(uint8_t seq, int16_t ax, int16_t ay, int16_t az,
                    int16_t gx, int16_t gy, int16_t gz) {
  // RSP1: ax ay az gx (8 bytes)
  uint8_t d1[8];
  put_be16(&d1[0], ax);
  put_be16(&d1[2], ay);
  put_be16(&d1[4], az);
  put_be16(&d1[6], gx);

  // RSP2: gy gz seq (7 bytes)
  uint8_t d2[8] = {0};
  put_be16(&d2[0], gy);
  put_be16(&d2[2], gz);
  d2[4] = seq;

  bool ok1 = send_frame(CAN_ID_RSP1, d1, 8);
  bool ok2 = send_frame(CAN_ID_RSP2, d2, 8);

  Serial.printf("RSP1 TX %s | RSP2 TX %s | seq=%u\n",
                ok1 ? "ok" : "FAIL",
                ok2 ? "ok" : "FAIL",
                seq);
}

// ===================== Arduino =====================
static uint8_t seq = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\nNode B (IMU Responder) boot...");
  Serial.printf("I2C SDA=%d SCL=%d | CAN TX=%d RX=%d\n", I2C_SDA, I2C_SCL, CAN_TX_GPIO, CAN_RX_GPIO);

  imu_ok = imu_init();
  Serial.println(imu_ok ? "IMU OK" : "IMU FAIL (check SDA/SCL/VCC/GND/address)");

  bool can_ok = can_init();
  Serial.println(can_ok ? "CAN OK" : "CAN FAIL (driver install/start failed)");

  Serial.println("Node B ready.");
}

void loop() {
  twai_message_t r;
  if (twai_receive(&r, pdMS_TO_TICKS(1000)) != ESP_OK) return;

  if (!r.extd && r.identifier == CAN_ID_REQ) {
    seq++;

    Serial.printf("REQ RX | dlc=%d | seq=%u\n", (int)r.data_length_code, seq);

    if (!imu_ok) {
      Serial.println("IMU not ready -> skip send");
      return;
    }

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  
    Serial.printf("IMU raw | ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n",
                  ax, ay, az, gx, gy, gz);

    send_imu_reply(seq, ax, ay, az, gx, gy, gz);
  }
}


