extern "C" {
  #include "driver/twai.h"
}

#define CAN_TX_GPIO 20
#define CAN_RX_GPIO 21

static const uint32_t CAN_ID_REQ  = 0x100;
static const uint32_t CAN_ID_RSP1 = 0x101;
static const uint32_t CAN_ID_RSP2 = 0x102;

#define CAN_MODE TWAI_MODE_NORMAL

static uint8_t seq = 0;

static inline int16_t be16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
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
  Serial.printf("install=%d start=%d\n", install_ret, start_ret);

  twai_clear_transmit_queue();
  twai_clear_receive_queue();
  return (install_ret == 0 && start_ret == 0);
}

void send_req(uint8_t s) {
  twai_message_t m = {};
  m.extd = 0;
  m.rtr  = 0;
  m.identifier = CAN_ID_REQ;
  m.data_length_code = 3;
  m.data[0] = 0xAA;
  m.data[1] = 0x55;
  m.data[2] = s;

  esp_err_t e = twai_transmit(&m, pdMS_TO_TICKS(50));
  Serial.printf("REQ TX %s | seq=%u\n", (e==ESP_OK)?"ok":"fail", s);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nNode A (Requester) boot...");

  if (!can_init()) Serial.println("CAN init FAILED");
  Serial.println("Node A ready.");
}

void loop() {
  seq++;
  send_req(seq);

  bool got1=false, got2=false;
  int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  uint8_t rsp2_seq = 0xFF;

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 300;

  while (millis() - t0 < TIMEOUT_MS) {
    twai_message_t r;
    if (twai_receive(&r, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    if (r.extd) continue;

    Serial.printf("RX id=0x%X dlc=%d\n", (unsigned)r.identifier, (int)r.data_length_code);

    if (r.identifier == CAN_ID_RSP1 && r.data_length_code == 8) {
      ax = be16(r.data[0], r.data[1]);
      ay = be16(r.data[2], r.data[3]);
      az = be16(r.data[4], r.data[5]);
      gx = be16(r.data[6], r.data[7]);
      got1 = true;
    }

    if (r.identifier == CAN_ID_RSP2 && r.data_length_code == 8) {
      gy = be16(r.data[0], r.data[1]);
      gz = be16(r.data[2], r.data[3]);
      rsp2_seq = r.data[7];
      if (rsp2_seq == seq) got2 = true;

      Serial.printf("RSP2 seq_in_pkt=%u expect=%u\n", rsp2_seq, seq);
    }

    if (got1 && got2) break;
  }

  if (got1 && got2) {
    Serial.printf("IMU ✅ ax=%d ay=%d az=%d gx=%d gy=%d gz=%d (seq=%u)\n",
                  ax, ay, az, gx, gy, gz, seq);
  } else {
    Serial.printf("timeout ❌ got1=%d got2=%d (seq=%u, rsp2_seq=%u)\n",
                  got1, got2, seq, rsp2_seq);
  }

  delay(200);
}

