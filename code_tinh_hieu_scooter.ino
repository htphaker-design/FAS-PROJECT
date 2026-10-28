#include <avr/wdt.h> // Watchdog chống treo mạch khi chạy 24/7

// =========================================================================
// CẤU HÌNH MỨC KÍCH RIÊNG CHO CHÂN 11 VÀ CHÂN 12
// =========================================================================
// 1. Cấu hình cho CHÂN 12 (Đang dùng Mạch Relay kích mức THẤP - Active LOW):
#define PIN12_ON_STATE    LOW   // Xuất 0V (LOW) để BẬT Relay chân 12
#define PIN12_OFF_STATE   HIGH  // Xuất 5V (HIGH) để TẮT Relay chân 12

// 2. Cấu hình cho CHÂN 11:
// - Nếu chân 11 cũng nối vào Relay mức thấp giống chân 12 -> Để là LOW và HIGH
// - Nếu chân 11 nối vào Đèn LED thường -> Đổi thành HIGH (ON) và LOW (OFF)
#define PIN11_ON_STATE    LOW   // Đổi thành HIGH nếu chân 11 là đèn LED
#define PIN11_OFF_STATE   HIGH  // Đổi thành LOW nếu chân 11 là đèn LED

// --- KHAI BÁO CHÂN PHẦN CỨNG ---
#define BUTTON_PIN       A0 // Chân nút nhấn kích tín hiệu (Nối GND)
#define PIN_20S          11 // Chân kích 20 giây khi nhận tín hiệu từ mạch kia
#define PIN_3S           12 // Chân kích 3 giây khi nhấn A0 trong lúc chân 11 đang kích
#define LINK_LED_PIN     22 // Chân đèn LED báo kết nối sóng LoRa (Chớp tắt chu kỳ 1s)

#define PIN_M0           4
#define PIN_M1           5
#define PIN_AUX          6

#define LoRaSerial Serial1  // Chân 18 (TX1), 19 (RX1) trên Arduino Mega

// --- ĐỊNH NGHĨA GIAO THỨC KHUNG TRUYỀN 5 BYTE ---
#define HEADER_1         0xAA
#define HEADER_2         0x55
#define CMD_TRIG_20S     0x01 // Lệnh kích chân 11 trong 20 giây ở mạch nhận
#define CMD_ACK          0x02 // Lệnh phản hồi đã nhận thành công
#define CMD_TRIG_3S      0x03 // Lệnh kích chân 12 trong 3 giây khi xác nhận trong 20s
#define CMD_PING         0x04 // Gói tin dò sóng định kỳ (Heartbeat cho đèn chân 22)
#define FRAME_LEN        5

// --- CẤU HÌNH THỜI GIAN (NON-BLOCKING MILLIS) ---
const unsigned long DURATION_20S       = 120000UL; // Thời gian kích chân 11: 20 giây
const unsigned long DURATION_3S        = 5000UL;  // Thời gian kích chân 12: 3 giây
const unsigned long BLINK_HALF_PERIOD  = 500UL;   // Đèn chân 22 chớp tắt chu kỳ 1s (500ms Sáng + 500ms Tắt)
const unsigned long HEARTBEAT_INTERVAL = 2000UL;  // Chu kỳ phát gói dò sóng: 2 giây
const unsigned long LINK_TIMEOUT       = 5500UL;  // Quá 5.5 giây không nhận gói -> Mất sóng

const unsigned long RELEASE_STABLE_MS  = 200UL;   // Chân A0 phải nhả (HIGH) đủ 200ms mới mở khóa nhận nút
const unsigned long PRESS_DEBOUNCE_MS  = 80UL;    // Chân A0 phải nhấn (LOW) giữ đủ 80ms mới tính là hợp lệ
const unsigned long ACK_TIMEOUT        = 600UL;   // Chờ phản hồi ACK: 600 ms
const unsigned long UART_TIMEOUT       = 150UL;   // Tự xóa rác UART sau 150 ms
const uint8_t       MAX_RETRIES        = 5;       // Số lần phát lại tối đa

const bool TRIGGER_PIN12_BOTH_SIDES = true;

// --- BIẾN QUẢN LÝ CHÂN 11 (20S) & CHÂN 12 (3S) ---
bool isPin11Active = false;
unsigned long pin11StartTime = 0;

bool isPin12Active = false;
unsigned long pin12StartTime = 0;

// --- BIẾN QUẢN LÝ ĐÈN BÁO SÓNG CHÂN 22 ---
bool isLoRaLinked = false;
unsigned long lastLoRaRxTime = 0;
unsigned long lastHeartbeatTxTime = 0;
unsigned long lastBlinkTime = 0;
uint8_t linkLedState = LOW;

// --- BIẾN CHỐNG KÍCH GIẢ NÚT NHẤN A0 ---
bool buttonArmed = false;
unsigned long highStateTimer = 0;
unsigned long lowStateTimer = 0;

// --- BIẾN TRUYỀN NHẬN LORA ---
bool waitingForAck = false;
uint8_t pendingCmd = CMD_TRIG_20S;
uint8_t currentPacketID = 0;
uint8_t retryCount = 0;
unsigned long lastTxTime = 0;

uint8_t rxBuffer[FRAME_LEN];
uint8_t rxIndex = 0;
unsigned long lastRxByteTime = 0;

uint8_t calculateCRC8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
    }
  }
  return crc;
}

void waitAuxHigh(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (digitalRead(PIN_AUX) == LOW) {
    if (millis() - start >= timeoutMs) break;
    wdt_reset();
  }
}

void configureLoRaMaxRange() {
  digitalWrite(PIN_M0, LOW);
  digitalWrite(PIN_M1, HIGH);
  delay(50);
  waitAuxHigh(500);

  while (LoRaSerial.available()) LoRaSerial.read();

  const uint8_t configCmd[] = {0xC0, 0x00, 0x07, 0x00, 0x00, 0x00, 0x62, 0x00, 0x28, 0x13};
  LoRaSerial.write(configCmd, sizeof(configCmd));
  LoRaSerial.flush();

  delay(100);
  waitAuxHigh(500);

  while (LoRaSerial.available()) LoRaSerial.read();

  digitalWrite(PIN_M0, LOW);
  digitalWrite(PIN_M1, LOW);
  delay(50);
  waitAuxHigh(500);

  while (LoRaSerial.available()) LoRaSerial.read();
}

bool sendLoRaFrame(uint8_t cmd, uint8_t packetID) {
  if (digitalRead(PIN_AUX) == LOW) return false;

  uint8_t frame[FRAME_LEN];
  frame[0] = HEADER_1;
  frame[1] = HEADER_2;
  frame[2] = cmd;
  frame[3] = packetID;
  frame[4] = calculateCRC8(frame, 4);

  LoRaSerial.write(frame, FRAME_LEN);
  return true;
}

// Bật chân 11 trong 20 giây
void activatePin11For20s(unsigned long now) {
  digitalWrite(PIN_20S, PIN11_ON_STATE);
  isPin11Active = true;
  pin11StartTime = now;
}

// Bật chân 12 trong 3 giây
void activatePin12For3s(unsigned long now) {
  digitalWrite(PIN_3S, PIN12_ON_STATE);
  isPin12Active = true;
  pin12StartTime = now;
}

void startReliableTx(uint8_t cmd, unsigned long now) {
  currentPacketID++;
  pendingCmd = cmd;
  retryCount = 0;
  waitingForAck = true;

  if (sendLoRaFrame(pendingCmd, currentPacketID)) {
    lastTxTime = now;
  } else {
    lastTxTime = now - ACK_TIMEOUT;
  }
}

void setup() {
  wdt_disable();

  // Đưa chân 11 và 12 lên mức HIGH (5V) TRƯỚC khi gọi pinMode OUTPUT
  // giúp Relay mức thấp không bị nháy "tạch" lúc mới cấp nguồn
  digitalWrite(PIN_20S, PIN11_OFF_STATE);
  pinMode(PIN_20S, OUTPUT);
  digitalWrite(PIN_20S, PIN11_OFF_STATE);

  digitalWrite(PIN_3S, PIN12_OFF_STATE);
  pinMode(PIN_3S, OUTPUT);
  digitalWrite(PIN_3S, PIN12_OFF_STATE);

  digitalWrite(LINK_LED_PIN, LOW);
  pinMode(LINK_LED_PIN, OUTPUT);
  digitalWrite(LINK_LED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_AUX, INPUT_PULLUP);

  LoRaSerial.begin(9600);
  configureLoRaMaxRange();

  randomSeed(analogRead(A5));
  unsigned long now = millis();
  lastHeartbeatTxTime = now - random(0, 1000);
  highStateTimer = now;
  lowStateTimer = now;

  wdt_enable(WDTO_4S);
}

void loop() {
  wdt_reset();
  unsigned long currentMillis = millis();

  // 1. ĐỌC CHÂN A0
  uint8_t reading = digitalRead(BUTTON_PIN);

  if (reading == HIGH) {
    lowStateTimer = currentMillis;
    if (!buttonArmed && (currentMillis - highStateTimer >= RELEASE_STABLE_MS)) {
      buttonArmed = true;
    }
  } else {
    highStateTimer = currentMillis;

    if (buttonArmed && (currentMillis - lowStateTimer >= PRESS_DEBOUNCE_MS)) {
      buttonArmed = false;

      if (isPin11Active) {
        activatePin12For3s(currentMillis);
        if (TRIGGER_PIN12_BOTH_SIDES) {
          startReliableTx(CMD_TRIG_3S, currentMillis);
        }
      } else {
        startReliableTx(CMD_TRIG_20S, currentMillis);
      }
    }
  }

  // 2. TỰ ĐỘNG PHÁT LẠI KHI CHƯA NHẬN ĐƯỢC ACK
  if (waitingForAck) {
    if (currentMillis - lastTxTime >= ACK_TIMEOUT) {
      if (retryCount < MAX_RETRIES) {
        if (sendLoRaFrame(pendingCmd, currentPacketID)) {
          retryCount++;
          lastTxTime = currentMillis;
        }
      } else {
        waitingForAck = false;
      }
    }
  }

  // 3. PHÁT GÓI DÒ SÓNG ĐỊNH KỲ (CHO ĐÈN CHÂN 22)
  if (!waitingForAck && (currentMillis - lastHeartbeatTxTime >= HEARTBEAT_INTERVAL)) {
    if (sendLoRaFrame(CMD_PING, 0)) {
      lastHeartbeatTxTime = currentMillis;
    }
  }

  // 4. NHẬN DỮ LIỆU LORA, KIỂM TRA CRC8 & XỬ LÝ LỆNH
  if (rxIndex > 0 && (currentMillis - lastRxByteTime > UART_TIMEOUT)) {
    rxIndex = 0;
  }

  while (LoRaSerial.available() > 0) {
    uint8_t b = LoRaSerial.read();
    lastRxByteTime = currentMillis;

    if (rxIndex == 0 && b != HEADER_1) continue;
    if (rxIndex == 1 && b != HEADER_2) {
      rxIndex = (b == HEADER_1) ? 1 : 0;
      continue;
    }

    rxBuffer[rxIndex++] = b;

    if (rxIndex == FRAME_LEN) {
      rxIndex = 0;

      if (calculateCRC8(rxBuffer, 4) == rxBuffer[4]) {
        uint8_t cmd = rxBuffer[2];
        uint8_t pktID = rxBuffer[3];

        lastLoRaRxTime = currentMillis;
        isLoRaLinked = true;
        lastHeartbeatTxTime = currentMillis - (HEARTBEAT_INTERVAL / 2);

        if (cmd == CMD_TRIG_20S) {
          activatePin11For20s(currentMillis);
          waitAuxHigh(100);
          sendLoRaFrame(CMD_ACK, pktID);
        } 
        else if (cmd == CMD_TRIG_3S) {
          activatePin12For3s(currentMillis);
          waitAuxHigh(100);
          sendLoRaFrame(CMD_ACK, pktID);
        }
        else if (cmd == CMD_ACK) {
          if (waitingForAck && pktID == currentPacketID) {
            waitingForAck = false;
          }
        }
      }
    }
  }

  // 5. HẸN GIỜ TẮT CHÂN 11 (SAU 20 GIÂY) & TẮT CHÂN 12 (SAU 3 GIÂY)
  if (isPin11Active && (currentMillis - pin11StartTime >= DURATION_20S)) {
    digitalWrite(PIN_20S, PIN11_OFF_STATE);
    isPin11Active = false;
  }

  if (isPin12Active && (currentMillis - pin12StartTime >= DURATION_3S)) {
    digitalWrite(PIN_3S, PIN12_OFF_STATE);
    isPin12Active = false;
  }

  // 6. ĐIỀU KHIỂN ĐÈN BÁO SÓNG CHÂN 22 (CHỚP TẮT CHU KỲ 1S KHI CÓ SÓNG)
  if (isLoRaLinked && (currentMillis - lastLoRaRxTime > LINK_TIMEOUT)) {
    isLoRaLinked = false;
  }

  if (isLoRaLinked) {
    if (currentMillis - lastBlinkTime >= BLINK_HALF_PERIOD) {
      lastBlinkTime = currentMillis;
      linkLedState = (linkLedState == LOW) ? HIGH : LOW;
      digitalWrite(LINK_LED_PIN, linkLedState);
    }
  } else {
    if (linkLedState != LOW) {
      linkLedState = LOW;
      digitalWrite(LINK_LED_PIN, LOW);
    }
  }
}