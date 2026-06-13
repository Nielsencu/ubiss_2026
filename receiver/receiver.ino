/*
 * Combined: 28BYJ-48 stepper + 4-digit 7-segment display + bidirectional ESP-NOW
 * ESP32-WROVER — dual-core
 *
 * Core 1 (loop):        stepper motor (blocking step() calls are fine here)
 * Core 0 (display task): 7-seg multiplexing  (shares Core 0 with the WiFi stack)
 *
 * ESP-NOW: both boards run THIS sketch; set BOARD_A true on one, false on the
 * other, fill in both MACs. A heartbeat carrying the current display value is
 * sent each loop pass; incoming messages are printed (add your reaction at the
 * "<-- hook" in onReceive).
 *
 * Stepper pins (via ULN2003):
 *   IN1 -> GPIO13,  IN3 -> GPIO27,  IN2 -> GPIO14,  IN4 -> GPIO26
 *
 * 7-segment display pins:
 *   Digits  D1-D4 : GPIO 2, 32, 33, 15
 *   Segments a-g, DP: GPIO 4, 5, 18, 19, 21, 22, 23, 25
 *
 * WiFi uses no GPIOs, so there are no pin conflicts with the above.
 */
#include <Stepper.h>
#include <WiFi.h>
#include <esp_now.h>

// ============================================================
//  ESP-NOW configuration
// ============================================================
#define BOARD_A false   // true on one board, false on the other

uint8_t macOfA[] = {0xE0, 0x8C, 0xFE, 0x57, 0xDE, 0x58};  // <-- Board A's MAC
uint8_t macOfB[] = {0xF4, 0x2D, 0xC9, 0x76, 0xED, 0x74};  // <-- Board B's MAC
uint8_t *peerMAC = BOARD_A ? macOfB : macOfA;             // talk to the other one

typedef struct {
  int   id;
  float value;
  char  text[16];
} Message;

Message outgoing;
Message incoming;

// Heartbeat cadence. The loop blocks for ~seconds per pass (stepper), so in
// practice this fires about once per loop iteration.
const unsigned long SEND_INTERVAL_MS = 2000UL;
unsigned long lastSendMs = 0;

// -------- STEPPER SETUP -------- //
const int stepsPerRevolution = 2048;
const int quarterTurn = stepsPerRevolution / 4;
//                                     IN1  IN3  IN2  IN4
Stepper myStepper(stepsPerRevolution, 13,  27,  14,  26);

// -------- 7-SEGMENT DISPLAY SETUP -------- //
const byte PATTERNS[] = {
  0x3F, // 0
  0x06, // 1
  0x5B, // 2
  0x4F, // 3
  0x66, // 4
  0x6D, // 5
  0x7D, // 6
  0x07, // 7
  0x7F, // 8
  0x6F  // 9
};
struct SevenSeg {
  int digits[4];
  int segments[8];
  int counter;
};
SevenSeg display;
volatile int displayValue = 0;

void clearAll() {
  for (int i = 0; i < 4; i++) {
    pinMode(display.digits[i], OUTPUT);
    digitalWrite(display.digits[i], LOW);
  }
  for (int i = 0; i < 8; i++)
    pinMode(display.segments[i], INPUT);
}
void showDigit(int pos, int num) {
  byte pattern = PATTERNS[num];
  digitalWrite(display.digits[pos], HIGH);
  for (int s = 0; s < 7; s++) {
    if (pattern & (1 << s)) {
      pinMode(display.segments[s], OUTPUT);
      digitalWrite(display.segments[s], LOW);
    }
  }
}
void showNumber(int num) {
  clearAll();
  int pos = 3 - display.counter;
  int divisor = 1;
  for (int i = 0; i < display.counter; i++) divisor *= 10;
  showDigit(pos, (num / divisor) % 10);
  display.counter = (display.counter + 1) % 4;
}

// -------- DISPLAY TASK (runs on Core 0) -------- //
void displayTask(void* param) {
  for (;;) {
    showNumber(displayValue);
    vTaskDelay(pdMS_TO_TICKS(4));
  }
}

// ============================================================
//  ESP-NOW module
// ============================================================
// NOTE: on arduino-esp32 3.x the send-callback's first argument is
// const wifi_tx_info_t* (it used to be const uint8_t* mac).
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "  [sent ok]" : "  [send FAIL]");
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));
  Serial.printf("RECV from %02X:%02X:%02X:%02X:%02X:%02X -> id=%d value=%.1f text=%s\n",
                info->src_addr[0], info->src_addr[1], info->src_addr[2],
                info->src_addr[3], info->src_addr[4], info->src_addr[5],
                incoming.id, incoming.value, incoming.text);
  if(incoming.id == 1) {
    displayValue++;
  }
  // <-- hook: react to a received message here. For example, to show the
  // OTHER board's number on your display instead of your own counter:
  //     displayValue = incoming.id;
  // Keep work here short — this runs in the WiFi callback context.
}

void sendMessage(int id, float value, const char *text) {
  outgoing.id    = id;
  outgoing.value = value;
  strncpy(outgoing.text, text, sizeof(outgoing.text) - 1);
  outgoing.text[sizeof(outgoing.text) - 1] = '\0';
  esp_now_send(peerMAC, (uint8_t *)&outgoing, sizeof(outgoing));
}

void espnowInit() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
  }
}

// ============================================================
//  MAIN
// ============================================================
void setup() {
  Serial.begin(115200);

  // Bring up the radio (its tasks live on Core 0).
  espnowInit();

  myStepper.setSpeed(10);

  display = {
    {2, 32, 33, 15},
    {4, 5, 18, 19, 21, 22, 23, 25},
    0
  };
  clearAll();

  // Launch display on Core 0
  xTaskCreatePinnedToCore(
    displayTask,   // function
    "display",     // name
    2048,          // stack size (bytes)
    NULL,          // parameter
    1,             // priority
    NULL,          // task handle (not needed)
    0              // Core 0
  );
}

void loop() {
  // ---- ESP-NOW heartbeat (non-blocking guard; sends the display value) ----
  unsigned long now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendMessage(displayValue, displayValue * 1.5f, BOARD_A ? "from A" : "from B");
    Serial.printf("SEND value=%d\n", displayValue);
  }

  // ---- Stepper Logic ----
  // Continuously rotate 90 degrees and back UNTIL we hit 10 paninis
  if (displayValue < 3) {
    // Stepper runs here on Core 1, free to block.
    Serial.println("clockwise");
    myStepper.step(-quarterTurn);
    delay(500);
    
    Serial.println("counterclockwise");
    myStepper.step(quarterTurn);
    delay(500);
  } 
  else {
    // Optional: Add a small delay so an idle loop doesn't hammer the CPU
    delay(10); 
  }
}