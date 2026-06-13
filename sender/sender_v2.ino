/*
  RFID + MP3 + dual-motor + ultrasonic robot  +  bidirectional ESP-NOW
  --------------------------------------------------------------------
  Robot behaviour (unchanged):
    - Motors        : drive forward continuously by default.
    - Default audio : loops panini.mp3 forever.
    - On obstacle   : when the HC-SR04 sees something within
                      DISTANCE_THRESHOLD_CM, the robot plays
                      player_hurt.mp3 and rotates in place for a
                      random window, then resumes forward + panini.
    - On RFID scan  : (audio/RFID logic preserved as-is)

  NEW: scan counting + finale
    - Every RFID scan increments rfidScanCount.
    - Scans 1-3 behave as before (play hurt, return to panini).
    - Once the count goes ABOVE three (the 4th scan), the robot stops
      driving and plays ending.mp3 all the way through. When that clip
      finishes it enters a DONE state: motors stopped, audio off, and
      the heartbeat paused — it does nothing further.

  ESP-NOW (new):
    - Both boards run THIS SAME sketch; set BOARD_A true on one,
      false on the other, and fill in both MAC addresses below.
    - Sends a heartbeat every SEND_INTERVAL_MS (non-blocking).
    - Sends an event message on obstacle and on RFID scan.
    - Receives messages via onReceive() and prints them. Add your
      own reaction where marked "<-- hook".

  IMPORTANT: there is no delay() in loop() — the MP3 decoder must be
  fed every iteration, so all timing is done with millis().
  --------------------------------------------------------------------
  Pin map (no conflicts; WiFi uses no GPIOs):
    Audio I2S  : BCLK 15, LRC 2, DOUT 27
    RFID  SPI  : SCK 18, MISO 19, MOSI 23, CS 5
    Servos     : servo_1 -> 26, servo_2 -> 25
    Ultrasonic : TRIG 33, ECHO 32  (ECHO via 5V->3.3V divider)
*/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <LittleFS.h>
#include "AudioFileSourceLittleFS.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <ESP32Servo.h>

// ============================================================
//  Configuration
// ============================================================
// ---- I2S pins (audio) ----
#define I2S_BCLK 15
#define I2S_LRC   2
#define I2S_DOUT 27
// ---- RFID chip-select pin (SPI: SCK18 MISO19 MOSI23 CS5) ----
#define RFID_SS_PIN 5
// ---- Servo pins ----
#define SERVO_1_PIN 26
#define SERVO_2_PIN 25
// ---- Servo pulse widths (microseconds) ----
#define FULL_CCW  1700
#define FULL_STOP 1500
#define FULL_CW   1300
// ---- Ultrasonic pins ----
#define TRIG_PIN 33
#define ECHO_PIN 32
// ---- Audio file paths (LittleFS) ----
const char *PANINI_MP3      = "/panini.mp3";
const char *HURT_MP3        = "/delicious.mp3";
const char *PLAYER_HURT_MP3 = "/classic_hurt.mp3";   // played on obstacle
const char *ENDING_MP3      = "/ending.mp3";        // played after >3 scans
// ---- Output gain (0.0 - 1.0+) ----
const float MAX_AUDIO_GAIN = 1.0f;
float audio_gain = MAX_AUDIO_GAIN;
// ---- Ultrasonic tuning ----
const float SOUND_CM_PER_US = 0.0343f;
// Short timeout on purpose: we only care about NEAR obstacles, and a
// long pulseIn() wait would stall the MP3 decoder and glitch audio.
const unsigned long ECHO_TIMEOUT_US = 6000UL;
// Trigger the reaction when something is closer than this.
const float DISTANCE_THRESHOLD_CM = 10.0f;
// How often to actually ping the sensor (keeps audio smooth).
const unsigned long DISTANCE_READ_INTERVAL_MS = 100UL;
// ---- Rotation reaction timing ----
const unsigned long ROTATE_MIN_MS = 500UL;
const unsigned long ROTATE_MAX_MS = 2000UL;
// ---- Scan limit ----
// More than this many RFID scans triggers the finale.
const int MAX_RFID_SCANS = 2;

// ============================================================
//  ESP-NOW configuration
// ============================================================
#define BOARD_A true   // true on one board, false on the other

// Each board's address. Get these from the MAC-finder sketch
// (esp_read_mac with ESP_MAC_WIFI_STA) and paste them in.
uint8_t macOfA[] = {0xE0, 0x8C, 0xFE, 0x57, 0xDE, 0x58};  // <-- Board A's MAC
uint8_t macOfB[] = {0xF4, 0x2D, 0xC9, 0x76, 0xED, 0x74};  // <-- Board B's MAC
uint8_t *peerMAC = BOARD_A ? macOfB : macOfA;             // talk to the other one

typedef struct {
  int   id;        // 0 = heartbeat, 1 = rfid scan, 2 = obstacle (your convention)
  float value;
  char  text[16];
} Message;

Message outgoing;
Message incoming;

// How often to send the heartbeat (non-blocking; replaces delay(2000)).
const unsigned long SEND_INTERVAL_MS = 2000UL;

// ============================================================
//  Runtime timers / state
// ============================================================
unsigned long lastDistanceReadMs = 0;
unsigned long rotateUntilMs      = 0;  // millis() deadline to stop rotating
unsigned long lastSendMs         = 0;  // millis() deadline for next heartbeat
int           heartbeatId        = 0;  // increments each heartbeat
int           rfidScanCount      = 0;  // how many cards we've scanned so far

// ============================================================
//  Motor module — two continuous-rotation servos
// ============================================================
Servo servo_1;
Servo servo_2;

void drive_forward() {
  servo_1.writeMicroseconds(FULL_CW);
  servo_2.writeMicroseconds(FULL_CCW);
}
void drive_backward() {
  servo_1.writeMicroseconds(FULL_CCW);
  servo_2.writeMicroseconds(FULL_CW);
}
// Both wheels same direction -> spin in place.
void rotate() {
  servo_1.writeMicroseconds(FULL_CW);
  servo_2.writeMicroseconds(FULL_CW);
}
void drive_stop() {
  servo_1.writeMicroseconds(FULL_STOP);
  servo_2.writeMicroseconds(FULL_STOP);
}
void motorInit() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo_1.setPeriodHertz(50);
  servo_1.attach(SERVO_1_PIN, FULL_CW, FULL_CCW);
  servo_1.writeMicroseconds(FULL_STOP);
  servo_2.setPeriodHertz(50);
  servo_2.attach(SERVO_2_PIN, FULL_CW, FULL_CCW);
  servo_2.writeMicroseconds(FULL_STOP);
}

// ============================================================
//  Ultrasonic module
// ============================================================
void ultrasonicInit() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}
// Returns distance in cm, or -1.0 if no echo (out of range / timeout).
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;  // timed out, nothing detected
  return (duration * SOUND_CM_PER_US) / 2.0f;
}

// ============================================================
//  Playback state machine
// ============================================================
enum class Track {
  PANINI,    // default: loop panini.mp3 forever
  HURT,      // play classic_hurt.mp3 once, then return to PANINI
  OBSTACLE,  // ultrasonic reaction: rotate + play player_hurt.mp3
  ENDING,    // >3 scans: play ending.mp3 once, all the way through
  DONE       // ending finished: motors off, audio off, do nothing
};
Track currentTrack = Track::PANINI;

// ============================================================
//  Audio module — one reusable MP3 player
// ============================================================
AudioGeneratorMP3       *mp3  = nullptr;
AudioFileSourceLittleFS *file = nullptr;
AudioOutputI2S          *out  = nullptr;

void setAudioGain(float gain) {
  float gainVal = gain;
  if (gainVal > MAX_AUDIO_GAIN) gainVal = MAX_AUDIO_GAIN;
  out->SetGain(gainVal);
}
void audioInit() {
  out = new AudioOutputI2S();
  out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  setAudioGain(MAX_AUDIO_GAIN);
}
void audioStop() {
  if (mp3)  { mp3->stop(); delete mp3;  mp3  = nullptr; }
  if (file) { delete file;              file = nullptr; }
}
void playTrack(const char *path) {
  audioStop();
  file = new AudioFileSourceLittleFS(path);
  mp3  = new AudioGeneratorMP3();
  mp3->begin(file, out);
  Serial.printf("Playing %s\n", path);
}
bool audioLoop() {
  if (!mp3 || !mp3->isRunning()) return false;
  if (!mp3->loop()) {
    audioStop();
    return false;
  }
  return true;
}

// ============================================================
//  RFID module
// ============================================================
MFRC522DriverPinSimple ss_pin(RFID_SS_PIN);
MFRC522DriverSPI driver{ss_pin};
MFRC522 mfrc522{driver};

void rfidInit() {
  mfrc522.PCD_Init();
}
bool rfidScanned() {
  if (!mfrc522.PICC_IsNewCardPresent()) return false;
  if (!mfrc522.PICC_ReadCardSerial())   return false;
  mfrc522.PICC_HaltA();
  return true;
}

// ============================================================
//  ESP-NOW module
// ============================================================
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "  [sent ok]" : "  [send FAIL]");
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));
  Serial.printf("RECV from %02X:%02X:%02X:%02X:%02X:%02X -> id=%d value=%.1f text=%s\n",
                info->src_addr[0], info->src_addr[1], info->src_addr[2],
                info->src_addr[3], info->src_addr[4], info->src_addr[5],
                incoming.id, incoming.value, incoming.text);

  // <-- hook: react to a received message here, e.g.
  // if (incoming.id == 2) { /* the other robot hit an obstacle */ }
  // Keep it short: this runs in the WiFi callback context, so don't
  // do blocking work (no delay, no long audio calls) in here.
}

// Fill the outgoing struct and fire it off (non-blocking).
void sendMessage(int id, float value, const char *text) {
  outgoing.id    = id;
  outgoing.value = value;
  strncpy(outgoing.text, text, sizeof(outgoing.text) - 1);
  outgoing.text[sizeof(outgoing.text) - 1] = '\0';
  Serial.printf("SEND -> id=%d value=%.1f text=%s\n", outgoing.id, outgoing.value, outgoing.text);
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
  peer.channel = 0;       // use current WiFi channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
  }
}

// ============================================================
//  Helpers
// ============================================================
// Begin the obstacle reaction: spin in place + play player_hurt,
// for a random window. Non-blocking (uses a millis deadline).
void startObstacleReaction() {
  unsigned long spinMs = random(ROTATE_MIN_MS, ROTATE_MAX_MS + 1);
  rotateUntilMs = millis() + spinMs;
  Serial.printf("Obstacle! Rotating for %lu ms.\n", spinMs);
  rotate();
  // Make sure we're audible (a prior RFID scan may have muted us).
  audio_gain = MAX_AUDIO_GAIN;
  setAudioGain(audio_gain);
  currentTrack = Track::OBSTACLE;
  playTrack(PLAYER_HURT_MP3);

  // Tell the other board we hit something.
  sendMessage(2, (float)spinMs, "obstacle");
}

// Begin the finale: stop driving, make sure we're audible, and play
// ending.mp3 once. When it finishes (handled in loop) we go to DONE.
void startEnding() {
  Serial.println("More than 3 scans — playing ending and shutting down.");
  drive_stop();
  audio_gain = MAX_AUDIO_GAIN;
  setAudioGain(audio_gain);
  currentTrack = Track::ENDING;
  playTrack(ENDING_MP3);
}

// ============================================================
//  Main
// ============================================================
void setup() {
  Serial.begin(115200);

  // Bring up the radio first so it's ready before audio starts.
  espnowInit();

  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed!");
    return;
  }
  randomSeed(esp_random());  // seed RNG so rotation time varies

  motorInit();
  audioInit();
  rfidInit();
  ultrasonicInit();

  drive_forward();
  Serial.println("Motors forward.");
  Serial.println("Ready — looping panini. Scan a card for hurt.");

  currentTrack = Track::PANINI;
  playTrack(PANINI_MP3);
}

void loop() {
  unsigned long now = millis();

  // ---- ESP-NOW heartbeat (non-blocking; replaces the old delay(2000)) ----
  // Paused once we reach DONE so the robot truly does nothing further.
  if (currentTrack != Track::DONE && now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendMessage(0, heartbeatId * 1.5f, BOARD_A ? "hi from A" : "hi from B");
    Serial.printf("SEND heartbeat id=%d\n", heartbeatId);
    heartbeatId++;
  }

  switch (currentTrack) {
    case Track::PANINI: {
      // A scan interrupts panini and triggers the hurt clip.
      if (rfidScanned()) {
        rfidScanCount++;
        Serial.printf("RFID scan #%d\n", rfidScanCount);

        // Tell the other board a card was scanned (carry the count).
        sendMessage(1, (float)rfidScanCount, "rfid");

        if (rfidScanCount > MAX_RFID_SCANS) {
          // 4th scan onward: play the finale, then halt for good.
          startEnding();
        } else {
          currentTrack = Track::HURT;
          playTrack(HURT_MP3);
        }
        break;
      }

      // Throttled obstacle check (single, short-timeout ping so it
      // doesn't stall the audio decoder).
      if (now - lastDistanceReadMs >= DISTANCE_READ_INTERVAL_MS) {
        lastDistanceReadMs = now;
        float distance = readDistanceCm();
        if (distance > 0 && distance < DISTANCE_THRESHOLD_CM) {
          startObstacleReaction();
          break;
        }
      }

      // No scan / no obstacle: keep panini looping seamlessly.
      if (!audioLoop()) {
        audio_gain += 0.1f; // gets louder
        setAudioGain(audio_gain);
        playTrack(PANINI_MP3);  // restart when the file ends
      }
      break;
    }

    case Track::OBSTACLE:
      // Keep player_hurt sounding for the whole rotation window;
      // if it ends early, replay it until the spin timer expires.
      if (!audioLoop()) {
        playTrack(PLAYER_HURT_MP3);
      }
      // Rotation finished -> resume forward + panini.
      if (millis() >= rotateUntilMs) {
        Serial.println("Rotation done — forward + panini.");
        drive_forward();
        currentTrack = Track::PANINI;
        playTrack(PANINI_MP3);
      }
      break;

    case Track::HURT:
      // Scans are ignored here so hurt can't retrigger itself.
      if (!audioLoop()) {
        Serial.println("Hurt done — resuming panini.");
        currentTrack = Track::PANINI;
        playTrack(PANINI_MP3);
      }
      break;

    case Track::ENDING:
      // Play ending.mp3 once, all the way through. Unlike the other
      // states we do NOT replay or loop — when it finishes, we stop.
      if (!audioLoop()) {
        Serial.println("Ending finished — halting. Goodbye.");
        drive_stop();
        audioStop();
        currentTrack = Track::DONE;
      }
      break;

    case Track::DONE:
      // Nothing left to do: motors stopped, audio off, heartbeat paused.
      // (The MP3 decoder isn't being fed because there's nothing playing.)
      break;
  }
}