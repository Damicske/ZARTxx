/*
  ZART03_Receiver_C6.ino  -  low-cost DIY receiver for a Schneider Harmony Pocket Remote (ZART03)
  ---------------------------------------------------------------------------------------------
  Radio    : the ZART uses a TI CC2520 (IEEE 802.15.4, 2.4 GHz, O-QPSK). An ESP32-C6 / ESP32-H2
             has a native 802.15.4 radio, so no extra RF module is needed.
  Hardware : ESP32-C6 Super Mini (or XIAO ESP32C6 / DevKitC-1 / ESP32-H2) + relay module
  Core     : Arduino-ESP32 3.x / 4.x
             Tools > USB CDC On Boot : Enabled   (needed for the serial monitor on the USB port)
             Tools > Zigbee mode     : Disabled  (this sketch drives the radio directly)

  Decoded frame format (15 bytes, sniffed from a ZART03 on channel 15):

      idx:  0  1   2  3  4  5  6  7  8  9 10  11   12 13 14
            FE SQ  00 0F 00 0D A0 DD 00 00 00 BB   07 7E 04
               |                              |
               |                              +-- button bitmask: 01 = B1, 02 = B2, 04 = B3,
               |                                  03 = B1+B2 (pairing), 00 = all released
               +-- sequence counter, restarts at 01 on every press, wraps FF -> 00
      All other bytes are constant per remote and are stored as its "signature" (ID).
      While a button is held the remote repeats the frame; after release it sends a few 00 frames.

  Works like the original ZARB receiver:
    - Pairing : start pair mode (BOOT 2..5 s, or 'P'), then hold B1 + B2 on the remote.
                Up to 8 remotes can be paired.
    - Run     : button N -> relay N (momentary). Several buttons at once work.
                Relays drop on the 00 release frame, or HOLD_MS after the last frame (out of range).
                One remote at a time; another remote takes over after OWNER_MS of silence.
    - Erase   : BOOT >= 6 s, or 'E'.

  Serial (115200) commands:
    h      = channel survey (hold a remote button during the ~7 s survey)
    + / -  = channel up / down manually
    s      = sniff on/off (frames with RSSI >= SNIFF_RSSI), a = sniff ALL frames
             (diagnostics only: off by default, switched off by pair mode; without sniff the
              receiver prints one line per button change, e.g. "remote 1: B1 pressed (rssi -62)")
    P      = pair mode (then hold B1 + B2 on the remote)
    E      = erase all paired remotes
    p      = print configuration
    t      = toggle mode on/off (each press toggles its relay instead of momentary)

  NOT a safety device: no stop category, no rolling code (a recorded frame can be replayed).
  Do not use it for hoists, cranes, gates near people or anything that can hurt someone.
*/

#include <Arduino.h>
#include <Preferences.h>
#include "esp_ieee802154.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ---------------------------------------------------------------- configuration
static const uint8_t  NUM_RELAYS       = 3;                       // ZART03 = 3 buttons (max 8)
static const uint8_t  RELAY_PIN[8]     = {0, 1, 2, 3, 4, 5, 6, 7}; // only the first NUM_RELAYS are used
static const bool     RELAY_ACTIVE_LOW = true;    // most cheap relay modules are active-low
static const uint8_t  LED_PIN          = 15;      // on-board LED (Super Mini / XIAO: GPIO15)
static const bool     LED_ACTIVE_LOW   = true;    // flip if the LED behaves inverted
static const uint8_t  LEARN_BTN_PIN    = 9;       // BOOT button on C6 and H2 boards

// Seeed XIAO ESP32C6 only: on-board RF switch (GPIO3 LOW = powered, GPIO14 LOW = built-in antenna)
static const bool     XIAO_RF_SWITCH   = false;   // false for ESP32-C6 Super Mini
static const bool     XIAO_EXT_ANTENNA = false;

static const uint32_t HOLD_MS          = 300;     // relays off this long after the last frame
static const uint32_t OWNER_MS         = 1000;    // silence before another remote may take over
static const uint32_t PAIR_TIME_MS     = 10000;   // pair window
static const int      PAIR_RSSI        = -75;     // remote must be close while pairing
static const int      SNIFF_RSSI       = -70;
static const int      SURVEY_RSSI      = -80;
static const uint32_t SURVEY_DWELL_MS  = 400;

// ---------------------------------------------------------------- protocol
#define MAX_PSDU     127
#define FRAME_LEN    15
#define FRAME_MAGIC  0xFE
#define IDX_SEQ      1
#define IDX_BTN      11
#define PAIR_MASK    0x03          // B1 + B2
#define MAX_REMOTES  8

struct Pkt {
  int8_t  rssi;
  uint8_t lqi;
  uint8_t chan;
  uint8_t len;                 // PSDU length without the 2 FCS bytes
  uint8_t data[MAX_PSDU];
};

struct Remote {
  uint8_t valid;
  uint8_t sig[FRAME_LEN];      // full frame; IDX_SEQ and IDX_BTN are ignored when comparing
};

// ---------------------------------------------------------------- globals
static Remote        remotes[MAX_REMOTES];
static uint8_t       radioChan = 0;            // 0 = unknown, else 11..26
static Preferences   prefs;
static QueueHandle_t rxQueue;

static bool     sniff = false, sniffAll = false, toggleMode = false;

// run state
static int      activeRemote = -1;
static uint32_t lastFrameMs  = 0;
static int8_t   lastRssi     = 0;
static uint8_t  buttons      = 0;              // bitmask from the last valid frame
static uint8_t  prevEff      = 0;
static uint8_t  latched      = 0;
static bool     suppress     = false;          // ignore buttons until release (right after pairing)

// pairing
static bool     pairing   = false;
static uint32_t pairStart = 0;

// survey
static bool     surveying = false;
static uint8_t  surveyChan = 11;
static uint32_t surveyStart = 0;
static uint16_t surveyCount[27];
static int8_t   surveyBestRssi[27];

// boot button
static uint32_t btnDownAt = 0;

// ---------------------------------------------------------------- radio
static void setChannel(uint8_t c) {
  esp_ieee802154_set_channel(c);
  esp_ieee802154_receive();
}

// Called by the 802.15.4 driver in ISR context for every received frame.
// frame[0] = PHR (length incl. 2-byte FCS), frame[1..] = PSDU.
extern "C" void IRAM_ATTR esp_ieee802154_receive_done(uint8_t* frame, esp_ieee802154_frame_info_t* info) {
  Pkt p;
  uint8_t phr = frame[0] & 0x7F;
  p.len  = (phr >= 2) ? (uint8_t)(phr - 2) : 0;
  p.rssi = info->rssi;
  p.lqi  = info->lqi;
  p.chan = info->channel;
  memcpy(p.data, &frame[1], p.len);
  esp_ieee802154_receive_handle_done(frame);   // give the buffer back to the driver

  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(rxQueue, &p, &woken);      // dropped if full
  if (woken) portYIELD_FROM_ISR();
}

// ---------------------------------------------------------------- helpers
static void printHex(const uint8_t* d, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) Serial.printf("%02X ", d[i]);
}

static void setRelay(uint8_t i, bool on) {
  digitalWrite(RELAY_PIN[i], (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

static void setLed(bool on) {
  digitalWrite(LED_PIN, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW);
}

static void saveConfig() {
  prefs.begin("zart", false);
  prefs.putBytes("rem", remotes, sizeof(remotes));
  prefs.putUChar("chan", radioChan);
  prefs.putBool("toggle", toggleMode);
  prefs.end();
}

static void loadConfig() {
  prefs.begin("zart", true);
  if (prefs.getBytes("rem", remotes, sizeof(remotes)) != sizeof(remotes)) memset(remotes, 0, sizeof(remotes));
  radioChan  = prefs.getUChar("chan", 0);
  toggleMode = prefs.getBool("toggle", false);
  prefs.end();
  if (radioChan < 11 || radioChan > 26) radioChan = 0;
}

static bool isZartFrame(const Pkt& p) {
  return p.len == FRAME_LEN && p.data[0] == FRAME_MAGIC;
}

static bool sigMatch(const uint8_t* a, const uint8_t* b) {
  for (uint8_t i = 0; i < FRAME_LEN; i++) {
    if (i == IDX_SEQ || i == IDX_BTN) continue;
    if (a[i] != b[i]) return false;
  }
  return true;
}

static int findRemote(const Pkt& p) {
  for (int r = 0; r < MAX_REMOTES; r++)
    if (remotes[r].valid && sigMatch(remotes[r].sig, p.data)) return r;
  return -1;
}

static void printSig(const uint8_t* s) {
  for (uint8_t i = 0; i < FRAME_LEN; i++) {
    if (i == IDX_SEQ || i == IDX_BTN) Serial.print("-- ");
    else Serial.printf("%02X ", s[i]);
  }
}

static void printConfig() {
  if (radioChan) Serial.printf("Radio channel: %u (%u MHz)\n", radioChan, 2405 + 5 * (radioChan - 11));
  else           Serial.println("Radio channel: unknown - run survey ('h')");
  Serial.printf("Relays: %u   Toggle mode: %s\n", NUM_RELAYS, toggleMode ? "ON" : "off (momentary)");
  int n = 0;
  for (int r = 0; r < MAX_REMOTES; r++) {
    if (!remotes[r].valid) continue;
    Serial.printf("Remote %d: ", r + 1);
    printSig(remotes[r].sig);
    Serial.println();
    n++;
  }
  if (!n) Serial.println("No remotes paired - start pair mode ('P' or BOOT 2-5 s), then hold B1 + B2.");
}

// ---------------------------------------------------------------- pairing / erase
static void startPair() {
  if (!radioChan) { Serial.println("Channel unknown - run the survey ('h') first."); return; }
  if (sniff) { sniff = false; sniffAll = false; Serial.println("sniff off"); }
  pairing = true;
  pairStart = millis();
  Serial.printf("\n>>> PAIR MODE: hold buttons 1 + 2 on the remote (%lu s)\n", (unsigned long)(PAIR_TIME_MS / 1000));
}

static void handlePairPkt(const Pkt& p) {
  if (!isZartFrame(p) || p.rssi < PAIR_RSSI) return;
  if ((p.data[IDX_BTN] & PAIR_MASK) != PAIR_MASK) return;

  pairing = false;
  suppress = true;                       // don't fire relays 1+2 until the buttons are released

  int r = findRemote(p);
  if (r >= 0) { Serial.printf(">>> remote already paired as remote %d\n", r + 1); return; }

  for (r = 0; r < MAX_REMOTES; r++) {
    if (remotes[r].valid) continue;
    remotes[r].valid = 1;
    memcpy(remotes[r].sig, p.data, FRAME_LEN);
    saveConfig();
    Serial.printf(">>> paired as remote %d: ", r + 1);
    printSig(remotes[r].sig);
    Serial.println();
    return;
  }
  Serial.println(">>> all 8 remote slots are in use - erase first ('E')");
}

static void eraseAll() {
  memset(remotes, 0, sizeof(remotes));
  activeRemote = -1;
  buttons = 0;
  saveConfig();
  Serial.println(">>> all remotes erased");
}

// ---------------------------------------------------------------- channel survey
static void startSurvey() {
  surveying = true;
  surveyChan = 11;
  memset(surveyCount, 0, sizeof(surveyCount));
  for (int i = 0; i < 27; i++) surveyBestRssi[i] = -128;
  Serial.println("\n>>> CHANNEL SURVEY: keep a remote button pressed (remote close to receiver)...");
  setChannel(surveyChan);
  surveyStart = millis();
}

static void surveyStep() {
  if (millis() - surveyStart < SURVEY_DWELL_MS) return;
  if (surveyChan < 26) {
    surveyChan++;
    setChannel(surveyChan);
    surveyStart = millis();
    return;
  }
  surveying = false;
  uint8_t best = 0;
  for (uint8_t c = 11; c <= 26; c++) {
    Serial.printf("  ch %2u (%u MHz): %3u frames  best rssi %4d\n",
                  c, 2405 + 5 * (c - 11), surveyCount[c], surveyBestRssi[c]);
    if (surveyCount[c] > 0 && (best == 0 || surveyCount[c] > surveyCount[best])) best = c;
  }
  if (best) {
    radioChan = best;
    saveConfig();
    Serial.printf(">>> remote found on channel %u\n", best);
  } else {
    Serial.println(">>> no ZART frames seen. Hold the button for the whole survey and retry.");
  }
  setChannel(radioChan ? radioChan : 11);
}

// ---------------------------------------------------------------- serial / button
static void handleSerial() {
  while (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'h': startSurvey(); break;
      case '+': radioChan = (radioChan >= 11 && radioChan < 26) ? radioChan + 1 : 11; setChannel(radioChan); saveConfig();
                Serial.printf("channel %u\n", radioChan); break;
      case '-': radioChan = (radioChan > 11) ? radioChan - 1 : 26; setChannel(radioChan); saveConfig();
                Serial.printf("channel %u\n", radioChan); break;
      case 's': sniff = !sniff; sniffAll = false; Serial.printf("sniff %s (rssi >= %d)\n", sniff ? "ON" : "off", SNIFF_RSSI); break;
      case 'a': sniffAll = !sniffAll; sniff = sniffAll; Serial.printf("sniff ALL %s\n", sniffAll ? "ON" : "off"); break;
      case 'P': startPair(); break;
      case 'E': case 'e': eraseAll(); break;
      case 'p': printConfig(); break;
      case 't': toggleMode = !toggleMode; latched = 0; saveConfig();
                Serial.printf("toggle mode %s\n", toggleMode ? "ON" : "off"); break;
      default: break;
    }
  }
}

// BOOT: hold 2..5 s and release -> pair mode, hold >= 6 s and release -> erase all
static void handleBootButton() {
  bool down = digitalRead(LEARN_BTN_PIN) == LOW;
  if (down) {
    if (btnDownAt == 0) btnDownAt = millis();
  } else if (btnDownAt != 0) {
    uint32_t held = millis() - btnDownAt;
    btnDownAt = 0;
    if (held >= 6000) eraseAll();
    else if (held >= 2000) startPair();
  }
}

// ---------------------------------------------------------------- frame handling
static void handleRunPkt(const Pkt& p) {
  if (!isZartFrame(p)) return;
  int r = findRemote(p);
  if (r < 0) return;

  uint32_t now = millis();
  if (activeRemote >= 0 && r != activeRemote && now - lastFrameMs < OWNER_MS) return;   // other remote owns the link

  activeRemote = r;
  lastFrameMs  = now;
  lastRssi     = p.rssi;
  buttons      = p.data[IDX_BTN];
  if (suppress && buttons == 0) suppress = false;
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  delay(500);
  for (int i = 0; i < NUM_RELAYS; i++) { pinMode(RELAY_PIN[i], OUTPUT); setRelay(i, false); }
  pinMode(LED_PIN, OUTPUT);
  setLed(false);
  pinMode(LEARN_BTN_PIN, INPUT_PULLUP);

  if (XIAO_RF_SWITCH) {
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);                                  // power the RF switch
    delay(100);
    pinMode(14, OUTPUT);
    digitalWrite(14, XIAO_EXT_ANTENNA ? HIGH : LOW);       // antenna select
  }

  rxQueue = xQueueCreate(16, sizeof(Pkt));
  loadConfig();

  esp_ieee802154_enable();
  esp_ieee802154_set_promiscuous(true);     // accept every frame, any PAN / address
  esp_ieee802154_set_rx_when_idle(true);
  setChannel(radioChan ? radioChan : 11);

  Serial.println("\nZART03 DIY receiver (802.15.4) ready. Commands: h + - s a P E p t");
  printConfig();
}

void loop() {
  handleSerial();
  handleBootButton();

  Pkt p;
  while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) {
    if (surveying) {
      if (isZartFrame(p) && p.rssi >= SURVEY_RSSI && p.chan >= 11 && p.chan <= 26) {
        surveyCount[p.chan]++;
        if (p.rssi > surveyBestRssi[p.chan]) surveyBestRssi[p.chan] = p.rssi;
      }
      continue;
    }
    if (sniff && !pairing && (sniffAll || p.rssi >= SNIFF_RSSI)) {
      Serial.printf("ch %2u  rssi %4d  lqi %3u  len %3u  ", p.chan, p.rssi, p.lqi, p.len);
      printHex(p.data, p.len);
      if (isZartFrame(p)) {
        int r = findRemote(p);
        Serial.printf(" seq %02X btn %02X ", p.data[IDX_SEQ], p.data[IDX_BTN]);
        if (r >= 0) Serial.printf("remote %d", r + 1); else Serial.print("(not paired)");
      }
      Serial.println();
    }
    if (pairing) { handlePairPkt(p); continue; }
    handleRunPkt(p);
  }

  uint32_t now = millis();
  if (surveying) surveyStep();
  if (pairing && now - pairStart > PAIR_TIME_MS) { pairing = false; Serial.println(">>> pair mode timed out"); }

  // link supervision
  if (now - lastFrameMs > HOLD_MS) { buttons = 0; suppress = false; }
  if (now - lastFrameMs > OWNER_MS) activeRemote = -1;

  // relays
  uint8_t eff = (surveying || pairing || suppress) ? 0 : buttons;

  // one compact line per button change (only when sniff is off)
  static uint8_t shownEff = 0;
  static int     shownRemote = -1;
  if (eff != shownEff) {
    if (!sniff) {
      if (eff) {
        Serial.printf("remote %d:", activeRemote + 1);
        for (int i = 0; i < 8; i++) if (eff & (1 << i)) Serial.printf(" B%d", i + 1);
        Serial.printf(" pressed (rssi %d)\n", lastRssi);
        shownRemote = activeRemote;
      } else {
        Serial.printf("remote %d: released%s\n", shownRemote + 1,
                      (now - lastFrameMs > HOLD_MS) ? " (signal lost)" : "");
      }
    }
    shownEff = eff;
  }
  if (toggleMode) latched ^= (eff & ~prevEff);             // rising edges toggle
  prevEff = eff;
  uint8_t out = toggleMode ? latched : eff;
  for (int i = 0; i < NUM_RELAYS; i++) setRelay(i, out & (1 << i));

  // LED
  if (btnDownAt && now - btnDownAt >= 6000)      setLed((now / 60) & 1);    // release now = erase
  else if (btnDownAt && now - btnDownAt >= 2000) setLed(true);              // release now = pair
  else if (surveying || pairing)                 setLed((now / 150) & 1);
  else                                           setLed(out != 0);
}
