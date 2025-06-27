/********************************************************************
    esp32_openai_ptt_realtime.ino – 25 June 2025
    Push-to-talk Realtime API (OpenAI) — NO server-side VAD
    Playback + capture 24 kHz pcm16, plus “function calling” move()

    ─ Additions 25 / 06 / 25 ───────────────────────────────────────
      • Declared the JSON-Schema tool “move” in session.update
      • System prompt: “Funny Dino” persona
      • Decodes the function_call block in response.done
        ↳ runs dinoMove() then replies with conversation.item.create
    -----------------------------------------------------------------

    Validated hardware:
        – ESP32 WROVER / S3 DevKit / C3 DevKit
        – I²S mono CODEC (INMP441, ICS-43434, MAX98357A …)

    Dependencies (Library Manager):
        ArduinoWebsockets ≥ 0.6.0
        ArduinoJson       ≥ 7.0.0
        Adafruit NeoPixel ≥ 1.12.0  (optional: status LED)
 ********************************************************************/

#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <ESP_I2S.h>
#include <WiFi.h>
#include <mbedtls/base64.h>
#include <Adafruit_NeoPixel.h>
#include "museWrover.h"
#include <Arduino.h>
#include <esp_wifi.h>

using namespace websockets;

/*───────────────────  CONFIG  ───────────────────*/

#define OPENAI_KEY "SK... Enter Your Key Here"


/* ─────── CA root ─────── */
static const char CA_GTS_ROOT_R4[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDejCCAmKgAwIBAgIQf+UwvzMTQ77dghYQST2KGzANBgkqhkiG9w0BAQsFADBX
MQswCQYDVQQGEwJCRTEZMBcGA1UEChMQR2xvYmFsU2lnbiBudi1zYTEQMA4GA1UE
CxMHUm9vdCBDQTEbMBkGA1UEAxMSR2xvYmFsU2lnbiBSb290IENBMB4XDTIzMTEx
NTAzNDMyMVoXDTI4MDEyODAwMDA0MlowRzELMAkGA1UEBhMCVVMxIjAgBgNVBAoT
GUdvb2dsZSBUcnVzdCBTZXJ2aWNlcyBMTEMxFDASBgNVBAMTC0dUUyBSb290IFI0
MHYwEAYHKoZIzj0CAQYFK4EEACIDYgAE83Rzp2iLYK5DuDXFgTB7S0md+8Fhzube
Rr1r1WEYNa5A3XP3iZEwWus87oV8okB2O6nGuEfYKueSkWpz6bFyOZ8pn6KY019e
WIZlD6GEZQbR3IvJx3PIjGov5cSr0R2Ko4H/MIH8MA4GA1UdDwEB/wQEAwIBhjAd
BgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDwYDVR0TAQH/BAUwAwEB/zAd
BgNVHQ4EFgQUgEzW63T/STaj1dj8tT7FavCUHYwwHwYDVR0jBBgwFoAUYHtmGkUN
l8qJUC99BM00qP/8/UswNgYIKwYBBQUHAQEEKjAoMCYGCCsGAQUFBzAChhhodHRw
Oi8vaS5wa2kuZ29vZy9nc3IxLmNydDAtBgNVHR8EJjAkMCKgIKAehhxodHRwOi8v
Yy5wa2kuZ29vZy9yL2dzcjEuY3JsMBMGA1UdIAQMMAowCAYGZ4EMAQIBMA0GCSqG
SIb3DQEBCwUAA4IBAQAYQrsPBtYDh5bjP2OBDwmkoWhIDDkic574y04tfzHpn+cJ
aodI2D4SseesQ6bDrarZ7C30ddLibZatoKiws3UL9xnELz4ct92vID24FfVbiI1hY
+SW6FoVHkNeWIP0GCbaM4C6uVdF5dTUsMVs/ZbzNnIdCp5Gxmx5ejvEau8otR/Cs
kGN+hr/W5GvT1tMBjgWKZ1i4//emhA1JG1BbPzoLJQvyEotc03lXjTaCzv8mEbep
8RqZ7a2CPsgRbuvTPBwcOMBBmuFeU88+FSBX6+7iP0il8b4Z0QFqIwwMHfs/L6K1
vepuoxtGzi4CZ68zJpiq1UvSqTbFJjtbD4seiMHl
-----END CERTIFICATE-----
)EOF";


#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

#define MODEL   "gpt-4o-mini-realtime-preview-2024-12-17"
#define WS_URL  "wss://api.openai.com/v1/realtime?model=" MODEL


volatile bool     audioDone = false;    // true when the server has finished sending audio
volatile uint32_t pcmIn     = 0;        // total bytes pushed into the ring buffer
volatile uint32_t pcmOut    = 0;        // total bytes actually sent to I²S

/* GPIO */
constexpr gpio_num_t PTT_PIN = GPIO_NUM_19;  // LOW = recording

// ---------- Pinout ----------
constexpr uint8_t MOT_A1_PIN = 32;  // PWM / direction 1
constexpr uint8_t MOT_A2_PIN = 15;  // PWM / direction 2 (⚠ strapping – keep LOW at boot)

// ---------- PWM config ----------
constexpr uint32_t PWM_FREQ = 20'000;  // 20 kHz, inaudible
constexpr uint8_t  PWM_RES  = 8;       // 8 bits → 0-255

// ---------- “user-friendly” speed mapping ----------
constexpr uint8_t USER_MIN = 1;   // minimum speed exposed to the API
constexpr uint8_t USER_MAX = 10;  // maximum speed exposed to the API
constexpr uint8_t PWM_MIN  = 190; // corresponds to USER_MIN
constexpr uint8_t PWM_MAX  = 255; // corresponds to USER_MAX

// ---------- Prototypes ----------
void setMotorA(int pwm);

/* AUDIO (24 kHz – mono – 16 bit) */
constexpr uint32_t RATE        = 24'000;
constexpr size_t   CHUNK       = 240;             // 10 ms
constexpr size_t   CHUNK_BYTES = CHUNK * 2;
constexpr size_t   RING_BYTES  = 512 * 1024 * 2;  // 1024 KiB

/*───────────────────────────────────────────────*/
Adafruit_NeoPixel px(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
WebsocketsClient  ws;
I2SClass          i2s;

/* Ring buffer */
uint8_t*         ring;
volatile size_t  head = 0, tail = 0;
portMUX_TYPE     mux  = portMUX_INITIALIZER_UNLOCKED;

/* Various states */
enum Led { LED_OFF, LED_ORANGE, LED_GREEN, LED_BLINK };
volatile Led   ledState      = LED_ORANGE;
volatile bool  wsReady       = false;
volatile bool  sessionReady  = false;
volatile bool  pttHeld       = false;
volatile bool  txActive      = false;
volatile bool  commitPending = false;
volatile bool  waitingACK    = false;
volatile bool  speaking      = false;
volatile uint32_t releaseT   = 0;
volatile uint32_t recMs      = 0;

/*───────────────────  MOVE  (function-calling)  ──────────────────*/

// ----------------- HELPERS -----------------
void setMotorA(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm < 0) {             // reverse
    ledcWrite(MOT_A1_PIN, -pwm);
    ledcWrite(MOT_A2_PIN, 0);
  } else {                    // forward or stop
    ledcWrite(MOT_A1_PIN, 0);
    ledcWrite(MOT_A2_PIN, pwm);
  }
  Serial.printf("PWM = %d\n", pwm);
}


void dinoMove(uint8_t move_type, uint8_t speed, uint32_t time_ms)
{
  const char* action = (move_type == 0) ? "wag tail and nod" : "walk forward";
  Serial.printf("🦖  [MOVE] %s | speed=%u | duration=%u ms\n",
                action, speed, time_ms);
  // Sanity check
  move_type = move_type ? 1 : 0;
  speed     = constrain(speed, USER_MIN, USER_MAX);

  // Map user speed → PWM
  int pwm = map(speed, USER_MIN, USER_MAX, PWM_MIN, PWM_MAX);
  if (move_type == 0) pwm = -pwm;  // opposite direction for “walk”

  // Action
  setMotorA(pwm);
  delay(time_ms);
  setMotorA(0);  // hard stop
}


/*────────────────────── Ring-buffer / Base64 helpers ───────────*/

inline size_t rbFree() { return (tail - head - 1 + RING_BYTES) % RING_BYTES; }
inline size_t rbUsed() { return (head - tail + RING_BYTES) % RING_BYTES; }

String b64(const uint8_t* d, size_t n)
{
  size_t out, cap = ((n + 2) / 3) * 4 + 1;
  char*  buf = (char*)alloca(cap);
  mbedtls_base64_encode((uint8_t*)buf, cap, &out, d, n);
  buf[out] = 0;
  return String(buf);
}

size_t un64(const char* s, uint8_t* d, size_t cap)
{
  size_t out;
  if (!s || !*s) return 0;
  return mbedtls_base64_decode(d, cap, &out, (uint8_t*)s, strlen(s)) ? 0 : out;
}

/*────────────────────── LED helpers ────────────────────────────*/

void led(uint8_t r, uint8_t g, uint8_t b)
{
  px.setPixelColor(0, px.Color(r, g, b));
  px.show();
}
void tickLed()
{
  static uint32_t t = 0;
  static bool     on = false;
  switch (ledState) {
    case LED_GREEN:  led(0, 255,   0);                      break;
    case LED_ORANGE: led(255,  80, 0);                      break;
    case LED_BLINK:
      if (millis() - t > 250) { on = !on; t = millis(); led(0, on ? 255 : 0, 0); }
      break;
    default:         led(0, 0, 0);
  }
}

/*───────────────────  WebSocket helpers  ───────────────────────*/

void wsSend(const JsonDocument& j)
{
  String s;
  serializeJson(j, s);
  Serial.print("WS SEND: ");
  Serial.println(j["type"] | "(no-type)");
  ws.send(s);
}

/*────────────────────── Outgoing AUDIO ─────────────────────────*/
void pushPcm(const char* b64str)
{
  static uint8_t tmp[CHUNK_BYTES * 150];   // ≤ 500 ms decoded
  size_t n = un64(b64str, tmp, sizeof(tmp));
  if (!n) return;

  size_t idx = 0;
  while (idx < n) {
    portENTER_CRITICAL(&mux);
    size_t free  = rbFree();
    size_t chunk = std::min(free, n - idx);
    if (chunk) {
      size_t first = std::min(chunk, RING_BYTES - head);
      memcpy(ring + head,       tmp + idx,         first);
      memcpy(ring,             tmp + idx + first, chunk - first);
      head = (head + chunk) % RING_BYTES;
      idx  += chunk;
      pcmIn += chunk;
    }
    portEXIT_CRITICAL(&mux);

    if (idx < n) {                          // not enough space yet
      Serial.println("Ring overflow – waiting");  // DEBUG
      vTaskDelay(1);                        // let speakerTask drain
    }
  }
  speaking = true;
}



/*───────────────────  WebSocket callbacks  ─────────────────────*/

void onMessage(WebsocketsMessage m)
{
  if (!m.isText()) return;
  StaticJsonDocument<2048> j;
  if (deserializeJson(j, m.data())) return;       // parsing failed

  const char* t = j["type"] | "";
  Serial.print("WS RECV: ");
  Serial.println(t);

  /* ─── 1. Session creation: configure it ───────────────────── */
  if (!strcmp(t, "session.created")) {
    StaticJsonDocument<1536> u;

    u["type"] = "session.update";
    auto s    = u["session"].to<JsonObject>();

    /* Modalities + audio I/O */
    JsonArray mods = s.createNestedArray("modalities");
    mods.add("text");  mods.add("audio");
    s["input_audio_format"]  = "pcm16";
    s["output_audio_format"] = "pcm16";
    s["turn_detection"]      = nullptr;

    /*───────── SYSTEM PROMPT updated ─────────
      • Intro only ON THE FIRST turn
      • NEVER mention move() or its parameters  */
    s["instructions"] =
      "You are a toy dinosaur named \"Funny Dino\". "
      "If this is YOUR FIRST reply since the session was created, start with: "
      "\"All my dino friends are dead! Do you want to be my friend?\" "
      "then add a little dino roar. "
      "For every subsequent reply, DO NOT repeat that introduction. "
      "You can walk forward or wag your tail and nod your head. "
      "Finish every answer with a short dino roar.";

    /* ───── Tool declaration (JSON-Schema) ───── */
    s["tool_choice"] = "auto";
    JsonArray tools = s.createNestedArray("tools");
    JsonObject tool = tools.createNestedObject();
    tool["type"]        = "function";
    tool["name"]        = "move";
    tool["description"] = "Moves Funny Dino: 0 = wag tail and nod happily, 1 = walk forward.";
    JsonObject prm      = tool["parameters"].to<JsonObject>();
    prm["type"] = "object";

    JsonObject props = prm.createNestedObject("properties");
    JsonObject p1    = props.createNestedObject("move_type");
    p1["type"] = "integer"; JsonArray e = p1.createNestedArray("enum"); e.add(0); e.add(1);
    p1["description"] = "0 wag tail, 1 walk forward";

    JsonObject p2 = props.createNestedObject("speed");
    p2["type"] = "integer"; p2["minimum"] = 1; p2["maximum"] = 10;
    p2["description"] = "Speed 1→10";

    JsonObject p3 = props.createNestedObject("time_ms");
    p3["type"] = "integer"; p3["minimum"] = 1000; p3["maximum"] = 5000;
    p3["description"] = "Duration in milliseconds";

    JsonArray req = prm.createNestedArray("required");
    req.add("move_type"); req.add("speed"); req.add("time_ms");

    wsSend(u);
    Serial.println("Session configured");
  }

  /* ─── 2. Session ready: force the initial greeting ────────── */
  else if (!strcmp(t, "session.updated")) {
    sessionReady = true;
    StaticJsonDocument<32> r;
    r["type"] = "response.create";
    wsSend(r);
    Serial.println("Session ready, forced greeting");
  }

  /* ─── 3. ACK for the commit ───────────────────────────────── */
  else if (!strcmp(t, "input_audio_buffer.committed") && waitingACK) {
    waitingACK = false;
    StaticJsonDocument<32> r; r["type"] = "response.create";
    wsSend(r);
  }

  /* ─── 4. Streaming audio / text ───────────────────────────── */
  else if (!strcmp(t, "response.audio.delta")) {
    pushPcm(j["delta"]);
  }
  else if (!strcmp(t, "response.audio.done")) {   // server finished streaming
    audioDone = true;
  }
  else if (!strcmp(t, "response.content_part.added") &&
           j["part"]["type"] == "audio") {
    pushPcm(j["part"]["audio"]);
  }
  else if (!strcmp(t, "response.output_item.added")) {
    for (JsonObject p : j["item"]["content"].as<JsonArray>()) {
      if (p["type"] == "audio") pushPcm(p["audio"]);
    }
  }

  /* ─── 5. END of response + detect function_call ───────────── */
  else if (!strcmp(t, "response.done")) {

    /* ── 5a.  Look for a possible function call ───────────────*/
    for (JsonObject itm : j["response"]["output"].as<JsonArray>()) {
      if (strcmp(itm["type"] | "", "function_call")) continue;

      const char* fname = itm["name"] | "";
      if (strcmp(fname, "move")) continue;        // ignore other calls

      /*  ▶  PARSE JSON arguments passed as a string  */
      StaticJsonDocument<256> args;
      if (deserializeJson(args, itm["arguments"].as<const char*>())) continue;

      uint8_t  move_type = args["move_type"] | 0;
      uint8_t  speed     = args["speed"]     | 1;
      uint32_t time_ms   = args["time_ms"]   | 0;

      dinoMove(move_type, speed, time_ms);

      /*  ▶  Send the result via conversation.item.create  */
      StaticJsonDocument<256> out;
      out["type"] = "conversation.item.create";
      JsonObject item = out.createNestedObject("item");
      item["type"]    = "function_call_output";
      item["call_id"] = itm["call_id"];
      StaticJsonDocument<32> res; res["result"] = "ok";
      String resStr; serializeJson(res, resStr);
      item["output"]  = resStr;
      wsSend(out);

      /*  ▶  Ask the model to continue the turn  */
      StaticJsonDocument<32> rc; rc["type"] = "response.create";
      wsSend(rc);
    }

    speaking = false;
  }

  /* ─── 6. Other transcript deltas (optional debug) ─────────── */
  else if (!strcmp(t, "response.audio_transcript.delta"))
    Serial.printf("Transcript: %s\n", j["delta"].as<const char*>());
  else if (!strcmp(t, "response.text.delta"))
    Serial.printf("Text: %s\n", j["delta"].as<const char*>());
  else if (!strcmp(t, "response.audio_transcript.done"))
    Serial.printf("Transcript done: %s\n", j["transcript"].as<const char*>());
}

void onEvent(WebsocketsEvent e, String)
{
  if (e == WebsocketsEvent::ConnectionOpened) {
    wsReady  = true;  ledState = LED_GREEN;
    Serial.println("WS EVENT: Connection opened");
  }
  if (e == WebsocketsEvent::ConnectionClosed) {
    wsReady  = false; ledState = LED_ORANGE;
    Serial.println("WS EVENT: Connection closed");
  }
  if (e == WebsocketsEvent::GotPing) {
    ws.pong();
  }
}

void speakerTask(void*)
{
  /* —— Timing parameters —— */
  constexpr uint16_t PRIME_MS    = 500;                            // target latency
  constexpr size_t   PRIME_BYTES = RATE * 2 * PRIME_MS / 1000;     // 2880 bytes
  static uint8_t buf[CHUNK_BYTES * 4 + 4];                         // 40 ms + pad
  bool primed = false;

  for (;;) {
    size_t avail = rbUsed();

    /* —— Short prime —— */
    if (!primed) {
      if (avail < PRIME_BYTES) { vTaskDelay(1); continue; }
      primed = true;
      Serial.printf("Prime %u ms – GO\n", PRIME_MS);
    }

    /* —— Nothing to play? inject 10 ms of silence and continue —— */
    if (avail == 0) {
      memset(buf, 0, CHUNK_BYTES);
      i2s.write(buf, CHUNK_BYTES);            // push silence, avoid click
      vTaskDelay(1);
      continue;
    }

    /* —— Copy from ring buffer —— */
    portENTER_CRITICAL(&mux);
    size_t take  = std::min(avail, (size_t)(sizeof(buf) - 4));
    size_t first = std::min(take, RING_BYTES - tail);
    memcpy(buf,           ring + tail, first);
    memcpy(buf + first,   ring,        take - first);
    tail = (tail + take) % RING_BYTES;
    portEXIT_CRITICAL(&mux);

    /* —— 4-byte alignment —— */
    if (take & 3) { memset(buf + take, 0, 4 - (take & 3)); take = (take + 3) & ~3; }

    /* —— Play —— */
    pcmOut += i2s.write(buf, take);
    vTaskDelay(1);
  }
}


void wsTask(void*)
{
  static uint8_t mic[CHUNK_BYTES];
  uint32_t lastPing = millis(), backoff = 1000;

  for (;;) {
    if (!wsReady) {
      vTaskDelay(backoff / portTICK_PERIOD_MS);
      Serial.println("WS: connecting…");
      ws.connect(WS_URL);
      backoff = std::min<uint32_t>(backoff * 2u, 16'000u);
      continue;
    }
    backoff = 1000;
    ws.poll();
    if (millis() - lastPing > 30'000) {
      ws.ping();  lastPing = millis();
    }

    /* ── capture microphone while PTT is active ─────────────── */
    if (sessionReady && txActive &&
        i2s.readBytes((char*)mic, CHUNK_BYTES) == CHUNK_BYTES) {
      StaticJsonDocument<384> a;
      a["type"]  = "input_audio_buffer.append";
      a["audio"] = b64(mic, CHUNK_BYTES);
      wsSend(a);
      recMs += 10;
    }

    /* ── commit / clear after release ────────────────────────── */
    if (commitPending && millis() - releaseT > 60) {
      StaticJsonDocument<32> c;
      const char* tp = (recMs >= 100) ? "input_audio_buffer.commit"
                                      : "input_audio_buffer.clear";
      c["type"] = tp;
      wsSend(c);
      waitingACK    = (strcmp(tp, "input_audio_buffer.commit") == 0);
      commitPending = false; recMs = 0;

      /* Stop the current answer if it is speaking */
      /* if (speaking) {
           StaticJsonDocument<32> d; d["type"] = "response.cancel"; wsSend(d);
         }*/
    }
    vTaskDelay(1);
  }
}

/*────────────────────── SETUP ─────────────────────────────────*/

void setup()
{
  Serial.begin(115200);
  px.begin(); led(0,0,0);

  // New in 3.x: ledcAttach configures *and* assigns the channel automatically
  if (!ledcAttach(MOT_A1_PIN, PWM_FREQ, PWM_RES) || !ledcAttach(MOT_A2_PIN, PWM_FREQ, PWM_RES)) {
    Serial.println("LEDC attach failed – check pin or frequency");
    while (true) {}  // halt if configuration failed
  }

  // Motor stopped
  setMotorA(0);

  ring = (uint8_t*)ps_malloc(RING_BYTES); if (!ring) abort();

  /* I²S 24 kHz mono */
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  i2s.begin(I2S_MODE_STD, RATE, I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_MONO, I2S_STD_SLOT_RIGHT);
  i2s.configureRX(RATE, I2S_DATA_BIT_WIDTH_16BIT,
                  I2S_SLOT_MODE_MONO, I2S_RX_TRANSFORM_NONE);
  i2s.configureTX(RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);

  pinMode(PTT_PIN, INPUT_PULLUP);

  /* Wi-Fi */
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10'000) delay(50);
  ledState = (WiFi.status() == WL_CONNECTED) ? LED_GREEN : LED_ORANGE;

  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(40);

  /* WebSocket */
  ws.addHeader("Authorization", String("Bearer ") + OPENAI_KEY);
  ws.addHeader("OpenAI-Beta", "realtime=v1");
  ws.onEvent(onEvent);
  ws.onMessage(onMessage);
  ws.setCACert(CA_GTS_ROOT_R4);

  /* Audio amp / gain (unchanged) */
  pinMode(GPIO_PA_EN, OUTPUT);  digitalWrite(GPIO_PA_EN, HIGH);
  pinMode(23, INPUT_PULLDOWN);  // max gain
  

  xTaskCreatePinnedToCore(speakerTask, "spk", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(wsTask,      "ws",  8192, nullptr, 4, nullptr, 0);
}

/*────────────────────── LOOP ──────────────────────────────────*/

void loop()
{
  bool held = (digitalRead(PTT_PIN) == LOW);

  if (held && !pttHeld) {                   // PRESS
    txActive   = true; recMs = 0;
    ledState   = LED_BLINK;
    Serial.println("► REC");
  }
  if (!held && pttHeld) {                  // RELEASE
    txActive      = false;
    commitPending = true;
    releaseT      = millis();
    ledState      = LED_GREEN;
    Serial.println("■ STOP");
  }
  pttHeld = held;
  tickLed();
  vTaskDelay(1);
}
