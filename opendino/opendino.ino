/****************************************************************************************
    esp32_opendino_ptt_realtime.ino — 30 Jun 2025
    ────────────────────────────────────────────────────────────────────────────────────
    • Realtime OpenAI “push-to-talk” client (24 kHz pcm16 I²S in/out) for ESP32.
    • Opens a Wi-Fi / settings *captive portal* when:
         – no Wi-Fi credentials are stored / connection fails, **or**
         – the PTT button (GPIO 19, active-LOW) is held during boot.
    • The portal (SSID **“OpenDino”**) lets you enter:
         – Wi-Fi SSID / password
         – OpenAI **API key**  (*sk-…*)
         – Full **system-prompt** (“instructions”) for the session.
      All three are saved to NVS.

    • While the portal is up, the NeoPixel is **solid orange**.
    • GPIO 21 drives the speaker amplifier (HIGH = mute at boot).
      GPIO 23 is pulled-down for maximum gain (proto board).

    Tested on: ESP32 WROVER, ESP32 S3 DevKit, ESP32-C3 DevKit
    Libraries  : ArduinoWebsockets ≥ 0.6.0, ArduinoJson ≥ 7.0.0,
                 WiFiManager ≥ 2.0.15-alpha, ESP_I2S, Adafruit NeoPixel
 ****************************************************************************************/

#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <ESP_I2S.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <mbedtls/base64.h>
#include <Adafruit_NeoPixel.h>
#include <esp_wifi.h>
#include <Arduino.h>

using namespace websockets;

/*────────────────────────── GPIO MAP ──────────────────────────*/
// I²S
#define I2S_SDOUT     26
#define I2S_BCLK       5
#define I2S_LRCK      25
#define I2S_MCLK       0
#define I2S_SDIN      35
// Push-to-talk (LOW = record)
constexpr gpio_num_t PTT_PIN        = GPIO_NUM_19;
// Speaker amplifier & prototype gain
constexpr gpio_num_t GPIO_PA_EN     = GPIO_NUM_21;   // HIGH = amp OFF
constexpr gpio_num_t GPIO_PROTO_GAIN = GPIO_NUM_23;  // pulled-down = max gain
// NeoPixel status LED
#define NEOPIXEL_PIN 22
#define NUMPIXELS     1
Adafruit_NeoPixel px(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
// (optional) motor GPIOs
constexpr uint8_t  MOT_A1_PIN = 32;
constexpr uint8_t  MOT_A2_PIN = 15;

/*──────────────────────── CLOUD END-POINT ─────────────────────*/
#define MODEL  "gpt-4o-mini-realtime-preview-2024-12-17"
#define WS_URL "wss://api.openai.com/v1/realtime?model=" MODEL

/*────────────────────── TLS root (Google) ─────────────────────*/
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
aodI2D4SseesQ6bDrarZ7C30ddLibZatoKiws3UL9xnELz4ct92vID24FfVbiI1h
Y+SW6FoVHkNeWIP0GCbaM4C6uVdF5dTUsMVs/ZbzNnIdCp5Gxmx5ejvEau8otR/C
skGN+hr/W5GvT1tMBjgWKZ1i4//emhA1JG1BbPzoLJQvyEotc03lXjTaCzv8mEbe
p8RqZ7a2CPsgRbuvTPBwcOMBBmuFeU88+FSBX6+7iP0il8b4Z0QFqIwwMHfs/L6K
1vepuoxtGzi4CZ68zJpiq1UvSqTbFJjtbD4seiMHl
-----END CERTIFICATE-----
)EOF";

/*────────────────────── NVS (API key & prompt) ───────────────*/
Preferences prefs;
const char *NVS_NS          = "opendino";
const char *NVS_APIKEY_KEY  = "api_key";
const char *NVS_PROMPT_KEY  = "prompt";

String openaiKey;     // sk-…
String sysPrompt;

/* default system-prompt */
const char *DEFAULT_PROMPT =
  "You are a toy dinosaur named \"Funny Dino\". "
  "If this is YOUR FIRST reply since the session was created, start with: "
  "\"All my dino friends are dead! Do you want to be my friend?\" "
  "then add a little dino roar. "
  "For every subsequent reply, DO NOT repeat that introduction. "
  "You can walk forward or wag your tail and nod your head. "
  "Finish every answer with a short dino roar.";

/*────────────────────── STATE FLAGS ──────────────────────────*/
enum LedMode { LED_OFF, LED_ORANGE, LED_GREEN, LED_BLINK };
volatile LedMode ledState = LED_ORANGE;

volatile bool wsReady       = false;
volatile bool sessionReady  = false;
volatile bool pttHeld       = false;
volatile bool txActive      = false;
volatile bool commitPending = false;
volatile bool waitingACK    = false;
volatile uint32_t releaseT  = 0;
volatile uint32_t recMs     = 0;
volatile bool speaking      = false;

/*──────────────────── AUDIO RING BUFFER ─────────────────────*/
constexpr uint32_t RATE        = 24'000;         // Hz
constexpr size_t   CHUNK       = 240;            // 10 ms
constexpr size_t   CHUNK_BYTES = CHUNK * 2;      // 16-bit mono
constexpr size_t   RING_BYTES  = 512 * 1024 * 2; // 1 MiB

uint8_t *ring;
volatile size_t head = 0, tail = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

inline size_t rbFree() { return (tail - head - 1 + RING_BYTES) % RING_BYTES; }
inline size_t rbUsed() { return (head - tail + RING_BYTES) % RING_BYTES; }

/*────────────────────── FORWARD DECL. ───────────────────────*/
void startConfigPortal();

/*────────────────────── WiFiManager fields ──────────────────*/
WiFiManager wm;
WiFiManagerParameter p_apikey("api",    "OpenAI API Key", "", 100,  "placeholder=\"sk-...\"");
WiFiManagerParameter p_prompt("prompt", "System prompt / instructions", "", 400, "style=\"height:300px\"");

/*────────────────────── HELPERS ─────────────────────────────*/
void led(uint8_t r, uint8_t g, uint8_t b) {
  px.setPixelColor(0, px.Color(r, g, b));
  px.show();
}
void tickLed() {
  static uint32_t t = 0;
  static bool on = false;
  switch (ledState) {
    case LED_GREEN:  led(0, 255, 0); break;
    case LED_ORANGE: led(255,  80, 0); break;
    case LED_BLINK:
      if (millis() - t > 250) { on = !on; t = millis(); led(0, on ? 255 : 0, 0); }
      break;
    default: led(0, 0, 0);
  }
}

/*────────────────────── NVS LOAD / SAVE ─────────────────────*/
void loadPrefs() {
  prefs.begin(NVS_NS, true);
  openaiKey = prefs.getString(NVS_APIKEY_KEY, "");
  sysPrompt = prefs.getString(NVS_PROMPT_KEY , DEFAULT_PROMPT);
  prefs.end();
}
void savePrefs() {
  prefs.begin(NVS_NS, false);
  prefs.putString(NVS_APIKEY_KEY, openaiKey);
  prefs.putString(NVS_PROMPT_KEY , sysPrompt);
  prefs.end();
}

/*────────────────────── B64 helpers ─────────────────────────*/
String b64(const uint8_t *d, size_t n) {
  size_t out, cap = ((n + 2) / 3) * 4 + 1;
  char *buf = (char *)alloca(cap);
  mbedtls_base64_encode((uint8_t *)buf, cap, &out, d, n);
  buf[out] = 0;
  return String(buf);
}
size_t un64(const char *s, uint8_t *d, size_t cap) {
  size_t out;
  if (!s || !*s) return 0;
  return mbedtls_base64_decode(d, cap, &out, (const uint8_t *)s, strlen(s)) ? 0 : out;
}

/*────────────────────── MOTOR helper (optional) ─────────────*/
constexpr uint32_t PWM_FREQ = 20'000;  // 20 kHz
constexpr uint8_t  PWM_RES  = 8;       // 8 bits (0-255)
void setMotorA(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) { ledcWrite(MOT_A1_PIN, -pwm); ledcWrite(MOT_A2_PIN, 0); }
  else         { ledcWrite(MOT_A1_PIN, 0);    ledcWrite(MOT_A2_PIN,  pwm); }
}

/*────────────────────── I²S & WebSockets ───────────────────*/
I2SClass         i2s;
WebsocketsClient ws;

/*────────────────────── CAPTIVE-PORTAL ─────────────────────*/
/*────────────────────────── onPortalSave ──────────────────────────*/
/*  Called by WiFiManager when the user presses any “Save …” button  */
void onPortalSave() {

  /* Read the raw POST fields straight from the captive-portal server */
  openaiKey = wm.server->arg("api");
  sysPrompt = wm.server->arg("prompt");

  /* -- basic sanitising: remove CR / LF and leading-trailing spaces -- */
  openaiKey.replace('\r', '\0');
  openaiKey.replace('\n', '\0');
  openaiKey.trim();

  sysPrompt.replace('\r', '\0');
  sysPrompt.replace('\n', '\0');
  sysPrompt.trim();
  if (sysPrompt.isEmpty()) sysPrompt = DEFAULT_PROMPT;

  savePrefs();            // persist to NVS
}

/*──────────────────────── startConfigPortal ───────────────────────*/
/*  Entire captive-portal including bigger text-areas and renamed    */
/*  top menu (“Wi-Fi Settings” → “Settings”).                        */
void startConfigPortal() {

  Serial.println(F("[Portal] starting captive portal"));
  ledState = LED_ORANGE;                       // solid orange while portal runs
  led(255, 80, 0);

  /* ---------- 1. Build custom HTML block (API key + prompt) ---------- */
  static char htmlBlock[1600];
  snprintf(htmlBlock, sizeof(htmlBlock), R"HTML(
<style>
  body      {font-family:system-ui,Arial;width:320px;margin:auto;padding:0 4px;}
  .field    {margin:14px 0;}
  label     {display:block;font-weight:600;margin-bottom:5px;}
  textarea,
  input     {width:100%%;padding:8px;border:1px solid #ccc;border-radius:4px;
             box-sizing:border-box;font-family:monospace;font-size:14px;}
  textarea  {resize:vertical;min-height:120px;}
</style>

<script>
  /* — Rename first navbar item “Wi-Fi Settings” → “Settings” — */
  document.addEventListener('DOMContentLoaded', ()=> {
    const hdr = document.querySelector('#header, h2');
    if (hdr && /Wi-?Fi Settings/i.test(hdr.textContent))
        hdr.textContent = 'Settings';
  });
</script>

<div class="field">
  <label>OpenAI API Key</label>
  <textarea name="api" rows="2" maxlength="200"
            autocorrect="off" autocapitalize="off"
            spellcheck="false" style="white-space:nowrap;overflow-x:auto;">%s</textarea>
</div>

<div class="field">
  <label>System prompt / instructions</label>
  <textarea name="prompt" rows="8" maxlength="400"
            spellcheck="false">%s</textarea>
</div>
)HTML",
    openaiKey.c_str(), sysPrompt.c_str());

  /* ---------- 2. Register that block as a WiFiManager “parameter” ---- */
  static WiFiManagerParameter p_custom(htmlBlock);
  wm.addParameter(&p_custom);

  /* ---------- 3. Portal global settings ----------------------------- */
  wm.setSaveParamsCallback(onPortalSave);
  wm.setBreakAfterConfig(true);          // exit when user clicks “Save”
  wm.setShowInfoUpdate(false);           // hide verbose info page

  /* ---------- 4. Launch access-point + web portal ------------------- */
  wm.startConfigPortal("OpenDino");      // blocking call

  /* ---------- 5. Reboot afterwards ---------------------------------- */
  Serial.println(F("[Portal] rebooting"));
  delay(300);
  ESP.restart();
}

/*────────────────────── WS SEND helper ─────────────────────*/
void wsSend(const JsonDocument &j) {
  String s; serializeJson(j, s); ws.send(s);
}

/*────────────────────── pushPcm (audio) ────────────────────*/
void pushPcm(const char *b64str) {
  static uint8_t tmp[CHUNK_BYTES * 150];   // up to 500 ms decoded
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
      idx += chunk;
    }
    portEXIT_CRITICAL(&mux);
    if (idx < n) vTaskDelay(1);
  }
  speaking = true;
}

/*────────────────────── WS CALLBACKS ───────────────────────*/
void onMessage(WebsocketsMessage m) {
  if (!m.isText()) return;
  StaticJsonDocument<2048> j;
  if (deserializeJson(j, m.data())) return;

  const char *t = j["type"] | "";
  Serial.printf("[WSS] %s\n", t);

  /* 1. session.created → configure */
  if (!strcmp(t, "session.created")) {
    StaticJsonDocument<1536> u;
    u["type"] = "session.update";
    JsonObject s = u["session"].to<JsonObject>();

    JsonArray mods = s.createNestedArray("modalities");
    mods.add("text"); mods.add("audio");
    s["input_audio_format"]  = "pcm16";
    s["output_audio_format"] = "pcm16";
    s["turn_detection"]      = nullptr;
    s["instructions"]        = sysPrompt.c_str();

    s["tool_choice"] = "auto";
    JsonArray tools = s.createNestedArray("tools");
    JsonObject tool = tools.createNestedObject();
    tool["type"] = "function"; tool["name"] = "move";
    tool["description"] = "Moves Funny Dino: 0=wag tail and nod happily, 1=walk forward.";
    JsonObject prm = tool["parameters"].to<JsonObject>();
    prm["type"] = "object";
    JsonObject props = prm.createNestedObject("properties");
    JsonObject p1 = props.createNestedObject("move_type");
    p1["type"] = "integer"; { JsonArray e = p1.createNestedArray("enum"); e.add(0); e.add(1); }
    p1["description"] = "0 wag tail, 1 walk forward";
    JsonObject p2 = props.createNestedObject("speed");
    p2["type"] = "integer"; p2["minimum"] = 1; p2["maximum"] = 10;
    p2["description"] = "Speed 1–10";
    JsonObject p3 = props.createNestedObject("time_ms");
    p3["type"] = "integer"; p3["minimum"] = 1000; p3["maximum"] = 5000;
    p3["description"] = "Duration in milliseconds";
    JsonArray req = prm.createNestedArray("required");
    req.add("move_type"); req.add("speed"); req.add("time_ms");

    wsSend(u);
  }

  /* 2. session.updated → force greeting */
  else if (!strcmp(t, "session.updated")) {
    sessionReady = true;
    StaticJsonDocument<32> r; r["type"] = "response.create";
    wsSend(r);
  }

  /* 3. commit ACK */
  else if (!strcmp(t, "input_audio_buffer.committed") && waitingACK) {
    waitingACK = false;
    StaticJsonDocument<32> r; r["type"] = "response.create";
    wsSend(r);
  }

  /* 4. audio streaming */
  else if (!strcmp(t, "response.audio.delta"))             pushPcm(j["delta"]);
  else if (!strcmp(t, "response.audio.done"))              speaking = false;
  else if (!strcmp(t, "response.content_part.added") &&
           j["part"]["type"] == "audio")                   pushPcm(j["part"]["audio"]);
  else if (!strcmp(t, "response.output_item.added")) {
    for (JsonObject p : j["item"]["content"].as<JsonArray>())
      if (p["type"] == "audio") pushPcm(p["audio"]);
  }

  /* 5. response.done → handle move() calls */
  else if (!strcmp(t, "response.done")) {
    for (JsonObject itm : j["response"]["output"].as<JsonArray>()) {
      if (strcmp(itm["type"] | "", "function_call")) continue;
      if (strcmp(itm["name"] | "", "move")) continue;

      StaticJsonDocument<256> args;
      if (deserializeJson(args, itm["arguments"].as<const char *>())) continue;
      uint8_t  move_type = args["move_type"] | 0;
      uint8_t  speed     = args["speed"]     | 1;
      uint32_t time_ms   = args["time_ms"]   | 1000;

      int pwm = map(speed, 1, 10, 190, 255);
      if (move_type == 0) pwm = -pwm;
      setMotorA(pwm); delay(time_ms); setMotorA(0);

      StaticJsonDocument<256> done;
      done["type"] = "conversation.item.create";
      JsonObject item = done.createNestedObject("item");
      item["type"]    = "function_call_output";
      item["call_id"] = itm["call_id"];
      StaticJsonDocument<32> res; res["result"] = "ok"; String resStr; serializeJson(res, resStr);
      item["output"]  = resStr;
      wsSend(done);
      StaticJsonDocument<32> rc; rc["type"] = "response.create"; wsSend(rc);
    }
  }
}

void onEvent(WebsocketsEvent e, String) {
  if (e == WebsocketsEvent::ConnectionOpened) {
    wsReady = true;  ledState = LED_GREEN; Serial.println("[WSS] opened");
  } else if (e == WebsocketsEvent::ConnectionClosed) {
    wsReady = false; ledState = LED_ORANGE; Serial.println("[WSS] closed");
  } else if (e == WebsocketsEvent::GotPing) ws.pong();
}

/*────────────────────── SPEAKER TASK (core 1) ───────────────*/
void speakerTask(void *) {
  static uint8_t buf[CHUNK_BYTES * 4 + 4];
  bool primed = false;
  constexpr uint32_t PRIME_MS    = 500;
  constexpr size_t   PRIME_BYTES = RATE * 2 * PRIME_MS / 1000;

  for (;;) {
    size_t avail = rbUsed();
    if (!primed) {
      if (avail < PRIME_BYTES) { vTaskDelay(1); continue; }
      primed = true; Serial.println("[Audio] primed");
    }
    if (avail == 0) { memset(buf, 0, CHUNK_BYTES); i2s.write(buf, CHUNK_BYTES); vTaskDelay(1); continue; }

    portENTER_CRITICAL(&mux);
    size_t take  = std::min(avail, (size_t)(sizeof(buf) - 4));
    size_t first = std::min(take, RING_BYTES - tail);
    memcpy(buf,           ring + tail, first);
    memcpy(buf + first,   ring,        take - first);
    tail = (tail + take) % RING_BYTES;
    portEXIT_CRITICAL(&mux);

    if (take & 3) { memset(buf + take, 0, 4 - (take & 3)); take = (take + 3) & ~3; }
    i2s.write(buf, take);
    vTaskDelay(1);
  }
}

/*────────────────────── WEBSOCKET TASK (core 0) ─────────────*/
void wsTask(void *) {
  static uint8_t mic[CHUNK_BYTES];
  uint32_t lastPing = millis(), backoff = 1000;

  for (;;) {
    if (!wsReady) {
      vTaskDelay(backoff / portTICK_PERIOD_MS);
      Serial.println("[WSS] connecting…");
      ws.connect(WS_URL);
      backoff = std::min<uint32_t>(backoff * 2, 16000);
      continue;
    }
    backoff = 1000;
    ws.poll();
    if (millis() - lastPing > 30000) { ws.ping(); lastPing = millis(); }

    if (sessionReady && txActive &&
        i2s.readBytes((char *)mic, CHUNK_BYTES) == CHUNK_BYTES) {
      StaticJsonDocument<384> a;
      a["type"]  = "input_audio_buffer.append";
      a["audio"] = b64(mic, CHUNK_BYTES);
      wsSend(a);
      recMs += 10;
    }

    if (commitPending && millis() - releaseT > 60) {
      StaticJsonDocument<32> c;
      const char *tp = (recMs >= 100) ? "input_audio_buffer.commit"
                                      : "input_audio_buffer.clear";
      c["type"] = tp;
      wsSend(c);
      waitingACK    = (strcmp(tp, "input_audio_buffer.commit") == 0);
      commitPending = false; recMs = 0;
    }
    vTaskDelay(1);
  }
}

/*──────────────────────────── SETUP ─────────────────────────*/
void setup() {
  Serial.begin(115200);
  px.begin(); led(0, 0, 0);
  pinMode(PTT_PIN, INPUT_PULLUP);

  /* Amp OFF at boot & max gain */
  pinMode(GPIO_PA_EN, OUTPUT);       digitalWrite(GPIO_PA_EN, HIGH);
  pinMode(GPIO_PROTO_GAIN, INPUT_PULLDOWN);

  /* Motor PWM channels */
pinMode(MOT_A1_PIN, INPUT_PULLDOWN);
pinMode(MOT_A2_PIN, INPUT_PULLDOWN);
  ledcAttach(MOT_A1_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOT_A2_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(MOT_A1_PIN, 0);
  ledcWrite(MOT_A2_PIN, 0);

  /* Wi-Fi — portal if no connection or PTT held */
  bool pttBoot = digitalRead(PTT_PIN) == LOW;
  WiFi.begin();
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(50);
  bool wifiOK = WiFi.status() == WL_CONNECTED;
  if (pttBoot || !wifiOK) startConfigPortal();

  /* Allocate ring-buffer in PSRAM */
  ring = (uint8_t *)ps_malloc(RING_BYTES);
  if (!ring) abort();

  /* I²S 24 kHz mono */
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  i2s.begin(I2S_MODE_STD, RATE, I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_MONO, I2S_STD_SLOT_RIGHT);
  i2s.configureRX(RATE, I2S_DATA_BIT_WIDTH_16BIT,
                  I2S_SLOT_MODE_MONO, I2S_RX_TRANSFORM_NONE);
  i2s.configureTX(RATE, I2S_DATA_BIT_WIDTH_16BIT,
                  I2S_SLOT_MODE_MONO);

  /* Load stored API key + prompt */
  loadPrefs();



  ledState = LED_GREEN;
  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(40);

  //Serial.println(openaiKey);

  /* Secure WebSocket */
  ws.setCACert(CA_GTS_ROOT_R4);
  ws.addHeader("Authorization", String("Bearer ") + openaiKey);
  ws.addHeader("OpenAI-Beta",   "realtime=v1");
  ws.onEvent(onEvent);
  ws.onMessage(onMessage);
  ws.connect(WS_URL);

  /* Tasks */
  xTaskCreatePinnedToCore(speakerTask, "spk", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(wsTask,      "ws" , 8192, nullptr, 4, nullptr, 0);

  Serial.println("[Setup] complete");
}

/*──────────────────────────── LOOP ─────────────────────────*/
void loop() {
  bool held = (digitalRead(PTT_PIN) == LOW);

  if (held && !pttHeld) {            // press
    txActive = true; recMs = 0; releaseT = millis();
    ledState = LED_BLINK; Serial.println("► REC");
  }
  if (!held && pttHeld) {            // release
    txActive = false;
    commitPending = true;
    releaseT = millis();
    ledState = LED_GREEN; Serial.println("■ STOP");
  }
  pttHeld = held;

  tickLed();
  vTaskDelay(1);
}
