/*
 * ============================================================================
 *  GARDEROBSBELYSNING — TOUCHPANEL (DIYmalls ESP32-2432S028R)
 *  Firmware v9.6-PANEL · bänkversion b6 · 2026-09-02
 *
 *  DISPLAYEN ÄR EN ILI9341 — fastställt genom prov på bänken 2026-09-02.
 *  v9.6 §11 utgår från ST7789 (V34: "två USB-portar = version 3"), och den
 *  inställningen gav helt vit skärm. Drivrutinen väljs i tft_setup.h.
 *
 *  NYTT I b7
 *   • Text som ändras skrivs över sin gamla bild i stället för att suddas först
 *     (setTextPadding). Nedräkningens siffra och statusfältet blinkade annars
 *     svart en bildruta varje sekund — det var flimret som satt kvar.
 *   • Accentlisten håller EN storhet per pixel (linjär ljusstyrka) och färgen
 *     räknas fram vid utsändning. De gula, gröna och röda pixlarna i slutet av
 *     släckanimationen var avrundning och strömtak som drog isär R, G och B.
 *   • Ingen kvitteringspuls på reglaget: ett drag som tappar trycket räknades som
 *     nya tryck och blixtrade hela listen mitt i nivåmätaren.
 *   • Att öppna NIVÅ-vyn skickar nivån direkt, så att gruppen tänds med en gång.
 *   • Kaskadspegeln har åtta steg (px 7 = ÖVRE, px 8 = NEDRE) nu när listerna
 *     tänds efter varandra i UNO:n.
 *
 *  NYTT I b6 (efter bänkvideorna)
 *   • Länkvakten räknade "now − S.at" osignerat med ett 'now' som lästs FÖRE
 *     nedlänken polldes: kom en $S i samma varv blev differensen ett jättetal och
 *     panelen ritade "Ingen kontakt" i ett varv (röd accentlist i en tick) — det
 *     var flimret vid knapptryck och listens "olika färger". Rättat.
 *   • Skärmen ritas om i delar: nedräkningen byter bara siffran och stapeln,
 *     ett lägesbyte bara texten och knapparna, av/på bara rutan. Ingen hel-
 *     omritning per sekund längre.
 *   • Accentlisten har en egen animator (50 Hz): regndroppe uppifrån och ned i
 *     KASKAD, hel toning i SAMTIDIGT, spegelvänd släckning, mjuk övergång till
 *     vilonivå, tryckpuls som återgår till samma nivå, glidande mätare i NIVÅ.
 *     Nivåbilden är ett tillstånd, inte en tidsfråga — listen slocknar inte
 *     längre "efter ett tag". RMT får fyra minnesblock: hela ramen ryms.
 *
 *  Följer "Kopplingsschema garderobsbelysning v9.6" §5, §7, §11 (paneldelen).
 *
 *  KRAV
 *   • Arduino-core "esp32" 3.x (testkompilerad med 3.3.11). Kort: "ESP32 Dev Module".
 *   • TFT_eSPI 2.5.43. Displaykonfigurationen ligger i **tft_setup.h i den här
 *     mappen** — TFT_eSPI hittar den själv och struntar då i bibliotekets
 *     User_Setup.h. Ingenting behöver ändras inne i biblioteksmappen.
 *   • XPT2046_Touchscreen 1.4 (Paul Stoffregen).
 *   • Accentlisten (§5B) drivs utan FastLED — direkt via ESP32:ns RMT. Den är
 *     byggd och PÅ: 8 pixlar lodrätt längs panelens vänsterkant, DIN uppåt.
 *
 *  LÄNKEN
 *   • Nedlänk: Serial1 RX-only på GPIO35, 115200 ($S 2 Hz, $P 20 Hz under sekvens).
 *   • Upplänk: Serial2 TX-only på GPIO22, 9600, INVERTERAD (idle LÅG) så att S8050
 *     på adapterkortet är öppen i vila → p4 vilar på 5 V vid UNO:ns D8 (V28/S12).
 *     UNO:n ska då ha UPLINK_INVERTED 0. (De två flaggorna är alltid komplementära.)
 *   • Konsol: Serial (CH340/micro-USB) 115200 — dra ALLTID ur JST XH först (V27).
 *
 *  PRINCIP (F3/F4): ett tryck är ett förslag. Panelen sänder $M/$B/$W/$E, UNO:n
 *  validerar, och skärmen ritar om först när $S speglar det nya tillståndet.
 * ============================================================================
 */

#define FW_VERSION "v9.6-PANEL b7"

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Preferences.h>

// Fångar det vanligaste uppstartsfelet vid kompilering i stället för som vit skärm:
#if !defined(ST7789_DRIVER) && !defined(ILI9341_DRIVER)
  #error "Displaykonfigurationen saknas. Filen tft_setup.h maste ligga i SAMMA mapp som garderob_panel.ino."
#endif
#if !defined(TFT_CS) || (TFT_CS != 15) || (TFT_DC != 2) || (TFT_SCLK != 14) || (TFT_MOSI != 13)
  #error "Fel pinnar for ESP32-2432S028R. Ligger en gammal User_Setup.h och stor? Se tft_setup.h."
#endif
#if !defined(TFT_BL) || (TFT_BL != 21)
  #error "TFT_BL maste vara 21 pa detta kort. Se tft_setup.h."
#endif
#include <esp_system.h>
#if __has_include(<esp_random.h>)
  #include <esp_random.h>
#endif

/* --------------------------------------------------------------------------
 *  1. KONFIGURATION
 * -------------------------------------------------------------------------- */
#define UPLINK_TX_INVERT   true    // komplement till UNO:ns UPLINK_INVERTED (0 ↔ true)
/* --- ACCENTLISTEN (§5B, rad 87) — monterad och i drift -------------------
 *  Listen sitter LODRÄTT längs panelens vänstra kant, 8 adresserbara pixlar,
 *  dataingången (DIN) i ÖVRE änden. Det avviker på tre punkter från §5B, som
 *  utgick från nio pixlar i vågrätt läge:
 *    · 8 px i stället för 9  → två pixlar per ljusgrupp går jämnt ut
 *    · lodrätt i stället för vågrätt → "pilen mot garderoben" blir en våg
 *      som rullar uppifrån och ned, inte i sidled
 *    · DIN uppåt → pixel 1 är den översta
 *
 *  Kaskadspegeln (2 px per grupp):
 *      px 1–2  Spot A (bortre)      \
 *      px 3–4  Spot B (mitt)         |  samma ordning som ljuset i rummet
 *      px 5–6  Spot C (närmast)      |
 *      px 7–8  stripsen (ÖVRE/NEDRE) /
 * ------------------------------------------------------------------------ */
#define ACCENT_STRIP       1       // listen är byggd på CN1/IO27
#define ACCENT_PIN         27
#define ACCENT_N           8       // antal pixlar på listen
#define ACCENT_MAX_MA      150     // V33/V47 — listtaket, mjukvarustyrt

// Fysisk montering. Ändra bara om listen sitter annorlunda eller vågen ser fel ut.
#define ACCENT_DIN_AT_TOP     1    // 1 = dataingången sitter i övre änden (pixel 1 = överst)
#define ACCENT_CASCADE_DOWN   1    // 1 = vågen rullar uppifrån och ned (Spot A överst,
                                   //     stripsen nederst). 0 vänder hela vågen. (V46)
#define ACCENT_GRB            1    // 1 = WS2812B/GRB. Byter grönt och rött plats: sätt 0.

// Nivåer i procent UPPLEVD ljusstyrka (CIE-kurvan läggs på i utmatningen, samma
// kurva som UNO:n kör på lamporna — listen och rummet tonar då i takt).
#define ACCENT_REST_PCT      30    // vilonivå när lamporna lyser: lägre än sekvensen,
                                   // så att tryckpulsen syns ovanpå
#define ACCENT_SEQ_PCT       65    // fyllnadsnivå under en pågående sekvens
#define ACCENT_HEAD_PCT     100    // regndroppens huvud (KASKAD)
#define ACCENT_IDLE_PCT       8    // glöd när systemet är TÄNT men lamporna släckta
                                   // (tomt rum). 0 = helt mörk. SLÄCKT är alltid mörk.
#define ACCENT_GAUGE_PCT     60    // mätaren i NIVÅ-vyerna
#define ACCENT_TRACK_PCT      4    // mätarens ofyllda del (svag "skala")
#define ACCENT_TOUCH_PCT     95    // tryckpulsens topp
#define ACCENT_TOUCH_UP_MS   40    // pulsens stigtid
#define ACCENT_TOUCH_DOWN_MS 260   // pulsens återgång (mjuk, exponentiell)
#define ACCENT_HEAD_SIGMA   0.75f  // droppens bredd i pixlar
#define ACCENT_TAIL_PX      1.6f   // ljusspårets längd bakom droppen (e-faltning)
#define ACCENT_REST_MS      700    // övergång sekvens → vilonivå ("stilren", ease-out)

#define BL_PIN             21
#define BL_IDLE            20      // ~8 % — skärmen släcks aldrig helt (§7)
#define BL_CAP             150     // full styrka bländar i garderobsmörker
#define BL_UP_MS           300
#define BL_DOWN_MS         500

#define Z_THRESHOLD_DEFAULT 600    // halva uppmätt Z vid normalt fingertryck (V37)
#define TOUCH_SAMPLE_MS    20      // 3 avläsningar = 60 ms (lager 2)
#define TOUCH_CONSIST_PX   25
#define WAKE_LOCKOUT_MS    500     // Ä4 — tryck i standby väcker bara
#define LOCAL_WAKE_MS      15000   // hur länge ett tryck håller skärmen vaken lokalt
#define LINK_TIMEOUT_MS    2000    // "Ingen kontakt med styrenheten"
#define P_TIMEOUT_MS       200     // ingen $P → reservväg via $S
#define P_IDLE_MS          2000    // UNO:ns vilopuls (måste matcha UNO:ns P_IDLE_MS)
#define REQ_RETRY_MS       150     // upprepning tills $S bekräftar (F8: max 1,45 s kaskad)
#define REQ_MAX_TRIES      17      // ≈2,5 s
#define B_MIN_INTERVAL_MS  100     // $B max 10/s
#define FLASH_MS           120     // tryckytan inverteras till bärnsten

// XPT2046 på egen buss (VSPI)
#define XPT_IRQ  36
#define XPT_MOSI 32
#define XPT_MISO 39
#define XPT_CLK  25
#define XPT_CS   33

// RGB-lysdioden (aktivt LÅG) hålls släckt
#define LED_R 4
#define LED_G 16
#define LED_B 17

// Färger (RGB565)
#define C_BG      0x0000
#define C_BAND    0x2124
#define C_BOX     0x39E7
#define C_TEXT    0xFFFF
#define C_DIM     0x9CD3
#define C_AMBER   0xFCC0
#define C_AMBERD  0x7A40
#define C_RED     0xF800
#define C_GREEN   0x2E44
#define C_TRACK   0x4A69

enum { MODE_MANUAL = 0, MODE_CASCADE = 1, MODE_SIMUL = 2 };
enum { VIEW_MAIN = 0, VIEW_ADJ_SPOT = 1, VIEW_ADJ_LIST = 2, VIEW_ERR = 3, VIEW_STANDBY = 4, VIEW_SEQ = 5 };
enum UiView { UI_MAIN = 0, UI_ADJ_SPOT, UI_ADJ_LIST, UI_ERR, UI_CAL };
enum BtnId { BT_NONE = 0, BT_MODE0, BT_MODE1, BT_MODE2, BT_POWER, BT_TITLE, BT_OTHER_GROUP, BT_SAVE, BT_CANCEL, BT_ACK, BT_SLIDER };

/* --------------------------------------------------------------------------
 *  2. GLOBALA OBJEKT OCH TILLSTÅND
 * -------------------------------------------------------------------------- */
TFT_eSPI tft;
SPIClass tsSpi(VSPI);
XPT2046_Touchscreen ts(XPT_CS, XPT_IRQ);
Preferences prefs;

// Systemtillstånd från UNO ($S) — enda sanningskällan
struct SysState { uint8_t mode, warm, spot, list, view, err, on; bool valid; uint32_t at; };
static SysState S = {1, 0, 100, 100, 4, 0, 1, false, 0};
static uint8_t  lvl[5] = {0, 0, 0, 0, 0};
static uint32_t lastPAt = 0, prevPAt = 0;
static int8_t   seqDir = 0;            // riktning på senaste $P-strömmen: +1 upp, −1 ned, 0 stilla
static bool     pStartedDark = false;  // strömmen började från helt släckt (= en tändning, inte en nivåomställning)
static uint32_t framesS = 0, framesP = 0, framesBad = 0;

// Lokal UI. Smutsflaggorna är avsiktligt finkorniga: varje flagga ritar om så
// lite som möjligt, för en helomritning syns som ett blink (bänkvideo 1 och 3).
static UiView   ui = UI_MAIN;
static bool     titleDirty = true;     // hela titelbandet
static bool     statusDirty = false;   // bara statustexten till höger i titeln
static bool     workDirty = true;      // hela arbetsytan
static bool     bodyDirty = false;     // arbetsytans nedre del (under av/på-rutan)
static bool     powerDirty = false;    // bara av/på-rutan
static bool     warmDirty = false;     // bara nedräkningens siffra och stapel
static bool     actionDirty = true;    // hela åtgärdsbandet
static bool     modeBtnDirty = false;  // bara de tre lägesknapparna (ingen bandrensning)
static bool     barsDirty = false, sliderDirty = false;
static uint32_t awakeUntil = 0, lockoutUntil = 0;
static uint8_t  adjGroup = 0;          // 0 = spot, 1 = list
static uint8_t  savedSpot = 100, savedList = 100;
static uint32_t adjEnteredAt = 0;
static uint8_t  errAckedCode = 0;
static bool     linkLost = true;
static bool     seqShown = false;
static uint32_t flashUntil = 0; static uint8_t flashId = 0; static UiView flashUi = UI_MAIN;
static uint32_t ghostSamples = 0, confirmedTouches = 0;
#if ACCENT_STRIP
static uint32_t accTouchAt = 0;        // senaste registrerade tryck (tryckpulsen på listen)
#endif

// Kalibrering (NVS "panel")
struct Cal { int16_t xmin, xmax, ymin, ymax; uint8_t swap; uint16_t zthr; bool valid; };
static Cal cal = {300, 3800, 300, 3800, 0, Z_THRESHOLD_DEFAULT, false};
static uint8_t  calStep = 0; static int16_t calRawX[4], calRawY[4];

// Bakgrundsljus
static float    blCur = BL_IDLE;

// Upplänk — sekvensnummer och väntande begäran
static uint16_t seqNo = 1;
struct Req { bool active; char kind; uint8_t val; uint8_t grp; uint16_t seq; uint32_t sentAt, nextAt; uint8_t tries; };
static Req req = {false, 0, 0, 0, 0, 0, 0, 0};
static uint32_t lastBAt = 0; static uint8_t lastBSent = 0; static bool bPending = false; static uint8_t bPendingVal = 0;

/* --------------------------------------------------------------------------
 *  3. HJÄLPFUNKTIONER — bakgrundsljus, text med ÅÄÖ, ramar
 * -------------------------------------------------------------------------- */
static void blInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BL_PIN, 5000, 8);
#else
  ledcSetup(0, 5000, 8); ledcAttachPin(BL_PIN, 0);
#endif
}
static void blWrite(uint8_t v) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(BL_PIN, v);
#else
  ledcWrite(0, v);
#endif
}

// CIE 1931-kurvan: upplevd nivå 0..100 → linjär 0..255 (samma kurva som UNO:n)
static uint8_t cie255(uint8_t pct) {
  float L = pct / 100.0f;
  float Y = (L <= 0.08f) ? L / 9.033f : powf((L + 0.16f) / 1.16f, 3.0f);
  int v = (int)(Y * 255.0f + 0.5f); if (v > 255) v = 255; if (v < 0) v = 0;
  return (uint8_t)v;
}

// Ritar UTF-8-text med Å/Ä/Ö (fonterna saknar dem): basbokstav + prickar/ring.
// datum: 0 = vänster, 1 = centrerad, 2 = höger (referens = textens överkant)
static void drawSE(const char *utf8, int x, int y, int font, uint8_t datum, uint8_t size = 1) {
  char ascii[64]; uint8_t mark[64]; int n = 0;
  for (const uint8_t *p = (const uint8_t *)utf8; *p && n < 63; ) {
    if (p[0] == 0xC3 && p[1]) {
      uint8_t c = p[1]; p += 2;
      switch (c) {
        case 0x84: ascii[n] = 'A'; mark[n++] = 2; break;   // Ä
        case 0x96: ascii[n] = 'O'; mark[n++] = 2; break;   // Ö
        case 0x85: ascii[n] = 'A'; mark[n++] = 1; break;   // Å
        case 0xA4: ascii[n] = 'a'; mark[n++] = 2; break;   // ä
        case 0xB6: ascii[n] = 'o'; mark[n++] = 2; break;   // ö
        case 0xA5: ascii[n] = 'a'; mark[n++] = 1; break;   // å
        default:   ascii[n] = '?'; mark[n++] = 0; break;
      }
    } else if (*p >= 0x80) {                      // annan UTF-8-sekvens → '?'
      uint8_t len = (*p >= 0xF0) ? 4 : (*p >= 0xE0) ? 3 : 2;
      ascii[n] = '?'; mark[n++] = 0; p += len;
    } else { ascii[n] = (char)*p; mark[n++] = 0; p++; }
  }
  ascii[n] = 0;
  tft.setTextSize(size);
  tft.setTextDatum(datum == 1 ? TC_DATUM : (datum == 2 ? TR_DATUM : TL_DATUM));
  int w = tft.textWidth(ascii, font);
  int left = (datum == 1) ? x - w / 2 : (datum == 2 ? x - w : x);
  tft.drawString(ascii, x, y, font);
  uint16_t col = tft.textcolor;
  for (int i = 0; i < n; i++) {
    if (!mark[i]) continue;
    char pre[64]; memcpy(pre, ascii, i); pre[i] = 0;
    char one[2] = { ascii[i], 0 };
    int gx = left + tft.textWidth(pre, font);
    int gw = tft.textWidth(one, font);
    int d = (font >= 4 ? 2 : 1) * size;          // prickstorlek
    int dy = y - d - 1; if (dy < 0) dy = 0;
    bool upper = (ascii[i] >= 'A' && ascii[i] <= 'Z');
    if (!upper) dy = y + (font >= 4 ? 4 : 2) * size;   // gemener har luft ovanför
    if (mark[i] == 2) {
      tft.fillRect(gx + gw / 4, dy, d, d, col);
      tft.fillRect(gx + gw - gw / 4 - d, dy, d, d, col);
    } else {
      tft.drawCircle(gx + gw / 2, dy + d / 2, d, col);
    }
  }
  tft.setTextSize(1);
}

static void sendFrame(const char *body) {   // "$<body>*XX\n" på upplänken
  uint8_t x = 0;
  for (const char *p = body; *p; p++) x ^= (uint8_t)*p;
  char out[48];
  snprintf(out, sizeof out, "$%s*%02X\n", body, x);
  Serial2.print(out);
}
static uint16_t nextSeq() { seqNo++; if (seqNo == 0 || seqNo > 65000) seqNo = 1; return seqNo; }
static void reqSendNow() {
  char b[32];
  switch (req.kind) {
    case 'M': snprintf(b, sizeof b, "M,%u,%u", req.val, req.seq); break;
    case 'E': snprintf(b, sizeof b, "E,%u,%u", req.val, req.seq); break;
    case 'W': snprintf(b, sizeof b, "W,%u", req.seq); break;
    default: return;
  }
  sendFrame(b);
  req.tries++;
  req.nextAt = millis() + REQ_RETRY_MS;
}
static void request(char kind, uint8_t val) {
  req.active = true; req.kind = kind; req.val = val; req.seq = nextSeq();
  req.sentAt = millis(); req.tries = 0;
  reqSendNow();
  Serial.printf("[%lu] -> $%c %u (seq %u)\n", (unsigned long)millis(), kind, val, req.seq);
}
static bool reqConfirmed() {
  if (!S.valid || S.at < req.sentAt) return false;   // behöver en $S efter sändningen
  switch (req.kind) {
    case 'M': return S.mode == req.val;
    case 'E': return S.on == req.val;
    case 'W': return S.view != VIEW_ADJ_SPOT && S.view != VIEW_ADJ_LIST;
  }
  return true;
}
static void reqService() {
  if (!req.active) return;
  if (reqConfirmed()) { req.active = false; return; }
  if ((int32_t)(millis() - req.nextAt) >= 0) {
    if (req.tries >= REQ_MAX_TRIES) { req.active = false; Serial.println("[req] ingen bekraftelse — ger upp"); return; }
    reqSendNow();
  }
}
static void sendB(uint8_t grp, uint8_t pct) {       // $B ×1, max 10/s
  char b[24]; snprintf(b, sizeof b, "B,%u,%u,%u", grp, pct, nextSeq());
  sendFrame(b); lastBAt = millis(); lastBSent = pct;
}

/* --------------------------------------------------------------------------
 *  4. NEDLÄNKEN — $S och $P
 * -------------------------------------------------------------------------- */
static char rxBuf[64]; static uint8_t rxLen = 0; static bool rxOverflow = false;
static uint16_t parseU(char **pp, bool *ok) {
  char *p = *pp; if (*p < '0' || *p > '9') { *ok = false; return 0; }
  char *e; unsigned long v = strtoul(p, &e, 10); if (v > 65535) { *ok = false; return 0; }
  if (*e == ',') e++; *pp = e; return (uint16_t)v;
}
static void onStatus(uint8_t mode, uint8_t warm, uint8_t spot, uint8_t list, uint8_t view, uint8_t err, uint8_t on) {
  SysState o = S;
  S.mode = mode; S.warm = warm; S.spot = spot; S.list = list; S.view = view; S.err = err; S.on = on;
  S.valid = true; S.at = millis(); framesS++;
  if (!o.valid || o.err != err) { titleDirty = true; workDirty = true; actionDirty = true; return; }
  // Därefter bara det som faktiskt ändrats:
  if (o.mode != mode) { titleDirty = true; bodyDirty = true; modeBtnDirty = true; }
  if (o.on != on)     { powerDirty = true; bodyDirty = true; }
  if (o.warm != warm) {
    statusDirty = true;                                 // "PIR nn s" / "vaken"
    if (warm && o.warm) warmDirty = true;               // 38 → 37: bara siffran och stapeln
    else bodyDirty = true;                              // nedräkningen börjar/slutar: texten byts
  }
  if (o.view != view) statusDirty = true;               // "vila"/"vaken" — aldrig helomritning
  if (o.spot != spot || o.list != list) { if (ui == UI_ADJ_SPOT || ui == UI_ADJ_LIST) sliderDirty = true; }
  if (err != errAckedCode) { if (err == 0) errAckedCode = 0; }
}
static void handleRx(char *buf) {
  char *star = strchr(buf, '*'); if (!star || star == buf) { framesBad++; return; }
  uint8_t x = 0; for (char *p = buf; p < star; p++) x ^= (uint8_t)*p;
  char *e; unsigned long want = strtoul(star + 1, &e, 16);
  if (e != star + 3 || want != x) { framesBad++; return; }
  *star = 0;
  if (buf[1] != ',') { framesBad++; return; }
  char *p = buf + 2; bool ok = true;
  if (buf[0] == 'S') {
    uint16_t f[7]; for (int i = 0; i < 7; i++) f[i] = parseU(&p, &ok);
    if (!ok || f[0] > 2 || f[4] > 5 || f[6] > 1) { framesBad++; return; }
    onStatus(f[0], f[1], f[2], f[3], f[4], f[5], f[6]);
  } else if (buf[0] == 'P') {
    uint16_t f[5]; for (int i = 0; i < 5; i++) f[i] = parseU(&p, &ok);
    if (!ok) { framesBad++; return; }
    int before = 0, after = 0;
    for (int i = 0; i < 5; i++) { before += lvl[i]; lvl[i] = f[i] > 100 ? 100 : (uint8_t)f[i]; after += lvl[i]; }
    if (after > before) seqDir = 1; else if (after < before) seqDir = -1;   // lika: behåll riktningen
    if ((int32_t)(millis() - lastPAt) >= (int32_t)P_TIMEOUT_MS) pStartedDark = (before == 0);   // ny ström: från släckt?
    prevPAt = lastPAt; lastPAt = millis(); framesP++; barsDirty = true;
  } else framesBad++;
}
static void pollDownlink() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '$') { rxLen = 0; rxOverflow = false; continue; }
    if (c == '\n' || c == '\r') {
      if (rxLen && !rxOverflow) { rxBuf[rxLen] = 0; handleRx(rxBuf); }
      rxLen = 0; rxOverflow = false; continue;
    }
    if (rxLen < sizeof rxBuf - 1) rxBuf[rxLen++] = c; else rxOverflow = true;
  }
}

/* --------------------------------------------------------------------------
 *  5. RITNING — tre band: titel 0–31 · arbetsyta 32–207 · åtgärder 208–239
 * -------------------------------------------------------------------------- */
// pStream: strömmar $P i 20 Hz, dvs en sekvens pågår just nu. Bakgrundsljuset och
//          sekvensstaplarna följer bara strömmande $P — vilopulsen (var 2:a s)
//          får inte blinka till skärmen. Nivåbilden lvl[] i sig är ett TILLSTÅND
//          som gäller tills nästa $P säger något annat; ingen tidsgräns.
static bool pStream() { return (int32_t)(millis() - lastPAt) < (int32_t)P_TIMEOUT_MS && (lastPAt - prevPAt) < 150UL; }
static bool pActive() { return pStream(); }
static bool locallyAwake() { return (int32_t)(millis() - awakeUntil) < 0; }
static bool awake() { return locallyAwake() || (S.valid && S.view != VIEW_STANDBY); }
static const char *modeName(uint8_t m) { return m == 0 ? "MANUELL" : (m == 1 ? "KASKAD" : "SAMTIDIGT"); }

struct Btn { int16_t x, y, w, h; uint8_t id; const char *label; };
static Btn btns[8]; static uint8_t nBtns = 0;
static void addBtn(int x, int y, int w, int h, uint8_t id, const char *label = NULL) {
  if (nBtns < 8) btns[nBtns++] = { (int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h, id, label };
}
static void drawBox(int x, int y, int w, int h, uint16_t fill, uint16_t txt, const char *label, int font = 2) {
  tft.fillRoundRect(x, y, w, h, 4, fill);
  tft.setTextColor(txt, fill);
  drawSE(label, x + w / 2, y + (h - tft.fontHeight(font)) / 2, font, 1);
}
static const int ACT_X[3] = { 8, 112, 216 }, ACT_Y = 211, ACT_W = 94, ACT_H = 26;   // tre rutor à 94 px (§7)
// Tryckytorna byggs om från vyn — oberoende av vilka band som just ritats om
static void layoutButtons() {
  nBtns = 0;
  if (ui == UI_CAL || !S.valid || linkLost) return;
  if (ui == UI_MAIN) {
    addBtn(60, 44, 200, 64, BT_POWER);
    addBtn(0, 0, 160, 32, BT_TITLE);
    for (int i = 0; i < 3; i++) addBtn(ACT_X[i], ACT_Y, ACT_W, ACT_H, BT_MODE0 + i, modeName(i));
  } else if (ui == UI_ADJ_SPOT || ui == UI_ADJ_LIST) {
    addBtn(0, 130, 320, 78, BT_SLIDER);
    addBtn(ACT_X[0], ACT_Y, ACT_W, ACT_H, BT_OTHER_GROUP, ui == UI_ADJ_SPOT ? "LISTER" : "SPOTTAR");
    addBtn(ACT_X[1], ACT_Y, ACT_W, ACT_H, BT_SAVE, "SPARA");
    addBtn(ACT_X[2], ACT_Y, ACT_W, ACT_H, BT_CANCEL, "AVBRYT");
  } else if (ui == UI_ERR) {
    addBtn(ACT_X[1], ACT_Y, ACT_W, ACT_H, BT_ACK, "KVITTERA");
  }
}

/* Text som ERSÄTTER sin egen gamla bild, utan att först sudda.
 *
 *  Det här är hela skillnaden mellan en siffra som byts och ett fält som blinkar.
 *  fillRect + drawString skriver varje bildpunkt två gånger: först bakgrund, sedan
 *  text. Mellan de två skrivningarna hinner panelen visa den tomma rutan — några
 *  millisekunder, men ögat ser det som ett blink varje gång texten ändras.
 *  Med setTextPadding ritar TFT_eSPI tecknen med ogenomskinlig bakgrund och fyller
 *  bara resten av fältet efteråt: varje bildpunkt skrivs exakt en gång.
 *
 *  Fältet får därför inte innehålla Å/Ä/Ö — prickarna ritas ovanpå i efterhand av
 *  drawSE() och skulle ge samma dubbelskrivning. Alla fält nedan är siffror och
 *  ren ASCII, och etiketterna (som har prickar) ritas en gång och står still.
 */
static void drawField(const char *ascii, int x, int y, int font, uint8_t datum, int padW) {
  tft.setTextSize(1);
  tft.setTextDatum(datum == 1 ? TC_DATUM : (datum == 2 ? TR_DATUM : TL_DATUM));
  tft.setTextPadding(padW);
  tft.drawString(ascii, x, y, font);
  tft.setTextPadding(0);
}

// Titelns högra del (status) — byts varje sekund under PIR-uppvärmningen och vid
// varje vila/vaken-växling, alltså skriv-en-gång.
static void drawTitleStatus() {
  statusDirty = false;
  if (ui == UI_CAL) return;
  char b[24]; const char *txt = b; uint16_t col;
  if (!S.valid || linkLost) { txt = "INGEN KONTAKT"; col = C_RED; }
  else if (S.err) { snprintf(b, sizeof b, "FEL %u", S.err); col = C_RED; }
  else if (S.warm) { snprintf(b, sizeof b, "PIR %u s", S.warm); col = C_DIM; }
  else { txt = (S.view == VIEW_STANDBY) ? "vila" : "vaken"; col = (S.view == VIEW_STANDBY) ? C_DIM : C_GREEN; }
  tft.setTextColor(col, C_BAND);
  drawField(txt, 312, 8, 2, 2, 118);          // 118 px räcker för "INGEN KONTAKT"
}
static void drawTitle() {
  titleDirty = false;
  if (ui == UI_CAL) return;                        // kalibreringen äger hela skärmen
  tft.fillRect(0, 0, 200, 32, C_BAND);
  tft.setTextColor(C_TEXT, C_BAND);
  if (ui == UI_ADJ_SPOT || ui == UI_ADJ_LIST) {
    drawSE(ui == UI_ADJ_SPOT ? "NIVÅ SPOTTAR" : "NIVÅ LISTER", 8, 4, 4, 0);
  } else {
    tft.setTextColor(C_AMBER, C_BAND);
    drawSE(S.valid ? modeName(S.mode) : "---", 8, 4, 4, 0);
  }
  drawTitleStatus();
}

static void drawSeqBars(int y0, int h) {          // 5 staplar A·B·C·ÖVRE·NEDRE
  static const char *names[5] = { "A", "B", "C", "Ö", "N" };
  int bw = 44, gap = 12, x0 = (320 - (5 * bw + 4 * gap)) / 2;
  for (int i = 0; i < 5; i++) {
    int x = x0 + i * (bw + gap);
    int fh = (h - 16) * lvl[i] / 100;
    tft.fillRect(x, y0, bw, h - 16 - fh, C_BG);
    tft.fillRect(x, y0 + (h - 16 - fh), bw, fh, i < 3 ? C_AMBER : C_AMBERD);
    tft.drawRect(x, y0, bw, h - 16, C_TRACK);
    tft.setTextColor(C_DIM, C_BG);
    drawSE(names[i], x + bw / 2, y0 + h - 15, 2, 1);
  }
}

// Siffran och reglaget i JUSTERA — ritas om separat vid varje $S-eko (ingen helskärmsblink)
static void drawAdjustDynamic() {
  sliderDirty = false;
  uint8_t val = (ui == UI_ADJ_SPOT) ? S.spot : S.list;
  tft.fillRect(80, 40, 120, 56, C_BG);
  tft.setTextColor(C_AMBER, C_BG);
  char b[8]; snprintf(b, sizeof b, "%u", val);
  tft.setTextDatum(TC_DATUM); tft.drawString(b, 150, 44, 6);
  int tx = 24, tw = 272, ty = 160, th = 14;             // reglage 20–100 %
  tft.fillRect(0, 146, 320, 40, C_BG);
  tft.fillRoundRect(tx, ty, tw, th, 4, C_TRACK);
  int kx = tx + (int)((long)(val - 20) * tw / 80);
  if (kx > tx) tft.fillRoundRect(tx, ty, kx - tx, th, 4, C_AMBERD);
  tft.fillRoundRect(kx - 10, ty - 10, 20, th + 20, 6, C_AMBER);
}

// Av/på-rutan. Bär TVÅ saker: överst nuvarande tillstånd, under det vad ett tryck
// gör. Utan andra raden läses ordet "SLÄCKT" som en knapp man trycker för att
// släcka, vilket är raka motsatsen till vad den gör. Färgen bär tillståndet på håll.
static void drawPowerBox() {
  powerDirty = false;
  bool on = S.on;
  uint16_t fill = on ? C_AMBER : C_BOX;
  tft.fillRoundRect(60, 44, 200, 64, 8, fill);
  tft.setTextColor(on ? C_BG : C_TEXT, fill);
  drawSE(on ? "TÄND" : "SLÄCKT", 160, 54, 4, 1);
  tft.setTextColor(on ? C_BG : C_DIM, fill);
  drawSE(on ? "tryck för att släcka" : "tryck för att tända", 160, 84, 2, 1);
}
// Nedräkningens siffra och stapel — det enda som byts en gång per sekund.
// Etiketten "Uppvärmning PIR:" står still och ritas av drawBody(); här byts bara
// siffran (skriv-en-gång, fast fältbredd så att raden inte flyttar sig när 10 blir 9)
// och den bit av stapeln som faktiskt försvunnit sedan förra sekunden.
static int16_t warmNumX = 160, warmNumW = 60, warmBarW = 0;
static void drawWarmDynamic() {
  warmDirty = false;
  char b[12]; snprintf(b, sizeof b, "%u s", S.warm);
  tft.setTextColor(C_TEXT, C_BG);
  drawField(b, warmNumX, 122, 4, 0, warmNumW);
  int w = 278 * S.warm / 60; if (w > 278) w = 278; if (w < 0) w = 0;
  if (w > warmBarW) tft.fillRect(21 + warmBarW, 159, w - warmBarW, 10, C_AMBERD);
  else if (w < warmBarW) tft.fillRect(21 + w, 159, warmBarW - w, 10, C_BG);
  warmBarW = w;
}
// Arbetsytans nedre del (y 116–207) i huvudvyn: sekvensstaplar, nedräkning eller lägestext
static void drawBody() {
  bodyDirty = false; barsDirty = false;
  tft.fillRect(0, 116, 320, 92, C_BG);
  if (pActive() && S.on) { drawSeqBars(116, 90); seqShown = true; return; }
  seqShown = false;
  if (S.warm && S.mode != MODE_MANUAL) {
    // Etikett + sifferfält läggs ut som en rad: etiketten står still, siffran byts.
    tft.setTextSize(1);
    int lw = tft.textWidth("Uppvarmning PIR: ", 4);
    warmNumW = tft.textWidth("60 s", 4) + 4;
    int x0 = (320 - (lw + warmNumW)) / 2; if (x0 < 4) x0 = 4;
    tft.setTextColor(C_TEXT, C_BG);
    drawSE("Uppvärmning PIR:", x0, 122, 4, 0);
    warmNumX = x0 + lw;
    tft.drawRect(20, 158, 280, 12, C_TRACK);
    warmBarW = 0;
    drawWarmDynamic();
    tft.setTextColor(C_DIM, C_BG); drawSE("rörelsedetektorn stabiliseras", 160, 180, 2, 1);
  } else {
    const char *d1, *d2;
    switch (S.mode) {
      case MODE_MANUAL: d1 = "Lyser hela tiden"; d2 = "återgår till KASKAD efter 2 h"; break;
      case MODE_CASCADE: d1 = "Kaskad vid rörelse"; d2 = "tänds i tur och ordning mot garderoben"; break;
      default: d1 = "Allt tänds samtidigt"; d2 = "när rörelsedetektorn ser dig"; break;
    }
    tft.setTextColor(C_TEXT, C_BG); drawSE(d1, 160, 124, 4, 1);
    tft.setTextColor(C_DIM, C_BG); drawSE(d2, 160, 156, 2, 1);
    if (S.err) { tft.setTextColor(C_RED, C_BG); drawSE("FEL kvarstår - se titelraden", 160, 184, 2, 1); }
    else { tft.setTextColor(C_TRACK, C_BG); drawSE("tryck på lägesnamnet: ljusstyrka", 160, 186, 2, 1); }
  }
}

static void drawWork() {
  workDirty = false; barsDirty = false; sliderDirty = false; bodyDirty = false; powerDirty = false; warmDirty = false;
  if (ui == UI_CAL) {
    static const int cx[4] = { 20, 300, 300, 20 }, cy[4] = { 20, 20, 220, 220 };
    tft.fillScreen(C_BG);
    tft.setTextColor(C_TEXT, C_BG);
    drawSE("Tryck mitt i korset", 160, 90, 4, 1);
    char b[24]; snprintf(b, sizeof b, "punkt %u av 4", calStep + 1);
    tft.setTextColor(C_DIM, C_BG); drawSE(b, 160, 130, 2, 1);
    tft.drawLine(cx[calStep] - 12, cy[calStep], cx[calStep] + 12, cy[calStep], C_AMBER);
    tft.drawLine(cx[calStep], cy[calStep] - 12, cx[calStep], cy[calStep] + 12, C_AMBER);
    tft.drawCircle(cx[calStep], cy[calStep], 6, C_AMBER);
    return;
  }
  tft.fillRect(0, 32, 320, 176, C_BG);
  if (!S.valid || linkLost) {
    tft.fillRoundRect(20, 80, 280, 70, 6, C_RED);
    tft.setTextColor(C_TEXT, C_RED);
    drawSE("Ingen kontakt med", 160, 88, 4, 1);
    drawSE("styrenheten", 160, 116, 4, 1);
    tft.setTextColor(C_DIM, C_BG);
    drawSE("Kontrollera nätdel, säkring, panelkabel", 160, 170, 2, 1);
    return;
  }
  if (ui == UI_ERR) {
    tft.fillRoundRect(20, 60, 280, 60, 6, C_RED);
    tft.setTextColor(C_TEXT, C_RED);
    char b[40]; snprintf(b, sizeof b, "FEL %u", S.err); drawSE(b, 160, 76, 4, 1);
    tft.setTextColor(C_TEXT, C_BG);
    if (S.err == 1) { drawSE("PCA9685 svarar inte (I2C 0x40)", 160, 135, 2, 1); drawSE("Se K09-K13, V51 och UNO-konsolen", 160, 160, 2, 1); }
    else if (S.err == 2) { drawSE("EEPROM ogiltig", 160, 135, 2, 1); drawSE("Skriv  defaults  i UNO-konsolen", 160, 160, 2, 1); }
    else drawSE("Okänt fel", 160, 135, 2, 1);
    return;
  }
  if (ui == UI_ADJ_SPOT || ui == UI_ADJ_LIST) {
    tft.setTextColor(C_DIM, C_BG); drawSE("%", 205, 62, 4, 0);
    drawSE(ui == UI_ADJ_SPOT ? "Spottarnas ljusstyrka" : "Listernas ljusstyrka", 160, 122, 2, 1);
    drawSE("20", 24, 184, 2, 0); drawSE("100", 296, 184, 2, 2);
    drawAdjustDynamic();
    return;
  }
  // HUVUDVY / STANDBY / UPPVÄRMNING / SEKVENSSPEGEL
  drawPowerBox();
  drawBody();
}

// En lägesknapp, ritad rakt över sin gamla bild — ingen bandrensning, inget blink
static void drawModeBtn(int i) {
  bool sel = (S.mode == i);
  drawBox(ACT_X[i], ACT_Y, ACT_W, ACT_H, sel ? C_AMBER : C_BOX, sel ? C_BG : C_TEXT, modeName(i));
}
static void drawModeBtns() { modeBtnDirty = false; for (int i = 0; i < 3; i++) drawModeBtn(i); }
static void drawAction() {
  actionDirty = false; modeBtnDirty = false;
  if (ui == UI_CAL) return;
  tft.fillRect(0, 208, 320, 32, C_BAND);
  if (!S.valid || linkLost) return;
  const int *xs = ACT_X, y = ACT_Y, w = ACT_W, h = ACT_H;   // 8 + 94 + 10 + 94 + 10 + 94 + 8 = 320 (§7)
  if (ui == UI_MAIN) {
    for (int i = 0; i < 3; i++) drawModeBtn(i);
  } else if (ui == UI_ADJ_SPOT || ui == UI_ADJ_LIST) {
    drawBox(xs[0], y, w, h, C_BOX, C_TEXT, ui == UI_ADJ_SPOT ? "LISTER" : "SPOTTAR");
    drawBox(xs[1], y, w, h, C_GREEN, C_TEXT, "SPARA");
    drawBox(xs[2], y, w, h, C_BOX, C_TEXT, "AVBRYT");
  } else if (ui == UI_ERR) {
    drawBox(xs[1], y, w, h, C_BOX, C_TEXT, "KVITTERA");
  }
}
// Återställ EN knapp efter tryckblinken (§7) — i stället för att rita om hela vyn
static void drawBtnNormal(uint8_t id) {
  if (!S.valid || linkLost || ui == UI_CAL) return;
  if (id == BT_POWER) { if (ui == UI_MAIN) drawPowerBox(); return; }
  if (id >= BT_MODE0 && id <= BT_MODE2) { if (ui == UI_MAIN) drawModeBtn(id - BT_MODE0); return; }
  for (uint8_t i = 0; i < nBtns; i++) if (btns[i].id == id && btns[i].label)
    drawBox(btns[i].x, btns[i].y, btns[i].w, btns[i].h, id == BT_SAVE ? C_GREEN : C_BOX, C_TEXT, btns[i].label);
}

/* --------------------------------------------------------------------------
 *  6. TOUCH — fyra filterlager (§7) + kalibrering (V37)
 * -------------------------------------------------------------------------- */
static void calLoad() {
  prefs.begin("panel", false);
  cal.valid = prefs.getBool("valid", false);
  cal.xmin = prefs.getShort("xmin", 300); cal.xmax = prefs.getShort("xmax", 3800);
  cal.ymin = prefs.getShort("ymin", 300); cal.ymax = prefs.getShort("ymax", 3800);
  cal.swap = prefs.getUChar("swap", 0);
  cal.zthr = prefs.getUShort("zthr", Z_THRESHOLD_DEFAULT);
  prefs.end();
}
static void calSave() {
  prefs.begin("panel", false);
  prefs.putBool("valid", cal.valid);
  prefs.putShort("xmin", cal.xmin); prefs.putShort("xmax", cal.xmax);
  prefs.putShort("ymin", cal.ymin); prefs.putShort("ymax", cal.ymax);
  prefs.putUChar("swap", cal.swap); prefs.putUShort("zthr", cal.zthr);
  prefs.end();
}
static void rawToScreen(int16_t rx, int16_t ry, int16_t &sx, int16_t &sy) {
  if (cal.swap) { int16_t t = rx; rx = ry; ry = t; }
  if (cal.xmax == cal.xmin || cal.ymax == cal.ymin) { sx = sy = 0; return; }
  long x = (long)(rx - cal.xmin) * 320 / (cal.xmax - cal.xmin);
  long y = (long)(ry - cal.ymin) * 240 / (cal.ymax - cal.ymin);
  if (x < 0) x = 0; if (x > 319) x = 319; if (y < 0) y = 0; if (y > 239) y = 239;
  sx = (int16_t)x; sy = (int16_t)y;
}
static void calFinish() {
  // punkt 0 = (20,20) 1 = (300,20) 2 = (300,220) 3 = (20,220)
  int dxRaw = abs((calRawX[1] + calRawX[2]) / 2 - (calRawX[0] + calRawX[3]) / 2);
  int dyRaw = abs((calRawY[1] + calRawY[2]) / 2 - (calRawY[0] + calRawY[3]) / 2);
  cal.swap = (dyRaw > dxRaw) ? 1 : 0;          // x-axeln ska variera mellan vänster/höger
  int16_t X[4], Y[4];
  for (int i = 0; i < 4; i++) { X[i] = cal.swap ? calRawY[i] : calRawX[i]; Y[i] = cal.swap ? calRawX[i] : calRawY[i]; }
  int xl = (X[0] + X[3]) / 2, xr = (X[1] + X[2]) / 2, yt = (Y[0] + Y[1]) / 2, yb = (Y[2] + Y[3]) / 2;
  // korsen ligger 20 px in från kanterna: extrapolera till 0..320 / 0..240
  cal.xmin = xl - (xr - xl) * 20 / 280;  cal.xmax = xr + (xr - xl) * 20 / 280;
  cal.ymin = yt - (yb - yt) * 20 / 200;  cal.ymax = yb + (yb - yt) * 20 / 200;
  cal.valid = (abs(cal.xmax - cal.xmin) > 500 && abs(cal.ymax - cal.ymin) > 500);
  if (!cal.valid) { cal.xmin = 300; cal.xmax = 3800; cal.ymin = 300; cal.ymax = 3800; cal.swap = 0; Serial.println("[cal] MISSLYCKADES (for litet spann) — kor  cal  igen"); }
  calSave();
  Serial.printf("[cal] swap=%u xmin=%d xmax=%d ymin=%d ymax=%d valid=%u\n", cal.swap, cal.xmin, cal.xmax, cal.ymin, cal.ymax, cal.valid);
  ui = UI_MAIN; titleDirty = workDirty = actionDirty = true;
}
static void calStart() { calStep = 0; ui = UI_CAL; titleDirty = workDirty = actionDirty = true; awakeUntil = millis() + 120000; }

// Touchtillstånd
static struct { int16_t x, y; uint16_t z; uint32_t t; bool pressed; } samp[3]; static uint8_t nSamp = 0;
static bool touchHeld = false;          // en bekräftad tryckning pågår (släpp krävs före nästa)
static uint8_t releaseCount = 0;
static uint8_t heldBtn = BT_NONE;

static uint8_t hitTest(int16_t x, int16_t y) {
  for (uint8_t i = 0; i < nBtns; i++)
    if (x >= btns[i].x && x < btns[i].x + btns[i].w && y >= btns[i].y && y < btns[i].y + btns[i].h) return btns[i].id;
  return BT_NONE;
}
static void flashBtn(uint8_t id) {                // visuell kvittering (§7): bärnsten i 120 ms
  for (uint8_t i = 0; i < nBtns; i++) if (btns[i].id == id && id != BT_SLIDER && id != BT_TITLE) {
    if (btns[i].label) drawBox(btns[i].x, btns[i].y, btns[i].w, btns[i].h, C_AMBER, C_BG, btns[i].label);
    else tft.fillRoundRect(btns[i].x, btns[i].y, btns[i].w, btns[i].h, 4, C_AMBER);
    flashUntil = millis() + FLASH_MS; flashId = id; flashUi = ui;
  }
}
static void enterAdjust(uint8_t grp, bool fresh) {   // fresh = från huvudvyn: minns de sparade nivåerna för AVBRYT
  adjGroup = grp; if (fresh) { savedSpot = S.spot; savedList = S.list; } adjEnteredAt = millis();
  ui = grp ? UI_ADJ_LIST : UI_ADJ_SPOT;
  titleDirty = workDirty = actionDirty = true;
  // Skicka nuvarande nivå direkt: det sätter UNO:n i JUSTERA-läge och tänder den
  // grupp som ska ställas in, så att man ser ljuset innan man rört reglaget.
  sendB(grp, grp ? S.list : S.spot);
}
static void leaveAdjust() { ui = UI_MAIN; titleDirty = workDirty = actionDirty = true; }
static void sliderTo(int16_t sx) {
  int v = 20 + (int)((long)(sx - 24) * 80 / 272);
  if (v < 20) v = 20; if (v > 100) v = 100;
  bPendingVal = (uint8_t)v; bPending = true;
}
static void onPress(int16_t x, int16_t y, uint16_t z) {
  confirmedTouches++;
  Serial.printf("[%lu] touch x=%d y=%d z=%u\n", (unsigned long)millis(), x, y, z);
  uint32_t now = millis();
  if (ui == UI_CAL) return;                        // hanteras i calService
  if (!awake()) {                                  // lager 4 (Ä4): väck bara
    awakeUntil = now + LOCAL_WAKE_MS; lockoutUntil = now + WAKE_LOCKOUT_MS;
#if ACCENT_STRIP
    accTouchAt = now;                              // väckningen kvitteras också
#endif
    return;
  }
  awakeUntil = now + LOCAL_WAKE_MS;
  if ((int32_t)(now - lockoutUntil) < 0) return;
  if (!S.valid || linkLost) return;
  uint8_t id = hitTest(x, y);
  heldBtn = id;
#if ACCENT_STRIP
  // Kvitteringspuls på listen — bara för tryck som TRÄFFAR en knapp, och aldrig
  // för reglaget. Ett drag med fingret tappar trycket då och då på en resistiv
  // panel; varje återtag räknas som ett nytt tryck, och listen blixtrade då till
  // full styrka mitt i mätarbilden (även när fingret hamnade strax utanför
  // reglagets ruta och träffen blev "ingen knapp").
  if (id != BT_NONE && id != BT_SLIDER) accTouchAt = now;
#endif
  switch (id) {
    case BT_MODE0: case BT_MODE1: case BT_MODE2:
      flashBtn(id); request('M', id - BT_MODE0); break;
    case BT_POWER:
      flashBtn(id); request('E', S.on ? 0 : 1); break;
    case BT_TITLE:
      if (ui == UI_MAIN && !S.err) enterAdjust(0, true); break;
    case BT_OTHER_GROUP:
      flashBtn(id); enterAdjust(adjGroup ? 0 : 1, false); break;
    case BT_SAVE:
      flashBtn(id); request('W', 0); leaveAdjust(); break;
    case BT_CANCEL:                                // återställ nivåerna, avsluta JUSTERA i UNO:n med $W
      flashBtn(id);
      if (S.spot != savedSpot) { sendB(0, savedSpot); delay(20); }
      if (S.list != savedList) { sendB(1, savedList); delay(20); }
      request('W', 0); leaveAdjust(); break;
    case BT_ACK:
      flashBtn(id); errAckedCode = S.err; ui = UI_MAIN; titleDirty = workDirty = actionDirty = true; break;
    case BT_SLIDER:
      sliderTo(x); break;
    default: break;
  }
}
static void calService(bool pressed, int16_t rx, int16_t ry) {
  static bool wasPressed = false; static uint8_t stable = 0; static int32_t ax = 0, ay = 0;
  if (pressed) {
    if (!wasPressed) { stable = 0; ax = ay = 0; }
    ax += rx; ay += ry; stable++;
    wasPressed = true;
  } else if (wasPressed) {
    wasPressed = false;
    if (stable >= 5) {
      calRawX[calStep] = ax / stable; calRawY[calStep] = ay / stable;
      Serial.printf("[cal] punkt %u raw=(%d,%d)\n", calStep, calRawX[calStep], calRawY[calStep]);
      if (++calStep >= 4) calFinish(); else workDirty = true;
    }
  }
}
static void touchService() {
  static uint32_t lastSample = 0;
  uint32_t now = millis();
  if (now - lastSample < TOUCH_SAMPLE_MS) return;
  lastSample = now;
  TS_Point p = ts.getPoint();
  bool pressed = p.z > cal.zthr;                   // lager 1: trycktröskel
  if (ui == UI_CAL) { calService(pressed, p.x, p.y); return; }
  int16_t sx = 0, sy = 0;
  if (pressed) rawToScreen(p.x, p.y, sx, sy);
  // skjut in i fönstret om tre
  if (nSamp < 3) nSamp++;
  samp[2] = samp[1]; samp[1] = samp[0];
  samp[0] = { sx, sy, (uint16_t)p.z, now, pressed };
  if (!pressed) {
    if (releaseCount < 255) releaseCount++;
    if (releaseCount >= 2 && touchHeld) { touchHeld = false; heldBtn = BT_NONE; }   // lager 3: släpp före nästa
    if (releaseCount == 1 && samp[1].pressed && !samp[2].pressed) ghostSamples++;   // ensam avläsning = spök
    return;
  }
  releaseCount = 0;
  if (touchHeld) {                                  // pågående tryck: bara reglaget följer fingret
    if (heldBtn == BT_SLIDER && (ui == UI_ADJ_SPOT || ui == UI_ADJ_LIST)) sliderTo(sx);
    return;
  }
  if (nSamp < 3 || !samp[1].pressed || !samp[2].pressed) return;
  // lager 2: samstämmighet — tre avläsningar inom 25 px av varandra inom 60 ms
  if (samp[0].t - samp[2].t > 60 + TOUCH_SAMPLE_MS) return;
  for (int i = 0; i < 3; i++) for (int j = i + 1; j < 3; j++)
    if (abs(samp[i].x - samp[j].x) > TOUCH_CONSIST_PX || abs(samp[i].y - samp[j].y) > TOUCH_CONSIST_PX) return;
  touchHeld = true;
  onPress((samp[0].x + samp[1].x + samp[2].x) / 3, (samp[0].y + samp[1].y + samp[2].y) / 3, samp[0].z);
}

/* --------------------------------------------------------------------------
 *  7. BAKGRUNDSLJUS OCH ACCENTLIST
 * -------------------------------------------------------------------------- */
static void blService() {
  static uint32_t last = 0; uint32_t now = millis();
  if (now - last < 10) return; float dt = (now - last); last = now;
  float target;
  if (pActive() && S.valid && S.on && !locallyAwake()) {
    // Rummets ljus styr skärmen (§7) — men bara när ingen står vid panelen, och
    // aldrig nedåt under en tändning (ljuset ovanför är redan på väg upp).
    uint8_t m = 0; for (int i = 0; i < 5; i++) if (lvl[i] > m) m = lvl[i];
    target = BL_IDLE + (BL_CAP - BL_IDLE) * (m / 100.0f);
    if (seqDir > 0 && target < blCur) target = blCur;
    blCur = target;                                 // $P är redan mjuk (20 Hz)
  } else {
    target = awake() ? BL_CAP : BL_IDLE;
    float rate = (BL_CAP - BL_IDLE) / (target > blCur ? (float)BL_UP_MS : (float)BL_DOWN_MS);
    if (blCur < target) { blCur += rate * dt; if (blCur > target) blCur = target; }
    else if (blCur > target) { blCur -= rate * dt; if (blCur < target) blCur = target; }
  }
  blWrite((uint8_t)(blCur + 0.5f));
}

#if ACCENT_STRIP
#include "esp32-hal-rmt.h"
/* Utmatningen håller EN storhet per pixel: den linjära ljusstyrkan accY (0–255).
 * Färgen (bärnsten, eller rött i felläget) räknas fram först när ramen skickas.
 *
 *  Det låter petigt men är hela orsaken till de gula, gröna och röda pixlarna i
 *  slutet av släckanimationen: tidigare låg R, G och B var för sig i minnet och
 *  skalades var för sig med heltalsdivision. Vid Y = 1 blev bärnsten (1,1,0) =
 *  GULT, och strömtaket kunde sedan nolla R och lämna (0,1,0) = GRÖNT. Med Y som
 *  enda storhet kan färgtonen inte längre gå isär — allt skalas före omvandlingen.
 */
static uint8_t accY[ACCENT_N];          // linjär ljusstyrka per pixel, kedjeindex
static bool    accFault = false;        // felläget lyser rött i stället för bärnsten
/* Linjär ljusstyrka → färg. Bärnsten är (255,140,20); förhållandet hålls hela
 * vägen ned, och under Y = 3 går det inte att hålla alls (G skulle bli 0 eller 1
 * och pixeln röd eller gul) — där lyfts nivån till 3, vilket är den svagaste
 * punkt där bärnsten fortfarande ÄR bärnsten. R ≥ G ≥ B garanteras alltid. */
static void accColour(uint8_t Y, uint8_t *r, uint8_t *g, uint8_t *b) {
  if (!Y) { *r = *g = *b = 0; return; }
  if (accFault) { *r = Y; *g = 0; *b = 0; return; }
  if (Y < 3) Y = 3;
  uint8_t R = Y;
  uint8_t G = (uint8_t)(((uint16_t)Y * 140 + 127) / 255); if (!G) G = 1; if (G > R) G = R;
  uint8_t B = (uint8_t)(((uint16_t)Y *  20 + 127) / 255); if (B > G) B = G;
  *r = R; *g = G; *b = B;
}
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static rmt_data_t accSym[ACCENT_N * 24];
// 10 MHz tick = 0,1 µs. FYRA minnesblock = 256 symboler: hela ramen (8×24 = 192)
// ryms i kanalminnet och drivrutinen behöver aldrig fylla på under sändning.
// Med två block (128) fylldes minnet på i avbrott mitt i ramen, och ett fördröjt
// avbrott (displayens SPI, touchen) gav fel färg på de sista pixlarna.
static void accentInit() {
  if (!rmtInit(ACCENT_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_4, 10000000))
    Serial.println("[list] VARNING: RMT kunde inte initieras pa GPIO27");
  else Serial.printf("[list] accentlist: %u px pa GPIO%u, DIN %s, vagen rullar %s\n",
                     ACCENT_N, ACCENT_PIN, ACCENT_DIN_AT_TOP ? "uppe" : "nere",
                     ACCENT_CASCADE_DOWN ? "nedat" : "uppat");
}
static void accentShow() {
  int i = 0;
  for (int px = 0; px < ACCENT_N; px++) {
    uint8_t r, g, b; accColour(accY[px], &r, &g, &b);
#if ACCENT_GRB
    uint8_t c[3] = { g, r, b };                        // WS2812B = GRB
#else
    uint8_t c[3] = { r, g, b };                        // RGB-varianter
#endif
    for (int k = 0; k < 3; k++) for (int b = 7; b >= 0; b--) {
      bool one = c[k] & (1 << b);
      accSym[i].level0 = 1; accSym[i].duration0 = one ? 8 : 4;
      accSym[i].level1 = 0; accSym[i].duration1 = one ? 4 : 8; i++;
    }
  }
  rmtWrite(ACCENT_PIN, accSym, ACCENT_N * 24, 10);
}
#else
static rmt_obj_t *accRmt = NULL; static rmt_data_t accSym[ACCENT_N * 24];
static void accentInit() { accRmt = rmtInit(ACCENT_PIN, RMT_TX_MODE, RMT_MEM_256); if (accRmt) rmtSetTick(accRmt, 100); }
static void accentShow() {
  if (!accRmt) return; int i = 0;
  for (int px = 0; px < ACCENT_N; px++) {
    uint8_t r, g, b; accColour(accY[px], &r, &g, &b);
#if ACCENT_GRB
    uint8_t c[3] = { g, r, b };
#else
    uint8_t c[3] = { r, g, b };
#endif
    for (int k = 0; k < 3; k++) for (int b = 7; b >= 0; b--) {
      bool one = c[k] & (1 << b);
      accSym[i].level0 = 1; accSym[i].duration0 = one ? 8 : 4;
      accSym[i].level1 = 0; accSym[i].duration1 = one ? 4 : 8; i++;
    }
  }
  rmtWrite(accRmt, accSym, ACCENT_N * 24);
}
#endif
/* --- Tre koordinatsystem, så att monteringen bara beskrivs på ett ställe ---
 *  KEDJA   0..N-1 = ordningen data skickas i, 0 = DIN-änden.
 *  TOPP    0..N-1 = fysiskt uppifrån och ned, 0 = översta pixeln.
 *  VÅG     0..N-1 = kaskadens ordning, 0 = den grupp som tänds först (Spot A).
 *  Animatorn räknar i TOPP; vågen används bara för att hitta pixelns ljusgrupp.
 * ------------------------------------------------------------------------ */
static inline int chainFromTop(int t) { return ACCENT_DIN_AT_TOP ? t : (ACCENT_N - 1 - t); }
static inline int waveFromTop(int t)  { return ACCENT_CASCADE_DOWN ? t : (ACCENT_N - 1 - t); }

// CIE 1931 fram och tillbaka i flyttal (0..1). Lamporna körs på samma kurva i
// UNO:n; därför tonar listen i takt med rummet när båda räknar i upplevd nivå.
static float cieLin(float L) {            // upplevd → linjär
  if (L <= 0) return 0; if (L >= 1) return 1;
  return (L <= 0.08f) ? L / 9.033f : powf((L + 0.16f) / 1.16f, 3.0f);
}
static float cieInv(float Y) {            // linjär → upplevd
  if (Y <= 0) return 0; if (Y >= 1) return 1;
  return (Y <= 0.008856f) ? Y * 9.033f : 1.16f * powf(Y, 1.0f / 3.0f) - 0.16f;
}
// UPPLEVD nivå 0..1 → linjär ljusstyrka för pixel t räknat uppifrån
static void setTopL(int t, float L) {
  if (t < 0 || t >= ACCENT_N) return;
  int Y = (int)(cieLin(L) * 255.0f + 0.5f);
  if (Y < 0) Y = 0; if (Y > 255) Y = 255;
  accY[chainFromTop(t)] = (uint8_t)Y;
}
static void accentClear() { for (int i = 0; i < ACCENT_N; i++) accY[i] = 0; }

// V33/V47: taket är ett mjukvaruvillkor, inte en förhoppning. ~20 mA per färg-
// kanal vid full styrka plus ~1 mA vila per pixel; överskrids taket skalas allt ned.
// Taket skalar LJUSSTYRKAN, aldrig färgkanalerna var för sig — annars flyttar
// heltalsdivisionen färgtonen (se accColour). Bärnsten drar ~33 mA per pixel vid
// full styrka, rött ~20 mA; formeln räknar på bärnsten och är alltså försiktig.
static void accentCap() {
  uint32_t mA = ACCENT_N;                                   // ~1 mA vila per pixel
  for (int i = 0; i < ACCENT_N; i++) mA += (uint32_t)accY[i] * 13 / 100;
  if (mA <= ACCENT_MAX_MA) return;
  uint32_t k = (uint32_t)(ACCENT_MAX_MA - ACCENT_N) * 256 / (mA - ACCENT_N);
  for (int i = 0; i < ACCENT_N; i++) accY[i] = (uint8_t)((uint32_t)accY[i] * k / 256);
}

// Testlägen från konsolen: 0 = av (normal drift), 1 = pixelvandring, 2 = allt tänt
static uint8_t accTest = 0, accTestPx = 0;
static uint32_t accTestNext = 0;

/* --- ANIMATORN ------------------------------------------------------------
 *  Körs i 50 Hz. Varje tick räknas ett MÅL per pixel (upplevd nivå 0..1) ur
 *  systemtillståndet, och pixelns nuvarande nivå glider mot målet med en
 *  hastighet som beror på situationen: snabbt när den följer en pågående
 *  sekvens, långsamt (ease-out ~0,7 s) vid övergången till vilonivån. Så blir
 *  varje övergång sömlös — det finns ingen väg mellan två lägen som hoppar.
 *
 *  Lägen (i prioritetsordning):
 *   FEL           långsam röd puls (§5B). Ingen kontakt räknas som fel.
 *   NIVÅ-vy       mätare som fylls nedifrån, glidande, med bråkdelspixel överst.
 *   KASKAD upp    regndroppe: ljust huvud rullar uppifrån och ned i takt med att
 *                 grupperna tänds (A överst → stripsen nederst), ljusspår bakom,
 *                 området bakom droppen ligger på sekvensnivån.
 *   KASKAD ned    samma droppe baklänges: nedifrån och upp, mörkt bakom.
 *   SAMTIDIGT/    hela listen följer lampornas medelnivå upp eller ned.
 *   MANUELL
 *   vila, tänt    vilonivån (lägre än sekvensen) skalad med lampornas nivå.
 *   vila, släckt  svag glöd om systemet är TÄNT, helt mörk om SLÄCKT.
 *  Tryckpulsen läggs ovanpå allt utom FEL: snabb upp, mjuk exponentiell återgång
 *  till exakt den nivå som gällde.
 * ------------------------------------------------------------------------ */
static float accCur[ACCENT_N];            // nuvarande upplevd nivå per pixel (TOPP)
static float accHead  = -1.0f;            // droppens filtrerade läge, <0 = ingen droppe
static float accGauge = 0.0f;             // mätarens filtrerade fyllnad, 0..N

/* Ljusgrupp per pixel, i vågens ordning (den ordning rummet tänds i):
 *      px 1–2  Spot A (bortre)      \
 *      px 3–4  Spot B (mitt)         |  samma ordning som ljuset i rummet
 *      px 5–6  Spot C (närmast)      |
 *      px 7    Strip ÖVRE            |
 *      px 8    Strip NEDRE          /
 *  Stripsen har fått var sin pixel nu när de också tänds efter varandra i
 *  kaskaden (UNO:ns CASCADE_ONSET 450 / 700 ms). Droppen får därmed åtta steg
 *  och når botten precis när den nedersta listen är uppe. */
static const uint8_t ACC_GRP[ACCENT_N] = { 0, 0, 1, 1, 2, 2, 3, 4 };

static float pct01(int pct) { return pct / 100.0f; }
// Gruppernas upplevda nivå relativt sitt eget tak (0..1): A, B, C, ÖVRE, NEDRE
static void groupLevels(float g[5]) {
  float capS = S.spot >= 20 ? pct01(S.spot) : 1.0f;
  float capL = S.list >= 20 ? pct01(S.list) : 1.0f;
  for (int i = 0; i < 5; i++) {
    float f = pct01(lvl[i]) / (i < 3 ? capS : capL);
    g[i] = cieInv(f > 1 ? 1 : f);
  }
}
static inline float gOfTop(const float g[5], int t) { return g[ACC_GRP[waveFromTop(t)]]; }
static inline float bell(float d, float sigma) { return expf(-0.5f * (d / sigma) * (d / sigma)); }
static inline float fmaxf3(float a, float b, float c) { float m = a > b ? a : b; return m > c ? m : c; }

static void accentService() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if ((int32_t)(now - last) < 20) return;      // 50 Hz
  last = now;
  accentClear();

  if (accTest == 1) {                          // vandrande pixel — räkna och se riktning
    if ((int32_t)(now - accTestNext) >= 0) {
      accTestNext = now + 700;
      accTestPx = (uint8_t)((accTestPx + 1) % ACCENT_N);
      Serial.printf("[list] pixel %u uppifran raknat\n", accTestPx + 1);
    }
    setTopL(accTestPx, 0.7f);
    accentCap(); accentShow(); return;
  }
  if (accTest == 2) {                          // allt tänt, för strömmätning (V47)
    for (int t = 0; t < ACCENT_N; t++) setTopL(t, 1.0f);
    accentCap(); accentShow(); return;
  }

  // FEL — långsam röd puls (§5B). Utebliven kontakt med styrenheten räknas som fel:
  // panelen ska synas vara ur funktion tvärs över rummet, inte bara på skärmen.
  accFault = (!S.valid || linkLost || S.err);
  if (accFault) {
    float p = 0.45f + 0.45f * sinf(now / 1000.0f * 3.1416f);
    for (int t = 0; t < ACCENT_N; t++) { setTopL(t, p); accCur[t] = 0; }
    accHead = -1.0f;
    accentCap(); accentShow(); return;
  }

  float g[5]; groupLevels(g);
  // Huvudets läge i pixlar = summan av gruppernas nivå över de åtta lägena. Samma
  // tal åt båda hållen: vid tändning växer det 0 → 8 (droppen går ned), vid
  // släckning krymper det 8 → 0 (fronten går upp), eftersom grupperna slocknar
  // nedifrån. meanG är samma summa per pixel.
  float sumG = 0; for (int w = 0; w < ACCENT_N; w++) sumG += g[ACC_GRP[w]];
  float meanG = sumG / ACCENT_N;
  bool  lit = lvl[0] || lvl[1] || lvl[2] || lvl[3] || lvl[4];
  bool  stream = pStream();
  float floorL = S.on ? pct01(ACCENT_IDLE_PCT) : 0.0f;   // SLÄCKT = mörk, TÄNT = glöd
  const float REST = pct01(ACCENT_REST_PCT), SEQ = pct01(ACCENT_SEQ_PCT), HEAD = pct01(ACCENT_HEAD_PCT);
  float target[ACCENT_N];
  float alpha;                                  // andel av avståndet till målet per tick

  if (ui == UI_ADJ_SPOT || ui == UI_ADJ_LIST) {
    // MÄTARE: fylls nedifrån i takt med reglaget (20–100 % → 0–8 px), glider.
    uint8_t pct = (ui == UI_ADJ_LIST) ? S.list : S.spot;
    float want = ((float)pct - 20.0f) * ACCENT_N / 80.0f; if (want < 0) want = 0; if (want > ACCENT_N) want = ACCENT_N;
    accGauge += (want - accGauge) * 0.25f;
    for (int t = 0; t < ACCENT_N; t++) {
      float fill = accGauge - (float)(ACCENT_N - 1 - t); if (fill < 0) fill = 0; if (fill > 1) fill = 1;
      target[t] = pct01(ACCENT_TRACK_PCT) + (pct01(ACCENT_GAUGE_PCT) - pct01(ACCENT_TRACK_PCT)) * fill;
    }
    alpha = 0.5f; accHead = -1.0f;
  }
  else if (stream && S.mode == MODE_CASCADE && seqDir > 0 && pStartedDark) {
    // REGNDROPPE NEDÅT. Huvudet står där tändningen kommit: A lyfter droppen från
    // toppen, spottarna för den ned till px 6, och de två listerna (som numera
    // tänds efter varandra) för den sista biten ned till botten.
    accHead = (accHead < 0) ? sumG : accHead + (sumG - accHead) * 0.5f;
    for (int t = 0; t < ACCENT_N; t++) {
      float base = gOfTop(g, t) * SEQ;                        // tänd yta bakom droppen
      float d = accHead - (float)t;                           // >0: droppen har passerat
      float head = HEAD * bell(d, ACCENT_HEAD_SIGMA);
      float tail = (d > 0) ? HEAD * expf(-d / ACCENT_TAIL_PX) : 0;
      target[t] = fmaxf3(base, head, tail);
      if (target[t] < floorL) target[t] = floorL;
    }
    alpha = 0.45f;
  }
  else if (stream && S.mode == MODE_CASCADE && seqDir < 0) {
    // DROPPEN BAKLÄNGES: släckfronten går nedifrån (stripsen) och upp (A),
    // ljus framkant, mörkt bakom — samma ordning som lamporna slocknar i.
    accHead = (accHead < 0) ? sumG : accHead + (sumG - accHead) * 0.5f;
    for (int t = 0; t < ACCENT_N; t++) {
      float base = gOfTop(g, t) * REST;                       // ännu tända grupper ligger kvar på vilonivån
      float d = (float)t - accHead;                           // >0: under huvudet = redan släckt
      float head = 0.7f * bell(d, ACCENT_HEAD_SIGMA);
      float tail = (d > 0) ? 0.7f * expf(-d / ACCENT_TAIL_PX) : 0;
      target[t] = fmaxf3(base, head, tail);
      if (target[t] < floorL) target[t] = floorL;
    }
    alpha = 0.45f;
  }
  else if (stream) {
    // SAMTIDIGT / MANUELL (och nivåomställningar utanför NIVÅ-vyn): hela listen
    // följer lampornas medelnivå — upp mot sekvensnivån, ned mot mörkt.
    float L = meanG * (seqDir >= 0 ? SEQ : REST);
    if (L < floorL) L = floorL;
    for (int t = 0; t < ACCENT_N; t++) target[t] = L;
    alpha = 0.45f; accHead = -1.0f;
  }
  else {
    // VILA: tänt rum → vilonivån skalad med lampornas nivå; släckt rum → glöd/mörkt.
    float L = lit ? meanG * REST : 0.0f;
    if (L < floorL) L = floorL;
    for (int t = 0; t < ACCENT_N; t++) target[t] = L;
    alpha = 1.0f - expf(-20.0f / (ACCENT_REST_MS / 3.0f));   // ease-out, ~ACCENT_REST_MS till vila
    accHead = -1.0f;
  }

  for (int t = 0; t < ACCENT_N; t++) accCur[t] += (target[t] - accCur[t]) * alpha;

  // TRYCKPULS (§7): snabb upp, mjuk exponentiell återgång till exakt föregående nivå.
  float pulse = 0.0f;
  if (accTouchAt) {
    uint32_t dt = now - accTouchAt;
    if (dt < ACCENT_TOUCH_UP_MS) pulse = pct01(ACCENT_TOUCH_PCT) * dt / (float)ACCENT_TOUCH_UP_MS;
    else if (dt < ACCENT_TOUCH_UP_MS + 4UL * ACCENT_TOUCH_DOWN_MS)
      pulse = pct01(ACCENT_TOUCH_PCT) * expf(-3.0f * (float)(dt - ACCENT_TOUCH_UP_MS) / ACCENT_TOUCH_DOWN_MS);
    else accTouchAt = 0;
  }
  for (int t = 0; t < ACCENT_N; t++) {
    float L = accCur[t] + pulse * (1.0f - accCur[t]);
    setTopL(t, L);
  }
  accentCap();
  accentShow();
}
#endif

/* --------------------------------------------------------------------------
 *  8. KONSOLEN (micro-USB)
 * -------------------------------------------------------------------------- */
static void printHelp() {
  Serial.println("# KOMMANDON: cal (touchkalibrering, V37) · z <n> (trycktroskel) · stat · help");
  Serial.println("#            on | off | mode 0|1|2 | spot <20-100> | list <20-100> | save  — testar upplanken");
  Serial.println("#            bl <0-255> (tvinga bakgrundsljus 5 s, V36)");
#if ACCENT_STRIP
  Serial.println("#            led  = vandrande pixel (racka och se riktning, V46)");
  Serial.println("#            leda = alla pixlar tanda (matstrom och VDD, V47)");
  Serial.println("#            ledo = tillbaka till normal drift");
#endif
}
static uint32_t blForceUntil = 0; static uint8_t blForceVal = 0;
static void handleCmd(char *line) {
  char *cmd = strtok(line, " "); if (!cmd) return;
  char *a1 = strtok(NULL, " ");
  if (!strcmp(cmd, "help")) printHelp();
  else if (!strcmp(cmd, "cal")) calStart();
  else if (!strcmp(cmd, "z") && a1) { cal.zthr = atoi(a1); calSave(); Serial.printf("# zthr=%u\n", cal.zthr); }
  else if (!strcmp(cmd, "stat")) {
    Serial.printf("# FW %s | link=%s S:mode=%u warm=%u spot=%u list=%u view=%u err=%u on=%u | frames S=%lu P=%lu bad=%lu\n",
      FW_VERSION, linkLost ? "NEJ" : "ok", S.mode, S.warm, S.spot, S.list, S.view, S.err, S.on,
      (unsigned long)framesS, (unsigned long)framesP, (unsigned long)framesBad);
    Serial.printf("# touch: bekraftade=%lu ensamma(spok)=%lu zthr=%u cal=%s swap=%u [%d..%d]x[%d..%d] | bl=%u | heap=%u\n",
      (unsigned long)confirmedTouches, (unsigned long)ghostSamples, cal.zthr, cal.valid ? "ok" : "SAKNAS", cal.swap,
      cal.xmin, cal.xmax, cal.ymin, cal.ymax, (unsigned)(blCur + 0.5f), (unsigned)ESP.getFreeHeap());
  }
  else if (!strcmp(cmd, "on")) request('E', 1);
  else if (!strcmp(cmd, "off")) request('E', 0);
  else if (!strcmp(cmd, "mode") && a1) request('M', atoi(a1) & 3);
  else if (!strcmp(cmd, "spot") && a1) sendB(0, atoi(a1));
  else if (!strcmp(cmd, "list") && a1) sendB(1, atoi(a1));
  else if (!strcmp(cmd, "save")) request('W', 0);
  else if (!strcmp(cmd, "bl") && a1) { blForceVal = atoi(a1); blForceUntil = millis() + 5000; blWrite(blForceVal); Serial.printf("# bl=%u i 5 s\n", blForceVal); }
#if ACCENT_STRIP
  else if (!strcmp(cmd, "led"))  { accTest = 1; accTestPx = 0; accTestNext = 0;
    Serial.printf("# listtest: en pixel i taget, 0,7 s var. Pixel 1 ska vara den %s.\n",
                  ACCENT_DIN_AT_TOP ? "OVERSTA" : "NEDERSTA");
    Serial.println("# Racka pixlarna: ska bli 8. Gar vandringen at fel hall — vand ACCENT_DIN_AT_TOP."); }
  else if (!strcmp(cmd, "leda")) { accTest = 2; Serial.println("# listtest: alla pixlar tanda (taket 150 mA gallar). Mat VDD vid listen (V47: 4,0-4,4 V)."); }
  else if (!strcmp(cmd, "ledo")) { accTest = 0; Serial.println("# listtest av — normal drift"); }
#endif
  else Serial.println("# okant — skriv help");
}
static char cmdBuf[40]; static uint8_t cmdLen = 0;
static void pollConsole() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (cmdLen) { cmdBuf[cmdLen] = 0; handleCmd(cmdBuf); } cmdLen = 0; }
    else if (cmdLen < sizeof cmdBuf - 1) cmdBuf[cmdLen++] = c;
  }
}

/* --------------------------------------------------------------------------
 *  9. SETUP OCH LOOP
 * -------------------------------------------------------------------------- */
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.printf("\n# GARDEROBSBELYSNING PANEL %s\n", FW_VERSION);
#if defined(ST7789_DRIVER)
  const char *drv = "ST7789";
#else
  const char *drv = "ILI9341";
#endif
  Serial.printf("# displaydrivrutin: %s  (byts i tft_setup.h: PANEL_DRIVER_ST7789 0/1)\n", drv);
  // Radion: WiFi-stacken länkas inte in alls (radion startas aldrig) — Bluetooth stängs uttryckligen.
  btStop();
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH); digitalWrite(LED_G, HIGH); digitalWrite(LED_B, HIGH);

  tft.init();
  tft.setRotation(1);                            // 320×240 liggande
  blInit(); blWrite(BL_IDLE);
  tft.fillScreen(C_BG);
  tft.setTextColor(C_AMBER, C_BG); drawSE("GARDEROBSBELYSNING", 160, 70, 4, 1);
  tft.setTextColor(C_TEXT, C_BG); drawSE(FW_VERSION, 160, 110, 4, 1);
  tft.setTextColor(C_DIM, C_BG);
  { char sb[48]; snprintf(sb, sizeof sb, "drivrutin %s - vantar pa styrenheten", drv); drawSE(sb, 160, 150, 2, 1); }
  tft.fillRect(20, 190, 120, 20, C_RED); tft.fillRect(140, 190, 60, 20, C_GREEN); tft.fillRect(200, 190, 100, 20, 0x001F);
  tft.setTextColor(C_TEXT, C_BG); drawSE("röd          grön          blå  (V34)", 160, 214, 2, 1);

  tsSpi.begin(XPT_CLK, XPT_MISO, XPT_MOSI, XPT_CS);
  ts.begin(tsSpi);
  ts.setRotation(1);
  calLoad();

  Serial1.begin(115200, SERIAL_8N1, 35, -1);     // nedlänk, RX only
  Serial2.begin(9600, SERIAL_8N1, -1, 22, UPLINK_TX_INVERT);   // upplänk, TX only
  seqNo = (uint16_t)(esp_random() % 60000) + 1;

#if ACCENT_STRIP
  accentInit();
#endif
  // Splash ~2 s medan BL tonar upp
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) { pollDownlink(); blCur = BL_IDLE + (BL_CAP - BL_IDLE) * (millis() - t0) / 2000.0f; blWrite((uint8_t)blCur); delay(10); }
  awakeUntil = millis() + LOCAL_WAKE_MS;
  tft.fillScreen(C_BG);
  Serial.printf("# cal %s (zthr %u). Skriv  cal  for touchkalibrering, help for kommandon\n", cal.valid ? "laddad" : "SAKNAS — kalibrera med 'cal'", cal.zthr);
  if (!cal.valid) calStart();
  titleDirty = workDirty = actionDirty = true;
}

void loop() {
  uint32_t now = millis();
  pollDownlink();
  pollConsole();
  reqService();
  touchService();

  // $B: skicka senaste reglagevärdet, max 10/s
  if (bPending && (now - lastBAt) >= B_MIN_INTERVAL_MS) { bPending = false; sendB(adjGroup, bPendingVal); }

  // Länkvakt (>2 s utan giltig $S). Signerad differens och FÄRSK millis(): S.at
  // kan ha satts i pollDownlink() ovan, alltså EFTER att 'now' lästes. Med den
  // gamla osignerade "now − S.at" blev det då ett jättetal → "Ingen kontakt" i
  // ett varv → helomritning två gånger. Det var flimret vid varje knapptryck.
  bool lost = !S.valid || (int32_t)(millis() - S.at) > (int32_t)LINK_TIMEOUT_MS;
  if (lost != linkLost) { linkLost = lost; titleDirty = workDirty = actionDirty = true; if (lost) Serial.println("[link] ingen kontakt med styrenheten"); else Serial.println("[link] kontakt"); }

  // fel → felvyn (tills kvitterad); försvinner felet lämnas vyn
  if (S.valid && !linkLost) {
    if (S.err && S.err != errAckedCode && ui != UI_ERR && ui != UI_CAL) { ui = UI_ERR; titleDirty = workDirty = actionDirty = true; }
    if (!S.err && ui == UI_ERR) { ui = UI_MAIN; titleDirty = workDirty = actionDirty = true; }
  }
  // JUSTERA-timeout lokalt (UNO:n har samma 20 s) och ingen lokal justering när skärmen somnat
  if ((ui == UI_ADJ_SPOT || ui == UI_ADJ_LIST) && (int32_t)(millis() - adjEnteredAt) > 20000 && !touchHeld && (int32_t)(millis() - lastBAt) > 20000) leaveAdjust();

  // sekvensspegeln: rita om arbetsytans nedre del när $P börjar/slutar
  bool pa = pActive();
  if (ui == UI_MAIN && S.valid && !linkLost && ((pa && S.on) != seqShown)) bodyDirty = true;

  // tryckblinkens slut: bara den knappen ritas tillbaka
  if (flashUntil && (int32_t)(now - flashUntil) >= 0) { flashUntil = 0; if (ui == flashUi) drawBtnNormal(flashId); }

  if (titleDirty) drawTitle(); else if (statusDirty) drawTitleStatus();
  if (workDirty) drawWork();
  else if (ui == UI_MAIN && S.valid && !linkLost) {
    if (powerDirty) drawPowerBox();
    if (bodyDirty) drawBody();
    else if (barsDirty && seqShown) { barsDirty = false; drawSeqBars(116, 90); }
    else if (warmDirty && !seqShown && S.warm && S.mode != MODE_MANUAL) drawWarmDynamic();
  }
  else if (sliderDirty && (ui == UI_ADJ_SPOT || ui == UI_ADJ_LIST) && S.valid && !linkLost) drawAdjustDynamic();
  if (actionDirty) drawAction(); else if (modeBtnDirty && ui == UI_MAIN && S.valid && !linkLost) drawModeBtns();
  // Flaggor som inte kunde användas i den här vyn ska inte ligga kvar och slå till senare
  powerDirty = bodyDirty = warmDirty = modeBtnDirty = false; barsDirty = false; sliderDirty = false;
  layoutButtons();                              // billigt; alltid i fas med vyn

  if (blForceUntil && (int32_t)(now - blForceUntil) < 0) { /* tvingat via konsol */ }
  else { blForceUntil = 0; blService(); }
#if ACCENT_STRIP
  accentService();
#endif
  delay(2);
}
