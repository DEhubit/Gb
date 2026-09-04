/*
 * ============================================================================
 *  GARDEROBSBELYSNING — STYRENHET (Arduino UNO R3 + KS0252 Screw Shield)
 *  Firmware v9.6-UNO · bänkversion b5 · 2026-09-02
 *
 *  Följer "Kopplingsschema garderobsbelysning v9.6" (§3, §4, §11, §23, §26)
 *  och "Kopplingsschema v3 · fem blad" (K01–K50).
 *
 *  NYTT I b6
 *   • SAMTIDIGT och MANUELL tänder allt på SAMMA tid (UP_SIMUL_MS). Tidigare
 *     hade spottarna 300 ms och stripsen 1 000 ms även i de lägena, så spottarna
 *     var uppe långt före listerna — "samtidigt" var det bara på papperet.
 *   • KASKAD: stripsen får varsin starttid (450 / 700 ms). De tändes tidigare
 *     samtidigt, och kaskadens sista steg blev därmed ett enda.
 *   • Fasspridningen ryms nu under 4095 (ON = 0/164/328/492/656). Ingen kanals
 *     puls vänder längre runt periodgränsen — CH0 (M5) låg med ON 3276 och hade
 *     som enda kanal huvuddelen av sin puls på andra sidan räknarens nollpunkt.
 *   • MODE2 skrivs uttryckligen (OCH = 0): OFF-registrets två byte träder i kraft
 *     tillsammans, vid STOP. Kom kortet upp med OCH = 1 fick kanalen ett felaktigt
 *     pulsvärde mellan de två byten — en bildruta fel ljus, en gång per skrivning.
 *   • Efterskrivning: 40 ms efter att en kanals toning tagit slut skrivs slutvärdet
 *     en gång till. En ram som förvanskats på bussen skulle annars bli kvar ända
 *     till nästa toning ("lamporna tappar signalen när dimningen tar slut").
 *   • Nivåinställningen tänder lamporna även när systemet är SLÄCKT.
 *   • EEPROM skrivs aldrig mitt i en sekvens (70 ms skrivpaus syntes som ett hack).
 *
 *  NYTT I b5 (efter bänkvideorna)
 *   • KASKAD släcker i OMVÄND ordning: stripsen först, sedan C, B, A — samma
 *     tidslinje som tändningen, spelad baklänges (1 450 ms).
 *   • $P skickas alltid (även när systemet är SLÄCKT, så att panelen ser
 *     nedtoningen), och en ram som inte fick plats i sändbufferten räknas inte
 *     som skickad — panelen tappade tidigare nivåbilden var ~10:e sekund.
 *   • Steglängden i toningen anpassas efter nivån (1/2/4/8): busslasten faller
 *     från ~96 % till under 40 % i kaskadens topp, utan synliga steg.
 *   • Upplänken lyssnar under nivåförhandsvisning (reglaget), men fortfarande
 *     inte under tänd-/släcksekvenser (F8). Wire-timeouten är 25 ms.
 *   • Nya konsolkommandon: pwmhz (byt PWM-frekvens i drift), i2cverify
 *     (skriv/läs-tillbaka-test av bussen) — för att ringa in lampflimmer.
 *
 *  HÅRDVARA SOM PROGRAMMET FÖRUTSÄTTER
 *   • PCA9685 @ 0x40 på A4/A5 (K11/K12), VCC 5 V (K09), GND (K10), OE→GND (K13).
 *     V+ ansluts ALDRIG. I²C körs på 400 kHz (se I2C_HZ_WANTED), prescale 11 = 508,6 Hz.
 *   • PWM-kanaler — UPPMÄTTA på det byggda kortet 2026-09-02 (numreringen löper
 *     alltså åt motsatt håll mot vad som först antogs):
 *         CH15 → M1 · Spot A (bortre)
 *         CH11 → M2 · Spot B (mitt)
 *         CH7  → M3 · Spot C (närmast)
 *         CH3  → M4 · Strip ÖVRE
 *         CH0  → M5 · Strip NEDRE
 *     Signal-GND (K19) tas ur CH15:s grupp i stället för CH0:s — elektriskt samma
 *     sak, alla sexton gruppers GND-stift är ett och samma nät (§4).
 *     (Ändra bara tabellen PCA_CH[] om du lött om.)
 *   • PIR på D2, aktivt LÅG (S8050-steget i PIR-lådan), pullup 4,7 k + 100 n på protoytan.
 *   • Panel: D1 (TX) = nedlänk 115200 (delas med USB-konsolen) · D8 = upplänk 9600
 *     via SoftwareSerial (D7 = dummy-TX, får aldrig anslutas).
 *   • EEPROM: läge · på/av · nivåer · MIN_PWM[5] · efterlystid · auto-återgång.
 *   • Watchdog 2 s (kräver Optiboot — standard på UNO R3, V25).
 *
 *  KONSOL (USB, 115200 baud, radslut LF eller CR): skriv  help
 * ============================================================================
 */

#define FW_VERSION "v9.6-UNO b6"

#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 *  1. KONSTANTER OCH TRIMBARA PARAMETRAR
 * -------------------------------------------------------------------------- */
#define PIN_PIR       2      // aktivt LÅG = rörelse
#define PIN_UP_RX     8      // upplänk från panelen (p4 → D8)
#define PIN_UP_TX     7      // dummy-TX, ansluts aldrig (§3)

// Upplänkens polaritet vid D8.  0 = p4 vilar på 5 V och pulsar mot 0 V (aktivt LÅG,
// panelen sänder med INVERTERAD UART-TX så att S8050 är öppen i vila — V28/S12: vila 5 V).
// 1 = panelen sänder med normal TX (vila 0,2 V vid D8).  Panelen och UNO MÅSTE ha
// komplementära inställningar: UPLINK_INVERTED här == !UPLINK_TX_INVERT i panelen.
#define UPLINK_INVERTED   0

// Watchdog 2 s (F6). Kräver Optiboot (original UNO R3). Har din UNO en gammal
// bootloader som hänger i omstartsloop vid  wdttest  → sätt 0.
#define WDT_ENABLE        1

// ---------------------------------------------------------------------------
//  BUSSHASTIGHETEN — avviker medvetet från §11, läs detta innan du ändrar
//
//  §11 föreskriver Wire.setClock(800000). På en 16 MHz UNO ger det TWBR = 2, och
//  ATmega328P:s datablad (avsnitt 22.5.2) säger uttryckligen: "TWBR should be 10 or
//  higher if the TWI operates in Master mode. If TWBR is lower than 10, the master
//  may produce an incorrect output on SDA and SCL for the remainder of the byte."
//  Högsta hastighet inom specifikationen är därför F_CPU/(16+2·10) = 444 kHz.
//  Dessutom är 800 kHz marginellt även elektriskt: med 3,2 kΩ pullup (10 k ∥ 4,7 k)
//  och ~150 pF busskapacitans är stigtiden ~0,58 µs mot ett SCL-högläge på ~0,6 µs.
//
//  Därför är förvalet 400 kHz. Konsekvensen står i §11:s egen bandbreddstabell:
//  4-byte OFF-skrivning tar 95 µs, och kaskadens överlappsfönster (450–600 ms, då
//  spot C stegar var 275:e µs samtidigt som båda stripsen stegar var 305:e µs)
//  kräver ~10 000 skrivningar/s = 96 % busslast. Motorn tappar aldrig ett PWM-värde
//  (F1 håller) men toningen töjs några tiondelar under just det fönstret.
//  Vill du ha marginal: sätt ONSET_STRIP_N till 700 (= §11 alternativ ①), då
//  försvinner överlappet och lasten faller till 66 %.
//  Vill du följa §11 bokstavligt: sätt 800000UL här. Koden provar då 800 kHz först
//  och faller automatiskt tillbaka 400 → 200 → 100 kHz om kortet inte svarar.
// ---------------------------------------------------------------------------
#define I2C_HZ_WANTED     400000UL

#define PCA_ADDR          0x40
#define PCA_REG_MODE1     0x00
#define PCA_REG_MODE2     0x01
#define PCA_REG_PRESCALE  0xFE
#define PCA_REG_LED0_ON_L 0x06
#define PCA_REG_LED0_OFF_L 0x08
#define PCA_PRESCALE      11     // 25 MHz / (4096 × 508,6 Hz) − 1 = 11 → 508,6 Hz (F2)

#define N_CH        5
#define MAX_PWM     3276         // 80 %-taket — termisk garanti (§10)
#define PWM_FULL_OFF 0x1000      // bit 4 i LEDn_OFF_H = helt av

// Fysisk kanal per plats (plats 0..4 = M1..M5). UPPMÄTT — se rubriken.
// Detta är den enda tabell som behöver ändras om du löder om: fasspridningen,
// taken, kaskadens onset och alla tester räknas på PLATSINDEX, inte kanalnummer.
static const uint8_t  PCA_CH[N_CH] = { 15, 11, 7, 3, 0 };
// Fasspridning (§11): kanalernas flanker läggs på olika ställen i PWM-perioden så
// att de fem lasterna inte slår till samtidigt. §11:s spridning (0/819/1638/2457/
// 3276) tar hela perioden i anspråk — men eftersom taket är 3276 av 4096 hamnar då
// slutet av pulsen bortom räknarens nollpunkt på fyra av fem kanaler, värst på
// CH0 (ON 3276) som får huvuddelen av sin puls på andra sidan. Spridningen ligger
// därför inom 4095 − MAX_PWM = 820 counts: 164 counts (80 µs) mellan flankerna
// räcker med god marginal för D4184:ans switchtransient, och ingen puls vänder
// längre runt. (Ändra bara om MAX_PWM ändras: ON[s] = s·(4096−MAX_PWM)/N_CH.)
static const uint16_t PCA_ON[N_CH] = { 0, 164, 328, 492, 656 };
static const char* const CH_NAME[N_CH] = { "M1 Spot A", "M2 Spot B", "M3 Spot C", "M4 Strip OVRE", "M5 Strip NEDRE" };

// Toningstider (§11)
#define UP_SPOT_MS     300       // spottarnas toning i KASKAD
#define UP_STRIP_MS    1000      // listernas toning i KASKAD
#define UP_SIMUL_MS    700       // SAMTIDIGT och MANUELL: SAMMA tid för alla fem.
                                 // Med 300/1000 ms var spottarna uppe 700 ms före
                                 // listerna — läget hette "samtidigt" men var det inte.
#define DOWN_MS        1000      // nedtoning vid PIR-timeout (SAMTIDIGT/MANUELL)
#define OFF_MS         300       // nedtoning vid $E av (SAMTIDIGT/MANUELL)
#define PREVIEW_MS     250       // omställning av nivå medan lampan lyser
#define SETTLE_MS      40        // efterskrivning av slutvärdet när en toning tagit slut

// Steglängd per nivå (se stepAt() i toningsmotorn). Under STEP_T1 skrivs varje
// värde (F1 — där ser ögat steg), däröver 2, 4 och 8. Ett steg om 8 vid 1 536 är
// 0,5 % linjärt och under 0,2 % upplevt, långt under ögats tröskel (~1 %), men
// antalet skrivningar för en full toning faller från 3 276 till ~800. Det är
// det som tar ned busslasten från ~96 % till under 40 % i kaskadens topp.
#define STEP_T1  128
#define STEP_T2  512
#define STEP_T3  1536

// Kaskadens onset (ms). De fem grupperna tänds i tur och ordning MOT garderoben:
// A 0 · B 150 · C 300 · Strip ÖVRE 450 · Strip NEDRE 700. Stripsen hade tidigare
// samma starttid och tändes därför som en enda grupp — kaskadens sista steg saknades.
// Den senare starten sänker dessutom busslasten i överlappsfönstret (§11 alt. ①).
#define ONSET_STRIP_O  450
#define ONSET_STRIP_N  700
static const uint16_t CASCADE_ONSET[N_CH] = { 0, 150, 300, ONSET_STRIP_O, ONSET_STRIP_N };
// Släckning i KASKAD = tändningen spelad baklänges: varje kanal startar när den
// i tändningen SLUTADE, räknat från sekvensens slut. NEDRE går först, sedan ÖVRE,
// och spottarna följer C → B → A med 300 ms var.
#define CASCADE_TOTAL_MS (ONSET_STRIP_N + UP_STRIP_MS)              // 1 700 ms
static const uint16_t CASCADE_OFF_ONSET[N_CH] = {
  CASCADE_TOTAL_MS - (0   + UP_SPOT_MS),                             // A     1400
  CASCADE_TOTAL_MS - (150 + UP_SPOT_MS),                             // B     1250
  CASCADE_TOTAL_MS - (300 + UP_SPOT_MS),                             // C     1100
  CASCADE_TOTAL_MS - (ONSET_STRIP_O + UP_STRIP_MS),                  // ÖVRE   250
  CASCADE_TOTAL_MS - (ONSET_STRIP_N + UP_STRIP_MS)                   // NEDRE    0
};

// Wire-timeout. Måste rymma en hel upplänksram (17 tecken à 1,04 ms — SoftwareSerial
// stänger avbrotten per tecken och TWI-avbrottet får vänta), annars registreras
// falska bussfel varje gång panelen sänder medan en toning pågår.
#define WIRE_TIMEOUT_US   25000UL

#define PIR_WARM_S        60     // PIR-varmräknare vid uppstart
#define PIR_DEBOUNCE_MS   50
#define DEFAULT_HOLD_S    120    // efterlystid efter sista rörelse (trimbar: hold <s>)
#define DEFAULT_MANUALMAX_MIN 120  // auto-återgång MANUELL→KASKAD (0 = av)
#define EEPROM_DELAY_MS   5000
#define ADJ_TIMEOUT_MS    20000
#define STATUS_PERIOD_MS  500
#define P_PERIOD_MS       50     // $P i 20 Hz under sekvens
#define P_IDLE_MS         2000   // $P var 2:a sekund i vila (nivåbild till panelen)
#define LOG_PERIOD_MS     5000

enum { MODE_MANUAL = 0, MODE_CASCADE = 1, MODE_SIMUL = 2 };
enum { VIEW_MAIN = 0, VIEW_ADJ_SPOT = 1, VIEW_ADJ_LIST = 2, VIEW_ERR = 3, VIEW_STANDBY = 4, VIEW_SEQ = 5 };
enum { ERR_NONE = 0, ERR_PCA = 1, ERR_EEPROM = 2 };

/* --------------------------------------------------------------------------
 *  2. TONINGSKURVAN — tidsdomänmetoden (F1)
 *  G_TAB[i] = tidpunkt (×65535) då PWM-värdet u = i/512 av spannet ska skrivas,
 *  för kurvan  v(t) = CIE1931( smoothstep(t/T) ).  Varje värde passeras exakt en
 *  gång; uppehållstiden per värde följer kurvan (dwell ∝ v^(−2/3) i mitten).
 * -------------------------------------------------------------------------- */
static const uint16_t G_TAB[513] PROGMEM = {
      0,  5163,  7391,  9140, 10644, 11974, 13072, 13998, 14802, 15515, 16158, 16744, 17283, 17784, 18251, 18689,
  19103, 19496, 19868, 20224, 20564, 20890, 21203, 21505, 21795, 22077, 22349, 22612, 22868, 23116, 23358, 23593,
  23822, 24046, 24264, 24478, 24686, 24890, 25090, 25286, 25477, 25666, 25850, 26031, 26209, 26384, 26557, 26726,
  26892, 27056, 27218, 27377, 27533, 27688, 27840, 27990, 28138, 28285, 28429, 28571, 28712, 28851, 28989, 29125,
  29259, 29392, 29523, 29653, 29781, 29908, 30034, 30159, 30282, 30404, 30525, 30645, 30764, 30881, 30998, 31113,
  31228, 31341, 31454, 31565, 31676, 31786, 31895, 32003, 32110, 32216, 32322, 32427, 32531, 32634, 32737, 32838,
  32939, 33040, 33139, 33238, 33337, 33435, 33532, 33628, 33724, 33819, 33914, 34008, 34102, 34195, 34287, 34379,
  34470, 34561, 34652, 34741, 34831, 34920, 35008, 35096, 35184, 35271, 35357, 35443, 35529, 35615, 35699, 35784,
  35868, 35952, 36035, 36118, 36200, 36283, 36364, 36446, 36527, 36608, 36688, 36768, 36848, 36927, 37006, 37085,
  37164, 37242, 37319, 37397, 37474, 37551, 37628, 37704, 37780, 37856, 37931, 38007, 38082, 38156, 38231, 38305,
  38379, 38453, 38526, 38599, 38672, 38745, 38817, 38890, 38962, 39034, 39105, 39177, 39248, 39319, 39389, 39460,
  39530, 39600, 39670, 39740, 39810, 39879, 39948, 40017, 40086, 40154, 40223, 40291, 40359, 40427, 40495, 40562,
  40630, 40697, 40764, 40831, 40898, 40965, 41031, 41097, 41163, 41229, 41295, 41361, 41427, 41492, 41557, 41623,
  41688, 41752, 41817, 41882, 41946, 42011, 42075, 42139, 42203, 42267, 42331, 42395, 42458, 42522, 42585, 42648,
  42711, 42774, 42837, 42900, 42963, 43025, 43088, 43150, 43212, 43275, 43337, 43399, 43461, 43523, 43584, 43646,
  43708, 43769, 43830, 43892, 43953, 44014, 44075, 44136, 44197, 44258, 44319, 44380, 44440, 44501, 44561, 44622,
  44682, 44743, 44803, 44863, 44923, 44983, 45043, 45103, 45163, 45223, 45283, 45343, 45402, 45462, 45522, 45581,
  45641, 45700, 45759, 45819, 45878, 45937, 45997, 46056, 46115, 46174, 46233, 46292, 46351, 46410, 46469, 46528,
  46587, 46646, 46705, 46764, 46823, 46881, 46940, 46999, 47058, 47116, 47175, 47234, 47292, 47351, 47410, 47468,
  47527, 47585, 47644, 47703, 47761, 47820, 47878, 47937, 47995, 48054, 48112, 48171, 48230, 48288, 48347, 48405,
  48464, 48522, 48581, 48640, 48698, 48757, 48815, 48874, 48933, 48991, 49050, 49109, 49168, 49226, 49285, 49344,
  49403, 49462, 49521, 49580, 49639, 49698, 49757, 49816, 49875, 49934, 49993, 50052, 50112, 50171, 50230, 50290,
  50349, 50409, 50468, 50528, 50588, 50647, 50707, 50767, 50827, 50887, 50947, 51007, 51067, 51127, 51188, 51248,
  51308, 51369, 51430, 51490, 51551, 51612, 51673, 51734, 51795, 51856, 51917, 51979, 52040, 52102, 52164, 52225,
  52287, 52349, 52411, 52474, 52536, 52598, 52661, 52724, 52787, 52850, 52913, 52976, 53039, 53103, 53166, 53230,
  53294, 53358, 53423, 53487, 53551, 53616, 53681, 53746, 53811, 53877, 53942, 54008, 54074, 54140, 54207, 54273,
  54340, 54407, 54474, 54542, 54609, 54677, 54745, 54814, 54882, 54951, 55020, 55090, 55159, 55229, 55299, 55370,
  55440, 55512, 55583, 55655, 55727, 55799, 55871, 55944, 56018, 56092, 56166, 56240, 56315, 56390, 56466, 56542,
  56618, 56695, 56773, 56850, 56929, 57008, 57087, 57167, 57247, 57328, 57410, 57492, 57574, 57658, 57742, 57826,
  57912, 57998, 58085, 58172, 58260, 58350, 58440, 58531, 58622, 58715, 58809, 58904, 59000, 59097, 59195, 59294,
  59395, 59497, 59601, 59706, 59812, 59921, 60031, 60143, 60257, 60373, 60491, 60612, 60735, 60862, 60991, 61124,
  61260, 61400, 61545, 61694, 61849, 62010, 62178, 62355, 62540, 62738, 62949, 63178, 63430, 63715, 64052, 64489,
  65535,
};

/* --------------------------------------------------------------------------
 *  3. EEPROM-STRUKTUREN (§11): sig · mode · enabled · spotMax · listMax · minPwm[5]
 *     + holdSec · manualMaxMin (tillägg i denna firmware) · sum
 * -------------------------------------------------------------------------- */
#define CFG_SIG 0xA6
struct Cfg {
  uint8_t  sig;
  uint8_t  mode;
  uint8_t  enabled;
  uint8_t  spotMax;
  uint8_t  listMax;
  uint16_t minPwm[N_CH];
  uint16_t holdSec;
  uint16_t manualMaxMin;
  uint8_t  sum;
};
static Cfg cfg;
static uint16_t eepromWrites = 0;       // V42-räknare
static uint32_t eepromDirtyAt = 0;
static bool     eepromDirty = false;

static uint8_t cfgChecksum(const Cfg &c) {
  const uint8_t *p = (const uint8_t *)&c;
  uint8_t s = 0x5A;
  for (size_t i = 0; i < sizeof(Cfg) - 1; i++) s = (uint8_t)(s * 31 + p[i]);
  return s;
}
static void cfgDefaults() {
  cfg.sig = CFG_SIG; cfg.mode = MODE_CASCADE; cfg.enabled = 1;
  cfg.spotMax = 100; cfg.listMax = 100;
  for (uint8_t i = 0; i < N_CH; i++) cfg.minPwm[i] = 0;
  cfg.holdSec = DEFAULT_HOLD_S; cfg.manualMaxMin = DEFAULT_MANUALMAX_MIN;
}
static bool cfgValid(const Cfg &c) {
  if (c.sig != CFG_SIG || c.sum != cfgChecksum(c)) return false;
  if (c.mode > 2 || c.enabled > 1) return false;
  if (c.spotMax < 20 || c.spotMax > 100 || c.listMax < 20 || c.listMax > 100) return false;
  for (uint8_t i = 0; i < N_CH; i++) if (c.minPwm[i] >= MAX_PWM) return false;
  if (c.holdSec < 5 || c.holdSec > 3600) return false;
  if (c.manualMaxMin > 1440) return false;
  return true;
}
static bool saveSettings() {            // skriv aldrig ett oförändrat värde (§11)
  cfg.sig = CFG_SIG; cfg.sum = cfgChecksum(cfg);
  eepromDirty = false;
  Cfg chk; EEPROM.get(0, chk);
  if (memcmp(&chk, &cfg, sizeof(Cfg)) == 0) return true;   // identiskt — ingen skrivning, ingen räkning
  EEPROM.put(0, cfg);                     // put = update: bara ändrade byte skrivs
  eepromWrites++;
  EEPROM.get(0, chk);
  return memcmp(&chk, &cfg, sizeof(Cfg)) == 0;
}
static void markDirty() { eepromDirty = true; eepromDirtyAt = millis(); }

/* --------------------------------------------------------------------------
 *  4. TILLSTÅND
 * -------------------------------------------------------------------------- */
SoftwareSerial up(PIN_UP_RX, PIN_UP_TX, UPLINK_INVERTED ? true : false);

// Överlever en omstart (RAM behåller sitt innehåll när processorn resettas men
// inte när matningen bryts). Ger ett entydigt svar på "startade watchdogen om mig?"
struct NoInit { uint32_t magic; uint16_t boots; uint8_t wdtFlag; };
static NoInit __attribute__((section(".noinit"))) ni;
#define NI_MAGIC 0x5A47D101UL

static uint32_t i2cHz = 0;
static uint8_t  errCode = ERR_NONE;
static uint8_t  view = VIEW_STANDBY;
static uint16_t lastSeq = 0xFFFF;
static bool     muteFrames = false;
static bool     logOn = true;

static uint8_t  previewSpot = 100, previewList = 100;
static uint16_t spotCap = MAX_PWM, listCap = MAX_PWM;
static uint8_t  adjGroup = 0xFF;          // 0 = spot, 1 = list, 0xFF = ingen justering pågår
static uint32_t lastAdjEvent = 0;

static bool     lit = false;              // "rummet är tänt" (sekvensmotorns huvudtillstånd)
static bool     presence = false;
static bool     pirRaw = false, pirStable = false;
static uint32_t pirEdgeAt = 0, lastMotionAt = 0;
static uint32_t warmUntil = 0;
static uint32_t manualSince = 0;
static bool     simMotion = false;

static uint32_t lastStatusAt = 0, lastPAt = 0, lastLogAt = 0;
static uint32_t loopMax = 0, loopMaxSeq = 0, loopPrev = 0;
static uint16_t i2cErrTotal = 0, i2cErrSeq = 0;
static uint8_t  i2cConsecFail = 0;
static uint32_t pcaRetryAt = 0;
static bool     seqActive = false, listening = false;
static uint32_t bootMs = 0;

// Testläge (konsol): engine avstängd, kanaler styrs för hand
static bool     testMode = false;
static uint8_t  testSlot = 0;
static uint32_t testNextAt = 0;
static uint16_t rampVal = 0;
static uint8_t  testKind = 0;             // 0 = raw, 1 = kanaltest, 2 = ramp

/* --------------------------------------------------------------------------
 *  5. PCA9685
 * -------------------------------------------------------------------------- */
static uint32_t sclActual() {                 // verklig SCL-frekvens ur TWBR/TWPS
  uint8_t ps = 1;
  switch (TWSR & 0x03) { case 1: ps = 4; break; case 2: ps = 16; break; case 3: ps = 64; break; }
  return F_CPU / (16UL + 2UL * (uint32_t)TWBR * ps);
}
static bool pcaPing() { Wire.beginTransmission(PCA_ADDR); return Wire.endTransmission() == 0; }
// Provar hastigheterna uppifrån och ned tills kortet svarar åtta gånger i rad
static bool i2cProbe() {
  // OBS: heter inte SP — det namnet ar upptaget av AVR:ens stackpekarregister
  static const uint32_t HZ_LIST[4] = { 800000UL, 400000UL, 200000UL, 100000UL };
  for (uint8_t i = 0; i < 4; i++) {
    if (HZ_LIST[i] > I2C_HZ_WANTED) continue;
    Wire.setClock(HZ_LIST[i]); i2cHz = HZ_LIST[i];
    delay(2);
    bool ok = true;
    for (uint8_t k = 0; k < 8 && ok; k++) { if (!pcaPing()) ok = false; delayMicroseconds(100); }
    if (ok) return true;
  }
  return false;
}
static void i2cScan() {
  Serial.print(F("# scan @ ")); Serial.print(sclActual() / 1000); Serial.println(F(" kHz:"));
  uint8_t n = 0;
  for (uint8_t a = 0x03; a < 0x78; a++) {
    wdt_reset();
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("#   svar fran 0x")); Serial.print(a, HEX);
      if (a == PCA_ADDR) Serial.print(F("  <-- PCA9685"));
      else if (a == 0x70) Serial.print(F("  <-- All-Call: chipet lever men lyssnar pa fel adress"));
      else Serial.print(F("  <-- adressbygel bryggad?"));
      Serial.println(); n++;
    }
    delayMicroseconds(200);
  }
  if (!n) Serial.println(F("#   inget svar fran nagon adress"));
}
static uint8_t pcaWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission();
}
static uint16_t chanCur[N_CH];            // senast skrivet PWM-värde (0 = av)

// Endast OFF-registret skrivs i drift (§11) — 4 byte på bussen, ~95 µs vid 400 kHz.
static uint8_t pcaWriteOff(uint8_t slot, uint16_t v) {
  uint16_t off = (v == 0) ? PWM_FULL_OFF : (uint16_t)((PCA_ON[slot] + v) & 0x0FFF);
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(PCA_REG_LED0_OFF_L + 4 * PCA_CH[slot]);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  uint8_t rc = Wire.endTransmission();
  if (rc) {
    i2cErrTotal++; if (seqActive) i2cErrSeq++;
    if (i2cConsecFail < 250) i2cConsecFail++;
    if (i2cConsecFail >= 3 && errCode == ERR_NONE) { errCode = ERR_PCA; pcaRetryAt = millis() + 2000; }
  } else {
    i2cConsecFail = 0;
    if (errCode == ERR_PCA) errCode = ERR_NONE;
  }
  return rc;
}
static void setChan(uint8_t slot, uint16_t v) {
  if (v > MAX_PWM) v = MAX_PWM;
  chanCur[slot] = v;
  pcaWriteOff(slot, v);
}
static bool pcaInit() {
  Wire.beginTransmission(PCA_ADDR);
  if (Wire.endTransmission() != 0) return false;
  pcaWrite8(PCA_REG_MODE1, 0x10);          // SLEEP=1 — krävs för prescale
  pcaWrite8(PCA_REG_PRESCALE, PCA_PRESCALE);
  pcaWrite8(PCA_REG_MODE1, 0x20); delay(5); // AI=1, SLEEP=0
  // MODE2 = 0x04: OUTDRV=1 (totempol, D4184:ans grind), INVRT=0, OCH=0.
  // OCH=0 betyder att utgångarna byter värde vid STOP — de två byten i OFF-
  // registret träder alltså i kraft TILLSAMMANS. Med OCH=1 (om kortet av någon
  // anledning kommer upp så) hinner låg byten verka ensam och kanalen får ett
  // felaktigt pulsvärde under en PWM-period vid varje skrivning = flimmer.
  pcaWrite8(PCA_REG_MODE2, 0x04);
  pcaWrite8(PCA_REG_MODE1, 0xA0);          // RESTART + AI
  // ON-registren skrivs EN gång (fasspridning); alla 16 kanaler helt av
  for (uint8_t c = 0; c < 16; c++) {
    uint16_t on = 0;
    for (uint8_t s = 0; s < N_CH; s++) if (PCA_CH[s] == c) on = PCA_ON[s];
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(PCA_REG_LED0_ON_L + 4 * c);
    Wire.write(on & 0xFF); Wire.write(on >> 8);
    Wire.write(0x00); Wire.write(0x10);    // OFF = helt av
    Wire.endTransmission();
  }
  for (uint8_t s = 0; s < N_CH; s++) chanCur[s] = 0;
  i2cConsecFail = 0;
  return true;
}

/* --------------------------------------------------------------------------
 *  6. TONINGSMOTORN — en aktiv toning per kanal, tidsdomänmetoden
 * -------------------------------------------------------------------------- */
struct Fade {
  bool     active;
  bool     up;
  uint16_t lo, hi;        // kurvans spann (lo ≥ MIN_PWM)
  uint16_t v;             // senast skrivet kurvvärde
  uint16_t vEnd;          // slutvärde (kan vara 0 = under lo)
  uint16_t vStop;         // sista kurvvärdet (vEnd klippt till [lo,hi])
  uint32_t t0;            // µs, justerad start
  uint16_t T64;           // varaktighet i 64 µs-enheter
  uint32_t dU1;           // spannposition per PWM-värde, 32-bit fixpunkt
  uint32_t tNext;         // µs då nästa värde ska skrivas
  uint32_t onsetAt;       // ms: väntar på onset om pendingOnset
  bool     pendingOnset;
  uint16_t onsetTarget;
  uint16_t onsetT;
};
static Fade fade[N_CH];
// Efterskrivning: när en toning tagit slut skrivs slutvärdet en gång till efter
// SETTLE_MS. Skrivningen är identisk med den föregående, så den kan inte i sig ge
// någon flank — men om den SISTA ramen i en toning förvanskades på bussen (PCA:n
// kvitterar ändå) skulle kanalen annars ligga kvar på fel ljus ända till nästa
// toning. Det är precis vad "listen tappar signalen när dimningen tar slut" ser
// ut som. Kostar fem I²C-skrivningar per sekvens.
static uint32_t settleAt[N_CH];

static uint16_t gLookup(uint16_t u16) {       // G(u) med linjär interpolation
  uint16_t idx = u16 >> 7;                     // 0..511
  uint8_t  frac = u16 & 0x7F;
  uint16_t a = pgm_read_word(&G_TAB[idx]);
  uint16_t b = pgm_read_word(&G_TAB[idx + 1]);
  return a + (uint16_t)(((uint32_t)(b - a) * frac) >> 7);
}
static uint32_t fadeDueUs(uint8_t slot, uint16_t u16) {   // (tar slot, inte Fade& — Arduino-prototyperna)
  uint16_t g = gLookup(u16);
  if (!fade[slot].up) g = 65535 - g;         // nedtoning = speglad kurva
  return fade[slot].t0 + ((((uint32_t)fade[slot].T64 * g) >> 16) << 6);
}
static uint8_t  stepAt(uint16_t v) { return v < STEP_T1 ? 1 : (v < STEP_T2 ? 2 : (v < STEP_T3 ? 4 : 8)); }
static uint16_t uOf(uint8_t slot, uint16_t v) {          // PWM-värde → position i spannet (0..65535)
  return (uint16_t)(((uint32_t)(v - fade[slot].lo) * fade[slot].dU1) >> 16);
}
static uint16_t nextVal(uint8_t slot, uint16_t v) {      // nästa kurvvärde efter v, steglängd efter nivå
  Fade &f = fade[slot];
  if (f.up) { uint8_t st = stepAt(v);             return (v + st >= f.vStop) ? f.vStop : v + st; }
  else      { uint8_t st = stepAt(v ? v - 1 : 0); return (v <= f.vStop + st) ? f.vStop : v - st; }
}
static uint16_t capFor(uint8_t slot) { return slot < 3 ? spotCap : listCap; }

static void fadeStop(uint8_t slot) { fade[slot].active = false; fade[slot].pendingOnset = false; }

// Starta toning från nuvarande värde till target under T ms.
//  fullSpan = true : kurvan spänner [MIN_PWM, tak] och startpunkten justeras så att
//                    en avbruten toning fortsätter där den är (riktningsbyten blir mjuka).
//  fullSpan = false: kurvan komprimeras till [cur, target] (nivåomställning i JUSTERA).
static void startFade(uint8_t slot, uint16_t target, uint16_t T_ms, bool fullSpan) {
  Fade &f = fade[slot];
  uint16_t cur = chanCur[slot];
  f.pendingOnset = false;
  if (target > MAX_PWM) target = MAX_PWM;
  if (cur == target) { f.active = false; return; }
  uint16_t lo = (cur < target) ? cur : target;
  uint16_t hi = (cur > target) ? cur : target;
  uint16_t mn = cfg.minPwm[slot];
  if (fullSpan) { lo = 0; uint16_t c = capFor(slot); if (c > hi) hi = c; }
  if (lo < mn) lo = mn;
  f.up = (target > cur);
  // start- och stoppvärde på kurvan
  uint16_t v0 = cur; if (v0 < lo) v0 = lo; if (v0 > hi) v0 = hi;
  uint16_t vStop = target; if (vStop < lo) vStop = lo; if (vStop > hi) vStop = hi;
  uint16_t travel = f.up ? (vStop - v0) : (v0 - vStop);
  if (hi <= lo + 8 || travel < 8) {          // för kort för en kurva — hoppa direkt
    setChan(slot, target); f.active = false; return;
  }
  f.lo = lo; f.hi = hi; f.vEnd = target; f.vStop = vStop;
  f.T64 = (uint16_t)(((uint32_t)T_ms * 1000UL) >> 6);
  f.dU1 = 0xFFFFFFFFUL / (uint32_t)(hi - lo);
  f.v = v0;
  uint16_t g0 = gLookup(uOf(slot, v0));
  if (!f.up) g0 = 65535 - g0;
  uint32_t now = micros();
  f.t0 = now - ((((uint32_t)f.T64 * g0) >> 16) << 6);
  f.active = true;
  if (f.up && cur < lo) setChan(slot, lo);   // hoppa in i det synliga området
  f.tNext = fadeDueUs(slot, uOf(slot, nextVal(slot, v0)));
}
static void scheduleOnset(uint8_t slot, uint16_t target, uint16_t T_ms, uint16_t delayMs) {
  Fade &f = fade[slot];
  if (delayMs == 0) { startFade(slot, target, T_ms, true); return; }
  f.active = false;
  f.pendingOnset = true; f.onsetAt = millis() + delayMs; f.onsetTarget = target; f.onsetT = T_ms;
}
static bool anyFadeActive() {
  for (uint8_t s = 0; s < N_CH; s++) if (fade[s].active || fade[s].pendingOnset) return true;
  return false;
}
// En iteration: högst ett registerskrivning per kanal och varv
static void fadeService() {
  uint32_t nowMs = millis();
  for (uint8_t s = 0; s < N_CH; s++) {
    Fade &f = fade[s];
    if (settleAt[s] && (int32_t)(nowMs - settleAt[s]) >= 0) { settleAt[s] = 0; pcaWriteOff(s, chanCur[s]); }
    if (f.pendingOnset) {
      if ((int32_t)(nowMs - f.onsetAt) >= 0) { f.pendingOnset = false; startFade(s, f.onsetTarget ? capFor(s) : 0, f.onsetT, true); }   // taket läses vid onset
      continue;
    }
    if (!f.active) continue;
    if ((int32_t)(micros() - f.tNext) < 0) continue;
    // nästa värde — högst ett steg per kanal och varv
    uint16_t nv = nextVal(s, f.v);
    f.v = nv;
    setChan(s, nv);
    if (nv == f.vStop) {
      f.active = false;
      if (chanCur[s] != f.vEnd) setChan(s, f.vEnd);   // ned under lo → 0
      settleAt[s] = nowMs + SETTLE_MS; if (!settleAt[s]) settleAt[s] = 1;
    } else {
      f.tNext = fadeDueUs(s, uOf(s, nextVal(s, nv)));
    }
  }
}

/* --------------------------------------------------------------------------
 *  7. SEKVENSMOTORN — läge, PIR, av/på-grind
 * -------------------------------------------------------------------------- */
static bool bigSeq = false;               // tänd-/släcksekvens pågår → upplänken tyst (F8)
// Tändning. KASKAD: grupperna startar efter varandra och har var sin toningstid
// (spottarna 300 ms, listerna 1 000 ms) — det är själva kaskaden. SAMTIDIGT och
// MANUELL: alla fem startar samtidigt OCH tonar lika länge, annars är spottarna
// framme långt före listerna och ljuset "vandrar" även i det läge som inte ska.
static void allOn(bool cascade) {
  bigSeq = true;
  for (uint8_t s = 0; s < N_CH; s++) {
    uint16_t T = cascade ? ((s < 3) ? UP_SPOT_MS : UP_STRIP_MS) : UP_SIMUL_MS;
    scheduleOnset(s, capFor(s), T, cascade ? CASCADE_ONSET[s] : 0);
  }
}
static void groupSet(uint8_t grp, bool on, uint16_t T_ms) {   // grp 0 = spot (M1–M3), 1 = list (M4–M5)
  uint8_t a = grp ? 3 : 0, b = grp ? N_CH : 3;
  for (uint8_t s = a; s < b; s++) startFade(s, on ? capFor(s) : 0, T_ms, true);
}
// Under nivåinställning lyser den grupp som ställs in — ALLTID, även när systemet
// står i SLÄCKT. Man ska kunna ställa ljusstyrkan och se resultatet utan att först
// tända rummet; grupperna släcks igen när inställningen avslutas (endAdjust).
static bool groupWantsPreview(uint8_t grp) { return adjGroup == grp; }
// Släckning. KASKAD: tändningen baklänges (stripsen först, sedan C, B, A) med
// tändningens egna toningstider — oavsett om det är PIR-timeout eller ett tryck
// på panelen. SAMTIDIGT/MANUELL: allt tonar ned samtidigt under offT.
static void allOff(bool cascade, uint16_t offT) {
  bigSeq = true;
  for (uint8_t s = 0; s < N_CH; s++) {
    if (groupWantsPreview(s < 3 ? 0 : 1)) continue;   // förhandsvisning håller gruppen tänd
    if (cascade) scheduleOnset(s, 0, (s < 3) ? UP_SPOT_MS : UP_STRIP_MS, CASCADE_OFF_ONSET[s]);
    else         startFade(s, 0, offT, true);
  }
}

static void applyPreview() {
  spotCap = (uint16_t)((uint32_t)MAX_PWM * previewSpot / 100UL);
  listCap = (uint16_t)((uint32_t)MAX_PWM * previewList / 100UL);
  // lampor som lyser följer med till den nya nivån
  for (uint8_t s = 0; s < N_CH; s++) {
    bool shouldBeOn = lit || groupWantsPreview(s < 3 ? 0 : 1);
    if (shouldBeOn && (chanCur[s] != 0 || fade[s].active || fade[s].pendingOnset) && !fade[s].pendingOnset) {
      if (chanCur[s] != capFor(s)) startFade(s, capFor(s), PREVIEW_MS, false);
    }
  }
}
static bool wantLit() {
  if (!cfg.enabled) return false;
  if (cfg.mode == MODE_MANUAL) return true;
  return presence;
}
static void sendStatus();
static void setLit(bool on, uint16_t offT) {
  if (on == lit) return;
  lit = on;
  if (on) allOn(cfg.mode == MODE_CASCADE);
  else    allOff(cfg.mode == MODE_CASCADE, offT);
}
static void endAdjust(bool commit) {
  uint8_t g = adjGroup;
  adjGroup = 0xFF;
  if (commit) { cfg.spotMax = previewSpot; cfg.listMax = previewList; markDirty(); }
  else { previewSpot = cfg.spotMax; previewList = cfg.listMax; }
  applyPreview();
  if (g != 0xFF && !lit) { groupSet(0, false, DOWN_MS); groupSet(1, false, DOWN_MS); }   // no-op för redan släckta
  view = VIEW_MAIN;
}
static void applyModeChange(uint8_t newMode) {
  cfg.mode = newMode; manualSince = millis(); markDirty();
  // wantLit utvärderas i loop(); MANUELL tänder, KASKAD/SAMTIDIGT släcker om rummet är tomt
}

/* --------------------------------------------------------------------------
 *  8. PROTOKOLLET (§5) — $S ut, $P ut, $M/$B/$W/$E in
 * -------------------------------------------------------------------------- */
static uint8_t pctOf(uint16_t v) { return (uint8_t)(((uint32_t)v * 100UL + MAX_PWM / 2) / MAX_PWM); }
static uint8_t computeView() {
  if (errCode) return VIEW_ERR;
  if (adjGroup == 0) return VIEW_ADJ_SPOT;
  if (adjGroup == 1) return VIEW_ADJ_LIST;
  if (seqActive) return VIEW_SEQ;
  return presence ? VIEW_MAIN : VIEW_STANDBY;
}
// Returnerar true när ramen verkligen lades i sändbufferten. Under en sekvens får
// sändningen aldrig blockera toningen: finns det inte plats släpps ramen och
// anroparen försöker igen nästa varv (tidsstämpeln uppdateras bara vid lyckad
// sändning — annars tappar panelen $P-pulsen varje gång loggraden är i vägen).
// Utanför sekvens får Serial.write blockera de få ms som behövs.
static bool sendFrame(const char *body) {
  uint8_t x = 0; size_t n = strlen(body);
  for (size_t i = 0; i < n; i++) x ^= (uint8_t)body[i];
  if (seqActive && Serial.availableForWrite() < (int)(n + 6)) return false;
  Serial.write('$'); Serial.write(body); Serial.write('*');
  if (x < 16) Serial.write('0');
  Serial.print(x, HEX); Serial.write('\n');
  return true;
}
static void sendStatus() {
  if (muteFrames) { lastStatusAt = millis(); return; }
  uint8_t warm = 0;
  int32_t left = (int32_t)(warmUntil - millis());
  if (left > 0) warm = (uint8_t)((left + 999) / 1000);
  char b[48];
  snprintf(b, sizeof b, "S,%u,%u,%u,%u,%u,%u,%u",
           cfg.mode, warm, previewSpot, previewList, view, errCode, cfg.enabled);
  if (sendFrame(b)) lastStatusAt = millis();
}
// $P skickas alltid — även när systemet är SLÄCKT. Nivåerna är sanna ändå (de
// tonar mot 0), och panelen/accentlisten ska följa nedtoningen, inte gissa.
static void sendLevels() {
  if (muteFrames) { lastPAt = millis(); return; }
  char b[40];
  snprintf(b, sizeof b, "P,%u,%u,%u,%u,%u", pctOf(chanCur[0]), pctOf(chanCur[1]), pctOf(chanCur[2]),
           pctOf(chanCur[3]), pctOf(chanCur[4]));
  if (sendFrame(b)) lastPAt = millis();
}

static bool seqIsNew(uint16_t sq) { if (sq == lastSeq) return false; lastSeq = sq; return true; }
static uint16_t nextU16(char **pp) {       // nästa komma-separerade heltal, 0xFFFF = fel
  char *p = *pp;
  if (*p < '0' || *p > '9') return 0xFFFF;
  char *end; unsigned long v = strtoul(p, &end, 10);
  if (v > 65534UL) return 0xFFFF;
  if (*end == ',') end++;
  *pp = end;
  return (uint16_t)v;
}
static void handleFrame(char *buf) {       // buf = texten mellan '$' och radslut
  char *star = strchr(buf, '*');
  if (!star || star == buf) return;
  uint8_t x = 0;
  for (char *p = buf; p < star; p++) x ^= (uint8_t)*p;
  char *e; unsigned long want = strtoul(star + 1, &e, 16);
  if (e != star + 3 || want != x) return;
  *star = 0;
  if (buf[1] != ',') return;
  if (logOn) { Serial.print(F("# rx ")); Serial.println(buf); }   // bänklogg: varje giltig ram från panelen
  char *p = buf + 2;
  if (buf[0] == 'M') {
    uint16_t m = nextU16(&p), sq = nextU16(&p);
    if (m == 0xFFFF || sq == 0xFFFF || m > 2) return;
    if (!seqIsNew(sq)) return;
    if (m != cfg.mode) applyModeChange((uint8_t)m);
    sendStatus(); return;
  }
  if (buf[0] == 'B') {                     // förhandsvisning, ingen EEPROM-skrivning
    uint16_t grp = nextU16(&p), pct = nextU16(&p), sq = nextU16(&p);
    if (grp == 0xFFFF || pct == 0xFFFF || sq == 0xFFFF) return;
    if (grp > 1 || pct < 20 || pct > 100) return;
    if (!seqIsNew(sq)) return;
    if (grp == 0) previewSpot = (uint8_t)pct; else previewList = (uint8_t)pct;
    lastAdjEvent = millis();
    if (adjGroup != grp) {
      if (adjGroup != 0xFF && !lit) groupSet(adjGroup, false, DOWN_MS);   // byte av grupp: släck den förra
      adjGroup = (uint8_t)grp;
      view = grp ? VIEW_ADJ_LIST : VIEW_ADJ_SPOT;
    }
    applyPreview();
    // Tänd gruppen om den ligger mörk — i tomt rum OCH när systemet står i SLÄCKT.
    // Är den redan tänd har applyPreview() just flyttat den till den nya nivån.
    { uint8_t s0 = grp ? 3 : 0;
      if (!lit && chanCur[s0] == 0 && !fade[s0].active) groupSet((uint8_t)grp, true, PREVIEW_MS); }
    sendStatus(); return;
  }
  if (buf[0] == 'W') {                     // spara
    uint16_t sq = nextU16(&p);
    if (sq == 0xFFFF || !seqIsNew(sq)) return;
    endAdjust(true);
    sendStatus(); return;
  }
  if (buf[0] == 'E') {                     // av/på
    uint16_t en = nextU16(&p), sq = nextU16(&p);
    if (en == 0xFFFF || sq == 0xFFFF || en > 1) return;
    if (!seqIsNew(sq)) return;
    if (en != cfg.enabled) {
      cfg.enabled = (uint8_t)en; manualSince = millis(); markDirty();
      if (!en) { if (adjGroup != 0xFF) endAdjust(false); setLit(false, OFF_MS); }
    }
    sendStatus(); return;
  }
}
static char upBuf[32]; static uint8_t upLen = 0; static bool upOverflow = false;
static void pollUplink() {
  while (up.available()) {
    char c = up.read();
    if (c == '$') { upLen = 0; upOverflow = false; continue; }
    if (c == '\n' || c == '\r') {
      if (upLen && !upOverflow) { upBuf[upLen] = 0; handleFrame(upBuf); }
      upLen = 0; upOverflow = false; continue;
    }
    if (upLen < sizeof upBuf - 1) upBuf[upLen++] = c; else upOverflow = true;
  }
}

/* --------------------------------------------------------------------------
 *  9. KONSOLEN (USB)
 * -------------------------------------------------------------------------- */
static void printHelp() {
  Serial.println(F("# KOMMANDON:"));
  Serial.println(F("#  on | off            tand/slack (E-flaggan)"));
  Serial.println(F("#  mode 0|1|2          0 MANUELL 1 KASKAD 2 SAMTIDIGT"));
  Serial.println(F("#  spot <20-100>       spotnivå (sparas efter 5 s)"));
  Serial.println(F("#  list <20-100>       listnivå"));
  Serial.println(F("#  hold <s>            efterlystid efter sista rorelse"));
  Serial.println(F("#  manualmax <ms>      auto-atergang MANUELL->KASKAD (0 = av)"));
  Serial.println(F("#  save                skriv EEPROM nu"));
  Serial.println(F("#  defaults            fabriksvarden + save"));
  Serial.println(F("#  stat | pir          tillstand / PIR-status"));
  Serial.println(F("#  sim                 simulera rorelse (en puls)"));
  Serial.println(F("#  test                kanaltest M1..M5 50 % i tur och ordning"));
  Serial.println(F("#  raw <1-5> <pwm>     satt kanal direkt (testlage)"));
  Serial.println(F("#  ramp <1-5>          TROSKELTEST: stegar upp fran 0, skicka valfri rad for stopp"));
  Serial.println(F("#  sniff <A0..A3|3-12> identifiera vilken kanal som nar en matpunkt (tradstump dit)"));
  Serial.println(F("#  min <1-5> <pwm>     satt MIN_PWM for kanal"));
  Serial.println(F("#  run                 lamna testlaget"));
  Serial.println(F("#  i2ctest             1000 OFF-skrivningar, tid per skrivning (V48)"));
  Serial.println(F("#  i2cverify           skriv/las-tillbaka 2000 ggr: hittar bitfel som PCA:n kvitterar"));
  Serial.println(F("#  pwmhz <24-1526>     byt PWM-frekvens i drift (flimmertest av lamporna)"));
  Serial.println(F("#  scan                skanna I2C-bussen efter alla adresser"));
  Serial.println(F("#  i2cspeed <kHz>      byt busshastighet, t.ex.  i2cspeed 400"));
  Serial.println(F("#  mute | log          $-ramar av/pa | periodisk logg av/pa"));
  Serial.println(F("#  wdttest             lås processorn -> watchdog ska starta om (V25)"));
}
static void printStat() {
  Serial.print(F("# FW ")); Serial.print(F(FW_VERSION));
  Serial.print(F(" | mode=")); Serial.print(cfg.mode);
  Serial.print(F(" on=")); Serial.print(cfg.enabled);
  Serial.print(F(" spot=")); Serial.print(cfg.spotMax);
  Serial.print(F(" list=")); Serial.print(cfg.listMax);
  Serial.print(F(" hold=")); Serial.print(cfg.holdSec);
  Serial.print(F("s manualmax=")); Serial.print(cfg.manualMaxMin); Serial.println(F("min"));
  Serial.print(F("# lit=")); Serial.print(lit);
  Serial.print(F(" pres=")); Serial.print(presence);
  Serial.print(F(" pir=")); Serial.print(pirStable);
  Serial.print(F(" view=")); Serial.print(view);
  Serial.print(F(" err=")); Serial.print(errCode);
  Serial.print(F(" i2cerr=")); Serial.print(i2cErrTotal);
  Serial.print(F(" eepromw=")); Serial.print(eepromWrites);
  Serial.print(F(" boots=")); Serial.print(ni.boots);
  Serial.print(F(" maxloop=")); Serial.print(loopMax); Serial.println(F("us"));
  Serial.print(F("# minPwm="));
  for (uint8_t s = 0; s < N_CH; s++) { Serial.print(cfg.minPwm[s]); Serial.print(s < 4 ? ',' : ' '); }
  Serial.print(F(" lvl="));
  for (uint8_t s = 0; s < N_CH; s++) { Serial.print(chanCur[s]); Serial.print(s < 4 ? ',' : '\n'); }
}
static void enterTest(uint8_t kind) {
  testMode = true; testKind = kind;
  for (uint8_t s = 0; s < N_CH; s++) { fadeStop(s); settleAt[s] = 0; setChan(s, 0); }
  lit = false;
}
static void leaveTest() {
  testMode = false;
  for (uint8_t s = 0; s < N_CH; s++) { fadeStop(s); settleAt[s] = 0; setChan(s, 0); }
  lit = false;
  Serial.println(F("# testlage av — sekvensmotorn ater i drift"));
}
static void doI2cTest() {
  uint16_t before = i2cErrTotal;
  uint32_t t = micros();
  for (uint16_t i = 0; i < 1000; i++) { pcaWriteOff(0, chanCur[0]); if ((i & 63) == 0) wdt_reset(); }
  t = micros() - t;
  uint32_t per = t / 1000;
  Serial.print(F("# i2ctest @ ")); Serial.print(sclActual() / 1000); Serial.print(F(" kHz: 1000 skrivningar pa "));
  Serial.print(t / 1000); Serial.print(F(" ms = ")); Serial.print(per);
  Serial.print(F(" us/skrivning (krav < 60 us, V48). Fel under testet: "));
  Serial.println(i2cErrTotal - before);
  if (per > 60) Serial.println(F("# > 60 us: pullupen ar for svag (V30, rad 16) eller hastigheten for lag"));
}
/* I2CVERIFY — skriv/läs-tillbaka-test. Skriver slumpvärden till OFF-registret på
   en LEDIG kanal (CH1, ingen lampa) och läser tillbaka dem. Varje avvikelse är en
   bitfel-korrupt överföring som PCA:n kvitterade som om den vore riktig — det
   syns aldrig som "fel" i i2cerr, bara som ett kort felaktigt PWM-värde = flimmer.
   Noll avvikelser på 2 000 skrivningar → bussen är inte orsaken till flimret.  */
static void doI2cVerify() {
  const uint8_t ch = 1;                       // oanvänd kanal
  uint16_t bad = 0, nack = 0, rdErr = 0;
  uint32_t t = micros();
  for (uint16_t i = 0; i < 2000; i++) {
    if ((i & 63) == 0) wdt_reset();
    uint16_t v = (uint16_t)(rand() & 0x0FFF);
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(PCA_REG_LED0_OFF_L + 4 * ch); Wire.write(v & 0xFF); Wire.write(v >> 8);
    if (Wire.endTransmission() != 0) { nack++; continue; }
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(PCA_REG_LED0_OFF_L + 4 * ch);
    if (Wire.endTransmission(false) != 0) { nack++; continue; }
    if (Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)2) != 2) { rdErr++; continue; }
    uint16_t r = Wire.read(); r |= (uint16_t)Wire.read() << 8;
    if (r != v) bad++;
  }
  t = micros() - t;
  Wire.beginTransmission(PCA_ADDR);           // kanalen helt av igen
  Wire.write(PCA_REG_LED0_OFF_L + 4 * ch); Wire.write(0x00); Wire.write(0x10);
  Wire.endTransmission();
  Serial.print(F("# i2cverify @ ")); Serial.print(sclActual() / 1000); Serial.print(F(" kHz: 2000 skriv+las pa "));
  Serial.print(t / 1000); Serial.print(F(" ms | felaktigt tillbakalast=")); Serial.print(bad);
  Serial.print(F(" nack=")); Serial.print(nack); Serial.print(F(" lasfel=")); Serial.println(rdErr);
  if (bad || nack || rdErr) Serial.println(F("# >0: bussen tappar bitar. Prova  i2cspeed 200  och kor om; hjalper det -> pullup/kabel (V30)."));
  else Serial.println(F("# 0 fel: bussen ar ren. Flimmer beror da inte pa I2C — testa  raw  och  pwmhz  (se help)."));
}
/* PWMHZ — byt PCA9685:s PWM-frekvens i drift. Vissa 12 V-spottar har egen
   strömdrivare som inte tål PWM på ingången vid alla frekvenser; om flimret
   försvinner vid en annan frekvens är det lampan, inte styrningen. Tillåtet
   24–1526 Hz. Kanalernas ON-fasspridning påverkas inte.                        */
static void setPwmHz(uint32_t hz) {
  if (hz < 24) hz = 24; if (hz > 1526) hz = 1526;
  uint32_t pre = (25000000UL + (4096UL * hz) / 2) / (4096UL * hz); if (pre) pre--;
  if (pre < 3) pre = 3; if (pre > 255) pre = 255;
  uint16_t keep[N_CH]; for (uint8_t s = 0; s < N_CH; s++) keep[s] = chanCur[s];
  pcaWrite8(PCA_REG_MODE1, 0x10);             // SLEEP=1 — prescale kräver vila
  pcaWrite8(PCA_REG_PRESCALE, (uint8_t)pre);
  pcaWrite8(PCA_REG_MODE1, 0x20); delay(5);
  pcaWrite8(PCA_REG_MODE1, 0xA0);             // RESTART — PWM fortsätter med samma register
  for (uint8_t s = 0; s < N_CH; s++) pcaWriteOff(s, keep[s]);
  Serial.print(F("# prescale ")); Serial.print(pre); Serial.print(F(" -> "));
  Serial.print(25000000UL / (4096UL * (pre + 1))); Serial.println(F(" Hz (aterstalls till 508 Hz vid omstart)"));
}
/* SNIFF — identifierar vilken PCA-kanal som faktiskt kommer fram till en punkt.
   Klipp en lös tråd mellan mätpunkten (t.ex. en D4184-moduls PWM-plint) och en
   ledig UNO-pinne. Koden lägger 50 % duty på en kanal i taget och mäter pulstiden
   på pinnen — den kanal som ger ~50 % och ~509 Hz är den som är ansluten dit.
   Mäter hela kedjan: PCA-stift -> ledare -> modulens ingång.                     */
static void doSniff(uint8_t pin) {
  pinMode(pin, INPUT);
  Serial.print(F("# sniff pa pinne ")); Serial.print(pin);
  Serial.println(F(" — 50 % duty pa en kanal i taget, ca 3 s totalt"));
  enterTest(0);
  int8_t hit = -1;
  for (uint8_t sl = 0; sl < N_CH; sl++) {
    for (uint8_t k = 0; k < N_CH; k++) setChan(k, k == sl ? 2048 : 0);
    delay(40); wdt_reset();
    uint32_t hi = 0, lo = 0; uint8_t got = 0;
    for (uint8_t i = 0; i < 6; i++) {
      unsigned long h = pulseIn(pin, HIGH, 25000UL);
      unsigned long l = pulseIn(pin, LOW, 25000UL);
      if (h && l) { hi += h; lo += l; got++; }
      wdt_reset();
    }
    Serial.print(F("#  CH")); Serial.print(PCA_CH[sl]); Serial.print(F(" (")); Serial.print(CH_NAME[sl]); Serial.print(F("): "));
    if (!got) {
      Serial.println(digitalRead(pin) ? F("konstant HOG — ingen puls") : F("konstant LAG — ingen puls"));
    } else {
      uint32_t per = (hi + lo) / got;
      uint8_t duty = (uint8_t)((hi * 100UL) / (hi + lo));
      Serial.print(duty); Serial.print(F(" % duty, "));
      Serial.print(per ? (1000000UL / per) : 0); Serial.print(F(" Hz"));
      if (duty >= 35 && duty <= 65) { Serial.print(F("   <== DEN HAR ar ansluten hit")); hit = (int8_t)sl; }
      Serial.println();
    }
  }
  for (uint8_t k = 0; k < N_CH; k++) setChan(k, 0);
  if (hit < 0) Serial.println(F("# Ingen kanal hittad. Sitter tradens andra ande pa PWM-plinten? Delar UNO och mätpunkten GND?"));
  else { Serial.print(F("# SVAR: matpunkten far CH")); Serial.print(PCA_CH[hit]); Serial.print(F(" = ")); Serial.println(CH_NAME[hit]); }
  Serial.println(F("# skriv  run  for att lamna testlaget"));
}

static void handleCommand(char *line) {
  if (testKind == 2 && testMode) {           // ramp: valfri rad (aven tom) stoppar
    Serial.print(F("# ramp stoppad vid ")); Serial.print(rampVal);
    Serial.print(F(" — spara med:  min ")); Serial.print(testSlot + 1); Serial.print(' '); Serial.println(rampVal);
    testKind = 0; return;
  }
  while (*line == ' ') line++;
  char *cmd = strtok(line, " ");
  if (!cmd) return;
  for (char *p = cmd; *p; p++) *p = (char)tolower(*p);
  char *a1 = strtok(NULL, " "), *a2 = strtok(NULL, " ");
  if (!strcmp(cmd, "help")) printHelp();
  else if (!strcmp(cmd, "on") || !strcmp(cmd, "off")) {
    uint8_t en = !strcmp(cmd, "on");
    if (en != cfg.enabled) { cfg.enabled = en; manualSince = millis(); markDirty();
      if (!en) { if (adjGroup != 0xFF) endAdjust(false); setLit(false, OFF_MS); } }
    Serial.print(F("# enabled=")); Serial.println(cfg.enabled); sendStatus();
  }
  else if (!strcmp(cmd, "mode") && a1) { uint8_t m = atoi(a1); if (m > 2) { Serial.println(F("# 0..2")); return; }
    if (m != cfg.mode) applyModeChange(m); Serial.print(F("# mode=")); Serial.println(cfg.mode); sendStatus(); }
  else if ((!strcmp(cmd, "spot") || !strcmp(cmd, "list")) && a1) {
    int v = atoi(a1); if (v < 20 || v > 100) { Serial.println(F("# 20..100")); return; }
    if (cmd[0] == 's') { cfg.spotMax = v; previewSpot = v; } else { cfg.listMax = v; previewList = v; }
    markDirty(); applyPreview(); sendStatus(); Serial.println(F("# ok (EEPROM om 5 s)"));
  }
  else if (!strcmp(cmd, "hold") && a1) { long v = atol(a1); if (v < 5 || v > 3600) { Serial.println(F("# 5..3600 s")); return; }
    cfg.holdSec = v; markDirty(); Serial.print(F("# hold=")); Serial.println(cfg.holdSec); }
  else if (!strcmp(cmd, "manualmax") && a1) { long v = atol(a1); if (v < 0) v = 0;
    cfg.manualMaxMin = (uint16_t)((v + 59999L) / 60000L); if (v > 0 && cfg.manualMaxMin == 0) cfg.manualMaxMin = 1;
    manualSince = millis(); markDirty();
    Serial.print(F("# manualmax=")); Serial.print((uint32_t)cfg.manualMaxMin * 60000UL); Serial.println(F(" ms (avrundat till hela minuter)")); }
  else if (!strcmp(cmd, "save")) { bool ok = saveSettings(); Serial.print(F("# EEPROM ")); Serial.println(ok ? F("skriven och verifierad") : F("FEL vid verifiering"));
    if (!ok) errCode = ERR_EEPROM; }
  else if (!strcmp(cmd, "defaults")) { cfgDefaults(); saveSettings(); previewSpot = previewList = 100; applyPreview(); Serial.println(F("# fabriksvarden skrivna")); sendStatus(); }
  else if (!strcmp(cmd, "stat")) printStat();
  else if (!strcmp(cmd, "pir")) { Serial.print(F("# D2=")); Serial.print(digitalRead(PIN_PIR) ? F("HOG (ingen rorelse)") : F("LAG (rorelse)"));
    Serial.print(F(" pres=")); Serial.print(presence); int32_t left = (int32_t)(warmUntil - millis());
    Serial.print(F(" varmKvar=")); Serial.print(left > 0 ? (left + 999) / 1000 : 0);
    Serial.print(F("s sedanRorelse=")); Serial.print((millis() - lastMotionAt) / 1000); Serial.println('s'); }
  else if (!strcmp(cmd, "sim")) { simMotion = true; if ((int32_t)(millis() - warmUntil) < 0) { warmUntil = millis(); Serial.println(F("# uppvarmningen avbruten")); }
    Serial.println(F("# simulerad rorelse")); }
  else if (!strcmp(cmd, "test")) { enterTest(1); testSlot = 0; testNextAt = 0;
    Serial.println(F("# kanaltest: 50 % i 3 s per kanal, M1..M5"));
    Serial.println(F("# matpunkt: modulens PWM-plint mot shieldens GND-plint (= samma nat som signal-GND)")); }
  else if (!strcmp(cmd, "raw") && a1 && a2) { int s = atoi(a1) - 1; long v = atol(a2);
    if (s < 0 || s >= N_CH || v < 0 || v > 4095) { Serial.println(F("# raw <1-5> <0-4095>")); return; }
    if (!testMode || testKind != 0) enterTest(0);
    chanCur[s] = (uint16_t)v; pcaWriteOff(s, (uint16_t)v);
    Serial.print(F("# ")); Serial.print(CH_NAME[s]); Serial.print(F(" = ")); Serial.println(v); }
  else if (!strcmp(cmd, "ramp") && a1) { int s = atoi(a1) - 1; if (s < 0 || s >= N_CH) { Serial.println(F("# ramp <1-5>")); return; }
    enterTest(2); testSlot = s; rampVal = 0; testNextAt = millis();
    Serial.print(F("# TROSKELTEST ")); Serial.print(CH_NAME[s]); Serial.println(F(": stegar +1 var 150 ms. Skicka en tom rad NAR LAMPAN PRECIS SYNS.")); }
  else if (!strcmp(cmd, "min") && a1 && a2) { int s = atoi(a1) - 1; long v = atol(a2);
    if (s < 0 || s >= N_CH || v < 0 || v >= MAX_PWM) { Serial.println(F("# min <1-5> <0-3275>")); return; }
    cfg.minPwm[s] = (uint16_t)v; markDirty(); Serial.print(F("# MIN_PWM ")); Serial.print(CH_NAME[s]); Serial.print(F(" = ")); Serial.println(v); }
  else if (!strcmp(cmd, "sniff") && a1) {
    uint8_t pin = 0xFF;
    if (a1[0] == 'a' || a1[0] == 'A') { uint8_t k = a1[1] - '0'; if (k < 4) pin = A0 + k; }
    else { int v = atoi(a1); if ((v >= 3 && v <= 6) || (v >= 9 && v <= 12)) pin = (uint8_t)v; }
    if (pin == 0xFF) Serial.println(F("# ledig pinne: A0 A1 A2 A3 eller 3 4 5 6 9 10 11 12 (D0/D1/D2/D7/D8/D13 ar upptagna)"));
    else doSniff(pin);
  }
  else if (!strcmp(cmd, "run")) leaveTest();
  else if (!strcmp(cmd, "i2ctest")) doI2cTest();
  else if (!strcmp(cmd, "i2cverify")) doI2cVerify();
  else if (!strcmp(cmd, "pwmhz") && a1) setPwmHz((uint32_t)atol(a1));
  else if (!strcmp(cmd, "scan")) i2cScan();
  else if (!strcmp(cmd, "i2cspeed") && a1) {
    uint32_t hz = (uint32_t)atol(a1) * 1000UL; if (hz < 10000UL) hz = 10000UL;
    Wire.setClock(hz); i2cHz = hz;
    Serial.print(F("# begart ")); Serial.print(hz / 1000); Serial.print(F(" kHz -> verkligt "));
    Serial.print(sclActual() / 1000); Serial.print(F(" kHz (TWBR=")); Serial.print(TWBR); Serial.println(')');
    if (TWBR < 10) Serial.println(F("# OBS: TWBR<10 — utanfor databladets master-spec"));
    Serial.println(pcaPing() ? F("# 0x40 svarar pa denna hastighet") : F("# INGET SVAR pa denna hastighet"));
  }
  else if (!strcmp(cmd, "mute")) { muteFrames = !muteFrames; Serial.print(F("# mute=")); Serial.println(muteFrames); }
  else if (!strcmp(cmd, "log")) { logOn = !logOn; Serial.print(F("# log=")); Serial.println(logOn); }
  else if (!strcmp(cmd, "wdttest")) {
#if WDT_ENABLE
    Serial.println(F("# laser processorn — omstart inom 2 s. Bannern ska komma tillbaka och"));
    Serial.println(F("# saga 'omstarten kom fran WATCHDOGEN'. En kort orange blink ar Optiboot."));
    Serial.flush(); ni.wdtFlag = 1; for (;;) {}
#else
    Serial.println(F("# WDT_ENABLE ar 0 — testet hoppas over"));
#endif
  }
  else Serial.println(F("# okant kommando — skriv help"));
}
static char cmdBuf[40]; static uint8_t cmdLen = 0;
static void pollConsole() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen || (testMode && testKind == 2)) { cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); }
      cmdLen = 0;
    } else if (cmdLen < sizeof cmdBuf - 1) cmdBuf[cmdLen++] = c;
  }
}
static void testService() {
  uint32_t now = millis();
  if (testKind == 1) {                       // kanaltest
    if (testNextAt == 0 || (int32_t)(now - testNextAt) >= 0) {
      if (testNextAt) { setChan(testSlot, 0); testSlot++; }
      if (testSlot >= N_CH) { Serial.println(F("# kanaltest klart")); leaveTest(); return; }
      setChan(testSlot, 2048);               // 50 % (≈2,5 V DC på PCA-stiftet)
      Serial.print(F("# CH")); Serial.print(PCA_CH[testSlot]); Serial.print(F(" → ")); Serial.println(CH_NAME[testSlot]);
      testNextAt = now + 3000;
    }
  } else if (testKind == 2) {                // ramp
    if ((int32_t)(now - testNextAt) >= 0) {
      testNextAt = now + 150;
      if (rampVal >= MAX_PWM) { Serial.println(F("# ramp nadde taket utan stopp")); testKind = 0; return; }
      rampVal++; setChan(testSlot, rampVal);
      if ((rampVal % 5) == 0) { Serial.print(F("# ramp ")); Serial.println(rampVal); }
    }
  }
}

/* --------------------------------------------------------------------------
 *  10. SETUP
 * -------------------------------------------------------------------------- */
void setup() {
  MCUSR = 0; wdt_disable();                // säker start efter WDT-omstart
  bool wdtRestart = false;
  if (ni.magic == NI_MAGIC) { ni.boots++; wdtRestart = ni.wdtFlag; ni.wdtFlag = 0; }
  else { ni.magic = NI_MAGIC; ni.boots = 1; ni.wdtFlag = 0; }

  pinMode(PIN_PIR, INPUT);                 // extern pullup 4,7 k finns på protoytan
  Serial.begin(115200);
  Serial.println();
  Serial.print(F("# GARDEROBSBELYSNING ")); Serial.println(F(FW_VERSION));
  Serial.println(F("# kanaler: CH15->M1 CH11->M2 CH7->M3 CH3->M4 CH0->M5 · prescale 11 (508,6 Hz)"));
  Serial.print(F("# start #")); Serial.print(ni.boots); Serial.println(F(" sedan stromtillslag (varje reset raknas — aven nar seriemonitorn oppnas)"));
  if (wdtRestart) Serial.println(F("# ^ den omstarten kom fran WATCHDOGEN — V25 GODKAND"));

  // Vilonivåer innan Wire tar över stiften (Wire.begin slår på interna pullups)
  pinMode(A4, INPUT); pinMode(A5, INPUT);
  delayMicroseconds(200);
  if (digitalRead(A4) == LOW || digitalRead(A5) == LOW)
    Serial.println(F("# VARNING: SDA eller SCL ligger LAGT i vila — ingen 5 V pa PCA-modulen, eller kortslutning"));

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true);   // en hängd buss får aldrig stoppa loopen (se definitionen)
#endif
  if (i2cProbe() && pcaInit()) {
    Serial.print(F("# PCA9685 svarar pa 0x40 vid ")); Serial.print(sclActual() / 1000);
    Serial.print(F(" kHz (TWBR=")); Serial.print(TWBR); Serial.println(F(") — prescale satt, alla kanaler av"));
    if (TWBR < 10) Serial.println(F("# OBS: TWBR<10 ligger utanfor ATmega328P:s master-spec — kor i2ctest"));
    if (sclActual() < 380000UL) {
      Serial.println(F("# VARNING: bussen gick bara att kora under 400 kHz. Toningen behover"));
      Serial.println(F("#   ca 10 000 skrivningar/s i kaskadens topp och kommer att slapa."));
      Serial.println(F("#   Atgard: lod rad 16 (2x 4,7 kOhm SDA->5V och SCL->5V) enligt V30."));
    }
  } else {
    errCode = ERR_PCA; pcaRetryAt = millis() + 3000;
    Serial.println(F("# FEL 1: PCA9685 svarar inte pa 0x40 vid nagon hastighet (800/400/200/100 kHz)."));
    Wire.setClock(100000UL);
    i2cScan();
    Serial.println(F("# Kor testprogrammet i2c_diagnos.ino — det pekar ut orsaken."));
  }

  EEPROM.get(0, cfg);
  if (cfgValid(cfg)) {
    Serial.print(F("# EEPROM last: mode=")); Serial.print(cfg.mode); Serial.print(F(" on=")); Serial.print(cfg.enabled);
    Serial.print(F(" spot=")); Serial.print(cfg.spotMax); Serial.print(F(" list=")); Serial.print(cfg.listMax);
    Serial.print(F(" hold=")); Serial.print(cfg.holdSec); Serial.println('s');
  } else {
    cfgDefaults();
    bool ok = saveSettings();
    Serial.print(F("# EEPROM ogiltig — fabriksvarden skrivna (KASKAD, pa, 100/100): "));
    Serial.println(ok ? F("verifierade") : F("VERIFIERING MISSLYCKADES (fel 2)"));
    if (!ok) errCode = ERR_EEPROM;
  }
  previewSpot = cfg.spotMax; previewList = cfg.listMax;
  spotCap = (uint16_t)((uint32_t)MAX_PWM * previewSpot / 100UL);
  listCap = (uint16_t)((uint32_t)MAX_PWM * previewList / 100UL);

  up.begin(9600);
  listening = true;
  bootMs = millis();
  warmUntil = bootMs + (uint32_t)PIR_WARM_S * 1000UL;
  lastMotionAt = bootMs;
  manualSince = bootMs;
  view = computeView();
  sendStatus();
  Serial.println(F("# klar. skriv  help  for kommandon"));
#if WDT_ENABLE
  wdt_enable(WDTO_2S);                     // F6 · V25
#endif
  loopPrev = micros();
}

/* --------------------------------------------------------------------------
 *  11. LOOP
 * -------------------------------------------------------------------------- */
void loop() {
  wdt_reset();
  uint32_t now = millis();

  // --- PIR: avläs, avstuda, uppvärmning ---
  bool raw = (digitalRead(PIN_PIR) == LOW) || simMotion;
  if (raw != pirRaw) { pirRaw = raw; pirEdgeAt = now; }
  if (now - pirEdgeAt >= PIR_DEBOUNCE_MS || simMotion) pirStable = pirRaw;
  bool warm = (int32_t)(now - warmUntil) < 0;
  if (pirStable && !warm) { lastMotionAt = now; if (!presence) { presence = true; view = computeView(); sendStatus(); } }
  simMotion = false;
  if (presence && (int32_t)(now - lastMotionAt) > (int32_t)cfg.holdSec * 1000L) { presence = false; view = computeView(); sendStatus(); }

  // --- PCA9685 återhämtning ---
  if (errCode == ERR_PCA && (int32_t)(now - pcaRetryAt) >= 0) {
    pcaRetryAt = now + 3000;
    if (i2cProbe() && pcaInit()) {
      errCode = ERR_NONE; lit = false;
      Serial.print(F("# PCA9685 ater i kontakt vid ")); Serial.print(sclActual() / 1000); Serial.println(F(" kHz"));
      sendStatus();
    }
  }

  if (testMode) {
    testService();
  } else {
    // --- auto-återgång MANUELL → KASKAD (§11) ---
    if (cfg.enabled && cfg.mode == MODE_MANUAL && cfg.manualMaxMin &&
        (int32_t)(now - manualSince) > (int32_t)cfg.manualMaxMin * 60000L) {
      cfg.mode = MODE_CASCADE; manualSince = now; markDirty();
      Serial.println(F("# auto-atergang: MANUELL -> KASKAD")); sendStatus();
    }
    // --- JUSTERA-timeout: 20 s utan händelse → återställ utan skrivning ---
    if (adjGroup != 0xFF && (int32_t)(now - lastAdjEvent) > (int32_t)ADJ_TIMEOUT_MS) { endAdjust(false); sendStatus(); }
    // --- av/på-grinden och sekvensmotorn ---
    bool want = wantLit();
    if (want != lit) setLit(want, DOWN_MS);
    fadeService();
  }

  // --- sekvensfönstret och lyssningsfönstret (F8) ---
  // seqActive = någon toning pågår ($P i 20 Hz, vy SEKVENS). Upplänken stängs bara
  // under tänd-/släcksekvenser (bigSeq): SoftwareSerial stänger avbrotten ~1 ms per
  // mottaget tecken och skulle rycka i kaskadens tidtabell. Under nivåförhandsvisning
  // (reglaget) hålls den öppen — annars når bara var tredje reglagerörelse fram.
  bool active = anyFadeActive();
  if (active && !seqActive) { seqActive = true; loopMaxSeq = 0; i2cErrSeq = 0; }
  if (!active && seqActive) {
    seqActive = false; bigSeq = false;
    sendLevels();                          // slutnivåerna
    if (logOn) { Serial.print(F("# sekvens klar: maxloop=")); Serial.print(loopMaxSeq); Serial.print(F(" us, i2c-fel=")); Serial.println(i2cErrSeq); }
  }
  bool wantListen = !(seqActive && bigSeq);
  if (wantListen && !listening) { up.listen(); listening = true; }
  if (!wantListen && listening) { up.stopListening(); listening = false; }
  if (listening) pollUplink();
  pollConsole();

  // --- ramar ut ---
  // OBS: tidsstämplarna kan ha satts EFTER att 'now' lästes (i pollUplink) — därför
  // signerade jämförelser, annars blir "now − stämpel" ett jättetal och ramen går
  // ut två gånger (och EEPROM-fördröjningen på 5 s föll bort på samma sätt).
  uint8_t v = computeView();
  if (v != view) { view = v; sendStatus(); }
  if ((int32_t)(now - lastStatusAt) >= (int32_t)STATUS_PERIOD_MS) sendStatus();
  if (seqActive && (int32_t)(now - lastPAt) >= (int32_t)P_PERIOD_MS) sendLevels();
  // Utanför sekvens: en långsam $P så att panelen (och accentlisten) alltid har
  // en aktuell bild av vad lamporna gör — annars vet den inget efter en omstart,
  // eller när ljuset stått stilla länge. Panelen skiljer på strömmande 20 Hz och
  // den här vilopulsen, så bakgrundsljuset påverkas inte.
  else if (!seqActive && (int32_t)(now - lastPAt) >= (int32_t)P_IDLE_MS) sendLevels();

  // --- EEPROM fördröjd skrivning ---
  // Aldrig mitt i en sekvens: EEPROM.put på 21 byte tar upp till 70 ms, och under
  // den tiden står toningsmotorn stilla. Det syns som ett hack i ljuset.
  if (eepromDirty && !seqActive && (int32_t)(now - eepromDirtyAt) >= (int32_t)EEPROM_DELAY_MS) {
    if (!saveSettings()) errCode = ERR_EEPROM;
    if (logOn) { Serial.print(F("# EEPROM skriven (#")); Serial.print(eepromWrites); Serial.println(')'); }
  }

  // --- looptidsmätaren (V26) ---
  uint32_t t = micros(), dt = t - loopPrev; loopPrev = t;
  if (dt > loopMax) loopMax = dt;
  if (seqActive && dt > loopMaxSeq) loopMaxSeq = dt;
  if (logOn && !seqActive && now - lastLogAt >= LOG_PERIOD_MS) {
    lastLogAt = now;
    Serial.print(F("# t=")); Serial.print((now - bootMs) / 1000);
    Serial.print(F("s mode=")); Serial.print(cfg.mode);
    Serial.print(cfg.enabled ? F(" on=1(TAND)") : F(" on=0(SLACKT: PIR kors men PWM ar grindad)"));
    Serial.print(F(" pir=")); Serial.print(pirStable); Serial.print(F(" pres=")); Serial.print(presence);
    if (presence) {                            // nedräkning till slocktid, syns i loggen
      uint32_t gone = (now - lastMotionAt) / 1000UL;
      Serial.print(F(" slacker_om="));
      Serial.print(gone >= cfg.holdSec ? 0 : cfg.holdSec - (uint16_t)gone); Serial.print('s');
    }
    Serial.print(F(" warm=")); Serial.print(warm); Serial.print(F(" view=")); Serial.print(view);
    Serial.print(F(" lvl=")); for (uint8_t s = 0; s < N_CH; s++) { Serial.print(pctOf(chanCur[s])); Serial.print(s < 4 ? ',' : ' '); }
    Serial.print(F("maxloop=")); Serial.print(loopMax); Serial.print(F("us i2cerr=")); Serial.print(i2cErrTotal);
    Serial.print(F(" err=")); Serial.println(errCode);
    loopMax = 0;
    loopPrev = micros();                   // utskriften ska inte räknas som looptid
  }
}
