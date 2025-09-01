/* =========================================================
   ESP32 RC BOT – MX1508 + SG90 + 2S Batarya + Web Arayüz
   v2 (yön uzunluğu=Gaz, %100=8.25V, kalibrasyon + EMA, şarj bildirimi yok)
   ========================================================= */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <math.h>  // fabsf için

/* ====== EEPROM Ayarları ====== */
#define EEPROM_SIZE 64
#define TRIM_ADDR_THROTTLE 0     // Gaz trim (int, 4 byte)
#define STEERING_ADDR_ANGLE 4    // Yön servo açısı (int, 4 byte)
#define SERVO_ADDR_MIN 8         // Servo min açı (int, 4 byte)
#define SERVO_ADDR_MAX 12        // Servo max açı (int, 4 byte)
#define MAGIC_ADDR 16            // Magic number (int, 4 byte) - ilk kurulum kontrolü
#define MAGIC_NUMBER 0x12345678

/* ====== Trim ve Servo Ayarları ====== */
int g_throttleTrim = 0;     // Gaz trim: -100 ile +100 arası
int g_steeringAngle = 90;   // Yön servo açısı: 0-180 derece
int g_servoMinAngle = 60;   // Servo minimum açı
int g_servoMaxAngle = 120;  // Servo maksimum açı

// Geçici trim değerleri (slider hareket ederken kullanılır)
int g_tempThrottleTrim = 0;
int g_tempSteeringAngle = 90;
int g_tempServoMinAngle = 60;
int g_tempServoMaxAngle = 120;

/* ====== Wi-Fi (AP) ====== */
const char* AP_SSID = "TEKNE_AP";
const char* AP_PASS = "12345678";

/* ====== Pinler ====== */
const int IN1       = 26;   // MX1508 IN1
const int IN2       = 25;   // MX1508 IN2
const int SERVO_PIN = 13;   // SG90 servo sinyal
const int ADC_PIN   = 34;   // Batarya ölçüm (ADC)

/* ====== Servo Ayarları ====== */
Servo rudder;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;
const int SERVO_CENTER = 90; // Orta açı

/* ====== Batarya Kalibrasyonu / Eşikler ====== */



/* Dinamik EMA + Yük kompanzasyonu ayarları */
const float LOAD_COMP_V_AT_100 = 0.22f;   // %100 gazda ~+0.22 V telafi (tune edilebilir)
const float EMA_ALPHA_MIN = 0.06f;        // yüksek yükte daha yavaş filtre
const float EMA_ALPHA_MAX = 0.18f;        // yük yokken daha hızlı

/* Kritik kesme histerezisi (zamanlı) */
const int      CRIT_CLEAR_PCT   = 15;      // kilidi bırakma eşiği
const uint32_t CRIT_ENTER_MS    = 1500;    // %10 altı en az 1.5 s → kilit
const uint32_t CRIT_CLEAR_MS    = 3000;    // %15 üstü en az 3 s → kilit çöz
volatile bool      g_critLock    = false;
volatile uint32_t  g_critEnterT0 = 0;
volatile uint32_t  g_critClearT0 = 0;





/* Ölçü aleti 8.42 V, ESP 8.30 V → ölçek düzeltmesi ~ 8.42/8.30 = 1.0145 */
const float R1 = 220000.0f, R2 = 100000.0f;
const float ADC_REF = 3.30f;
const int   ADC_MAX = 4095;
/* İNCE AYAR: CAL_A’yı 1.012–1.018 aralığında gerekirse değiştir. */
const float CAL_A = 0.97308f;     // ölçek düzeltme
const float CAL_B = 0.105f;     // ofset düzeltme (gerekirse ~±0.03 ekleyebilirsin)
const float BMS_CUTOFF_V = 6.00; // 2S güvenlik
const int   CRIT_BATT_PCT = 10;  // kritik % (gaz kes)
const float FULL_100_V = 8.25f;  // %100 kabul gerilimi

/* SoC Tablosu (2S Li-ion, yaklaşık, orta noktalar korunur) */
const int NPTS = 11;
const float socVolt[NPTS] = {8.25,8.10,7.95,7.80,7.60,7.40,7.20,7.00,6.80,6.60,6.40};
const float socPerc[NPTS] = {100,  90,  80,  70,  60,  50,  40,  30,  20,  10,   5};

/* ====== EMA (yumuşatma) ======
   α küçük oldukça daha sakin görüntü. 0.10–0.20 önerilir. */
const float EMA_ALPHA = 0.12f;
float g_vFilt = NAN;   // filtreli voltaj
float g_pctFilt = NAN; // filtreli yüzde

/* ====== Durum ====== */
volatile int g_thrPct = 0; // -100..+100
volatile int g_strPct = 0; // -100..+100
int lastLoggedThr = 999, lastLoggedStr = 999;

AsyncWebServer server(80);

/* ====== Yardımcılar ====== */
static inline int map_i(int x, int in_min, int in_max, int out_min, int out_max) {
  return (int)((long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void setMotor(int speed) { // -255..+255
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(IN1, speed);
    analogWrite(IN2, 0);
  } else if (speed < 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, -speed);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }
}

/* ====== EEPROM Fonksiyonları ====== */
void loadSettingsFromEEPROM() {
  int magic;
  EEPROM.get(MAGIC_ADDR, magic);
  
  if (magic == MAGIC_NUMBER) {
    // Ayarlar daha önce kaydedilmiş, yükle
    EEPROM.get(TRIM_ADDR_THROTTLE, g_throttleTrim);
    EEPROM.get(STEERING_ADDR_ANGLE, g_steeringAngle);
    EEPROM.get(SERVO_ADDR_MIN, g_servoMinAngle);
    EEPROM.get(SERVO_ADDR_MAX, g_servoMaxAngle);
    
    // Güvenlik kontrolü
    g_throttleTrim = constrain(g_throttleTrim, -100, 100);
    g_steeringAngle = constrain(g_steeringAngle, 0, 180);
    g_servoMinAngle = constrain(g_servoMinAngle, 0, 180);
    g_servoMaxAngle = constrain(g_servoMaxAngle, 0, 180);
    
    // Geçici değerleri de güncelle
    g_tempThrottleTrim = g_throttleTrim;
    g_tempSteeringAngle = g_steeringAngle;
    g_tempServoMinAngle = g_servoMinAngle;
    g_tempServoMaxAngle = g_servoMaxAngle;
    
    Serial.println("Ayarlar EEPROM'dan yüklendi:");
    Serial.printf("Gaz Trim: %d, Yön Açısı: %d°\n", g_throttleTrim, g_steeringAngle);
    Serial.printf("Servo Min: %d°, Max: %d°\n", g_servoMinAngle, g_servoMaxAngle);
  } else {
    // İlk kurulum, varsayılan ayarları kaydet
    saveSettingsToEEPROM();
    Serial.println("İlk kurulum - varsayılan ayarlar kaydedildi");
  }
}

void saveSettingsToEEPROM() {
  EEPROM.put(TRIM_ADDR_THROTTLE, g_throttleTrim);
  EEPROM.put(STEERING_ADDR_ANGLE, g_steeringAngle);
  EEPROM.put(SERVO_ADDR_MIN, g_servoMinAngle);
  EEPROM.put(SERVO_ADDR_MAX, g_servoMaxAngle);
  EEPROM.put(MAGIC_ADDR, MAGIC_NUMBER);
  
  EEPROM.commit();
  Serial.println("Ayarlar EEPROM'a kaydedildi");
}

float readBatteryVoltageRaw() {
  // 5 grup × 16 örnek: her grubun ortalamasını al, sonra ortalamaların MEDYAN’ını al.
  const int GROUPS = 5, NS = 16;
  uint16_t grp[GROUPS];

  for (int g = 0; g < GROUPS; g++) {
    uint32_t acc = 0;
    for (int i = 0; i < NS; i++) {
      acc += analogReadMilliVolts(ADC_PIN);   // eFuse kalibrasyonlu mV
      delayMicroseconds(80);
    }
    grp[g] = acc / NS; // mV
  }
  // küçük diziyi sırala → medyan
  for (int i=0;i<GROUPS-1;i++) for (int j=i+1;j<GROUPS;j++) if (grp[j]<grp[i]) { uint16_t t=grp[i]; grp[i]=grp[j]; grp[j]=t; }
  float adc_mV = grp[GROUPS/2];

  float vBat = (adc_mV / 1000.0f) * ((R1 + R2) / R2); // bölücü geri hesap
  return CAL_A * vBat + CAL_B;                        // kalibrasyon
}


float socFromVoltage(float v) {
  if (v >= FULL_100_V) return 100.0f;        // %100 = 8.25V ve üstü
  if (v <= BMS_CUTOFF_V) return 0.0f;
  for (int i = 0; i < NPTS - 1; i++) {
    if (v <= socVolt[i] && v >= socVolt[i + 1]) {
      float t = (v - socVolt[i + 1]) / (socVolt[i] - socVolt[i + 1]);
      return socPerc[i + 1] + t * (socPerc[i] - socPerc[i + 1]);
    }
  }
  return 0.0f;
}

/* EMA filtresi: her çağrıda ham ölçümü alır, g_vFilt/g_pctFilt günceller */
uint32_t g_lastBattMs = 0;

void sampleBatteryFiltered(float &vOut, int &pctOut) {
  uint32_t now = millis();
  // En fazla 200 ms'de bir ham örnek al
  if (now - g_lastBattMs >= 200 || isnan(g_vFilt)) {
    g_lastBattMs = now;
    float v = readBatteryVoltageRaw();

    // Slew limit: bir örneklemede en fazla ±0.05 V değişsin
    if (isnan(g_vFilt)) g_vFilt = v;
    float dv = v - g_vFilt;
    if (dv > 0.05f) dv = 0.05f;
    if (dv < -0.05f) dv = -0.05f;
    g_vFilt += dv;

    // Yüzde hesabı + hafif EMA
    float pct = socFromVoltage(g_vFilt);
    if (isnan(g_pctFilt)) g_pctFilt = pct;
    g_pctFilt = g_pctFilt + 0.20f * (pct - g_pctFilt);
  }

  vOut  = g_vFilt;
  pctOut = (int)roundf(g_pctFilt);
}


void setSteerPct(int pct) {
  pct = constrain(pct, -100, 100);
  g_strPct = pct;
  
  // Yön kontrolü: pct değerine göre servo açısını ayarla
  // pct = 0 olduğunda g_tempSteeringAngle kullan
  // pct != 0 olduğunda min-max arasında haritalama yap
  int targetAngle;
  if (pct == 0) {
    targetAngle = g_tempSteeringAngle; // Trim açısı
  } else {
    targetAngle = map_i(pct, -100, 100, g_tempServoMinAngle, g_tempServoMaxAngle);
  }
  
  rudder.write(targetAngle);
}

void setMotorWithTrim(int pct) {
  pct = constrain(pct, -100, 100);
  
  // Geçici trim değerini kullan
  int adjustedPct = pct + g_tempThrottleTrim;
  adjustedPct = constrain(adjustedPct, -100, 100);
  
  // Motor PWM değerine çevirme
  setMotor(map_i(adjustedPct, -100, 100, -255, 255));
}

/* ====== HTML (UTF-8) – 16:9 + Pointer Capture; şarj bildirimi KALDIRILDI ====== */
const char index_html[] PROGMEM = R"HTML(
<!doctype html><html lang="tr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>RC Bot Kontrol</title>
<style>
  :root{
    --bg:#0f0f10; --panel:#141416; --slot:#0b0b0c; --border:#ffffff;
    --knob:#d9d9dc; --ink:#e9e9ea; --muted:#9aa0a6; --ok:#2ecc71; --warn:#e53935; --accent:#19a0ff;
  }
  *{box-sizing:border-box; -webkit-tap-highlight-color:transparent}
  html,body{height:100%;margin:0;background:var(--bg);color:var(--ink);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;overflow:hidden;user-select:none}
  .stageWrap{position:fixed;inset:0;display:grid;place-items:center}
  .stage{position:relative;background:var(--panel);border:3px solid var(--border);border-radius:20px;aspect-ratio:16/9;width:min(100vw,100vh*16/9);height:min(100vh,100vw*9/16);}
  .topbar{position:absolute;left:2.5%;right:2.5%;top:2.5%;display:flex;gap:16px;align-items:center;justify-content:space-between}
  .status{display:flex;gap:10px;align-items:center;font-weight:600}
  .dot{width:14px;height:14px;border-radius:50%;background:#777;box-shadow:0 0 0 2px rgba(255,255,255,.15) inset}
  .online .dot{background:var(--ok)} .offline .dot{background:#d9534f}
  .battWrap{display:flex;gap:12px;align-items:center}
  .batt{position:relative;width:160px;height:32px;border:2px solid #555;border-radius:8px;background:#0b0b0c;overflow:hidden}
  .batt::after{content:"";position:absolute;right:-10px;top:7px;width:8px;height:18px;border:2px solid #555;border-left:none;border-radius:0 4px 4px 0}
  .fill{position:absolute;left:0;top:0;bottom:0;width:0%;}
  .fill.green{background:linear-gradient(90deg,#57e389,#2ecc71)}
  .fill.red{background:linear-gradient(90deg,#ff8a80,#e53935)}
  .btext{display:flex;flex-direction:column;line-height:1.1}
  .small{color:var(--muted);font-size:12px}
  .fsBtn{width:44px;height:44px;border-radius:12px;border:2px solid #71d2ff;display:grid;place-items:center}
  .fsIcon{width:22px;height:22px;border:3px solid var(--accent);border-radius:3px}
  .pad{position:absolute;background:var(--slot);border:3px solid var(--border);border-radius:36px}
  .knob{position:absolute;display:grid;place-items:center;background:var(--knob);border-radius:50%;width:1px;height:1px;cursor:grab}
  .knob:active{cursor:grabbing}
  .label{
  position:absolute; color:#fff; font-size:28px; font-weight:800; letter-spacing:.04em;
  transform: translate(-50%, 0px);  /* -8px yukarı iter (istediğin kadar değiştir) */
  text-align:center;
}

  .pad, .knob{touch-action:none}
  
  /* Trim Paneli Stilleri */
  .trimPanel{
    position:fixed;top:10%;left:10%;right:10%;bottom:10%;
    background:var(--panel);border:3px solid var(--border);border-radius:20px;
    padding:20px;display:none;z-index:2000;
    overflow-y:auto;
  }
  .trimHeader{
    display:flex;justify-content:space-between;align-items:center;
    margin-bottom:20px;padding-bottom:10px;
    border-bottom:2px solid var(--border);
  }
  .trimTitle{color:var(--ink);font-size:24px;font-weight:700;margin:0}
  .closeBtn{
    width:40px;height:40px;border:none;background:var(--warn);color:white;
    border-radius:50%;cursor:pointer;font-size:24px;font-weight:bold;
    display:grid;place-items:center;
  }
  .closeBtn:hover{opacity:0.8}
  .trimBtn{width:44px;height:44px;border-radius:12px;border:2px solid #71d2ff;background:var(--panel);color:var(--accent);cursor:pointer;display:grid;place-items:center;font-weight:bold;font-size:18px}
  .trimBtn:hover{background:var(--accent);color:var(--panel)}
  .trimContent{display:grid;grid-template-columns:1fr 1fr;gap:20px;margin-bottom:20px}
  .trimGroup{background:var(--slot);border:2px solid var(--border);border-radius:12px;padding:15px}
  .trimLabel{color:var(--ink);font-size:16px;margin-bottom:8px;font-weight:600}
  .trimSlider{width:100%;margin:8px 0;height:8px;border-radius:4px;background:var(--slot);outline:none;appearance:none}
  .trimSlider::-webkit-slider-thumb{width:20px;height:20px;border-radius:50%;background:var(--accent);cursor:pointer;appearance:none}
  .trimSlider::-moz-range-thumb{width:20px;height:20px;border-radius:50%;background:var(--accent);cursor:pointer;border:none}
  .trimValue{color:var(--accent);font-size:14px;text-align:center;font-weight:600;margin-top:5px}
  .saveBtn{width:100%;padding:15px;background:var(--ok);color:white;border:none;border-radius:12px;cursor:pointer;font-weight:700;font-size:18px}
  .saveBtn:hover{opacity:0.8}
  .trimFooter{text-align:center;border-top:2px solid var(--border);padding-top:15px}
  
  @media (max-width: 768px) {
    .trimContent{grid-template-columns:1fr;gap:15px}
    .trimPanel{padding:15px}
  }
</style>
</head>
<body>
<div class="stageWrap"><div class="stage" id="stage">
  <div class="topbar">
    <div class="status offline" id="netStat"><span class="dot"></span><span id="netTxt">Çevrimdışı</span></div>
    <div class="battWrap">
      <div class="batt" aria-label="Batarya"><div class="fill green" id="bfill"></div></div>
      <div class="btext"><span id="bpct" style="font-weight:800">--%</span><span class="small" id="bvolt">--.-- V</span></div>
    </div>
    <div style="display:flex;gap:8px;align-items:center">
      <div class="trimBtn" id="trimBtn" title="Trim Ayarları">⚙</div>
      <div class="fsBtn" id="fsBtn" title="Tam ekran"><div class="fsIcon"></div></div>
    </div>
  </div>
  
  <!-- Trim Paneli -->
  <div class="trimPanel" id="trimPanel">
    <div class="trimHeader">
      <h2 class="trimTitle">Trim Ayarları</h2>
      <button class="closeBtn" id="closeBtn" title="Kapat">×</button>
    </div>
    
    <div class="trimContent">
      <div class="trimGroup">
        <div class="trimLabel">Gaz Trim</div>
        <input type="range" class="trimSlider" id="throttleTrim" min="-100" max="100" value="0">
        <div class="trimValue" id="throttleTrimValue">0</div>
      </div>
      
      <div class="trimGroup">
        <div class="trimLabel">Yön Servo Açısı</div>
        <input type="range" class="trimSlider" id="steeringAngle" min="0" max="180" value="90">
        <div class="trimValue" id="steeringAngleValue">90°</div>
      </div>
      
      <div class="trimGroup">
        <div class="trimLabel">Servo Min Açı</div>
        <input type="range" class="trimSlider" id="servoMin" min="0" max="180" value="60">
        <div class="trimValue" id="servoMinValue">60°</div>
      </div>
      
      <div class="trimGroup">
        <div class="trimLabel">Servo Max Açı</div>
        <input type="range" class="trimSlider" id="servoMax" min="0" max="180" value="120">
        <div class="trimValue" id="servoMaxValue">120°</div>
      </div>
    </div>
    
    <div class="trimFooter">
      <button class="saveBtn" id="saveTrim">KALICI KAYDET</button>
    </div>
  </div>

  <!-- SOL: GAZ (dikey slot) -->
  <div class="pad" id="thrPad" aria-label="Gaz"></div>
  <div class="knob" id="thrKnob"></div>
  <div class="label" id="thrLbl">GAZ</div>

  <!-- SAĞ: YÖN (yatay slot) -->
  <div class="pad" id="strPad" aria-label="Yön"></div>
  <div class="knob" id="strKnob"></div>
  <div class="label" id="strLbl">YÖN</div>
</div></div>

<script>
/* ==== Yerleşim ve ölçekleme ==== */
const stage = document.getElementById('stage');
const thrPad = document.getElementById('thrPad'), thrKnob = document.getElementById('thrKnob'), thrLbl = document.getElementById('thrLbl');
const strPad = document.getElementById('strPad'), strKnob = document.getElementById('strKnob'), strLbl = document.getElementById('strLbl');
const fsBtn = document.getElementById('fsBtn');
const bfill = document.getElementById('bfill'), bpct = document.getElementById('bpct'), bvolt = document.getElementById('bvolt');
const netStat = document.getElementById('netStat'), netTxt = document.getElementById('netTxt');

/* ==== Trim Kontrolleri ==== */
const trimBtn = document.getElementById('trimBtn');
const trimPanel = document.getElementById('trimPanel');
const closeBtn = document.getElementById('closeBtn');
const throttleTrim = document.getElementById('throttleTrim');
const steeringAngle = document.getElementById('steeringAngle');
const servoMin = document.getElementById('servoMin');
const servoMax = document.getElementById('servoMax');
const saveTrim = document.getElementById('saveTrim');

let geom = {};

function layout(){
  const W = stage.clientWidth, H = stage.clientHeight;

  const padBorder = 3;
  const knobD = Math.round(Math.min(W,H) * 0.18);
  const thrPadW = Math.round(W * 0.18), thrPadH = Math.round(H * 0.70);

  // YÖN slotu: uzunluğu GAZ ile aynı
  const strPadW = thrPadH;
  const strPadH = thrPadW;

  // Sol dikey pad
  thrPad.style.width = thrPadW+"px"; thrPad.style.height = thrPadH+"px";
  thrPad.style.left = Math.round(W*0.05)+"px"; thrPad.style.top = Math.round(H*0.17)+"px";
  thrPad.style.borderRadius = Math.round(thrPadW*0.35)+"px";

  // Sağ yatay pad
  strPad.style.width = strPadW+"px"; strPad.style.height = strPadH+"px";
  strPad.style.left = Math.round(W - strPadW - W*0.05)+"px";
  strPad.style.top  = Math.round(H*0.40)+"px";  // daha küçük => daha yukarı
  strPad.style.borderRadius = Math.round(strPadH*0.35)+"px";

  // Knob boyutu
  [thrKnob,strKnob].forEach(k=>{
    k.style.width = knobD+"px"; k.style.height = knobD+"px";
    k.style.boxShadow="0 4px 12px rgba(0,0,0,.35) inset,0 4px 10px rgba(0,0,0,.25)";
  });

  // Etiketler
  const labelGap = Math.round(H*0.02);
  thrLbl.style.left = (thrPad.offsetLeft + thrPadW/2) + "px";
  thrLbl.style.top  = (thrPad.offsetTop + thrPadH + labelGap) + "px";

  strLbl.style.left = (strPad.offsetLeft + strPadW/2) + "px";
  strLbl.style.top  = (strPad.offsetTop + strPadH + labelGap) + "px";


  // Hareket sınırları
  geom.thr = { cx: thrPad.offsetLeft + thrPadW/2,
               top: thrPad.offsetTop + padBorder + knobD/2,
               bottom: thrPad.offsetTop + thrPadH - padBorder - knobD/2 };
  geom.str = { cy: strPad.offsetTop + strPadH/2,
               left: strPad.offsetLeft + padBorder + knobD/2,
               right: strPad.offsetLeft + strPadW - padBorder - knobD/2 };

  setThrPct(0,false); setStrPct(0,false);
}
window.addEventListener('resize', layout);
new ResizeObserver(layout).observe(stage);

/* ==== Ortak durum ==== */
let lastThr = 0, lastStr = 0;

/* ==== %-konum ↔ piksel (GÜNCEL TEK SÜRÜM) ==== */
function setThrPct(p, sendNow=true){
  p = Math.max(-100, Math.min(100, p));
  const y = geom.thr.top + (100 - (p+100)/2) * (geom.thr.bottom - geom.thr.top)/100;
  thrKnob.style.left = (geom.thr.cx - thrKnob.clientWidth/2) + "px";
  thrKnob.style.top  = (y - thrKnob.clientHeight/2) + "px";
  lastThr = p;
  if(sendNow) send(lastThr, lastStr);
}
function setStrPct(p, sendNow=true){
  p = Math.max(-100, Math.min(100, p));
  const x = geom.str.left + (p+100)/200 * (geom.str.right - geom.str.left);
  strKnob.style.left = (x - strKnob.clientWidth/2) + "px";
  strKnob.style.top  = (geom.str.cy - strKnob.clientHeight/2) + "px";
  lastStr = p;
  if(sendNow) send(lastThr, lastStr);
}

/* ==== Sağlam pointer sürükleme (pad & knob) ==== */
/* Her kontrol kendi pointer'ını bağımsız takip eder. */
const thrState = { active: false, id: null };
const strState = { active: false, id: null };

function pctFromY(y){
  return Math.max(-100, Math.min(100, Math.round((geom.thr.bottom - y) * 200 / (geom.thr.bottom - geom.thr.top) - 100)));
}
function pctFromX(x){
  return Math.max(-100, Math.min(100, Math.round((x - geom.str.left) * 200 / (geom.str.right - geom.str.left) - 100)));
}

function beginControl(e, which){
  const id = e.pointerId ?? 1;
  if(which === "thr"){
    thrState.active = true; thrState.id = id;
    e.target.setPointerCapture?.(id);
    const y = Math.max(geom.thr.top, Math.min(geom.thr.bottom, e.clientY));
    setThrPct(pctFromY(y));
  }else{
    strState.active = true; strState.id = id;
    e.target.setPointerCapture?.(id);
    const x = Math.max(geom.str.left, Math.min(geom.str.right, e.clientX));
    setStrPct(pctFromX(x));
  }
  e.preventDefault();
}

function moveControl(e){
  const id = e.pointerId ?? 1;
  if(thrState.active && id === thrState.id){
    const y = Math.max(geom.thr.top, Math.min(geom.thr.bottom, e.clientY));
    setThrPct(pctFromY(y));
  }
  if(strState.active && id === strState.id){
    const x = Math.max(geom.str.left, Math.min(geom.str.right, e.clientX));
    setStrPct(pctFromX(x));
  }
}

function easeToCenter(currentVal, setter, onDone){
  const start = performance.now();
  const dur = 120;
  function step(t){
    const k = Math.min(1, (t - start) / dur);
    const v = Math.round((1 - k) * currentVal);
    setter(v, false);
    if(k < 1) requestAnimationFrame(step);
    else { setter(0, true); onDone?.(); }
  }
  requestAnimationFrame(step);
}

function endControl(e){
  const id = e.pointerId ?? 1;
  if(thrState.active && id === thrState.id){
    const current = lastThr;
    thrState.active = false; thrState.id = null;
    e.target.releasePointerCapture?.(id);
    easeToCenter(current, setThrPct);
  }
  if(strState.active && id === strState.id){
    const current = lastStr;
    strState.active = false; strState.id = null;
    e.target.releasePointerCapture?.(id);
    easeToCenter(current, setStrPct);
  }
}

[thrPad,thrKnob].forEach(el=>el.addEventListener('pointerdown', ev=>beginControl(ev,"thr")));
[strPad,strKnob].forEach(el=>el.addEventListener('pointerdown', ev=>beginControl(ev,"str")));
document.addEventListener('pointermove', moveControl, {passive:false});
document.addEventListener('pointerup', endControl);
document.addEventListener('pointercancel', endControl);
document.addEventListener('lostpointercapture', endControl);

/* ==== İletişim & Telemetri ==== */
function send(thr, steer){ fetch(`/set?thr=${thr}&steer=${steer}`).catch(()=>{}); }
let onlineTimer=null;
async function poll(){
  try{
    const r = await fetch('/stat',{cache:"no-store"});
    if(!r.ok) throw 0;
    const s = await r.json(); // {v,pct}
    applyBatt(s.v, s.pct);
    setOnline(true);
  }catch(e){ setOnline(false); }
}
function setOnline(ok){
  if(ok){ netStat.classList.add('online'); netStat.classList.remove('offline'); netTxt.textContent="Çevrimiçi";
          clearTimeout(onlineTimer); onlineTimer=setTimeout(()=>setOnline(false),2000); }
  else { netStat.classList.add('offline'); netStat.classList.remove('online'); netTxt.textContent="Çevrimdışı"; }
}
setInterval(poll,600); poll();

/* Kalp atışı: 80ms’de bir son bilinen durumu gönder */
setInterval(()=>{ send(lastThr, lastStr); }, 80);
document.addEventListener('visibilitychange', ()=>{ if(!document.hidden){ send(lastThr, lastStr); }});

/* ==== Batarya (şarj yok, %20 ve altı kırmızı + tek uyarı) ==== */

/* ==== Batarya (cooldown'lu uyarı) ==== */
let lowWarned = false;
let lowWarnCooldownUntil = 0;   // ms cinsinden; performance.now() ile karşılaştıracağız

function applyBatt(v, pct){
  const now = performance.now();

  bpct.textContent = `${pct}%`;
  bvolt.textContent = `${v.toFixed(2)} V`;
  bfill.style.width = Math.max(0, Math.min(100, pct)) + "%";
  bfill.classList.remove("green","red");

  if (pct <= 20) {
    bfill.classList.add("red");
    // cooldown: sadece belirli aralıklarla uyar
    if (!lowWarned && now > lowWarnCooldownUntil) {
      lowWarned = true;
      lowWarnCooldownUntil = now + 15000; // 15 sn cooldown (istersen 8000-30000 arası dene)
      alert("⚠️ Batarya düşük: %20 ve altı!");
    }
  } else {
    bfill.classList.add("green");
    // küçük histerezis: %22’nin üstünde tekrar uyarı verebilecek hale gelsin
    if (pct > 22) lowWarned = false;
  }
}


/* ==== Tam ekran ==== */
fsBtn.addEventListener('click', async ()=>{
  try{
    if(!document.fullscreenElement) await stage.requestFullscreen();
    else await document.exitFullscreen();
  }catch(e){}
  setTimeout(layout,120);
});

/* ==== Trim Kontrolleri ==== */
// Panel açma/kapama
trimBtn.addEventListener('click', ()=>{
  trimPanel.style.display = 'block';
  loadTrimSettings();
});

closeBtn.addEventListener('click', ()=>{
  trimPanel.style.display = 'none';
});

// Panel dışına tıklayınca kapat
trimPanel.addEventListener('click', (e)=>{
  if(e.target === trimPanel) trimPanel.style.display = 'none';
});

// Slider değer güncellemeleri ve anlık uygulama
throttleTrim.addEventListener('input', ()=>{
  const val = parseInt(throttleTrim.value);
  document.getElementById('throttleTrimValue').textContent = val;
  applyTrimLive('throttleTrim', val);
});

steeringAngle.addEventListener('input', ()=>{
  const val = parseInt(steeringAngle.value);
  document.getElementById('steeringAngleValue').textContent = val + '°';
  applyTrimLive('steeringAngle', val);
});

servoMin.addEventListener('input', ()=>{
  const val = parseInt(servoMin.value);
  document.getElementById('servoMinValue').textContent = val + '°';
  applyTrimLive('servoMin', val);
});

servoMax.addEventListener('input', ()=>{
  const val = parseInt(servoMax.value);
  document.getElementById('servoMaxValue').textContent = val + '°';
  applyTrimLive('servoMax', val);
});

// Anlık trim uygulama (geçici)
function applyTrimLive(param, value){
  const data = {};
  data[param] = value;
  fetch('/setTrimLive', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(data)
  }).catch(e => console.error('Anlık trim hatası:', e));
}

// Trim ayarlarını yükleme
async function loadTrimSettings(){
  try{
    const r = await fetch('/getTrim');
    if(r.ok){
      const data = await r.json();
      throttleTrim.value = data.throttleTrim;
      steeringAngle.value = data.steeringAngle;
      servoMin.value = data.servoMin;
      servoMax.value = data.servoMax;
      
      // Değerleri görsel olarak güncelle
      document.getElementById('throttleTrimValue').textContent = data.throttleTrim;
      document.getElementById('steeringAngleValue').textContent = data.steeringAngle + '°';
      document.getElementById('servoMinValue').textContent = data.servoMin + '°';
      document.getElementById('servoMaxValue').textContent = data.servoMax + '°';
    }
  }catch(e){console.error('Trim yükleme hatası:', e);}
}

// Trim ayarlarını kaydetme
saveTrim.addEventListener('click', async ()=>{
  try{
    const data = {
      throttleTrim: parseInt(throttleTrim.value),
      steeringAngle: parseInt(steeringAngle.value),
      servoMin: parseInt(servoMin.value),
      servoMax: parseInt(servoMax.value)
    };
    
    const r = await fetch('/setTrim', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(data)
    });
    
    if(r.ok){
      alert('✅ Trim ayarları EEPROM\'a kalıcı olarak kaydedildi!\n\nGaz trim: ' + data.throttleTrim + '\nYön servo açısı: ' + data.steeringAngle + '°\nMin açı: ' + data.servoMin + '°\nMax açı: ' + data.servoMax + '°');
      trimPanel.style.display = 'none';
    } else {
      alert('❌ Kaydetme hatası! Tekrar deneyin.');
    }
  }catch(e){
    console.error('Trim kaydetme hatası:', e);
    alert('❌ Bağlantı hatası! İnternet bağlantınızı kontrol edin.');
  }
});

/* İlk yerleşim */
layout();
</script>

</body></html>
)HTML";



/* ======================= Setup ======================= */
void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  analogReadResolution(12);

  analogSetPinAttenuation(ADC_PIN, ADC_11db);   // ~3.6 V giriş aralığı
  analogReadResolution(12);

  // EEPROM'u başlat ve ayarları yükle
  EEPROM.begin(EEPROM_SIZE);
  loadSettingsFromEEPROM();

  rudder.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  rudder.write(g_steeringAngle);  // Başlangıç servo açısı

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  /* Ana sayfa */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send_P(200, "text/html; charset=utf-8", index_html);
  });

  /* Kontrol: gaz / yön */
  server.on("/set", HTTP_GET, [](AsyncWebServerRequest* r) {
    if (r->hasParam("thr")) {
      int thrPct = constrain(r->getParam("thr")->value().toInt(), -100, 100);

      float vNow; int pctNow;
      sampleBatteryFiltered(vNow, pctNow);

      // Kritik pil koruması (%10 altı → gaz kes; %20 eşiği sadece UI'da uyarı)
      uint32_t now = millis();

// (c) Zamanlı histerezis
if (!g_critLock) {
  if (pctNow <= CRIT_BATT_PCT) {
    if (g_critEnterT0 == 0) g_critEnterT0 = now;
    if (now - g_critEnterT0 >= CRIT_ENTER_MS) g_critLock = true;
  } else {
    g_critEnterT0 = 0;
  }
}

if (g_critLock) {
  thrPct = 0;  // kilitliyken gaz kes
  if (pctNow >= CRIT_CLEAR_PCT) {
    if (g_critClearT0 == 0) g_critClearT0 = now;
    if (now - g_critClearT0 >= CRIT_CLEAR_MS) {
      g_critLock = false;
      g_critEnterT0 = 0;
      g_critClearT0 = 0;
    }
  } else {
    g_critClearT0 = 0;
  }
}


      g_thrPct = thrPct;
      setMotorWithTrim(g_thrPct);
    }
    if (r->hasParam("steer")) {
      setSteerPct(r->getParam("steer")->value().toInt());
    }
    if (g_thrPct != lastLoggedThr || g_strPct != lastLoggedStr) {
      Serial.printf("THR: %+d%%  STR: %+d%%\n", g_thrPct, g_strPct);
      lastLoggedThr = g_thrPct;
      lastLoggedStr = g_strPct;
    }
    r->send(200, "text/plain; charset=utf-8", "OK");
  });

  /* Telemetri: batarya voltaj/% (EMA uygulanmış) */
  server.on("/stat", HTTP_GET, [](AsyncWebServerRequest* r){
    float v; int pct; sampleBatteryFiltered(v, pct);
    char buf[80];
    snprintf(buf, sizeof(buf), "{\"v\":%.2f,\"pct\":%d}", v, pct);
    r->send(200, "application/json; charset=utf-8", buf);
  });

  /* Trim ayarlarını getirme */
  server.on("/getTrim", HTTP_GET, [](AsyncWebServerRequest* r){
    char buf[200];
    snprintf(buf, sizeof(buf), 
      "{\"throttleTrim\":%d,\"steeringAngle\":%d,\"servoMin\":%d,\"servoMax\":%d}",
      g_throttleTrim, g_steeringAngle, g_servoMinAngle, g_servoMaxAngle);
    r->send(200, "application/json; charset=utf-8", buf);
  });

  /* Anlık trim ayarlama (geçici, EEPROM'a yazmaz) */
  server.on("/setTrimLive", HTTP_POST, [](AsyncWebServerRequest* r){}, NULL, 
    [](AsyncWebServerRequest* r, uint8_t *data, size_t len, size_t index, size_t total) {
      String jsonStr = String((char*)data);
      
      if(jsonStr.indexOf("\"throttleTrim\":") > -1) {
        int idx = jsonStr.indexOf("\"throttleTrim\":") + 15;
        g_tempThrottleTrim = jsonStr.substring(idx, jsonStr.indexOf(',', idx) > -1 ? jsonStr.indexOf(',', idx) : jsonStr.indexOf('}', idx)).toInt();
        g_tempThrottleTrim = constrain(g_tempThrottleTrim, -100, 100);
      }
      
      if(jsonStr.indexOf("\"steeringAngle\":") > -1) {
        int idx = jsonStr.indexOf("\"steeringAngle\":") + 16;
        g_tempSteeringAngle = jsonStr.substring(idx, jsonStr.indexOf(',', idx) > -1 ? jsonStr.indexOf(',', idx) : jsonStr.indexOf('}', idx)).toInt();
        g_tempSteeringAngle = constrain(g_tempSteeringAngle, 0, 180);
      }
      
      if(jsonStr.indexOf("\"servoMin\":") > -1) {
        int idx = jsonStr.indexOf("\"servoMin\":") + 11;
        g_tempServoMinAngle = jsonStr.substring(idx, jsonStr.indexOf(',', idx) > -1 ? jsonStr.indexOf(',', idx) : jsonStr.indexOf('}', idx)).toInt();
        g_tempServoMinAngle = constrain(g_tempServoMinAngle, 0, 180);
      }
      
      if(jsonStr.indexOf("\"servoMax\":") > -1) {
        int idx = jsonStr.indexOf("\"servoMax\":") + 11;
        g_tempServoMaxAngle = jsonStr.substring(idx, jsonStr.indexOf(',', idx) > -1 ? jsonStr.indexOf(',', idx) : jsonStr.indexOf('}', idx)).toInt();
        g_tempServoMaxAngle = constrain(g_tempServoMaxAngle, 0, 180);
      }
      
      r->send(200, "application/json; charset=utf-8", "{\"success\":true}");
    });

  /* Trim ayarlarını kalıcı kaydetme (EEPROM'a yaz) */
  server.on("/setTrim", HTTP_POST, [](AsyncWebServerRequest* r){}, NULL, 
    [](AsyncWebServerRequest* r, uint8_t *data, size_t len, size_t index, size_t total) {
      // JSON parse (basit)
      String jsonStr = String((char*)data);
      
      // Basit JSON parsing
      int throttleTrimIdx = jsonStr.indexOf("\"throttleTrim\":") + 15;
      int steeringAngleIdx = jsonStr.indexOf("\"steeringAngle\":") + 16;
      int servoMinIdx = jsonStr.indexOf("\"servoMin\":") + 11;
      int servoMaxIdx = jsonStr.indexOf("\"servoMax\":") + 11;
      
      if(throttleTrimIdx > 15) {
        g_throttleTrim = jsonStr.substring(throttleTrimIdx, jsonStr.indexOf(',', throttleTrimIdx)).toInt();
        g_throttleTrim = constrain(g_throttleTrim, -100, 100);
        g_tempThrottleTrim = g_throttleTrim; // Geçici değeri de güncelle
      }
      
      if(steeringAngleIdx > 16) {
        g_steeringAngle = jsonStr.substring(steeringAngleIdx, jsonStr.indexOf(',', steeringAngleIdx)).toInt();
        g_steeringAngle = constrain(g_steeringAngle, 0, 180);
        g_tempSteeringAngle = g_steeringAngle; // Geçici değeri de güncelle
      }
      
      if(servoMinIdx > 11) {
        g_servoMinAngle = jsonStr.substring(servoMinIdx, jsonStr.indexOf(',', servoMinIdx)).toInt();
        g_servoMinAngle = constrain(g_servoMinAngle, 0, 180);
        g_tempServoMinAngle = g_servoMinAngle; // Geçici değeri de güncelle
      }
      
      if(servoMaxIdx > 11) {
        String maxStr = jsonStr.substring(servoMaxIdx, jsonStr.indexOf('}', servoMaxIdx));
        g_servoMaxAngle = maxStr.toInt();
        g_servoMaxAngle = constrain(g_servoMaxAngle, 0, 180);
        g_tempServoMaxAngle = g_servoMaxAngle; // Geçici değeri de güncelle
      }
      
      // EEPROM'a kalıcı kaydet
      saveSettingsToEEPROM();
      
      Serial.println("=== TRIM AYARLARI KALICI OLARAK KAYDEDİLDİ ===");
      Serial.printf("Gaz: %d, Yön Açısı: %d°, Min: %d°, Max: %d°\n",
                   g_throttleTrim, g_steeringAngle, g_servoMinAngle, g_servoMaxAngle);
      
      r->send(200, "application/json; charset=utf-8", "{\"success\":true}");
    });

  server.begin();
}

/* ======================= Loop ======================= */
void loop() {
  // Asenkron server kullanıldığı için loop boş.
}
