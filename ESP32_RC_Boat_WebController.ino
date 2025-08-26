/* =========================================================
   ESP32 RC BOT – MX1508 + SG90 + 2S Batarya + Web Arayüz
   v2 (yön uzunluğu=Gaz, %100=8.25V, kalibrasyon + EMA, şarj bildirimi yok)
   ========================================================= */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <math.h>  // fabsf için

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
  int angle = map_i(pct, -100, 100, 73, 123);
  rudder.write(angle);
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
    <div class="fsBtn" id="fsBtn" title="Tam ekran"><div class="fsIcon"></div></div>
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


  rudder.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  rudder.write(SERVO_CENTER);

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
      setMotor(map_i(g_thrPct, -100, 100, -255, 255));
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

  server.begin();
}

/* ======================= Loop ======================= */
void loop() {
  // Asenkron server kullanıldığı için loop boş.
}
