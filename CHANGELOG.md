# Değişiklik Günlüğü (Changelog)

Bu dosya ESP32 RC Boat WebController projesindeki tüm önemli değişiklikleri içerir.

## [v2.0] - 2025-09-01

### 🔄 Büyük Değişiklikler
- **Trim sistemi tamamen yeniden yapılandırıldı**
- Eski `g_steeringTrim` ve `g_servoCenterAngle` kaldırıldı
- Yeni `g_steeringAngle` ile doğrudan servo açı kontrolü (0-180 derece)

### ✨ Yeni Özellikler
- **4 Parametreli Trim Sistemi:**
  - Gaz trim (-100 ile +100)
  - Yön servo açısı (0-180 derece)
  - Servo minimum açı ayarı
  - Servo maksimum açı ayarı
- EEPROM adresleri optimize edildi
- Web arayüzünde doğrudan derece değeri gösterimi
- JSON API güncellemeleri

### 🛠️ Teknik İyileştirmeler
- EEPROM kullanımı 64 byte'a optimize edildi
- Servo açı kontrolü daha hassas
- Geçici trim değerleri için ayrı değişkenler
- Web arayüzü daha kullanıcı dostu

### 📝 Kod Değişiklikleri
- `setSteerPct()` fonksiyonu doğrudan açı haritalama için güncellendi
- EEPROM okuma/yazma fonksiyonları yeni parametreler için adapt edildi
- HTML/JavaScript arayüzü yenilendi

---

## [v1.5] - 2025-08-XX

### 🧪 Deneysel Özellikler
- **İlk trim sistemi denemesi**
- Basit EEPROM tabanlı ayar saklama
- Gelişmiş batarya yönetimi
- EMA filtreli voltaj ölçümü
- Dinamik yük kompanzasyonu
- Kritik batarya koruması

### 🔧 Teknik Detaylar
- ESPAsyncWebServer optimizasyonları
- Gelişmiş ADC ölçüm sistemi
- Histerezis tabanlı kritik batarya kontrolü

---

## [v1.0] - 2025-08-XX

### 🎯 İlk Basit Sürüm
- **Hiç trim sistemi YOK - sadece temel kontrol**
- WiFi Access Point (AP) modu
- MX1508 motor sürücü desteği
- SG90 servo kontrolü (sabit 45-135° arası)
- Basit batarya voltaj ölçümü
- Minimalist web tabanlı kontrol arayüzü

### 🔧 Özellikler
- ESPAsyncWebServer ile basit web arayüzü
- Real-time joystick kontrolü
- Temel batarya seviye göstergesi
- Kritik batarya koruması (basit)
- Sade, anlaşılır kod yapısı

---

## Versiyon Erişimi

- **v2.0 (Güncel)**: Ana branch (`main`) - Gelişmiş trim sistemi
- **v1.5 (Ara)**: Tag ile erişim (`git checkout v1.5`) - İlk trim denemesi  
- **v1.0 (Basit)**: Tag ile erişim (`git checkout v1.0`) - Hiç trim YOK

### GitHub Releases
Bu versiyonlara GitHub'dan da erişebilirsiniz:
- [v2.0 Release](https://github.com/OguzhanOZGEN/ESP32_RC_Boat_WebController/releases/tag/v2.0)
- [v1.5 Release](https://github.com/OguzhanOZGEN/ESP32_RC_Boat_WebController/releases/tag/v1.5)
- [v1.0 Release](https://github.com/OguzhanOZGEN/ESP32_RC_Boat_WebController/releases/tag/v1.0)

### Eski Versiyona Geçiş
```bash
git checkout v1.0   # En basit versiyona geçiş (trim YOK)
git checkout v1.5   # İlk trim denemesine geçiş  
git checkout v2.0   # Gelişmiş trim sistemine geçiş
git checkout main   # Son versiyona geri dönüş
```
