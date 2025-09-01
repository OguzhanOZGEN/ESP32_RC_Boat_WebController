# Changelog

All notable changes to ESP32 RC Boat WebController project.

## [v2.0] - 2025-09-01
## Version Access

- **v2.0 (Current)**: Main branch (`main`) - Advanced trim system
- **v1.0 (Original)**: Tag access (`git checkout v1.0`) - Original code

### GitHub Releases
Access these versions from GitHub:
- [v2.0 Release](https://github.com/OguzhanOZGEN/ESP32_RC_Boat_WebController/releases/tag/v2.0)
- [v1.0 Release](https://github.com/OguzhanOZGEN/ESP32_RC_Boat_WebController/releases/tag/v1.0)

### Switch to Previous Version
```bash
git checkout v1.0   # Switch to original version
git checkout main   # Return to latest version
```

---

## Versiyon Erişimi

- **v2.0 (Güncel)**: Ana dal (`main`) - Gelişmiş trim sistemi
- **v1.0 (Orijinal)**: Tag erişimi (`git checkout v1.0`) - Orijinal kod

### GitHub Sürümleri
Bu versiyonlara GitHub'dan erişin:
- [v2.0 Sürümü](https://github.com/OguzhanOZGEN/ESP32_RC_Boat_WebController/releases/tag/v2.0)
- [v1.0 Sürümü](https://github.com/OguzhanOZGEN/ESP32_RC_Boat_WebController/releases/tag/v1.0)

### Önceki Versiyona Geçiş
```bash
git checkout v1.0   # Orijinal versiyona geçiş
git checkout main   # Son versiyona geri dönüş
```
- Complete trim system redesign
- Removed old `g_steeringTrim` and `g_servoCenterAngle` variables
- New `g_steeringAngle` with direct servo angle control (0-180 degrees)

### New Features
- 4-parameter trim system:
  - Throttle trim (-100 to +100)
  - Steering servo angle (0-180 degrees)
  - Servo minimum angle setting
  - Servo maximum angle setting
- EEPROM address optimization
- Direct degree value display in web interface
- JSON API updates

### Technical Improvements
- EEPROM usage optimized to 64 bytes
- More precise servo angle control
- Separate variables for temporary trim values
- More user-friendly web interface

### Code Changes
- `setSteerPct()` function updated for direct angle mapping
- EEPROM read/write functions adapted for new parameters
- HTML/JavaScript interface redesigned

---

## [v1.0] - 2025-08-XX

### Initial Release
- Basic RC boat control with MX1508 motor driver
- SG90 servo control
- WiFi Access Point mode
- Web-based control interface
- Battery voltage monitoring
- ESPAsyncWebServer implementation
- Real-time joystick control
- Battery level indicator
- Critical battery protection
- EMA filtered voltage measurement
- Dynamic load compensation

---

# Değişiklik Günlüğü

ESP32 RC Boat WebController projesindeki tüm önemli değişiklikler.

## [v2.0] - 2025-09-01

### Büyük Değişiklikler
- Trim sistemi tamamen yeniden tasarlandı
- Eski `g_steeringTrim` ve `g_servoCenterAngle` değişkenleri kaldırıldı
- Yeni `g_steeringAngle` ile doğrudan servo açı kontrolü (0-180 derece)

### Yeni Özellikler
- 4 parametreli trim sistemi:
  - Gaz trim (-100 ile +100)
  - Yön servo açısı (0-180 derece)
  - Servo minimum açı ayarı
  - Servo maksimum açı ayarı
- EEPROM adresleri optimize edildi
- Web arayüzünde doğrudan derece değeri gösterimi
- JSON API güncellemeleri

### Teknik İyileştirmeler
- EEPROM kullanımı 64 byte'a optimize edildi
- Servo açı kontrolü daha hassas
- Geçici trim değerleri için ayrı değişkenler
- Web arayüzü daha kullanıcı dostu

### Kod Değişiklikleri
- `setSteerPct()` fonksiyonu doğrudan açı haritalama için güncellendi
- EEPROM okuma/yazma fonksiyonları yeni parametreler için adapt edildi
- HTML/JavaScript arayüzü yenilendi

---

## [v1.0] - 2025-08-XX

### İlk Sürüm
- MX1508 motor sürücü ile temel RC tekne kontrolü
- SG90 servo kontrolü
- WiFi Access Point modu
- Web tabanlı kontrol arayüzü
- Batarya voltaj ölçümü
- ESPAsyncWebServer implementasyonu
- Gerçek zamanlı joystick kontrolü
- Batarya seviye göstergesi
- Kritik batarya koruması
- EMA filtreli voltaj ölçümü
- Dinamik yük kompanzasyonu

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
