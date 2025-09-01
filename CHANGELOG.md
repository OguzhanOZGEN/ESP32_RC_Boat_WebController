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

## [v1.0] - 2025-08-XX

### 🎯 İlk Sürüm
- **Temel RC Bot kontrolü**
- WiFi Access Point (AP) modu
- MX1508 motor sürücü desteği
- SG90 servo kontrolü
- Batarya voltaj ölçümü
- Web tabanlı kontrol arayüzü
- Basit trim sistemi
- EEPROM ile ayar saklama

### 🔧 Özellikler
- ESPAsyncWebServer ile web arayüzü
- Real-time joystick kontrolü
- Batarya seviye göstergesi
- Kritik batarya koruması
- EMA filtreli voltaj ölçümü
- Dinamik yük kompanzasyonu

---

## Versiyon Erişimi

- **v2.0 (Güncel)**: Ana branch (`main`)
- **v1.0 (Eski)**: Tag ile erişim (`git checkout v1.0`)

### GitHub Releases
Bu versiyonlara GitHub'dan da erişebilirsiniz:
- [v2.0 Release](https://github.com/OguzhanOZGEN/ESP32_RC_Boat_WebController/releases/tag/v2.0)
- [v1.0 Release](https://github.com/OguzhanOZGEN/ESP32_RC_Boat_WebController/releases/tag/v1.0)

### Eski Versiyona Geçiş
```bash
git checkout v1.0  # Eski versiyona geçiş
git checkout main   # Son versiyona geri dönüş
```
