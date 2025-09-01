# ESP32 RC Boat Web Controller

ESP32-powered remote control boat with web-based control interface. Features motor control, servo steering, and real-time 2S Li-ion battery monitoring.

## Features

**Control System**
- ESP32 Access Point mode (no internet required)
- Web-based joystick control interface
- Real-time throttle and steering control
- Auto-center when released

**Hardware Support**
- MX1508 motor driver with PWM control
- SG90 servo for rudder control
- 2S Li-ion battery monitoring with voltage divider
- Critical battery protection (10% cutoff)

**Advanced Features (v2.0)**
- 4-parameter trim system
- Direct servo angle control (0-180°)
- EEPROM settings storage
- Customizable servo min/max angles
- Throttle trim adjustment (-100 to +100%)

**Monitoring**
- Real-time voltage and percentage display
- Charging detection (voltage > 9V)
- EMA filtered measurements
- Dynamic load compensation

## Hardware Requirements

- ESP32 Development Board
- MX1508 Motor Driver
- SG90 Servo Motor  
- 2x 18650 Li-ion Cells (2S configuration)
- BMS Protection Board
- Voltage divider circuit (220kΩ + 100kΩ)

## Usage

1. Upload code to ESP32
2. Connect to WiFi: `TEKNE_AP` (password: `12345678`)  
3. Open browser to `192.168.4.1`
4. Control boat with web interface
5. Monitor battery status in real-time

## Versions

- **v1.0**: Original basic control system
- **v2.0**: Advanced trim system with EEPROM storage

## License

This project is open-source under the MIT License.

---

# ESP32 RC Tekne Web Kontrolcüsü

ESP32 tabanlı web arayüzlü uzaktan kumandalı tekne. Motor kontrolü, servo yönlendirme ve gerçek zamanlı 2S Li-ion batarya izleme özellikleri.

## Özellikler

**Kontrol Sistemi**
- ESP32 Access Point modu (internet gerektirmez)
- Web tabanlı joystick kontrol arayüzü
- Gerçek zamanlı gaz ve yön kontrolü
- Bırakıldığında otomatik merkeze dönüş

**Donanım Desteği**
- PWM kontrollü MX1508 motor sürücü
- Dümen kontrolü için SG90 servo
- Voltaj bölücü ile 2S Li-ion batarya izleme
- Kritik batarya koruması (%10 kesim)

**Gelişmiş Özellikler (v2.0)**
- 4 parametreli trim sistemi
- Doğrudan servo açı kontrolü (0-180°)
- EEPROM ayar saklama
- Özelleştirilebilir servo min/max açıları
- Gaz trim ayarı (-100 ile +100%)

**İzleme**
- Gerçek zamanlı voltaj ve yüzde gösterimi
- Şarj algılama (voltaj > 9V)
- EMA filtreli ölçümler
- Dinamik yük kompanzasyonu

## Donanım Gereksinimleri

- ESP32 Geliştirme Kartı
- MX1508 Motor Sürücü
- SG90 Servo Motor
- 2x 18650 Li-ion Pil (2S konfigürasyon)
- BMS Koruma Kartı
- Voltaj bölücü devresi (220kΩ + 100kΩ)

## Kullanım

1. Kodu ESP32'ye yükleyin
2. WiFi'ye bağlanın: `TEKNE_AP` (şifre: `12345678`)
3. Tarayıcıda `192.168.4.1` adresini açın
4. Web arayüzü ile tekneyi kontrol edin
5. Batarya durumunu gerçek zamanlı izleyin

## Versiyonlar

- **v1.0**: Orijinal temel kontrol sistemi  
- **v2.0**: EEPROM depolama ile gelişmiş trim sistemi

## Lisans

Bu proje MIT Lisansı altında açık kaynaklıdır.
