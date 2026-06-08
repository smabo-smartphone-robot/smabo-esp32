# ボード別 config.json テンプレート

DCモーター(TB6612FNG)の制御7本と I2C を、**ヘッダ上で隣り合うピン**に割り当てた
ボード別テンプレートです。剥く前の連結ジャンプワイヤを一直線に挿せます。

## 使い方
使うボードのファイルを、デバイスのルートに `config.json` という名前でコピー（Thonnyでアップロード）するだけ。
起動時に `config.py` の DEFAULTS へ deep-merge されます。WiFi欄は自分の環境に合わせて編集してください。

## TB6612FNG 制御ヘッダの並び（全ボード共通）
基板の印字（上→下）: `PWMA → AIN2 → AIN1 → STBY → BIN1 → BIN2 → PWMB → GND → VCC`
下表の GPIO をこの順でリボン接続。GND→GNDピン / VCC→3V3。

---

## 1. ESP32-S3-DevKitC-1 / DevKitM-1 / YD-ESP32-S3 (N16R8)
`config.esp32s3-devkitc1.json`
左側ヘッダの連続パッド `4,5,6,7,15,16,17` を使用。

| TB6612FNG | GPIO |
|---|---|
| PWMA | 4 |
| AIN2 | 5 |
| AIN1 | 6 |
| STBY | 7 |
| BIN1 | 15 |
| BIN2 | 16 |
| PWMB | 17 |

I2C(PCA9685): SCL=10, SDA=11（下段ブロックの隣接ペア）

## 2. Seeed XIAO ESP32-S3
`config.xiao-esp32s3.json`
片側の連続パッド `D0..D6 = GPIO 1,2,3,4,5,6,43` を使用（43=U0TXDだがネイティブUSB-CDC使用時は空き）。

| TB6612FNG | GPIO | XIAOラベル |
|---|---|---|
| PWMA | 1 | D0 |
| AIN2 | 2 | D1 |
| AIN1 | 3 | D2 |
| STBY | 4 | D3 |
| BIN1 | 5 | D4 |
| BIN2 | 6 | D5 |
| PWMB | 43 | D6 |

I2C(PCA9685): SCL=9(D10), SDA=8(D9)（反対側の隣接ペア）

## 3. ESP32(無印) DevKit 38pin
`config.esp32-classic.json`
左側ヘッダの出力可能な連続パッド `32,33,25,26,27,14,12` を使用
（34/35/36/39 は入力専用なのでモータ出力には使わない）。

| TB6612FNG | GPIO |
|---|---|
| PWMA | 32 |
| AIN2 | 33 |
| AIN1 | 25 |
| STBY | 26 |
| BIN1 | 27 |
| BIN2 | 14 |
| PWMB | 12 |

I2C(PCA9685): SCL=22, SDA=21（無印ESP32の定番）

---

## 注意
- ピン並びは各ボードの**公式レイアウト前提**です。互換クローンは印字が違う場合があるので、
  必ず基板のシルク印刷の並び順と照合してください。
- DCを使うには `"modes": {"dc_drive": true}` を追記して有効化。
- TB6612FNG は別途 VM(モータ電源)/GND(コモン)/AO1,AO2(左)/BO1,BO2(右) の結線が必要。
