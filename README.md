# smabo-esp32

ESP32用ロボットファームウェア（MicroPython製）。

スマホアプリ（smabo-app）からのWebSocket JSON命令を受け取り、走行・サーボ・オドメトリを制御します。



## 動作環境

- ESP32（DevKit / S3 / XIAO ESP32S3）
- MicroPython v1.22 以降
- 接続機器: PCA9685（サーボ）、TB6612FNG（DCモータ）


## セットアップ

### 1. MicroPython を書き込む

[micropython.org](https://micropython.org/download/ESP32_GENERIC/) から対象ボードの `.bin` を取得し、`esptool` でフラッシュします。

```bash
esptool.py --port /dev/ttyUSB0 erase_flash
esptool.py --port /dev/ttyUSB0 write_flash -z 0x1000 ESP32_GENERIC-*.bin
```

### 2. WiFi設定ファイルを作成する


```bash
# ボードに合わせてテンプレートを選択
cp configs/config.esp32-classic.json config.json
# SSID と password を自分の環境に合わせて書き換える
```

`config.json` の最小構成:

```json
{
    "wifi": {
        "ssid": "YOUR-SSID",
        "password": "YOUR-PASSWORD"
    }
}
```

他の設定は省略すると `config.py` の DEFAULTS が使われます。

### 3. ファイルを転送する

`mpremote` または `rshell` / `ampy` でファイルを書き込みます。

```bash
# mpremote の例（全ファイルを一括転送）
mpremote connect /dev/ttyUSB0 cp config.json :
mpremote connect /dev/ttyUSB0 cp *.py :
```

### 4. 起動確認

```bash
mpremote connect /dev/ttyUSB0 repl
# Ctrl+D でソフトリセット → シリアルログで WiFi 接続と WebSocket 待受を確認
```


## モジュール構成

| ファイル | 役割 |
|---------|------|
| `main.py` | 起動エントリ・asyncio イベントループ |
| `config.py` | 永続設定（RAM + config.json、デバウンス保存） |
| `wifi_manager.py` | WiFi 接続・自動再接続 |
| `ws_server.py` | RFC 6455 WebSocket サーバ（外部ライブラリ不要） |
| `robot.py` | オーケストレータ（rosbridgeプロトコル・モード管理） |
| `pca9685.py` | PCA9685 PWM ドライバ（サーボ用 I2C） |
| `servo_controller.py` | JointGroup（全サーボ共通） |
| `random_motion.py` | グループ単位のランダム動作 |
| `dc_motors.py` | TB6612 差動駆動（cmd_vel受信・デッドマン停止） |
| `encoder.py` | GPIO割り込みによるエンコーダカウント |
| `odometry.py` | 差動駆動オドメトリ → nav_msgs/Odometry |


## 設定テンプレート

`configs/` に各ボード向けのサンプルがあります。

| ファイル | 対象ボード |
|---------|-----------|
| `config.esp32-classic.json` | ESP32 DevKit（38ピン標準） |
| `config.esp32s3-devkitc1.json` | ESP32-S3 DevKitC-1 |
| `config.xiao-esp32s3.json` | Seeed XIAO ESP32S3 |


## 通信プロトコル

WebSocket `ws://<ESP32-IP>:9090` でJSON送受信。フォーマットはrosbridgeの v2.0 互換。認証なし（信頼済みLAN内利用前提）。

主なトピック:

| 方向 | トピック | 内容 |
|------|---------|------|
| 受信 | `/cmd_vel` | 走行速度指令 |
| 受信 | `/servo/command` | サーボ軌道指令 |
| 送信 | `/odom` | オドメトリ |
| 送信 | `/joint_states` | 関節角度 |



