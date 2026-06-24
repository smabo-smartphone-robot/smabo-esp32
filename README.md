# smabo-esp32

ESP32用ロボットファームウェア（MicroPython製）。

中継サーバ（smabo-brain）へWebSocketクライアントとして接続し、JSON命令を受け取って走行・サーボを制御します。エンコーダのホイール速度を送信し、オドメトリの積分は smabo-brain 側で行います。

設定（config / mode）は WebSocket ではなく、ESP32 が直接公開する **REST API** で smabo-web から読み書きします（`http_server.py`）。リアルタイム制御・テレメトリは従来どおり smabo-brain 経由の WebSocket です。



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
    },
    "brain": {
        "host": "192.168.1.100",
        "port": 9090
    }
}
```

`brain.host` には smabo-brain を動かすマシンの IP を指定します（必須）。他の設定は省略すると `config.py` の DEFAULTS が使われます。

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
# Ctrl+D でソフトリセット → シリアルログで WiFi 接続と smabo-brain への接続を確認
```


## モジュール構成

| ファイル | 役割 |
|---------|------|
| `main.py` | 起動エントリ・asyncio イベントループ |
| `config.py` | 永続設定（RAM + config.json、デバウンス保存） |
| `wifi_manager.py` | WiFi 接続・自動再接続 |
| `ws_client.py` | RFC 6455 WebSocket クライアント（smabo-brainへ接続・自動再接続、外部ライブラリ不要） |
| `http_server.py` | config/mode の REST API サーバ（smabo-web が直接アクセス、CORS 対応） |
| `robot.py` | オーケストレータ（rosbridgeプロトコル・モード管理） |
| `pca9685.py` | PCA9685 PWM ドライバ（サーボ用 I2C） |
| `servo_controller.py` | JointGroup（全サーボ共通） |
| `random_motion.py` | グループ単位のランダム動作 |
| `dc_motors.py` | TB6612 差動駆動（cmd_vel受信・デッドマン停止） |
| `encoder.py` | GPIO割り込みによるエンコーダカウント |
| `wheel_publisher.py` | エンコーダ → ホイール速度（/wheel_vel）送信 |
| `lidar_ld06.py` | LD06 ライダ（UART直結）→ `/scan`（sensor_msgs/LaserScan）送信。Nav2 のセンサ入力 |


## 設定テンプレート

`configs/` に各ボード向けのサンプルがあります。

| ファイル | 対象ボード |
|---------|-----------|
| `config.esp32-classic.json` | ESP32 DevKit（38ピン標準） |
| `config.esp32s3-devkitc1.json` | ESP32-S3 DevKitC-1 |
| `config.xiao-esp32s3.json` | Seeed XIAO ESP32S3 |


## 通信プロトコル

起動時に smabo-brain（`ws://<brain-host>:<port>/esp32`、接続先は `config.json` の `brain` で指定）へクライアントとして接続し、JSON送受信します。フォーマットはrosbridgeの v2.0 互換。認証なし（信頼済みLAN内利用前提）。

主なトピック:

| 方向 | トピック | 内容 |
|------|---------|------|
| 受信 | `/cmd_vel` | 走行速度指令 |
| 受信 | `/servo/command` | サーボ軌道指令 |
| 送信 | `/wheel_vel` | ホイール速度（left/right m/s, dt）。オドメトリ積分は smabo-brain 側 |
| 送信 | `/joint_states` | 関節角度 |
| 送信 | `/scan` | LD06 ライダのスキャン（`modes.lidar` 有効時）。Nav2 のセンサ入力 |
| 送信 | `set_config` | 全 config スナップショット。smabo-brain のオドメトリ同期用（下記参照。rosbridge モードでは送信しない） |

### 設定 REST API（smabo-web ↔ ESP32 直通）

config / mode は smabo-brain を介さず、ESP32 の HTTP サーバ（既定 `:80`、CORS 全許可）に
smabo-web から直接アクセスします。

| メソッド | パス | 内容 |
|----------|------|------|
| `GET`  | `/config` | 現在の全 config を JSON で返す |
| `POST` | `/config` | config パッチを deep-merge（ピン/バス/WiFi 変更時は再起動） |
| `POST` | `/mode`   | サブシステムの有効/無効（servos / dc_drive / encoder_drive / lidar） |

smabo-brain のオドメトリ積分は車輪ジオメトリ・共分散・frame 名を必要とするため、
ESP32 は brain への接続時および config/mode 変更時に、全 config を WebSocket で
`{"op":"set_config","config":{…}}` として brain に push します（brain 内部の odom 同期専用）。


## smabo-brain-ros（rosbridge）連携

ROS 2 スタック（[smabo-brain-ros](https://github.com/smabo-smartphone-robot/smabo-brain-ros)：
rosbridge_suite + Nav2 + MoveIt2）に接続する場合は `config.json` で次を設定します。

```json
{ "brain": { "host": "<ros-host>", "port": 9090, "rosbridge": true } }
```

- 接続先パスは rosbridge の `"/"`（レガシー smabo-brain は `"/esp32"`）。`brain.path` で上書き可。
- rosbridge は publish 前の **advertise** と受信のための **subscribe** を要求するため、
  ESP32 は接続時に送信トピック（`/esp32/wheel_vel`・`/esp32/joint_states`・`/esp32/scan`）を
  advertise し、受信トピック（`/cmd_vel`・`/servo/command`）を subscribe します。
- rosbridge モードでは `set_config` を送りません（オドメトリのパラメータは ROS 側の launch で
  与えるため）。送信元 prefix `/esp32` は ROS 側の relay が剥がします。

`rosbridge:false`（既定）ならレガシー smabo-brain への従来動作のままです。

### LD06 ライダ（`modes.lidar`）

LD06 ライダをマイコンの UART に直結（ライダ TX → MCU RX）し、`/scan` を配信します。
`modes.lidar` を有効にすると `lidar_ld06.py` が UART を読み取り、1回転ごとに
`sensor_msgs/LaserScan` を publish します（Nav2 のコストマップ／SLAM の入力）。

| `lidar` 設定 | 既定 | 説明 |
|---|---|---|
| `uart` / `rx` / `tx` | `1` / `20` / `-1` | UART 番号・RX ピン・TX（未使用） |
| `baud` | `230400` | LD06 のボーレート |
| `frame_id` | `laser` | `/scan` の frame（smabo_description のライダ frame と一致） |
| `bins` | `360` | `ranges` 配列長（1 bin ≈ 1°） |
| `range_min` / `range_max` | `0.05` / `12.0` | 有効距離 (m) |

有効化は REST（`POST /mode {"lidar": true}`）か `config.json` の `modes.lidar`。
UART ピン変更は再起動せずホットリロードします。



