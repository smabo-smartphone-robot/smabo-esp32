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


### Wi-Fi を Bluetooth で設定する（Improv）

起動時に Wi-Fi へつながらないと、ESP32 は Bluetooth で `smabo-<hostname>` という名前を出して待ちます。smabo-web の設定画面（または smabo-esp32 のテンプレート）にある「ESP32 Wi-Fi setup over Bluetooth」で **Find ESP32** を押し、SSID とパスワードを送ると、ESP32 がつないでみて、成功したら `config.json` に保存し、設定 API の URL を返します。smabo-web はその URL を ESP32 のホストに設定します。

- [Improv Wi-Fi](https://www.improv-wifi.com/ble/) の BLE 版なので、他の Improv 対応ツールからも設定できます。
- ブラウザは Chrome か Edge で、ページを `https://` か `http://localhost` で開いたときだけ使えます（Web Bluetooth の制限）。
- Bluetooth を出すのは Wi-Fi につながらないときだけです。設定が済むと 10 秒後に止めます。つながっている ESP32 の Wi-Fi は Bluetooth からは変えられません（設定 API か `config.json` で変えます）。
- Wi-Fi につながらない間は、近くの誰でも Wi-Fi 設定を送れます。気になる場合は `"improv": {"enabled": false}` で止めます。

## モジュール構成

| ファイル | 役割 |
|---------|------|
| `main.py` | 起動エントリ・asyncio イベントループ |
| `config.py` | 永続設定（RAM + config.json、デバウンス保存） |
| `wifi_manager.py` | WiFi 接続・自動再接続 |
| `improv.py` | Wi-Fi につながらないときの Bluetooth 設定（Improv Wi-Fi） |
| `ws_client.py` | RFC 6455 WebSocket クライアント（smabo-brainへ接続・自動再接続、外部ライブラリ不要） |
| `http_server.py` | config/mode の REST API サーバ（ペアリング後は brain の中継用の鍵がある要求だけ） |
| `pairing.py` | brain とのペアリング（token・招待コード・中継用の鍵の確認） |
| `robot.py` | オーケストレータ（サブシステムの起動停止・モード管理・受信指令の処理） |
| `brain_link.py` | brain との通信（advertise / subscribe・publish・受信フレームの解釈） |
| `protocol_gen.py` | トピック表とプロトコル番号（smabo-protocol の `gen.py` が生成。手で直さない） |
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

起動時に brain（`ws://<brain-host>:<port>/`、接続先は `config.json` の `brain` で指定）へクライアントとして接続し、JSON送受信します。フォーマットはrosbridgeの v2.0 互換。

### brain とのペアリング

smabo-brain はペアリングした機器だけを受け付けます（`pairing.py`）。

1. smabo-web の「Devices」→「Add ESP32」で招待コードを出す
2. ESP32 に届ける。Bluetooth の Wi-Fi 設定（Improv）を smabo-web から行うと、brain の場所と招待コードも一緒に送られます。Wi-Fi につながっていてペアリング前なら、招待ボックスから ESP32 のアドレスに送るか、`POST /config {"brain": {"invite": "ABCD-EFGH"}}` を送ります
3. ESP32 は次の接続でコードを見せ、brain から token と中継用の鍵を受け取ります。token は `config.json` の `brain.token` に、鍵は SHA-256 だけを `http.relay_key_sha256` に保存します

接続のたびに、最初のフレームで token を見せます（`{"op": "auth", "token": …}`）。ペアリング後の REST API は、brain が付ける中継用の鍵（`X-Smabo-Relay-Key`）のない要求を 401 で断り、CORS も返しません。smabo-web の設定は brain の `/devices/esp32/…` を通ります。token・招待コード・鍵のハッシュは、`GET /config` にも `/config` トピックにも出しません（代わりに `brain.paired` を返します）。

smabo-web で外す（または brain を `--reset-pairing` で初期化する）と、brain に断られた時点で ESP32 はペアリングを忘れ、また招待コードを受け付けます。`--no-auth` の brain と smabo-brain-ros は token に答えないので、2 秒待ってから従来どおりつなぎます。

brain は smabo-brain でも smabo-brain-ros でも同じ手順なので、どちらを使うかの設定はありません。接続のたびに、送信トピックを **advertise**、受信トピックを **subscribe** してから config スナップショットを送ります。送信トピックには送信元 prefix `/esp32` が付き、brain が剥がします。実装は `brain_link.py` です。

主なトピック:

| 方向 | トピック | 内容 |
|------|---------|------|
| 受信 | `/cmd_vel` | 走行速度指令 |
| 受信 | `/servo/command` | サーボ軌道指令 |
| 受信 | `/ping` | 疎通確認（受信した `data` をそのまま `/pong` で返す） |
| 送信 | `/pong` | `/ping` への応答（smabo-web の WebSocket 疎通確認用） |
| 送信 | `/wheel_vel` | ホイール速度（left/right m/s, dt）。オドメトリ積分は smabo-brain 側 |
| 送信 | `/joint_states` | 関節角度 |
| 送信 | `/scan` | LD06 ライダのスキャン（`modes.lidar` 有効時）。Nav2 のセンサ入力 |
| 送信 | `/config` | config スナップショット（std_msgs/String の data に JSON。Wi-Fi 設定は除く）。smabo-brain のオドメトリ同期用（下記参照） |

### 設定 REST API

config / mode は ESP32 の HTTP サーバ（既定 `:80`）にあります。smabo-web は brain の
`/devices/esp32/…` を通して読み書きします（brain が中継用の鍵を付けて転送）。
ペアリング前と `--no-auth` の brain では、smabo-web が直接アクセスします（CORS 全許可）。

| メソッド | パス | 内容 |
|----------|------|------|
| `GET`  | `/config` | 現在の全 config を JSON で返す |
| `POST` | `/config` | config パッチを deep-merge（ピン/バス/WiFi 変更時は再起動） |
| `POST` | `/mode`   | サブシステムの有効/無効（servos / dc_drive / encoder_drive / lidar） |

smabo-brain のオドメトリ積分は車輪ジオメトリ・共分散・frame 名を必要とするため、
ESP32 は brain への接続時および config/mode 変更時に、config を `/esp32/config` で
brain に送ります（brain 内部の odom 同期専用。web へは流れません）。


## smabo-brain-ros（ROS 2）連携

ROS 2 スタック（[smabo-brain-ros](https://github.com/smabo-smartphone-robot/smabo-brain-ros)：
rosbridge_suite + Nav2 + MoveIt2）につなぐ場合も、`brain.host` をそのマシンにするだけで、
ほかの設定は要りません。送信元 prefix `/esp32` は ROS 側の relay が剥がします。
古い `config.json` に残っている `brain.rosbridge` は無視されます。

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

## テスト

ESP32 がなくても、PC の Python でモーターの計算・設定の読み書き・停止タイマーを確かめられます。`machine` モジュールは `tests/conftest.py` の偽物に置き換わります。

```bash
pip install pytest
python -m pytest tests
```

## バージョンと互換性

ほかの smabo リポジトリとは、同じ minor 番号（`0.2.x` など）どうしで組み合わせます。対応表は
[smabo-brain の README](https://github.com/smabo-smartphone-robot/smabo-brain#バージョンと互換性) にあります。
