# cugo_rs485_motor_control

CuGoV4 + Raspberry Pi 4 で、BLVD10KM (BLVM510K-G45) を RS485(Modbus RTU) 制御するための最小実装です。

## 対応範囲

- Modbus RTU Function Code: `0x03`, `0x06`, `0x10`
- BLV操作に必要な最低限コマンド
  - 回転速度設定（Operation Data No.0..7）
  - 正転 / 逆転 / 停止

## 想定環境

- Raspberry Pi 4
- Python 3.9+
- USB-RS485変換器
- モータドライバ: BLVD10KM

## ドライバ事前設定（BLVD10KM）

- `SW2-No.5 = ON`（Modbus）
- `SW2-No.1..3 = OFF/OFF/OFF`（9600bps）
- `SW2-No.4, No.6 = OFF`（ON禁止）
- `SW2-No.7 = ON`（終端。1台試験時はON推奨）
- `SW3 = 1`, `SW2-No.8 = OFF`（Slave ID=1）

2台チェーン接続で左右制御する場合は、左右で異なる Slave ID を設定します。

- 左: `SW3 = 2`, `SW2-No.8 = OFF`（Slave ID=2）
- 右: `SW3 = 1`, `SW2-No.8 = OFF`（Slave ID=1）
- 終端抵抗 `SW2-No.7` はバスの末端ドライバだけ ON

通信パラメータ初期値は `8E1`（Even parity, 1 stop bit）です。

## 配線

- BLVD CN5/CN6 `TR+` -> USB-RS485 `A(+)`
- BLVD CN5/CN6 `TR-` -> USB-RS485 `B(-)`
- `GND` 共通

通信できない場合は `A/B` を入れ替えて確認してください。

## セットアップ

```bash
cd ~/src/cugo_rs485_motor_control
python3 -m pip install pyserial
```

## 実行

## コマンドライン引数一覧

### `scripts/main.py`（左右2台制御）

| 引数 | デフォルト | 概要 |
| --- | --- | --- |
| `--port` | `/dev/ttyUSB0` | シリアルポート |
| `--baudrate` | `9600` | 通信ボーレート |
| `--left-slave` | `2` | 左モータのModbusスレーブID |
| `--right-slave` | `1` | 右モータのModbusスレーブID |
| `--left-dir-sign` | `-1` | `run` の左方向符号（`-1`で反転） |
| `--right-dir-sign` | `1` | `run` の右方向符号（`-1`で反転） |
| `--debug` | `OFF` | 速度情報のリアルタイム表示を有効化 |
| `--debug-interval` | `0.5` | リアルタイム表示周期 [s]（`--debug`時のみ） |

| コマンド | 引数 | 概要 |
| --- | --- | --- |
| `set-speed` | `--left RPM --right RPM` | 左右の速度を同時設定 |
| `run` | `--left-dir fwd/rev/stop --right-dir fwd/rev/stop [--left-rpm RPM --right-rpm RPM]` | 左右の方向を同時指令（任意で同時に速度設定） |
| `stop` | `--instant`（任意） | 左右を同時停止 |
| `read-speed` | なし | 左右の速度情報（設定/指令/フィードバック）を読み取り |
| `read-speed-setting` | なし | 左右の設定速度（Speed No.2）を読み取り |

固定値: parity=`E`、stopbits=`1`、timeout=`0.3` 秒
サブコマンド未指定時は `read-speed-setting` が実行されます。

### 1) 左右の速度設定

```bash
python3 scripts/main.py --port /dev/ttyUSB0 --left-slave 2 --right-slave 1 set-speed --left 500 --right 500
```

### 2) 左右とも正転

```bash
python3 scripts/main.py --port /dev/ttyUSB0 --left-slave 2 --right-slave 1 run --left-dir fwd --right-dir fwd
```

### 2-1) 速度を同時に指令して正転

```bash
python3 scripts/main.py --port /dev/ttyUSB0 --left-slave 2 --right-slave 1 run --left-dir fwd --right-dir fwd --left-rpm 500 --right-rpm 500
```

`run` コマンドは、デフォルトで CuGo向けに左モータの向きを反転して扱います（`--left-dir-sign -1`）。  
モータ単体の向きをそのまま使いたい場合は `--left-dir-sign 1 --right-dir-sign 1` を指定してください。

### 3) 旋回（左正転・右逆転）

```bash
python3 scripts/main.py --port /dev/ttyUSB0 --left-slave 2 --right-slave 1 run --left-dir fwd --right-dir rev
```

### 4) 左右停止

```bash
python3 scripts/main.py --port /dev/ttyUSB0 --left-slave 2 --right-slave 1 stop
```

### 5) 左右の速度情報を読み取り

```bash
python3 scripts/main.py --port /dev/ttyUSB0 --left-slave 2 --right-slave 1 read-speed
```

### 6) 速度情報をリアルタイム表示（Ctrl+Cで終了）

```bash
python3 scripts/main.py --port /dev/ttyUSB0 --left-slave 2 --right-slave 1 --debug --debug-interval 0.5 read-speed
```

## モジュール構成

- `scripts/main.py`
  - 左右2台制御用CLI
- `scripts/cugo_rs485_motor_control/modbus_rtu.py`
  - Modbus RTUフレーム生成/CRC/応答検証
- `scripts/cugo_rs485_motor_control/blv_motor.py`
  - BLV向け高レベル制御（速度・正逆転・停止）
