# cugo_rs485_motor_control

CuGoV4 + Raspberry Pi 4 で、BLVD10KM (BLVM510K-G45) を RS485(Modbus RTU) 制御するための最小実装です。

## ■ 対応範囲

- Modbus RTU Function Code: `0x03`, `0x06`, `0x10`
- BLV操作に必要な最低限コマンド
  - 回転速度設定（Operation Data No.0..7）
  - 正転 / 逆転 / 停止

## ■ 想定環境

- Raspberry Pi 4
- Python 3.9+
- USB-RS485変換器 ([Amazon](https://amzn.asia/d/04F2nQpA))
  - RJ45 ([Amazon](https://amzn.asia/d/0dXFUSVF)) も一緒に使用する
- モータドライバ: BLVD10KM
  - [データシート（通信編）](https://www.orientalmotor.co.jp/ja/system/files?file=product_detail/manual/HM-5101J.pdf)  
  - [データシート（仕様）](https://www.orientalmotor.co.jp/system/files/product_detail/manual/HM-5100J.pdf?serviceTypeNo=18&senimotoId=1&brandCode=BL&seriesCode=FJ00)  

## ■ ドライバ事前設定（BLVD10KM）

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

## ■ 配線

<img src=docs/rs485_connector.png width=50%><img src=docs/blvd10km_rs485.png width=50%>

- BLVD CN5/CN6 `TR+` -> USB-RS485 `A(+)`
- BLVD CN5/CN6 `TR-` -> USB-RS485 `B(-)`  
- BLVD CN5/CN6 `GND`  -> USB-RS485 `GND`


## ■ セットアップ

```bash
cd ~/src/cugo_rs485_motor_control
python3 -m pip install pyserial
```

## ■ 実行

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

### 6) 速度情報をリアルタイム表示

```bash
python3 scripts/main.py --port /dev/ttyUSB0 --left-slave 2 --right-slave 1 --debug --debug-interval 0.5 read-speed
```

### `scripts/propo_control.py`（RCプロポ入力で左右2台制御）

`ch-C`/`ch-D`/`ch-H` のPWM入力を使って左右モータを直接制御します。  
`ch-H` は出力スケール（段階）として扱います。

- デフォルト最大並進速度: `1.89 km/h`（`0.525 m/s`）
- デフォルト最大角速度: `3.14 rad/s`
- デフォルトPWM校正値:
  - `CH-C`: min=`945`, center=`1484`, max=`2132`
  - `CH-D`: min=`1114`, center=`1501`, max=`1870`
  - `CH-H`: min=`997`, center=`1484`, max=`1994`
- デフォルトGPIO(BCM): `CH-C=24`, `CH-D=4`, `CH-H=14`

起動例:

```bash
sudo python3 scripts/propo_control.py --port /dev/ttyUSB0 --left-slave 2 --right-slave 1
```

主なオプション:

- `--max-v-kmh`: 最大並進速度 [km/h]（デフォルト `1.89`）
- `--max-w-radps`: 最大角速度 [rad/s]
- `--h-reduction`: `CH-H`最小時の出力倍率（`0.0 < value <= 0.5`）
- `--neutral-deadband-us`: 中立デッドバンド[us]（デフォルト `60`）
- `--axis-zero-eps`: 正規化後の軸値がしきい値以下なら強制0（デフォルト `0.05`）
- `--v-axis` / `--w-axis` / `--swap-axes`: 軸割り当て
- `--invert-v` / `--no-invert-w`: 符号調整
- 後進時は操舵方向を反転し、スティックの左右と車体の旋回感を合わせる
- `--verbose`: 毎周期ログ表示
- `--dry-run`: RS485送信なしで入力と演算のみ確認

注意:

- GPIO入力を使うため `RPi.GPIO` が必要です。
- GPIOアクセスのため `sudo` 実行を推奨します。

## ■ モジュール構成

- `scripts/main.py`
  - 左右2台制御用CLI
- `scripts/propo_control.py`
  - RCプロポ入力（PWM）での左右2台制御CLI
- `scripts/cugo_rs485_motor_control/modbus_rtu.py`
  - Modbus RTUフレーム生成/CRC/応答検証
- `scripts/cugo_rs485_motor_control/blv_motor.py`
  - BLV向け高レベル制御（速度・正逆転・停止）
