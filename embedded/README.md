# 電源・表示コントローラ

`/home/tako/src/pico_esp32_bridge` から、RP2040電源制御とESP32表示パネルに必要な部分を取り込みました。元プロジェクトには変更を加えません。取り込み時の出典とSHA-256は `import-manifest.json` です。一般UARTブリッジ、書き込み専用ブリッジ、旧診断ファームウェアは含みません。

- `pico_power_master/`: INA226測定、SSR、物理ボタン、WS2812、パネル通信、Pythonホスト通信。
- `esp32_panel_slave/`: 既存の表示・キー処理。通信形式は元の電源コントローラと同じで、今回のホスト統合によるESP32の再書き込みは不要です。
- `tests/`: SDKを模擬して実際のCメインループを実行する試験と、測定・パネル・電源制御の試験。

## 接続

| Pico端子 | 接続・動作 |
| --- | --- |
| GP2 | SSR出力、High=ON、起動時OFF |
| GP5 | 電源ボタン、押下=Low、内部プルアップ。起動時に押されていれば一度離すまで無効 |
| GP26 / GP27 | INA226 SDA / SCL、I2C1、アドレス0x44、R100=0.1Ω |
| GP8 / GP9 | ESP32へTX / RX、UART1、115200 baud |
| GP6 / GP7 | ESP32 BOOT / RESET |
| GP28 | WS2812、30灯、PIO+DMA |
| USB | PythonへCDC通信、USB固有番号で識別 |

物理ボタンがSSRをON/OFFします。PythonからONするコマンドはありません。既存の電圧・電流・電力表示切替と、ブザー無効設定を引き継いでいます。

## Pythonでの利用

トップレベルJSONに次の設定を追加します。固有番号は `uv run hg-cart devices` で確認できます。

```json
"power": {
  "enabled": true,
  "serial": "DE610441238C2E35",
  "port": "",
  "telemetry_timeout_s": 0.5,
  "usb_min_v": 3.5,
  "usb_max_v": 6.5
}
```

`serial` 指定時はUSB ID 2e8a:000aと固有番号の両方で特定します。見つからなくても古いポート番号へフォールバックしません。`power` を省略／無効にした旧設定ではESC単独動作のままで、SSR停止は使えません。`config.local.json` は新Picoファームウェア導入済みのこの機体用に、固有番号を設定し `enabled=true` にしています。配布例では無効です。

```bash
# 電源測定・Statusを見るだけ。SSR・LED・セッションを変更しない
uv run hg-cart power --config config.local.json --seconds 10

# 通常操作。--logには入力・左右ESC状態に加えてpower情報も保存
uv run hg-cart drive --config config.local.json --live --log
```

`drive --live` はPicoとの接続を必須にします。USB給電だけでもESC設定を適用できますが、両ESCのバス電圧がそれぞれの `esc.bus_min_v..bus_max_v` に入るまで回転指令を送りません。約5 V（初期設定3.5〜6.5 V）は `USB_ONLY`、その他の低電圧・SSR OFFは `WAIT_POWER` です。待機中も入力とPicoの通信を続けます。電源復帰後は両スティック中立・十字キー解放を確認してから走行可能になります。INA226がSSRのどちら側にある構成でも、駆動電圧の判定には各ESCのバス電圧を使います。センサ読み出し失敗・古い測定・過電圧では停止します。

**Bは両ESC停止とSSR OFFを指示し、停止を保持します。** Startは両スティック中立・十字キー解放・物理電源ボタン解放中に停止保持だけを解除します。再投入は物理ボタンで行います。Ctrl-C・時間切れ・異常終了でも両ESC停止とSSR OFFを試みます。入力切断は停止保持になります。`relay` / `ssr_command_on` はSSR出力指令で、接点フィードバックや機械的な停止の測定ではありません。ソフトウェア停止は車輪の保持ブレーキではありません。

## LEDとログ

| 状態 | LED |
| --- | --- |
| BOOT / USB_ONLY / WAIT_POWER | 青 |
| READY | 水色 |
| DRIVING | 緑 |
| observer_lock自動復帰待ち | 橙 |
| B・通信断の停止保持 / FAULT | 赤 |
| INA226未取得 | 黄橙 |
| ESP32パネル切断 | 紫 |

先頭24灯は上記状態、末尾6灯は操作モード（水色=tank、橙=joystick）を表示します。Statusの `drive_mode` と `mode_led_rgb` でも確認できます。停止保持・FAULTでは末尾も赤です。停止保持は他の表示より優先します。パネル切断や未取得は走行状態の色より優先します。LED更新は250 ms周期、状態通知は100 ms周期です。Pythonが未接続のときは元の単独動作の色と物理ボタン操作を使います。

ログの `power` に `voltage_v`, `current_a`, `power_w`, `current_saturated`, `ssr_command_on`, `estop`, `status_age_s`, 生の `status` を記録します。生Statusには表示モード、キー、LED色、セッション、通信連番、測定経過時間、停止理由も含みます。

**R100=0.1Ωの現在の測定回路は約±0.819 Aで電流測定が飽和します。** 飽和時は電圧を表示しつつ `current_a` / `power_w` を `null`、`current_saturated=true` にします。各ESCのq軸電流上限2 Aとは別の測定値です。大きな入力電流を測るにはシャントと対応するファームウェア設定の変更が必要です。電流値を根拠にSSRを自動ONする機能や追加の過電流遮断は実装していません。

## USBプロトコル1

ASCII 1行1コマンド、CR/LF、115200設定。`cap` の応答は次の通りです。

```
power_protocol=1 watchdog_ms=750 remote_on=0 led_state=1 estop_latch=1 drive_modes=1 shunt_uohm=100000
```

| コマンド | 意味 |
| --- | --- |
| `status` | 測定・SSR指令・パネル・停止保持・LED・ホスト状態を1行で返す |
| `claim ID` | 非ゼロ32bitセッションを取得。有効な別セッションがあれば拒否。SSRは変更しない |
| `host ID SEQ STATE tank/joystick` | 連番が増加するheartbeat、LED状態と操作モード。停止保持は解除しない |
| `estop` | セッションに関係なくSSR OFF＋停止保持 |
| `clear ID` | 有効な接続、SSR OFF、物理ボタン解放時だけ停止保持を解除。ONしない |
| `release ID` | SSR OFFにして所有権を返す。停止保持は残る |
| `off` | SSR OFF。旧プロトコルとの互換コマンド |
| `mode V/I/P`, `panel`, `buttons`, `scan`, `reset`, `beep` | 既存の表示・診断。`reset`はESP32のリセット |

Pythonの制御ループ自身が100 msごとにheartbeatを送り、ACKを300 ms以内、Statusを設定値（初期500 ms）以内に受信します。Picoはheartbeatが750 ms途絶えるかUSB CDC接続が切れればSSR OFF＋停止保持にします。別スレッドが停止した制御ループの代わりにheartbeatを送り続ける構成ではありません。Pico本体にも2秒のハードウェアwatchdogがあり、再起動時はSSR OFFです。通信復旧だけでは自動再投入しません。

## ビルド

元プロジェクトを参照せず、このツリーからビルドできます。Pico SDK、ARM toolchain、CMake、picotoolを別途インストールしてください。このPCの設定例です。

```bash
python3 tools/build_embedded.py \
  --sdk /home/tako/.pico-sdk/sdk/2.3.1 \
  --toolchain /home/tako/.pico-sdk/toolchain/15_2_Rel1 \
  --picotool-dir /home/tako/.pico-sdk/picotool/2.3.1/picotool \
  --with-esp32
```

`--with-esp32` にはPlatformIO Coreとespressif32@6.12.0が必要です。省略すればPicoだけビルドします。出力は `artifacts/firmware/power-controller/` のUF2、ELF、任意のESP32イメージとSHA-256付きmanifestです。ビルドスクリプト自体は機体を書き換えません。

```bash
.venv/bin/python -m unittest discover -s tests -v
python3 -m unittest discover -s embedded/tests -v
```

Pico書き込み前には走行プログラムを終了し、両ESC停止とSSR OFFを確認します。BOOTSELドライブにUF2をコピーするか、対応picotoolで対象の固有番号を指定して書き込み、検証してください。新Python通信には新Picoファームウェアが必要で、旧版は起動時に拒否します。

## 回転させない実機確認

Pico書き込み後、走行プログラムを終了してから実行します。実際の制御ループへ中立・B・Startだけを入力し、ESCへの回転・armコマンドは検証用ポートでも拒否します。SSR OFF、停止保持解除、heartbeatを止めた場合のラッチ、最後の両PWM OFFを記録します。SSRをONにはしません。

```bash
.venv/bin/python tools/verify_power_standby.py \
  --config config.local.json --serial DE610441238C2E35 \
  --output artifacts/hardware/power-standby-new
```

出力フォルダは新しい名前を指定してください。この試験だけでは実負荷・物理ボタンによるON・SSR接点の実際の遮断性能を確認したことにはなりません。

## この機体への導入記録（2026-11-08）

Pico固有番号 `DE610441238C2E35` に統合版UF2を書き込み、picotoolの読み戻し検証が成功しました。ESP32の表示通信は互換のため、ESP32・左右ESCのファームウェアは変更していません。

- UF2 SHA-256: `31b64e52f795828249cce53bccafcbefec085174bb8016d07610a5ab30763f84`
- 旧Pico全フラッシュの保存・検証済み: `artifacts/hardware/2026-11-08/power-integration/pico-before.uf2`
- 停止試験: `artifacts/hardware/2026-11-08/power-integration/standby-2/`
- USB給電のみの約5 Vで待機、模擬B入力で停止保持、模擬Start入力で保持解除のみ、heartbeat停止で `host_timeout` のラッチを実機確認しました。LED出力値は青と赤を確認しています。
- 最後はSSR出力OFF、停止保持解除、両ESC `idle/fault=none/MOE=0/CCER=0`。回転指令は送っていません。SSR ONからの遮断・実走行はこの試験に含みません。

BOOTSELへはアプリのUSB固有番号を指定して `picotool reboot -u -f` で移行できます。BOOTSELでのUSB文字列はアプリの固有番号と異なったため、`picotool info -a` のflash idとUSB接続位置を照合し、その時点のbus/addressに限定して保存・書き込みしました。bus/addressの数値は再接続で変わるため固定しないでください。

### モード表示対応版（同日更新）

現在の導入版は `drive_modes=1` に対応します。UF2 SHA-256は `05687f73a720474dcd58f4af191db9887ab02f91dc33e0a5ad549aa551444471` です。旧版のプログラムは `artifacts/hardware/2026-11-08/imu/pico-before-modes.uf2`、書き込み・実機検証記録は同じ `imu/` フォルダです。

模擬Xboxロゴ入力でtank/joystickの往復を実際のホスト制御ループへ通し、PicoのStatusで末尾LEDが水色／橙へ変わることを確認しました。WT61の静止校正と連続受信、B/Start・通信断監視も通過しています。両ESCのPWMとSSRは最後にOFF、回転指令は送っていません。接地走行のYawゲイン調整・実際の右旋回によるIMU符号確認は未実施です。
