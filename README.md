# so101-ble-teleop

M5Stack AtomS3 を 2 台使い、[SO-101](https://github.com/TheRobotStudio/SO-ARM100) のリーダーアームからフォロワーアームへ BLE で関節角を転送して無線テレオペレーションするファームウェアです。PC は不要です。

```
[Leader arm] --UART--> [AtomS3 (leader)] ~~BLE~~> [AtomS3 (follower)] --UART--> [Follower arm]
   STS3215 x6           GATT Server                 GATT Client                   STS3215 x6
```

## 特徴

- 100Hz で 6 軸の位置を送信 (接続間隔 7.5〜15ms)
- AtomS3 の画面に接続状態・サーボ応答・各関節位置を表示
- 画面ボタンのメニューで **キャリブレーション** と **ペアリング** を操作
- 位置値の 4095→0 跨ぎによる急激な動作を防止
- 通信途絶時は位置保持、再接続時は低速でリーダー姿勢へ寄せる

## ハードウェア

| 項目 | 内容 |
|---|---|
| マイコン | M5Stack AtomS3 x2 |
| サーボ | Feetech STS3215 x6 (ID 1〜6、1Mbps) |
| 接続 | AtomS3 **G7 = TX**, **G8 = RX** → 半二重変換付きサーボアダプタ (Waveshare Bus Servo Adapter / FE-URT-1 など) の UART 端子 |

> STS3215 は 1 線式の半二重通信です。G7/G8 をサーボ信号線に直結せず、半二重変換を行うアダプタを介してください。GND は共通にします。

サーボ ID とボーレートは LeRobot の `setup_motors` 実行後の状態 (ID 1〜6 = shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper / 1Mbps) を前提としています。

## ビルド・書き込み

[PlatformIO](https://platformio.org/) を使います。

```sh
pio run -e leader   -t upload   # リーダー側 AtomS3
pio run -e follower -t upload   # フォロワー側 AtomS3
```

## 使い方

画面ボタンで操作します。

| 画面 | クリック | 長押し (0.8 秒) |
|---|---|---|
| 通常 | — | メニューを開く |
| メニュー | 次の項目 | 決定 |
| 確認画面 | キャンセル | 実行 |

### 1. キャリブレーション (各アームで 1 回)

1. メニュー → `Calibrate`
2. アームを基本姿勢にする (フォロワーはこの時点で脱力するので支えてください)
3. 長押しで実行。`OK 6/6` と各軸 2048 付近が表示されれば完了

基本姿勢で位置値が 2048 になるよう、**各サーボの EEPROM (位置オフセット) を書き換えます**。角度リミットも同じ量だけシフトします。可動範囲が 0/4095 の境界から離れるため、跨ぎによる急変を根本的に防げます。

> ⚠ LeRobot でキャリブレーションした Homing_Offset は上書きされます。LeRobot に戻す場合は再キャリブレーションしてください。

### 2. ペアリング

1. リーダー・フォロワーの両方でメニュー → `Pairing`
2. フォロワーが最初に見つけたペアリング待ちのリーダーと接続し、両方に相手のアドレスが保存されます

ペアリングは 1:1 で、実行するたびに以前のペア情報はクリアされます。通常時はペア相手以外とは接続しません。

### 3. 操作

両方を起動するとペア相手と自動接続し、フォロワーが `FOLLOWING (SOFT)` で低速にリーダー姿勢へ寄せた後、`FOLLOWING` で追従します。

## 安全機構

- **跨ぎ対策 (リーダー)**: 生値の半周以上の変化は跨ぎとみなして連続値に展開し、送信値は 0〜4095 で飽和させます
- **急変防止 (フォロワー)**: 1 パケットあたりの移動量を 200 step に制限。接続直後・途絶復帰時は 8 step に制限して寄せます
- **リミット**: 各サーボの角度リミット内に目標値をクランプ
- **フェイルセーフ**: 200ms 受信が途切れたら最後の位置で保持
- トルク ON 前に目標位置を現在位置に合わせ、古い目標への急な移動を防止

赤く表示された関節は、跨ぎで飽和中 / リミットでクランプ中 / 移動量制限中であることを示します。

## 構成

```
include/so101_link.h        BLE UUID・パケット定義 (共通)
src/common/so101_bus.*      サーボアクセス (FTServo ラッパ)・キャリブレーション
src/common/so101_motion.h   跨ぎ展開・移動量制限
src/common/peer_store.*     ペア相手の保存 (NVS)
src/common/ui.*             画面表示・ボタン (M5Unified)
src/leader/main.cpp         リーダー
src/follower/main.cpp       フォロワー
```

## 使用ライブラリ

- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) 1.4
- [FTServo](https://github.com/ftservo/FTServo_Arduino) (FEETECH 公式)
- [M5Unified](https://github.com/m5stack/M5Unified)

## ライセンス

[MIT](LICENSE)
