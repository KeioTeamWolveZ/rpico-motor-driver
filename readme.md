# wolvez2024 rpico-motor-controller
## About
XIAO RP2040を用いてモータの速度・位置制御を行います．

## Requirements
- [Raspberry Pi Pico VSCode Extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)

## Environment
- **Pico SDK v1.5.1** on Raspberry Pi Pico VSCode Extension

## How to build and upload
### 1. Clone Repo
```shell
git clone git@github.com:KeioTeamWolveZ/rpico-motor-driver.git --recurse-submodule
```

### 2. Import project
![screenshot1](https://github.com/KeioTeamWolveZ/rpico-motor-driver/assets/58695125/53a5289f-a410-49ab-803d-996aefd64429)
> [!IMPORTANT]
> SDKのバージョンをv1.5.1にすること．

### 3. Build
右下の `Build UF2` をクリックする．

### 4. Upload
XIAOをBOOTボタン(B)を押しながらパソコンに接続．  
`./build/rpico-motor-driver.uf2` をマイコンのフォルダにドラッグアンドドロップして書き込む．

## Sync stall diagnostics

`MOTORS_SYNC_FAULT_STATUS` は最後に発生した同期制御faultを1行の
`SYNC_FAULT key=value ...` 形式で表示します．faultが未記録の場合は
`SYNC_FAULT state=NONE` を返します．既存の
`ERR SYNC_LEFT_STALL` / `ERR SYNC_RIGHT_STALL` は変更しません．

例:

```text
SYNC_FAULT state=VALID type=RIGHT_STALL wheel_id=0 target_deg=35.569 requested_speed_deg_s=180.000 left_raw_count=123 right_raw_count=456 left_progress_deg=0.277 right_progress_deg=0.000 right_idle_ms=1500 elapsed_ms=1500
```

fault detailは次のfaultで上書きされ，`SAFE` / `STOP` /
`MOTORS_SYNC_CANCEL` では消去されません．明示的に消去する場合は
`MOTORS_SYNC_FAULT_CLEAR` を送信します．
