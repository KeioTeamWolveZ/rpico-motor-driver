# wolvez2024 rpico-motor-controller
## About
XIAO RP2040を用いてモータの速度・位置制御を行います．

## Requirements
- [Raspberry Pi Pico VSCode Extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)

## How to build and upload
### 1. Clone Repo
```shell
git clone git@github.com:KeioTeamWolveZ/rpico-motor-driver.git --recurse-submodule
```

### 2. Import project
![screenshot1](https://github.com/KeioTeamWolveZ/rpico-motor-driver/assets/58695125/53a5289f-a410-49ab-803d-996aefd64429)

### 3. Build
右下の `Build UF2` をクリックする．

### 4. Upload
XIAOをBOOTボタン(B)を押しながらパソコンに接続．  
`./build/rpico-motor-driver.uf2` をマイコンのフォルダにドラッグアンドドロップして書き込む．
