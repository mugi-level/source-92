// VGMファイルから曲データを読み込んで、YM2608Bから音を出す
// 再生するVGMファイルはSDカードに格納しておいてくださいな
// Copyright (c) 2026 Mugio (mugio_ch)
// This software is released under the MIT License, see LICENSE.

// WiFi機能を無効に
// 2026.06.12
// +ADPCM データブロック処理を追加 0x67コマンド
// +GPIOピン節約するためにボタンを分圧で接続
// 2026.06.13
// +ADPCM 処理内容を修正 0x57コマンド
// release version 1.7 at 2026.09.13

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "hal/gpio_ll.h"
#include "driver/ledc.h"
#include <WiFi.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// --- ピン定義 ---
//GPIO 10,11,12,13,14 SD Card
//GPIO  0, 3,45,46 Boot Strapping
//GPIO 19,20 USB
//GPIO 43,44 UART
//GPIO 35,36,37 PSRAM

//YM2608 Control
const int PIN_D[8] =  {4, 5, 6, 7, 15, 16, 17, 18}; // D0～D7
const int OPNA_CLK  =  1;
const int OPNA_A0   =  2; 
const int OPNA_A1   = 42; 
const int OPNA_WR   = 41;
const int OPNA_CS   = 40;
const int OPNA_IC   = 39;
//etc.
const int SD_CS     = 10;
const int I2C_SDA   =  8;
const int I2C_SCL   =  9;
const int BTN_ADC_PIN = 14;

// --- ボタンIDの定義 ---
enum ButtonID {
  BTN_NONE = 0,
  BTN_1_ID,
  BTN_2_ID,
  BTN_3_ID,
  BTN_4_ID
};

// --- 抵抗分圧のADC閾値設定 (ESP32: 12bit / 0～4095) ---
const int BTN1_ADC_VAL =    0; // BTN1 押下時のADC値
const int BTN2_ADC_VAL =  830; // BTN2 押下時のADC値
const int BTN3_ADC_VAL = 1630; // BTN3 押下時のADC値
const int BTN4_ADC_VAL = 3040; // BTN4 押下時のADC値
const int ADC_MARGIN   =  200; // 許容誤差（±200）

// --- 高速レジスタ操作用の配線マスク定義 ---
// D0~D7が使用する全GPIOピンのビット論理和（マスク）をあらかじめ計算しておく
// GPIO 4, 5, 6, 7, 15, 16, 17, 18
const uint32_t BUS_PIN_MASK = (1ULL << 4)  | (1ULL << 5)  | (1ULL << 6)  | (1ULL << 7) |
                              (1ULL << 15) | (1ULL << 16) | (1ULL << 17) | (1ULL << 18);

// 8ビットのデータ（0〜255）を各GPIOのビット配置へ一瞬で変換するためのルックアップテーブル（LUT）
// 毎回ビットシフトのループを回すと遅いため、256バイトの配列としてPSRAMや内蔵RAMに展開します。
uint32_t data_to_gpio_lut[256];

// --- グローバル変数 ---
//File vgmFile;
bool isPlaying = false;
uint8_t* vgm_data_buffer = NULL; // PSRAM上のVGM全データバッファ
size_t vgm_file_size = 0;        // ファイルの総サイズ
size_t vgm_ptr = 0;              // 現在の読み込みファイルポインタ（インデックス）
uint8_t current_block_id;
uint32_t vgm_loop_offset = 0; 
uint32_t vgm_YM2608_clock = 8000000;
uint8_t vgm_loop_count = 0;
uint8_t vgm_loops = 2;
uint64_t next_vgm_execute_us = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- 状態管理用 ---
enum Mode { MODE_SELECT_DIR, MODE_SELECT_FILE };
Mode currentMode = MODE_SELECT_DIR;
String currentDir = "/"; // 現在表示中のフォルダパス
std::vector<String> dirList;  // フォルダ一覧
std::vector<String> fileList; // ファイル一覧
int dirIndex = 0;
int fileIndex = 0;

// ADPCMデータの一時格納バッファ (0x67コマンド用)
#define MAX_ADPCM_Block 16
struct AdpcmBlock {
  uint8_t* buffer = NULL;
  size_t length = 0;
};

AdpcmBlock ADPCM_Blocks[MAX_ADPCM_Block];

// --- 割り込み同期用の変数（volatile指定が必須） ---
volatile uint8_t* adpcm_play_ptr = NULL;
volatile size_t adpcm_remaining_bytes = 0;

// --- ハードウェア制御関数群 ---
// --- 高速化されたデータバス書き込み関数 ---
void IRAM_ATTR writeDataBus(uint8_t value) {
  // 1. ルックアップテーブルから、この値に対応するGPIOのビットパターン（Hにすべきピン）を一瞬で取得
  uint32_t set_mask = data_to_gpio_lut[value];
  
  // 2. 逆に、Lにすべきピンのマスクを計算（全バスピンのうち、Hにならないピン）
  uint32_t clear_mask = BUS_PIN_MASK & (~set_mask);

  // 3. レジスタへ直接書き込み（1〜2クロックで全ピンが同時に確定する）
  // ※ESP32-S3のGPIO 0〜31は GPIO.out_w1ts / w1tc で制御します
  GPIO.out_w1tc = clear_mask; // Lにしたいピンを同時に引き下げる
  GPIO.out_w1ts = set_mask;   // Hにしたいピンを同時に引き上げる
}

// 配列から1バイト読み込む代替関数
uint8_t readVgmByte() {
  if (vgm_ptr < vgm_file_size) {
    return vgm_data_buffer[vgm_ptr++];
  }
  return 0x66; // 万が一範囲を超えたらEndコマンドを返す
}

// 配列から複数バイト読み込む代替関数
void readVgmBytes(uint8_t* dest, size_t len) {
  if (vgm_ptr + len <= vgm_file_size) {
    memcpy(dest, &vgm_data_buffer[vgm_ptr], len);
    vgm_ptr += len;
  }
}

// YM2608にデータを書き込む
void writeOPNA(uint8_t port, uint8_t reg, uint8_t data) {
  portDISABLE_INTERRUPTS(); // OPNA書き込み中、データバスを独占する
  digitalWrite(OPNA_A1, port);
  digitalWrite(OPNA_A0, LOW);
  writeDataBus(reg);// adr. write
  digitalWrite(OPNA_CS, LOW); 
  digitalWrite(OPNA_WR, LOW);
  delayMicroseconds(1);
  digitalWrite(OPNA_WR, HIGH); 
  digitalWrite(OPNA_CS, HIGH);
  delayMicroseconds(2); 
  digitalWrite(OPNA_A0, HIGH);
  writeDataBus(data);// dat. write
  digitalWrite(OPNA_CS, LOW); 
  digitalWrite(OPNA_WR, LOW);
  delayMicroseconds(1);
  digitalWrite(OPNA_WR, HIGH); 
  digitalWrite(OPNA_CS, HIGH);
  portENABLE_INTERRUPTS(); // 解放
  delayMicroseconds(10); 
}

// VGMファイルのウェイト処理
void vgmWaitSamples(uint32_t samples) {
  if (samples == 0) return;

  // サンプル数から、待つべき正確なマイクロ秒（浮動小数点を用いて誤差を無くす）を計算し、目標時刻に加算
  next_vgm_execute_us += (uint64_t)((double)samples * 1000000.0 / 44100.0);

  // 目標時刻になるまでひたすら待つ（ビジーループ）
  while ((uint64_t)esp_timer_get_time() < next_vgm_execute_us) {
    // 1ミリ秒以上待つ必要がある場合は、他のタスク（WDTやシステム用）に一瞬だけ譲る
    if ((next_vgm_execute_us - esp_timer_get_time()) > 2000) {
      vTaskDelay(1); 
    } else {
      asm volatile("nop;"); // 短い時間はNOPで超精密に待つ
    }
  }
}

// GPIO書き込み処理
void initDataBusLUT() {
  for (int val = 0; val < 256; val++) {
    uint32_t gpio_bits = 0;
    if ((val >> 0) & 1) gpio_bits |= (1ULL << 4);  // D0 -> GPIO4
    if ((val >> 1) & 1) gpio_bits |= (1ULL << 5);  // D1 -> GPIO5
    if ((val >> 2) & 1) gpio_bits |= (1ULL << 6);  // D2 -> GPIO6
    if ((val >> 3) & 1) gpio_bits |= (1ULL << 7);  // D3 -> GPIO7
    if ((val >> 4) & 1) gpio_bits |= (1ULL << 15); // D4 -> GPIO15
    if ((val >> 5) & 1) gpio_bits |= (1ULL << 16); // D5 -> GPIO16
    if ((val >> 6) & 1) gpio_bits |= (1ULL << 17); // D6 -> GPIO17
    if ((val >> 7) & 1) gpio_bits |= (1ULL << 18); // D7 -> GPIO18
    
    data_to_gpio_lut[val] = gpio_bits;
  }
}

// リトルエンディアン処理
uint32_t readLE(uint8_t* buf, size_t size) {
  uint32_t val = 0;
  for (size_t i = 0; i < size; i++) {
    val |= (buf[i] << (8 * i));
  }
  return val;
}

// セットアップ
void setup() {
  WiFi.disconnect(true);
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("Wire_begin");
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  Serial.println("SSD1306_begin");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }

  pinMode(BTN_ADC_PIN, INPUT);
  analogReadResolution(12); // 12bit分解能 (0~4095)
  
  // GPIOからクロック出力
  pinMode(OPNA_CLK, OUTPUT); 
  initYM2608Clock(OPNA_CLK);

  // YM2608 ピン初期化
  for (int i = 0; i < 8; i++) {
    pinMode(PIN_D[i], OUTPUT);
  }
  pinMode(OPNA_A0, OUTPUT); pinMode(OPNA_WR, OUTPUT); 
  pinMode(OPNA_CS, OUTPUT); pinMode(OPNA_IC, OUTPUT);
  pinMode(OPNA_A1, OUTPUT);
  digitalWrite(OPNA_CS, HIGH); 
  digitalWrite(OPNA_WR, HIGH); digitalWrite(OPNA_A0, HIGH); 
  digitalWrite(OPNA_A1, LOW);
  Serial.println("Resetting YM2608..."); 
  digitalWrite(OPNA_IC, LOW);
  delay(50);
  digitalWrite(OPNA_IC, HIGH);
  muteOPNA();

  // 初回のファイルリスト取得と描画
  updateLists();
  drawDisplay();
  initDataBusLUT();//データバス用レジスタの初期化
}

// --- メインループ ---
void loop(){
  bool changed = false;

  if (!isPlaying) {
  // ボタン1: 次へ (旧: BTN2 -> BTN_2_ID)
  if (checkButton(BTN_2_ID)) {
    if (currentMode == MODE_SELECT_DIR && !dirList.empty()) {
      dirIndex = (dirIndex + 1) % dirList.size();
      if (dirList[dirIndex] == "System Volume Information") dirIndex = 0;
    } else if (currentMode == MODE_SELECT_FILE && !fileList.empty()) {
      fileIndex = (fileIndex + 1) % fileList.size();
    }
    changed = true;
  }

  // ボタン2: 前へ (旧: BTN1 -> BTN_1_ID)
  if (checkButton(BTN_1_ID)) {
    if (currentMode == MODE_SELECT_DIR && !dirList.empty()) {
      dirIndex = (dirIndex - 1 + dirList.size()) % dirList.size();
      if(dirIndex == 1) dirIndex = 0;
    } else if (currentMode == MODE_SELECT_FILE && !fileList.empty()) {
      fileIndex = (fileIndex - 1 + fileList.size()) % fileList.size();
    }
    changed = true;
  }

  // ボタン3: 決定 (旧: BTN3 -> BTN_3_ID)
  if (checkButton(BTN_3_ID)) {
    if (currentMode == MODE_SELECT_DIR) {
      if (!dirList.empty()) {
        if (dirList[dirIndex] == "[ root ]") {
          currentDir = "/";
        } else {
          currentDir = "/" + dirList[dirIndex] + "/";
        }
        currentMode = MODE_SELECT_FILE;
        fileIndex = 0;
        updateLists();
      }
    } else if (currentMode == MODE_SELECT_FILE) {
      if (!fileList.empty()) {
        String fullPath = currentDir + fileList[fileIndex];
        loadFileToPSRAM(fullPath);
      }
    }
    changed = true;
  }

  // ボタン4: 戻る (旧: BTN4 -> BTN_4_ID)
  if (checkButton(BTN_4_ID)) {
    if (currentMode == MODE_SELECT_FILE) {
      currentMode = MODE_SELECT_DIR;
      changed = true;
    }
  }

    if (!isPlaying && changed) {
      drawDisplay();
    }
    delay(10); // チャタリング防止用小休止

  } else {

    VGM_parser();
  }
}

//VGMファイルを読み込んで演奏開始
void PSRAM_use(){ 
  // VGMヘッダ解析 (メモリ上で行う)
  uint8_t offsetBuf[4];

  vgm_ptr = 0x1c;//ループ位置
  readVgmBytes(offsetBuf, 4);
  uint32_t raw_loop_offset = readLE(offsetBuf,4);
  if (raw_loop_offset == 0) {
    vgm_loop_offset = 0; // ループなし
    } else {
    vgm_loop_offset = 0x1c + raw_loop_offset; // 実際のデータ配列のインデックス
    Serial.printf("Loop offset:%d\n", vgm_loop_offset);
    vgm_loop_count = 0;
  }

  vgm_ptr = 0x48; // YM2608クロック
  readVgmBytes(offsetBuf, 4);
  uint32_t ym_clock_set = readLE(offsetBuf, 4);
  vgm_YM2608_clock = ym_clock_set;
  Serial.printf("YM2608 Clock:%d\n", vgm_YM2608_clock);

  vgm_ptr = 0x34; // 演奏開始位置
  readVgmBytes(offsetBuf, 4);
  uint32_t data_offset = readLE(offsetBuf, 4);
  vgm_ptr = 0x34 + data_offset;  // 演奏開始位置（データストリームの先頭）へシーク
  Serial.println("VGM Playback Started (Timer Interrupt Mode)...");
  isPlaying = true;
  current_block_id = 0; 
  next_vgm_execute_us = esp_timer_get_time(); // micros()の代わりに高精度なesp_timerを使用
}

// VGMファイルのパーサー
void VGM_parser() {
  if (!isPlaying || vgm_data_buffer == NULL) {
    delay(100);
    return;
  }

  // ファイルの終端チェック
  if (vgm_ptr >= vgm_file_size) {
    Serial.println("Reached end of memory buffer.");
    isPlaying = false;
    return;
  }

  // 再生中にボタン4 (BTN4) が押されたら、強制的に再生終了処理へ移行する
  // チャタリングを考慮し、LOW（押されている）かつ少し待ってもLOWなら中断とみなす
  if (getPressedButton() == BTN_4_ID) {
    delayMicroseconds(1000); 
    if (getPressedButton() == BTN_4_ID) {
      Serial.println("[VGM] Interrupted by BTN4.");
      
      // ボタンが離されるまで待つ
      while(getPressedButton() == BTN_4_ID) {
        vTaskDelay(1); 
      }
      
      // 0x66（終端）と同じ消音・終了処理をここで実行
      muteOPNA();
      portDISABLE_INTERRUPTS();
      adpcm_play_ptr = NULL; adpcm_remaining_bytes = 0;
      portENABLE_INTERRUPTS();      
      isPlaying = false;
      currentMode = MODE_SELECT_FILE; // ファイル選択モードに戻す
      drawDisplay();                  // ディスプレイを再描画
      return;
    }
  }

  uint8_t cmd = readVgmByte(); 
  uint8_t paramBuf[16]; 
  uint32_t w_freq;
  
  switch (cmd) {
    case 0x56://YM2608 Port 0
      readVgmBytes(paramBuf, 2); 
      writeOPNA(0,paramBuf[0], paramBuf[1]);//YM2608へ書き込み
      break;

case 0x57: { // YM2608 Port 1
      // アドレス計算用の一時変数（VGMの連続再生でも状態を保持できるようにstatic）
      static uint16_t b_start = 0, b_stop = 0, b_limit = 0xFFFF;
      static bool is_1bit = false;
      
      uint8_t reg = readVgmByte();
      uint8_t dat = readVgmByte();
      
      if (reg == 0x01) {
        // Control 2: Bit1 が 0 ならVGMは 1-bit RAM モードを意図している
        is_1bit = ((dat & 0x02) == 0);
        dat = dat | 0x02; // 実機は8-bit RAMなので強制的に8-bitモードへ
        writeOPNA(1, reg, dat);
      }
      else if (reg == 0x02) { // Start Address L
        b_start = (b_start & 0xFF00) | dat; 
        uint16_t v = is_1bit ? b_start / 8 : b_start; 
        writeOPNA(1, 0x02, v & 0xFF); writeOPNA(1, 0x03, v >> 8); 
      }
      else if (reg == 0x03) { // Start Address H
        b_start = (b_start & 0x00FF) | (dat << 8); 
        uint16_t v = is_1bit ? b_start / 8 : b_start; 
        writeOPNA(1, 0x02, v & 0xFF); writeOPNA(1, 0x03, v >> 8); 
      }
      else if (reg == 0x04) { // Stop Address L
        b_stop = (b_stop & 0xFF00) | dat; 
        uint16_t v = is_1bit ? b_stop / 8 : b_stop; 
        writeOPNA(1, 0x04, v & 0xFF); writeOPNA(1, 0x05, v >> 8); 
      }
      else if (reg == 0x05) { // Stop Address H
        b_stop = (b_stop & 0x00FF) | (dat << 8); 
        uint16_t v = is_1bit ? b_stop / 8 : b_stop; 
        writeOPNA(1, 0x04, v & 0xFF); writeOPNA(1, 0x05, v >> 8); 
      }
      else if (reg == 0x0C) { // Limit Address L
        b_limit = (b_limit & 0xFF00) | dat; 
        uint16_t v = is_1bit ? b_limit / 8 : b_limit; 
        writeOPNA(1, 0x0C, v & 0xFF); writeOPNA(1, 0x0D, v >> 8); 
      }
      else if (reg == 0x0D) { // Limit Address H
        b_limit = (b_limit & 0x00FF) | (dat << 8); 
        uint16_t v = is_1bit ? b_limit / 8 : b_limit; 
        writeOPNA(1, 0x0C, v & 0xFF); writeOPNA(1, 0x0D, v >> 8); 
      }
      else {
        writeOPNA(1, reg, dat);
      }
      break;
    }

    case 0x61://wait
      readVgmBytes(paramBuf, 2);
      vgmWaitSamples(readLE(paramBuf, 2));
      break;
    
    case 0x62://wait 735 samples
      vgmWaitSamples(735);
      break;

    case 0x63://wait 882 samples
      vgmWaitSamples(882);
      break;

    case 0x66://終端
      vgm_loop_count ++;
      if (vgm_loop_offset > 0 and vgm_loop_count <= vgm_loops) {
        // ループポイントが存在する場合、ポインタをループ先に戻して演奏を続行！
        vgm_ptr = vgm_loop_offset;
        Serial.println("[VGM] Loop triggered!");
        break;
      } else {
        // ループがない曲はそのまま終了
        Serial.println("[VGM] Playback End.");
        muteOPNA();
        portDISABLE_INTERRUPTS();
        adpcm_play_ptr = NULL; adpcm_remaining_bytes = 0;
        portENABLE_INTERRUPTS();      
        isPlaying = false;
        currentMode = MODE_SELECT_FILE; // ファイル選択モードに戻す
        drawDisplay();                  // ディスプレイを再描画
      }
      break;
      
    case 0x67: { // Data Block
      uint8_t dummy = readVgmByte(); // 0x66 を読み飛ばす
      uint8_t data_type = readVgmByte(); // データタイプ (0x81ならYM2608 DELTA-T)
      
      // データサイズ(4バイト)を取得
      uint32_t data_size = readLE(&vgm_data_buffer[vgm_ptr], 4);
      vgm_ptr += 4;
      
      if (data_type == 0x81) {
        // ROMサイズ(4バイト)を取得 (今回は使わないので読み飛ばし)
        uint32_t rom_size = readLE(&vgm_data_buffer[vgm_ptr], 4);
        vgm_ptr += 4;
        
        // DRAM上の開始アドレス(4バイト)を取得
        uint32_t start_addr = readLE(&vgm_data_buffer[vgm_ptr], 4);
        vgm_ptr += 4;
        
        // 実際の波形データサイズ (データサイズ全体から ヘッダの8バイト分を引いた値)
        uint32_t payload_size = data_size - 8;
        
        // --- ここでESP32からYM2608のDRAMへ直接転送を実行 ---
        Serial.printf("ADPCM-B Loading... Start:0x%04X, Size:%d\n", start_addr, payload_size);
        writeYM2608_ADPCM_B_RAM(start_addr, &vgm_data_buffer[vgm_ptr], payload_size);
        
        // ポインタを書き込んだデータの分だけ進める
        vgm_ptr += payload_size;
      } else {
        // YM2608のADPCM-B以外のデータブロックの場合はスキップする
        vgm_ptr += data_size;
      }
      next_vgm_execute_us = esp_timer_get_time(); // (環境によっては micros() を使用してください)
      break;    
    }

    default:
      if ((cmd & 0xF0) == 0x70) {
        vgmWaitSamples((cmd & 0x0F) + 1);
      }
      break;
  }
}

// 指定したディレクトリ内のフォルダ・ファイル一覧を更新する関数
void updateLists() {
  dirList.clear();
  fileList.clear();

  // フォルダ一覧は常にルート直下を検索（仕様に合わせて調整可能）
  File root = SD.open("/");
  if(root){
    // ルート自体を選択肢に含める用
    dirList.push_back("[ ROOT ]");
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      if (entry.isDirectory()) {
        dirList.push_back(String(entry.name()));
      }
      entry.close();
    }
    root.close();
  }

  // 現在選択されたフォルダ内のファイルを検索
  File dir = SD.open(currentDir);
  if(dir){
    while (true) {
      File entry = dir.openNextFile();
      if (!entry) break;
      if (!entry.isDirectory()) {
        fileList.push_back(String(entry.name()));
      }
      entry.close();
    }
    dir.close();
  }
}

// OLED描画処理
void drawDisplay() {
  display.clearDisplay();
  
  // 1行目: フォルダ情報の表示
  display.setCursor(0, 0);
  if (currentMode == MODE_SELECT_DIR) {
    display.print("Dir: ");
    if (!dirList.empty()) display.println(dirList[dirIndex]);
    else display.println("No Dirs");
  } else {
    display.print("Dir: ");
    display.println(currentDir);
  }

  // 2行目: ファイル情報の表示
  display.setCursor(0, 16); // テキストサイズ1の場合、y=16で2行目へ
  if (currentMode == MODE_SELECT_FILE) {
    display.print("VGM: ");
    if (!fileList.empty()) display.println(fileList[fileIndex]);
    else display.println("No Files");
  } else {
    display.println("(Select Dir First)");
  }
  display.display();
}

// 現在押されているボタンのIDを取得する
ButtonID getPressedButton() {
  int val = analogRead(BTN_ADC_PIN);
    if (abs(val - BTN1_ADC_VAL) < ADC_MARGIN) return BTN_1_ID;
  if (abs(val - BTN2_ADC_VAL) < ADC_MARGIN) return BTN_2_ID;
  if (abs(val - BTN3_ADC_VAL) < ADC_MARGIN) return BTN_3_ID;
  if (abs(val - BTN4_ADC_VAL) < ADC_MARGIN) return BTN_4_ID;
    return BTN_NONE; // どのボタンも押されていない
}

// 簡易的なチャタリング防止＆ボタン離脱待ち付きの判定関数
bool checkButton(ButtonID targetBtn) {
  if (getPressedButton() == targetBtn) {
    delay(50); // デバウンス
    if (getPressedButton() == targetBtn) {
      // ボタンが離されるまで待つ（または判定から外れるまで）
      while (getPressedButton() == targetBtn) {
        delay(10);
      }
      return true;
    }
  }
  return false;
}

// PSRAMへの格納処理
void loadFileToPSRAM(String path) {
  // 既にバッファにデータがある場合は解放
  if (vgm_data_buffer != nullptr) {
    free(vgm_data_buffer);
    vgm_data_buffer = nullptr;
    Serial.println("既存のバッファを解放しました。");
  }

  Serial.print("ファイルを読み込み中: ");
  Serial.println(path);

  File vgmFile = SD.open(path, FILE_READ);
  if (!vgmFile) {
    Serial.println("ファイルのオープンに失敗しました。");
    return;
  }

  vgm_file_size = vgmFile.size();
  Serial.print("ファイルサイズ: ");
  Serial.print(vgm_file_size);
  Serial.println(" bytes");

  // ps_malloc を使用してPSRAM上に領域を確保
  vgm_data_buffer = (uint8_t*)ps_malloc(vgm_file_size);

  if (vgm_data_buffer == nullptr) {
    Serial.println("PSRAMのメモリ確保に失敗しました");
    vgmFile.close();
    return;
  }

  // データの読み込み
  size_t readSize = vgmFile.read(vgm_data_buffer, vgm_file_size);
  vgmFile.close();

  Serial.print("PSRAMへの格納が完了しました。読み込みサイズ: ");
  Serial.println(readSize);

  // OLEDに完了画面を一時表示
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(path);
  display.setCursor(0, 24);
  display.println("Playing...");
  display.display();
  PSRAM_use();
}

void initYM2608Clock(int clockPin) {
  // LEDCタイマーの設定 (YM2608のクロック8MHz用)
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .duty_resolution  = LEDC_TIMER_1_BIT, // 1-bit（0か1）で50%デューティを作る
    .timer_num        = LEDC_TIMER_0,
    .freq_hz          = vgm_YM2608_clock,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // LEDCチャンネルの設定
  ledc_channel_config_t ledc_channel = {
    .gpio_num       = clockPin,
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = LEDC_TIMER_0,
    .duty           = 1,                  // デューティ比 50% (1-bit解像度なので1で半分)
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel);  
  Serial.println("YM2608 Master Clock initialized on GPIO.");
}

// YM2608の消音
void muteOPNA(){
  // --- FM音源の消音処理 (すべてのチャンネルをキーオフ) ---
  // YM2608のFMキーオン/オフポインタは Port 0 の 0x28 レジスタです。
  // 下位3ビットでチャンネル指定、上位4ビットで各オペレータのオフ(0)を設定します。
  for (uint8_t ch = 0; ch < 3; ch++) {
    writeOPNA(0, 0x28, 0x00 | ch); // FM Ch1〜3 キーオフ
    writeOPNA(0, 0x28, 0x04 | ch); // FM Ch4〜6 キーオフ
    writeOPNA(0, 0x08, 0x00); // SSG Ch-A 音量0
    writeOPNA(0, 0x09, 0x00); // SSG Ch-B 音量0
    writeOPNA(0, 0x0A, 0x00); // SSG Ch-C 音量0
    writeOPNA(1, 0x10, 0x80); // ADPCM flag reset <- 不要かもしれない
  }
}

// -----------------------------------------------------
// YM2608のDRAMへADPCMデータを転送する関数（修正版）
// -----------------------------------------------------
void writeYM2608_ADPCM_B_RAM(uint32_t start_addr, const uint8_t* data, uint32_t length) {
  if (length == 0 || data == NULL) return;

  // 1. ADPCM-B リセット
  writeOPNA(1, 0x00, 0x01); 
  delayMicroseconds(10);

  // 2. Control 2: メモリタイプ設定 (RAM 8-bitの場合は 0x02)
  writeOPNA(1, 0x01, 0x02); 

  // 3. YM2608のアドレスは32バイト(0x20)境界でしか指定できないため、切り捨てたアドレスをセット
  uint32_t aligned_start_addr = start_addr & ~0x1F;
  writeOPNA(1, 0x02, (aligned_start_addr >> 5) & 0xFF);
  writeOPNA(1, 0x03, (aligned_start_addr >> 13) & 0xFF);

  // 4. 終了アドレスは安全のため最大値(0xFFFF)にしておく（途中で書き込みが止まらないように）
  writeOPNA(1, 0x04, 0xFF);
  writeOPNA(1, 0x05, 0xFF);

  // 5. データ書き込みモード開始
  writeOPNA(1, 0x00, 0x60); 
  delayMicroseconds(10);

  // 6. 【重要】32バイト境界からの「ズレ(端数)」分だけ無音データ(0x80)を空打ちし、
  // 内部アドレスカウンタを実際の start_addr まで進める
  uint32_t offset = start_addr & 0x1F;
  for(uint32_t i = 0; i < offset; i++) {
    writeOPNA(1, 0x08, 0x80); // 0x80はADPCMの無音データ
  }

  // 7. 実際の波形データを流し込む
  for(uint32_t i = 0; i < length; i++) {
    writeOPNA(1, 0x08, data[i]);
  }

  // 8. 書き込み終了 (リセット状態に戻す)
  writeOPNA(1, 0x00, 0x01); 
}
