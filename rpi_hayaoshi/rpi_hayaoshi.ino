#include <Arduino.h>
#include <BTstackLib.h>
#include <SPI.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "button_monitor.pio.h"

#define NUM_BUTTONS 16    // ボタン数
#define NUM_SAMPLES 1     // 1サンプルずつ転送する（エッジ検出ごと）
#define MAX_FIFO_LEVEL 8  // FIFOの最大レベル
#define RESET_COMMAND 'R' // BLE用のリセットコマンド
#define MAX_HISTORY 16            // 最大履歴数
#define ENTRY_SIZE 5              // 各エントリのサイズ（5バイト）
#define TIMESTAMP_MAX 4294967295  // マイクロ秒の最大値（4バイトで表現）
#define BUFFER_SIZE 1000  // DMA転送先のメモリのサイズ
#define CLK 250000 //kHZ クロック

void reset_button_states();

// BLEHandler クラス定義
class BLEHandler {
public:
  BLEHandler() {}

  void setup() {
    Serial.begin(9600);

    // BLEコールバックの設定
    BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
    BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
    BTstack.setGATTCharacteristicRead(gattReadCallback);
    BTstack.setGATTCharacteristicWrite(gattWriteCallback);

    // GATTサービスと特性の設定
    BTstack.addGATTService(new UUID("B8E06067-62AD-41BA-9231-206AE80AB551"));
    characteristic_handle = BTstack.addGATTCharacteristicDynamic(
      new UUID("f897177b-aee8-4767-8ecc-cc694fd5fce0"),
      ATT_PROPERTY_READ | ATT_PROPERTY_WRITE | ATT_PROPERTY_NOTIFY, 0);

    // characteristic_handleからBLECharacteristicを生成
    characteristic = BLECharacteristic(gatt_client_characteristic_t{ characteristic_handle });

    // Bluetoothの起動と広告の開始
    BTstack.setup();
    BTstack.startAdvertising();
  }

  void loop() {
    BTstack.loop();
  }

  // ボタン押下通知と履歴の更新
  void notifyButtonPress(int button_id, uint32_t timestamp) {
    // ボタンIDとタイムスタンプを履歴に保存
    button_press_history[history_index][0] = button_id;
    button_press_history[history_index][1] = (timestamp >> 24) & 0xFF;  // タイムスタンプ上位
    button_press_history[history_index][2] = (timestamp >> 16) & 0xFF;
    button_press_history[history_index][3] = (timestamp >> 8) & 0xFF;
    button_press_history[history_index][4] = timestamp & 0xFF;  // タイムスタンプ下位

    // 履歴のインデックスを更新（最大数に達したら0に戻る）
    history_index = (history_index + 1) % MAX_HISTORY;

    // クライアントに通知
    characteristic_data = button_id + '0';
    BTstack.writeCharacteristicWithoutResponse(
      nullptr, &characteristic, (uint8_t *)&characteristic_data, sizeof(characteristic_data));
  }

  // コールバック：デバイス接続
  static void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
    (void)device;
    if (status == BLE_STATUS_OK) {
      Serial.println("Device connected!");
    }
  }

  // コールバック：デバイス切断
  static void deviceDisconnectedCallback(BLEDevice *device) {
    (void)device;
    Serial.println("Disconnected.");
  }

  // コールバック：GATTリード
  static uint16_t gattReadCallback(uint16_t value_handle, uint8_t *buffer, uint16_t buffer_size) {
    if (buffer && value_handle == characteristic_handle) {
      // 履歴を一括送信（最大80バイト = 16エントリ x 5バイト）
      uint16_t length = MAX_HISTORY * ENTRY_SIZE;
      if (buffer_size < length) {
        length = buffer_size;  // バッファサイズに収まるよう調整
      }
      memcpy(buffer, button_press_history, length);
      return length;
    }
    return 0;
  }

  // コールバック：GATTライト
  static int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
    if (value_handle == characteristic_handle && size > 0) {
      if (buffer[0] == RESET_COMMAND) {
        resetButtonStates();
        Serial.println("Received reset command.");
      }
      return 0;
    }
    return -1;
  }

private:
  static char characteristic_data;
  static uint16_t characteristic_handle;
  static BLECharacteristic characteristic;
  static uint8_t button_press_history[MAX_HISTORY][ENTRY_SIZE];  // ボタン押下履歴
  static uint8_t history_index;

  // ボタン状態のリセット
  static void resetButtonStates() {
    memset(button_press_history, 0, sizeof(button_press_history));  // 履歴をクリア
    history_index = 0;
    Serial.println("Button states have been reset.");
    reset_button_states();
  }
};

// BLEHandlerの静的メンバ初期化
char BLEHandler::characteristic_data = 'N';
uint16_t BLEHandler::characteristic_handle;
BLECharacteristic BLEHandler::characteristic;
uint8_t BLEHandler::button_press_history[MAX_HISTORY][ENTRY_SIZE] = { 0 };
uint8_t BLEHandler::history_index = 0;

// BLEHandlerインスタンス
BLEHandler bleHandler;

// グローバル変数
uint32_t button_press_times[NUM_BUTTONS];  // ボタンのタイムスタンプを記録
uint32_t button_state[BUFFER_SIZE];
// 読み出しポインタの初期化
uint32_t read_pointer = 0;           // 読み込み位置を管理
uint32_t write_pointer = 0;          // 書き込み位置を管理
volatile bool dma_complete = false;  // DMA転送完了フラグ

int dma_chan0 = 0;
bool use_dma_chan0 = true;  // どちらのDMAチャンネルを使うかを示すフラグ
uint32_t reference_time = 0;
uint sm;  // ステートマシン番号をグローバル変数として宣言

// DMA転送再設定関数
void configure_dma_transfer(PIO pio, uint sm, int dma_channel) {
  dma_channel_config c = dma_channel_get_default_config(dma_channel);

  // 読み取り元は固定（PIOのFIFO）
  channel_config_set_read_increment(&c, false);

  // 書き込み先はインクリメント（バッファ内の次の位置に書き込む）
  channel_config_set_write_increment(&c, true);

  // PIOのDREQを使用（DMA要求信号）
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

  // リングバッファの設定
  // 書き込みアドレスを循環させるよう設定（2^6 = 64バイトのリングバッファとして設定）
  channel_config_set_ring(&c, true, 6);

  // DMAの設定（1サンプルずつ転送）
  dma_channel_configure(
    dma_channel,
    &c,
    &button_state[write_pointer],  // 現在の書き込み先アドレス（リングバッファの位置に従う）
    &pio->rxf[sm],                 // PIOのRX FIFOのソース
    1,                             // 1サンプルずつ転送
    true                           // DMA転送をすぐに開始
  );
}

// 32ビットの値をビット形式で表示する関数
void print_bits(uint32_t value) {
  for (int i = 31; i >= 0; i--) {
    Serial.print((value >> i) & 1);  // 各ビットをシフトして取り出し
  }
  Serial.println();
}

// 割り込みハンドラ内でフラグを設定し、次のDMA転送を再設定
void dma_handler0() {
  dma_channel_acknowledge_irq0(dma_chan0);  // 割り込みをクリア
  dma_complete = true;

  // 書き込み位置の更新
  write_pointer = (write_pointer + 1) % BUFFER_SIZE;

  // 次のDMA転送を再設定して開始
  configure_dma_transfer(pio0, sm, dma_chan0);
}

// ボタンピン設定、GPIO初期化
void setup_buttons() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(i, INPUT_PULLUP);  // 各GPIOをプルアップ入力に設定
  }
}

// ボタン状態をリセットする関数
void reset_button_states() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    button_press_times[i] = -1;  // 全てのボタンのタイムスタンプをリセット
  }
  reference_time = micros();  // タイムスタンプを取得
  Serial.println("[INFO]Button states have been reset.");
}

void button_monitor_program_init(PIO pio, uint sm, uint offset, uint pin_base, uint pin_count) {
  pio_sm_config c = button_monitor_program_get_default_config(offset);
  sm_config_set_in_pins(&c, pin_base);
  pio_sm_set_consecutive_pindirs(pio, sm, pin_base, pin_count, false);  // ピンを入力に設定
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);                        // FIFOジョインを有効にしてFIFO容量を拡張
  pio_sm_init(pio, sm, offset, &c);                                     // PIOステートマシン初期化
}

void poll_usbserial_input_non_blocking() {
  static char buffer[128];  // 入力を格納するバッファ
  static int index = 0;     // バッファ内の現在のインデックス

  // USBシリアルが接続されているか確認
  if (Serial) {
    int c = Serial.read();  // 非ブロッキングでシリアルデータを取得

    if (c != -1) {                   // データがある場合のみ処理
      if (c == '\n' || c == '\r') {  // 改行コードを受け取った場合
        buffer[index] = '\0';        // 文字列の終端
        Serial.printf("[CMD]Received USB Serial Input: %s\n", buffer);

        // "reset" コマンドを検出
        if (strcmp(buffer, "reset") == 0) {
          reset_button_states();  // ボタン状態をリセット
        } else {
          Serial.printf("[ERROR]Unknown command: %s\n", buffer);
        }

        // 入力処理後にバッファをリセット
        index = 0;
      } else {
        // 文字をバッファに追加
        if (index < sizeof(buffer) - 1) {
          buffer[index++] = c;
        }
      }
    }
  }
}

void printBinary(uint32_t number, int bits = 32) {
  for (int i = bits - 1; i >= 0; i--) {
    // 各ビットをチェックし、1なら'1'を、0なら'0'を表示
    Serial.print((number & (1 << i)) ? '1' : '0');
  }
  Serial.println();  // 改行を追加
}

// メインプログラム
void setup() {
  // シリアル通信初期化
  Serial.begin(115200);

  // 少し待つ
  delay(1000);

  Serial.println("[INFO]Raspberry-pi-quiz-buzzer");

  // システムクロックを250MHzに設定
  Serial.print("[INFO]Setting system clock to 250MHz...");
  set_sys_clock_khz(CLK, true);
  Serial.println(" done");

  // ボタン配列を初期化
  reset_button_states();

  // ボタンGPIO初期設定
  Serial.print("[INFO]Setting up buttons...");
  setup_buttons();
  Serial.println(" done");

  // PIO初期化
  Serial.print("[INFO]Initialising PIO...");
  PIO pio = pio0;
  uint offset = pio_add_program(pio, &button_monitor_program);
  sm = pio_claim_unused_sm(pio, true);
  button_monitor_program_init(pio, sm, offset, 0, NUM_BUTTONS);  // GPIO 0を基準に設定
  // PIOステートマシンのクロックをシステムクロックに同期
  pio_sm_set_clkdiv(pio, sm, 1.0f);  // クロック分周を1に設定

  // PIOステートマシンを有効化
  pio_sm_set_enabled(pio, sm, true);
  Serial.println(" done");

  // DMAチャンネルを取得
  Serial.print("[INFO]Claiming DMA channels...");
  dma_chan0 = dma_claim_unused_channel(true);
  Serial.println(" done");

  // 初期DMA転送の設定
  Serial.print("[INFO]Configuring DMA transfer...");
  configure_dma_transfer(pio, sm, dma_chan0);  // 初期はdma_chan0を使用
  Serial.println(" done");

  // DMAチャンネルの割り込みを有効化
  Serial.print("[INFO]Enabling DMA channel interrupts...");
  dma_channel_set_irq0_enabled(dma_chan0, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler0);
  irq_set_enabled(DMA_IRQ_0, true);
  Serial.println(" done");

  // タイムスタンプを取得
  reference_time = micros();

  // BLEをセットアップ
  bleHandler.setup();

  // LEDピン（GPIO 25）を出力に設定
  pinMode(LED_BUILTIN, OUTPUT);

  // LEDを点灯
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("[SUCCESS]Ready to press buttons!");
}

void loop() {
  bleHandler.loop();
  poll_usbserial_input_non_blocking();
  // DMAの現在の書き込み位置を取得
  uint32_t write_pointer = dma_channel_hw_addr(dma_chan0)->write_addr;
  write_pointer = (write_pointer - (uintptr_t)button_state) / sizeof(uint32_t);
  // 新しいデータを読み出し、処理する
  while (read_pointer != write_pointer) {
    dma_complete = false;  // フラグをリセット

    uint32_t current_time = micros();             // タイムスタンプを取得
    uint32_t state = button_state[read_pointer];  // DMAで転送されたボタン状態
    int pressed_buttons[NUM_BUTTONS];
    Serial.printf("[PRESS]write_pointer=%d, read_pointer=%d, ", write_pointer, read_pointer);
    printBinary(state);
    int pressed_count = 0;

    // ボタンの押下状態を確認
    for (int j = 0; j < NUM_BUTTONS; j++) {
      if (!(state & (1 << j))) {  // ボタンが押された場合
        if (button_press_times[j] == -1) {
          pressed_buttons[pressed_count++] = j;
        }
      }
    }

    if (pressed_count > 0) {
      // 同着が発生した場合はランダムに選択
      int selected_button;
      if (pressed_count > 1) {
        selected_button = pressed_buttons[random(pressed_count)];
        Serial.printf("[TIE] Randomly selected button id=%d from %d simultaneous presses\n", selected_button + 1, pressed_count);
      } else {
        selected_button = pressed_buttons[0];
        read_pointer = (read_pointer + 1) % BUFFER_SIZE;  // 全部読み終わったら読み取りポインタをインクリメント
      }
      button_press_times[selected_button] = current_time - reference_time;  // タイムスタンプを記録
      float press_time = (current_time - reference_time) / 1000000.0f;
      Serial.printf("[PRESS]id=%d,time=%f\n", selected_button + 1, press_time);
      bleHandler.notifyButtonPress(selected_button + 1, (current_time - reference_time));
    } else {
      read_pointer = (read_pointer + 1) % BUFFER_SIZE;  // 新着情報がない場合も読み取りポインタをインクリメント（チャタリングなど）
    }

    // DMAのエラーステータスを確認
    if (dma_channel_is_busy(dma_chan0) && dma_channel_get_irq0_status(dma_chan0)) {
      Serial.println("[Error]DMA0 transfer error detected!");
      dma_channel_acknowledge_irq0(dma_chan0);  // エラーフラグをクリア
    }
  }
}
