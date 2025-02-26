#include <esp_now.h>
#include <WiFi.h>
#include <driver/i2s.h>

#define AMP_BCLK  26    // BCLK untuk MAX98357A (Master)
#define AMP_LRCLK 25    // LRCLK
#define AMP_DATA  22    // Data Out
#define BOOT_BUTTON 0   // Tombol BOOT bawaan ESP32
#define MUTE_BUTTON 33  // Tombol untuk menonaktifkan speaker

// MAC Address Slave (ubah sesuai dengan unit Slave Anda)
uint8_t peerAddress[] = {0xCC, 0xDB, 0xA7, 0x94, 0x85, 0x50};

typedef struct {
  int frequency;
} Message;

Message sendData;
Message rxMessage;

// Flag untuk mengizinkan pengiriman hanya sekali
bool readyToSend = true;     
bool buttonTriggered = false; // untuk mencegah trigger berulang selama tombol ditekan
bool busy = false;            // indikator unit sedang memproses
bool speakerEnabled = true;   // Flag untuk status speaker

// Inisialisasi I2S untuk menghasilkan tone
void i2sInit() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = AMP_BCLK,
    .ws_io_num = AMP_LRCLK,
    .data_out_num = AMP_DATA,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// Fungsi untuk menghasilkan tone melalui I2S
void generateTone(int freq, int duration_ms) {
  if (!speakerEnabled) return; // Jika speaker dimatikan, jangan memutar suara

  const int sampleRate = 44100;
  const int amplitude = 3000;
  int numSamples = sampleRate / freq;
  int16_t samples[numSamples];

  for (int i = 0; i < numSamples; i++) {
    samples[i] = amplitude * sin(2.0 * PI * freq * i / sampleRate);
  }

  size_t bytes_written;
  unsigned long startTime = millis();
  while (millis() - startTime < duration_ms) {
    i2s_write(I2S_NUM_0, samples, numSamples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
  }
}

// Callback ESP-NOW untuk menerima data dari Slave
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  memcpy(&rxMessage, data, sizeof(rxMessage));
  Serial.print("Received Frequency: ");
  Serial.println(rxMessage.frequency);

  busy = true;
  
  if (rxMessage.frequency == 2000) {
    if (speakerEnabled) {  // Hanya putar suara jika speaker aktif
      generateTone(rxMessage.frequency, 1000);
      Serial.println("Playing 2kHz from Slave");
    } else {
      Serial.println("Speaker is disabled, not playing sound");
    }
  }
  
  busy = false;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  pinMode(MUTE_BUTTON, INPUT_PULLUP);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  i2sInit();
}

void loop() {
  bool buttonState = digitalRead(BOOT_BUTTON);
  bool muteState = digitalRead(MUTE_BUTTON);

  // Toggle status speaker jika tombol mute ditekan
  if (muteState == LOW) {
    delay(200);
    if (digitalRead(MUTE_BUTTON) == LOW) {
      speakerEnabled = !speakerEnabled;
      Serial.println(speakerEnabled ? "Speaker Enabled" : "Speaker Disabled");
      delay(300);
      while (digitalRead(MUTE_BUTTON) == LOW); // Tunggu sampai tombol dilepas
    }
  }

  // Kirim data hanya jika unit siap (readyToSend true), belum trigger, tidak sibuk, dan tombol ditekan
  if (readyToSend && !buttonTriggered && !busy && buttonState == LOW) {
    delay(50);  // Debounce
    if (digitalRead(BOOT_BUTTON) == LOW) {
      sendData.frequency = 1000;  // Master mengirim 1kHz ke Slave
      esp_now_send(peerAddress, (uint8_t*)&sendData, sizeof(sendData));
      Serial.println("Sent 1kHz to Slave");
      readyToSend = false;      // Nonaktifkan pengiriman lebih lanjut
      buttonTriggered = true;   // Tandai tombol sudah ditekan
    }
  }
  
  // Reset trigger tombol ketika dilepas (namun readyToSend tetap false)
  if (buttonState == HIGH) {
    buttonTriggered = false;
  }
}
