#include <driver/i2s.h>
#include <math.h>

#define AMP_BCLK  26     // Bit Clock untuk amplifier
#define AMP_LRCLK 25     // Word Select untuk amplifier
#define AMP_DATA  22     // Data Out untuk amplifier

// Gunakan I2S_NUM_0 untuk output sine wave
#define I2S_AMP_PORT I2S_NUM_0

#define SAMPLE_RATE       16000
#define SINE_FREQ         1500.0    // Sine wave 1000 Hz
#define SINE_BUFFER_SIZE  256       // Jumlah frame stereo (setiap frame = 2 sampel)

int16_t sineBuffer[SINE_BUFFER_SIZE * 2]; // *2 untuk channel kiri dan kanan
float phase = 0.0;
float phaseIncrement = (2.0 * M_PI * SINE_FREQ) / SAMPLE_RATE;

void setup() {
  Serial.begin(115200);
  Serial.println("Sine Wave Playback at 1000 Hz (Stereo)");

  // Konfigurasi I2S untuk MAX98357A dengan output stereo
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // Output stereo
    .communication_format = I2S_COMM_FORMAT_I2S,     // Coba juga I2S_COMM_FORMAT_I2S_MSB jika perlu
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = SINE_BUFFER_SIZE,  // Dalam satuan frame (satu frame = 2 sampel)
    .use_apll = false
  };

  i2s_driver_install(I2S_AMP_PORT, &i2s_config, 0, NULL);

  i2s_pin_config_t pin_config = {
    .bck_io_num = AMP_BCLK,
    .ws_io_num  = AMP_LRCLK,
    .data_out_num = AMP_DATA,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };
  i2s_set_pin(I2S_AMP_PORT, &pin_config);
}

void loop() {
  // Isi buffer dengan data sine wave secara stereo
  for (int i = 0; i < SINE_BUFFER_SIZE; i++) {
    float sample = sin(phase);
    int16_t sample_val = (int16_t)(sample * 32767);  // Skala ke 16-bit
    // Duplikasi sampel ke channel kiri dan kanan
    sineBuffer[2 * i]     = sample_val;  // channel kiri
    sineBuffer[2 * i + 1] = sample_val;  // channel kanan

    phase += phaseIncrement;
    if (phase >= 2.0 * M_PI) {
      phase -= 2.0 * M_PI;
    }
  }
  
  size_t bytesWritten = 0;
  // Kirim data ke amplifier; ukuran data = jumlah frame * 2 (channel) * 2 (byte per sampel)
  i2s_write(I2S_AMP_PORT, (const char*)sineBuffer, SINE_BUFFER_SIZE * 2 * sizeof(int16_t), &bytesWritten, portMAX_DELAY);
}
