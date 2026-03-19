/*
 * ===================================================================================
 * CROSSOVER DIGITAL 3 VÍAS (LR2) MULTI-NÚCLEO
 * VERSIÓN CON HMI NEXTION Y CONTROL DUAL DE VOLUMEN
 *
 * CARACTERÍSTICAS (v4.1):
 * - LED de Volumen Alto (Pin 4): Se activa si el volumen supera 85%
 * - Control de volumen en pasos de 10% (Nextion y Comandos Serial)
 * - Potenciómetro con histéresis del 20% para mayor estabilidad
 * ===================================================================================
 */

#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <driver/i2s.h>
#include "freertos/ringbuf.h" 

#define LED_BUILTIN 2

// ============================================
// --- VARIABLES PARA SONIDO DE CONFIRMACIÓN ---
// ============================================
volatile bool reproducir_beep_conexion = false;
int beep_samples_remaining = 0;
float beep_phase = 0.0f;
const int BEEP_DURACION_SAMPLES = 8820; // 200ms a 44100Hz

// ============================================
// --- PINES I2S (PCM5102) ---
// ============================================
#define I2S_BCLK    27
#define I2S_LRC     32
#define I2S_DOUT    25
#define I2S_NUM     I2S_NUM_0

// ============================================
// --- PINES Y CONFIGURACIÓN NEXTION ---
// ============================================
#define NEXTION_SERIAL Serial2
#define NEXTION_RX 16
#define NEXTION_TX 17
#define NEXTION_BAUD 9600

// ============================================
// --- POTENCIÓMETRO ---
// ============================================
#define POT_PIN 34 // Pin para el potenciómetro de volumen
int lastPotValue = -100; // Valor inicial para forzar la primera lectura
unsigned long lastPotRead = 0;

// ============================================
// --- NUEVO PIN: LED INDICADOR DE VOLUMEN ALTO ---
// ============================================
#define PIN_LED_VOLUMEN_ALTO 4   // LED que se activa con volumen alto (>85%)

// ============================================
// MODOS DE DIAGNÓSTICO
// ============================================
enum ModoTest {
  MODO_NORMAL, MODO_SOLO_BAJOS, MODO_SOLO_MEDIOS, MODO_SOLO_ALTOS,
  MODO_SIN_BAJOS, MODO_SIN_MEDIOS, MODO_SIN_ALTOS
};
ModoTest modoActual = MODO_NORMAL;

// ============================================
// ESTRUCTURA DE FILTROS BIQUAD
// ============================================
typedef struct {
  float b0, b1, b2, a1, a2, x1, x2, y1, y2, dc_offset;
} BiquadFilter;

BiquadFilter filtroBajosLP1_L, filtroBajosLP1_R;
BiquadFilter filtroMediosHP1_L, filtroMediosLP1_L;
BiquadFilter filtroMediosHP1_R, filtroMediosLP1_R;
BiquadFilter filtroAltosHP1_L, filtroAltosHP1_R;
BiquadFilter filtroDCBlock1_L, filtroDCBlock1_R;

// ============================================
// PARÁMETROS DEL CROSSOVER Y VOLUMEN
// ============================================
float fcBajos = 300.0;
float fcAltos = 3000.0;
float fs_global = 44100.0;
float gananciaBajos = 0.85;
float gananciaMedios = 0.95;
float gananciaAltos = 0.95;
volatile float volumenMaster = 0.6; 

// ============================================
// BLUETOOTH Y ESTADO GLOBAL
// ============================================
BluetoothA2DPSink a2dp_sink;
bool bluetoothConectado = false;
String cancionActual = "Esperando...";
bool isPlaying = false;
unsigned long ultimoParpadeo = 0;

// ============================================
// --- VARIABLES DE ESTADO NEXTION ---
// ============================================
unsigned long tiempoInicioCancion = 0;
unsigned long ultimoUpdateNextion = 0;
unsigned long ultimoUpdateWaveform = 0;
const int WAVEFORM_UPDATE_INTERVAL = 100; // ms
const int NEXTION_UPDATE_INTERVAL = 1000; // ms
volatile float waveform_accumulator = 0;
volatile int waveform_samples_count = 0;

// ============================================
// BUFFERS Y TAREAS
// ============================================
RingbufHandle_t ringbuffer;   
#define MAX_CHUNK_SAMPLES 512
static int16_t out_buffer[MAX_CHUNK_SAMPLES * 2];
static uint32_t dither_seed = 12345;
TaskHandle_t xTaskDSP;      

// ============================================
// --- FUNCIONES DE COMUNICACIÓN NEXTION ---
// ============================================
void nextion_send_command(const char* cmd) {
  NEXTION_SERIAL.print(cmd);
  NEXTION_SERIAL.write(0xFF);
  NEXTION_SERIAL.write(0xFF);
  NEXTION_SERIAL.write(0xFF);
}

void nextion_update_text(const char* obj_name, const char* text) {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%s.txt=\"%s\"", obj_name, text);
  nextion_send_command(buffer);
}

void nextion_update_number(const char* obj_name, int value) {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%s.val=%d", obj_name, value);
  nextion_send_command(buffer);
}

// ============================================
// --- PROCESADOR DE MENSAJES NEXTION ---
// ============================================
void nextion_process_message() {
  
  if (NEXTION_SERIAL.available() >= 7) { 
    
    if (NEXTION_SERIAL.read() == 0x65) {
      
      uint8_t pageId = NEXTION_SERIAL.read();
      uint8_t buttonId = NEXTION_SERIAL.read();
      uint8_t eventType = NEXTION_SERIAL.read();
      
      uint8_t end1 = NEXTION_SERIAL.read();
      uint8_t end2 = NEXTION_SERIAL.read();
      uint8_t end3 = NEXTION_SERIAL.read();

      if (end1 == 0xFF && end2 == 0xFF && end3 == 0xFF && eventType == 0x00) {
        
        Serial.printf("HMI BIN -> Evento: Release (0x00), ID: %d\n", buttonId);

        switch(buttonId) {
            case 2: // b0 (Anterior)
                Serial.println("HMI -> Comando: CANCIÓN ANTERIOR");
                a2dp_sink.previous();
                break;

            case 3: // bt0 (Play/Pause)
                if (isPlaying) {
                    Serial.println("HMI -> Comando: PAUSA");
                    a2dp_sink.pause();
                } else {
                    Serial.println("HMI -> Comando: PLAY");
                    a2dp_sink.play();
                }
                break;

            case 4: // b1 (Siguiente)
                Serial.println("HMI -> Comando: SIGUIENTE CANCIÓN");
                a2dp_sink.next();
                break;

            case 9: // b2 (Volumen -)
                volumenMaster = constrain(volumenMaster - 0.10f, 0.0f, 0.95f);
                Serial.printf("HMI -> Volumen: %.0f%%\n", volumenMaster * 100);
                break;

            case 10: // b3 (Volumen +)
                volumenMaster = constrain(volumenMaster + 0.10f, 0.0f, 0.95f);
                Serial.printf("HMI -> Volumen: %.0f%%\n", volumenMaster * 100);
                break;

            default:
                Serial.printf("HMI BIN -> ID no reconocido: %d\n", buttonId);
                break;
        }
      } else if (eventType == 0x01) {
          // Ignorando el evento "Press"
      } else {
          while(NEXTION_SERIAL.available() > 0) NEXTION_SERIAL.read();
      }
    } else {
      while(NEXTION_SERIAL.available() > 0) NEXTION_SERIAL.read();
    }
  }
}

// ============================================
// --- FUNCIÓN LECTURA POTENCIÓMETRO ---
// ============================================
void checkPotentiometer() {
    // Leer el potenciómetro solo cada 100ms para evitar ruido
    if (millis() - lastPotRead > 100) { 
        lastPotRead = millis();
        int potValue = analogRead(POT_PIN);
        
        // Histéresis aumentada: Solo actualizar si el cambio es mayor a ~20% del rango
        // 4095 * 0.20 ≈ 819, usamos 820 para un umbral del 20%
        if (abs(potValue - lastPotValue) > 820) { 
            lastPotValue = potValue;
            
            // Mapear 0-4095 a 0.0-0.95
            float newVolume = (float)potValue / 4095.0f * 0.95f; 
            volumenMaster = constrain(newVolume, 0.0f, 0.95f);
            
            Serial.printf("POT (Pin 34) -> Valor: %d -> Volumen: %.0f%%\n", potValue, volumenMaster * 100);
        }
    }
}

// ============================================
// --- FUNCIÓN: CONTROL DE LED DE VOLUMEN ALTO ---
// ============================================
void control_led_volumen() {
    // Si el volumen supera el 85%, encender LED de advertencia
    if (volumenMaster > 0.85f) {
        digitalWrite(PIN_LED_VOLUMEN_ALTO, HIGH);
    } else {
        digitalWrite(PIN_LED_VOLUMEN_ALTO, LOW);
    }
}

// ============================================
// FUNCIONES DE FILTROS
// ============================================
void initBiquad(BiquadFilter *f) { f->b0 = f->b1 = f->b2 = 0.0f; f->a1 = f->a2 = 0.0f; f->x1 = f->x2 = 0.0f; f->y1 = f->y2 = 0.0f; f->dc_offset = 0.0f; }
void calcHighPassLR(BiquadFilter *f, float fc, float fs) { float w0 = 2.0 * PI * fc / fs, c = cosf(w0), s = sinf(w0), Q = 0.7071f, a = s / (2.0f * Q), a0 = 1.0f + a; f->b0=(1+c)/2.0f/a0; f->b1=-(1+c)/a0; f->b2=(1+c)/2.0f/a0; f->a1=-2*c/a0; f->a2=(1-a)/a0; }
void calcLowPassLR(BiquadFilter *f, float fc, float fs) { float w0 = 2.0 * PI * fc / fs, c = cosf(w0), s = sinf(w0), Q = 0.7071f, a = s / (2.0f * Q), a0 = 1.0f + a; f->b0=(1-c)/2.0f/a0; f->b1=(1-c)/a0; f->b2=(1-c)/2.0f/a0; f->a1=-2*c/a0; f->a2=(1-a)/a0; }
void calcDCBlocker(BiquadFilter *f, float fc, float fs) { float w0 = 2.0 * PI * fc / fs, c = cosf(w0), s = sinf(w0), Q = 0.5f, a = s / (2.0f * Q), a0 = 1.0f + a; f->b0 = (1+c)/2.0f/a0; f->b1 = -(1+c)/a0; f->b2 = (1+c)/2.0f/a0; f->a1 = -2*c/a0; f->a2 = (1-a)/a0; }

float processBiquad(BiquadFilter *f, float input) {
  const float denormal = 1.0e-30f;
  float output = f->b0 * input + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
  f->x2 = f->x1; f->x1 = input; f->y2 = f->y1; f->y1 = output;
  if (fabsf(f->x1) < denormal) f->x1 = 0.0f; if (fabsf(f->x2) < denormal) f->x2 = 0.0f;
  if (fabsf(f->y1) < denormal) f->y1 = 0.0f; if (fabsf(f->y2) < denormal) f->y2 = 0.0f;
  if (fabsf(output) < denormal) output = 0.0f;
  return output;
}

float softClip(float x) { const float t = 0.7f; if (x > 0.98f) return 0.98f; if (x < -0.98f) return -0.98f; if (fabsf(x) > t) { float s = (x > 0) ? 1.0f : -1.0f; return s * (t + (1.0f - t) * tanhf((fabsf(x) - t) * 3.0f)); } return x; }
float generateDither() { dither_seed=dither_seed*1664525+1013904223; float r1=(float)(dither_seed&0xFFFF)/65536.0f-0.5f; dither_seed=dither_seed*1664525+1013904223; float r2=(float)(dither_seed&0xFFFF)/65536.0f-0.5f; return(r1+r2)*0.5f/32768.0f; }

// ============================================
// RECALCULAR FILTROS
// ============================================
void recalcularFiltros() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║       RECALCULANDO FILTROS LR2         ║"); 
  Serial.println("╠════════════════════════════════════════╣");
  initBiquad(&filtroDCBlock1_L); initBiquad(&filtroDCBlock1_R);
  calcDCBlocker(&filtroDCBlock1_L, 2.0f, fs_global); calcDCBlocker(&filtroDCBlock1_R, 2.0f, fs_global);
  initBiquad(&filtroBajosLP1_L); initBiquad(&filtroBajosLP1_R);
  calcLowPassLR(&filtroBajosLP1_L, fcBajos, fs_global); calcLowPassLR(&filtroBajosLP1_R, fcBajos, fs_global);
  initBiquad(&filtroMediosHP1_L); initBiquad(&filtroMediosLP1_L);
  initBiquad(&filtroMediosHP1_R); initBiquad(&filtroMediosLP1_R);
  calcHighPassLR(&filtroMediosHP1_L, fcBajos, fs_global); calcLowPassLR(&filtroMediosLP1_L, fcAltos, fs_global);
  calcHighPassLR(&filtroMediosHP1_R, fcBajos, fs_global); calcLowPassLR(&filtroMediosLP1_R, fcAltos, fs_global);
  initBiquad(&filtroAltosHP1_L); initBiquad(&filtroAltosHP1_R);
  calcHighPassLR(&filtroAltosHP1_L, fcAltos, fs_global); calcHighPassLR(&filtroAltosHP1_R, fcAltos, fs_global);
  Serial.printf("║ ✓ Bajos:  LR2 LP @ %.0f Hz (g:%.2f)  ║\n", fcBajos, gananciaBajos); 
  Serial.printf("║ ✓ Medios: LR2 BP @ %.0f-%.0f Hz     ║\n", fcBajos, fcAltos);
  Serial.printf("║ ✓ Altos:  LR2 HP @ %.0f Hz (g:%.2f) ║\n", fcAltos, gananciaAltos);
  Serial.println("╚════════════════════════════════════════╝\n");
}

void inicializarFiltros(float fs) {
  fs_global = fs;
  recalcularFiltros();
}

// ============================================
// CALLBACK DE AUDIO
// ============================================
void audio_data_callback(const uint8_t *data, uint32_t len) {
  xRingbufferSend(ringbuffer, data, len, pdMS_TO_TICKS(10));
}

// ============================================
// TAREA de PROCESAMIENTO DSP
// ============================================
void i2s_processing_task(void *parameter) {
  size_t item_size;
  while (true) {
    uint8_t *data = (uint8_t*)xRingbufferReceive(ringbuffer, &item_size, pdMS_TO_TICKS(100));
    
    // Si hay que reproducir el beep y no hay datos de audio
    if (data == NULL && reproducir_beep_conexion && beep_samples_remaining > 0) {
      int samples_to_generate = min(beep_samples_remaining, MAX_CHUNK_SAMPLES);
      
      for (int i = 0; i < samples_to_generate; i++) {
        // Generar tono de 1000Hz
        float sample = 0.25f * sinf(beep_phase);
        
        // Envelope simple
        float envelope = 1.0f;
        int total_beep = BEEP_DURACION_SAMPLES;
        int current_pos = total_beep - beep_samples_remaining + i;
        
        if (current_pos < 1000) {
          envelope = (float)current_pos / 1000.0f;
        } else if (current_pos > total_beep - 1000) {
          envelope = (float)(total_beep - current_pos) / 1000.0f;
        }
        
        sample *= envelope;
        int16_t sample_int = (int16_t)(sample * 32767.0f);
        
        out_buffer[i * 2] = sample_int;
        out_buffer[i * 2 + 1] = sample_int;
        
        beep_phase += 2.0f * PI * 1000.0f / 44100.0f;
        if (beep_phase > 2.0f * PI) beep_phase -= 2.0f * PI;
      }
      
      size_t bytes_written;
      i2s_write(I2S_NUM, out_buffer, samples_to_generate * 4, &bytes_written, portMAX_DELAY);
      beep_samples_remaining -= samples_to_generate;
      
      if (beep_samples_remaining <= 0) {
        reproducir_beep_conexion = false;
        Serial.println("🔊 Beep de conexión reproducido");
      }
      continue;
    }
    
    if (data == NULL) continue;
    
    int16_t *samples_in = (int16_t*)data;
    int totalSamplesToProcess = item_size / 4; 
    int samplesProcessed = 0;

    while (samplesProcessed < totalSamplesToProcess) {
      int numSamplesInChunk = totalSamplesToProcess - samplesProcessed;
      if (numSamplesInChunk > MAX_CHUNK_SAMPLES) numSamplesInChunk = MAX_CHUNK_SAMPLES;
      
      for (int i = 0; i < numSamplesInChunk; i++) {
        int inputIndex = samplesProcessed + i;
        float inputL = (float)samples_in[inputIndex * 2] * (1.0f / 32768.0f);
        float inputR = (float)samples_in[inputIndex * 2 + 1] * (1.0f / 32768.0f);

        waveform_accumulator += fabsf(inputL);
        waveform_samples_count++;
        
        inputL = processBiquad(&filtroDCBlock1_L, inputL);
        inputR = processBiquad(&filtroDCBlock1_R, inputR);
        
        float bajosL = processBiquad(&filtroBajosLP1_L, inputL) * gananciaBajos;
        float mediosL = processBiquad(&filtroMediosLP1_L, processBiquad(&filtroMediosHP1_L, inputL)) * gananciaMedios;
        float altosL = processBiquad(&filtroAltosHP1_L, inputL) * gananciaAltos;
        
        float bajosR = processBiquad(&filtroBajosLP1_R, inputR) * gananciaBajos;
        float mediosR = processBiquad(&filtroMediosLP1_R, processBiquad(&filtroMediosHP1_R, inputR)) * gananciaMedios;
        float altosR = processBiquad(&filtroAltosHP1_R, inputR) * gananciaAltos;
        
        float outputL = 0.0f, outputR = 0.0f;
        switch(modoActual) {
          case MODO_NORMAL:
            outputL = bajosL + mediosL + altosL;
            outputR = bajosR + mediosR + altosR;
            break;
          default:
            outputL = bajosL + mediosL + altosL;
            outputR = bajosR + mediosR + altosR;
            break;
        }
        
        outputL *= volumenMaster;
        outputR *= volumenMaster;
        outputL = softClip(outputL);
        outputR = softClip(outputR);
        
        int32_t tempL = (int32_t)((outputL + generateDither()) * 32767.0f);
        int32_t tempR = (int32_t)((outputR + generateDither()) * 32767.0f);
        
        if (tempL > 32767) tempL = 32767; if (tempL < -32768) tempL = -32768;
        if (tempR > 32767) tempR = 32767; if (tempR < -32768) tempR = -32768;
        
        out_buffer[i * 2] = (int16_t)tempL;
        out_buffer[i * 2 + 1] = (int16_t)tempR;
      }
      
      size_t bytes_written;
      i2s_write(I2S_NUM, out_buffer, numSamplesInChunk * 4, &bytes_written, portMAX_DELAY);
      samplesProcessed += numSamplesInChunk;
    } 
    vRingbufferReturnItem(ringbuffer, (void*)data);
  }
}

// ============================================
// CALLBACKS BLUETOOTH
// ============================================
void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
  if (id == ESP_AVRC_MD_ATTR_TITLE) {
    cancionActual = String((char*)text);
    Serial.printf("♪ %s\n", cancionActual.c_str());
    nextion_update_text("g0", cancionActual.c_str());
    tiempoInicioCancion = millis();
  }
}

void audio_state_callback(esp_a2d_audio_state_t state, void *context){
    if (state == ESP_A2D_AUDIO_STATE_STARTED) {
        isPlaying = true;
        if (tiempoInicioCancion == 0) tiempoInicioCancion = millis();
    } else if (state == ESP_A2D_AUDIO_STATE_STOPPED || state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND) {
        isPlaying = false;
    }
}

void connection_state_callback(esp_a2d_connection_state_t state, void* context) {
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    Serial.println("\n🔵 BLUETOOTH CONECTADO");
    int sampleRate = a2dp_sink.sample_rate();
    if (sampleRate == 0) sampleRate = 44100;
    Serial.printf("   Sample rate: %d Hz\n", sampleRate);
    i2s_set_sample_rates(I2S_NUM, sampleRate);
    if ((float)sampleRate != fs_global) inicializarFiltros((float)sampleRate);
    bluetoothConectado = true;
    digitalWrite(LED_BUILTIN, HIGH);
    
    vTaskResume(xTaskDSP);
    
    // *** ACTIVAR BEEP DE CONEXIÓN (se reproduce en la tarea DSP) ***
    delay(500); // Pequeña pausa para que se estabilice el audio
    beep_samples_remaining = BEEP_DURACION_SAMPLES;
    beep_phase = 0.0f;
    reproducir_beep_conexion = true;
    Serial.println("🔊 Activando sonido de conexión...");
  } else {
    Serial.println("\n🔴 BLUETOOTH DESCONECTADO\n");
    bluetoothConectado = false;
    digitalWrite(LED_BUILTIN, LOW);
    reproducir_beep_conexion = false;
    beep_samples_remaining = 0;
    vTaskSuspend(xTaskDSP); 
    i2s_zero_dma_buffer(I2S_NUM);
    isPlaying = false;
    nextion_update_text("g0", "Desconectado");
    nextion_update_text("t0", "00:00");
  }
}

// ============================================
// COMANDOS SERIAL
// ============================================
void procesarComandos() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); cmd.toLowerCase();
  
  if (cmd == "normal") { modoActual = MODO_NORMAL; Serial.println("🎵 MODO: NORMAL"); }
  else if (cmd == "bajos") { modoActual = MODO_SOLO_BAJOS; Serial.println("🎵 MODO: SOLO BAJOS"); }
  else if (cmd == "v+") {
    volumenMaster = constrain(volumenMaster + 0.10f, 0.0f, 0.95f);
    Serial.printf("🔊 Volumen: %.0f%%\n", volumenMaster * 100);
  }
  else if (cmd == "v-") {
    volumenMaster = constrain(volumenMaster - 0.10f, 0.0f, 0.95f);
    Serial.printf("🔉 Volumen: %.0f%%\n", volumenMaster * 100);
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║ CROSSOVER 3 VÍAS + LED DE VOLUMEN      ║");
  Serial.println("║   ARQUITECTURA OPTIMIZADA (LR2-CORE1)  ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // --- CONFIGURACIÓN DE LED DE VOLUMEN ALTO ---
  pinMode(PIN_LED_VOLUMEN_ALTO, OUTPUT);
  digitalWrite(PIN_LED_VOLUMEN_ALTO, LOW);
  
  Serial.printf("✓ LED Volumen Alto (>85%%): GPIO%d\n", PIN_LED_VOLUMEN_ALTO);

  pinMode(POT_PIN, INPUT);
  Serial.printf("✓ Potenciómetro (Volumen): GPIO%d\n", POT_PIN);

  NEXTION_SERIAL.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  delay(100);
  nextion_send_command("rest");
  delay(500);
  nextion_update_text("g0", "Iniciando...");
  Serial.printf("✓ Nextion HMI en: RX(%d), TX(%d)\n", NEXTION_RX, NEXTION_TX);
  
  Serial.println("🔧 Configurando I2S a 16-BIT...");
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8, .dma_buf_len = 512, .use_apll = true,
    .tx_desc_auto_clear = true, .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK, .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT, .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_set_sample_rates(I2S_NUM, 44100);
  i2s_zero_dma_buffer(I2S_NUM);
  Serial.println("   ✓ I2S configurado (MODO 16-BIT)");
  
  inicializarFiltros(44100.0);

  ringbuffer = xRingbufferCreate(8192, RINGBUF_TYPE_BYTEBUF);
  if (ringbuffer == NULL) Serial.println("❌ Error al crear RingBuffer!");

  Serial.println("🚀 Creando tarea de DSP en Core 1...");
  xTaskCreatePinnedToCore(
      i2s_processing_task, "DSP_Task", 4096, NULL, 5, &xTaskDSP, 1
  );
  vTaskSuspend(xTaskDSP); 
  
  Serial.println("🔵 Iniciando Bluetooth A2DP...");
  a2dp_sink.set_stream_reader(audio_data_callback, false); 
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_on_connection_state_changed(connection_state_callback);
  a2dp_sink.set_on_audio_state_changed(audio_state_callback); 
  a2dp_sink.start("NORBERTICO");
  
  Serial.println("   ✓ Bluetooth iniciado\n");
  Serial.println("📱 Dispositivo: NORBERTICO");
  Serial.println("\n💡 ARQUITECTURA MULTI-NÚCLEO ACTIVADA");
  Serial.println("   ✓ BT/HMI (Core 0) -> RingBuffer -> DSP (Core 1) -> I2S");
  Serial.println("\n💡 Escribe 'help' para comandos\n");
}

// ============================================
// LOOP
// ============================================
void loop() {
  // 1. Procesar comandos seriales
  procesarComandos();
  
  // 2. Procesar mensajes de la Nextion
  nextion_process_message();

  // 3. Procesar el potenciómetro
  checkPotentiometer(); 

  // 4. LED parpadeando si está desconectado
  if (!bluetoothConectado) {
    if (millis() - ultimoParpadeo > 500) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      ultimoParpadeo = millis();
    }
  }
  
  unsigned long currentTime = millis();

  // 5. Actualizar Timer y Waveform en Nextion
  if (bluetoothConectado && isPlaying) {
      
      // Actualizar el timer cada segundo
      if (currentTime - ultimoUpdateNextion >= NEXTION_UPDATE_INTERVAL) {
          ultimoUpdateNextion = currentTime;
          unsigned long elapsedSeconds = (currentTime - tiempoInicioCancion) / 1000;
          char timeStr[6];
          snprintf(timeStr, sizeof(timeStr), "%02d:%02d", (int)(elapsedSeconds / 60), (int)(elapsedSeconds % 60));
          nextion_update_text("t0", timeStr);
      }

      // Actualizar la forma de onda (ID 6)
      if (currentTime - ultimoUpdateWaveform >= WAVEFORM_UPDATE_INTERVAL) {
          ultimoUpdateWaveform = currentTime;
          
          float current_accumulator = waveform_accumulator;
          int current_samples_count = waveform_samples_count;
          waveform_accumulator = 0;
          waveform_samples_count = 0;

          if (current_samples_count > 0) {
              float avg_level = (current_accumulator / current_samples_count);
              int waveform_value = constrain((int)(avg_level * 500.0f), 0, 255); 
              char cmd[32];
              snprintf(cmd, sizeof(cmd), "add 6,0,%d", waveform_value); 
              nextion_send_command(cmd);
          }
      }
  }
  
  // 6. *** NUEVO: Controlar LED de volumen alto ***
  control_led_volumen();
  
  // 7. Reporte de estado periódico (con info de LED)
  static unsigned long ultimoReporte = 0;
  if (bluetoothConectado && (millis() - ultimoReporte > 30000)) {
    Serial.printf("📊 Audio OK | Vol: %.0f%% | B:%.2fx M:%.2fx A:%.2fx | LED_Vol: %s\n",
                  volumenMaster * 100, gananciaBajos, gananciaMedios, gananciaAltos,
                  digitalRead(PIN_LED_VOLUMEN_ALTO) ? "ON" : "OFF");
    ultimoReporte = millis();
  }
  
  delay(10); // Pausa general del loop (Core 0)
}