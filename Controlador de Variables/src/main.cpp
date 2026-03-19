// Control de Temperatura e Iluminación con HMI Nextion
// ESP32 + Nextion Display - Componentes: v0, x0, n1, n2, b210
// --- AÑADIDO POR BLUETOOTH ---
// Incluimos la librería para Bluetooth Serial
#include <BluetoothSerial.h>
// --- FIN AÑADIDO ---

#include <Arduino.h>

// --- Definiciones de Hardware ---
#define PIN_SENSOR_TEMP 34
#define PIN_CALENTADOR 26
#define PIN_VENTILADOR 25
#define PIN_SENSOR_LDR 4
#define PIN_LEDS_POTENCIA 27

// --- UART para Nextion (Serial2 en ESP32) ---
#define NEXTION_SERIAL Serial2
#define NEXTION_RX 16  // GPIO16 como RX
#define NEXTION_TX 17  // GPIO17 como TX
#define NEXTION_BAUD 9600

// --- Parámetros PWM ---
#define LEDC_CANAL_VENT 0
#define LEDC_FRECUENCIA 5000
#define LEDC_RESOLUCION 10
#define LEDC_CANAL_LEDS 1
#define LEDC_FRECUENCIA_L 1000
#define LEDC_RESOLUCION_L 10
#define VALOR_MAX_PWM ((1 << LEDC_RESOLUCION) - 1)

// --- Parámetros de Calibración ---
#define SAMPLE_CNT 128
#define FILTER_SIZE 10
#define RANGO_HISTERESIS 1.0
const float CALIBRATION_OFFSET = 0.0;

// --- AÑADIDO POR BLUETOOTH ---
// Objeto para Bluetooth Serial
BluetoothSerial SerialBT;
// Variable de estado para Encender/Apagar desde la App
bool sistema_activo = true; 
// --- FIN AÑADIDO ---
void set_potencia_ventilador(int porcentaje);
void set_potencia_leds(int porcentaje);
// --- Variables Globales ---
float T_CONTROL = 25.0;  // Modificable desde Nextion (v0) y Bluetooth
float temperatura_actual = 0.0;
int iluminacion_ldr_raw = 0;
float iluminacion_percent = 0.0;
int leds_potencia = 0;
int ventilador_potencia = 0;

// Filtro de media móvil
static float temp_buffer[FILTER_SIZE] = {0};
static int buffer_index = 0;
static bool buffer_filled = false;

// Control de actualización Nextion
unsigned long last_nextion_update = 0;
const unsigned long NEXTION_UPDATE_INTERVAL = 1000;

// --- Funciones Nextion ---
// (Aquí van todas tus funciones Nextion sin cambios)
// ...
void nextion_send_command(const char* cmd) {
    NEXTION_SERIAL.print(cmd);
    NEXTION_SERIAL.write(0xFF);
    NEXTION_SERIAL.write(0xFF);
    NEXTION_SERIAL.write(0xFF);
    NEXTION_SERIAL.flush();
}
void nextion_update_number(const char* obj_name, int value) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%s.val=%d", obj_name, value);
    nextion_send_command(buffer);
}
void nextion_update_temp_control(float temp) {
    nextion_update_number("v0", (int)temp);
}
void nextion_update_temp_medida(float temp) {
    int parte_entera = (int)temp;
    float decimal = temp - parte_entera;
    int parte_decimal = (int)(decimal * 100 + 0.5);
    if (parte_decimal >= 100) {
        parte_entera++;
        parte_decimal = 0;
    }
    if (parte_decimal < 0) {
        parte_decimal = 0;
    }
    nextion_update_number("x0", parte_entera);
    nextion_update_number("n1", parte_decimal);
}
void nextion_update_iluminacion(float percent) {
    int porcentaje_entero = (int)(percent + 0.5);
    porcentaje_entero = constrain(porcentaje_entero, 0, 100);
    nextion_update_number("n2", porcentaje_entero);
}
void nextion_update_text(const char* obj_name, const char* text) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%s.txt=\"%s\"", obj_name, text);
    nextion_send_command(buffer);
}
void nextion_update_temp_texto() {
    char temp_str[16];
    int parte_entera = (int)temperatura_actual;
    float decimal = temperatura_actual - parte_entera;
    int parte_decimal = (int)(decimal * 100 + 0.5);
    if (parte_decimal >= 100) {
        parte_entera++;
        parte_decimal = 0;
    }
    snprintf(temp_str, sizeof(temp_str), "%d.%02d", parte_entera, parte_decimal);
    nextion_update_text("temp", temp_str);
}
bool nextion_read_number(int32_t* result) {
    uint8_t temp[4] = {0};
    if (NEXTION_SERIAL.available() < 4) {
        return false;
    }
    temp[0] = NEXTION_SERIAL.read();
    temp[1] = NEXTION_SERIAL.read();
    temp[2] = NEXTION_SERIAL.read();
    temp[3] = NEXTION_SERIAL.read();
    *result = temp[0] + (temp[1] << 8) + (temp[2] << 16) + (temp[3] << 24);
    return true;
}
void nextion_process_message() {
    while (NEXTION_SERIAL.available() > 0) {
        uint8_t byte = NEXTION_SERIAL.read();
        if (byte == 0x71) {
            delay(10);
            if (NEXTION_SERIAL.available() >= 7) {
                int32_t value;
                if (nextion_read_number(&value)) {
                    uint8_t end1 = NEXTION_SERIAL.read();
                    uint8_t end2 = NEXTION_SERIAL.read();
                    uint8_t end3 = NEXTION_SERIAL.read();
                    if (end1 == 0xFF && end2 == 0xFF && end3 == 0xFF) {
                        if (value >= 15 && value <= 40) {
                            T_CONTROL = (float)value;
                            Serial.println();
                            Serial.print(" ✓  Nueva Temp. Control desde Nextion: ");
                            Serial.print(T_CONTROL, 1);
                            Serial.println("°C");
                            nextion_update_temp_control(T_CONTROL);
                        } else {
                            Serial.println();
                            Serial.print(" ✗  Temperatura fuera de rango (15-40 ° C): ");
                            Serial.println(value);
                        }
                    }
                }
            }
        }
        else if (byte == 0x65) {
            delay(10);
            while (NEXTION_SERIAL.available() > 0) {
                NEXTION_SERIAL.read();
            }
        }
        else if (byte >= 0x00 && byte <= 0x88) {
            uint8_t ff_count = 0;
            while (NEXTION_SERIAL.available() > 0 && ff_count < 3) {
                uint8_t b = NEXTION_SERIAL.read();
                if (b == 0xFF) ff_count++;
                else ff_count = 0;
            }
        }
    }
}
// ...
// (Fin de tus funciones Nextion)

// --- AÑADIDO POR BLUETOOTH ---
/**
 * @brief Procesa los comandos recibidos por Bluetooth Serial (SerialBT).
 * Comandos esperados:
 * "A": Activa el sistema de control.
 * "B": Desactiva el sistema de control y apaga actuadores.
 * "Txx.x": Establece la temperatura de control (ej. "T27.5").
 */
void procesar_bluetooth() {
    if (SerialBT.available()) {
        char primer_char = SerialBT.read();

        // Comando "A" (ENCENDER)
        if (primer_char == 'A') {
            sistema_activo = true;
            Serial.println();
            Serial.println("✓ Sistema ACTIVADO por Bluetooth.");
            // Opcional: enviar confirmación a la app
            // SerialBT.println("Sistema ACTIVADO");
        }
        
        // Comando "B" (APAGAR)
        else if (primer_char == 'B') {
            sistema_activo = false;
            // Apagamos los actuadores inmediatamente
            digitalWrite(PIN_CALENTADOR, LOW);
            set_potencia_ventilador(0);
            set_potencia_leds(0);
            Serial.println();
            Serial.println("✗ Sistema DESACTIVADO por Bluetooth.");
            // Opcional: enviar confirmación a la app
            // SerialBT.println("Sistema DESACTIVADO");
        }
        
        // Comando "T" (Temperatura)
        else if (primer_char == 'T') {
            // Espera un momento para que lleguen el resto de los datos
            delay(20); 
            // Lee el resto del número como un String
            String temp_str = SerialBT.readString();
            
            float nueva_temp = temp_str.toFloat();

            // Validar rango (igual que en Nextion)
            if (nueva_temp >= 15.0 && nueva_temp <= 40.0) {
                T_CONTROL = nueva_temp;
                Serial.println();
                Serial.print(" ✓  Nueva Temp. Control desde Bluetooth: ");
                Serial.print(T_CONTROL, 1);
                Serial.println("°C");

                // CRÍTICO: Actualizar Nextion para que esté sincronizado
                nextion_update_temp_control(T_CONTROL);
            } else {
                Serial.println();
                Serial.print(" ✗  Temp. Bluetooth fuera de rango (15-40 ° C): ");
                Serial.println(nueva_temp);
            }
        }

        // Limpiar el buffer de BT por si llega basura
        while (SerialBT.available() > 0) {
            SerialBT.read();
        }
    }
}
// --- FIN AÑADIDO ---


// --- Funciones PWM y Control ---
void configurar_pwm() {
    ledcSetup(LEDC_CANAL_VENT, LEDC_FRECUENCIA, LEDC_RESOLUCION);
    ledcAttachPin(PIN_VENTILADOR, LEDC_CANAL_VENT);
    ledcSetup(LEDC_CANAL_LEDS, LEDC_FRECUENCIA_L, LEDC_RESOLUCION_L);
    ledcAttachPin(PIN_LEDS_POTENCIA, LEDC_CANAL_LEDS);
}
void set_potencia_ventilador(int porcentaje) {
    ventilador_potencia = constrain(porcentaje, 0, 100);
    int duty_cycle = map(ventilador_potencia, 0, 100, 0, VALOR_MAX_PWM);
    ledcWrite(LEDC_CANAL_VENT, duty_cycle);
}
void set_potencia_leds(int porcentaje) {
    leds_potencia = constrain(porcentaje, 0, 100);
    int duty_cycle = map(leds_potencia, 0, 100, 0, VALOR_MAX_PWM);
    ledcWrite(LEDC_CANAL_LEDS, duty_cycle);
}

// --- Lectura de Sensores ---
float get_filtered_temperature(float temp) {
    temp_buffer[buffer_index] = temp;
    buffer_index++;
    if (buffer_index >= FILTER_SIZE) {
        buffer_index = 0;
        buffer_filled = true;
    }
    int count = buffer_filled ? FILTER_SIZE : buffer_index;
    if (count == 0) return temp;
    float sum = 0;
    for (int i = 0; i < count; i++) {
        sum += temp_buffer[i];
    }
    return sum / count;
}
float leer_temperatura() {
    long sum_mv = 0;
    for (int i = 0; i < SAMPLE_CNT; i++) {
        sum_mv += analogReadMilliVolts(PIN_SENSOR_TEMP);
        delay(1);
    }
    long avg_mv = sum_mv / SAMPLE_CNT;
    float temp_raw = (avg_mv / 10.0) + CALIBRATION_OFFSET;
    if (temp_raw < 0) temp_raw = 0;
    if (temp_raw > 100) temp_raw = 100;
    float temp_filtered = get_filtered_temperature(temp_raw);
    return temp_filtered;
}
float leer_iluminacion() {
    iluminacion_ldr_raw = analogRead(PIN_SENSOR_LDR);
    const int LDR_MIN_ADC = 241;
    const int LDR_MAX_ADC = 4095;
    int lectura_limitada = constrain(iluminacion_ldr_raw, LDR_MIN_ADC, LDR_MAX_ADC);
    long mapeo_entero = map(lectura_limitada, LDR_MIN_ADC, LDR_MAX_ADC, 100, 0);
    iluminacion_percent = constrain((float)mapeo_entero, 0.0, 100.0);
    return iluminacion_percent;
}

// --- Lógica de Control ---
void control_temperatura() {
    float T = temperatura_actual;
    float Tc = T_CONTROL;
    float T_R1_MIN = Tc - RANGO_HISTERESIS;
    float T_R1_MAX = Tc + RANGO_HISTERESIS;
    float T_R2_MAX = Tc + 3.0;
    float T_R3_MAX = Tc + 5.0;
    if (T >= T_R1_MIN && T <= T_R1_MAX) {
        digitalWrite(PIN_CALENTADOR, LOW);
        set_potencia_ventilador(0);
    }
    else if (T < T_R1_MIN) {
        digitalWrite(PIN_CALENTADOR, HIGH);
        set_potencia_ventilador(0);
    }
    else if (T > T_R1_MAX && T < T_R2_MAX) {
        digitalWrite(PIN_CALENTADOR, LOW);
        set_potencia_ventilador(50);
    }
    else if (T >= T_R2_MAX && T <= T_R3_MAX) {
        digitalWrite(PIN_CALENTADOR, LOW);
        set_potencia_ventilador(80);
    }
    else if (T > T_R3_MAX) {
        digitalWrite(PIN_CALENTADOR, LOW);
        set_potencia_ventilador(100);
    }
}
void control_iluminacion() {
    float ni = iluminacion_percent;
    if (ni < 20.0) {
        set_potencia_leds(100);
    } else if (ni >= 20.0 && ni < 30.0) {
        set_potencia_leds(80);
    } else if (ni >= 30.0 && ni < 40.0) {
        set_potencia_leds(60);
    } else if (ni >= 40.0 && ni < 60.0) {
        set_potencia_leds(50);
    } else if (ni >= 60.0 && ni < 80.0) {
        set_potencia_leds(30);
    } else {
        set_potencia_leds(0);
    }
}

// --- Setup y Loop ---
void setup() {
    delay(100);
    Serial.begin(9600);
    delay(500);

    // --- AÑADIDO POR BLUETOOTH ---
    // Iniciar Bluetooth Serial con un nombre
    SerialBT.begin("ESP32_Control");
    Serial.println();
    Serial.println("Bluetooth inicializado. Listo para conectar en 'ESP32_Control'");
    // --- FIN AÑADIDO ---

    // Inicializar comunicación con Nextion
    NEXTION_SERIAL.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
    delay(100);

    // Limpiar buffer de Nextion
    while (NEXTION_SERIAL.available()) {
        NEXTION_SERIAL.read();
    }

    // Reset de Nextion (opcional)
    nextion_send_command("rest");
    delay(500);
    nextion_send_command("bkcmd=3");
    delay(100);

    // Configurar ADC
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_SENSOR_TEMP, ADC_11db);
    analogSetPinAttenuation(PIN_SENSOR_LDR, ADC_11db);

    // Configurar pines
    pinMode(PIN_CALENTADOR, OUTPUT);
    digitalWrite(PIN_CALENTADOR, LOW);
    pinMode(PIN_SENSOR_TEMP, INPUT);
    pinMode(PIN_SENSOR_LDR, INPUT);

    // Configurar PWM
    configurar_pwm();
    set_potencia_ventilador(0);
    set_potencia_leds(0);

    // Enviar valores iniciales a Nextion
    delay(300);
    nextion_update_temp_control(T_CONTROL);
    nextion_update_temp_medida(25.0);
    nextion_update_text("t_temp", "25.00");
    nextion_update_iluminacion(50.0);

    Serial.println();
    Serial.println(" ╔═══════════════════════════════════════════╗ ");
    Serial.println(" ║   Sistema Domótico con HMI Nextion         ║ ");
    Serial.println(" ╚═══════════════════════════════════════════╝ ");
    Serial.print("Componentes Nextion: v0, x0, n1, n2, b210\n");
    Serial.print("T° Control Inicial: ");
    Serial.print(T_CONTROL, 1);
    Serial.println("°C");
    Serial.println("Nextion Serial2: RX=GPIO16, TX=GPIO17");
    Serial.println("─────────────────────────────────────────────");
}

void loop() {
    // 1. Procesar mensajes desde Nextion
    nextion_process_message();

    // --- AÑADIDO POR BLUETOOTH ---
    // 1b. Procesar mensajes desde Bluetooth
    procesar_bluetooth();
    // --- FIN AÑADIDO ---

    // 2. Lectura de Sensores
    temperatura_actual = leer_temperatura();
    iluminacion_percent = leer_iluminacion();

    // --- Lógica de Control MODIFICADA ---
    // 3. Lógica de Control (solo si el sistema está activo)
    if (sistema_activo) {
        control_temperatura();
        control_iluminacion();
    } else {
        // Si el sistema está inactivo (comando "B"), forzar apagado
        digitalWrite(PIN_CALENTADOR, LOW);
        set_potencia_ventilador(0);
        set_potencia_leds(0);
    }
    // --- FIN MODIFICACIÓN ---

    // 4. Actualizar pantalla Nextion periódicamente
    unsigned long current_millis = millis();
    if (current_millis - last_nextion_update >= NEXTION_UPDATE_INTERVAL) {
        nextion_update_temp_medida(temperatura_actual);
        nextion_update_temp_texto();
        nextion_update_iluminacion(iluminacion_percent);
        last_nextion_update = current_millis;
    }

    // 5. Monitor Serial USB (Debug)
    Serial.print("["); Serial.print(millis()/1000); Serial.print("s] ");
    // --- MODIFICADO POR BLUETOOTH ---
    if (!sistema_activo) {
        Serial.print("[SISTEMA APAGADO] | ");
    }
    // --- FIN MODIFICACIÓN ---
    Serial.print("Tc:"); Serial.print(T_CONTROL, 1); Serial.print("°C | ");
    Serial.print("T:"); Serial.print(temperatura_actual, 2); Serial.print("°C | ");
    Serial.print("LEDs:"); Serial.print(leds_potencia); Serial.print("% | ");
    Serial.print("Vent:"); Serial.print(ventilador_potencia); Serial.print("% | ");
    Serial.print("Ilum:"); Serial.print(iluminacion_percent, 1); Serial.print("%");
    Serial.print(" [ADC:"); Serial.print(iluminacion_ldr_raw); Serial.println("]");

    delay(1000); // El delay original está bien para este bucle
}