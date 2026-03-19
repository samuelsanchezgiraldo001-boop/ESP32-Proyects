#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "ds18b20.h"

// Configuración de pines
#define ONE_WIRE_GPIO 18        // GPIO34 para el DS18B20 (data)
#define RELAY_GPIO 26           // GPIO26 para el optoacoplador

// Umbrales de temperatura
#define TEMP_SETPOINT 70.0
#define TEMP_HYSTERESIS 3.0
#define TEMP_MIN (TEMP_SETPOINT - TEMP_HYSTERESIS)  // 67°C
#define TEMP_MAX (TEMP_SETPOINT + TEMP_HYSTERESIS)  // 73°C

// Filtro de media móvil
#define FILTER_SIZE 10
static float temp_buffer[FILTER_SIZE] = {0};
static int buffer_index = 0;
static bool buffer_filled = false;

void emergency_shutdown(void) {
    gpio_set_level(RELAY_GPIO, 1);
    printf("\nAPAGADO DE EMERGENCIA\n");
}

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

static void init_hw(void)
{
    // Configurar GPIO26 para relay
    gpio_reset_pin(RELAY_GPIO);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Iniciar apagado
    gpio_set_level(RELAY_GPIO, 1);
    
    // Inicializar DS18B20
    ds18b20_init(ONE_WIRE_GPIO);
}

void app_main()
{
    init_hw();
    
    printf("\n==============================================\n");
    printf("CONTROL DE TEMPERATURA CON DS18B20\n");
    printf("Temperatura objetivo: %.1f°C\n", TEMP_SETPOINT);
    printf("Banda de histeresis: +/-%.1f°C\n", TEMP_HYSTERESIS);
    printf("  - Enciende si T < %.1f°C\n", TEMP_MIN);
    printf("  - Apaga si T > %.1f°C\n", TEMP_MAX);
    printf("DS18B20: GPIO34 | Relay: GPIO26\n");
    printf("==============================================\n\n");
    
    // Buscar dispositivos DS18B20
    printf("Buscando sensores DS18B20...\n");
    ds18b20_addr_t devices[1];
    int device_count = ds18b20_scan_devices(ONE_WIRE_GPIO, devices, 1);
    
    if (device_count == 0) {
        printf("ERROR: No se encontro DS18B20 en GPIO34\n");
        printf("Verifica:\n");
        printf("  - Conexion fisica (VCC, GND, DATA)\n");
        printf("  - Resistencia pull-up de 4.7k en DATA\n");
        while(1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    
    printf("Sensor encontrado! Direccion: ");
    for(int i = 0; i < 8; i++) {
        printf("%02X ", devices[0].addr[i]);
    }
    printf("\n\n");
    
    // FASE 1: Calentamiento inicial
    printf(">>> FASE 1: CALENTAMIENTO INICIAL (10s) <<<\n\n");
    
    gpio_set_level(RELAY_GPIO, 0);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    int gpio_state = gpio_get_level(RELAY_GPIO);
    printf("GPIO26 = %d -> %s\n\n", gpio_state, 
           gpio_state == 0 ? "ENCENDIDO" : "ERROR");
    
    for (int i = 10; i > 0; i--)
    {
        float temperature = ds18b20_get_temp(devices[0]);
        float temp_filtered = get_filtered_temperature(temperature);
        
        printf("[%2ds] Temp: %.2f°C | Filtrada: %.2f°C | Estado: ENCENDIDO\n", 
               i, temperature, temp_filtered);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    printf("\n>>> FASE 2: CONTROL AUTOMATICO <<<\n\n");
    
    int current_gpio = gpio_get_level(RELAY_GPIO);
    bool heater_on = (current_gpio == 0);
    
    printf("Estado inicial: GPIO26=%d -> %s\n\n", 
           current_gpio, heater_on ? "ENCENDIDO" : "APAGADO");
    
    while (1)
    {
        float temp_raw = ds18b20_get_temp(devices[0]);
        
        // Verificar lectura valida
        if (temp_raw == DS18B20_ERROR || temp_raw < -55.0 || temp_raw > 125.0) {
            printf("ERROR: Lectura invalida del sensor (%.2f°C)\n", temp_raw);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        
        float temperature = get_filtered_temperature(temp_raw);
        
        // Proteccion sobre-temperatura
        if(temperature > 85.0) {
            emergency_shutdown();
            printf("CRITICO: %.2f°C - Sistema detenido\n", temperature);
            while(1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
        }
        
        printf("[Temp Raw: %.2f°C | Filtrada: %.2f°C | GPIO26=%d] ", 
               temp_raw, temperature, gpio_get_level(RELAY_GPIO));
        
        // Control ON/OFF con histeresis
        if (temperature < TEMP_MIN && !heater_on)
        {
            gpio_set_level(RELAY_GPIO, 0);  // ENCENDER
            vTaskDelay(100 / portTICK_PERIOD_MS);
            heater_on = true;
            printf("-> ENCENDER\n");
        }
        else if (temperature > TEMP_MAX && heater_on)
        {
            gpio_set_level(RELAY_GPIO, 1);  // APAGAR
            vTaskDelay(100 / portTICK_PERIOD_MS);
            heater_on = false;
            int final_state = gpio_get_level(RELAY_GPIO);
            printf("-> APAGAR %s\n", final_state == 1 ? "OK" : "ERROR");
            
            if(final_state != 1) {
                emergency_shutdown();
            }
        }
        else
        {
            printf("-> %s\n", heater_on ? "ENCENDIDO" : "APAGADO");
        }
        
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}