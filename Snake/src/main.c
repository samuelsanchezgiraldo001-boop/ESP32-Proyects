#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h" 
#include "driver/gpio.h"
#include "esp_system.h" 
#include <stdbool.h>

extern uint32_t esp_random(void);

// ===== CONSTANTES =====
#define ROWS 8
#define COLS 6

// PINES BOTONES
#define BTN_UP     36
#define BTN_DOWN   35
#define BTN_LEFT   34
#define BTN_RIGHT  39

// LED MATRIZ PINES
const int rowPins[ROWS] = {26, 25, 33, 32, 19, 21, 22, 23};
const int colRed[COLS]   = {12, 27, 2, 5, 4, 17};
const int colGreen[COLS] = {13, 14, 15, 18, 0, 16};

// ===== VARIABLES DE ESTADO =====
typedef struct { int x, y; } Point;
Point snake[48];
int snakeLength = 2;
int dirX = 1, dirY = 0;
Point food = {3, 3};
Point trapFood = {-1, -1};
bool hasTrap = false;
bool gameOver = false;
bool diedFromTrap = false;
int matrix[ROWS][COLS] = {0};
static int current_row_to_scan = 0;
int score = 0;
int highScore = 0;

// Dígitos optimizados para matriz 6x8
const uint8_t digits[10][ROWS] = {
    {0b01111000,0b11001100,0b11001100,0b11001100,0b11001100,0b11001100,0b11001100,0b01111000}, //0
    {0b00110000,0b01110000,0b00110000,0b00110000,0b00110000,0b00110000,0b00110000,0b11111100}, //1
    {0b01111000,0b11001100,0b00001100,0b00011000,0b00110000,0b01100000,0b11000000,0b11111100}, //2
    {0b01111000,0b11001100,0b00001100,0b00111000,0b00001100,0b00001100,0b11001100,0b01111000}, //3
    {0b00011000,0b00111000,0b01011000,0b10011000,0b11111100,0b00011000,0b00011000,0b00011000}, //4
    {0b11111100,0b11000000,0b11000000,0b11111000,0b00001100,0b00001100,0b11001100,0b01111000}, //5
    {0b01111000,0b11001100,0b11000000,0b11111000,0b11001100,0b11001100,0b11001100,0b01111000}, //6
    {0b11111100,0b00001100,0b00011000,0b00110000,0b01100000,0b01100000,0b01100000,0b01100000}, //7
    {0b01111000,0b11001100,0b11001100,0b01111000,0b11001100,0b11001100,0b11001100,0b01111000}, //8
    {0b01111000,0b11001100,0b11001100,0b11001100,0b01111100,0b00001100,0b11001100,0b01111000}  //9
};

// Dibujo de serpiente para pantalla de inicio
const uint8_t snakeImage[ROWS] = {
    0b00000000,
    0b00111100,
    0b01000010,
    0b01011010,
    0b01000010,
    0b00111100,
    0b00011000,
    0b00000000
};

// Dibujo de calavera para game over
const uint8_t skullImage[ROWS] = {
    0b00111100,
    0b01111110,
    0b11011011,
    0b11111111,
    0b01111110,
    0b00111100,
    0b01010010,
    0b00000000
};

// -----------------------------------------------------
// --- DISPLAY ---
// -----------------------------------------------------

void IRAM_ATTR display_timer_callback(void* arg) {
    int prev_row = (current_row_to_scan - 1 + ROWS) % ROWS;
    gpio_set_level(rowPins[prev_row], 0);

    for (int c = 0; c < COLS; c++) {
        if (matrix[current_row_to_scan][c] == 1) {
            gpio_set_level(colGreen[c], 1);
            gpio_set_level(colRed[c], 0);
        } else if (matrix[current_row_to_scan][c] == 2) {
            gpio_set_level(colGreen[c], 0);
            gpio_set_level(colRed[c], 1);
        } else if (matrix[current_row_to_scan][c] == 3) {
            gpio_set_level(colGreen[c], 1);
            gpio_set_level(colRed[c], 1);
        } else {
            gpio_set_level(colGreen[c], 0);
            gpio_set_level(colRed[c], 0);
        }
    }

    gpio_set_level(rowPins[current_row_to_scan], 1);
    current_row_to_scan = (current_row_to_scan + 1) % ROWS;
}

void init_gpio() {
    for (int i = 0; i < ROWS; i++) {
        gpio_reset_pin(rowPins[i]);
        gpio_set_direction(rowPins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(rowPins[i], 0);
    }
    
    for (int i = 0; i < COLS; i++) {
        gpio_reset_pin(colRed[i]);
        gpio_set_direction(colRed[i], GPIO_MODE_OUTPUT);
        gpio_set_level(colRed[i], 0);
        
        gpio_reset_pin(colGreen[i]);
        gpio_set_direction(colGreen[i], GPIO_MODE_OUTPUT);
        gpio_set_level(colGreen[i], 0);
    }
}

void init_display_timer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &display_timer_callback,
        .name = "matrix_scan"
    };
    esp_timer_handle_t matrix_timer;
    esp_timer_create(&timer_args, &matrix_timer);
    esp_timer_start_periodic(matrix_timer, 1500);
}

// -----------------------------------------------------
// --- ENTRADAS DE BOTON ---
// -----------------------------------------------------

void init_buttons() {
    printf("Inicializando botones...\n");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_UP) | (1ULL << BTN_DOWN) |
                        (1ULL << BTN_LEFT) | (1ULL << BTN_RIGHT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    printf("Botones configurados:\n");
    printf("  UP (36): %d\n", gpio_get_level(BTN_UP));
    printf("  DOWN (35): %d\n", gpio_get_level(BTN_DOWN));
    printf("  LEFT (34): %d\n", gpio_get_level(BTN_LEFT));
    printf("  RIGHT (39): %d\n", gpio_get_level(BTN_RIGHT));
    printf("(1=sin presionar, 0=presionado)\n\n");
}

bool anyButtonPressed() {
    bool pressed = (gpio_get_level(BTN_UP) == 0 ||
                   gpio_get_level(BTN_DOWN) == 0 ||
                   gpio_get_level(BTN_LEFT) == 0 ||
                   gpio_get_level(BTN_RIGHT) == 0);
    
    if (pressed) {
        printf(">>> Boton detectado!\n");
    }
    
    return pressed;
}

void checkInput() {
    int up = gpio_get_level(BTN_UP);
    int down = gpio_get_level(BTN_DOWN);
    int left = gpio_get_level(BTN_LEFT);
    int right = gpio_get_level(BTN_RIGHT);

    if (up == 0 && dirY != 1) { 
        dirX = 0; dirY = -1;
    }
    else if (down == 0 && dirY != -1) { 
        dirX = 0; dirY = 1;
    }
    else if (left == 0 && dirX != 1) { 
        dirX = -1; dirY = 0;
    }
    else if (right == 0 && dirX != -1) { 
        dirX = 1; dirY = 0;
    }
}

// -----------------------------------------------------
// --- GAME LOGIC ---
// -----------------------------------------------------

void clearMatrix(void) { memset(matrix, 0, sizeof(matrix)); }

void spawnFood() {
    Point newFood;
    bool collision;
    do {
        collision = false;
        newFood.x = rand() % COLS;
        newFood.y = rand() % ROWS;
        for (int i = 0; i < snakeLength; i++)
            if (newFood.x == snake[i].x && newFood.y == snake[i].y)
                collision = true;
        if (hasTrap && newFood.x == trapFood.x && newFood.y == trapFood.y)
            collision = true;
    } while (collision);
    food = newFood;
}

void spawnTrapFood() {
    if (rand() % 100 < 30) {
        Point newTrap;
        bool collision;
        do {
            collision = false;
            newTrap.x = rand() % COLS;
            newTrap.y = rand() % ROWS;
            for (int i = 0; i < snakeLength; i++)
                if (newTrap.x == snake[i].x && newTrap.y == snake[i].y)
                    collision = true;
            if (newTrap.x == food.x && newTrap.y == food.y)
                collision = true;
        } while (collision);
        trapFood = newTrap;
        hasTrap = true;
        printf("⚠️  ¡Comida trampa amarilla apareció en (%d,%d)!\n", trapFood.x, trapFood.y);
    }
}

void moveSnake() {
    if (gameOver) return;
    Point oldTail = snake[snakeLength - 1];
    for (int i = snakeLength - 1; i > 0; i--)
        snake[i] = snake[i - 1];
    snake[0].x += dirX;
    snake[0].y += dirY;

    if (snake[0].x < 0) snake[0].x = COLS - 1;
    if (snake[0].x >= COLS) snake[0].x = 0;
    if (snake[0].y < 0) snake[0].y = ROWS - 1;
    if (snake[0].y >= ROWS) snake[0].y = 0;

    for (int i = 1; i < snakeLength; i++)
        if (snake[i].x == snake[0].x && snake[i].y == snake[0].y) {
            gameOver = true;
            diedFromTrap = false;
            printf("\n");
            printf("╔═════════════════════════════╗\n");
            printf("║      💥 GAME OVER! 💥       ║\n");
            printf("║   Te mordiste a ti mismo    ║\n");
            printf("╠═════════════════════════════╣\n");
            printf("║  Score Final: %-3d          ║\n", score);
            printf("║  Longitud: %-3d              ║\n", snakeLength);
            
            if (score > highScore) {
                highScore = score;
                printf("║  🏆 NUEVO RECORD: %-3d 🏆   ║\n", highScore);
            } else {
                printf("║  Record actual: %-3d        ║\n", highScore);
            }
            printf("╚═════════════════════════════╝\n\n");
            return;
        }

    if (hasTrap && snake[0].x == trapFood.x && snake[0].y == trapFood.y) {
        gameOver = true;
        diedFromTrap = true;
        printf("\n");
        printf("╔═════════════════════════════╗\n");
        printf("║      ☠️  GAME OVER! ☠️       ║\n");
        printf("║   ¡Comiste la comida trampa! ║\n");
        printf("╠═════════════════════════════╣\n");
        printf("║  Score Final: %-3d          ║\n", score);
        printf("║  Longitud: %-3d              ║\n", snakeLength);
        
        if (score > highScore) {
            highScore = score;
            printf("║  🏆 NUEVO RECORD: %-3d 🏆   ║\n", highScore);
        } else {
            printf("║  Record actual: %-3d        ║\n", highScore);
        }
        printf("╚═════════════════════════════╝\n\n");
        return;
    }

    if (snake[0].x == food.x && snake[0].y == food.y) {
        if (snakeLength < 48) {
            snake[snakeLength] = oldTail;
            snakeLength++;
        }
        score++;
        printf("┌─────────────────────────────┐\n");
        printf("│  🍎 COMIDA!                 │\n");
        printf("│  Score actual: %-3d          │\n", score);
        printf("│  Longitud: %-3d              │\n", snakeLength);
        printf("│  RECORD: %-3d                │\n", highScore);
        printf("└─────────────────────────────┘\n\n");
        
        spawnFood();
        spawnTrapFood();
    }
}

void updateMatrix() {
    clearMatrix();
    for (int i = 0; i < snakeLength; i++)
        matrix[snake[i].y][snake[i].x] = 1;
    matrix[food.y][food.x] = 2;
    if (hasTrap)
        matrix[trapFood.y][trapFood.x] = 3;
}

// -----------------------------------------------------
// --- DISPLAY FUNCIONES ---
// -----------------------------------------------------

void showDigit(int num) {
    clearMatrix();
    if (num < 0 || num > 9) return;
    for (int y = 0; y < ROWS; y++)
        for (int x = 0; x < COLS; x++)
            matrix[y][x] = (digits[num][y] >> (7 - x - 2)) & 1 ? 1 : 0;
}

void showSnake() {
    clearMatrix();
    for (int y = 0; y < ROWS; y++)
        for (int x = 0; x < COLS; x++)
            matrix[y][x] = (snakeImage[y] >> (7 - x - 2)) & 1 ? 1 : 0;
}

void showSkull() {
    clearMatrix();
    for (int y = 0; y < ROWS; y++)
        for (int x = 0; x < COLS; x++)
            matrix[y][x] = (skullImage[y] >> (7 - x - 2)) & 1 ? 2 : 0;
}

// -----------------------------------------------------
// --- ANIMACIONES ---
// -----------------------------------------------------

void showXAnimation() {
    clearMatrix();
    
    for(int i = 0; i < ROWS && i < COLS; i++) {
        matrix[i][i] = 2;
    }
    
    for(int i = 0; i < ROWS; i++) {
        int col = COLS - 1 - i;
        if(col >= 0 && col < COLS) {
            matrix[i][col] = 2;
        }
    }
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    clearMatrix();
}

void showTrapDeathAnimation() {
    clearMatrix();
    
    for (int blink = 0; blink < 3; blink++) {
        for (int y = 0; y < ROWS; y++) {
            matrix[y][0] = 2;
            matrix[y][COLS-1] = 2;
        }
        for (int x = 0; x < COLS; x++) {
            matrix[0][x] = 2;
            matrix[ROWS-1][x] = 2;
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);
        clearMatrix();
        vTaskDelay(200 / portTICK_PERIOD_MS);
        
        int midY = ROWS / 2;
        int midX = COLS / 2;
        for (int i = 0; i < ROWS; i++) {
            matrix[i][midX] = 2;
        }
        for (int i = 0; i < COLS; i++) {
            matrix[midY][i] = 2;
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);
        clearMatrix();
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    for (int flash = 0; flash < 4; flash++) {
        for (int y = 0; y < ROWS; y++) {
            for (int x = 0; x < COLS; x++) {
                matrix[y][x] = 2;
            }
        }
        vTaskDelay(150 / portTICK_PERIOD_MS);
        clearMatrix();
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}

void demoAnimation() {
    printf("Mostrando demo del juego...\n");
    printf("(Presiona cualquier boton para saltar el demo)\n\n");
    
    Point demoSnake[15];
    int demoLength = 4;
    
    for (int i = 0; i < demoLength; i++) {
        demoSnake[i] = (Point){0, 0};
    }
    
    int demoDirX = 1;
    int demoDirY = 0;
    
    Point demoFood = {COLS-1, ROWS-1};
    Point demoTrap = {COLS/2, ROWS/2};
    bool demoHasTrap = true;
    
    int demoScore = 0;
    int stepsInCurrentDirection = 0;
    
    int moveSequence[] = {
        5, 4, 5, 3, 3, 4, 2, 5, 4, 3, 4, 6, 5, 7, 5, 4
    };
    
    int sequenceLength = sizeof(moveSequence) / sizeof(moveSequence[0]);
    int currentMoveIndex = 0;
    
    int directions[][2] = {
        {1, 0}, {0, 1}, {-1, 0}, {0, -1}
    };
    int currentDir = 0;
    
    for(int frame = 0; frame < 150; frame++) {
        for (int i = demoLength - 1; i > 0; i--) {
            demoSnake[i] = demoSnake[i - 1];
        }
        
        demoSnake[0].x += demoDirX;
        demoSnake[0].y += demoDirY;
        
        if (demoSnake[0].x < 0) demoSnake[0].x = COLS - 1;
        if (demoSnake[0].x >= COLS) demoSnake[0].x = 0;
        if (demoSnake[0].y < 0) demoSnake[0].y = ROWS - 1;
        if (demoSnake[0].y >= ROWS) demoSnake[0].y = 0;
        
        stepsInCurrentDirection++;
        
        if (stepsInCurrentDirection >= moveSequence[currentMoveIndex]) {
            stepsInCurrentDirection = 0;
            currentMoveIndex = (currentMoveIndex + 1) % sequenceLength;
            currentDir = (currentDir + 1) % 4;
            demoDirX = directions[currentDir][0];
            demoDirY = directions[currentDir][1];
        }
        
        if (demoSnake[0].x == demoFood.x && demoSnake[0].y == demoFood.y) {
            if (demoLength < 15) demoLength++;
            demoScore++;
            
            demoFood.x = rand() % COLS;
            demoFood.y = rand() % ROWS;
            
            if (rand() % 100 < 50) {
                demoTrap.x = rand() % COLS;
                demoTrap.y = rand() % ROWS;
                if (demoTrap.x == demoFood.x && demoTrap.y == demoFood.y) {
                    demoTrap.x = (demoTrap.x + 1) % COLS;
                }
                demoHasTrap = true;
            } else {
                demoHasTrap = false;
            }
            
            printf("Demo Score: %d (Longitud: %d)\n", demoScore, demoLength);
        }
        
        if (demoHasTrap && demoSnake[0].x == demoTrap.x && demoSnake[0].y == demoTrap.y) {
            printf("⚠️ ¡Demo: Casi come la trampa!\n");
            demoHasTrap = false;
        }
        
        clearMatrix();
        
        for (int i = 0; i < demoLength; i++) {
            if (demoSnake[i].x >= 0 && demoSnake[i].x < COLS &&
                demoSnake[i].y >= 0 && demoSnake[i].y < ROWS) {
                matrix[demoSnake[i].y][demoSnake[i].x] = 1;
            }
        }
        
        matrix[demoFood.y][demoFood.x] = 2;
        
        if (demoHasTrap) {
            matrix[demoTrap.y][demoTrap.x] = 3;
        }
        
        vTaskDelay(180 / portTICK_PERIOD_MS);
        
        if (anyButtonPressed()) {
            printf("Demo interrumpida por el usuario\n");
            clearMatrix();
            vTaskDelay(300 / portTICK_PERIOD_MS);
            return;
        }
    }
    
    clearMatrix();
    printf("Demo finalizada - Score final: %d\n", demoScore);
}

// -----------------------------------------------------
// --- MAIN LOOP ---
// -----------------------------------------------------

void app_main() {
    printf("\n\n");
    printf("╔═══════════════════════════════════╗\n");
    printf("║                                   ║\n");
    printf("║       🐍 SNAKE GAME ESP32 🐍      ║\n");
    printf("║                                   ║\n");
    printf("╠═══════════════════════════════════╣\n");
    printf("║  Controles:                       ║\n");
    printf("║    ⬆️  ARRIBA  → Pin 36           ║\n");
    printf("║    ⬇️  ABAJO   → Pin 35           ║\n");
    printf("║    ⬅️  IZQUIERDA → Pin 34         ║\n");
    printf("║    ➡️  DERECHA → Pin 39           ║\n");
    printf("╠═══════════════════════════════════╣\n");
    printf("║  🟢 Verde = Serpiente             ║\n");
    printf("║  🔴 Rojo  = Comida buena          ║\n");
    printf("║  🟡 Amarillo = Comida trampa ☠️   ║\n");
    printf("║                                   ║\n");
    printf("║  💡 Atraviesa las paredes!        ║\n");
    printf("║  ⚠️  Evita la comida amarilla!    ║\n");
    printf("║  ⚠️  No te muerdas a ti mismo!    ║\n");
    printf("╚═══════════════════════════════════╝\n\n");
    
    init_gpio();
    init_buttons();
    init_display_timer();
    srand((unsigned int)esp_random());

    demoAnimation();
    
    printf("\n┌─────────────────────────────┐\n");
    printf("│  Presiona cualquier boton   │\n");
    printf("│    para iniciar el juego    │\n");
    printf("└─────────────────────────────┘\n");
    clearMatrix();
    showSnake();
    while (!anyButtonPressed()) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    snake[0] = (Point){2, 3};
    snake[1] = (Point){1, 3};
    snakeLength = 2;
    dirX = 1; dirY = 0;
    score = 0;
    gameOver = false;
    diedFromTrap = false;
    hasTrap = false;
    spawnFood();
    spawnTrapFood();
    printf("\n🎮 ¡Juego iniciado! 🎮\n");
    printf("Record a superar: %d\n\n", highScore);

    while (1) {
        checkInput();
        
        if (!gameOver) {
            moveSnake();
            updateMatrix();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        } else {
            if (diedFromTrap) {
                showTrapDeathAnimation();
            } else {
                showXAnimation();
            }
            
            clearMatrix();
            showSkull();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            
            clearMatrix();
            showDigit(score % 10);
            printf("Presiona cualquier boton para reiniciar...\n\n");
            while (!anyButtonPressed()) vTaskDelay(20 / portTICK_PERIOD_MS);

            printf("\n🔄 Reiniciando juego...\n");
            printf("Record actual: %d\n\n", highScore);
            snakeLength = 2;
            dirX = 1; dirY = 0;
            score = 0;
            gameOver = false;
            diedFromTrap = false;
            hasTrap = false;
            snake[0] = (Point){2, 3};
            snake[1] = (Point){1, 3};
            spawnFood();
            spawnTrapFood();
        }
    }
}