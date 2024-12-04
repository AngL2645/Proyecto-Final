#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <stdbool.h>

// Número de ciclos de lavado
static const uint8_t CICLOS_LAVADO = 10;

// Variables de tiempo (en segundos)
static float tiempo_vaciado = 1.0;
static float tiempo_llena = 1.0;
// static float tiempo_baja = 1.0;
static float tiempo_espera_fondo = 1.0;
// static float tiempo_suba = 1.0;
static float tiempo_bomba_jabon_on = 1.0;
static float tiempo_bomba_jabon_off = 1.0;

// Duty cycles en porcentaje (1-100)
static float duty_giro = 50.0;
static float duty_subir = 80.0;
static float duty_bajar = 50.0;

static bool ciclos_activos = true; // Indica si el sistema está en ciclos de lavado
static const char *TAG = "SistemaLavado";

#define BOTON_GPIO 32                // pin para el botón
#define FINAL_CARRERA_ARRIBA_GPIO 27 // pin para final de carrera arriba
#define FINAL_CARRERA_ABAJO_GPIO 35  // pin para final de carrera abajo

// Pines para bombas y motores
#define BOMBA1_GPIO 33
#define BOMBA2_GPIO 22
#define BOMBA3_GPIO 25
#define PWM_GIRO_GPIO 26
#define PWM_SUBIR_GPIO 14
#define PWM_BAJAR_GPIO 19

// Pines para LEDs indicadores
#define LED_GIRO_GPIO 21
#define LED_BOMBA1_GPIO 15 // Llenado
#define LED_BOMBA2_GPIO 5 // Vaciado
#define LED_BOMBA3_GPIO 4 // Jabon

// Configuración de PWM
#define PWM_FREQUENCY 1000               // Frecuencia del PWM en Hz
#define PWM_RESOLUTION LEDC_TIMER_10_BIT // Resolución de 10 bits (0-1023)

// Función para inicializar GPIOs
void init_gpio()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOMBA1_GPIO) | (1ULL << BOMBA2_GPIO) | (1ULL << BOMBA3_GPIO) |
                        (1ULL << LED_GIRO_GPIO) | (1ULL << LED_BOMBA1_GPIO) |
                        (1ULL << LED_BOMBA2_GPIO) | (1ULL << LED_BOMBA3_GPIO), // Añadido LED de espera en el fondo
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
}

// Configuración de PWM (LEDC)
void init_pwm(ledc_channel_t channel, gpio_num_t gpio_num)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio_num,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);
}
void inicializar_boton()
{
    gpio_config_t config;
    config.pin_bit_mask = (1ULL << BOTON_GPIO) | (1ULL << FINAL_CARRERA_ARRIBA_GPIO) | (1ULL << FINAL_CARRERA_ABAJO_GPIO);
    config.mode = GPIO_MODE_INPUT;               // Configurar como entrada
    config.pull_up_en = GPIO_PULLUP_ENABLE;      // Habilitar resistencia pull-up
    config.pull_down_en = GPIO_PULLDOWN_DISABLE; // Deshabilitar resistencia pull-down
    config.intr_type = GPIO_INTR_DISABLE;        // Sin interrupciones

    gpio_config(&config);
}

// Convertir duty cycle de porcentaje (1-100) a valor de resolución
uint32_t calcular_duty(float duty_percent)
{
    return (uint32_t)((duty_percent / 100.0) * ((1 << PWM_RESOLUTION) - 1));
}

// Función para controlar LEDs
void control_led(gpio_num_t led_gpio, bool encender)
{
    gpio_set_level(led_gpio, encender ? 1 : 0);
}

// Control de motores con PWM
void control_motor(ledc_channel_t channel, float duty_cycle)
{
    uint32_t duty = calcular_duty(duty_cycle);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}
void detener_motores()
{
    // Detener todos los motores (establecer duty cycle a 0)
    control_motor(LEDC_CHANNEL_0, 0); // Motor de subida
    control_motor(LEDC_CHANNEL_1, 0); // Motor de bajada
    control_motor(LEDC_CHANNEL_2, 0); // Motor de giro
}
// Tarea para la bomba de jabón
void tarea_bomba_jabon(void *pvParameters)
{
    while (1)
    {
        if (!ciclos_activos)
        {
            control_motor(LEDC_CHANNEL_0, 0); // Motor de subida
            control_motor(LEDC_CHANNEL_1, 0); // Motor de bajada
            control_motor(LEDC_CHANNEL_2, 0); // Motor de giro
            gpio_set_level(BOMBA3_GPIO, 0);
            control_led(LED_BOMBA3_GPIO, false);
            ESP_LOGI(TAG, "Bomba de jabón detenida: ciclos completados");
            vTaskDelete(NULL); // Termina la tarea cuando los ciclos estén completos
        }

        // Control de la bomba de jabón sin afectar los motores
        gpio_set_level(BOMBA3_GPIO, 1);
        control_led(LED_BOMBA3_GPIO, true);
        vTaskDelay(pdMS_TO_TICKS(tiempo_bomba_jabon_on * 1000));

        gpio_set_level(BOMBA3_GPIO, 0);
        control_led(LED_BOMBA3_GPIO, false);
        vTaskDelay(pdMS_TO_TICKS(tiempo_bomba_jabon_off * 1000));
    }
}

void app_main()
{
    // Inicializar el GPIO del botón
    inicializar_boton(); // Configura el GPIO como entrada
    // Inicializar motores y bombas
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Iniciando sistema...");
    init_gpio();
    init_pwm(LEDC_CHANNEL_0, PWM_SUBIR_GPIO);
    init_pwm(LEDC_CHANNEL_1, PWM_BAJAR_GPIO);
    init_pwm(LEDC_CHANNEL_2, PWM_GIRO_GPIO);
    detener_motores();
    control_led(LED_GIRO_GPIO, false);

    while (true) // Bucle principal
    {
        // Verifica si el botón está presionado
        if (gpio_get_level(BOTON_GPIO) == 0) // Nivel bajo indica botón presionado
        {
            ESP_LOGI(TAG, "Botón presionado. Iniciando sistema...");

            // Realiza las acciones iniciales de vacío y llenado

            ESP_LOGI(TAG, "Llenando depósito...");
            control_led(LED_BOMBA1_GPIO, true);
            gpio_set_level(BOMBA1_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(tiempo_llena * 1000));
            gpio_set_level(BOMBA1_GPIO, 0);
            control_led(LED_BOMBA1_GPIO, false);

            // Crear la tarea de la bomba de jabón sin interferir con los motores
            xTaskCreate(tarea_bomba_jabon, "TareaBombaJabon", 2048, NULL, 1, NULL);

            // Ejecutar los ciclos de lavado
            for (int ciclo = 0; ciclo < CICLOS_LAVADO; ciclo++)
            {
                ESP_LOGI(TAG, "Iniciando ciclo de lavado %d", ciclo + 1);
                ciclos_activos = true;
                // Verificar estado del botón durante cada ciclo
                if (gpio_get_level(BOTON_GPIO) == 1) // Botón no presionado
                {
                    ESP_LOGW(TAG, "Botón liberado. Deteniendo el sistema...");
                    ciclos_activos = false;
                    detener_motores();
                    break;
                }

                // Activar giro
                ESP_LOGI(TAG, "Girando...");
                control_led(LED_GIRO_GPIO, true);
                control_motor(LEDC_CHANNEL_2, duty_giro);

                // Esperar en el fondo
                ESP_LOGI(TAG, "Esperando en el fondo...");
                vTaskDelay(pdMS_TO_TICKS(tiempo_espera_fondo * 1000));


                //------------------------------------------------------------------------------------
                // Movimiento de subida controlado por final de carrera
                ESP_LOGI(TAG, "Subiendo...");
                control_motor(LEDC_CHANNEL_0, duty_subir);
                while (gpio_get_level(FINAL_CARRERA_ARRIBA_GPIO) != 1)
                {

                    ESP_LOGI(TAG, "Esperando activación del final de carrera ARRIBA...");
                    vTaskDelay(pdMS_TO_TICKS(10));       // Evitar saturar la CPU
                    if (gpio_get_level(BOTON_GPIO) == 1) // Botón no presionado
                    {
                        ESP_LOGW(TAG, "Botón liberado. Deteniendo el sistema...");
                        ciclos_activos = false;
                        detener_motores();
                        control_led(LED_GIRO_GPIO, false);

                        break;
                    }
                }
                control_motor(LEDC_CHANNEL_0, 0);

                ESP_LOGI(TAG, "Máquina llegó a la posición arriba.");
                // Pequeño retardo para asegurar que el motor se detuvo
                vTaskDelay(pdMS_TO_TICKS(10));

                // Movimiento de bajada controlado por final de carrera
                ESP_LOGI(TAG, "Bajando...");
                control_motor(LEDC_CHANNEL_1, duty_bajar);
                while (gpio_get_level(FINAL_CARRERA_ABAJO_GPIO) != 1)
                {

                    ESP_LOGI(TAG, "Esperando activación del final de carrera ABAJO...");
                    vTaskDelay(pdMS_TO_TICKS(10));       // Evitar saturar la CPU
                    if (gpio_get_level(BOTON_GPIO) == 1) // Botón no presionado
                    {
                        ESP_LOGW(TAG, "Botón liberado. Deteniendo el sistema...");
                        ciclos_activos = false;
                        detener_motores();
                        control_led(LED_GIRO_GPIO, false);

                        break;
                    }
                }
                control_motor(LEDC_CHANNEL_1, 0);
                ESP_LOGI(TAG, "Máquina llegó a la posición abajo.");
                // Pequeño retardo para asegurar que el motor se detuvo
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            ESP_LOGI(TAG, "Vaciando depósito...");
            control_led(LED_BOMBA2_GPIO, true);
            gpio_set_level(BOMBA2_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(tiempo_vaciado * 1000));
            gpio_set_level(BOMBA2_GPIO, 0);
            control_led(LED_BOMBA2_GPIO, false);
            //----------------------------------------------------------------------------------------------------------------
            // Finalizar el sistema tras completar todos los ciclos
            ESP_LOGI(TAG, "Todos los ciclos de lavado han sido completados");
            ciclos_activos = false;
            detener_motores();
            control_led(LED_GIRO_GPIO, false);


            // Esperar a que el botón sea liberado antes de continuar
            ESP_LOGE(TAG, "Esperando que el botón sea liberado...");
            while (gpio_get_level(BOTON_GPIO) == 0)
            {
                vTaskDelay(pdMS_TO_TICKS(100)); // Espera mientras el botón siga presionado
            }

            ESP_LOGI(TAG, "Botón liberado. Sistema en espera...");
        }

        else
        {
            // Botón no presionado, sistema en espera
            ESP_LOGI(TAG, "Esperando que el botón sea presionado...");
            vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña espera para no saturar la CPU
        }
    }
}
