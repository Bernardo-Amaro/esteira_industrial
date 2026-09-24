// Esteira Industrial (ESP32 + FreeRTOS) — 4 tasks com touch ISR & Instrumentação
// - ENC_SENSE (periódica 5 ms) -> notifica SPD_CTRL
// - SPD_CTRL (hard RT) -> controle PI simulado, trata HMI (soft) se solicitado
// - SORT_ACT (hard RT, evento Touch B) -> aciona "desviador"
// - SAFETY_TASK (hard RT, evento Touch D) -> E-stop
// Compilado com ESP-IDF 5.x / 6.x

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"

#define TAG "ESTEIRA"

// ====== Mapeamento dos touch pads ======
#define TP_OBJ   TOUCH_PAD_NUM7   // Touch B -> detecção de objeto (desvio)
#define TP_HMI   TOUCH_PAD_NUM8   // Touch C -> HMI/telemetria
#define TP_ESTOP TOUCH_PAD_NUM9   // Touch D -> E-stop

// ====== Periodicidade, prioridades, stack ======
#define ENC_T_MS        5
#define PRIO_ESTOP      5
#define PRIO_ENC        3
#define PRIO_CTRL       2
#define PRIO_SORT       4
#define STK             3072

// ====== Limites de Deadline (em microssegundos) para auditoria ======
#define DEADLINE_SORT_US  2000  // Exemplo: 2 ms max para acionar o desviador
#define DEADLINE_ESTOP_US 1500  // Exemplo: 1.5 ms max para o E-stop atuar

// ====== Handles/IPC ======
static TaskHandle_t hENC = NULL, hCTRL = NULL, hSORT = NULL, hSAFE = NULL;
static TaskHandle_t hCtrlNotify = NULL;

typedef struct {
    int64_t t_evt_us;   
} sort_evt_t;
static QueueHandle_t qSort = NULL;

static SemaphoreHandle_t semEStop = NULL;
static SemaphoreHandle_t semHMI   = NULL;

typedef struct {
    float rpm;
    float pos_mm;
    float set_rpm;
} belt_state_t;

static belt_state_t g_belt = { .rpm = 0.f, .pos_mm = 0.f, .set_rpm = 120.0f };

// Contadores globais de desempenho para o relatório
static uint32_t g_sort_misses = 0;
static uint32_t g_estop_misses = 0;

// ====== Util: busy loop previsível (~WCET) ======
static inline void cpu_tight_loop_us(uint32_t us)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        __asm__ __volatile__("nop");
    }
}

// ====== ISR dos Touch Pads ======
static void tierras_touch_isr_handler(void *arg)
{
    BaseType_t HPW = pdFALSE;
    uint32_t pad_intr = touch_pad_get_status(); 
    touch_pad_clear_status();                   

    int64_t t_now = esp_timer_get_time(); // timestamp_evento

    if ((pad_intr & (1 << TP_OBJ)) && qSort) {
        sort_evt_t ev = { .t_evt_us = t_now };
        xQueueSendFromISR(qSort, &ev, &HPW);
    }

    if ((pad_intr & (1 << TP_HMI)) && semHMI) {
        xSemaphoreGiveFromISR(semHMI, &HPW);
    }

    if ((pad_intr & (1 << TP_ESTOP)) && semEStop) {
        xSemaphoreGiveFromISR(semEStop, &HPW);
    }

    if (HPW) {
        portYIELD_FROM_ISR();
    }
}

// ====== ENC_SENSE (periódica 5 ms): estima velocidade/posição ======
static void task_enc_sense(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    const TickType_t T = pdMS_TO_TICKS(ENC_T_MS); 

    for (;;) {
        float err = g_belt.set_rpm - g_belt.rpm;
        g_belt.rpm += 0.05f * err;             
        g_belt.pos_mm += (g_belt.rpm / 60.0f) * (ENC_T_MS / 1000.0f) * 100.0f; 

        cpu_tight_loop_us(700);

        if (hCtrlNotify) xTaskNotifyGive(hCtrlNotify);

        vTaskDelayUntil(&next, T);
    }
}

// ====== SPD_CTRL (encadeada): controle PI simulado + HMI (soft) ======
static void task_spd_ctrl(void *arg)
{
    // um controlador PI minimalista e simulado
    float kp = 0.4f, ki = 0.1f, integ = 0.f;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // acorda após ENC_SENSE

        // Controle (hard): alvo é g_belt.set_rpm, atuando em g_belt.rpm (simulado)
        float err = g_belt.set_rpm - g_belt.rpm;
        integ += err * (ENC_T_MS / 1000.0f);
        float u = kp * err + ki * integ;
        // "Aplicar" u: aqui apenas ajustamos o setpoint ligeiramente (mock)
        g_belt.set_rpm += 0.1f * u;

        // Carga determinística ~1.2 ms
        cpu_tight_loop_us(1200);

        // Trecho não-crítico (soft): se HMI solicitada, imprime e retorna rápido
        if (xSemaphoreTake(semHMI, 0) == pdTRUE) {
            ESP_LOGI(TAG, "HMI: rpm=%.1f set=%.1f pos=%.1fmm", g_belt.rpm, g_belt.set_rpm, g_belt.pos_mm);
            cpu_tight_loop_us(400); // ~0.4 ms (soft)
        }
    }
}

// ====== SORT_ACT (evento Touch B): aciona "desviador" no tempo certo ======
static void task_sort_act(void *arg)
{
    sort_evt_t ev;
    for (;;) {
        if (xQueueReceive(qSort, &ev, portMAX_DELAY) == pdTRUE) {
            int64_t t_start = esp_timer_get_time(); // timestamp_task_start
            
            cpu_tight_loop_us(700);
            
            int64_t t_end = esp_timer_get_time();   // timestamp_task_end
            
            int64_t latency = t_start - ev.t_evt_us;
            int64_t total_time = t_end - ev.t_evt_us;

            if (total_time > DEADLINE_SORT_US) {
                g_sort_misses++;
            }

            ESP_LOGI(TAG, "[METRICA_SORT] timestamp_evento=%lld timestamp_task_start=%lld timestamp_task_end=%lld latencia_isr_us=%lld tempo_total_us=%lld misses=%lu",
                     (long long)ev.t_evt_us, (long long)t_start, (long long)t_end, 
                     (long long)latency, (long long)total_time, g_sort_misses);
        }
    }
}

// ====== SAFETY_TASK (evento Touch D): E-stop ======
static void task_safety(void *arg)
{
    for (;;) {
        if (xSemaphoreTake(semEStop, portMAX_DELAY) == pdTRUE) {
            int64_t t_start = esp_timer_get_time(); // timestamp_task_start
            
            g_belt.set_rpm = 0.f;
            cpu_tight_loop_us(900);
            
            int64_t t_end = esp_timer_get_time();   // timestamp_task_end
            int64_t total_time = t_end - t_start;

            if (total_time > DEADLINE_ESTOP_US) {
                g_estop_misses++;
            }

            ESP_LOGW(TAG, "[METRICA_ESTOP] timestamp_task_start=%lld timestamp_task_end=%lld tempo_exec_us=%lld misses=%lu",
                     (long long)t_start, (long long)t_end, (long long)total_time, g_estop_misses);
        }
    }
}

// ====== Inicialização do Touch ======
static void init_touch_pads(void)
{
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V5);

    // Configura os pinos individuais
    touch_pad_config(TP_OBJ, 0);
    touch_pad_config(TP_HMI, 0);
    touch_pad_config(TP_ESTOP, 0);

    // Máscara com os 3 parâmetros exigidos pela API legada
    touch_pad_set_group_mask(0, 0, (1 << TP_OBJ) | (1 << TP_HMI) | (1 << TP_ESTOP));

    // Inicia o filtro de leituras
    touch_pad_filter_start(10);
    vTaskDelay(pdMS_TO_TICKS(50)); 

    // Calibração dos limiares (threshold) baseados no valor de repouso
    uint16_t val = 0;
    
    touch_pad_read_filtered(TP_OBJ, &val);
    if (val > 0) touch_pad_set_thresh(TP_OBJ, val * 2 / 3);

    touch_pad_read_filtered(TP_HMI, &val);
    if (val > 0) touch_pad_set_thresh(TP_HMI, val * 2 / 3);

    touch_pad_read_filtered(TP_ESTOP, &val);
    if (val > 0) touch_pad_set_thresh(TP_ESTOP, val * 2 / 3);

    // Registra a interrupção geral de touch e ativa
    touch_pad_isr_register(tierras_touch_isr_handler, NULL);
    touch_pad_intr_enable();
}

// ====== app_main ======
void app_main(void)
{
    // 1. Cria IPCs
    semEStop = xSemaphoreCreateBinary();
    semHMI   = xSemaphoreCreateBinary();
    qSort    = xQueueCreate(5, sizeof(sort_evt_t));

    if (!semEStop || !semHMI || !qSort) {
        ESP_LOGE(TAG, "Falha ao criar recursos de IPC!");
        return;
    }

    // 2. Inicializa os touch pads físicos
    init_touch_pads();

    // 3. Cria tasks
    xTaskCreatePinnedToCore(task_safety,   "SAFETY",   STK, NULL, PRIO_ESTOP, &hSAFE,  0);
    xTaskCreatePinnedToCore(task_enc_sense,"ENC_SENSE",STK, NULL, PRIO_ENC,   &hENC,   0);
    xTaskCreatePinnedToCore(task_spd_ctrl, "SPD_CTRL", STK, NULL, PRIO_CTRL,  &hCTRL,  0);
    xTaskCreatePinnedToCore(task_sort_act, "SORT_ACT", STK, NULL, PRIO_SORT,  &hSORT,  0);

    hCtrlNotify = hCTRL;

    // Remove a task principal para manter o escalonador ativo em segundo plano
    vTaskDelete(NULL);
}