#include <Arduino.h>

// --- CONFIGURACION ---
const uint32_t FS_HZ = 300000u;   // 300 kHz de muestreo
const uint16_t N_BLOCK = 512;     // Bloque de procesamiento (igual que tu ejemplo)
const uint16_t N_DMA = N_BLOCK * 2; // Buffer Doble
const int ADC_MID = 2048;         // ADC 12-bit mid

// --- COEFICIENTES Q14 (escalado por 2^14 = 16384) ---
const int32_t B0_INT = 13060;   // 0.7970982 * 16384
const int32_t B1_INT = 0;       // 0.0 * 16384
const int32_t B2_INT = -13060;  // -0.7970982 * 16384
const int32_t A1_INT = -5562;   // -0.33947239 * 16384
const int32_t A2_INT = 13119;   // 0.80072545 * 16384

// --- ESTRUCTURA DEL FILTRO (Direct Form I, Q14) ---
struct Biquad {
  int32_t x1 = 0, x2 = 0;
  int32_t y1 = 0, y2 = 0;

  inline int32_t step(int32_t x0) {
    // Usar acumulador de 64 bits para evitar overflow
    int64_t acc = 0;
    acc += (int64_t)B0_INT * (int64_t)x0;
    acc += (int64_t)B1_INT * (int64_t)x1;
    acc += (int64_t)B2_INT * (int64_t)x2;
    // RESTAR los términos de la parte denominadora estándar:
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    acc -= (int64_t)A1_INT * (int64_t)y1;
    acc -= (int64_t)A2_INT * (int64_t)y2;

    // Shift Q14
    int32_t y0 = (int32_t)(acc >> 14);

    // Actualizar estados
    x2 = x1; x1 = x0;
    y2 = y1; y1 = y0;

    return y0;
  }
};
Biquad filtro;

// --- BUFFERS ---
volatile uint16_t adc_dma_buf[N_DMA];
struct Frame { uint16_t vin; uint16_t vout; };
Frame tx_buf[N_BLOCK];

volatile bool procesar_mitad1 = false;
volatile bool procesar_mitad2 = false;

// --- VARIABLES HAL ---
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim3; 

// --- PROTOCOLO SERIAL (igual que tu ejemplo) ---
const uint8_t MAGIC[4] = {0xA5, 0x5A, 0xA5, 0x5A};

// Declaraciones HAL (vienen del core/STM32CubeMX)
extern "C" void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_TIM3_Init(void);

void setup() {
  Serial.begin(1000000);

  // Inicializar Hardware Bajo Nivel
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();

  // Calibrar ADC
  HAL_ADCEx_Calibration_Start(&hadc1);

  // Arrancar DMA Circular (ADC -> adc_dma_buf)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, N_DMA);

  // Arrancar Timer (genera trigger de muestreo)
  HAL_TIM_Base_Start(&htim3);
}

void loop() {
  static uint16_t seq = 0;

  if (procesar_mitad1) {
    procesar_bloque(0);
    enviar_datos(seq++);
    procesar_mitad1 = false;
  }

  if (procesar_mitad2) {
    procesar_bloque(N_BLOCK);
    enviar_datos(seq++);
    procesar_mitad2 = false;
  }
}

// --- PROCESAMIENTO DEL BLOQUE ---
void procesar_bloque(uint16_t offset) {
  for (int i = 0; i < N_BLOCK; i++) {
    uint16_t raw = adc_dma_buf[offset + i];
    int32_t input = (int32_t)raw - ADC_MID;      // centrar en 0
    int32_t output = filtro.step(input);

    int32_t out_final = output + ADC_MID;       // volver a 0..4095
    if (out_final < 0) out_final = 0;
    if (out_final > 4095) out_final = 4095;

    tx_buf[i].vin = raw;
    tx_buf[i].vout = (uint16_t)out_final;
  }
}

// --- ENVIO POR SERIAL del bloque (misma estructura que tu ejemplo) ---
void enviar_datos(uint16_t seq) {
  Serial.write(MAGIC, 4);
  uint32_t fs = FS_HZ;
  Serial.write((uint8_t*)&fs, 4);
  uint16_t n = N_BLOCK;
  Serial.write((uint8_t*)&n, 2);
  Serial.write((uint8_t*)&seq, 2);
  Serial.write((uint8_t*)tx_buf, sizeof(tx_buf));
  Serial.write(MAGIC, 4);
}

// --- INTERRUPCIONES y callbacks ---
extern "C" void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  procesar_mitad1 = true;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  procesar_mitad2 = true;
}

// =============================================================
// CONFIGURACION DE HARDWARE (ADC, TIM, DMA)
// =============================================================

void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  // Para obtener fs = 300 kHz con reloj APB1 a 72 MHz:
  // (Prescaler+1) * (Period+1) = 72e6 / 300e3 = 240
  // elegimos Prescaler = 23 -> 24; Period = 9 -> 10; 24*10 = 240
  htim3.Init.Prescaler = 23;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
}

void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&hdma_adc1);

  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
}
