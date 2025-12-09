#include <Arduino.h>

//================= CONFIGURACIÓN GENERAL =================//
const uint32_t FS_HZ = 200000;     // Frecuencia de muestreo
const uint16_t N_BLOCK = 512;
const uint16_t N_DMA = N_BLOCK * 2;
const int ADC_MID = 2048;

//================= COEFICIENTES DEL BIQUAD (Q14) =================//
// Chebyshev I, fc=44kHz, BW=7kHz, ganancia≈8
// b = [ 0.7970982   0        -0.7970982 ]
// a = [ 1         -0.33947239   0.80072545 ]

int32_t B0_INT = 13054;
int32_t B1_INT = 0;
int32_t B2_INT = -13054;
int32_t A1_INT = 5560;
int32_t A2_INT = -13114;

//================= ESTRUCTURA BIQUAD =================//
struct Biquad {
  int32_t x1 = 0, x2 = 0;
  int32_t y1 = 0, y2 = 0;

  inline int32_t step(int32_t x0) {
    int64_t acc = 0;

    acc += (int64_t)B0_INT * x0;
    acc += (int64_t)B1_INT * x1;
    acc += (int64_t)B2_INT * x2;
    acc -= (int64_t)A1_INT * y1;
    acc -= (int64_t)A2_INT * y2;

    int32_t y0 = acc >> 14;

    // Protección contra clipping interno
    if (y0 > 20000) y0 = 20000;
    if (y0 < -20000) y0 = -20000;

    x2 = x1;  x1 = x0;
    y2 = y1;  y1 = y0;

    return y0;
  }
};

Biquad filtro;

//================= BUFFERS =================//
volatile uint16_t adc_dma_buf[N_DMA];

struct Frame { uint16_t vin; uint16_t vout; };
Frame tx_buf[N_BLOCK];

volatile bool procesar_mitad1 = false;
volatile bool procesar_mitad2 = false;

//================= HAL =================//
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim3;

//================= PROTOCOLO =================//
const uint8_t MAGIC[4] = {0xA5, 0x5A, 0xA5, 0x5A};


//=========== DECLARACIONES HAL ===========//
extern "C" void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_TIM3_Init(void);


//================= SETUP =================//
void setup() {
  Serial.begin(115200);

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, N_DMA);
  HAL_TIM_Base_Start(&htim3);
}


//================= LOOP =================//
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


//================= PROCESAR =================//
void procesar_bloque(uint16_t offset) {
  for (int i = 0; i < N_BLOCK; i++) {

    uint16_t raw = adc_dma_buf[offset + i];
    int32_t input = (int32_t)raw - ADC_MID;

    int32_t output = filtro.step(input);

    int32_t out_final = output + ADC_MID;

    if (out_final < 0) out_final = 0;
    if (out_final > 4095) out_final = 4095;

    tx_buf[i].vin = raw;
    tx_buf[i].vout = (uint16_t)out_final;
  }
}


//================= ENVÍO =================//
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


//================= INTERRUPCIONES DMA =================//
extern "C" void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  procesar_mitad1 = true;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  procesar_mitad2 = true;
}


//================= CONFIG. DISPOSITIVOS =================//
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
  htim3.Init.Prescaler = 71;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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

