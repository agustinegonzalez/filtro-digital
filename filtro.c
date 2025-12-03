float b[] = {2.98181864f ,-11.92727456f  ,17.89091183f ,-11.92727456f   ,2.98181864f}
float a[] = {1.0f,-1.33293302f , 1.05282113f ,-0.3734322f  , 0.05754151f}

// Variables del estado del filtro
float w[5] = {0}; // orden = 4 → 5 estado
                  //
float iir_filter(float x)
{
    // Filtro IIR Direct Form II Transposed
    float y = b[0]*x + w[0];

    for(int i=1; i<5; i++){
        w[i-1] = b[i]*x + w[i] - a[i]*y;
    }

    return y;
}
uint16_t read_adc()
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}
float x = ((float)read_adc() - 2048.0f) / 2048.0f;
while(1)
{
    float xin = ((float)read_adc() - 2048.0f) / 2048.0f;

    float yout = iir_filter(xin);

    enviar_uart(xin, yout);
}
void enviar_uart(float x, float y)
{
    char buffer[64];
    int n = sprintf(buffer, "%f,%f\n", x, y);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, n, HAL_MAX_DELAY);
}

