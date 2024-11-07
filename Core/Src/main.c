#include "main.h"
#include "usb_device.h"

#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"

// индикатор, что значение температуры выше нормы 
uint8_t flag = 0;

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void measure_task(void const * argument);
void diod_task(void const * argument);

void sht31_write_command(uint16_t cmd);
void sht31_reset();
uint16_t sht31_read_status();
uint8_t crc8(const uint8_t* data, int len);
void sht31_init();
int sht31_read_temp_hum(float* temp, float* humidity);

void measure_task(void const * argument);
void diod_task(void const * argument);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  sht31_init();

  xTaskCreate(measure_task, "Task 1", 1000, NULL, 1, NULL );
  xTaskCreate(diod_task, "Task 2", 1000, NULL, 1, NULL );
  vTaskStartScheduler();

  while (1)
  {

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void sht31_write_command(uint16_t cmd) {
  uint8_t buf[2];
  buf[0] = (cmd >> 8);
  buf[1] = (cmd & 0xFF);
  HAL_I2C_Master_Transmit(&hi2c1, SHT31_DEFAULT_ADDR, buf, 2, 1);
}

void sht31_reset() {
  sht31_write_command(SHT31_SOFTRESET);
  HAL_Delay(10);
}

uint16_t sht31_read_status() {
  sht31_write_command(SHT31_READSTATUS);
  uint8_t val[1];
  HAL_I2C_Master_Receive(&hi2c1, SHT31_DEFAULT_ADDR, val, 1, 1);
  uint16_t stat = (val[0] << 8);
  HAL_I2C_Master_Receive(&hi2c1, SHT31_DEFAULT_ADDR, val, 1, 1);
  stat |= val[0];
  return stat;
}

uint8_t crc8(const uint8_t* data, int len) {
  uint8_t poly = 0x31;
  uint8_t crc = 0xFF;

  for ( int j = len; j; --j ) {
    crc ^= *data++;

    for ( int i = 8; i; --i ) {
        crc = (crc & 0x80) ? (crc << 1) ^ poly : (crc << 1);
    }
  }
  return crc;
}

void sht31_init() {
  sht31_reset();
  sht31_read_status();
}

int sht31_read_temp_hum(float* temp, float* humidity) {
  uint8_t read_buf[6];
  sht31_write_command(SHT31_MEAS_HIGHREP);
  HAL_Delay(500);
  HAL_I2C_Master_Receive(&hi2c1, SHT31_DEFAULT_ADDR, read_buf, 6, 1);

  uint16_t ST, SRH;
  ST = ((read_buf[0] << 8) | read_buf[1]);

  if (read_buf[2] != crc8((uint8_t *) read_buf, 2)) {
    return 1;
  }

  SRH = ((read_buf[3] << 8) | read_buf[4]);

  if (read_buf[5] != crc8((uint8_t *) read_buf + 3, 2)) {
    return 1;
  }

  double s = ST;
  s *= 175;
  s /= 0xFFFF;
  s = -45 + s;
  *temp = s;
  
  double sh = SRH;
  sh *= 100;
  sh /= 0xFFFF;
  *humidity = sh;
  return 0;
}

void measure_task(void const * argument) {
  float temp = 0, hum = 0;
  char buff[127];
  const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
  for(;;) {
    sht31_read_temp_hum(&temp, &hum);
    if (temp >= 30 || hum >= 70) flag = 1;
    else flag = 0;
    snprintf(buff, 127, "temp = %d hum = %d\r\n", (int)temp * 10, (int)hum * 10);
    // передача по USB
    CDC_Transmit_FS((uint8_t *)buff, strlen(buff));
    vTaskDelay( xDelay );
  }
}

void diod_task(void const * argument) {
  int ms_delay = 500;

  for(;;) {
    if (flag) ms_delay = 150;
    else ms_delay = 500;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(ms_delay);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
