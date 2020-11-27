/**
 * 	ADXL345 Program Code
 * 	Make sure the ADXL board has CS tied high
 */
#include "main.h"

#define adxl_address 0x53<<1	// DEVID shifted left 1 bit per HAL_i2c.h
#define X_VAL -5
#define Y_VAL -50
#define Y_VAL_RIGHT 50
#define Z_VAL 70

HAL_StatusTypeDef i2c_status;
HAL_UART_StateTypeDef uart_state;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

uint8_t data_rec[6];
uint8_t chipid = 0;
int8_t count = 0;
int16_t x,y,z;
float xg, yg, zg;
char x_char[3], y_char[3], z_char[3];

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void Sys_Init();
void SystemClock_Config(void);
void adxl_write(uint8_t, uint8_t);
void adxl_read_values(uint8_t);
void adxl_read_address(uint8_t);
void ADXL_Init(void);
void led_on(void);
void led_off(void);
void reset_count(void);
void run_adxl(void);
void play_track(void);
void uart_write(void);
int check_z_val(void);
int check_y_val(void);
int check_y_value_right(void);
int check_x_val(void);

int main(void)
{
  Sys_Init();
  i2c_status = HAL_I2C_IsDeviceReady(&hi2c1, adxl_address, 10, 500);
  uart_state = HAL_UART_GetState(&huart1);
  uart_write();
  while (1)
  {
	  run_adxl();
  }
  return 0;
}

void Sys_Init(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  ADXL_Init();
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;	// per MP3trigger datasheet
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	HAL_GPIO_WritePin(LED_BUILT_IN_GPIO_Port, LED_BUILT_IN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = LED_BUILT_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_BUILT_IN_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = TRIG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = USART_RX_Pin;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = USART_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USART_TX_GPIO_Port, &GPIO_InitStruct);
}

void ADXL_Init(void)
{
	adxl_read_address(0x00); // read the DEVID
	adxl_write(0x31, 0x01);  // data_format range= +- 4g
	adxl_write(0x2d, 0x00);  // reset all bits
	adxl_write(0x2d, 0x08);  // power_cntl measure and wake up 8hz
}

void run_adxl(void)
{
    adxl_read_values(0x32);
    x = ((data_rec[1]<<8)|data_rec[0]);
    y = ((data_rec[3]<<8)|data_rec[2]);
    z = ((data_rec[5]<<8)|data_rec[4]);
    xg = x * .0078;
    yg = y * .0078;
    zg = z * .0078;
    //check_x_val() ? led_on() : led_off();
    if(check_y_val()) play_track();
    //if(check_y_val_right()) led_off();
    //count++;
}

void adxl_write(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, adxl_address, data, 2, 100);
}

void adxl_read_values(uint8_t reg)
{
	HAL_I2C_Mem_Read(&hi2c1, adxl_address, reg, 1, data_rec, 6, 100);
}

void adxl_read_address(uint8_t reg)
{
	HAL_I2C_Mem_Read(&hi2c1, adxl_address, reg, 1, &chipid, 1, 100);
}

void led_on(void)
{
	HAL_GPIO_WritePin(LED_BUILT_IN_GPIO_Port, LED_BUILT_IN_Pin, GPIO_PIN_RESET);
}

void led_off(void)
{
	HAL_GPIO_WritePin(LED_BUILT_IN_GPIO_Port, LED_BUILT_IN_Pin, GPIO_PIN_SET);
}

int check_z_val(void)
{
	return (z < Z_VAL);
}

int check_y_val(void)
{
	return (y < Y_VAL);
}

int check_y_val_right(void)
{
	return (y > Y_VAL_RIGHT);
}

int check_x_val(void)
{
	return (x < X_VAL);
}

void play_track(void)
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, SET);
}

void uart_write(void)
{
	uint8_t cmd = 't';
	uint8_t track = 3;
	HAL_UART_Transmit(&huart1, &cmd, sizeof(cmd), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, &track, sizeof(track), HAL_MAX_DELAY);
	HAL_Delay(5);
}

void reset_count(void)
{
	count = 0;
}

void Error_Handler(void)
{

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
