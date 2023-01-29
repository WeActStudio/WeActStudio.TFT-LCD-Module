/* USER CODE BEGIN Header */

/*---------------------------------------
- WeAct Studio Official Link
- taobao: weactstudio.taobao.com
- aliexpress: weactstudio.aliexpress.com
- github: github.com/WeActTC
- gitee: gitee.com/WeAct-TC
- blog: www.weact-tc.cn
---------------------------------------*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "logo.h"
#include "dwt_stm32_delay.h"
#include "touch_xpt2046.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t cmd;
uint16_t tp_result;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  char textbuf[50];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();

  LCD_Init();

  HAL_Delay(100);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, 30);

  tp_dev.init();

  LCD_Clear(BLACK);

  LCD_ShowPicture(0, 0, 240, 40, gImage_logo);

  if(lcddev.id == 0)
    LCD_ShowString(10, 40, (uint8_t *)"2.8 Inch ILI9341 LCD", RED, BLACK, 16, 0);
  else
    LCD_ShowString(10, 40, (uint8_t *)"2.8 Inch ST7789 LCD", RED, BLACK, 16, 0);

  LCD_ShowString(10, 56, (uint8_t *)"LCD_W:", GREEN, BLACK, 16, 0);
  LCD_ShowIntNum(58, 56, LCD_W, 3, WHITE, BLACK, 16);

  LCD_ShowString(90, 56, (uint8_t *)"LCD_H:", BLUE, BLACK, 16, 0);
  LCD_ShowIntNum(138, 56, LCD_H, 3, WHITE, BLACK, 16);

  for (uint8_t j = 0; j < 5; j++)
  {
    LCD_ShowPicture(0, 120 + j * 40, 240, 40, gImage_logo);
  }
  HAL_Delay(1000);
  LCD_Fill(0, 120, 240, 320, BLACK);

  uint32_t tick;
  uint32_t vcc;
  uint16_t vauxin;

  uint32_t brightness;
  uint32_t brightness_lux, brightness_set,brightness_min_set;
  brightness_min_set = 15;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    if (tick % 500 == 0)
//    {
//      LCD_ShowPicture(0, 200, 240, 40, gImage_logo);
//    }
//    if (tick % 1000 == 0)
//    {
//      LCD_Fill(0, 200, 240, 240, BLACK);
//    }
    
    if (tick % 5 == 0)
    {
      if (tp_dev.sta & TP_PRES_DOWN)
      {
        sprintf(textbuf, "Touch X:%5d,Y:%5d", tp_dev.x, tp_dev.y);
        LCD_ShowString(10, 72, (uint8_t *)textbuf, WHITE, BLACK, 16, 0);
        if (tp_dev.x < lcddev.width && tp_dev.y < lcddev.height)
          LCD_DrawPoint(tp_dev.x, tp_dev.y, RED);
      }
    }
    
    if (tick % 500 == 0)
    {
      vcc = TP_Read_Vbat();
      sprintf(textbuf, "VCC Voltage:%4dmV", vcc);
      LCD_ShowString(10, 88, (uint8_t *)textbuf, WHITE, BLACK, 16, 0);
    }
    
    if (tick % 80 == 0)
    {
      vauxin = TP_Read_AUXIN();
      sprintf(textbuf, "AUX AD Value:%4d", vauxin);
      LCD_ShowString(10, 104, (uint8_t *)textbuf, WHITE, BLACK, 16, 0);

      // set brightness
      brightness_lux = vauxin / 100 * 3;
      if (brightness_lux >= (100-brightness_min_set))
        brightness_lux = 100-brightness_min_set;
      brightness_set = 100 - brightness_lux;

      if (brightness > brightness_set)
      {
        if (brightness - brightness_set < 5)
          brightness = brightness_set;
        else
          brightness -= 5;
      }
      else if (brightness < brightness_set)
      {
        if (brightness_set - brightness < 5)
          brightness = brightness_set;
        else
          brightness += 5;
      }
      __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, brightness);
    }

    tp_dev.scan(0);

    tick++;
    HAL_Delay(0);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
