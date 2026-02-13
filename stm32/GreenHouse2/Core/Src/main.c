/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// آدرس آی‌سی نمایشگر OLED در پروتئوس معمولاً 0x78 است
#define SSD1306_I2C_ADDR 0x78

// تعریف پین سنسور دما DS18B20 (فرض بر پایه PA0)
#define DS18B20_PORT GPIOA
#define DS18B20_PIN  GPIO_PIN_0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float temperature = 0;
uint16_t ldr_value = 0;    // مقدار نور
uint16_t soil_value = 0;   // مقدار رطوبت خاک (پتانسیومتر)
uint8_t system_mode = 0;   // 0: AUTO, 1: MANUAL
uint8_t door_open = 0;     // وضعیت درب
char msg[50];              // برای ارسال پیام‌های سریال
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read_Sensors(void);
void Control_Logic(void);
void Update_Display(void);
void Process_Serial_Commands(void);

// توابع کمکی سنسور دما DS18B20
void delay_us(uint32_t us);
uint8_t DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// تاخیر میکروثانیه برای سنسور دما
void delay_us(uint32_t us) {
    uint32_t count = us * 8; // تقریب برای 72MHz
    while(count--);
}

// تابع خواندن سنسور دما (ساده شده برای پروتئوس)
float Get_Temperature(void) {
    uint8_t low, high;
    uint16_t temp_raw;
    if (DS18B20_Start()) {
        DS18B20_Write(0xCC); // Skip ROM
        DS18B20_Write(0x44); // Convert T
        HAL_Delay(10);        // در واقعیت بیشتر است، در پروتئوس کمتر
        DS18B20_Start();
        DS18B20_Write(0xCC);
        DS18B20_Write(0xBE); // Read Scratchpad
        low = DS18B20_Read();
        high = DS18B20_Read();
        temp_raw = (high << 8) | low;
        return (float)temp_raw / 16.0;
    }
    return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // فعال‌سازی PWM گرمایش
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // فعال‌سازی PWM نور

  sprintf(msg, "System Initialized...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Read_Sensors();            // خواندن تمام سنسورها
    Process_Serial_Commands(); // بررسی دستورات سریال در حالت دستی
    Control_Logic();           // اعمال منطق (فن، پمپ، گرمایش)
    Update_Display();          // بروزرسانی OLED و سریال

    HAL_Delay(500);            // نرخ بروزرسانی نیم ثانیه
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Read_Sensors(void) {
    // خواندن دما
    temperature = Get_Temperature();

    // اصلاح خواندن ADC: به جای کانفیگ مجدد، فقط استارت کن
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        ldr_value = HAL_ADC_GetValue(&hadc1); // مقدار اول (نور)
    }
    if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        soil_value = HAL_ADC_GetValue(&hadc1); // مقدار دوم (رطوبت)
    }
    HAL_ADC_Stop(&hadc1);

    // خواندن دکمه‌ها (PB0 و PB1)
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
        HAL_Delay(200); // Debounce
        system_mode = !system_mode;
    }
    door_open = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
}

void Control_Logic(void) {
    if (system_mode == 0) { // AUTO
        // فن روی PA3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (temperature > 30.0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // پمپ روی PA4
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (soil_value < 1500) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // گرمایش PWM روی PA6 (تایمر 3 کانال 1)
        if (temperature < 20.0) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 900);
        else __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

        // نور PWM روی PA7 (تایمر 3 کانال 2)
        if (ldr_value < 1000) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 900);
        else __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}

void Process_Serial_Commands(void) {
    if (system_mode == 1) { // فقط در حالت MANUAL
        uint8_t rx_data;
        if (HAL_UART_Receive(&huart1, &rx_data, 1, 10) == HAL_OK) {
            if (rx_data == 'F') HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3); // Toggle Fan
            if (rx_data == 'P') HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4); // Toggle Pump
        }
    }
}

void Update_Display(void) {
    // ارسال اطلاعات به پورت سریال برای مانیتورینگ
    sprintf(msg, "Temp:%.1f L:%d S:%d Mode:%s Door:%s\r\n",
            temperature, ldr_value, soil_value,
            system_mode ? "MAN" : "AUTO",
            door_open ? "OPEN" : "CLOSE");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

    // در اینجا توابع OLED فراخوانی می‌شوند (بسته به کتابخانه‌ای که در پروتئوس دارید)
    // به دلیل محدودیت فضا، کد کامل درایور OLED SSD1306 در اینجا نیامده اما ساختار آماده است.
}

// توابع پایه برای راه اندازی پروتکل 1-Wire سنسور دما
uint8_t DS18B20_Start(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    delay_us(480);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
    delay_us(80);

    if (!(HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN))) {
        delay_us(400);
        return 1;
    }
    return 0;
}

void DS18B20_Write(uint8_t data) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);

    for (int i=0; i<8; i++) {
        if ((data & (1<<i)) != 0) {
            HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
            delay_us(1);
            HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
            delay_us(60);
        } else {
            HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
            delay_us(60);
            HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
            delay_us(1);
        }
    }
}

uint8_t DS18B20_Read(void) {
    uint8_t value = 0;
    for (int i=0; i<8; i++) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = DS18B20_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);

        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
        delay_us(2);

        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);
        if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)) value |= (1<<i);
        delay_us(60);
    }
    return value;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
// بقیه فایل به صورت خودکار توسط CubeMX مدیریت می‌شود...
