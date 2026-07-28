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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ADC-Grenzwerte */
#define ADC_MIN_VALUE           0U
#define ADC_CENTER_VALUE        1961U
#define ADC_MAX_VALUE           3921U

/* Totzone im normierten Bereich */
#define JOYSTICK_DEAD_ZONE      5

/* Motorwerte */
#define MOTOR_MIN_POWER         (-127)
#define MOTOR_MAX_POWER         127

/* UART-Protokoll */
#define MOTOR_ADDRESS           0xC5U
#define MOTOR_POWER_COMMAND     0x25U
#define MOTOR_STOP_COMMAND      0x26U

#define MOTOR_FRAME_SIZE        5U

/* Sendeintervall */
#define MOTOR_SEND_PERIOD_MS    30U
#define UART_SEND_TIMEOUT_MS    10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint32_t current_time = 0U;
volatile uint32_t main_loop_counter = 0U;

/*
 * ADC-DMA-Puffer:
 *
 * adc_raw[0] = X-Achse
 * adc_raw[1] = Y-Achse
 * adc_raw[2] = Z-Achse
 */
volatile uint16_t adc_raw[3] =
{
    ADC_CENTER_VALUE,
    ADC_CENTER_VALUE,
    ADC_CENTER_VALUE
};

/*
 * Normierte Joystickwerte:
 * -127 bis +127
 */
volatile int16_t joystick_x = 0;
volatile int16_t joystick_y = 0;
volatile int16_t joystick_z = 0;

/*
 * Berechnete Motorwerte:
 * -127 bis +127
 */
volatile int16_t motor_value_A = 0;
volatile int16_t motor_value_B = 0;

/*
 * Zuletzt gesendete UART-Frames.
 * Diese Arrays können ebenfalls in Live Expressions
 * betrachtet werden.
 */
uint8_t tx_frame_A[MOTOR_FRAME_SIZE] = {0U};
uint8_t tx_frame_B[MOTOR_FRAME_SIZE] = {0U};

/*
 * Test- und Diagnosevariablen.
 */
volatile uint32_t motor_send_counter = 0U;
volatile uint32_t uart_error_counter_A = 0U;
volatile uint32_t uart_error_counter_B = 0U;

static uint32_t last_motor_send_time = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static int16_t scale_adc_to_motor(uint16_t adc_value);

static void update_joystick_values(void);
static void calculate_motor_values(void);

static uint8_t calculate_checksum(
    uint8_t byte_0,
    uint8_t byte_1,
    uint8_t byte_2,
    uint8_t byte_3);

static void build_frame_A(void);
static void build_frame_B(void);

static void send_frame_A(void);
static void send_frame_B(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int16_t scale_adc_to_motor(uint16_t adc_value)
{
    int32_t adc;
    int32_t adc_min;
    int32_t adc_center;
    int32_t adc_max;
    int32_t result;

    adc        = (int32_t)adc_value;
    adc_min    = (int32_t)ADC_MIN_VALUE;
    adc_center = (int32_t)ADC_CENTER_VALUE;
    adc_max    = (int32_t)ADC_MAX_VALUE;

    /*
     * Untere Hälfte:
     * ADC_MIN_VALUE bis ADC_CENTER_VALUE
     * wird auf -127 bis 0 skaliert.
     */
    if (adc < adc_center)
    {
        if (adc <= adc_min)
        {
            result = MOTOR_MIN_POWER;
        }
        else
        {
            result =
                -((adc_center - adc) * MOTOR_MAX_POWER) /
                 (adc_center - adc_min);
        }
    }

    /*
     * Obere Hälfte:
     * ADC_CENTER_VALUE bis ADC_MAX_VALUE
     * wird auf 0 bis +127 skaliert.
     */
    else
    {
        if (adc >= adc_max)
        {
            result = MOTOR_MAX_POWER;
        }
        else
        {
            result =
                ((adc - adc_center) * MOTOR_MAX_POWER) /
                (adc_max - adc_center);
        }
    }

    /*
     * Kleine Werte um die Mittelstellung auf null setzen.
     * Die Grenzen -5 und +5 werden ebenfalls eingeschlossen.
     */
    if ((result >= -JOYSTICK_DEAD_ZONE) &&
        (result <= JOYSTICK_DEAD_ZONE))
    {
        result = 0;
    }

    return (int16_t)result;
}

static void update_joystick_values(void)
{
    joystick_x = scale_adc_to_motor(adc_raw[0]);
    joystick_y = scale_adc_to_motor(adc_raw[1]);
    joystick_z = scale_adc_to_motor(adc_raw[2]);
}

static void calculate_motor_values(void)
{
    int16_t absolute_x;
    int16_t reduced_power;

    /*
     * Betrag der Lenkauslenkung bestimmen.
     */
    if (joystick_x < 0)
    {
        absolute_x = -joystick_x;
    }
    else
    {
        absolute_x = joystick_x;
    }

    if (absolute_x > MOTOR_MAX_POWER)
    {
        absolute_x = MOTOR_MAX_POWER;
    }

    /*
     * Reduzierte Leistung berechnen:
     *
     * X =   0: 100 Prozent des Schubs
     * X =  64: ungefähr 50 Prozent des Schubs
     * X = 127: 0 Prozent des Schubs
     */
    reduced_power =
        (int16_t)(
            ((int32_t)joystick_y *
             (MOTOR_MAX_POWER - absolute_x)) /
            MOTOR_MAX_POWER);

    /*
     * Kein Vorwärts- oder Rückwärtsschub:
     * Beide Motoren stehen, unabhängig von X.
     */
    if (joystick_y == 0)
    {
        motor_value_A = 0;
        motor_value_B = 0;
    }

    /*
     * Quadrant: vorwärts und rechts
     *
     * Motor A ist außen und behält den Schub.
     * Motor B ist innen und wird reduziert.
     */
    else if ((joystick_y > 0) &&
             (joystick_x >= 0))
    {
        motor_value_A = joystick_y;
        motor_value_B = reduced_power;
    }

    /*
     * Quadrant: vorwärts und links
     *
     * Motor A ist innen und wird reduziert.
     * Motor B ist außen und behält den Schub.
     */
    else if ((joystick_y > 0) &&
             (joystick_x < 0))
    {
        motor_value_A = reduced_power;
        motor_value_B = joystick_y;
    }

    /*
     * Quadrant: rückwärts und rechts
     *
     * Beide Werte sind negativ.
     * Motor A wird betragsmäßig reduziert.
     */
    else if ((joystick_y < 0) &&
             (joystick_x >= 0))
    {
        motor_value_A = reduced_power;
        motor_value_B = joystick_y;
    }

    /*
     * Quadrant: rückwärts und links
     *
     * Beide Werte sind negativ.
     * Motor B wird betragsmäßig reduziert.
     */
    else
    {
        motor_value_A = joystick_y;
        motor_value_B = reduced_power;
    }
}

static uint8_t calculate_checksum(
    uint8_t byte_0,
    uint8_t byte_1,
    uint8_t byte_2,
    uint8_t byte_3)
{
    uint8_t checksum;

    checksum = byte_0;
    checksum ^= byte_1;
    checksum ^= byte_2;
    checksum ^= byte_3;

    return checksum & 0x7FU;
}

static void build_frame_A(void)
{
    uint8_t direction;
    uint8_t power;

    tx_frame_A[0] = MOTOR_ADDRESS;

    if (motor_value_A == 0)
    {
        tx_frame_A[1] = MOTOR_STOP_COMMAND;
        tx_frame_A[2] = 0U;
        tx_frame_A[3] = 0U;
    }
    else
    {
        if (motor_value_A > 0)
        {
            direction = 1U;
            power = (uint8_t)motor_value_A;
        }
        else
        {
            direction = 0U;
            power = (uint8_t)(-motor_value_A);
        }

        tx_frame_A[1] = MOTOR_POWER_COMMAND;
        tx_frame_A[2] = direction;
        tx_frame_A[3] = power;
    }

    tx_frame_A[4] = calculate_checksum(
        tx_frame_A[0],
        tx_frame_A[1],
        tx_frame_A[2],
        tx_frame_A[3]);
}

static void build_frame_B(void)
{
    uint8_t direction;
    uint8_t power;

    tx_frame_B[0] = MOTOR_ADDRESS;

    if (motor_value_B == 0)
    {
        tx_frame_B[1] = MOTOR_STOP_COMMAND;
        tx_frame_B[2] = 0U;
        tx_frame_B[3] = 0U;
    }
    else
    {
        if (motor_value_B > 0)
        {
            direction = 1U;
            power = (uint8_t)motor_value_B;
        }
        else
        {
            direction = 0U;
            power = (uint8_t)(-motor_value_B);
        }

        tx_frame_B[1] = MOTOR_POWER_COMMAND;
        tx_frame_B[2] = direction;
        tx_frame_B[3] = power;
    }

    tx_frame_B[4] = calculate_checksum(
        tx_frame_B[0],
        tx_frame_B[1],
        tx_frame_B[2],
        tx_frame_B[3]);
}

static void send_frame_A(void)
{
    HAL_StatusTypeDef uart_result;

    /*
     * RS485-Sendebetrieb aktivieren:
     *
     * DE  = High
     * /RE = High
     */
    HAL_GPIO_WritePin(
        MotorA_DE_GPIO_Port,
        MotorA_DE_Pin,
        GPIO_PIN_SET);

    HAL_GPIO_WritePin(
        MotorA_NRE_GPIO_Port,
        MotorA_NRE_Pin,
        GPIO_PIN_SET);

    uart_result = HAL_UART_Transmit(
        &huart1,
        tx_frame_A,
        MOTOR_FRAME_SIZE,
        UART_SEND_TIMEOUT_MS);

    /*
     * Wieder in den Empfangsbetrieb wechseln:
     *
     * DE  = Low
     * /RE = Low
     */
    HAL_GPIO_WritePin(
        MotorA_DE_GPIO_Port,
        MotorA_DE_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        MotorA_NRE_GPIO_Port,
        MotorA_NRE_Pin,
        GPIO_PIN_RESET);

    if (uart_result != HAL_OK)
    {
        uart_error_counter_A++;
    }
}

static void send_frame_B(void)
{
    HAL_StatusTypeDef uart_result;

    HAL_GPIO_WritePin(
        MotorB_DE_GPIO_Port,
        MotorB_DE_Pin,
        GPIO_PIN_SET);

    HAL_GPIO_WritePin(
        MotorB_NRE_GPIO_Port,
        MotorB_NRE_Pin,
        GPIO_PIN_SET);

    uart_result = HAL_UART_Transmit(
        &huart3,
        tx_frame_B,
        MOTOR_FRAME_SIZE,
        UART_SEND_TIMEOUT_MS);

    HAL_GPIO_WritePin(
        MotorB_DE_GPIO_Port,
        MotorB_DE_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        MotorB_NRE_GPIO_Port,
        MotorB_NRE_Pin,
        GPIO_PIN_RESET);

    if (uart_result != HAL_OK)
    {
        uart_error_counter_B++;
    }
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
  MX_DMA_Init();
  MX_ADC4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /*
   * ADC vor dem Start kalibrieren.
   */
  if (HAL_ADCEx_Calibration_Start(
          &hadc4,
          ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }

  /*
   * ADC4 mit zirkulärem DMA starten.
   */
  if (HAL_ADC_Start_DMA(
          &hadc4,
          (uint32_t *)(uintptr_t)adc_raw,
          3U) != HAL_OK)
  {
      Error_Handler();
  }

  /*
   * RS485 zunächst auf Empfang stellen.
   */
  HAL_GPIO_WritePin(
      MotorA_DE_GPIO_Port,
      MotorA_DE_Pin,
      GPIO_PIN_RESET);

  HAL_GPIO_WritePin(
      MotorA_NRE_GPIO_Port,
      MotorA_NRE_Pin,
      GPIO_PIN_RESET);

  HAL_GPIO_WritePin(
      MotorB_DE_GPIO_Port,
      MotorB_DE_Pin,
      GPIO_PIN_RESET);

  HAL_GPIO_WritePin(
      MotorB_NRE_GPIO_Port,
      MotorB_NRE_Pin,
      GPIO_PIN_RESET);

  last_motor_send_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  main_loop_counter++;

      /*
       * Aktuelle DMA-Werte normieren.
       * Diese Berechnung läuft kontinuierlich.
       */
      update_joystick_values();

      /*
       * Motorwerte kontinuierlich berechnen.
       */
      calculate_motor_values();

      /*
       * Aktuelle Zeit erfassen.
       */
      current_time = HAL_GetTick();

      /*
       * Nur die UART-Übertragung findet alle 30 ms statt.
       */
      if ((uint32_t)(current_time - last_motor_send_time) >=
          MOTOR_SEND_PERIOD_MS)
      {
          last_motor_send_time = current_time;

          build_frame_A();
          build_frame_B();

          send_frame_A();
          send_frame_B();

          motor_send_counter++;
      }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
