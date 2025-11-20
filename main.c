/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *****************************************************************************
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Stepper pin */
#define IN1 GPIO_PIN_0   // PA0
#define IN2 GPIO_PIN_1   // PA1
#define IN3 GPIO_PIN_4   // PA4
#define IN4 GPIO_PIN_0   // PB0
#define PORT1 GPIOA
#define PORT2 GPIOB

#define DHT11_PORT GPIOC
#define DHT11_PIN  GPIO_PIN_1
/* USER CODE END Includes */
typedef enum {
    CMD_NONE = 0,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_CENTER
} StepCmd_t;

/* USER CODE BEGIN PTD */
static QueueHandle_t xCmdQueue;
//TaskHandle_t xStepperTaskHandle= NULL;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/* DWT microsecond delay */
void DWT_Delay_Init(void);
void delay_us(uint32_t us);

/* DHT11 functions */
void DHT11_SetPinOutput(void);
void DHT11_SetPinInput(void);
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read(void);

/* Stepper helpers */
static void stepMotor(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
static void rotateRight(int steps);
static void rotateLeft(int steps);

/* Tasks */
void vUARTTask(void *pvParameters);
void vStepperTask(void *pvParameters);
void vDHT11Task(void *pvParameters);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */



  DWT_Delay_Init();

      xCmdQueue = xQueueCreate(4, sizeof(StepCmd_t));

      xTaskCreate(vUARTTask, "UART", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
      xTaskCreate(vStepperTask, "STEPPER", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
      xTaskCreate(vDHT11Task, "DHT11", 384, NULL, tskIDLE_PRIORITY + 1, NULL);

      vTaskStartScheduler();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/* UART task: receive commands and push to queue */

void vUARTTask(void *pvParameters)
{
    (void) pvParameters;
    uint8_t rx_buf[32];
    StepCmd_t cmd;
    for (;;)
    {
        memset(rx_buf, 0, sizeof(rx_buf));
        if (HAL_UART_Receive(&huart2, rx_buf, sizeof(rx_buf)-1, 2000) == HAL_OK)
        {
            if (strstr((char*)rx_buf, "RIGHT")) cmd = CMD_RIGHT;
            else if (strstr((char*)rx_buf, "LEFT")) cmd = CMD_LEFT;
            else if (strstr((char*)rx_buf, "CENTER")) cmd = CMD_CENTER;
            else cmd = CMD_NONE;

            if (cmd != CMD_NONE)
            {
                xQueueSend(xCmdQueue, &cmd, portMAX_DELAY);


            }
        }
        else
        {

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* Stepper task: perform rotations on commands */
/* Stepper task: continuous movement based on latest command */
void vStepperTask(void *pvParameters)
{
    (void) pvParameters;
    StepCmd_t cmd = CMD_NONE;
    StepCmd_t newCmd;

    for (;;)
    {
        // Check for new command (non-blocking with short timeout)
        if (xQueueReceive(xCmdQueue, &newCmd, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            cmd = newCmd;  // Update current command
        }

        // Execute one step based on current command
        if (cmd == CMD_LEFT)
        {
            // Single step left
        	stepMotor(1,0,0,0);
        	stepMotor(0,1,0,0);
        	stepMotor(0,0,1,0);
        	stepMotor(0,0,0,1);
        }
        else if (cmd == CMD_RIGHT)
        {
            // Single step right

            stepMotor(0,0,0,1);
            stepMotor(0,0,1,0);
            stepMotor(0,1,0,0);
            stepMotor(1,0,0,0);
        }
        else if (cmd == CMD_CENTER)
        {
            // Stop - do nothing, motor stays idle
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        else
        {
            // No command - stop
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/* DHT11 task: read sensor and send over UART every ~2s */
void vDHT11Task(void *pvParameters)
{
    (void) pvParameters;
    char msg[64];
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, Checksum;
    uint16_t sum;
    for (;;)
    {
        DHT11_Start();
        if (DHT11_Check_Response())
        {
            Rh_byte1   = DHT11_Read();
            Rh_byte2   = DHT11_Read();
            Temp_byte1 = DHT11_Read();
            Temp_byte2 = DHT11_Read();
            Checksum   = DHT11_Read();

            sum = (uint16_t)Rh_byte1 + (uint16_t)Rh_byte2 + (uint16_t)Temp_byte1 + (uint16_t)Temp_byte2;
            if (sum == Checksum)
            {
                snprintf(msg, sizeof(msg), "DHT11: Temp=%dC Hum=%d%%\r\n", Temp_byte1, Rh_byte1);
            }
            else
            {
                snprintf(msg, sizeof(msg), "DHT11: checksum error\r\n");
            }
        }
        else
        {
            snprintf(msg, sizeof(msg), "DHT11: no response\r\n");
        }
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* Stepper functions */
static void stepMotor(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    HAL_GPIO_WritePin(PORT1, IN1, a ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PORT1, IN2, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PORT1, IN3, c ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PORT2, IN4, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(5));
}

static void rotateRight(int steps)
{
    for (int i = 0; i < steps; i++)
    {
        stepMotor(1,0,0,0);
        stepMotor(0,1,0,0);
        stepMotor(0,0,1,0);
        stepMotor(0,0,0,1);
    }
}

static void rotateLeft(int steps)
{
    for (int i = 0; i < steps; i++)
    {
        stepMotor(0,0,0,1);
        stepMotor(0,0,1,0);
        stepMotor(0,1,0,0);
        stepMotor(1,0,0,0);
    }
}

/* DWT microsecond delay */
void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t clk_cycle_start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000UL) * us;

    while ((DWT->CYCCNT - clk_cycle_start) < cycles)
    {
        taskYIELD();   // ← allow other tasks to run
    }
}


/* DHT11 low-level */
void DHT11_SetPinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_SetPinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_Start(void)
{
    DHT11_SetPinOutput();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);
    DHT11_SetPinInput();
}

uint8_t DHT11_Check_Response(void)
{
    uint8_t response = 0;
    uint32_t timeout = 0;

    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
    {
        delay_us(1);
        if (++timeout > 100) return 0;
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET)
    {
        delay_us(1);
        if (++timeout > 200) return 0;
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
    {
        delay_us(1);
        if (++timeout > 200) return 0;
    }

    response = 1;
    return response;
}

uint8_t DHT11_Read(void)
{
    uint8_t i = 0;
    uint8_t result = 0;
    uint32_t timeout;

    for (i = 0; i < 8; i++)
    {
        timeout = 0;
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET)
        {
            delay_us(1);
            if (++timeout > 200) return 0;
        }

        delay_us(40);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
        {
            result |= (1 << (7 - i));
            timeout = 0;
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
            {
                delay_us(1);
                if (++timeout > 200) break;
            }
        }
    }
    return result;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
