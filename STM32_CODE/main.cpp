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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid.hpp"
#include "Motors.hpp"
#include "sensors.h"
#include <cstring>
#include <cstdio>
#include <bitset>
#include <cmath>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUF_SIZE 16
#define STOP -999
#define rxD 25
#define txD 25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
static uint8_t rxbuf[RXBUF_SIZE];
volatile int16_t latest_heading;
volatile uint8_t heading_ready = 0;

int32_t heading = 0;

const char* req;

static float ki = 0.0f;
static float kd = 0.0f;
static float kp = 0.0f;

constexpr int16_t MIN_PWM = 150;      // motor stiction floor (30–60 typical)
constexpr int16_t PWM_MAX = 255;

// One-time (outside loop)
int16_t xErrPrev = 0;
int16_t thErrPrev = 0;

bool kp_set = 0, ki_set = 0, kd_set = 0;

static uint8_t sensor_states; 

const static int base_speed = 200; //Set the base speed


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void uart_rx_start(void);
void HAL_UARTex_RxEventCallback(UART_HandleTypeDef *huart3, uint16_t Size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void uart_drain_rx(UART_HandleTypeDef *huart) {

    // Clear error flags defensively (order doesn’t matter here)
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

}


static inline void motors_stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

static inline float wrap180(float a) {
    while (a > 180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}

static inline int16_t theta_from_wrong_axis(float theta_wrong_deg)
{
    float a   = wrap180(theta_wrong_deg);                 // [-180,180]
    float mag = 90.0f - std::min(90.0f, std::fabs(a));    // deviation from vertical [0..90]
    float th  = (a >= 0.0f ? +1.0f : -1.0f) * mag;        // signed small angle
    return static_cast<int16_t>(std::lround(th));
}

static inline int clampi(int v, int lo, int hi) {
    if (v < lo) return lo; if (v > hi) return hi; return v;
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
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  Motors::Channels motor_ch = {
      .left_in1 = TIM_CHANNEL_1,
      .left_in2 = TIM_CHANNEL_2,
      .right_in1 = TIM_CHANNEL_3,
      .right_in2 = TIM_CHANNEL_4};

  sensors::Inputs sensor_inputs = {
      .s0 = GPIO_PIN_0,
      .s1 = GPIO_PIN_1,
      .s2 = GPIO_PIN_2,
      .s3 = GPIO_PIN_3,
      .s4 = GPIO_PIN_4,
      .s5 = GPIO_PIN_5
  };

  Motors motors(&htim1, motor_ch);

  sensors sensors(sensor_inputs);


  uint8_t res_float[4];

  //
  while (!kp_set)
  {
    req = "G kp\n";
    uart_drain_rx(&huart3);
    HAL_UART_Transmit(&huart3, (uint8_t *)req, strlen(req), txD);
    if (HAL_UART_Receive(&huart3, res_float, sizeof(res_float), rxD) == HAL_OK)
    {
      memcpy(&kp, res_float, sizeof(float));
      char msg[48];
      int32_t milli = (int32_t)(kp * 1000.0f);
      int n = snprintf(msg, sizeof(msg), "R kp=%ld.%03ld\r\n",
                       (long)(milli / 1000),
                       (long)labs(milli % 1000));
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, n, txD);
      kp_set = 1;
    }
  }

  while (!ki_set)
  {
    req = "G ki\n";
    uart_drain_rx(&huart3);
    HAL_UART_Transmit(&huart3, (uint8_t *)req, strlen(req), txD);
    if (HAL_UART_Receive(&huart3, res_float, sizeof(res_float), rxD) == HAL_OK)
    {
      memcpy(&ki, res_float, sizeof(float));
      char msg[48];
      int32_t milli = (int32_t)(ki * 1000.0f);
      int n = snprintf(msg, sizeof(msg), "R ki=%ld.%03ld\r\n",
                       (long)(milli / 1000),
                       (long)labs(milli % 1000));
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, n, txD);
      ki_set = 1;
    }
  }

  while (!kd_set)
  {
    req = "G kd\n";
    uart_drain_rx(&huart3);
    HAL_UART_Transmit(&huart3, (uint8_t *)req, strlen(req), txD);
    if (HAL_UART_Receive(&huart3, res_float, sizeof(res_float), rxD) == HAL_OK)
    {
      memcpy(&kd, res_float, sizeof(float));
      char msg[48];
      int32_t milli = (int32_t)(kd * 1000.0f);
      int n = snprintf(msg, sizeof(msg), "R kd=%ld.%03ld\r\n",
                       (long)(milli / 1000),
                       (long)labs(milli % 1000));
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, n, txD);
      kd_set = 1;
    }
  }

  HAL_Delay(5000);



  control::PID tpid(kp, ki, kd, -50.0f, 50.0f);
  control::PID spid(1.0f, 0.0f, 0.0f, -30.0f, 30.0f);
  /* Infinite loop */
  while (1)
  {
      req = "G H\n"; //Get theta from PI camera
      uart_drain_rx(&huart3); //Drain UART buffer
      HAL_UART_Transmit(&huart3, (uint8_t*)req, strlen(req), txD); //Transmit the request over UART

      uint8_t h_rec[2];  // signed 16-bit coming back
      if (HAL_UART_Receive(&huart3, h_rec, sizeof h_rec, rxD) == HAL_OK) 
      {
          // Little-endian: low byte first (matches AB FF -> 0xFFAB -> -85)
          heading = (int16_t)((uint16_t)h_rec[0] | ((uint16_t)h_rec[1] << 8));
  
          char msg[32]; //Prepare response message
          snprintf(msg, sizeof msg, "R: %d\r\n", (int)heading); //Pack the message into a format that can be sent 
          HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), txD); //Transmit uart over uart
      }
      else
      {
          const char err[] = "UART receive failed\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), txD); // use txD here, not rxD
      }

      if(heading == -999){ //PI sents -999 when camera detects no vectors
        const char err[] = "R NO VECTOR/STOP REQUESTED";
        motors.resetIntegral(); //Stop integrating (Depricated)
        motors_stop(); //Stop the motors (this dosnt work for some reason)
        //HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), txD);
      } else{

        // This gets DT for our PID controllers
        static uint32_t last_ms = HAL_GetTick(); 
        uint32_t now_ms = HAL_GetTick();
        float dt = (now_ms - last_ms) * 1e-3f;
        if (dt <= 0.0f) dt = 1e-3f;
        last_ms = now_ms;


        /*
        std::bitset<8> sensorBits = sensors.getValues();
        int16_t x_error = sensors.getShiftErr(sensorBits);

        float u_x = spid.update(x_error, dt, 1);
        int throttle = int(u_x);
        */
 

        

        //For some reason, the heading likes to reference from 180/-1
        if(heading > 90){
          heading = heading - 180;
        } else if(heading < -90){
          heading = 180 - heading;
        }
        bool valid = (heading != -999);
        //float e_norm = (float)heading / float(90.0f);
        
        float u = tpid.update(heading, dt, valid); //Feed the heading (theta ) into the theta PID controller and get U (error)
        int turn = int(u); //Convert u into something usefukl (an int)


        int left_speed = base_speed  + turn; //Add the PID error to the left speed
        int right_speed = base_speed -turn;  //Subtract the PID Turn error from the right speed

        //In theory, the expression for the proper PID is (left speed = base_speed + turn_PID + speed_pid)
        //In theory, the expression for the proper PID is (rsightspeedpeed = base_speed - turn_PID + speed_pid)

        
        left_speed = clampi(left_speed, 150, 250); //Clamp left speed to ranges (MIN PWM is 150 to make wheels spin)
        right_speed = clampi(right_speed, 150, 250);

        motors.setLeft(left_speed); //SET 
        motors.setRight(right_speed);

        char msg[64];
        snprintf(msg, sizeof msg, "R L %d | R %d | h %d \r\n", (int)left_speed, (int)right_speed, int(heading));
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), txD);






        /*
        int16_t pixy_error = motors.pidController(kp, ki, kd, heading, thErrPrev, 0.1f);
        thErrPrev = heading;

        char msg[32];
        snprintf(msg, sizeof msg, "R Error from PID: %d\r\n", (int)pixy_error);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), txD);

        int max_speed = 230;
        int min_speed = 155;
        int base_speed = 175;

        uint8_t left_speed = base_speed + pixy_error;
        uint8_t right_speed = base_speed - pixy_error;

        if(left_speed > max_speed) left_speed = max_speed;
        if(left_speed < min_speed) left_speed = min_speed;
        if(right_speed > max_speed) right_speed = max_speed;
        if(right_speed < min_speed) right_speed = min_speed;

        motors.setLeft(left_speed);
        motors.setRight(right_speed);

        */


        

        /*
        int max_speed = 200;
        int32_t previous_error;
        int16_t correction = motors.pidController(kp, ki, kd, heading, previous_error, 0.1f);
        previous_error = heading;
        uint8_t base_speed = 167;

        uint8_t left_speed = base_speed + correction;
        uint8_t right_speed = base_speed - correction;

        if (left_speed > max_speed) left_speed = max_speed;
        if (right_speed > max_speed) right_speed = max_speed;
        if (left_speed < 0) left_speed = 0;
        if (right_speed < 0) right_speed = 0;

        motors.setLeft(left_speed);
        motors.setRight(right_speed);
        */

        

        /*
          for(int i = 0; i < 255; ++i){
          motors.setLeft(i);
          motors.setRight(i);
          char msg[32];
          snprintf(msg, sizeof msg, "R: %d\r\n", i);
          HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), txD);
          HAL_Delay(100);
        }
        
        */



      }

      // optional small delay to avoid hammering the link
      // HAL_Delay(5);
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 14;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void uart_rx_start(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxbuf, RXBUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size){
  if(huart->Instance != USART3) return;


  if (size == 2) {
      int16_t v = (int16_t)((uint16_t)rxbuf[0] | ((uint16_t)rxbuf[1] << 8));
      latest_heading = v;
      heading_ready = 1;
  } else {
      // Resync policy: ignore or add a tiny state machine to find 2-byte alignment
      // Example: if (size > 2) scan rxbuf for a known header, etc.
  }

  // Re-arm immediately (HAL restarts for you in recent HAL; if not, call uart_rx_start() again)
  // Safe option:
  uart_rx_start();
}



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
