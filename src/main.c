/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//----- STATE -----
#define STATE_BREAK  0x01
#define STATE_RUN    0x02
//----- STATE -----

//----- DFrobot 4WD -----
#define GPIOPWMFront           GPIOB
#define GPIOPWMRear            GPIOA
#define GPIODir                GPIOC

#define GPIO_PIN_PWMFrontL    GPIO_PIN_0
#define GPIO_PIN_PWMFrontR    GPIO_PIN_1
#define GPIO_PIN_PWMRearL     GPIO_PIN_7
#define GPIO_PIN_PWMRearR     GPIO_PIN_6
#define GPIO_PIN_DirFrontLO   GPIO_PIN_0
#define GPIO_PIN_DirFrontLI   GPIO_PIN_1
#define GPIO_PIN_DirFrontRO   GPIO_PIN_2
#define GPIO_PIN_DirFrontRI   GPIO_PIN_3
#define GPIO_PIN_DirRearLO    GPIO_PIN_4
#define GPIO_PIN_DirRearLI    GPIO_PIN_5
#define GPIO_PIN_DirRearRO    GPIO_PIN_7
#define GPIO_PIN_DirRearRI    GPIO_PIN_6

#define HIGH  GPIO_PIN_SET
#define LOW   GPIO_PIN_RESET
//----- DFrobot 4WD -----
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//----- UART -----
HAL_StatusTypeDef urx_status;
uint8_t utx[8], urx[8];
//----- UART -----

//----- IR sensor -----
int IRsensor_OpTime = 30;     /* 30 ms 주기로 IR 센서 샘플링 */
int tim4_IRsensor_OpCnt;
GPIO_PinState IRsensor_pa8, IRsensor_pb4, IRsensor_pb5, IRsensor_pb10;
uint8_t IRsensor_value;
//----- IR sensor -----

//----- HC-SR04 -----
int    HCSR04_OpTime          = 500;    /* 측정 간격 500 ms */
double HCSR04_VELOCITY_PARAM  = 0.017;  /* 340 m/s ÷ 2(왕복) = 0.017 cm/µs */
double HCSR04_TIMER_PERIOD    = 4.0;    /* TIM2 틱 주기 4 µs */
double HCSR04_duration, HCSR04_distance;

int  tim2_HCSR04_OpCnt;
char tim2_HCSR04_Start;
char tim2_HCSR04_TrigHigh, tim2_HCSR04_TrigLow;
char tim2_HCSR04_EchoState;
uint8_t tim2_EchoSample;
int  tim2_EchoCnt;
char tim2_EchoRising, tim2_EchoFalling;
//----- HC-SR04 -----

//----- DFrobot 4WD -----
uint32_t pwm_percent_FL;
uint32_t pwm_percent_FR;
uint32_t pwm_percent_RL;
uint32_t pwm_percent_RR;
//----- DFrobot 4WD -----

//----- STATE -----
uint8_t uState;
//----- STATE -----

//----- GPIO EXTI PC13 -----
char chPC13_Rising;
//----- GPIO EXTI PC13 -----

//----- LD2 toggle (PA5) -----
uint32_t LD2_toggle_BREAK  = 900;   /* STATE_BREAK: 900 ms 느린 점멸 */
uint32_t LD2_toggle_RUN    = 300;   /* STATE_RUN:   300 ms 빠른 점멸 */
uint32_t LD2_toggle_OpTime = 900;
uint32_t LD2_toggle_cnt;
//----- LD2 toggle (PA5) -----
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --------------------------------------------------------------------------
 * UART: printf 리디렉션 (최종 빌드에서 HAL_UART_Transmit 비활성화)
 * -------------------------------------------------------------------------- */
PUTCHAR_PROTOTYPE
{
    //HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/* --------------------------------------------------------------------------
 * IR 센서: 4채널 GPIO 읽기 → 1바이트 인코딩
 *   IRsensor_value = (PB10<<3) | (PB5<<2) | (PB4<<1) | (PA8<<0)
 *   검은 선 = 0, 흰 바탕 = 1
 * -------------------------------------------------------------------------- */
void IRsensor_Read(void)
{
    IRsensor_value = 0;

    IRsensor_pa8  = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
    IRsensor_pb4  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
    IRsensor_pb5  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
    IRsensor_pb10 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

    IRsensor_value = (IRsensor_pb10 << 3)
                   + (IRsensor_pb5  << 2)
                   + (IRsensor_pb4  << 1)
                   + (IRsensor_pa8  << 0);

    //printf("ir(%d,%d,%d,%d)=%02x\r\n",
    //        IRsensor_pb10, IRsensor_pb5, IRsensor_pb4, IRsensor_pa8,
    //        IRsensor_value);
}

/* --------------------------------------------------------------------------
 * 모터 드라이버: TIM3 PWM 채널 시작/정지
 * -------------------------------------------------------------------------- */
void Motor_PWM_Start(void)
{
    HAL_StatusTypeDef tim3_STch1, tim3_STch2, tim3_STch3, tim3_STch4;

    tim3_STch1 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    if (HAL_OK == tim3_STch1) { printf("---tim3_STch1=%d---OK\r\n",  tim3_STch1); }
    else                      { printf("---tim3_STch1=%d---err\r\n", tim3_STch1); }

    tim3_STch2 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    if (HAL_OK == tim3_STch2) { printf("---tim3_STch2=%d---OK\r\n",  tim3_STch2); }
    else                      { printf("---tim3_STch2=%d---err\r\n", tim3_STch2); }

    tim3_STch3 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    if (HAL_OK == tim3_STch3) { printf("---tim3_STch3=%d---OK\r\n",  tim3_STch3); }
    else                      { printf("---tim3_STch3=%d---err\r\n", tim3_STch3); }

    tim3_STch4 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    if (HAL_OK == tim3_STch4) { printf("---tim3_STch4=%d---OK\r\n",  tim3_STch4); }
    else                      { printf("---tim3_STch4=%d---err\r\n", tim3_STch4); }
}

void Motor_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}

/* CCR = (ARR * duty_percent) / 100 */
void Motor_Change_Speed(uint32_t pwm_value_FL, uint32_t pwm_value_FR,
                        uint32_t pwm_value_RL, uint32_t pwm_value_RR)
{
    htim3.Instance->CCR3 = (htim3.Instance->ARR * pwm_value_FL) / 100; /* CH3 → FL */
    htim3.Instance->CCR4 = (htim3.Instance->ARR * pwm_value_FR) / 100; /* CH4 → FR */
    htim3.Instance->CCR2 = (htim3.Instance->ARR * pwm_value_RL) / 100; /* CH2 → RL */
    htim3.Instance->CCR1 = (htim3.Instance->ARR * pwm_value_RR) / 100; /* CH1 → RR */
}

/* DIR_FRONT: IN=HIGH, OUT=LOW → 전진
   DIR_BACK:  IN=LOW,  OUT=HIGH → 후진  */
typedef enum {
    DIR_BACK  = GPIO_PIN_RESET,
    DIR_FRONT
} Motor_Dir_t;

void Motor_FrontL_Direction(Motor_Dir_t dir)
{
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontLI,  dir);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontLO, !dir);
}

void Motor_FrontR_Direction(Motor_Dir_t dir)
{
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontRI,  dir);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontRO, !dir);
}

void Motor_RearL_Direction(Motor_Dir_t dir)
{
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearLI,  dir);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearLO, !dir);
}

void Motor_RearR_Direction(Motor_Dir_t dir)
{
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearRI,  dir);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearRO, !dir);
}

/* 음수 PWM → 해당 바퀴 후진 (uint32_t 템플릿을 int로 확장하여 역전 가능) */
void Motor_Forwart_Start(int pwm_FL, int pwm_FR, int pwm_RL, int pwm_RR)
{
    if (pwm_FL < 0) { Motor_FrontL_Direction(DIR_BACK);  pwm_FL = -pwm_FL; }
    else            { Motor_FrontL_Direction(DIR_FRONT); }

    if (pwm_FR < 0) { Motor_FrontR_Direction(DIR_BACK);  pwm_FR = -pwm_FR; }
    else            { Motor_FrontR_Direction(DIR_FRONT); }

    if (pwm_RL < 0) { Motor_RearL_Direction(DIR_BACK);   pwm_RL = -pwm_RL; }
    else            { Motor_RearL_Direction(DIR_FRONT);  }

    if (pwm_RR < 0) { Motor_RearR_Direction(DIR_BACK);   pwm_RR = -pwm_RR; }
    else            { Motor_RearR_Direction(DIR_FRONT);  }

    Motor_Change_Speed(pwm_FL, pwm_FR, pwm_RL, pwm_RR);
}

void Motor_Halt(void)
{
    Motor_Change_Speed(0, 0, 0, 0);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontLI, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontLO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontRI, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontRO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearLI,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearLO,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearRI,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearRO,  GPIO_PIN_RESET);
    printf("halt\r\n");
}

/* --------------------------------------------------------------------------
 * EXTI 콜백: PC13 (사용자 버튼 B1) Rising Edge → 주행 시작 트리거
 * -------------------------------------------------------------------------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        chPC13_Rising = 'y';
        printf("HAL_GPIO_EXTI_Callback chPC13_Rising=%c\r\n", chPC13_Rising);
    }
}

/* --------------------------------------------------------------------------
 * 타이머 인터럽트 콜백
 *   TIM2 (4 µs) : HC-SR04 측정 상태 머신
 *   TIM4 (1 ms) : IR 센서 30 ms 폴링 + LD2 LED 토글
 * -------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ---- TIM2: HC-SR04 상태 머신 ---- */
    if (htim->Instance == htim2.Instance) {

        tim2_HCSR04_OpCnt += 1;

        /* 1단계: 측정 간격 대기 (500 ms = 125,000 틱) */
        if ('n' == tim2_HCSR04_Start) {
            if (((HCSR04_OpTime * 1000) / 4) < tim2_HCSR04_OpCnt) {
                tim2_HCSR04_OpCnt    = 0;
                tim2_HCSR04_Start    = 'y';
                tim2_HCSR04_TrigHigh = 'y';
            }
        }

        if ('y' == tim2_HCSR04_Start) {

            /* 2단계: TRIG HIGH 출력 (PC11) */
            if ('y' == tim2_HCSR04_TrigHigh) {
                tim2_HCSR04_TrigHigh = 'n';
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
                tim2_HCSR04_TrigLow  = 'y';
            }

            /* 3단계: 100 µs 후 TRIG LOW (25 틱) */
            if ('y' == tim2_HCSR04_TrigLow) {
                if ((100 / 4) < tim2_HCSR04_OpCnt) {
                    tim2_HCSR04_OpCnt     = 0;
                    tim2_HCSR04_TrigLow   = 'n';
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
                    tim2_EchoSample       = 0x00;
                    tim2_EchoCnt          = 0;
                    tim2_EchoRising       = 'n';
                    tim2_EchoFalling      = 'n';
                    tim2_HCSR04_EchoState = 'y';
                }
            }

            /* 4단계: Echo 계측 (4비트 시프트 레지스터 노이즈 필터) */
            if ('y' == tim2_HCSR04_EchoState) {

                /* PC10 샘플링: 0x03=Rising Edge, 0x0F=HIGH, 0x0C=Falling Edge */
                tim2_EchoSample <<= 1;
                tim2_EchoSample  &= 0x0F;
                if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)) {
                    tim2_EchoSample += 1;
                }

                if (0x03 == tim2_EchoSample) { tim2_EchoRising  = 'y'; }
                if (0x0F == tim2_EchoSample) { tim2_EchoCnt     += 1;  }
                if (0x0C == tim2_EchoSample) { tim2_EchoFalling = 'y'; }

                /* 타임아웃: 30 ms = 7,500 틱 */
                if (((30 * 1000) / 4) < tim2_HCSR04_OpCnt) {
                    printf("HCSR04 timerout\r\n");
                    tim2_HCSR04_OpCnt     = 0;
                    tim2_HCSR04_EchoState = 'n';
                    tim2_HCSR04_Start     = 'n';
                }

                /* Falling Edge 감지 → 거리 계산 */
                if ('y' == tim2_EchoFalling) {
                    tim2_HCSR04_OpCnt = 0;
                    tim2_EchoCnt     += 3;   /* 엣지 감지 지연 보정 */
                    HCSR04_duration   = HCSR04_TIMER_PERIOD * (double)tim2_EchoCnt;
                    HCSR04_distance   = HCSR04_VELOCITY_PARAM * HCSR04_duration;
                    tim2_HCSR04_EchoState = 'n';
                    tim2_HCSR04_Start     = 'n';
                    //printf("dist=%d.%2d\r\n",
                    //        (int)HCSR04_distance,
                    //        (int)((HCSR04_distance - (int)HCSR04_distance) * 100));
                }
            }
        }
    }

    /* ---- TIM4: IR 센서 폴링 + LD2 LED 토글 ---- */
    if (htim->Instance == htim4.Instance) {

        /* IR 센서: 30 ms 주기 샘플링 */
        tim4_IRsensor_OpCnt += 1;
        if (IRsensor_OpTime < tim4_IRsensor_OpCnt) {
            tim4_IRsensor_OpCnt = 0;
            IRsensor_Read();
        }

        /* LD2 LED: 상태에 따라 900 ms / 300 ms 토글 */
        LD2_toggle_cnt += 1;
        if (LD2_toggle_OpTime < LD2_toggle_cnt) {
            LD2_toggle_cnt = 0;
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
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
  HAL_StatusTypeDef tim2_it_start;
  HAL_StatusTypeDef tim4_it_start;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  printf("******* Project: JBK-DFrobot-AutonomousDriving *******\r\n");

  /* TIM2 인터럽트 시작 (HC-SR04, 4 µs 주기) */
  tim2_it_start = HAL_TIM_Base_Start_IT(&htim2);
  if (HAL_OK == tim2_it_start) { printf("HAL_TIM_Base_Start_IT OK(tim2)\r\n");    }
  else                         { printf("HAL_TIM_Base_Start_IT error(tim2)\r\n"); }

  /* TIM4 인터럽트 시작 (IR 센서 + LED, 1 ms 주기) */
  tim4_it_start = HAL_TIM_Base_Start_IT(&htim4);
  if (HAL_OK == tim4_it_start) { printf("HAL_TIM_Base_Start_IT OK(tim4)\r\n");    }
  else                         { printf("HAL_TIM_Base_Start_IT error(tim4)\r\n"); }

  /* IR 센서 초기화 */
  tim4_IRsensor_OpCnt = 0;

  /* HC-SR04 초기화 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
  tim2_HCSR04_OpCnt     = 0;
  tim2_HCSR04_Start     = 'n';
  tim2_HCSR04_TrigHigh  = 'n';
  tim2_HCSR04_TrigLow   = 'n';
  tim2_HCSR04_EchoState = 'n';
  tim2_EchoSample       = 0x00;
  tim2_EchoCnt          = 0;
  tim2_EchoRising       = 'n';
  tim2_EchoFalling      = 'n';

  /* 모터 드라이버 초기화 */
  Motor_PWM_Start();
  Motor_Change_Speed(0, 0, 0, 0);
  Motor_Change_Speed(0, 0, 0, 0);
  HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontLI, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontLO, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontRI, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontRO, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearLI,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearLO,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearRI,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirRearRO,  GPIO_PIN_RESET);

  /* 상태 머신 초기화 */
  uState = STATE_BREAK;

  /* EXTI 플래그 초기화 */
  chPC13_Rising = 'n';

  /* LED 초기화: STATE_BREAK → 900 ms 느린 점멸 */
  LD2_toggle_cnt    = 0;
  LD2_toggle_OpTime = LD2_toggle_BREAK;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* STATE_BREAK: PC13 버튼(EXTI) 입력 대기 */
    if (STATE_BREAK == uState) {
      printf("STATE_BREAK\r\n");
      if ('y' == chPC13_Rising) {
        chPC13_Rising     = 'n';
        printf("ooooooo -Motor Start- ooooooo\r\n");
        Motor_Forwart_Start(100, 100, 100, 100);
        uState            = STATE_RUN;
        LD2_toggle_OpTime = LD2_toggle_RUN;
      }

    /* STATE_RUN: 라인 트레이싱 */
    } else if (STATE_RUN == uState) {

      //-------Emergency (비상 거리 감지, 현재 비활성화)-------
      /*if ((int)HCSR04_distance < 10) {
        Motor_Halt();
        printf("Motor emergency stop\r\n");
        uState            = STATE_BREAK;
        LD2_toggle_OpTime = LD2_toggle_BREAK;
      }*/

      switch (IRsensor_value) {
        case 0x00:  /* 전 센서 선 위 → 직선 전진 */
          printf("STATE_RUN\r\n");
          Motor_Forwart_Start(100, 100, 100, 100);
          break;
        case 0x01:  /* 우측 이탈 (약) → 완만한 좌회전 */
          printf("Motor Left Curve start\r\n");
          Motor_Forwart_Start(0, 100, 0, 100);
          break;
        case 0x03:  /* 우측 이탈 (급) → 급격한 좌회전, 좌측 후진 30% */
          printf("Motor Left Curve start\r\n");
          Motor_Forwart_Start(-30, 100, -30, 100);
          break;
        case 0x08:  /* 좌측 이탈 (약) → 완만한 우회전 */
          printf("Motor Right Curve start\r\n");
          Motor_Forwart_Start(100, 0, 100, 0);
          break;
        case 0x0C:  /* 좌측 이탈 (급) → 급격한 우회전, 우측 후진 30% */
          printf("Motor Right Curve start\r\n");
          Motor_Forwart_Start(100, -30, 100, -30);
          break;
        default:    /* 라인 이탈 → 비상 정지 */
          Motor_Halt();
          printf("xxxxxxx -Motor Stop- xxxxxxx\r\n");
          uState            = STATE_BREAK;
          LD2_toggle_OpTime = LD2_toggle_BREAK;
          break;
      }
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 640-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DirFrontLO_Pin|DirFrontLI_Pin|DirFrontRO_Pin|DirFrontRI_Pin
                          |DirBackLO_Pin|DirBackLI_Pin|DirBackRI_Pin|DirBackRO_Pin
                          |Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DirFrontLO_Pin DirFrontLI_Pin DirFrontRO_Pin DirFrontRI_Pin
                           DirBackLO_Pin DirBackLI_Pin DirBackRI_Pin DirBackRO_Pin
                           Trig_Pin */
  GPIO_InitStruct.Pin = DirFrontLO_Pin|DirFrontLI_Pin|DirFrontRO_Pin|DirFrontRI_Pin
                          |DirBackLO_Pin|DirBackLI_Pin|DirBackRI_Pin|DirBackRO_Pin
                          |Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LO_Pin RI_Pin LI_Pin */
  GPIO_InitStruct.Pin = LO_Pin|RI_Pin|LI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RO_Pin */
  GPIO_InitStruct.Pin = RO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_Pin */
  GPIO_InitStruct.Pin = Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
