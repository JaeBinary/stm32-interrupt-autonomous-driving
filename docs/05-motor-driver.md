# L298N 모터 드라이버 제어

H-Bridge 방향 제어와 TIM3 PWM 속도 제어를 조합하여 4WD 모터 드라이버를 구동하는 방법을 설명합니다.

부호 있는 정수(signed int)로 PWM 값을 전달하면 음수 값이 해당 바퀴를 후진으로 전환합니다. 이를 통해 Tank Turn(제자리 회전)과 급격한 방향 전환을 단일 함수 호출로 구현합니다.

---

<br>

## L298N H-Bridge 동작 원리

L298N은 IN/OUT 핀 쌍으로 한 채널의 회전 방향을 결정합니다.

| IN 핀 (I) | OUT 핀 (O) | 모터 동작 |
|-----------|-----------|---------|
| HIGH | LOW | 전진 |
| LOW | HIGH | 후진 |
| LOW | LOW | 정지 (브레이크) |

STM32에서 IN/OUT 핀은 GPIOC를 통해 제어합니다.

---

<br>

## GPIO 핀 매핑

| 핀 | 매크로 | 담당 바퀴 / 역할 |
|----|--------|----------------|
| PC0 | `GPIO_PIN_DirFrontLO` | 전방 좌측 OUT |
| PC1 | `GPIO_PIN_DirFrontLI` | 전방 좌측 IN |
| PC2 | `GPIO_PIN_DirFrontRO` | 전방 우측 OUT |
| PC3 | `GPIO_PIN_DirFrontRI` | 전방 우측 IN |
| PC4 | `GPIO_PIN_DirRearLO`  | 후방 좌측 OUT |
| PC5 | `GPIO_PIN_DirRearLI`  | 후방 좌측 IN |
| PC6 | `GPIO_PIN_DirRearRO`  | 후방 우측 OUT |
| PC7 | `GPIO_PIN_DirRearRI`  | 후방 우측 IN |

---

<br>

## 방향 제어 추상화

방향 제어 코드를 매번 GPIO 핀 단위로 쓰면 실수가 생기기 쉽습니다. `Motor_Dir_t` 열거형과 바퀴별 방향 함수로 추상화합니다.

```c
typedef enum {
    DIR_BACK  = GPIO_PIN_RESET,   // 후진 → IN=LOW, OUT=HIGH
    DIR_FRONT                     // 전진 → IN=HIGH, OUT=LOW
} Motor_Dir_t;

void Motor_FrontL_Direction(Motor_Dir_t dir)
{
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontLI,  dir);
    HAL_GPIO_WritePin(GPIODir, GPIO_PIN_DirFrontLO, !dir);
}
// FR, RL, RR도 동일한 패턴으로 구현
```

`DIR_FRONT`를 전달하면 IN=HIGH, OUT=LOW가 되어 전진합니다. `DIR_BACK`을 전달하면 반전됩니다.

---

<br>

## 속도 제어: CCR 레지스터 직접 쓰기

`Motor_Change_Speed()`는 ARR 기준 퍼센트로 CCR을 계산하여 4채널에 동시에 적용합니다.

```c
void Motor_Change_Speed(uint32_t pwm_FL, uint32_t pwm_FR,
                        uint32_t pwm_RL, uint32_t pwm_RR)
{
    htim3.Instance->CCR3 = (htim3.Instance->ARR * pwm_FL) / 100; // CH3 → FL
    htim3.Instance->CCR4 = (htim3.Instance->ARR * pwm_FR) / 100; // CH4 → FR
    htim3.Instance->CCR2 = (htim3.Instance->ARR * pwm_RL) / 100; // CH2 → RL
    htim3.Instance->CCR1 = (htim3.Instance->ARR * pwm_RR) / 100; // CH1 → RR
}
```

---

<br>

## 부호 있는 PWM으로 바퀴별 독립 방향 제어

강의 템플릿의 `Motor_Forwart_Start()`는 파라미터가 `uint32_t`여서 항상 전진 방향만 지원합니다.

이 프로젝트에서는 파라미터를 `int`(부호 있는 정수)로 변경하여 음수 값을 입력하면 해당 바퀴가 후진하도록 확장했습니다.

```c
// 강의 템플릿 (단방향 전진만 가능)
void Motor_Forwart_Start(uint32_t pwm_FL, uint32_t pwm_FR,
                         uint32_t pwm_RL, uint32_t pwm_RR);

// 학생 구현 (음수 = 해당 바퀴 후진)
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
```

---

<br>

## 주행 패턴 예시

| 호출 | 동작 | 설명 |
|------|------|------|
| `Motor_Forwart_Start(100, 100, 100, 100)` | 직진 전진 | 4바퀴 전진 100% |
| `Motor_Forwart_Start(0, 100, 0, 100)` | 완만한 좌회전 | 좌측 정지, 우측 전진 |
| `Motor_Forwart_Start(-30, 100, -30, 100)` | 급격한 좌회전 | 좌측 후진 30%, 우측 전진 100% |
| `Motor_Forwart_Start(100, 0, 100, 0)` | 완만한 우회전 | 우측 정지, 좌측 전진 |
| `Motor_Forwart_Start(100, -30, 100, -30)` | 급격한 우회전 | 우측 후진 30%, 좌측 전진 100% |

좌측 바퀴와 우측 바퀴가 **반대 방향**으로 회전하면 Tank Turn(제자리 회전)이 됩니다. 예를 들어 `Motor_Forwart_Start(-50, 50, -50, 50)`을 호출하면 로봇이 제자리에서 좌로 회전합니다.

---

<br>

## 정지 함수

`Motor_Halt()`는 모든 채널을 0으로 설정하고 방향 핀을 LOW/LOW로 만들어 완전히 정지합니다.

```c
void Motor_Halt(void)
{
    Motor_Change_Speed(0, 0, 0, 0);
    Motor_Front_Direction(LOW, LOW, LOW, LOW);
    Motor_Rear_Direction(LOW, LOW, LOW, LOW);
}
```

---

<br>

## 참고 사항

- `Motor_PWM_Start()`를 먼저 호출해야 TIM3 채널 출력이 활성화됩니다. 초기화 시 한 번만 호출합니다.
- PWM 값 범위는 0~100입니다. 100을 초과하면 CCR이 ARR을 넘어 항상 HIGH가 됩니다.
