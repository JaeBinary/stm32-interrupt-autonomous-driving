# IR 센서 기반 라인 추종

TIM4 ISR으로 4채널 IR 센서를 주기적으로 샘플링하고, 센서 값을 1바이트로 인코딩하여 라인 위치를 판단하는 방법을 설명합니다.

IR 센서 값을 1바이트 비트 패턴으로 합산하면 `switch` 문 하나로 모든 주행 상황을 처리할 수 있습니다. ISR이 샘플링을 담당하므로 메인 루프는 판단 로직에만 집중합니다.

---

<br>

## IR 센서 동작 원리

IR(Infrared) 반사형 센서는 적외선을 바닥에 발사하고 반사광의 강도를 측정합니다.

- **검은 선 위**: 적외선이 흡수되어 반사광이 약함 → 출력 `LOW (0)`
- **흰 바탕 위**: 적외선이 반사되어 반사광이 강함 → 출력 `HIGH (1)`

이 프로젝트에서는 센서 4개를 가로로 배치하여 라인의 좌우 위치를 감지합니다.

---

<br>

## 센서 배치와 핀 매핑

4개의 IR 센서를 우측부터 좌측 순서로 배치합니다.

```
바닥면 (진행 방향 →)
┌─────────────────────────┐
│  PA8   PB4   PB5   PB10 │
│  (bit0)(bit1)(bit2)(bit3)│
│  우측  ←─────────  좌측  │
└─────────────────────────┘
```

| STM32 핀 | 비트 위치 | 센서 위치 |
|---------|---------|---------|
| PA8  | bit 0 | 가장 우측 |
| PB4  | bit 1 | 우측 중간 |
| PB5  | bit 2 | 좌측 중간 |
| PB10 | bit 3 | 가장 좌측 |

---

<br>

## 센서 값 인코딩

`IRsensor_Read()` 함수는 4개의 GPIO 핀을 읽어 1바이트 값으로 합산합니다.

```c
void IRsensor_Read(void)
{
    IRsensor_value = 0;

    IRsensor_pa8  = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
    IRsensor_pb4  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
    IRsensor_pb5  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
    IRsensor_pb10 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

    IRsensor_value = (IRsensor_pb10 << 3)
                   | (IRsensor_pb5  << 2)
                   | (IRsensor_pb4  << 1)
                   | (IRsensor_pa8  << 0);
}
```

예를 들어 PA8과 PB4만 HIGH이면 `IRsensor_value = 0x03`이 됩니다.

---

<br>

## 폴링 주기 설정

TIM4는 **1 ms** 주기로 인터럽트를 발생시킵니다. ISR 안에서 카운터가 `IRsensor_OpTime`(ms)을 초과하면 `IRsensor_Read()`를 호출합니다.

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim4.Instance) {
        tim4_IRsensor_OpCnt += 1;
        if (IRsensor_OpTime < tim4_IRsensor_OpCnt) { // 1ms 단위
            tim4_IRsensor_OpCnt = 0;
            IRsensor_Read();
        }
    }
}
```

| 버전 | `IRsensor_OpTime` | 실제 샘플링 주기 |
|------|------------------|----------------|
| 강의 템플릿 | 300 | 300 ms |
| 학생 구현 (`src/main.c`) | 30 | 30 ms |

30 ms 폴링은 템플릿의 300 ms보다 10배 빠르게 라인 변화를 감지합니다. 고속 주행 중 커브에서 센서 값을 빠르게 갱신할 수 있습니다.

---

<br>

## 라인 위치 해석

`IRsensor_value`의 비트 패턴으로 로봇의 현재 위치를 파악합니다.

| `IRsensor_value` | 패턴 (bit3~0) | 해석 | 필요 동작 |
|-----------------|-------------|------|---------|
| `0x00` | `0000` | 전 센서 검은 선 위 | 직선 전진 |
| `0x01` | `0001` | 우측 끝 센서만 흰 바탕 | 완만한 좌회전 |
| `0x03` | `0011` | 우측 두 센서 흰 바탕 | 급격한 좌회전 |
| `0x08` | `1000` | 좌측 끝 센서만 흰 바탕 | 완만한 우회전 |
| `0x0C` | `1100` | 좌측 두 센서 흰 바탕 | 급격한 우회전 |
| `0x0F` | `1111` | 전 센서 흰 바탕 | 라인 이탈 → 정지 |

0은 검은 선, 1은 흰 바탕을 나타냅니다. 라인 위에 있는 센서가 0을 출력합니다.

---

<br>

## TIM4 설정

```
f_tick = 64 MHz / PSC / ARR
       = 64,000,000 / 64 / 1,000
       = 1,000 Hz  →  주기 1 ms
```

| 레지스터 | 계산값 | 실제 설정값 |
|---------|--------|------------|
| PSC | 64 | 64 - 1 = **63** |
| ARR | 1,000 | 1,000 - 1 = **999** |

TIM4 ISR에서 IR 센서 샘플링과 LD2 LED 토글을 동시에 처리합니다. 두 기능이 독립 카운터로 동작하므로 서로 영향을 주지 않습니다.

---

<br>

## 참고 사항

- `IRsensor_value`는 ISR 안에서만 갱신됩니다. 메인 루프에서 읽을 때 값이 변경 중일 수 있으나, 1바이트 단위 접근은 STM32에서 원자적(atomic)으로 처리됩니다.
- 센서 출력 극성(검은 선 = 0 또는 1)은 IR 센서 모듈의 종류에 따라 다를 수 있습니다. 이 프로젝트에서는 검은 선 위에서 0, 흰 바탕에서 1로 읽힙니다.
- 실제 주행 판단 로직(`switch(IRsensor_value)`)과 Motor 함수 연동은 [07-autonomous-driving.md](./07-autonomous-driving.md)에서 설명합니다.
