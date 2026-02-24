# 라인 트레이싱 자율주행 구현

`switch(IRsensor_value)`로 센서 패턴을 판단하고 `Motor_Forwart_Start()`를 호출하여 직진·좌회전·우회전·비상 정지를 자동으로 수행하는 방법을 설명합니다.

센서 값이 바뀔 때마다 `switch` 문이 즉시 모터 명령을 결정하므로 별도의 제어 알고리즘 없이 라인 추종이 가능합니다. 부호 있는 PWM 확장 덕분에 바퀴 역전을 포함한 급격한 커브도 단일 함수 호출로 처리합니다.

---

<br>

## 강의 템플릿과 학생 구현의 차이

강의 템플릿(`docs/2. 기말고사 범위/`)에는 `switch` 내부가 주석으로 비어 있습니다.

```c
// 강의 템플릿 (08-DFrobot-LeftRightCurve.txt)
//-----Straight code 추가-----
//-----Left Curve code 추가-----
//-----Right Curve code 추가-----
```

`src/main.c`(학생 구현)에서는 이 부분을 완성하고, `Motor_Forwart_Start()`의 파라미터 타입을 `uint32_t`에서 `int`로 변경하여 음수 PWM으로 바퀴를 역전시킵니다.

| 항목 | 강의 템플릿 | 학생 구현 |
|------|----------|---------|
| `Motor_Forwart_Start` 파라미터 | `uint32_t` (항상 전진) | `int` (음수 = 후진) |
| `IRsensor_OpTime` | 300 ms | 30 ms |
| switch 케이스 구현 | 주석 (미완성) | 5개 케이스 완성 |
| 비상 정지 조건 | `0x0F` (전 센서 흰 바탕) | `default` (예상 외 패턴) |

---

<br>

## 주행 판단 로직

`STATE_RUN` 상태에서 메인 루프는 매 반복마다 `IRsensor_value`를 확인합니다.

```c
switch (IRsensor_value) {
case 0x00: // 전 센서 검은 선 위 → 직선
    Motor_Forwart_Start(100, 100, 100, 100);
    break;
case 0x01: // 우측 끝만 흰 바탕 → 완만한 좌회전
    Motor_Forwart_Start(0, 100, 0, 100);
    break;
case 0x03: // 우측 두 센서 흰 바탕 → 급격한 좌회전
    Motor_Forwart_Start(-30, 100, -30, 100);
    break;
case 0x08: // 좌측 끝만 흰 바탕 → 완만한 우회전
    Motor_Forwart_Start(100, 0, 100, 0);
    break;
case 0x0C: // 좌측 두 센서 흰 바탕 → 급격한 우회전
    Motor_Forwart_Start(100, -30, 100, -30);
    break;
default:   // 예상 외 패턴 → 비상 정지
    Motor_Halt();
    uState = STATE_BREAK;
    LD2_toggle_OpTime = LD2_toggle_BREAK; // 900ms 느린 점멸
    break;
}
```

---

<br>

## 각 케이스 설명


### 직진 (`0x00`)

4개 센서 모두 검은 선 위에 있습니다. 모든 바퀴에 동일한 속도를 줍니다.

```c
Motor_Forwart_Start(100, 100, 100, 100);
```

<br>

### 완만한 좌회전 (`0x01`)

우측 끝 센서(PA8, bit0)만 흰 바탕을 감지합니다. 로봇이 우측으로 약간 치우쳤습니다. 좌측 바퀴를 정지하고 우측 바퀴만 전진하여 부드럽게 좌회전합니다.

```c
Motor_Forwart_Start(0, 100, 0, 100); // 좌측 0%, 우측 100%
```

<br>

### 급격한 좌회전 (`0x03`)

우측 두 센서(PA8, PB4)가 흰 바탕입니다. 로봇이 우측으로 크게 벗어났습니다. 좌측 바퀴를 30%로 후진시키고 우측을 100% 전진하여 Tank Turn에 가까운 회전을 수행합니다.

```c
Motor_Forwart_Start(-30, 100, -30, 100); // 좌측 역방향 30%, 우측 100%
```

<br>

### 완만한 우회전 (`0x08`)

좌측 끝 센서(PB10, bit3)만 흰 바탕입니다. 우측 바퀴를 정지하고 좌측만 전진합니다.

```c
Motor_Forwart_Start(100, 0, 100, 0); // 좌측 100%, 우측 0%
```

<br>

### 급격한 우회전 (`0x0C`)

좌측 두 센서(PB5, PB10)가 흰 바탕입니다. 우측 바퀴를 30%로 후진시킵니다.

```c
Motor_Forwart_Start(100, -30, 100, -30); // 좌측 100%, 우측 역방향 30%
```

<br>

### 비상 정지 (`default`)

위에서 정의하지 않은 패턴이 감지되면 즉시 정지하고 `STATE_BREAK`로 전환합니다.

```c
Motor_Halt();
uState = STATE_BREAK;
LD2_toggle_OpTime = LD2_toggle_BREAK; // 900ms 느린 점멸로 전환
```

강의 템플릿의 비상 정지 조건은 `0x0F`(전 센서 흰 바탕)이었습니다. 학생 구현에서는 `default`를 사용하여 예상하지 못한 모든 패턴에서 즉시 정지합니다.

---

<br>

## 직진 판단 보완: 강의 템플릿 비교

강의 템플릿(07-DFrobot-straight emergency.txt)의 비상 정지는 `0x0F`에만 반응합니다.

```c
// 강의 템플릿의 비상 정지
if (0x0F == IRsensor_value) {
    Motor_Halt();
    uState = STATE_BRAKE;
}
```

학생 구현에서는 `switch`의 `default`가 이 역할을 대신하며, `0x0F` 외에도 정의되지 않은 센서 패턴 전체에서 정지합니다.

---

<br>

## 전체 동작 흐름

```
[전원 투입]
    │
    ▼
[STATE_BREAK]  ← 900ms 느린 LED 점멸
    │  PC13 버튼 누름 → EXTI ISR → chPC13_Rising = 'y'
    ▼
[STATE_RUN]    ← 300ms 빠른 LED 점멸
    │
    ├─ 0x00 → Motor_Forwart_Start(100, 100, 100, 100)  직진
    ├─ 0x01 → Motor_Forwart_Start(0, 100, 0, 100)       완만한 좌회전
    ├─ 0x03 → Motor_Forwart_Start(-30, 100, -30, 100)   급격한 좌회전
    ├─ 0x08 → Motor_Forwart_Start(100, 0, 100, 0)       완만한 우회전
    ├─ 0x0C → Motor_Forwart_Start(100, -30, 100, -30)   급격한 우회전
    └─ default → Motor_Halt() → [STATE_BREAK]
```

---

<br>

## 참고 사항

- `IRsensor_value`는 TIM4 ISR에서 30 ms마다 갱신됩니다. 메인 루프에서 `switch`를 매 반복마다 실행하더라도 센서 값 자체는 30 ms 간격으로만 바뀝니다.
- PWM 음수 값(-30)의 절댓값 30%는 커브 반경 조정 가능 범위에서 선택한 값입니다. 값을 크게 하면 회전 반경이 줄어들고 작게 하면 커집니다.
- IR 센서 배치, 인코딩 방식, TIM4 설정은 [04-ir-sensor.md](./04-ir-sensor.md)를 참고하세요.
- `Motor_Forwart_Start()` 구현과 `Motor_Dir_t` 열거형은 [05-motor-driver.md](./05-motor-driver.md)를 참고하세요.
