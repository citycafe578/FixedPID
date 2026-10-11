# FixedPID 使用指南

[English](USAGE.md) | [繁體中文](USAGE.zh-TW.md)

這份指南說明如何在實際專案中使用 FixedPID：單位與縮放、該呼叫哪個更新函式、每個設定的作用、數值在邊界情況下的行為，以及可以直接套用的範例。功能總覽與效能測試請看 [README](../README.zh-TW.md)。

## 目錄

1. [安裝](#1-安裝)
2. [快速開始](#2-快速開始)
3. [單位與縮放](#3-單位與縮放)
4. [選擇更新函式](#4-選擇更新函式)
5. [設定總表](#5-設定總表)
6. [限幅與抗積分飽和](#6-限幅與抗積分飽和)
7. [微分濾波](#7-微分濾波)
8. [速度型](#8-速度型)
9. [輸入範圍與數值行為](#9-輸入範圍與數值行為)
10. [範例](#10-範例)
11. [調參流程](#11-調參流程)
12. [疑難排解](#12-疑難排解)
13. [在中斷與多工中使用](#13-在中斷與多工中使用)
14. [執行測試](#14-執行測試)

---

## 1. 安裝

* **Arduino IDE / PlatformIO（手動）**：把 `src/FixedPID.h` 和 `src/FixedPID.cpp` 複製到專案裡，或把整個儲存庫資料夾放進 `libraries/`。
* **PlatformIO（從 GitHub）**：

  ```ini
  lib_deps = https://github.com/citycafe578/FixedPID.git
  ```

然後 `#include "FixedPID.h"`。函式庫只需要 `<stdint.h>` 和相容 GCC 或 Clang 的編譯器（使用 `__builtin_add_overflow` / `__builtin_mul_overflow`）。它不使用浮點數、動態記憶體，也不依賴任何 Arduino API。

FixedPID 以 32 位元 MCU 為目標，已在 ESP32 上測試；8 位元 AVR 板不是目標平台。

## 2. 快速開始

```cpp
#include "FixedPID.h"

FixedPID pid;

void setup() {
  pid.setTunings(1500, 200, 50);        // Kp = 1.5, Ki = 0.2 /s, Kd = 0.05 s
  pid.setOutputLimits(-10000, 10000);
  pid.setIntegralLimits(-5000, 5000);
}

void loop() {
  static uint32_t last = micros();
  uint32_t now = micros();
  uint32_t dt_us = now - last;          // 無號相減，micros() 繞回時依然正確
  last = now;

  int32_t output = pid.update(targetValue, measuredValue, dt_us);
  // ... 用 `output` 驅動致動器
}
```

如果迴圈以固定頻率執行（硬體計時器、RTOS tick），改用固定頻率版本，就不需要傳 `dt_us`：

```cpp
pid.setFrequency(1000);                 // 1 kHz
int32_t output = pid.updateFixedRate(targetValue, measuredValue);
```

## 3. 單位與縮放

FixedPID 只用整數運算，所有數值的單位如下：

| 項目 | 型別 | 意義 |
| --- | --- | --- |
| `target`、`input` | `int32_t` | 單位自訂（RPM、編碼器刻度、0.01 °C……），兩者必須一致。 |
| 誤差 | 內部 `int64_t` | `target - input`，範圍為 33 位元（`INT32_MIN - INT32_MAX` 也放得下）。 |
| `kp`、`ki`、`kd` | `int32_t` | 實際增益**乘以 `PID_SCALE` = 1000**。 |
| `dt_us` | `uint32_t` | 時間步長，單位**微秒**。 |
| 輸出 | `int32_t` | 你要的致動器命令單位。 |

控制器計算的是

```
output = Kp * e  +  Ki * ∫e dt  +  Kd * de/dt
```

其中 `e = target - input`，`Kp`、`Ki`、`Kd` 是實際（浮點）增益，儲存的值為 `K * 1000`：

| 項 | 實際增益單位 | 儲存值 | 例 |
| --- | --- | --- | --- |
| P | 每單位誤差的輸出 | `Kp * 1000` | `Kp = 1.5` → `1500` |
| I | 每（誤差單位 · 秒）的輸出 | `Ki * 1000` | `Ki = 0.2 /s` → `200` |
| D | 每（誤差單位 / 秒）的輸出 | `Kd * 1000` | `Kd = 0.05 s` → `50` |

驗算範例：

* P：誤差 `100`、`kp = 1500` → `1500 * 100 / 1000 = 150`。
* I：誤差 `100` 持續 1 秒、`ki = 200` → `0.2 * 100 * 1 = 20`。
* D：誤差在 1 ms 內跳 `10`、`kd = 50` → `0.05 * 10 / 0.001 = 500`。

小於 `0.001` 的增益無法表示。如果需要很小的增益，請改變誤差或輸出的單位。

## 4. 選擇更新函式

| 函式 | 使用時機 |
| --- | --- |
| `update(target, input, dt_us)` | 迴圈週期會變動（例如由 `loop()` 或感測器中斷驅動）。每次呼叫都傳入實測的 `dt_us`。 |
| `updateFixedRate(target, input)` | 迴圈由計時器或 RTOS tick 以固定頻率驅動。事先呼叫一次 `setFrequency(hz)`。 |
| `updateVelocity(target, input, dt_us)` | 致動器要的是「變化量」（增量式致動器），週期可變。見[速度型](#8-速度型)。 |
| `updateVelocityFixedRate(target, input)` | 同上，固定頻率。 |

四個函式回傳的都是新的控制器輸出，並受 `setOutputLimits()` 限制。在能整除 1,000,000 的頻率（500、1000、2000、4000……Hz），對於一般工作範圍內的輸入，`update(.., 1000000 / hz)` 與 `updateFixedRate(..)` 的結果完全相同（由模糊測試檢查）。

不要在同一個實例上混用位置型與速度型：兩者的內部狀態不同。每個控制迴圈請使用一個 `FixedPID`。

## 5. 設定總表

預設值是剛建構出來的 `FixedPID` 所使用的值。

| 函式 | 預設 | 作用 |
| --- | --- | --- |
| `setTunings(kp, ki, kd)` | `0, 0, 0` | 增益，各為 `實際值 * 1000`。全為 0 時輸出恆為 `0`。 |
| `setOutputLimits(min, max)` | `INT32_MIN, INT32_MAX` | 限制回傳的輸出。若 `min > max` 會自動對調。 |
| `setIntegralLimits(min, max)` | `INT32_MIN, INT32_MAX` | 限制**積分項對輸出的貢獻**（單位是輸出，不是誤差）。`min > max` 時自動對調。僅位置型使用。 |
| `setFrequency(hz)` | 未設定 | `*FixedRate` 函式需要。有效範圍 1 – 1,000,000 Hz，成功時回傳 `true`。 |
| `setErrorDeadband(edb)` | `0` | 若 `|error| <= edb`，誤差視為 `0`（P、I、D 都看到 0）。 |
| `setErrorIntegralThreshold(eit)` | `INT32_MAX` | 只有 `|error| <= eit` 時積分才會累積；超出範圍時積分凍結（不會歸零）。 |
| `setDerivativeFilter(alpha)` | `1000`（關閉） | 對微分項做低通濾波，`0 – 1000`。見[微分濾波](#7-微分濾波)。 |
| `reset()` | – | 清除所有內部狀態（積分、前次誤差、速度型輸出、濾波器、餘數）。設定值保留。 |

注意事項：

* 頻率為 `0` 或高於 1 MHz 時，`setFrequency()` 會**回傳 `false`**，並停用固定頻率函式：在設定有效頻率之前，它們一律回傳 `0`。請檢查回傳值。
* `setFrequency()` 與 `setTunings()` 的呼叫順序不拘。
* `dt_us == 0` 時，`update()` / `updateVelocity()` 回傳 `0`，且不會更動任何狀態。
* 執行中可以修改設定。迴圈暫停後重新啟動、切換模式、或致動器曾被停用時，請呼叫 `reset()`。

## 6. 限幅與抗積分飽和

**輸出限幅**會限制回傳值。請設成致動器真正能接受的範圍（例如 8 位元 PWM 為 `0 … 255`）。

**積分限幅**限制 I 項單獨能為輸出增加多少。內部的積分狀態會一起被夾住，所以到達限幅之後，誤差反向時積分會立刻開始下降，而不是先「償還」一段看不見的超額。

**抗積分飽和**：位置型中，只有在未限幅的輸出尚未超出限幅、或誤差方向不會把輸出推得更遠時，新的積分值才會被保留。也就是說，當輸出已飽和而誤差仍想把它推得更遠時，積分就停止增加。

建議的設定：

```cpp
pid.setOutputLimits(0, 255);       // 致動器的實際範圍
pid.setIntegralLimits(-100, 100);  // I 項最多為輸出增減 ±100
```

**誤差死區**（`setErrorDeadband`）會完全去除小誤差，避免目標附近致動器抖動。注意誤差離開死區時輸出會跳一下，因為死區並不會把自己的寬度扣掉。

**積分閾值**（`setErrorIntegralThreshold`）讓積分只在接近目標時才啟用。它**不會**在範圍外把積分歸零，只是讓它停止變動。目標大幅變動時很有用。

## 7. 微分濾波

```
filteredD = (alpha * rawD + (1000 - alpha) * previousFilteredD) / 1000
```

| `alpha` | 效果 |
| ---: | --- |
| `1000` | 不濾波（預設）。 |
| `500` | 每筆新值占 50 %。 |
| `100` | 強平滑。 |
| `0` | 濾波值凍結在上一個值（D 項不再有反應）。 |

**`alpha` 越低，平滑越強**（延遲也越大）。大於 1000 的值視為 1000。只有 `kd != 0` 時濾波器才會運作。速度型中，濾波的對象是微分項的增量。

微分作用在**誤差**上，所以 `target` 突然改變時，D 項會出現一個單步尖峰（derivative kick）。如果你的目標是階躍式變化，可以把 `kd` 設小一點、使用濾波，或自己先把目標做成斜坡再傳給 `update()`。

## 8. 速度型

`updateVelocity()` / `updateVelocityFixedRate()` 每一步計算的是輸出的**變化量**，加到內部的累加器；回傳的就是這個累加器，並受 `setOutputLimits()` 限制。

和位置型不同的地方：

* **不使用** `setIntegralLimits()`。累加器只受 `setOutputLimits()` 限制，它同時也扮演抗積分飽和的角色。
* `reset()` 之後（以及剛建構時）累加器從 `0` 開始，目前無法用致動器現有的值預先載入。
* P、I、D 每一步的貢獻是「不足一個輸出單位」的小數，餘數會帶到下一次呼叫，所以緩慢而微小的變化不會因四捨五入而消失。
* D 項使用誤差的二階差分，`target` 階躍時仍會出現尖峰。

適用於會對命令做積分的致動器（以增量移動的閥門、步進式定位器等）。其他情況用位置型比較簡單。

## 9. 輸入範圍與數值行為

### 保證

* 任何 `int32_t` 的 target、input、增益，任何 `uint32_t` 的 `dt_us`，任何有效頻率：函式庫都不會讓 `int64_t` 溢位，也沒有未定義行為。會溢位的中間乘積改為**飽和**處理，所以輸出只會停在對應的輸出限幅。
* 回傳值一定落在 `[outputMin, outputMax]` 內。
* 中間值放得進 `int64_t` 時，結果是精確的；極端組合（超大增益 × 超大誤差 × 超大 `dt`）可能比數學上的精確值更早飽和。
* 這些行為由單元測試、128 位元參考模型，以及加上 sanitizer 的模糊測試檢查（見[執行測試](#14-執行測試)）。

### 進位方式

* 所有除法都向零截斷。
* 位置型：P 與 D 每步截斷；積分保有完整解析度。
* 速度型：P、I、D 的餘數會帶到下一步。
* 固定頻率模式下，多數頻率的 `1,000,000 / hz` 不是整數；小數部分會累積進積分，所以時間基準保持精確。

### 值得知道的細節

* 預設積分閾值是 `INT32_MAX`，所以大於它的誤差（只有 `target` 與 `input` 正負相反且數值極大時才會發生）不會積分。如果你的數值逼近整個 `int32_t` 範圍，請明確設定閾值。
* 接受負增益（反向作用的迴圈）。在極端數值下，兩個方向相反的超大項可能在互相抵消之前就先飽和；若需要反向作用，改為把誤差或輸出取負號會比較可預測。
* 函式不是執行緒安全的。見[在中斷與多工中使用](#13-在中斷與多工中使用)。

## 10. 範例

### 10.1 馬達轉速（迴圈週期可變）

```cpp
#include "FixedPID.h"

FixedPID speedPid;
uint32_t lastUs;

void setup() {
  speedPid.setTunings(800, 3000, 0);      // Kp = 0.8, Ki = 3.0 /s
  speedPid.setOutputLimits(0, 255);       // PWM 範圍
  speedPid.setIntegralLimits(0, 255);
  lastUs = micros();
}

void loop() {
  uint32_t now = micros();
  uint32_t dt = now - lastUs;
  lastUs = now;

  int32_t pwm = speedPid.update(targetRpm, readRpm(), dt);
  writePwm(pwm);
}
```

### 10.2 溫度（慢速迴圈、感測器有雜訊）

用百分之一度當單位，整數才有足夠的解析度。

```cpp
FixedPID heater;

void setup() {
  heater.setTunings(2000, 50, 0);                // Kp = 2.0, Ki = 0.05 /s
  heater.setOutputLimits(0, 255);
  heater.setIntegralLimits(0, 255);
  heater.setErrorDeadband(10);                   // 忽略 ±0.10 °C 的感測雜訊
  heater.setErrorIntegralThreshold(500);         // 距目標 5.00 °C 內才積分
}

void loop() {                                    // 每秒執行一次
  static uint32_t last = millis();
  uint32_t now = millis();
  int32_t out = heater.update(targetCentiDeg, readCentiDeg(), (now - last) * 1000UL);
  last = now;
  writeHeater(out);
  delay(1000);
}
```

### 10.3 固定頻率迴圈（計時器驅動，例如飛控）

```cpp
FixedPID rollRate;

void setup() {
  rollRate.setTunings(1200, 400, 30);
  rollRate.setOutputLimits(-1000, 1000);
  rollRate.setIntegralLimits(-300, 300);
  rollRate.setDerivativeFilter(250);             // 平滑陀螺儀在 D 項的雜訊
  if (!rollRate.setFrequency(1000)) {
    // 頻率無效：請處理錯誤，否則固定頻率呼叫只會回傳 0
  }
}

// 由你的 1 kHz 計時器任務呼叫
void controlTick() {
  int32_t cmd = rollRate.updateFixedRate(rollSetpoint, gyroRoll);
  mixAndOutput(cmd);
}
```

### 10.4 暫停後重新啟動

```cpp
void onEnable() {
  pid.reset();          // 清掉舊的積分與前次誤差
}
```

## 11. 調參流程

1. 先設 `ki = 0`、`kd = 0`，輸出限幅設成符合致動器。
2. 提高 `kp`，直到反應夠快但開始過衝或振盪，再退回約一半。
3. 加上 `ki`，直到穩態誤差在可接受的時間內消失。設定積分限幅，讓 I 項單獨無法把致動器推到極限。
4. 只有在需要抑制過衝時才加 `kd`。若 D 項雜訊大，降低濾波的 `alpha`（例如 `500`，再 `250`）。
5. 如果輸出在目標附近抖動，加一點死區。如果因為積分造成大幅變動時過衝，加上積分閾值。

記得儲存的增益是 `實際值 * 1000`：儲存值變 `1` 就是實際增益變 `0.001`。

## 12. 疑難排解

| 現象 | 可能原因與處理 |
| --- | --- |
| 輸出恆為 `0` | 增益全為 0；`dt_us` 為 `0`；或呼叫 `*FixedRate` 函式但沒有有效的 `setFrequency()`（請檢查它的回傳值）；或誤差落在死區內。 |
| 輸出卡在限幅 | 已達輸出限幅且誤差方向不變；請檢查單位與增益縮放（別忘了 `* 1000`）。 |
| 長時間飽和後出現緩慢漂移或過衝 | 設定 `setIntegralLimits()`（位置型），並讓輸出限幅符合致動器。 |
| 重新啟用迴圈時輸出跳動 | 重新啟動迴圈時呼叫 `reset()`。 |
| 輸出雜訊大 | 降低微分濾波的 `alpha`、加死區，或降低 `kd`。 |
| 結果比預期小很多 | 儲存的增益是 `實際值 * 1000`；`ki` 是每秒、`dt_us` 是微秒。 |
| 固定頻率的輸出和 `update()` 略有差異 | 只會發生在不能整除 1,000,000 的頻率；固定頻率版本較精確。 |

## 13. 在中斷與多工中使用

* 函式庫不用動態記憶體、不用浮點數、也沒有阻塞呼叫；在 32 位元 MCU 上，每次呼叫約為數微秒（實際時間依晶片而定）。
* 實例不可重入、也不是執行緒安全的。只從單一情境呼叫，或用鎖／臨界區保護。不要在沒有保護的情況下讓 ISR 與任務共用同一個實例。
* `reset()` 與各個設定函式會改變和更新函式相同的狀態，請從與 `update` 相同的情境呼叫。

## 14. 執行測試

儲存庫包含在 PC 上執行的測試，以及在 ESP32 上執行的測試。

在 PC 上（需要 `make` 與 GCC 或 Clang）：

```
make test          # 單元測試 + 模糊測試，以 AddressSanitizer 與 UBSan 編譯
make fuzz SCALE=20 # 較長時間的模糊測試
```

* `test/test.cpp` 是單元測試套件，同一個檔案也能在 ESP32（Arduino 框架）上執行，並在序列埠印出通過／失敗摘要與跑分。
* `test/host/fuzz_test.cpp` 會把每條更新路徑的第一次呼叫與 128 位元參考模型比對、對四個更新函式丟入隨機的極端設定、檢查可變 dt 與固定頻率是否一致，並確認 `reset()` 能還原成全新狀態。

相同的檢查會在每次 push 時於 GitHub Actions 執行（`.github/workflows/ci.yml`）。
