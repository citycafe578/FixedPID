# FixedPID

[English](README.md) | [繁體中文](README.zh-TW.md)

一個為嵌入式系統設計的小型定點數 PID 控制器。

FixedPID 使用整數運算，並以 `int64_t` 作為中間計算型別，取代浮點數運算。它適合需要可預測的整數計算、低運算開銷，以及明確控制取樣時間的應用。

## 功能特色

* 支援 P / PI / PID 控制

* 定點數增益表示（`PID_SCALE = 1000`）

* `int32_t` 輸入與輸出

* `int64_t` 中間計算

* 可變時間步長（`update`）

* 固定頻率高速路徑（`updateFixedRate`）

* 速度型 PID（`updateVelocity` / `updateVelocityFixedRate`）

* 輸出限幅

* 積分限幅

* 抗積分飽和（Anti-windup）

* 誤差死區（Error deadband）

* 積分閾值控制（Integral threshold control）

* 微分低通濾波

* 控制器狀態重置

* `dt = 0` 保護

* 專為嵌入式系統設計

## 目前狀態

**版本：v0.2**

此版本改善了運算效能、擴充 API，並增加測試覆蓋率。

---

## 基本用法

```cpp
#include "FixedPID.h"

FixedPID pid;

void setup() {

  pid.setTunings(
    1500,  // Kp = 1.5
    200,   // Ki = 0.2
    50     // Kd = 0.05
  );

  pid.setOutputLimits(-10000, 10000);
  pid.setIntegralLimits(-5000, 5000);
}

void loop() {

  int32_t target = 1000;
  int32_t input  = 950;
  uint32_t dt_us = 1000;  // 1 ms

  int32_t output = pid.update(target, input, dt_us);
}
```

### 固定頻率模式（更快）

當控制迴圈以固定頻率執行時，可以使用固定頻率模式：

```cpp
pid.setFrequency(1000);  // 1000 Hz
int32_t output = pid.updateFixedRate(target, input);
```

---

## 定點數增益

```cpp
#define PID_SCALE 1000
```

```text
儲存值 = 增益 × 1000
```

| 期望增益 | 傳入 `setTunings()` 的值 |
| ---: | -------------------: |
| 0.01 |                   10 |
| 0.05 |                   50 |
|  0.1 |                  100 |
|  0.5 |                  500 |
|  1.0 |                 1000 |
|  1.5 |                 1500 |
|  2.0 |                 2000 |

---

## API 總覽

### 核心功能

| 函式                               | 說明                     |
| -------------------------------- | ---------------------- |
| `setTunings(kp, ki, kd)`         | 設定 P、I、D 增益（×1000 定點數） |
| `setOutputLimits(min, max)`      | 限制最終輸出範圍               |
| `setIntegralLimits(min, max)`    | 限制積分項範圍                |
| `reset()`                        | 清除積分與微分相關狀態            |
| `update(target, input, dt_us)`   | 使用可變時間步長執行一次 PID 計算    |
| `updateFixedRate(target, input)` | 固定頻率高速運算路徑             |

### v0.2 新增

| 函式                               | 用途                                                                                                                                                 |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| `setFrequency(hz)`                 | 設定 `updateFixedRate` / `updateVelocityFixedRate` 使用的固定迴圈頻率，並預先計算 Ki/Kd 的時間縮放，以降低每次更新的運算成本。                         |
| `setErrorDeadband(edb)`            | 忽略 ±deadband 範圍內的微小誤差，可降低接近目標值時由感測器雜訊造成的抖動。                                                                              |
| `setErrorIntegralThreshold(eit)`   | 限制積分累積只發生在 `error` 的絕對值小於或等於此閾值時，讓積分作用集中在接近目標值的區域。                                                          |
| `setDerivativeFilter(alpha)`       | 對 D 項套用低通濾波。alpha 越高，濾波效果越強，對雜訊的放大越低。                                                                                      |
| `updateVelocity(...)`              | 使用可變時間頻率的速度型 PID，對部分增量式與速率型致動器可能具有更平滑的控制特性。                                                                     |
| `updateVelocityFixedRate(...)`     | 使用固定頻率的速度型 PID。                                                                                                                             |

---

## API 詳細說明

### `setFrequency(uint32_t hz)`

告知控制器預期的迴圈頻率（Hz）。使用固定頻率更新函式前，必須先呼叫此函式。

```cpp
pid.setFrequency(1000);  // 1 kHz control loop
```

### `setErrorDeadband(uint32_t edb)`

小於此值的誤差會被視為零，進而不產生對應的控制動作。適合用於感測器雜訊較大的情況。

```cpp
pid.setErrorDeadband(5);
```

### `setErrorIntegralThreshold(int32_t eit)`

只有當誤差的絕對值小於或等於此閾值時，積分項才會累積：

```text
|error| <= threshold
```

這是接近目標值時才啟用積分的閾值控制，不是一般所稱的「積分分離」（在大誤差時停用積分）。

```cpp
pid.setErrorIntegralThreshold(20);
```

### `setDerivativeFilter(uint32_t alpha)`

對微分項套用低通濾波，以降低高頻雜訊對 D 項的影響。

```cpp
pid.setDerivativeFilter(200);  // 實際數值需依系統特性進行調整
```

### `update` vs `updateFixedRate`

| 模式    | 函式                               | 適用情況                               |
| ----- | -------------------------------- | ---------------------------------- |
| 可變 dt | `update(target, input, dt_us)`   | 迴圈週期存在抖動的情況，例如無人機飛控                |
| 固定頻率  | `updateFixedRate(target, input)` | 嚴格固定週期的 Timer／中斷驅動迴圈；通常具有較低的單次運算成本 |

### 速度型 PID

```cpp
int32_t out = pid.updateVelocity(target, input, dt_us);

// 或

pid.setFrequency(1000);

int32_t out = pid.updateVelocityFixedRate(target, input);
```

速度型 PID 計算的是輸出的增量，因此在某些受控系統中可能具有較平滑的控制特性，也有助於降低微分突波（derivative kick）的影響。

---

## 抗積分飽和（Anti-Windup）

當輸出已經飽和，且目前誤差會使輸出進一步朝飽和方向增加時，積分狀態不會繼續更新。

這可以限制積分飽和，同時仍會使用當前週期計算出的積分貢獻。

建議搭配 `setOutputLimits()` 與 `setIntegralLimits()` 一起使用。

---

## 測試

v0.2 擴充了自動化測試，包含功能測試、邊界案例，以及固定頻率路徑的一致性測試。

```text
========================================
             TEST RESULT
========================================
Total : 73
PASS  : 73
FAIL  : 0
========================================
       ALL TESTS PASSED
========================================
```

測試涵蓋：

* P / I / D 行為

* 可變時間步長與 `dt = 0`

* Reset

* 輸出與積分限幅

* Anti-windup

* 定點數縮放

* 整數邊界條件

* 長時間運行與 dt 抖動

* 固定頻率與可變 dt 的一致性

* 新增 API 路徑（依測試套件涵蓋範圍）

---

## 效能測試（Benchmark）

測試平台：**Nologo ESP32-C3 SuperMini @ 160 MHz**

數值越低越好（每次 `update` 所需時間）。

### FixedPID v0.2（本版本）

| 模式                           |       P only |           PI |     Full PID |
| ---------------------------- | -----------: | -----------: | -----------: |
| **可變 dt** (`update`)         | **2.098 µs** | **3.110 µs** | **3.126 µs** |
| **固定頻率** (`updateFixedRate`) | **0.589 µs** | **1.897 µs** | **2.149 µs** |

### 與前一版 FixedPID 測試結果比較

較早期的公開數據（相同等級的平台／測試設定）：

| 模式             |    Full PID |
| -------------- | ----------: |
| 前一版 `update()` |    ~4.83 µs |
| **v0.2 可變 dt** | **3.13 µs** |
| **v0.2 固定頻率**  | **2.15 µs** |

相較於前一版 Full PID 數據，預估效能提升：

| 路徑    | 相較於 ~4.83 µs 的加速比 |
| ----- | ----------------: |
| 可變 dt |         **~1.5×** |
| 固定頻率  |         **~2.2×** |

### 與其他函式庫比較

來自先前在 ESP32-C3 上進行的同類型測試結果：

| 函式庫                     | Full PID 約耗時 |
| ----------------------- | -----------: |
| QuickPID                |    ~13.62 µs |
| AutoPID (r-downing)     |    ~18.74 µs |
| **FixedPID v0.2 可變 dt** | **~3.13 µs** |
| **FixedPID v0.2 固定頻率**  | **~2.15 µs** |

相對速度（Full PID，數值越高代表速度越快）：

| 比較對象                       | 約略倍數        |
| -------------------------- | ----------- |
| FixedPID 可變 dt vs QuickPID | **~4.4× 快** |
| FixedPID 可變 dt vs AutoPID  | **~6.0× 快** |
| FixedPID 固定頻率 vs QuickPID  | **~6.3× 快** |
| FixedPID 固定頻率 vs AutoPID   | **~8.7× 快** |

```text
Full PID 耗時（µs，越低越好）

AutoPID      |################## 18.74
QuickPID     |############# 13.62
FixedPID var |### 3.13
FixedPID fix |## 2.15
```

以上數據僅作為參考，並非對所有平台的絕對效能保證。實際結果會受到 MCU、時脈頻率、編譯器旗標與系統設定影響。

FastPID 與其他整數型 PID 函式庫在 8-bit AVR 平台上仍具有其優勢；FixedPID 則主要針對 32-bit 嵌入式系統，以及需要可變 `dt` 的應用進行設計與最佳化。

---

## 設計目標

1. 純整數 PID 運算

2. 可預測的數值行為

3. 明確的時間工作頻率控制（可變與固定頻率版本）

4. 實用的抗積分飽和與限幅功能

5. 在現代 MCU 上維持低運算開銷（例如 ESP32-C3）

6. 精簡、易讀且易於使用的 API

---

## 未來規劃（Roadmap）

可能的更新：

* 更多微分取樣方式，例如 derivative-on-measurement

* 設定點加權／斜坡

* 輸出變化率限制（Slew-rate limiting）

* 無擾動切換（Bumpless transfer）輔助

* 更多 MCU 效能測試（UNO、STM32 等）

* 更多文件與範例

---

## Acknowledgments / Feedback from v0.2

在前一版 v0.2 的實作與測試過程中，我發現了一些與整數運算、固定頻率計算以及測試覆蓋率相關的問題。

主要問題包括：

* `PID_SCALE` 在 P / I / D 增益計算中的使用方式不正確
* `setFrequency()` 存在潛在的除以零問題
* 速度型積分項存在精度損失
* 固定頻率微分計算存在精度損失
* 儲存前一次誤差值時使用的整數寬度不足
* `setErrorIntegralThreshold()` 的文件說明不一致
* 部分 API 尚未有完整的功能測試覆蓋

以上問題目前都已完成修正，同時也擴充了相對應的測試。

目前的測試套件除了基本功能之外，也增加了對以下邊界情況的測試：

* 頻率上下限
* 整數範圍極限
* Error Deadband
* Integral Threshold
* Derivative Filter
* 速度型積分的精度
* 固定頻率計算

前一版在測試與實際開發過程中發現的主要運算與實作問題，目前都已經完成修正。

### 目前狀態

目前 FixedPID 正在我的 **ESP32 無人機專案**上進行實際測試。

我目前**暫時不會在這個版本中放入最終的 Library 檔案**。

在正式發布更新後的 Library 檔案之前，我希望先將 FixedPID 實際整合至無人機的飛控系統中進行測試，確認 PID 在真實的飛行控制環境下能夠維持穩定且可靠的行為。

等到 FixedPID 在無人機上的實際測試完成後，我會再更新 Repository，加入最終版本的 Library 檔案以及完整文件。

這將會是下一個正式版本發布前的最後驗證階段。

---

## 授權

授權資訊請參閱儲存庫中的 license 檔案。
