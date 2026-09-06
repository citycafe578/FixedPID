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

* 積分分離閾值（Integral separation threshold）

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
| `setErrorIntegralThreshold(eit)`   | 積分分離：只有當 `error` 的絕對值達到或超過此閾值時才累積積分，可避免大型暫態期間積分過度累積。                                                          |
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

只有當誤差的絕對值達到或超過此閾值時，積分項才會累積。

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

## Acknowledgments / Feedback from v0.1

感謝所有嘗試過 v0.1 並在首版發布後提供回饋的各位。

**已採納的社群 (Arduino.Taipei) 建議：**

* **速度型 PID** — 新增 `updateVelocity()` 與 `updateVelocityFixedRate()`，以減少積分暴衝問題，並在某些場景下讓增益調整更平滑。

* **減少中間變數 / 更精簡的更新路徑** — 減少不必要的狀態並收緊 hot path 後，在效能回饋中獲得改善（尤其對 ESP32-C3 / RISC-V 具有關鍵影響）。

* **固定頻率高速路徑** — `setFrequency()` + `updateFixedRate()` 用於固定週期迴圈，降低每次運算成本。

* **實用的調參輔助** — 誤差死區、積分分離閾值、微分低通濾波，用於處理雜訊感測器與真實控制迴圈。

* **更廣的測試** — 測試套件擴充至 73 個案例，包含固定頻率一致性與長時間抖動測試。

**本次未採納的建議 (以及原因)：**

* **2的次方 (`2^N`) 增益/時間縮放 + 僅用位移運算** — 速度上確實提升了非常多，但會破壞現有的 `PID_SCALE = 1000` / `TIME_SCALE = 1000000` API 以及所有現有的調參、文件與測試。視為未來可能的 breaking change，而非 v0.2 的小改動。

* **極致的、針對特定架構的微優化 (sub-1 µs on C3)** — 原型路徑在 ESP32-C3 上已達到 <1 µs，但在其餘核心/處理器架構上並非明顯優勢。比起在單一 MCU 上的極限數字，我更偏好可攜性。

* **將延遲壓到 ~0.5 µs** — 對於常見的設備來說並無太多的需求；下一個優先事項是真實情況的測試，如實裝上無人機並評估飛行測試時的穩定性，而非進一步的優化。

**下一個重點**

將 FixedPID 整合進我的開源無人機專案中進行飛行測試。如果在空中的表現穩定，將會以 Arduino 函式庫的形式發布供更廣泛使用。進一步的功能 (例如更強的 derivative-on-measurement 預設值、feed-forward、notch 相關輔助) 將在驗證之後才會加入。

---

## 授權

授權資訊請參閱儲存庫中的 license 檔案。
