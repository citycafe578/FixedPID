# FixedPID

[English](README.md) | [繁體中文](README.zh-TW.md)

文件：[Usage guide](docs/USAGE.md) · [使用指南（繁體中文）](docs/USAGE.zh-TW.md)

一個為嵌入式系統設計的小型定點數 PID 控制器。

FixedPID 使用整數運算，並以 `int64_t` 作為中間計算型別，取代浮點數運算。它適合需要可預測的整數計算、低運算開銷，以及明確控制取樣時間的應用。

## 功能特色

* 支援 P / PI / PID 控制

* 定點數增益表示（`PID_SCALE = 1000`）

* `int32_t` 輸入與輸出

* `int64_t` 中間計算

* 可變時間步長（`update`）

* 固定頻率路徑，預先計算時間縮放（`updateFixedRate`）

* 速度型 PID（`updateVelocity` / `updateVelocityFixedRate`）

* 輸出限幅

* 積分限幅

* 抗積分飽和（Anti-windup）

* 誤差死區（Error deadband）

* 積分閾值控制（Integral threshold control）

* 微分低通濾波

* 控制器狀態重置

* `dt = 0` 保護
* 輸入範圍保護：任何 `int32_t` 輸入、增益與時間步長都以飽和運算處理，不會溢位

* 專為嵌入式系統設計

## 目前狀態

**版本：v1.0.0**

第一個穩定版本。修正了 v0.2.1 已知的錯誤，新增輸入範圍保護、使用指南與自動化測試，而且比以前更快。公開 API 不變，唯一差別是 `setFrequency()` 現在回傳 `bool`。詳見[更新紀錄](#更新紀錄)。

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

### 固定頻率模式

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
| `updateFixedRate(target, input)` | 固定頻率運算路徑（預先計算時間縮放） |

### v0.2 新增

| 函式                               | 用途                                                                                                                                                 |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| `setFrequency(hz)`                 | 設定 `updateFixedRate` / `updateVelocityFixedRate` 使用的固定迴圈頻率，並預先計算 Ki/Kd 的時間縮放。                         |
| `setErrorDeadband(edb)`            | 忽略 ±deadband 範圍內的微小誤差，可降低接近目標值時由感測器雜訊造成的抖動。                                                                              |
| `setErrorIntegralThreshold(eit)`   | 限制積分累積只發生在 `error` 的絕對值小於或等於此閾值時，讓積分作用集中在接近目標值的區域。                                                          |
| `setDerivativeFilter(alpha)`       | 對 D 項套用低通濾波。alpha 越低，濾波越強（`1000` = 不濾波，`0` = 保持前一次濾波值）。                                                                                      |
| `updateVelocity(...)`              | 使用可變時間頻率的速度型 PID，對部分增量式與速率型致動器可能具有更平滑的控制特性。                                                                     |
| `updateVelocityFixedRate(...)`     | 使用固定頻率的速度型 PID。                                                                                                                             |

---

## API 詳細說明

### `setFrequency(uint32_t hz)`

告知控制器預期的迴圈頻率（Hz）。使用固定頻率更新函式前，必須先呼叫此函式。

有效範圍為 1 ～ 1,000,000 Hz；範圍外會回傳 `false` 並停用固定頻率路徑（此時 `updateFixedRate()` / `updateVelocityFixedRate()` 一律回傳 0），請檢查回傳值。頻率不能整除 1,000,000 時（例如 3000 Hz），積分的時間基準會以餘數補償，不會累積誤差。

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
| 固定頻率  | `updateFixedRate(target, input)` | 嚴格固定週期的 Timer／中斷驅動迴圈；不需要每次傳入 dt |

### 速度型 PID

```cpp
int32_t out = pid.updateVelocity(target, input, dt_us);

// 或

pid.setFrequency(1000);

int32_t out = pid.updateVelocityFixedRate(target, input);
```

速度型 PID 計算的是輸出的增量，因此在某些受控系統中可能具有較平滑的控制特性。

注意：速度型的 D 項同樣作用在誤差上，設定點階躍時仍會出現微分突波；若要避免，需要 derivative-on-measurement（列於 Roadmap）。速度型的輸出累積值由 `setOutputLimits()` 限幅，因此**不使用** `setIntegralLimits()`。

---

## 抗積分飽和（Anti-Windup）

當輸出已經飽和，且目前誤差會使輸出進一步朝飽和方向增加時，積分狀態不會繼續更新。

這可以限制積分飽和，同時仍會使用當前週期計算出的積分貢獻。

建議搭配 `setOutputLimits()` 與 `setIntegralLimits()` 一起使用。

---

## 測試

v1.0.0 包含自動化測試，包含功能測試、邊界案例，以及固定頻率路徑的一致性測試。

```text
========================================
             TEST RESULT
========================================
Total : 140
PASS  : 140
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

* 積分狀態限幅與反向解除

* 速度型 P / I / D 的餘數補償

* 頻率無法整除 1,000,000 時的固定頻率時間基準

* `setFrequency()` 回傳值與顛倒的限幅參數

* 輸入範圍保護：每條更新路徑在極端增益、誤差與時間步長下的行為

### 在 PC 上執行測試

`make test` 會以 AddressSanitizer 與 UBSan 編譯並執行單元測試與隨機化的模糊測試（需要 `make` 與 GCC 或 Clang）。模糊測試會把每條更新路徑與 128 位元參考模型比對、對四個更新函式丟入隨機的極端設定，並檢查可變 dt 與固定頻率是否一致、`reset()` 是否能還原成全新狀態。相同的檢查會在每次 push 時於 GitHub Actions 執行。詳見[使用指南](docs/USAGE.zh-TW.md#14-執行測試)。

---

## 效能測試（Benchmark）

數值越低越好（每次 `update` 所需時間）。

### FixedPID v1.0.0（ESP32）

測試平台：**ESP32（ESP32-D0WD-V3，雙核 @ 240 MHz）**，同一份韌體上 140 項測試全部通過。

| 模式                           |       P only |           PI |     Full PID |
| ---------------------------- | -----------: | -----------: | -----------: |
| **可變 dt** (`update`)         | **1.028 µs** | **1.675 µs** | **2.180 µs** |
| **固定頻率** (`updateFixedRate`) | **1.033 µs** | **1.620 µs** | **2.063 µs** |

與同一塊 ESP32 上的 v0.2.2 比較。v1.0.0 加上了輸入範圍保護，但仍然更快，因為除以 1000 與 1e9 的 64 位元除法改用了 32 位元硬體除法：

| 模式（Full PID） |   v0.2.2 | v1.0.0 |
| ------------ | -------: | -----: |
| 可變 dt        | 2.424 µs | **2.180 µs** |
| 固定頻率         | 2.462 µs | **2.063 µs** |

ESP32 沒有 64 位元硬體除法，所以固定頻率路徑在 ESP32 上省下的時間不多；下方 ESP32-C3 的差距則大得多。

### ESP32-C3 結果（v0.2.1）

測試平台：**Nologo ESP32-C3 SuperMini @ 160 MHz**

| 模式                           |       P only |           PI |     Full PID |
| ---------------------------- | -----------: | -----------: | -----------: |
| **可變 dt** (`update`)         | **2.098 µs** | **3.110 µs** | **3.126 µs** |
| **固定頻率** (`updateFixedRate`) | **0.589 µs** | **1.897 µs** | **2.149 µs** |

### 與前一版 FixedPID 測試結果比較

較早期的公開數據（相同等級的平台／測試設定）：

| 模式             |    Full PID |
| -------------- | ----------: |
| 前一版 `update()` |    ~4.83 µs |
| **v0.2.1 可變 dt** | **3.13 µs** |
| **v0.2.1 固定頻率**  | **2.15 µs** |

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
| **FixedPID v0.2.1 可變 dt** | **~3.13 µs** |
| **FixedPID v0.2.1 固定頻率**  | **~2.15 µs** |

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

預計在之後的大版本更新：

* 更多微分取樣方式，例如 derivative-on-measurement

* 設定點加權／斜坡

* 輸出變化率限制（Slew-rate limiting）

* 無擾動切換（Bumpless transfer）輔助

* 更多 MCU 效能測試（UNO、STM32 等）

* 更多文件與範例

---

## 更新紀錄

### v1.0.0

第一個穩定版本。公開 API 與 v0.2.2 相同。

新增：

* **輸入範圍保護**：任何 `int32_t` 輸入與增益、任何 `uint32_t` 時間步長現在都不會造成 `int64_t` 溢位或未定義行為；會溢位的中間乘積改為飽和處理，輸出一定落在輸出限幅內。以前在極端增益、誤差或時間步長下可能無聲溢位。
* **速度型餘數**：可變 dt 的微分餘數現在與固定頻率路徑使用相同的累積規則，兩者結果完全一致。
* **ESP32 上更快**：除以 1000 與 1e9 的 64 位元除法改用 32 位元硬體除法，運算元明確很小時略過乘法溢位檢查。ESP32 實測（可變 dt／固定頻率）：P 1.03／1.03 µs、PI 1.68／1.62 µs、完整 PID 2.18／2.06 µs。
* **測試**：由 125 項增加到 140 項。新增可在 PC 上執行的測試、對照 128 位元參考模型並加上 sanitizer 的模糊測試（`make test`），以及 GitHub Actions 工作流程。
* **文件與封裝**：新增[使用指南](docs/USAGE.zh-TW.md)（英文與正體中文）、`library.properties`，並把每個範例改放進各自的資料夾（`examples/<名稱>/<名稱>.ino`，Arduino IDE 需要這個結構）。

### v0.2.2

修正：

* **積分限幅**：內部積分狀態現在會和積分項一起被限幅，不會再在 `setIntegralLimits()` 之後繼續累積；誤差反向時會立刻開始解除。同時消除預設限幅下長時間運行可能發生的 `ki × integral` 之 `int64_t` 溢位。
* **速度型 P 與 D**：P、D 項不足 1 LSB 的餘數現在會帶到下一次計算（I 項原本就有）。以前 `Kp` 很小、誤差變化緩慢時，輸出可能完全不動。
* **固定頻率時間基準**：`updateFixedRate()` / `updateVelocityFixedRate()` 的積分不再捨去 `1,000,000 / hz` 的小數部分。300 kHz 時積分原本約少 10 %。
* **`setFrequency()`**：現在回傳 `bool`（0 Hz 或高於 1 MHz 回傳 `false`），不再靜默失敗。
* `setOutputLimits()` / `setIntegralLimits()` 在 `min > max` 時會自動對調。
* README：修正繁體中文版 `setDerivativeFilter` 說明顛倒的問題（alpha 越低濾波越強），並移除「速度型能降低微分突波」的說法。

變更：

* 重複的更新程式碼合併為共用函式，並移除 P-only 快速路徑。
* `FixedPID.cpp` 不再引入 `Arduino.h`。
* 測試由 104 項增加到 125 項。

### v0.2.1

修正 v0.2 檢視時發現的問題：

* `PID_SCALE` 在 P / I / D 增益計算中的使用方式不正確。
* `setFrequency()` 可能除以零；現在會拒絕 0 Hz 與高於 1 MHz 的頻率。
* 速度型積分項精度損失；現在餘數會帶到下一次計算。
* 固定頻率微分精度損失；現在改由 `kd × hz` 計算。
* 儲存前一次誤差值時使用的整數寬度不足。
* `setErrorIntegralThreshold()` 的文件說明不一致。
* 移除未使用的 `<math.h>`。
* 測試由 73 項增加到 104 項。

### v0.2

* 新增 `setFrequency` 與 `updateFixedRate`（預先計算時間縮放的固定頻率路徑）。
* 新增速度型 PID：`updateVelocity`、`updateVelocityFixedRate`。
* 新增調校輔助：`setErrorDeadband`、`setErrorIntegralThreshold`、`setDerivativeFilter`。
* 精簡更新路徑，減少中間變數。
* 新增繁體中文 README 與另外兩個範例。
* 測試擴充到 73 項。

### v0.1

* 第一個公開版本：P / PI / PID、定點增益、`int32_t` 輸入輸出、`int64_t` 中間運算、可變時間步長、輸出與積分限幅、抗積分飽和、`reset()` 與 `dt = 0` 保護。

---

## 目前狀態

此版本已包含函式庫檔案（`src/`），140 項測試在 ESP32 上全部通過。

FixedPID 也正在我的 **ESP32 無人機專案**上進行實際驗證。和任何控制函式庫一樣，請在自己的硬體上確認行為後再使用。

FixedPID 以現代 32 位元 MCU 為主要目標，已在 ESP32 上測試；Arduino UNO 等 8 位元 AVR 板並非目標平台，也尚未測試。

---

## 授權

授權資訊請參閱儲存庫中的 license 檔案。
