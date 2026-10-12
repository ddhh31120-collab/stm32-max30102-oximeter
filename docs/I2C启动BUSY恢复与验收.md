# I2C 启动 BUSY：恢复与验收

日期：2026-10-12。适用：本工程 STM32F103C8T6、I2C1、PB6=SCL、PB7=SDA，单主机总线。

## 已知现场与未确定的原因

用户最初反馈重新下载或重启调试后进入 Error_Handler，config_status=HAL_BUSY；随后反馈完整断电后也仍然 BUSY。任务与队列计数为零，故障在传感器初始化阶段，不是满队列实验导致任务无法运行。

不能仅凭 HAL_BUSY 区分：外部线路/模块持续拉低、复位打断通信、STM32 I2C 内部状态异常。也不能把此前删除寄存器观察项后恢复，直接当作所有复发的原因。

ST 的 ES096 rev15 第 2.8.7 节描述了模拟滤波器导致 BUSY 卡住、普通复位不足以解除的问题；NXP UM10204 描述 SDA 卡低时的有限时钟恢复。这些是恢复方案的依据，不是本板根因已确认的证据。

- [ST ES096](https://www.st.com/resource/en/errata_sheet/es096-stm32f101x8b-stm32f102x8b-and-stm32f103x8b-mediumdensity-device-limitations-stmicroelectronics.pdf)
- [NXP UM10204](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)

## 本次修改

只修改 MAX30102 驱动及其头文件，保留用户的 main、任务、队列和测试代码。原配置写入顺序仍由 MAX30102_ConfigureOnce 执行；公开接口仍为 MAX30102_Init(void)。

```text
启动时执行完整传感器配置
  ├─ 成功 → 正常创建队列、任务
  └─ 失败
       ├─ 普通 NACK 等错误 → 返回原错误
       └─ BUSY 或超时 → 最多一次总线恢复
              ├─ 恢复失败 → 返回错误，保留诊断
              └─ 恢复成功 → 从传感器复位开始重做完整配置一次
                                  └─ 返回这次配置的真实结果
```

恢复时先停用 I2C，将两脚改为开漏输出并释放。SCL 拉不高则有限等待后失败；若 SDA 卡低，最多发送 9 个时钟尝试释放。两线均释放后执行 ST 勘误中的电平转换与检查，再恢复复用开漏模式，通过 HAL_I2C_Init 完成 SWRST、时序恢复和 PE 使能，检查 BUSY 已解除后才重试传感器配置。

每个电平等待上限 5 次 HAL_Delay(1)，时钟脉冲最多 9 个，没有无限恢复循环。前提是 HAL 时基正常、IRQ 已使能；此函数只由启动流程使用，不能当作运行中可随意调用的恢复接口。暂未加入运行中自动复位传感器，避免静默清 FIFO、混用旧算法历史或影响任务间数据顺序。

真实的断线、缺少上拉、模块掉电、SCL/SDA 短路等问题，软件恢复不能代替硬件修复。

## 如何下载与观察

1. 退出当前调试，用本工程重新编译、下载，重新进入调试并运行。助手已完成构建，未替用户操作下载。
2. 暂保持 test_pause_sample=0、test_pause_receive=0，先验证启动与正常采样，不同时运行满队列实验。
3. Watch 添加 config_status 和 max30102_recovery，并展开后者。它们是 RAM 数据；不要重新添加实时 SR1/SR2/DR，也不要展开 I2C 外设寄存器窗口。
4. config_status=HAL_OK 且 sample_count 持续增加、速率接近 100、VOFA 更新，才算本次板端启动通过。随后进行几次重新下载/重启调试以及一次完整上电验证，记录恢复次数与返回值。单次恢复不等于根因已消除。

## max30102_recovery 的字段

| 字段 | 含义 |
|---|---|
| attempts | 本次运行中恢复函数进入次数；正常启动为 0，本次单次初始化最多恢复 1 次 |
| first_init_status | 第一次完整配置尝试的返回状态 |
| bus_status | 总线恢复返回状态；只有 attempts 非零才解读该字段 |
| retry_init_status | 恢复成功后重试配置的结果；只有 bus_status=HAL_OK 且确实恢复过才解读 |
| stage | 恢复最后执行到的步骤；失败时停留在失败步骤 |
| error_before / sr2_before | 失败后、开始恢复前捕获的 HAL 错误及 SR2 快照 |
| scl_before / sda_before | 恢复前实际引脚读数，1 为高、0 为低 |
| scl_after / sda_after | 恢复流程结束时的引脚读数，1 为高、0 为低 |
| clock_pulses | 因 SDA 卡低而尝试的恢复时钟数，0～9；不包含随后勘误序列里的转换 |

不要仅凭恢复前引脚为低就认定外部短路：故障中的 STM32 自己也可能拉低线路。判断外部持续拉低，更应看禁用 I2C、以 GPIO 释放后是否仍失败，即 stage。

| stage（十进制） | 枚举尾部 | 解释 |
|---:|---|---|
| 0 | NOT_RUN | 没有执行恢复 |
| 1 | GUARD | 句柄仍被占用或调用上下文不合适，没有强行操作引脚 |
| 2 | RELEASE_SCL | 释放 SCL 后仍读不到高电平，优先查 SCL/模块供电、接线和上拉 |
| 3 | RELEASE_SDA | SDA 经有限时钟后仍无法释放，查 SDA/模块供电、接线和上拉 |
| 4～7 | SDA_LOW / SCL_LOW / SCL_HIGH / SDA_HIGH | 勘误序列中的电平检查失败 |
| 8 | REINIT | HAL 重新初始化 I2C 失败 |
| 9 | VERIFY_IDLE | 重初始化后 BUSY 仍未解除 |
| 10 | DONE | 总线恢复成功；仍需看 retry_init_status 和最终 config_status |

原 first_config_fail_* 仍保存首次配置失败现场。即使重试成功，它们仍可能显示最初的 HAL_BUSY；这不是最终结果，最终看 config_status。

## 验证与回退

- Keil：0 错误、0 警告；构建日志 MDK-ARM/i2c_recovery_build.log。
- GCC 主机故障注入：覆盖正常启动不恢复、BUSY/超时恢复、SDA 第 3 个时钟释放、SCL/SDA 持续卡低、NACK 不误恢复、部分配置失败从头重试、二次失败不无限循环、BUSY 不消失、HAL 重初始化失败及锁/中断上下文保护。
- 主机模拟不能验证芯片模拟滤波器、上拉、电平和时序；上述板端验收尚待用户执行。
- 备份：D:\watch yuanma\OV-Watch\Software\backups\MAX30102_i2c_before_fix_20261012_0c6df582，包含修改前的 max30102.c 和 max30102.h。需要回退时成对恢复这两个文件，再重新构建。

## 主机测试复现

在 MAX30102 项目目录用 PowerShell 执行（电脑安装的 GCC 路径）：

```powershell
$testExe = Join-Path $env:TEMP 'max30102_recovery_test.exe'
& 'C:\msys64\ucrt64\bin\gcc.exe' -std=c11 -Wall -Wextra -Werror -I tests/host -I Core/Inc tests/host/max30102_recovery_test.c Core/Src/max30102.c -o $testExe
if ($LASTEXITCODE -eq 0) { & $testExe }
```

测试使用实际驱动 C 文件和独立的模拟 HAL。tests/host 下的文件不加入 Keil 固件工程。
