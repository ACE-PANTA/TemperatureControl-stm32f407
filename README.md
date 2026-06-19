# STM32 温度控制系统说明

本项目基于 STM32F407，实现加热体温度采集、手动/自动输出控制、串口屏显示、串口/TCP 参数配置，以及以太网诊断。系统面向具有明显热惯性和纯滞后的温控对象。

## 1. 系统组成

| 模块 | 引脚/接口 | 说明 |
|---|---|---|
| NTC 温度反馈 | PA0 / ADC1 | 控制对象温度反馈 |
| DS18B20 | PB5 | 主板或环境温度采集 |
| 加热输出 | PE5 | 加热 MOS 控制 |
| 风扇 PWM | TIM9 CH2 | 负输出时驱动散热风扇 |
| 按键 | PD1~PD4 | 增加、减少、步进、自动/手动 |
| HMI 串口屏 | USART3 / PD8, PD9 | 本地显示与交互 |
| 调试串口 | USART1 / PA9, PA10 | 参数配置与状态查询 |
| 以太网 | RMII + LAN8720A | TCP 配置、状态上报、ping 诊断 |
| PHY 复位 | PB0 | LAN8720A reset |

默认网络参数：

| 项目 | 默认值 |
|---|---|
| IP | `192.168.1.100` |
| 网关 | `192.168.1.1` |
| 子网掩码 | `255.255.255.0` |
| TCP 端口 | `8000` |
| MAC | `02:00:00:00:00:01` |

PC 直连测试时，建议将 PC 设置为 `192.168.1.10 / 255.255.255.0`，然后执行：

```text
ping 192.168.1.100
nc 192.168.1.100 8000
```

## 2. 控制原理

系统有两种工作模式：

| 模式 | 含义 |
|---|---|
| `MAN` | 手动模式，直接使用手动 PWM 输出 |
| `AUTO` | 自动模式，由控制算法计算输出 |

自动模式下又分为两个控制阶段：

| 阶段 | 含义 |
|---|---|
| `TRAN` | 变温阶段，使用位置式 PID，适合较大误差时快速接近目标 |
| `FINE` | 微调阶段，误差进入近区后启用融合 PID，小步调整，减少目标附近振荡 |

输出范围为 `-100 ~ 100`：

- 正值：加热输出。
- 负值：关闭加热并驱动风扇散热。
- 微调阶段输出下限为 0，避免目标附近频繁启动风扇。

### 2.1 变温 PID

变温阶段使用位置式 PID：

```text
u = Kp * e + Ki * integral(e) + Kd * de
```

为适应热系统的大惯性，加入了积分分离、自适应积分、积分限幅和过冲泄放。大误差时减少或清除积分，目标附近再逐步恢复积分，避免上升阶段积分堆积导致严重过冲。

### 2.2 微调 PID

当 `fine_enable=1` 且误差进入 `fine_entry_max` 范围内，系统进入 `FINE`。微调阶段同时计算增量式修正和位置式修正，并按误差大小融合，输出变化受 `fine_range` 限制。

微调阶段的目标是稳定地逼近设定值，而不是快速大幅修正。

### 2.3 Smith 预估

热系统常见问题是输出变化后，传感器要延迟一段时间才能看到温度变化。Smith 预估器用一阶模型估算当前温度趋势，让 PID 看到一个更接近“当前过程状态”的反馈值。

默认关闭。启用后：

```text
PFB = FB + predictor_lead
```

其中：

- `FB` 是实测温度。
- `PFB` 是 PID 实际使用的预估反馈。
- `predictor_lead` 由模型增益、时间常数、纯滞后和混合系数计算，并受最大修正量限制。

调参时应观察 `GET=STATE` 中的 `FB` 和 `PFB`。如果 `PFB` 超前过多，说明 Smith 参数过激，应降低 `blend` 或 `maxlead`。

### 2.4 Smith 与 PID 的配合方式

当前控制链路是：

```text
PWM 输出历史
   -> Smith 热模型
   -> 得到预估反馈 PFB
   -> PID 使用 target_temp - PFB 计算误差
   -> PID 输出新的 PWM
```

也就是说，PID 的结构和参数仍然有效，Smith 只改变 PID 看到的反馈量：

```text
未启用 Smith: error = target_temp - FB
启用 Smith:   error = target_temp - PFB
```

这样做的目的，是在传感器尚未完全反映温度变化时，让 PID 提前感知加热或散热趋势，减少“等温度反馈变了才开始收输出”的迟滞。实际温度采样仍然参与闭环，Smith 只是提供有限幅的超前修正，因此模型不准时可以通过 `blend` 和 `maxlead` 降低影响。

## 3. 参数与默认值

### 3.1 基本参数

| 参数 | 默认值 | 范围 | 说明 |
|---|---:|---|---|
| `manual_flag` | 1 | 0/1 | 0=自动，1=手动 |
| `target_temp` | 30.0 | -10~100 | 自动模式目标温度 |
| `manual_pwm` | 0 | -100~100 | 手动输出 |
| `step_value` | 1 | 1/5/10 | 按键步进 |

### 3.2 变温参数

| 参数 | 默认值 | 范围 | 说明 |
|---|---:|---|---|
| `tran_kp` | 3.0 | 0.1~50 | 比例系数 |
| `tran_ki` | 0.3 | 0~5 | 积分系数 |
| `tran_kd` | 1.0 | 0~10 | 微分系数 |
| `tran_interval` | 3 | 1~60 s | 输出更新间隔 |
| `tran_sep_threshold` | 10.0 | 1~30 | 积分分离阈值 |
| `tran_i_min_scale` | 0.20 | 0~1 | 自适应积分最小比例 |
| `tran_i_full_error` | 2.0 | 0.1~10 | 满积分误差范围 |
| `tran_i_limit` | 80.0 | 1~300 | 积分限幅 |
| `tran_i_overshoot_leak` | 0.85 | 0~1 | 过冲/死区积分泄放 |

### 3.3 微调参数

| 参数 | 默认值 | 范围 | 说明 |
|---|---:|---|---|
| `fine_enable` | 1 | 0/1 | 是否启用微调 |
| `fine_kp` | 1.5 | 0.1~20 | 微调比例 |
| `fine_ki` | 0.1 | 0~3 | 微调积分 |
| `fine_kd` | 2.0 | 0~10 | 微调微分 |
| `fine_interval` | 8 | 1~60 s | 微调更新间隔 |
| `fine_range` | 5.0 | 1~20 | 单次最大输出变化 |
| `fine_entry_min` | 1.0 | 0.1~5 | 近区辅助范围 |
| `fine_entry_max` | 3.0 | 1~10 | 进入微调的误差范围 |
| `stable_window` | 20 | 10~120 s | 稳定性观察窗口 |
| `stable_delta` | 1.0 | 0.2~5 | 窗口内稳定波动阈值 |

### 3.4 Smith 参数

| 参数 | 默认值 | 范围 | 说明 |
|---|---:|---|---|
| `smith_enable` | 0 | 0/1 | 是否启用 Smith 预估 |
| `smith_gain` | 40.0 | 1~200 | 100% 输出对应的模型稳态温升 |
| `smith_tau` | 120 | 5~3600 s | 一阶模型时间常数 |
| `smith_delay` | 30 | 0~180 s | 纯滞后时间 |
| `smith_blend` | 0.70 | 0~1 | 预估混合比例 |
| `smith_max_lead` | 8.0 | 0.5~30 | 最大超前修正量 |

### 3.5 共享参数

| 参数 | 默认值 | 范围 | 说明 |
|---|---:|---|---|
| `pid_deadband` | 0.3 | 0.1~2.0 | 目标附近死区 |

## 4. 通信协议

串口和 TCP 使用同一套文本协议。

```text
请求: !CMD=VALUE\r\n
请求: !CMD=VALUE*XX\r\n
响应: !ACK=OK*XX\r\n
响应: !ACK=ERR*XX\r\n
```

`*XX` 是从 `!` 到 `*` 前一字符的异或校验。设备接收时允许不带校验，设备发送时会带校验。

### 4.1 控制命令

| 命令 | 示例 | 说明 |
|---|---|---|
| `MODE` | `!MODE=AUTO` / `!MODE=MAN` | 自动/手动切换 |
| `TEMP` | `!TEMP=58.0` | 设置目标温度 |
| `PWM` | `!PWM=50` | 设置手动 PWM，仅手动模式有效 |
| `STEP` | `!STEP=5` | 设置按键步进 |

### 4.2 PID 调参

| 命令 | 格式 |
|---|---|
| `TRAN` | `!TRAN=kp,ki,kd[,interval,sep]` |
| `IADAPT` | `!IADAPT=min_scale,full_error,limit,leak` |
| `FINE` | `!FINE=kp,ki,kd[,interval,range,entry_min,entry_max,stable_window,stable_delta]` |
| `FINEEN` | `!FINEEN=0` 或 `!FINEEN=1` |
| `DEADBAND` | `!DEADBAND=0.3` |
| `SMITH` | `!SMITH=enable,gain,tau,delay,blend,maxlead` |
| `SMITHEN` | `!SMITHEN=0` 或 `!SMITHEN=1` |

Smith 保守启用示例：

```text
!SMITH=1,40,120,30,0.5,5
```

### 4.3 查询命令

| 命令 | 说明 |
|---|---|
| `!GET=STATE` | 当前模式、阶段、PWM、目标、实测反馈、预估反馈 |
| `!GET=PID` | 返回 TRAN、FINE、DEADBAND、SMITH |
| `!GET=TRAN` | 查询变温参数 |
| `!GET=IADAPT` | 查询自适应积分参数 |
| `!GET=FINE` | 查询微调参数 |
| `!GET=SMITH` | 查询 Smith 参数 |
| `!GET=NET` | 查询网络参数 |
| `!GET=ETH` | 查询以太网状态 |
| `!GET=CONFIG` | 返回主要配置 |

`GET=STATE` 示例：

```text
STATE=MODE:0,PHASE:TRAN,PWM:35,GOAL:580,FB:575,PFB:579
```

字段说明：

| 字段 | 说明 |
|---|---|
| `MODE` | 0=自动，1=手动 |
| `PHASE` | `MAN` / `TRAN` / `FINE` |
| `PWM` | 当前输出，-100~100 |
| `GOAL` | 目标温度 x10 |
| `FB` | 实测温度 x10 |
| `PFB` | PID 使用的反馈 x10 |

### 4.4 保存与恢复

| 命令 | 说明 |
|---|---|
| `!SAVE=1` | 立即保存当前配置到 Flash |
| `!RESET=1` | 恢复默认配置，保留 MAC |

参数修改后也会延迟自动保存。

## 5. 以太网诊断

`!GET=ETH` 示例：

```text
ETH=LINK:1,PHY:0,ERR:0,PHYID:0007C0F1,RX:5,TX:5,ARP:2,ICMP:3,TCP:0
```

| 字段 | 说明 |
|---|---|
| `LINK` | 1=网线连接，0=未连接 |
| `PHY` | 检测到的 PHY 地址，255 表示未找到 |
| `ERR` | 以太网错误码 |
| `PHYID` | PHY 芯片 ID |
| `RX/TX` | 收发帧计数 |
| `ARP` | ARP 回复计数 |
| `ICMP` | ping 回复计数 |
| `TCP` | 1=有 TCP 客户端连接 |

错误码：

| 代码 | 含义 |
|---:|---|
| 0 | 正常 |
| 1 | 未找到 PHY |
| 2 | 无链路 |
| 3 | 自协商超时 |
| 4 | ETH 未初始化 |

## 6. 调参建议

1. 先关闭 Smith，仅调 PID。

```text
!SMITHEN=0
!MODE=AUTO
!TEMP=目标温度
```

2. 大误差升温慢：适当增加 `tran_kp` 或 `tran_ki`。
3. 过冲明显：降低 `tran_kp`，增大 `tran_kd`，或增大 `tran_interval`。
4. 目标附近振荡：减小 `fine_range`，增大 `fine_interval`，或适当增大 `pid_deadband`。
5. 明显纯滞后导致“反应晚”：再启用 Smith，从小 `blend` 和小 `maxlead` 开始。

建议启用 Smith 的初始值：

```text
!SMITH=1,40,120,30,0.3,4
```

如果 `PFB` 比 `FB` 超前过多，先降低 `blend`；如果仍过冲，再降低 `maxlead` 或增大 `tau`。

## 7. 工程结构

```text
USER/
  main.c              系统初始化和主循环
  app_config.c/.h     参数配置、命令分发、Flash 配置入口
  flash_params.c/.h   Flash 持久化
  pid_control.c/.h    PID、Smith 预估、加热/风扇输出
  app_comm.c/.h       串口/TCP 命令帧和状态广播
  app_input.c/.h      按键处理

HARDWARE/
  ETH/                LAN8720A RMII 以太网驱动
  ADC/                NTC 采样
  DS18B20/            DS18B20 温度采集
  HMI/                串口屏
  PWM/                PWM 输出
  TIMER/              定时器
```

Keil 工程文件：

```text
USER/DS18B20.uvprojx
```

目标芯片：STM32F407VETx。

## 8. 常见问题

| 现象 | 排查方向 |
|---|---|
| ping 不通 | 先查 `!GET=ETH` 的 `LINK/ERR/RX/TX/ARP/ICMP` |
| TCP 连不上 | 确认能 ping 通，且连接端口为 `8000` |
| 手动 PWM 无效 | `PWM` 只在 `MODE=MAN` 下有效 |
| 自动模式升温慢 | 调 `TRAN`，优先检查 `tran_kp/tran_ki/tran_interval` |
| 目标附近抖动 | 调 `FINE`、`DEADBAND`，必要时关闭 Smith 验证 |
| 启用 Smith 后过冲 | 降低 `smith_blend` 或 `smith_max_lead` |
