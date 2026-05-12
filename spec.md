# MotionIEC 运动控制库架构分析与注塑机液压控制改造方案
 
---
 
## 一、项目概述
 
MotionIEC 是一个基于 **PLCopen Motion Control** 标准（Part 1 & Part 2）的运动控制库，设计用于与 **Beremiz** 等 IEC 61131-3 PLC 运行时集成。代码由 XSLT 模板（YSL2）自动生成，实现了 PLCopen 标准定义的完整功能块集合。
 
### 文件组成
 
| 文件 | 大小 | 功能 |
|------|------|------|
| `pous.xml` | 5167行 | IEC 61131-3 接口定义（PLCopen XML格式），定义所有功能块的输入/输出变量、数据类型、枚举等 |
| `MotionKernel.c` | 10374行 | 运动控制内核实现，包含数据结构、轨迹生成、轴状态机、所有功能块的C实现 |
| `MotionKernel.h` | 50行 | 功能块函数声明（`__mcl_cmd_MC_*` 系列） |
| `motion.c` | 24行 | PLC运行时集成入口（Init/Cleanup/Retrieve/Publish 四个生命周期函数） |
 
---
 
## 二、架构层次分析
 
### 整体分层架构
 
```
┌─────────────────────────────────────────────────────────┐
│                   PLC 应用层                              │
│         （用户编写的 ST/FBD/LD/SFC 程序）                  │
│     调用 MC_Power, MC_MoveAbsolute, MC_MoveVelocity 等    │
├─────────────────────────────────────────────────────────┤
│               IEC 接口层 (pous.xml)                       │
│   PLCopen 标准功能块定义：输入/输出变量、数据类型、枚举      │
│   提供标准化的 IEC 61131-3 接口给 PLC 应用程序调用          │
├─────────────────────────────────────────────────────────┤
│              功能块实现层 (__mcl_cmd_MC_*)                 │
│   每个 IEC 功能块对应一个 C 函数实现                        │
│   处理 Execute/Enable 边沿检测、命令注册、状态反馈          │
├─────────────────────────────────────────────────────────┤
│               轴命令执行层 (__MK_ComputeAxis)              │
│   命令通道管理、轴状态机、运动模式切换                      │
│   命令快照(snapshot)、缓冲/混合模式处理                     │
├─────────────────────────────────────────────────────────┤
│              轨迹生成层 (GenTraj / OTG)                    │
│   在线轨迹生成(Online Trajectory Generation)               │
│   2阶/3阶速度控制、位置控制、力矩控制                       │
│   凸轮插补、齿轮耦合、Profile插补                           │
├─────────────────────────────────────────────────────────┤
│            PLC 运行时集成层 (motion.c)                     │
│   Init → Retrieve → [PLC Task 执行] → Publish 周期循环    │
├─────────────────────────────────────────────────────────┤
│                 驱动器/硬件抽象层                           │
│   通过 axis_s 结构体的 Raw 变量与驱动器交换数据              │
│   RawPositionSetPoint / ActualRawPosition / RawTorque等   │
└─────────────────────────────────────────────────────────┘
```
 
---
 
## 三、核心模块分析
 
### 3.1 数据类型与枚举定义 (MotionKernel.c:58-117, pous.xml)
 
**IEC 接口层**（pous.xml）定义了标准数据类型，供 PLC 应用程序使用：
 
| 类型 | 说明 |
|------|------|
| `AXIS_REF` | 轴引用（INT），轴实例的索引标识 |
| `MC_BUFFER_MODE` | 缓冲模式：Aborting/Buffered/BlendingLow/High/Previous/Next |
| `MC_DIRECTION` | 运动方向：绝对位置/正/负/最短路径/当前方向 |
| `MC_HOMING_PROCEDURES` | 回零方式：直接/限位开关/参考脉冲/绝对等8种 |
| `MC_EXECUTION_MODE` | 执行模式：立即/排队 |
| `MC_SOURCE` | 数据源：实际值/设定值 |
| `MC_COMBINE_MODE` | 轴组合模式：加/减 |
 
### 3.2 功能块数据结构 (MotionKernel.c:118-1248)
 
每个 PLCopen 功能块对应一个 C 结构体，包含：
- `EN/ENO` - IEC 使能信号
- 输入变量（Axis, Execute, Position, Velocity 等）
- 输出变量（Done, Busy, Active, Error, ErrorID 等）
- 内部状态变量（Execute0 等，用于边沿检测）
 
### 3.3 轨迹生成引擎 OTG (MotionKernel.c:1308-1900)
 
**核心数据结构 `GentrajData`**：
 
```c
typedef struct {
    mc_real T;          // 周期时间 (秒)
    mc_real Gacc, Gdec; // 加速度/减速度限制
    mc_real J;          // 加加速度(Jerk)限制
    mc_real Vmvt;       // 运动最大速度
    mc_real Pmin, Pmax; // 软限位
    mc_real Pf, Vf, Tf; // 目标位置/速度/力矩
    mc_real Pn, Vn, Gn; // 当前位置/速度/加速度
    mc_real Jn;         // 当前Jerk
    GentrajCtrlMode ctrl_mode; // 控制模式
    GentrajState state;        // 运动状态
    mc_real Voverride, Goverride, Joverride; // 倍率因子
    int Stop, Stopped;  // 停止控制
} GentrajData;
```
 
**控制模式枚举 `GentrajCtrlMode`**：
 
| 模式 | 说明 |
|------|------|
| `gt_Off` | 关闭 |
| `gt_Position` | 位置控制（点到点运动） |
| `gt_PositionEnd` | 位置到达等待 |
| `gt_Velocity` | 速度控制 |
| `gt_Torque` | 力矩控制 |
| `gt_PositionProfile` | 位置曲线跟踪 |
| `gt_VelocityProfile` | 速度曲线跟踪 |
| `gt_AccelerationProfile` | 加速度曲线跟踪 |
| `gt_Cam` | 凸轮(电子凸轮) |
| `gt_Gear` | 齿轮(电子齿轮) |
| `gt_Homing` | 回零 |
 
**轨迹生成核心算法**：
 
- **2阶速度控制 `VCtrl2`**：基于加减速约束的速度-位置控制，计算最优速度曲线
- **3阶速度控制 `VCtrl3`**：带Jerk约束的S曲线速度控制，支持梯形/三角形/楔形加速度规划
- **凸轮3阶插补 `CamCubicInterpolate`**：Hermite三次多项式插补
- **Profile三次插补 `ProfileCubicInterpolation`**：时间-位置/速度曲线的三次多项式插补
- **力矩控制 `TorqueControl`**：力矩斜坡控制
 
### 3.4 轴状态机 (MotionKernel.c:1949-1959)
 
严格遵循 **PLCopen Motion Control Part 1** 状态机模型：
 
```
                    MC_Power(Enable=TRUE)
                         ↓
    ┌──────────┐    ┌───────────┐    MC_Home     ┌──────────┐
    │ Disabled │───→│ Standstill│──────────────→ │  Homing  │
    └──────────┘    └───────────┘                └──────────┘
         ↑               │                           │
   Power OFF         MC_Move*                    Done/Error
         │               ↓                           │
         │         ┌──────────────┐                   │
         │         │DiscreteMotion│←──────────────────┘
         │         └──────────────┘
         │               │
         │          MC_MoveVelocity
         │               ↓
         │         ┌────────────────┐
         │         │ContinuousMotion│
         │         └────────────────┘
         │               │
         │          MC_CamIn/GearIn
         │               ↓
         │         ┌──────────────────┐
         │         │SynchronizedMotion│
         │         └──────────────────┘
         │
    ┌──────────┐    ┌──────────┐
    │ErrorStop │←───│ Stopping │←── MC_Stop (任意运动状态)
    └──────────┘    └──────────┘
         │
    MC_Reset → Standstill
```
 
### 3.5 轴内部结构 `_AXIS_REF_s` (MotionKernel.c:2509-2539)
 
```c
struct _AXIS_REF_s {
    axis_cmd_chnl_s motion;           // 主运动命令通道
    axis_cmd_chnl_s motion_buffered;  // 缓冲运动命令通道
    axis_cmd_chnl_s superimposed;     // 叠加运动通道
    axis_cmd_chnl_s phasing;          // 相位调整通道
    axis_cmd_chnl_s phasing_buffered; // 缓冲相位通道
    mc_axisstate_enum State;          // PLCopen状态
    GentrajData OTG;                  // 主轨迹生成器
    GentrajData SOTG;                 // 叠加轨迹生成器
    GentrajData POTG;                 // 相位轨迹生成器
    axis_s public;                    // 公共参数(位置/速度/力矩等)
    // ... 快照数据
};
```
 
### 3.6 命令通道 `axis_cmd_chnl_s` (MotionKernel.c:2298-2311)
 
```c
typedef struct {
    int Lock;                    // 命令锁定标志
    int Starting;                // 命令启动标志
    int Completed;               // 命令完成标志
    int Aborted;                 // 命令中止标志
    int Error;                   // 错误标志
    int ErrorID;                 // 错误ID
    void* Data;                  // 指向FB实例的指针
    mc_axissnap_enum Type;       // FB类型
    mc_axissnap_union snapshot;  // 命令参数快照
    mc_real EndVelocity;         // 结束速度
    mc_real BlendVelocity;       // 混合速度
} axis_cmd_chnl_s;
```
 
### 3.7 轴公共参数 `axis_s` (MotionKernel.c:2365-2436)
 
包含与驱动器交互的所有参数：
 
| 分类 | 变量 | 说明 |
|------|------|------|
| 驱动控制 | Power, CommunicationReady | 使能/通信状态 |
| 原始数据 | ActualRawPosition/Velocity/Torque | 驱动器反馈原始值 |
| 设定值 | RawPositionSetPoint/VelocitySetPoint/TorqueSetPoint | 发送给驱动器的原始设定值 |
| 运动模式 | AxisMotionMode (csp/csv/cst/hm) | CSP位置/CSV速度/CST力矩/回零 |
| PLCopen参数 | CommandedPosition, ActualVelocity, ActualTorque 等 | 标准MC参数(#1-#1018) |
| 比例转换 | RatioNumerator/Denominator | 单位转换比例 |
| 限位 | SWLimitPos/Neg, EnableLimitPos/Neg | 软件限位 |
| 回零 | HomingVelocity, HomingLimitWindow | 回零参数 |
| IO | DigitalInputs/Outputs, TouchProbe | 数字IO和探针 |
 
---
 
## 四、功能块分类与调用关系
 
### 4.1 完整功能块清单
 
#### A. 管理类功能块（Administrative）
| 功能块 | IEC接口(pous.xml) | C实现 | 说明 |
|--------|-------------------|-------|------|
| MC_Power | ✅ | `__mcl_cmd_MC_Power` | 轴使能/禁用 |
| MC_Reset | ✅ | `__mcl_cmd_MC_Reset` | 错误复位 |
| MC_SetPosition | ✅ | `__mcl_cmd_MC_SetPosition` | 设置当前位置 |
| MC_SetOverride | ✅ | `__mcl_cmd_MC_SetOverride` | 设置速度/加速度倍率 |
 
#### B. 状态读取类功能块（Status）
| 功能块 | 说明 |
|--------|------|
| MC_ReadActualPosition | 读取实际位置 |
| MC_ReadActualVelocity | 读取实际速度 |
| MC_ReadActualTorque | 读取实际力矩 |
| MC_ReadStatus | 读取PLCopen轴状态 |
| MC_ReadMotionState | 读取运动状态(加速/减速/匀速/方向) |
| MC_ReadAxisInfo | 读取轴信息(限位/仿真/就绪等) |
| MC_ReadAxisError | 读取轴错误代码 |
| MC_ReadParameter / MC_ReadBoolParameter | 读取参数 |
 
#### C. 单轴运动类功能块（Single Axis Motion）— **核心保留**
| 功能块 | 控制模式 | 说明 |
|--------|----------|------|
| MC_Home | gt_Homing | 回零 |
| MC_Stop | gt_Velocity→0 | 紧急停止 |
| MC_Halt | gt_Velocity→0 | 受控停止(可被中断) |
| **MC_MoveAbsolute** | **gt_Position** | **绝对位置运动 ★** |
| **MC_MoveRelative** | **gt_Position** | **相对位置运动 ★** |
| MC_MoveAdditive | gt_Position | 叠加位置运动 |
| **MC_MoveVelocity** | **gt_Velocity** | **速度控制运动 ★** |
| MC_MoveContinuousAbsolute | gt_Position→gt_Velocity | 连续绝对运动(到达后保持速度) |
| MC_MoveContinuousRelative | gt_Position→gt_Velocity | 连续相对运动 |
| **MC_TorqueControl** | **gt_Torque** | **力矩/压力控制 ★** |
 
#### D. 高级运动类功能块（Advanced）— 可暂时舍去
| 功能块 | 说明 | 改造建议 |
|--------|------|----------|
| MC_MoveSuperimposed / MC_HaltSuperimposed | 叠加运动 | 暂时舍去 |
| MC_PositionProfile / MC_VelocityProfile / MC_AccelerationProfile | 曲线跟踪 | 暂时舍去 |
| MC_CamIn / MC_CamOut / MC_CamTableSelect | 电子凸轮 | 暂时舍去 |
| MC_GearIn / MC_GearOut / MC_GearInPos | 电子齿轮 | 暂时舍去 |
| MC_PhasingAbsolute / MC_PhasingRelative | 相位调整 | 暂时舍去 |
| MC_CombineAxes | 轴组合 | 暂时舍去 |
| MC_DigitalCamSwitch / MC_TouchProbe / MC_AbortTrigger | 凸轮开关/探针 | 暂时舍去 |
 
#### E. IO类功能块
| 功能块 | 说明 |
|--------|------|
| MC_ReadDigitalInput | 读数字输入 |
| MC_ReadDigitalOutput / MC_WriteDigitalOutput | 读/写数字输出 |
 
#### F. 参数类功能块
| 功能块 | 说明 |
|--------|------|
| MC_WriteParameter / MC_WriteBoolParameter | 写参数 |
| MC_GetTorqueLimit / MC_SetTorqueLimit | 力矩限制读写 |
 
#### G. 辅助类功能块（可舍去）
| 功能块 | 说明 |
|--------|------|
| MC_Sim | 仿真模式设置 |
| MC_CreateTrigger / MC_CreateOutput | 创建触发器/输出 |
| MC_ToIntArray / MC_ReadOutputBits/Bit | 数组/位操作 |
| MC_CreatePathDescription | 路径描述 |
| MC_*_FROM_CSV | CSV数据加载(依赖Python) |
 
---
 
## 五、业务处理流程
 
### 5.1 PLC 运行时生命周期
 
```
__init_motion() ─→ __MK_Init()
        │
        │  ┌────────────────────────────────────────┐
        │  │          PLC 周期任务循环                 │
        ↓  │                                        │
__retrieve_motion() ─→ __MK_Retrieve()             │
        │   ├─ 更新软限位到OTG                       │
        │   ├─ 检测意外断电 → ErrorStop              │
        │   ├─ 仿真模式: 用设定值更新实际值            │
        │   └─ 真实模式: 原始位置→工程单位转换          │
        │        ├─ 32位溢出检测与补偿                 │
        │        ├─ 16点滑动窗口最小二乘法计算速度      │
        │        └─ 力矩值单位换算                     │
        ↓                                           │
  [PLC 用户程序执行]                                  │
        │  调用 MC_Power, MC_MoveAbsolute 等FB       │
        │  → __mcl_cmd_MC_*() 被调用                 │
        │  → 命令注册到 axis_cmd_chnl_s              │
        ↓                                           │
__publish_motion() ─→ __MK_Publish()               │
        │   └─ for 每个已分配轴:                     │
        │        __MK_ComputeAxis(index)             │
        │        ├─ 处理命令通道锁定/启动              │
        │        ├─ 轴状态机转换                      │
        │        ├─ GenTraj() 轨迹生成                │
        │        ├─ 设定值换算(工程→原始单位)           │
        │        └─ 写入 RawSetPoint                  │
        │                                           │
        └───────────────────────────────────────────┘
 
__cleanup_motion() ─→ __MK_Cleanup() (空实现)
```
 
### 5.2 功能块调用时序（以 MC_MoveAbsolute 为例）
 
```
PLC程序:                    功能块实现层:              命令执行层:           轨迹生成层:
                                                    
MoveAbs.Execute=TRUE  →  __mcl_cmd_MC_MoveAbsolute()
                           │
                           ├─ 检测 Execute 上升沿
                           ├─ 检查 BufferMode
                           ├─ 拍快照(Snapshot):
                           │   保存 Position, Velocity,
                           │   Acceleration, Deceleration, Jerk
                           ├─ 写入命令通道:
                           │   target_cmd->Type = MC_MoveAbsolute
                           │   target_cmd->Lock = 1
                           └─ 设置 Busy=1
                                                    
                                                    __MK_ComputeAxis():
                                                    │
                                                    ├─ 检测 Lock=1
                                                    ├─ 检查状态机许可:
                                                    │   State != Stopping/Homing/
                                                    │           ErrorStop/Disabled
                                                    ├─ State → DiscreteMotion
                                                    ├─ OTG.ctrl_mode = gt_Position
                                                    ├─ 设置 OTG 参数:
                                                    │   Pf, Vmvt, Gacc, Gdec, J
                                                    └─ 调用 GenTraj(&OTG)
                                                                              │
                                                                    gt_Position分支:
                                                                    ├─ 计算 MinMaxG
                                                                    ├─ 检查是否可到达
                                                                    ├─ J!=0 → VCtrl3() S曲线
                                                                    │  J==0 → VCtrl2() 梯形曲线
                                                                    └─ 更新 Pn, Vn, Gn
                                                    
                                                    继续:
                                                    ├─ 工程单位→Raw换算
                                                    ├─ 写 RawPositionSetPoint
                                                    ├─ 到达检测 → Completed=1
                                                    └─ 反馈 Done/Active 到FB
```
 
### 5.3 数据流
 
```
驱动器反馈                       运动内核                        驱动器控制
═══════════                  ═══════════                     ═══════════
ActualRawPosition  ──→  __MK_Retrieve()                
ActualRawVelocity  ──→    │                             
ActualRawTorque    ──→    ├─ 单位换算                    
PowerFeedback      ──→    ├─ 速度估算(最小二乘)          
DigitalInputs      ──→    └─ ActualPosition/Velocity     
                              │                          
                         [PLC FB调用]                     
                              │                          
                         __MK_Publish()                  
                              ├─ 状态机处理               
                              ├─ GenTraj()轨迹生成        
                              ├─ 单位换算                 
                              └──→ RawPositionSetPoint  ──→ 驱动器
                                   RawVelocitySetPoint  ──→ 驱动器  
                                   RawTorqueSetPoint    ──→ 驱动器
                                   AxisMotionMode       ──→ 驱动器(csp/csv/cst)
                                   Power                ──→ 驱动器使能
```
 
---
 
## 六、注塑机液压运动控制改造方案
 
### 6.1 目标系统分析
 
**注塑机液压系统特点**：
- **执行器**：液压缸（线性运动），由伺服泵控系统驱动
- **控制量**：位置(mm)、速度(mm/s)、压力(MPa/bar)
- **结构**：单缸多泵（多台伺服泵并联驱动一个液压缸）
- **动作**：合模、顶针、射胶、座台（4组独立轴）
- **嵌入式平台**：资源有限，需精简代码
 
### 6.2 与当前架构的映射关系
 
| MotionIEC 概念 | 注塑机液压系统 |
|----------------|----------------|
| AXIS_REF（轴） | 液压缸（合模缸、射胶缸、顶针缸、座台缸） |
| MC_MoveAbsolute | 位置控制（行程控制，如合模位置、射胶位置） |
| MC_MoveVelocity | 速度控制（流量控制，如射胶速度、合模速度） |
| MC_TorqueControl | 压力闭环控制（如保压、合模力、背压） |
| Position | 液压缸位移 (mm) |
| Velocity | 液压缸速度 (mm/s) |
| Torque → Pressure | 系统压力 (MPa) |
| RatioNumerator/Denominator | 编码器脉冲→mm 转换比例 |
| 驱动器 | 伺服泵控器（变频器/伺服驱动器） |
| AxisMotionMode (csp/csv/cst) | 泵控模式（位置/流量/压力模式） |
 
### 6.3 需保留的功能块（精简清单）
 
#### 必须保留（核心功能）
 
```
管理类:
  ├─ MC_Power              → 泵站/液压系统使能
  ├─ MC_Reset              → 错误复位
  ├─ MC_SetPosition        → 位置校准（模具厚度标定等）
  └─ MC_SetOverride        → 速度/压力倍率调节
 
运动类:
  ├─ MC_MoveAbsolute       → 位置控制（行程控制）
  ├─ MC_MoveVelocity       → 速度控制（流量控制）
  ├─ MC_TorqueControl      → 压力闭环控制（★ 最关键）
  ├─ MC_Stop               → 紧急停止
  └─ MC_Halt               → 受控停止
 
状态读取:
  ├─ MC_ReadActualPosition → 读取缸位置
  ├─ MC_ReadActualVelocity → 读取缸速度
  ├─ MC_ReadActualTorque   → 读取系统压力
  ├─ MC_ReadStatus         → 读取轴状态
  └─ MC_ReadAxisError      → 读取错误
 
参数类:
  ├─ MC_ReadParameter / MC_WriteParameter     → 参数读写
  ├─ MC_ReadBoolParameter / MC_WriteBoolParameter
  └─ MC_GetTorqueLimit / MC_SetTorqueLimit    → 压力限制
```
 
#### 可暂时舍去
 
```
高级运动类 (约3000行代码):
  ├─ MC_MoveSuperimposed / MC_HaltSuperimposed  (叠加运动)
  ├─ MC_MoveAdditive                            (叠加位置)
  ├─ MC_MoveContinuousAbsolute/Relative         (连续运动)
  ├─ MC_PositionProfile / VelocityProfile / AccelerationProfile (曲线跟踪)
  ├─ MC_CamIn / MC_CamOut / MC_CamTableSelect  (电子凸轮)
  ├─ MC_GearIn / MC_GearOut / MC_GearInPos     (电子齿轮)
  ├─ MC_PhasingAbsolute / MC_PhasingRelative   (相位调整)
  └─ MC_CombineAxes                            (轴组合)
 
探针/凸轮开关类:
  ├─ MC_DigitalCamSwitch
  ├─ MC_TouchProbe
  └─ MC_AbortTrigger
 
辅助类:
  ├─ MC_Sim (仿真—可保留用于调试)
  ├─ MC_*_FROM_CSV (依赖Python，嵌入式不适用)
  └─ MC_CreatePathDescription / MC_ToIntArray 等
```
 
### 6.4 需要新增/修改的模块
 
#### A. 压力闭环控制增强（最重要的改造）
 
当前 `MC_TorqueControl` 是开环力矩斜坡控制，需增强为闭环压力控制：
 
```c
// 新增：压力闭环PID控制器
typedef struct {
    mc_real Kp;              // 比例系数
    mc_real Ki;              // 积分系数
    mc_real Kd;              // 微分系数
    mc_real PressureTarget;  // 目标压力 (MPa)
    mc_real PressureActual;  // 实际压力 (MPa)
    mc_real Integral;        // 积分累积
    mc_real LastError;       // 上次误差
    mc_real OutputMin;       // 输出下限(最小泵转速/排量)
    mc_real OutputMax;       // 输出上限(最大泵转速/排量)
    mc_real DeadBand;        // 死区
} PressurePID;
 
// 在 GenTraj 的 gt_Torque 分支中替换为:
// PressurePIDControl(d, &pid) → 计算泵控输出
```
 
#### B. 单缸多泵管理模块（新增）
 
```c
// 多泵并联管理
typedef struct {
    int PumpCount;              // 泵数量
    AXIS_REF PumpAxes[MAX_PUMP_COUNT]; // 各泵的轴引用
    mc_real FlowDistribution[MAX_PUMP_COUNT]; // 流量分配比例
    mc_real TotalFlowDemand;    // 总流量需求
    int ActivePumpMask;         // 活跃泵位掩码
    // 节能策略
    mc_real MinFlowThreshold;   // 最小流量阈值(低于此值减少泵数)
} MultiPumpManager;
 
// IEC 功能块接口
// MC_PumpGroupEnable   - 泵组使能
// MC_PumpFlowDistribute - 流量分配
// MC_PumpGroupStatus   - 泵组状态读取
```
 
#### C. 驱动器适配层修改
 
```c
// 当前: 伺服电机驱动器 (EtherCAT CiA402)
// 改造: 伺服泵控驱动器
 
// 修改 axis_s 结构，增加液压相关变量:
typedef struct {
    // ... 保留原有变量 ...
    
    // 新增液压专用参数
    IEC_LREAL ActualPressure;        // 实际压力传感器反馈 (MPa)
    IEC_LREAL PressureSetPoint;      // 压力设定值
    IEC_LREAL ActualFlow;            // 实际流量 (L/min)
    IEC_LREAL FlowSetPoint;          // 流量设定值
    IEC_LREAL PumpSpeed;             // 泵转速 (RPM)
    IEC_LREAL PumpDisplacement;      // 泵排量
    IEC_BOOL  PressureSensorValid;   // 压力传感器有效
    IEC_LREAL MaxPressure;           // 最大系统压力
    IEC_LREAL PressureFilterCoeff;   // 压力滤波系数
} axis_hydraulic_s;
 
// 修改 AxisMotionMode 枚举:
typedef enum {
    mc_mode_none,
    mc_mode_position,  // 位置控制模式 (原csp)
    mc_mode_velocity,  // 速度/流量控制模式 (原csv)
    mc_mode_pressure,  // 压力闭环控制模式 (原cst→改为压力)
} mc_axismotionmode_enum;
```
 
#### D. 单位系统适配
 
```c
// 原有: 旋转运动单位
//   位置: 度(°) 或脉冲
//   速度: °/s
//   力矩: N·m
//   RatioNumerator/Denominator = 65536/360 (脉冲/度)
 
// 改造: 线性液压运动单位
//   位置: mm
//   速度: mm/s
//   压力: MPa (替代力矩)
//   RatioNumerator/Denominator → 编码器分辨率/缸行程
```
 
### 6.5 注塑机动作对应的控制策略
 
#### 合模动作 (Clamping)
 
```
快速合模 → MC_MoveVelocity(高速) + MC_MoveAbsolute(到模具接触位置)
     ↓
慢速合模 → MC_MoveVelocity(低速) + MC_MoveAbsolute(到合模位置)  
     ↓
高压锁模 → MC_TorqueControl(目标压力=锁模力/缸面积)
     ↓
保压     → MC_TorqueControl(维持锁模压力)
     ↓
开模     → MC_MoveVelocity(开模速度) + MC_MoveAbsolute(开模位置)
 
控制流程:
  MC_Power(Enable=TRUE)
  MC_MoveVelocity(Velocity=快速合模速度)     // 速度控制
  MC_MoveAbsolute(Position=模具接触位置)      // 位置控制到慢速切换点
  MC_MoveVelocity(Velocity=低压慢速)          // 低压合模
  MC_TorqueControl(Torque=高压锁模压力)       // 压力闭环锁模
```
 
#### 射胶动作 (Injection)
 
```
射胶阶段(多段速度/压力):
  第1段 → MC_MoveVelocity(V1) 至 位置P1   // 速度控制
  第2段 → MC_MoveVelocity(V2) 至 位置P2   // 速度控制
  ...
  VP切换 → 速度→压力切换（到达设定位置或压力触发）
     ↓
保压阶段:
  第1段 → MC_TorqueControl(Pressure1, Time1)  // 压力闭环
  第2段 → MC_TorqueControl(Pressure2, Time2)
  ...
  冷却  → MC_TorqueControl(背压) 或 MC_Halt
 
控制流程:
  MC_MoveVelocity(Velocity=射胶速度1)
  // 到达位置P1后
  MC_MoveVelocity(Velocity=射胶速度2)
  // VP切换触发后
  MC_TorqueControl(Torque=保压压力1)          // 压力闭环
```
 
#### 顶针动作 (Ejector)
 
```
顶出 → MC_MoveAbsolute(Position=顶出位置, Velocity=顶出速度)
     ↓
退回 → MC_MoveAbsolute(Position=退回位置, Velocity=退回速度)
     ↓
可选: 多次顶出/振动顶出
 
控制流程:
  MC_MoveAbsolute(Position=顶出位置)   // 位置+速度控制
  MC_MoveAbsolute(Position=原点位置)   // 位置+速度控制
```
 
#### 座台动作 (Carriage/Nozzle)
 
```
前进(射嘴接触) → MC_MoveAbsolute(Position=前进位置)
               或 MC_TorqueControl(接触压力)   // 压力保持
     ↓
后退           → MC_MoveAbsolute(Position=后退位置)
 
控制流程:
  MC_MoveAbsolute(Position=前进位置)
  MC_TorqueControl(Torque=射嘴接触压力)  // 压力保持
```
 
---
 
## 七、改造实施计划
 
### 第一阶段：代码精简与重构（1-2周）
 
| 步骤 | 内容 | 预计工作量 |
|------|------|-----------|
| 1.1 | 代码拆分：将单文件 MotionKernel.c 拆分为多个模块文件 | 2天 |
| 1.2 | 移除高级功能块（电子凸轮/齿轮/叠加/曲线/路径等） | 1天 |
| 1.3 | 移除 Python 依赖（CSV加载等） | 0.5天 |
| 1.4 | 精简 OTG：只保留 gt_Position/gt_Velocity/gt_Torque/gt_Off 模式 | 1天 |
| 1.5 | 精简轴结构：移除 SOTG/POTG、superimposed/phasing 通道 | 0.5天 |
| 1.6 | 减少静态分配：MAX_AXIS_COUNT 64→8, DEFAULT_INSTANCE_COUNT 256→16 | 0.5天 |
 
**建议的目录结构**：
```
motioniec/
├── inc/
│   ├── motion_types.h        ← 数据类型、枚举定义
│   ├── motion_kernel.h       ← 内核API声明
│   ├── motion_fb.h           ← 功能块结构体定义
│   └── motion_hydraulic.h    ← 液压扩展类型定义
├── src/
│   ├── motion.c              ← PLC运行时集成入口
│   ├── motion_kernel.c       ← 内核初始化/Retrieve/Publish
│   ├── motion_otg.c          ← 轨迹生成算法
│   ├── motion_axis.c         ← 轴状态机与命令处理
│   ├── motion_fb_admin.c     ← 管理类FB (Power/Reset/SetPosition等)
│   ├── motion_fb_motion.c    ← 运动类FB (MoveAbsolute/Velocity/Torque等)
│   ├── motion_fb_status.c    ← 状态读取FB
│   ├── motion_fb_param.c     ← 参数读写FB
│   ├── motion_pressure_pid.c ← 压力闭环PID控制器 (新增)
│   └── motion_multi_pump.c   ← 多泵管理 (新增)
├── pous.xml                  ← IEC接口定义（精简后）
├── CMakeLists.txt            ← 构建系统
└── README.md
```
 
### 第二阶段：液压适配（1-2周）
 
| 步骤 | 内容 | 预计工作量 |
|------|------|-----------|
| 2.1 | 修改 axis_s 结构，增加液压参数(压力/流量/泵速) | 1天 |
| 2.2 | 修改单位系统：旋转→线性，力矩→压力 | 1天 |
| 2.3 | 实现压力闭环PID控制器（替换简单力矩斜坡） | 2天 |
| 2.4 | 实现多泵管理模块（流量分配、泵切换策略） | 2天 |
| 2.5 | 修改 __MK_Retrieve：适配压力传感器、位移传感器数据 | 1天 |
| 2.6 | 修改 __MK_Publish：适配伺服泵控输出 | 1天 |
 
### 第三阶段：注塑机应用层功能块（1-2周）
 
| 步骤 | 内容 | 预计工作量 |
|------|------|-----------|
| 3.1 | 实现 FB_ClampControl（合模控制—多段速+高压锁模） | 2天 |
| 3.2 | 实现 FB_InjectionControl（射胶控制—多段速+VP切换+保压） | 2天 |
| 3.3 | 实现 FB_EjectorControl（顶针控制—多次顶出） | 1天 |
| 3.4 | 实现 FB_CarriageControl（座台控制—前进+接触压力保持） | 1天 |
| 3.5 | 更新 pous.xml 增加新功能块的 IEC 接口定义 | 1天 |
 
### 第四阶段：嵌入式适配与测试（1-2周）
 
| 步骤 | 内容 | 预计工作量 |
|------|------|-----------|
| 4.1 | 移除 `<values.h>` 依赖，替换为标准 `<float.h>` / `<limits.h>` | 0.5天 |
| 4.2 | 消除动态内存分配（当前代码已主要使用静态分配，检查确认） | 0.5天 |
| 4.3 | 优化浮点运算（根据目标平台评估是否需要定点数） | 1-2天 |
| 4.4 | 适配嵌入式编译器（消除 GCC 扩展依赖） | 1天 |
| 4.5 | 仿真测试（使用 MC_Sim 模式验证控制逻辑） | 2天 |
| 4.6 | 实际硬件联调 | 根据实际情况 |
 
### 预计代码量变化
 
| 项目 | 原始 | 精简后 | 增加 | 最终 |
|------|------|--------|------|------|
| MotionKernel.c | 10374行 | ~4000行 | — | — |
| 拆分后总代码 | — | — | ~1500行(液压+注塑) | ~5500行 |
| pous.xml | 5167行 | ~2500行 | ~500行(新FB) | ~3000行 |
 
---
 
## 八、风险与建议
 
### 8.1 技术风险
 
| 风险 | 说明 | 对策 |
|------|------|------|
| 代码生成耦合 | 当前代码由 XSLT 自动生成，手动修改后无法重新生成 | 建议改造后以手写代码为主，放弃自动生成 |
| 实时性 | OTG算法使用 `sqrt`/`pow` 等浮点运算 | 评估嵌入式平台FPU性能，必要时用查表法或定点数 |
| 压力闭环稳定性 | 液压系统非线性强，简单PID可能不够 | 预留前馈补偿、自适应PID接口 |
| 多泵协调 | 多泵同时切换可能引起压力/流量波动 | 设计渐进切换策略，避免同时启停 |
 
### 8.2 关键建议
 
1. **保持 PLCopen 标准接口**：即使底层改为液压控制，上层 IEC 接口应尽量保持 PLCopen 标准兼容，降低 PLC 应用移植难度
2. **先做仿真验证**：利用现有 MC_Sim 仿真模式，在无硬件条件下验证控制逻辑
3. **压力控制是核心**：注塑机最关键的控制是压力闭环（保压、锁模），建议优先完善此部分
4. **VP切换策略**：射胶过程中速度→压力的切换是注塑质量的关键，需设计可靠的切换逻辑（位置触发/压力触发/时间触发）
5. **安全机制**：液压系统需要溢流阀软件模拟、最大压力限制、紧急泄压等安全措施