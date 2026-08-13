# 无足端力传感器时的接触估计方案

> 记录日期：2026-08-12  
> 适用硬件：电机编码器 + IMU，无足端力传感器  
> 适用机器人：sd05（Gazebo / 真机）  
> 当前采用方案：方案一（GaitSchedule 规划接触状态）

---

## 1. 问题背景

项目默认的 `LinearKalmanFilter` 在 `StateEstimateBase::updateContact()` 中使用足端力传感器判断接触状态：

```cpp
contact_flag_[i] = foot_force_state_interface_[i].get().get_value() > feet_force_threshold_;
```

对于只有 **电机编码器 + IMU** 的硬件配置，无法提供 `foot_force_state_interface_`，因此必须改造接触检测逻辑。

`LinearKalmanFilter` 本身不检测接触，它只负责状态估计；接触检测需要在卡尔曼滤波之前完成，为卡尔曼滤波提供 `contact_flag_`。

---

## 2. 方案概览

| 方案 | 名称 | 数据需求 | 当前状态 |
|------|------|---------|---------|
| 方案一 | GaitSchedule 规划接触状态 | 当前时间 + 步态模板 | **已采用** |
| 方案二 | 电机电流阈值法 | 电机电流 | 待开发 |
| 方案三 | 位置跟踪误差法 | 电机位置 + 期望位置 | 待开发 |
| 方案四 | 多源融合检测 | GaitSchedule + 电机电流 + 位置误差 | 长期目标 |
| 方案五 | 外部定位辅助 | 外部 `/odom` 话题 | 有条件时加入 |

---

## 3. 方案一：GaitSchedule 规划接触状态（当前采用）

### 3.1 思路

不依赖真实传感器判断触地，而是直接使用 MPC 的步态调度器中已经规划好的接触模式。

`GaitSchedule` 内部维护一个 `ModeSchedule`，包含：

- `eventTimes`：模式切换时间点
- `modeSequence`：对应时间区间内的接触模式编号

通过当前时间查询当前 mode，再转换为 4 个足端的 `contact_flag_`。

### 3.2 核心改动点

1. `StateEstimateBase.h`
   - 增加 `std::shared_ptr<ocs2::legged_robot::GaitSchedule> gait_schedule_` 成员
   - 增加 `setGaitSchedule()` 接口

2. `StateEstimateBase.cpp`
   - 修改 `updateContact()`，从 GaitSchedule 查询当前 mode：

   ```cpp
   void StateEstimateBase::updateContact(scalar_t currentTime)
   {
       if (gait_schedule_ == nullptr)
       {
           contact_flag_.fill(true);
           return;
       }

       auto modeSchedule = gait_schedule_->getModeSchedule(currentTime, currentTime);
       size_t mode = modeSchedule.modeAtTime(currentTime);
       contact_flag_ = ocs2::legged_robot::modeNumber2StanceLeg(mode);
   }
   ```

3. `CtrlComponent.cpp`
   - 在 `setupStateEstimate()` 中把 `GaitSchedule` 传入估计器：

   ```cpp
   auto gait_schedule = legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule();
   estimator_->setGaitSchedule(gait_schedule);
   ```

4. `KalmanFilterEstimate::update()`
   - 调用 `updateContact(observation_.time)`，传入当前时间

5. `robot_control.yaml`
   - 删除 `foot_force_name` 和 `foot_force_interfaces` 配置

### 3.3 数据需求

- `observation_.time`：控制器运行时间
- `gait.info`：步态模板（stance、trot、pace 等）
- `control_inputs_.command`：用户命令，选择当前步态

### 3.4 trot 步态示例

`trot` 模板定义：

```cpp
modeSequence  = {LF_RH, RF_LH}
switchingTimes = {0.0, 0.3, 0.6}
```

周期 `T = 0.6s`，GaitSchedule 沿时间轴周期性展开：

| 时间（相对周期起点） | mode | 着地足 |
|---------------------|------|--------|
| 0.0 ~ 0.3s | 9 (`LF_RH`) | LF + RH |
| 0.3 ~ 0.6s | 6 (`RF_LH`) | RF + LH |

`modeNumber2StanceLeg(mode)` 把 mode 编号转成 `std::array<bool, 4>{LF, RF, LH, RH}`。

### 3.5 优点

- 改动小，实现快
- 不需要力传感器
- 能快速验证 NMPC + WBC 控制链路

### 3.6 缺点与风险

- **开环假设**：假设机器人严格按照规划步态运行
- 真实打滑、提前/延迟触地会导致状态估计偏差
- 无法检测摔倒或异常接触
- 要求 `observation_.time` 与仿真/真机时间严格同步
- 启动阶段建议保持 `STANCE`，站稳后再切换步态

---

## 4. 方案二：电机电流阈值法

### 4.1 思路

摆动腿触地瞬间电机电流会明显增大，用电流阈值判断触地。

### 4.2 实现要点

```cpp
void updateContact()
{
    for (int leg = 0; leg < 4; leg++)
    {
        double total_current = 0.0;
        for (int joint = 0; joint < 3; joint++)
        {
            int idx = leg * 3 + joint;
            total_current += std::abs(motor_current_[idx]);
        }
        double avg_current = total_current / 3.0;
        contact_flag_[leg] = avg_current > current_threshold_;
    }
}
```

### 4.3 数据需求

- 电机电流（每个关节）

### 4.4 优点

- 只需要电机数据
- 比 GaitSchedule 更接近真实接触

### 4.5 缺点

- 阈值依赖地面硬度、运动速度
- 摩擦和惯性可能引起电流波动
- 需要标定

---

## 5. 方案三：位置跟踪误差法

### 5.1 思路

触地后实际关节位置跟不上期望位置，产生跟踪误差，用误差阈值判断触地。

### 5.2 实现要点

```cpp
void updateContact()
{
    for (int leg = 0; leg < 4; leg++)
    {
        double max_error = 0.0;
        for (int joint = 0; joint < 3; joint++)
        {
            int idx = leg * 3 + joint;
            double error = std::abs(q_desired_[idx] - q_actual_[idx]);
            max_error = std::max(max_error, error);
        }
        contact_flag_[leg] = max_error > pos_error_threshold_;
    }
}
```

### 5.3 数据需求

- 电机实际位置
- 电机期望位置

### 5.4 优点

- 不需要电流
- 物理意义明确

### 5.5 缺点

- 低速或轻触地时误差小
- 齿轮间隙、摩擦会引入噪声

---

## 6. 方案四：多源融合检测（长期目标）

### 6.1 思路

结合 GaitSchedule + 电机电流 + 位置误差，综合判断触地状态。

### 6.2 实现要点

```cpp
void updateContact(scalar_t currentTime)
{
    auto planned = getPlannedContact(currentTime);
    auto currentDetected = detectFromCurrent();
    auto errorDetected = detectFromPositionError();

    for (int leg = 0; leg < 4; leg++)
    {
        if (currentDetected[leg] == errorDetected[leg])
        {
            contact_flag_[leg] = currentDetected[leg];
        }
        else
        {
            contact_flag_[leg] = planned[leg];  // fallback
        }
    }
}
```

### 6.3 优点

- 比单一方法可靠
- 电机数据异常时可用 GaitSchedule 兜底

### 6.4 缺点

- 需要调多个阈值
- 计算量稍大

---

## 7. 方案五：外部定位辅助

### 7.1 思路

如果有 VIO、LIO、运动捕捉等外部里程计，可直接使用 `FromOdomTopic` 估计器，不依赖接触检测。

### 7.2 配置

```yaml
estimator_type: "FromOdomTopic"
odom_topic: "/your_odom_topic"
```

### 7.3 优点

- 不依赖接触检测
- 定位精度高

### 7.4 缺点

- 需要额外传感器或算法

---

## 8. 后续优先级

| 优先级 | 方案 | 建议阶段 |
|--------|------|---------|
| P0 | 方案一：GaitSchedule | 当前立即实施，跑通控制链路 |
| P1 | 方案二/三：基于电机的触地检测 | 真机调试阶段加入 |
| P2 | 方案四：融合检测 | 长期优化，提高真机鲁棒性 |
| P3 | 方案五：外部定位 | 有条件时加入 |

---

## 9. 相关代码位置

- `libraries/controller_common/include/controller_common/FSM/StateEstimateBase.h`
- `libraries/controller_common/src/FSM/StateEstimateBase.cpp`
- `controllers/ocs2_quadruped_controller/src/control/CtrlComponent.cpp`
- `controllers/ocs2_quadruped_controller/src/estimator/LinearKalmanFilter.cpp`
- `controllers/ocs2_quadruped_controller/src/control/GaitManager.cpp`
- `descriptions/sd05/sd05_description/config/ocs2/gait.info`
- `descriptions/sd05/sd05_description/config/ocs2/reference.info`
- `descriptions/sd05/sd05_description/config/robot_control.yaml`

---

## 10. 注意事项

1. `LinearKalmanFilter` 本身不检测接触，它只使用 `contact_flag_` 做状态估计。
2. 接触检测必须在卡尔曼滤波预测/更新之前完成。
3. 方案一是开环方案，仿真中效果通常不错，真机上需谨慎验证。
4. 最终推荐目标是方案四：电机数据检测 + GaitSchedule 兜底。
5. 如果用方案一，启动后建议先保持 `STANCE` 模式，等机器人站稳再切换 trot。
