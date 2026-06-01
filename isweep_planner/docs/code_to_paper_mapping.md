# iSweep 代码-论文映射文档

本文档用于把 `isweep_planner` 当前实现直接映射到中文小论文的方法章节，便于后续撰写“方法”“实验设置”和“系统实现”部分。

## 1. 论文题目建议

建议题目：

`一种面向复杂环境的风险感知全局-局部协同导航方法`

如果后续实验更强调动态障碍，可扩展为：

`一种面向静动态混合环境的风险感知全局-局部协同导航方法`

## 2. 方法主线与代码位置

### 2.1 风险分段的全局路径生成方法

对应论文章节：

- `3.2.1 基于拓扑搜索的全局候选路径生成`
- `3.2.2 路径离散化与风险段判定`
- `3.2.3 高低风险段的差异化轨迹求解`
- `3.2.4 轨迹失败时的局部恢复与回退机制`

主要代码：

- [common.h](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/include/isweep_planner/core/common.h)
- [se2_sequence_generator.cpp](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/src/optimization/midend/se2_sequence_generator.cpp)
- [svsdf_runtime_evaluate.cpp](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/src/framework/svsdf_runtime_evaluate.cpp)

关键映射：

- `RiskLevel` 定义了论文里的高风险段和低风险段。
- `MotionSegment` 是“风险分段”后的直接数据载体。
- `SE2SequenceGenerator::discretizePath()` 将拓扑路径离散为 `SE2State` 序列。
- `IsHighRiskConfiguration()` 结合 `esdf`、`safeYawCount` 与 `yaw_distance` 判定局部配置是否属于高风险。
- `SE2SequenceGenerator::generate()` 输出带 `risk` 标签的 `MotionSegment` 序列。
- `SvsdfRuntime::EvaluateCandidate()` 对高风险段优先执行严格 SE(2) 求解，对低风险段执行轻量 R² 求解。
- 当整条拼接轨迹不可行时，`EvaluateCandidate()` 会围绕瓶颈位置对相邻低风险段执行局部严格升级，再尝试恢复。

论文写法建议：

- 这一部分应表述为“风险分段与差异化求解机制”。
- 风险判定更适合写成“工程化风险判据”或“启发式风险准则”，不要写成完整数学最优判别器。

### 2.2 风险参考轨迹构建方法

对应论文章节：

- `3.3.1 风险参考点的数据组织`
- `3.3.2 净空、风险等级与建议速度的生成方式`
- `3.3.3 风险参考轨迹的发布与可视化表达`

主要代码：

- [svsdf_runtime_reference.cpp](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/src/framework/svsdf_runtime_reference.cpp)
- `msg/GlobalReferencePoint.msg`
- `msg/RiskAwareGlobalReference.msg`

关键映射：

- `BuildRiskAwareGlobalReference()` 将段级轨迹采样为全局风险参考轨迹。
- 每个参考点包含：
  - `x, y, yaw`
  - `risk_level`
  - `clearance`
  - `segment_id`
  - `s`
  - `preferred_speed`
- 几何信息来自分段优化后的轨迹，`risk_level` 继承自 `MotionSegment::risk`，`clearance` 由现有 SVSDF evaluator 查询，`preferred_speed` 由风险等级映射得到。

论文写法建议：

- 强调“全局结果不再只是纯几何 Path，而是携带风险语义的参考轨迹”。
- `preferred_speed` 更适合描述成“建议速度”或“风险相关速度先验”。

### 2.3 风险自适应局部跟踪与重规划机制

对应论文章节：

- `3.4.1 局部参考窗口提取`
- `3.4.2 高风险与低风险跟踪模式切换`
- `3.4.3 基于净空与偏差的局部轨迹生成`
- `3.4.4 重规划触发条件与控制输出`

主要代码：

- [risk_aware_local_planner.cpp](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/src/local_planner/risk_aware_local_planner.cpp)
- [planner.yaml](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/config/planner.yaml)

关键映射：

- `ExtractLocalReferenceWindow()` 从全局风险参考轨迹中提取局部窗口。
- `DetermineLocalPlannerMode()` 在 `LOW_RISK_FOLLOW`、`HIGH_RISK_STRICT` 和 `BLOCKED_RECOVERY` 间切换。
- `BuildLocalTrajectory()` 在低风险模式下允许适度平滑，在高风险模式下严格约束参考偏差。
- `EvaluateLocalTrajectory()` 验证局部路径的碰撞与净空要求。
- `CheckNeedReplan()` 基于横向误差、航向误差和严格模式偏离判断是否触发重规划。
- `UpdateCommand()` 将风险窗口的建议速度和航向误差转化为可执行控制量。

论文写法建议：

- 把这一块表述成“风险自适应局部跟踪机制”或“风险感知参考跟踪器”。
- 不建议写成“全新的局部最优控制器”或“复杂优化型局部规划器”。

## 3. 核心参数如何映射到论文

主要文件：

- [planner.yaml](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/config/planner.yaml)

建议在论文附表中突出以下参数：

- `se2/discretization_step`
  - 对应路径离散化密度与风险分段分辨率。
- `reference/sample_dt`
  - 对应风险参考轨迹采样密度。
- `local_planner/high_risk_ratio_threshold`
  - 对应高低风险局部模式切换阈值。
- `local_planner/high_risk_lateral_replan_threshold`
  - 对应高风险窗口下的横向偏差重规划阈值。
- `local_planner/high_risk_yaw_replan_threshold`
  - 对应高风险窗口下的航向误差重规划阈值。
- `local_planner/strict_reference_deviation_threshold`
  - 对应严格模式下对风险参考轨迹的最大允许偏离。
- `local_planner/low_risk_clearance_margin`
  - 对应低风险模式的净空裕度。
- `local_planner/high_risk_clearance_margin`
  - 对应高风险模式的净空裕度。

## 4. 日志与实验现象的对应关系

本轮已统一部分日志措辞，后续实验截图或 rosout 记录可以直接对应论文叙述：

- `Risk-aware segment solve: ... risk=HIGH solver=strict_se2`
  - 对应“高风险段严格求解”。
- `Risk-aware segment solve: ... risk=LOW solver=planar_r2`
  - 对应“低风险段轻量求解”。
- `Risk-aware local strict upgrade: ...`
  - 对应“围绕瓶颈的局部严格升级”。
- `Risk-aware candidate degraded after local recovery failure`
  - 对应“局部恢复失败后的退化输出”。

## 5. 当前论文表述边界

建议在论文中保持以下边界，避免表述过强：

- 风险判定目前是规则驱动，而不是完整理论风险模型。
- 局部规划器当前更接近风险感知参考跟踪器，而非复杂优化型局部控制器。
- 动态障碍融合和系统桥接更偏工程实现，适合放在系统实现或实验平台部分，不建议升格为主创新。

## 6. 本轮整理涉及文件

- [common.h](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/include/isweep_planner/core/common.h)
- [planner.yaml](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/config/planner.yaml)
- [se2_sequence_generator.cpp](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/src/optimization/midend/se2_sequence_generator.cpp)
- [svsdf_runtime_evaluate.cpp](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/src/framework/svsdf_runtime_evaluate.cpp)
- [svsdf_runtime_reference.cpp](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/src/framework/svsdf_runtime_reference.cpp)
- [risk_aware_local_planner.cpp](/home/zhl/robot3/nav_ws/src/NEXTE_Sentry_Nav/isweep_planner/src/local_planner/risk_aware_local_planner.cpp)
