# 七轴机械臂仿真
> 使用 MATLAB 对七轴机械臂进行仿真，目的为快速验证算法与机械参数

## 依赖
- MATLAB2025b
- Robotics Toolbox（Peter Corke）
- Optimization Toolbox（`ikcon` 使用 `fmincon`）

## 架构

### 目录结构
```
src/
├── main.m              程序入口
├── model/
│   ├── ArmModel.m      七轴机械臂模型类
│   └── TechCore.m      圆柱目标位姿、范围采样和可达性验证
└── planning/           路径规划相关算法
```

## 圆柱目标范围验证

从项目根目录运行，位置统一使用米，`TechCore` 角度统一使用度。
`src/main.m` 的输入区另提供 `position_unit`：默认使用毫米输入，进入 `TechCore` 前转换为米。
绘图和求解器始终使用米，避免将 500 mm 误当作 500 m。
`theta` 是外法向在场景 XY 平面的投影相对 +X 的方位角，范围 `[-90,90]`；
`phi` 是外法向与场景 +Z 的夹角，范围 `[0,90]`；
`alpha` 绕圆柱自身 +X 轴按右手定则旋转，范围 `[-45,45]`。
零自转姿态明确为 `Rz(theta) * Ry(phi-90)`。

场景坐标系相对基座平移 `[0.1,0,0]` 米并绕 Z 旋转 180 度，
显式变换保存在 `core.T_base_scene`，其方向为场景坐标到基座坐标。
末端目标点固定在圆柱坐标系 `[-0.075,0,0]` 米。
`ToolRotation` 表示末端坐标系相对圆柱坐标系的旋转，默认同向，可通过
`core.set_tool_rotation(R_cylinder_tool)` 修改；修改姿态对应关系不改变目标点。

```matlab
addpath('src/model', 'src/planning');
arm = ArmModel('Show', false);
core = TechCore('ToolRotation', eye(3));

% 单个位姿：返回基座下末端目标、场景下圆柱、基座下圆柱的齐次变换。
[T_base_tool, T_scene_cylinder, T_base_cylinder] = ...
    core.pose([0.25, 0, 0.25], 0, 45, 0);
[q, info] = arm.inverse_transform(T_base_tool, 'Attempts', 3);

% 每行对应 x、y、z、theta、phi、alpha 的 [下界, 上界]。
% 示例范围仅展示接口，请替换为实际任务范围。
ranges = [0.20, 0.30;
         -0.05, 0.05;
          0.20, 0.30;
            -30, 30;
             30, 60;
            -15, 15];
counts = [3, 3, 3, 3, 2, 2]; % 各维采样数，包含端点
cloud = core.sample_range(ranges, counts); % 仅生成目标，不求逆解
result = core.evaluate(arm, ranges, counts, ...
    'Attempts', 3, 'PositionTolerance', 1e-4, ...
    'OrientationTolerance', 1e-3, 'MaxSamples', 100000);
core.draw_cloud(result);
```

固定范围自动只采样一次；非固定范围至少需要两个样本。六维样本数量为各维采样数的乘积，
默认上限为 100000。建议先使用较稀疏的采样，批量逆解可能耗时较长。
`ArmModel` 原有的 `[x,y,z,roll,pitch,yaw]` 接口中角度仍使用弧度。

`result.samples` 保存场景下的 `[x,y,z,theta,phi,alpha]`，
`targetTransforms` 和 `targetPositionsBase` 保存基座下末端目标；
`success`、`jointConfigurations` 和误差字段保存逐个位姿的验证结果，失败时关节角为 NaN。
`positionsScene`、`positionsBase` 为去重后的圆柱底面中心，
`positionIndex` 将每个位姿映射到相应位置，`successFraction` 为各位置的姿态样本通过比例。
点云绘制基座下圆柱底面中心：绿色表示全部采样姿态通过，黄色表示部分通过，红色表示均未通过。
`allSampledPosesPassed` 表示本次全部样本是否通过。

逆解通过需要满足求解器成功退出、关节限位、位置误差和姿态误差要求。
位置容差单位为米，姿态容差单位为弧度。多初值依次使用当前关节配置、限位中点和随机配置，
验证不改变机械臂状态。需要复现随机初值时，可在调用前设置 `rng(0)`。
未通过表示未找到有效解；有限采样不能证明连续范围内任意位姿均可达。
此验证不包含碰撞检测或从当前配置到目标的路径连通性检查。

## 随机点云例程

直接运行 `src/main.m`。输入区保留圆柱位置和角度范围，使用以下参数控制随机点云：

```matlab
position_unit = 'mm';
position_samples = 1000;
orientations_per_position = 20;
random_seed = 0;
```

XYZ 在范围内均匀随机采样；每个位置独立采样多组姿态。
`theta`、`alpha` 均匀采样，`cos(phi)` 均匀采样，使法向按球面面积均匀分布。
固定范围保持原值。随机采样不保证包含范围端点，也不能保证覆盖所有姿态。
默认 1000 个位置、每位置 20 个姿态，共 20000 次位姿验证；批量逆解可能耗时较长，
可先降低两个采样数量。修改种子可生成另一组可复现的随机样本。

直接使用随机采样接口时，`ranges` 的位置部分仍以米为单位：

```matlab
rng(0);
cloud = core.sample_range(ranges, [1000, 20], 'Sampling', 'random');
result = core.evaluate(arm, ranges, [1000, 20], ...
    'Sampling', 'random', 'Attempts', 3);
ax = core.draw_cloud(result);
core.draw_reference(arm, ax);
```

`main.m` 会绘制圆柱中心结果汇总和实际末端目标点云两张图。
两张图统一使用固定场景 O′ 坐标，叠加当前关节配置在参考高度下的骨架。
红、绿、蓝箭头分别表示各自的 X、Y、Z 轴；场景原点为 `[0,0,0]` 米，
基座原点为 `[0.1,0,h]` 米。参考高度取允许高度的中点，虚线显示基座升降区间。
骨架不代表每个采样目标的逆解姿态。

## 基座升降联合求解

`main.m` 中 `base_height_range = [290,490]` 始终以毫米填写，与 `position_unit` 独立。
每个位姿允许单独选取高度；联合优化七轴关节和一个升降变量，不穷举固定高度。
调用 `evaluate` 时对应 `BaseHeightRange` 参数以米表示，默认 `[0.290,0.490]`。
`ArmModel.inverse_lift_transform(T_scene_tool, heightRange, ...)` 使用 SQP 求解，
优化变量按各自限位归一化，允许 `[h,h]` 固定高度。
高度为 h 时 `T_base_scene` 的 Z 平移为 `-h`，XY 偏移与绕 Z 的旋转保持原定义。
`TechCore.T_base_scene` 常量及原 `pose` 接口表示 h=0 时的参考变换；
批量求解使用新增的 `targetTransformsScene`、`targetPositionsScene`，并保留旧的零高度参考字段。
`result.baseHeight` 保存每个成功样本的高度（米），失败为 NaN，
`baseHeightRange` 保存本次边界。绿色表示全部采样姿态都找到解，允许各自使用不同高度。
不会自动执行升降机构或机械臂运动，仍不检查碰撞与运动路径。

## CPU 并行加速

`main.m` 默认设置 `use_parallel = true`、`parallel_workers = 2`，稳定运行后可增加数量。
需要 Parallel Computing Toolbox 和可用许可证；缺少时提示并回退到串行。
设置 `use_parallel = false` 可强制串行，不启动并行池。
首次运行自动建立进程池；已有进程池直接复用，最多使用指定数量的 worker，
不自动关闭或重建已有池。若已有线程池则报错，需要先关闭线程池并改用进程池。
进程池启动失败（例如本机端口连接超时）时输出错误原因并回退到串行，
`result.parallelFallbackReason` 保存原因；逆解计算中的异常仍正常报出。

每个 worker 使用独立的机械臂副本，保留调用时的当前关节配置。
所有绘图与结果汇总在客户端进行。并行使用 `parfeval` 分批处理各目标，
不启用 `fmincon` 内部并行。客户端主动轮询 `PollableDataQueue`，不使用嵌套 `afterEach` 监听器。
`ik_seed` 控制每个位姿的随机逆解初值，按样本序号分配 Threefry 子流，
与 worker 数和调度顺序无关；点云本身仍由 `random_seed` 控制。
在输入、种子和数值环境相同的情况下，可以对照串行和并行结果。

直接调用时可使用：

```matlab
result = core.evaluate(arm, ranges, [1000, 20], ...
    'Sampling', 'random', 'UseParallel', true, 'Workers', 2, 'IKSeed', 0);
```

`result.usedParallel`、`workers` 记录实际执行方式；`parallelSetupTime` 记录并行池准备时间，
`solveTime` 记录验证循环耗时（包含首次向 worker 传输模型的开销）。
小批量任务可能因启动和传输开销而更慢，大批量任务更适合并行。

`exitflag = 5` 表示预测的目标函数变化过小，并不等于无解。
仍需通过位姿误差和关节限位检查；程序保留原有容差与通过条件。
`ArmModel` 过滤第三方 `ikcon` 多余的 `exitflag_` 回显，保留其他文本输出。
`main.m` 输出失败样本的误差统计和前 20 条诊断，完整表保存在 `failed_diagnostics`。
误差与退出状态取自各样本保存的最佳残差尝试，超差计数可能重叠，不代表严格不可达。

运行过程中默认每完成 100 个位姿输出完成数量、通过/失败数量和累计求解耗时。
`main.m` 中的 `progress_every` 控制间隔，设为 0 关闭周期进度；
`print_each_sample = true` 会逐个位姿输出样本编号、PASS/FAIL、位置误差（mm）和姿态误差（度）。
直接调用 `evaluate` 时对应参数为 `ProgressEvery` 和 `PrintEachSample`。
并行输出通过 PollableDataQueue 在客户端统一处理，顺序是完成顺序，样本编号对应 `result.samples` 的行号。
客户端每 5 秒输出已开始、已完成、运行中数量和耗时，即使首个样本还未结束也有状态提示。
若尚未收到 worker 消息，已开始数量显示为 0。关闭所有进度选项后不输出此等待提示。
这些状态针对单个位姿，一个圆柱位置全部采样姿态是否通过仍以最终汇总为准。
逐条输出大量样本可能影响性能，默认关闭。

`show_solver_stages = true`（接口参数 `ShowSolverStages`）在首个位姿完成前输出
样本开始、每次初值尝试开始及优化器返回状态，用于区分 worker 准备和逆解耗时。
开启 `print_each_sample` 后持续输出这些阶段；否则首个完成之后仅输出周期进度。
尝试结束的退出状态不是最终通过判定，仍需等待位姿误差检查。
通知位于 `evalc` 捕获范围之外，串行模式也可及时显示。
若只有“attempt started”，表示该次优化调用尚未返回；并行等待提示仍每 5 秒输出。
逐样本 PASS/FAIL 日志额外包含求得的基座高度（毫米），失败高度为 NaN。

## 自动搜索杆长

`src/main.m` 默认开启 `optimize_lengths = true`，自动调整 `L(3).d` 和 `L(5).d`，
目标为两者总和最小。杆长对全部目标固定，每个目标仍可独立选择关节角和允许范围内的基座高度。
设置 `optimize_lengths = false` 可只验证当前默认杆长。当前入口中的升降设置为 290–430 mm。

```matlab
optimize_lengths = true;
length_resolution_mm = 1;
max_length_candidates = 100;
length_search_attempts = 8;
```

不设置人为的最大杆长，只要求非负，允许搜索零长度边界。
先从当前杆长出发，若未找到全通过组合则扩大总长度搜索范围，并检查不同长度比例。
找到总长度为 S 的可行组合后，只搜索 `d3 + d5 < S` 的区域。
随后进行粗网格搜索和逐步减半步长的局部搜索，默认细化到 1 mm。
该步长仅表示局部搜索尺度，不代表全局最优误差界，也不保证以 1 mm 整数输出。
`MaxCandidates` 是计算预算，不是机械尺寸限制；达到预算可能尚未找到可行解或完成细化。

所有候选使用相同的随机目标和逐目标逆解随机初值。先串行检查最多 8 个探测目标，
包括此前失败的目标；通过探测后才调用完整评估，可复用 CPU 进程池并行验证全部目标。
只有每个位置的每个采样姿态全部通过，才记录为最优可行组合。
默认提高逆解尝试次数到 8，但探测或完整验证失败仍可能是局部收敛导致，不能据此证明不可达。
探测阶段一次逆解期间没有心跳输出；完整评估阶段沿用原进度和等待提示。

每完成一个候选，会将 `search` 保存到项目根目录的 `link_length_search.mat`，运行结束后也会保存最终状态。
相同路径的旧搜索记录会被覆盖；该文件用于检查结果，不提供自动续跑。
`length_search.bestLengths`、`bestTotalLength` 单位为米，`bestResult` 保存全部位姿的关节角、
基座高度、误差和点云。`history` 各列依次为 d3、d5、完整验证通过数、是否全通过、累计秒数；
探测拒绝时完整通过数为 NaN。`status` 区分预算耗尽、完成局部细化和零总长度。
没有全通过组合时，`foundFeasible = false`、`bestResult` 为空，入口不会绘制误导性的最优结果。
有可行组合时，即使预算耗尽也会输出并绘制当前找到的最短组合。
搜索结果只针对固定采样点云，不证明连续范围全可达或数学上的全局最优，不包含碰撞和路径检查。

也可手动指定模型杆长（米）：

```matlab
arm = ArmModel('Show', false, 'LinkLengths', [0.14804, 0.20508]);
lengths = arm.get_link_lengths(); % [L(3).d, L(5).d]
```

## 验证

```matlab
addpath('tests');
test_link_lengths; % 仅几何和参数检查，不运行逆解或开启并行池
test_tech_core;
test_parallel_evaluation; % 需要 Parallel Computing Toolbox，会启动或复用进程池
```
