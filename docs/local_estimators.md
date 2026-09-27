# 估计器源码迁移

`DualImuArmEstimator` 和 `FlexArmEstimator` 的运行时源码已从本机
`D:\postgraduate_life\e_skin\arm_data_collector\flexarm_estimator\src\flexarm`
迁入 `sleeve_arm/estimation/flexarm/`。无需单独安装 `flexarm-estimator`，
无需 editable install、修改 `PYTHONPATH` 或保留相邻源码目录。

```python
from sleeve_arm.estimation.flexarm import DualImuArmEstimator
from sleeve_arm.estimation.flexarm import FlexArmEstimator
from sleeve_arm.estimation.flexarm.calibration import FlexCalibration
```

原来的 `from flexarm ...` 是外部包导入；项目内调用已改成上述路径。
命令行不变，继续使用 `--shoulder-predictor dual_imu` 或
`--shoulder-predictor flexarm_estimator`。`--reuse-calibration`、模型目录、
标定文件和 JSON 格式也不变。现有环境里已安装的旧包可以保留，项目不会导入它。

## 迁移范围与来源

共原样复制 13 个运行时 Python 文件：双 IMU 估计、三柔性传感器推理、
标定、归一化、因果特征、运动门控、动作去抖、角度滤波及模型读写。
没有迁入训练脚本、诊断 CLI、打包文件、虚拟环境或生成模型。

来源仓库 HEAD 为 `1cf3aff17996744377e7a7c287c8a556dfbcae3c`；采用的是
用户当时的本地工作区，其中含未提交改动，并非仅复制该提交。
[provenance.json](../sleeve_arm/estimation/flexarm/provenance.json)
记录各源文件的 SHA-256。迁移时 13 个文件逐字节一致，没有改写数学实现。
后续修改应直接维护本地源码，不再通过重装 wheel 更新。

源目录的 `dual_imu.py` 比之前安装的 0.3.0 wheel 新：新增连续前后举、
外展和 YXY 第三旋转角输出，并要求右手正交解剖坐标系。项目现有轴设置满足
这一条件。这里保留源实现；控制链仍使用原有 `direction`、`magnitude_deg`
和 `confidence`，没有切换肩部映射。新增的 `upper_arm_rotation_deg`
也没有替代现有 IMU3/IMU4 twist estimator。休息→前抬→旋转的标定顺序、
EMA、同步和超时参数均不变。

## 依赖和模型文件

`DualImuArmEstimator` 只需要 NumPy，不需要训练权重、joblib 或 scikit-learn。
本地包保留懒加载：只有导入 `FlexArmEstimator` 时才加载模型依赖。
`requirements.txt` 已删除本地 wheel 条目，保留项目通用依赖；不再执行任何
`pip install flexarm-estimator`。

三柔性传感器预测仍需要 NumPy、joblib、`scikit-learn==1.7.2` 和已有权重。
源码迁移不会消除这些数值/模型依赖。`model_dir` 仍从配置文件目录解析，必须
指向完整模型目录，包含：

```text
metadata.json
calibration.json
action_classifier.joblib
angle_forward.joblib
angle_backward.joblib
angle_lateral.joblib
```

本次检查时，源项目 `models/` 下只有 JSON 元数据和诊断图，没有上述 `.joblib`
权重，因此没有复制一个不完整的模型目录，也没有改动已有 `model_dir` 或训练
新模型。使用 `flexarm_estimator` 时需提供原来训练好的完整模型文件；
`dual_imu` 不受此限制。模型读写仍保存/加载原来的分类器、回归器和 JSON，
保留 scikit-learn 版本一致性检查。

## 离线验证

- 迁入源项目的双 IMU、特征、标定/归一化、门控、滤波和配置测试。
- 用迁移前 wheel 捕获的 golden 数据比较标定结果、旧估计字段和完整
  `MotionIntent`，覆盖前后举、外展、Rest、Transition、安装偏转和身体运动。
- 使用无训练的确定性模型替身验证本地 `from_pretrained`、预测、标定和复用，
  以及模型版本检查。它验证文件格式和调用链，不代表真实权重的预测精度。
- 在子进程禁止导入外部 `flexarm` 后运行 `fake`、`aurora-fake` 的完整
  双 IMU CLI；另单独禁止 joblib/scikit-learn 验证双 IMU 懒加载边界。

本次未连接真实传感器或机器人。

迁移后全量测试结果：**520 passed**。运行测试的 Python 进程通过
`MetaPathFinder` 禁止导入 `flexarm` / `flexarm.*`，子进程 CLI 测试也单独
设置相同限制；不需要卸载或修改用户已有的 Python 环境。
