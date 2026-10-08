# 架构文档维护规则

## 什么时候必须更新

以下改动与代码在同一次提交中更新架构文档：

- 新增、删除、改名 ROS 包；
- 更改工作空间依赖方向或 vendor 来源；
- 新增顶层 launch profile；
- 更改稳定话题、action、service 或 TF 所有者；
- 更改安全链、自动执行默认值或控制模式；
- 新增传感器、计算主机或部署方式；
- 一项“实验代码”成为正式运行基线。

仅调参、修复内部算法或新增一次性结果文件通常不需要改总图，但应更新所属包 README 或实施
记录。

## 更新位置

| 改动 | 至少更新 |
|---|---|
| 包边界 | `PACKAGE_CATALOG.md` |
| 数据/控制方向 | `README.md` 功能分层图与 `RUNTIME_FLOWS.md` |
| TF | `README.md` TF 所有权表 |
| 启动组合 | `RUNTIME_FLOWS.md` 典型启动组合 |
| 实施结果 | `../ARCHITECTURE.md` 或对应 `*_IMPLEMENTATION_PLAN.md` |
| 单包内部参数 | 该包 README 与 YAML 注释 |

## 自动完整性检查

在科研工作区执行：

```bash
python3 tools/check_architecture_docs.py
```

脚本扫描底盘工作区、科研包和 vendor 的 `package.xml`，确保每个包都出现在包目录中。它不能
判断文字是否准确，因此代码评审仍需核对职责与接口。

## 图表约定

- 使用 Mermaid，保证 GitHub/编辑器可直接渲染，文本 diff 也可审查。
- 组件图表达静态依赖，sequence diagram 表达运行时顺序，state diagram 表达模式切换。
- 图中只放稳定模块与接口；频繁变化的参数放 YAML/README，不复制进图。
- 箭头方向表示数据或控制的实际方向，不表示“代码目录包含”。
- 可选链路必须标为可选，不能画成默认依赖。

## 文档完成定义

架构变更完成前确认：

1. 新成员能从架构入口找到所属包和顶层 launch。
2. 每个稳定接口能找到唯一生产者和主要消费者。
3. TF 所有权没有两个节点同时声明。
4. 实机、mock、Isaac 和离线 profile 的边界清楚。
5. 安全相关默认值和旁路被明确写出。
6. 完整性检查通过，文档链接有效。
