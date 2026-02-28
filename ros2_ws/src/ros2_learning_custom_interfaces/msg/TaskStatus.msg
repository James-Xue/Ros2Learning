# 任务编排节点 (TaskRunner) 的状态汇报消息。
# 由 ros2_learning_task_runner 发布到 /task_status 话题。

# 状态机当前状态字符串，如 "IDLE"、"NAVIGATING_TO_PICKUP" 等
string state

# 当前阶段描述
string phase

# 已完成的采集点索引（0-based）
uint32 pickup_index

# 采集点总数
uint32 pickup_total

# 最近一次错误描述，无错误时为空字符串
string last_error
