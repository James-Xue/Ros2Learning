---
name: codex
description: 调用 codex-cli 对代码或目录进行分析、总结、审查、review。当用户提到"用 codex 分析"、"调用 codex"、"让 codex 总结"、"codex-cli"、"codex review"、"用 codex 检查"、"codex 排查" 时使用此 skill。不适用于仅讨论 codex 概念而不实际调用命令的场景。
version: 1.1.0
disable-model-invocation: true
allowed-tools: Bash
---

# Codex CLI 调用规范

## 正确命令格式（非交互执行）

```bash
codex exec --full-auto -C "<绝对路径>" "<prompt>"
```

> **为什么必须用 `exec` 子命令**：顶层 `codex --full-auto` 启动的是交互式 TUI，无法在脚本中使用。`codex exec` 才是非交互批处理模式。

## 快速自检清单（调用前必查）

- [ ] 使用了 `exec` 子命令（非顶层 `codex`）
- [ ] `-C` 后跟的是**绝对路径**（不是相对路径）
- [ ] prompt 中包含"只读分析，不要修改任何文件"
- [ ] 设置了 `timeout: 180000`（Bash 工具的外层超时，非 codex 参数）
- [ ] 目标目录在 Git 仓库内，否则加 `--skip-git-repo-check`

## 参数速查

| 场景 | 命令 |
|---|---|
| 分析某个包（标准） | `codex exec --full-auto -C "/abs/path" "分析...不要修改文件"` |
| 目标不在 Git 仓库 | `codex exec --full-auto --skip-git-repo-check -C "/abs/path" "..."` |
| 指定模型 | `codex exec --full-auto -m gpt-4o -C "/abs/path" "..."` |
| 保存最后一条输出 | `codex exec --full-auto -C "/abs/path" -o /tmp/out.md "..."` |

> `-o / --output-last-message`：仅保存模型最后一条回复到文件，不是完整事件流。

## 调用步骤

1. 确认目标目录的**绝对路径**（用 Glob 或 Bash 查询）
2. 判断目标是否为 Git 仓库（`git -C <dir> rev-parse 2>/dev/null`），不是则加 `--skip-git-repo-check`
3. 构造命令：`codex exec --full-auto -C "<绝对路径>" "<prompt>"`
4. Bash 工具设置 `timeout: 180000`（codex 分析可能较慢，这是外层执行器超时配置）
5. 若用户未指定语言，默认在 prompt 中加"请用中文回答"

## 最小安全 Prompt 模板

```
请用中文[分析/总结/审查]<目标>的[架构/问题/...]。
只读分析，不要修改任何文件，不要执行任何写操作。
```

## 常见错误（禁止）

```bash
# ❌ 顶层 codex 是交互模式，不能用于批处理
codex --full-auto "..."

# ❌ --approval-mode 参数不存在（正确参数是 --ask-for-approval 或 -a）
codex exec --approval-mode full-auto "..."

# ❌ -q 参数不存在
codex exec -q "..."

# ❌ 使用相对路径（含空格或特殊字符时必炸）
codex exec --full-auto -C ./some/path "..."

# ❌ 在非 Git 目录运行时未加 --skip-git-repo-check
codex exec --full-auto -C /tmp/noRepo "..."

# ❌ timeout 写在 codex 命令里（它是 Bash 工具的外层参数，不是 codex 参数）
codex exec --timeout 180000 ...

# ✅ 正确
codex exec --full-auto -C "/root/Ros2Learning/ros2_ws/src/some_pkg" \
  "请用中文分析这个包的架构。只读，不要修改任何文件。"
```

## 失败回退策略

1. 命令报错：先执行 `codex exec --help` 核对参数名，不要盲目重试
2. 目录不存在：用 Glob/Bash 确认路径后再调用
3. 非 Git 仓库报错：追加 `--skip-git-repo-check`
4. 超时：缩小分析范围，只针对关键文件出 prompt

## 版本说明

本文档基于 `codex v0.106.0`。CLI 升级后请用 `codex --help` 和 `codex exec --help` 校验参数是否变化。
