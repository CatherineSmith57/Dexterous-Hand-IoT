# Local Import Note

这个目录是从本地上游工作副本复制过来的：

1. 源路径：`D:\IoT\orca_core\orca_core`

本次复制的目标是：

1. 让 `team_project` 拥有一份可直接引用的 `orca_core` 源码快照
2. 避免把上游仓库历史和本地环境直接混入团队主仓库

本次未复制的内容：

1. `.git`
2. `.venv`
3. `.pytest_cache`
4. `.vscode`
5. `__pycache__`

当前建议：

1. 把这里当作 `third_party` 依赖目录使用
2. 如果后续需要修改 `orca_core`，先在团队内说明改动目的
3. 如果后续想保留更干净的上游关系，再考虑改成 `submodule` 或 `subtree`

