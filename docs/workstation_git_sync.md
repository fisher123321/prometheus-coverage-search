# 工位电脑获取双机协同覆盖功能包

## 分支约定

- GitHub 仓库：`git@github.com:fisher123321/prometheus-coverage-search.git`
- 双机功能分支：`two-uav-coverage`
- 不使用 GitHub 的 `main` 分支；它已有独立提交，不能直接覆盖。

## 工位电脑的目标路径

工位电脑已有一模一样的普罗米修斯工作空间，因此不克隆第二份仓库，也不切换其 `main` 分支。只把功能包恢复到下面这个**固定路径**：

```text
/home/wjy/Prometheus/Modules/two_uav_coverage_search
```

先确认该路径不存在，或已经备份其中未提交的修改；下面的恢复操作只会覆盖该路径。

## 从 GitHub 只拉取该功能包

```bash
cd /home/wjy/Prometheus
git remote add coverage-source git@github.com:fisher123321/prometheus-coverage-search.git
git fetch coverage-source two-uav-coverage
git restore --source=coverage-source/two-uav-coverage --staged --worktree \
  Modules/two_uav_coverage_search
```

若 `coverage-source` 已存在，不要重复 `git remote add`，改为：

```bash
git remote set-url coverage-source git@github.com:fisher123321/prometheus-coverage-search.git
```

导入后检查：

```bash
git status --short Modules/two_uav_coverage_search
git log -1 --oneline coverage-source/two-uav-coverage
```

这会把远端包恢复到 `/home/wjy/Prometheus/Modules/two_uav_coverage_search`，并让该路径显示为工位仓库中的未提交修改；确认无误后，再按工位仓库自己的分支规范提交。不要在工位工作区执行 `git add .`。

## 可直接交给工位 Codex 的指令

```text
在 /home/wjy/Prometheus 中，不修改或切换现有 main 分支，也不触碰其他文件。
请添加（或校正）GitHub remote：
git@github.com:fisher123321/prometheus-coverage-search.git
然后 fetch 分支 two-uav-coverage，并只恢复路径
Modules/two_uav_coverage_search 到当前工作区的
/home/wjy/Prometheus/Modules/two_uav_coverage_search。完成后输出 git status --short
Modules/two_uav_coverage_search 和该远端分支最新提交；不要执行 git add .、commit、push 或 force 操作。
```

## 常见问题

- `Permission denied (publickey)`：工位电脑尚未配置能访问该 GitHub 账户的 SSH key；先执行 `ssh -T git@github.com` 检查并配置密钥。
- 推送被拒绝：先执行 `git pull --ff-only`；若有本地提交和远端提交同时存在，停止并先检查差异，不能使用 `--force`。
- 本文中的 `/home/wjy` 需替换为工位电脑实际用户名路径。
