# 工位电脑获取双机协同覆盖功能包

## 分支约定

- GitHub 仓库：`git@github.com:fisher123321/prometheus-coverage-search.git`
- 双机功能分支：`two-uav-coverage`
- 不使用 GitHub 的 `main` 分支；它已有独立提交，不能直接覆盖。
- `two-uav-coverage` 是轻量分支，仓库根目录就是本功能包本身，不包含整套 Prometheus 历史。

## 工位电脑的目标路径

工位电脑已有一模一样的普罗米修斯工作空间，因此不克隆第二份普罗米修斯，也不切换其 `main` 分支。只把功能包克隆到下面这个**固定路径**：

```text
/home/wjy/Prometheus/Modules/two_uav_coverage_search
```

先确认该路径不存在；若已有旧包，先自行备份其中的未提交修改。下面的操作不会触碰 `/home/wjy/Prometheus` 下的其他目录。

## 从 GitHub 拉取该功能包

```bash
git clone --branch two-uav-coverage --single-branch --depth 1 \
  git@github.com:fisher123321/prometheus-coverage-search.git \
  /home/wjy/Prometheus/Modules/two_uav_coverage_search
```

导入后检查：

```bash
cd /home/wjy/Prometheus/Modules/two_uav_coverage_search
git status
git log -1 --oneline
```

这会在最终路径创建一个独立的 Git 功能包仓库；它可由上层 Prometheus 工作空间正常编译。后续修改和提交都在此功能包目录内完成，不要在 `/home/wjy/Prometheus` 根目录执行 `git add .`。

后续同步或推送功能包修改：

```bash
cd /home/wjy/Prometheus/Modules/two_uav_coverage_search
git pull --ff-only
git add .
git commit -m "Update two UAV coverage search"
git push
```

## 可直接交给工位 Codex 的指令

```text
不要修改或切换 /home/wjy/Prometheus 的现有 main 分支，也不触碰其中其他文件。
确认 /home/wjy/Prometheus/Modules/two_uav_coverage_search 不存在后，将 GitHub 仓库
git@github.com:fisher123321/prometheus-coverage-search.git 的 two-uav-coverage 分支
以单分支、浅克隆方式直接克隆到
/home/wjy/Prometheus/Modules/two_uav_coverage_search。完成后进入该目录，输出 git status
和 git log -1 --oneline；不要执行 force 操作。
```

## 常见问题

- `Permission denied (publickey)`：工位电脑尚未配置能访问该 GitHub 账户的 SSH key；先执行 `ssh -T git@github.com` 检查并配置密钥。
- 推送被拒绝：先执行 `git pull --ff-only`；若有本地提交和远端提交同时存在，停止并先检查差异，不能使用 `--force`。
- 本文中的 `/home/wjy` 需替换为工位电脑实际用户名路径。
