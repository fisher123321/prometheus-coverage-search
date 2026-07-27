# 笔记本电脑获取双机协同覆盖功能包

只获取此功能包，不修改 Prometheus 工作空间根目录的 Git 分支。

## 首次获取

```bash
package_dir="$HOME/Prometheus/Modules/two_uav_coverage_searching_pkg"
test ! -e "$package_dir" || { echo "请先备份或移除已有目录：$package_dir"; exit 1; }
git clone --branch two-uav-coverage --single-branch --depth 1 \
  git@github.com:fisher123321/prometheus-coverage-search.git "$package_dir"
cd "$package_dir"
test -f package.xml && git status --short && git log -1 --oneline
```

远程仓库的根目录就是 ROS 功能包。后续按笔记本现有 Prometheus 工作空间的编译方式编译即可。

## 后续更新

```bash
cd "$HOME/Prometheus/Modules/two_uav_coverage_searching_pkg"
git status --short
git pull --ff-only
```

有未提交修改时先处理修改，再执行 `git pull --ff-only`。不要在 `/home/wjy/Prometheus` 根目录执行 `git add .`，也不要使用 `git reset --hard` 或 `git push --force`。

## 给笔记本 Codex 的指令

```text
只操作 $HOME/Prometheus/Modules/two_uav_coverage_searching_pkg。
目录不存在时，从 git@github.com:fisher123321/prometheus-coverage-search.git 的
two-uav-coverage 分支单分支浅克隆；目录存在时先输出 git status --short，
有未提交修改则停止。完成后确认 package.xml 存在并输出 git log -1 --oneline。
不要修改 Prometheus 根仓库，也不要执行 force 或 reset --hard。
```
