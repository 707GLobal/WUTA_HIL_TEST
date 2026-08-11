## Git 子模块说明

`FSD` 是通过 git submodule 引入的独立仓库（`https://github.com/GaoMingHa0/WUTA-FSD.git`），
其提交记录与主仓库相互独立，主仓库只记录 FSD 当前指向的 commit。

### 首次克隆（含子模块）

```bash
git clone <主仓库地址>
cd WUTA_HIL_TEST
git submodule update --init --recursive
```

### 子模块已存在，拉取最新代码

```bash
cd FSD
git pull            # 拉取 FSD 最新提交
cd ..
git add FSD         # 更新主仓库中 FSD 指向的 commit
git commit -m "update FSD"
```

### 更新子模块到远程最新

```bash
git submodule update --remote FSD
git add FSD
git commit -m "update FSD"
```

### 修改 FSD 内部代码

```bash
cd FSD
# 修改代码后提交到 FSD 自己的分支
git add . && git commit -m "xxx"
git push
cd ..
git add FSD
git commit -m "update FSD"
```

## 注意事项

- 不要在子模块目录内直接修改主仓库的内容，FSD 有自己的远程仓库。
- 子模块默认处于 detached HEAD 状态，若需在其上开发，请先切换到对应分支：
  `cd FSD && git checkout <分支名>`
