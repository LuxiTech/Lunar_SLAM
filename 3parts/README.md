# 第三方依赖部署

`3parts/` 中的第三方源码使用 Git Submodule 管理。主仓库只保存每个仓库的
远程地址和固定提交，不保存第三方源码副本。

## 新设备首次部署

推荐在克隆主仓库时同时拉取所有子模块：

```bash
git clone --recurse-submodules <主仓库地址> lunar_slam
cd lunar_slam
./scripts/setup_3parts.sh --runtime cpu
```

如果主仓库已经克隆：

```bash
git pull
git submodule sync --recursive
git submodule update --init --recursive
./scripts/setup_3parts.sh --runtime cpu
```

`--recursive` 不能省略，因为 HLoc 自身还包含 SuperPoint、D2-Net 等嵌套子模块。

## 安装方式

只拉取源码，不安装 Python 依赖和模型：

```bash
./scripts/setup_3parts.sh --runtime none --skip-models
```

安装 CPU 回退环境：

```bash
./scripts/setup_3parts.sh --runtime cpu
```

CPU 依赖来自 `project/luxi_hloc/requirements-cpu.txt`，安装目标是
`3parts/hloc_python/`，不会写入系统 Python。

在 Jetson 上安装 GPU 环境时，需要提供与 JetPack/CUDA 版本匹配的 Python wheel
索引。不要直接使用普通 PyPI 的 Torch 代替 Jetson CUDA wheel：

```bash
LUXI_HLOC_GPU_INDEX_URL=<Jetson-wheel-索引地址> \
  ./scripts/setup_3parts.sh --runtime gpu
```

也可以通过参数提供：

```bash
./scripts/setup_3parts.sh \
  --runtime gpu \
  --gpu-index-url <Jetson-wheel-索引地址>
```

GPU 模式会执行以下操作：

1. 将通用依赖安装到 `3parts/hloc_python/`；
2. 将 GPU Torch/TorchVision 覆盖层安装到 `3parts/hloc_gpu_python/`；
3. 验证 PyTorch 能否识别 CUDA 设备。

本项目验证过的基线是 Jetson AGX Orin、CUDA 12.6、PyTorch 2.8.0。

## 模型缓存

默认情况下，安装脚本会预下载 HLoc 当前使用的 NetVLAD 和 LightGlue 权重。模型
缓存目录为：

```text
3parts/hloc_models/
```

该目录是可重新下载的本机资源，不进入 Git。若当前只需要源码或暂时没有网络：

```bash
./scripts/setup_3parts.sh --runtime cpu --skip-models
```

后续首次构建 HLoc 地图或启动定位时，上游 HLoc/LightGlue 也会按需下载缺少的模型。

## 更新第三方源码

普通部署应使用主仓库锁定的提交：

```bash
git submodule update --init --recursive
```

不要在部署设备上直接执行 `git submodule update --remote`，否则可能跳过主仓库
验证过的版本。如果确实需要升级，应先在开发机中切换子模块提交、完成构建与测试，
再提交主仓库中的 submodule 指针。

## Git 记录位置

- `.gitmodules`：记录子模块路径和远程仓库地址；
- 主仓库 Git 树中的 `160000` 条目：记录每个子模块的准确提交；
- `project/luxi_hloc/requirements-*.txt`：记录 CPU/GPU Python 版本；
- `3parts/hloc_models/`：本机模型缓存，不受 Git 追踪。

可用下面的命令确认 `3parts` 源码是否已经正确提交为子模块：

```bash
git ls-files --stage 3parts
git submodule status --recursive
```

源码目录对应的模式应为 `160000`。如果显示大量普通文件，而不是 `160000`，
说明主仓库尚未完成从源码副本到 Submodule 的转换。
