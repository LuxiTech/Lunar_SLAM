# Docker 镜像构建说明

本目录保存上游 `metavision_driver` 项目的 Dockerfile，主要用于 CI 或跨平台构建验证。当前 `Ultimate SLAM` 工作空间日常开发不依赖这些 Docker 镜像。

## 构建基础镜像

进入 `.github/docker` 目录后构建基础镜像：

```bash
os_flavor=focal
ros1_flavor=noetic
ros2_flavor=galactic
your_dockerhub_name=<你的 DockerHub 用户名>
combined=${os_flavor}_${ros1_flavor}_${ros2_flavor}
docker build -t ${your_dockerhub_name}/${combined} - < Dockerfile.${combined}
```

推送镜像：

```bash
docker login
docker push ${your_dockerhub_name}/${combined}
```

## 构建带 MetaVision SDK 的镜像

如果从零开始构建，需要先从 Prophesee 官网下载 SDK，并取得对应 apt 仓库配置，再写入 `_metavision` 镜像构建过程。

构建并推送：

```bash
docker build -t ${your_dockerhub_name}/${combined}_metavision - < Dockerfile.${combined}_metavision
docker push ${your_dockerhub_name}/${combined}_metavision
```

## 构建 Jetson / ARM 镜像

跨架构构建前先安装并启用 qemu：

```bash
sudo apt-get install qemu binfmt-support qemu-user-static
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

构建并推送 Jetson 镜像：

```bash
docker build . -f Dockerfile.jetson_r34_noetic_metavision -t ${your_dockerhub_name}/jetson_r34_noetic_metavision
docker push ${your_dockerhub_name}/jetson_r34_noetic_metavision
```
