## Chapter02. Docker命令大全

### 1.容器命令

如下命令省去了docker前缀

#### 生命周期

> - run  容器运行
> - start/stop/restart  启动/保存停止/重启 一个或多个已经被停止的容器
> - kill  立即停止一个容器进程
> - rm 删除一个或者多个容器
> - pause/unpause 暂停/恢复容器进程
> - create  创建一个新的容器但不启动它，同run
> - exec  在运行的容器中执行命令

``` shell
docker run [OPTIONS] IMAGE [COMMAND] [ARG...]
```

常用的options

- **-d:** 后台运行容器，并返回容器ID；
- **-i:** 以交互模式运行容器，通常与 -t 同时使用；
- **-P:** 随机端口映射，容器内部端口**随机**映射到主机的高端口
- **-p:** 指定端口映射，格式为：**主机(宿主)端口:容器端口**
- **-t:** 为容器重新分配一个伪输入终端，通常与 -i 同时使用；

#### 容器的操作

> - ps  列出容器
> - inspect  获取容器/镜像的元数据。
> - top  查看容器中运行的进程信息，支持 ps 命令参数。
> - attach   连接到正在运行中的容器。
> - events  从服务器获取实时事件
> - logs  获取容器的日志
> - wait  阻塞运行直到容器停止，然后打印出它的退出代码。
> - export  将文件系统作为一个tar归档文件导出到STDOUT
> - port  列出指定的容器的端口映射，或者查找将PRIVATE_PORT NAT到面向公众的端口。

#### 容器rootfs命令

> - commit  从容器创建一个新的镜像。
> - cp  用于容器与主机之间的数据拷贝。
> - diff   检查容器里文件结构的更改

### 2镜像命令

#### 镜像仓库

> - login  登陆到一个Docker镜像仓库，如果未指定镜像仓库地址，默认为官方仓库 Docker Hub
>
> - logout  登出一个Docker镜像仓库，如果未指定镜像仓库地址，默认为官方仓库 Docker Hub
> - pull   从镜像仓库拉取或者更新指定镜像
> - push  将本地的镜像上传到镜像仓库,要先登陆到镜像仓库
> - search  从Docker Hub查找镜像

#### 本地镜像管理

> - images : 列出本地镜像。
> - **rmi :** 删除本地一个或多少镜像。
> - **build** 命令用于使用 Dockerfile 创建镜像。
> - **history :** 查看指定镜像的创建历史。
> - **save :** 将指定镜像保存成 tar 归档文件。
> - **load :** 导入使用 docker save 命令导出的镜像。
> - **import :** 从归档文件中创建镜像。

#### Docker配置信息命令

> - info : 显示 Docker 系统信息，包括镜像和容器数。。
>
> - version :显示 Docker 版本信息。

