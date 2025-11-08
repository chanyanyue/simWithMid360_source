# 仿真环境设置步骤

## 1. 克隆代码仓库

```bash
git clone https://github.com/chanyanyue/simWithMid360_source.git
```

## 2. 移动文件位置

完成了前一章讲义的同学如果 `~/` 目录下已经有 `PX4-Autopilot` 文件夹，重命名或者删除旧的文件夹，防止冲突

```bash
  cd ~/simWithMid360_source
  mv livox_ws PX4-Autopilot sim_ws ~/
  cd ..
  ls
  rm -rf simWithMid360_source
```

完成后可以看到 `livox_ws`、`sim_ws`、`PX4-Autopilot` 三个工作空间在 `~/` 目录下。`simWithMid360_source` 这个文件夹可以删除。

## 3. MAVROS 配置

MAVROS 的配置与上一章讲义相同，已完成的可忽略。

## 4. PX4-Autopilot 编译

**注意：** 这一步编译的是新的 PX4-Autopilot，而不是前一章讲义的 PX4-Autopilot，不要编译错了

```bash
cd ~/PX4-Autopilot
sudo apt-get update
sudo bash ./Tools/setup/ubuntu.sh
sudo apt-get install libboost-all-dev gcc-arm-none-eabi
pip3 install --user kconfiglib jinja2 pyyaml jsonschema empy==3.3.4 pyros-genmsg toml numpy packaging future
make px4_sitl_default gazebo
```

`pip3` 报错提示缺什么就安装什么（问 AI），然后重复执行上一步命令直到编译完成。编译完成后会弹出 Gazebo，关闭即可。

```bash
# 完成上面的编译后
cd ~/PX4-Autopilot
make px4_fmu-v5_default
```

## 5. livox_ws 编译

```bash
sudo apt update
cd ~/livox_ws/src/Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
cd ~/livox_ws/src/livox_ros_driver2
pip install catkin-pkg
sudo apt-get update
sudo apt-get install libceres-dev
./build.sh ROS1
```

`cmake` 报错提示缺什么就安装什么（问 AI），优先使用 `sudo apt install` 和`pip/pip3`的方式下载，实在不行才考虑源码编译缺失的软件包。然后重复执行上一步命令直到编译进度条开始。

若编译的进度条开始后，出现部分报错，无法编译完成，继续执行下面的步骤：

```bash
cd ~/livox_ws
catkin_make
```

编译完成即可。

### 环境变量配置

将下面的内容添加到 `~/.bashrc` 的文件末尾：

    export GAZEBO_MODEL_PATH="$HOME/sim_ws/src/tutorials/tutorial_gazebo/models:$GAZEBO_MODEL_PATH"
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/noetic/lib
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/noetic/lib
    source $HOME/livox_ws/devel/setup.bash
    source $HOME/PX4-Autopilot/Tools/setup_gazebo.bash $HOME/PX4-Autopilot/ $HOME/PX4-Autopilot/build/px4_sitl_default

    source $HOME/sim_ws/devel/setup.bash
    export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/PX4-Autopilot/:$HOME/PX4-Autopilot/Tools/sitl_gazebo"

## 6. sim_ws 编译

```bash
cd ~/sim_ws
source ~/livox_ws/devel/setup.bash
catkin_make
```

编译完成即可。

## 7. mid360.csv 配置

```bash
gedit ~/sim_ws/src/tutorials/tutorial_gazebo/models/livox_mid360/model.sdf
```

在第 69 行的

`<csv_file_name>/home/airhust/sim_ws/src/tutorials/tutorial_gazebo/models/livox_mid360/mid360.csv</csv_file_name>`

中把 `airhust` 改为你自己的用户名。

## 8. 运行例程

```bash
sudo apt install tmux
sudo apt-get install ros-noetic-tf2-sensor-msgs
cd ~/sim_ws/src/tutorials/tutorial_gazebo/shell
./basic.sh
```
