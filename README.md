## Kinect3D

Kinect3D is a system designed for real time 3D reconstruction using multiply Kinect v2 camera. 

### 环境配置

#### linfreenect2 - kinect v2 SDK

https://gist.github.com/madelinegannon/10f62caba7184b90ea43a734768e5147
配置地址：https://github.com/OpenKinect/libfreenect2
在编译examples的时候，需要改CMakeLists.txt Line56为： FIND_PACKAGE(OpenGL REQUIRED)
在编译tools/streamer_recorder的时候，改动：
Line 61 FIND_PACKAGE(OpenCV REQUIRED HINTS "$ENV{HOME}/lib/opencv3_4_11/share/OpenCV")
Line 35 FIND_PACKAGE(freenect2 REQUIRED HINTS "$ENV{HOME}/lib/freenect2")

#### Opencv3.4

https://opencv.org/releases/
下载opencv_contrib，将其解压到opencv根目录下
在build文件夹中执行cmake指令，指令参数设置见cmake.sh，这个过程中会下载一些依赖
make -j $(nproc)　注：在make过程中如果报错，可能是因为下载的文件未生效，尝试重新cmake
make install
使用pkg-config管理opencv路径
将export PKG_CONFIG_PATH=/home/benjamin/lib/opencv3_4_11/lib/pkgconfig添加到.bashrc中
重启生效
VScode中配置："`pkg-config", "--cflags", "--libs", "opencv`",

#### cuda配置

sudo pacman -S cuda cudnn
Packages (6) gcc8-8.4.0-1 gcc8-libs-8.4.0-1 nvidia-340xx-utils-340.108-1 opencl-nvidia-340xx-340.108-1 cuda-10.2.89-5 cudnn-7.6.5.32-4
The cuda binaries are in /opt/cuda/bin
The cuda samples are in /opt/cuda/samples
The cuda docs are in /opt/cuda/doc
注：GTX750Ti对应的cuda版本是390，若版本不对时，开机进入不了桌面，使用Ctrl+Alt+F3进入命令行，卸载cuda与cudnn。需要使用Rs连同依赖包一起删除。

#### SDK配置

https://gist.github.com/madelinegannon/10f62caba7184b90ea43a734768e5147
https://github.com/OpenKinect/libfreenect2/blob/master/README.md#linux　

#### gcc指定版本编译

下载所需版本http://mirror.hust.edu.cn/gnu/gcc/
安装依赖　./contrib/download_prerequisites 
mkdir gcc-build-6.4.0
cd gcc-build-4.9.4
./configure --enable-checking=release --enable-languages=c,c++ --disable-multilib
make -j $(nproc) 注：这一步用时较久

#### OpenNI2安装

git clone https://github.com/occipital/OpenNI2.gitsudo 
./OpenNI2/Packaging/Linux/install.sh
cd ./OpenNI2/Packaging/Linux/
source OpenNIDevEnvironment
sudo cp ./primesense-usb.rules /etc/udev/rules.d/