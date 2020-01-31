# 動作環境

以下の環境にて動作確認を行っています。

 - ROS Melodic
   - OS: Ubuntu 18.04.3 LTS
   - ROS Distribution: Melodic Morenia 1.14.3
   - Rviz 1.12.16
   - Gazebo 9.0.0
 
# インストール方法

### プログラムのインストール方法

```sh
cd ~/catkin_ws/src/
rm -rf ./crane_x7_ros
git clone https://github.com/Souya25/crane_x7_ros.git
(cd ~/catkin_ws && catkin_make)
``` 

### RealSense D435iのインストール方法

[こちら](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)を参考にインストールします。

# RealSense D435i の取り付け方

以下の写真のように取り付けます。

![写真１](https://github.com/Souya25/crane_x7_ros/blob/master/pic/setting1.png?raw=true)

![写真2](https://github.com/Souya25/crane_x7_ros/blob/master/pic/setting2.png?raw=true)

# システムの起動方法

CRANE_X7の制御信号ケーブルを接続した状態で次のコマンドを実行します。
```sh
sudo chmod 777 /dev/ttyUSB0
roslaunch crane_x7_moveit_config demo.launch
```

次にRealSense D435iを起動させます。
```sh
roslaunch realsense2_camera rs_camera.launch
```

うんこを設置した後、次のコマンドを実行します。

```sh
rosrun crane_x7_robot_design3 opencv.cpp
```

次に別端末で以下のコマンドを実行します。
```sh
rosrun crane_x7_robot_design3 pick_and_pitching.py
```

上記を動作させると[こちら↓](https://www.youtube.com/watch?v=mAbgiquSygA&feature=youtu.be)のような動きになります。

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/mAbgiquSygA/0.jpg)](https://www.youtube.com/watch?v=mAbgiquSygA&feature=youtu.be)

