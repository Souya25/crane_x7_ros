## 動作環境

以下の環境にて動作確認を行っています。


- ROS Melodic
  - OS: Ubuntu 18.04.3 LTS
  - ROS Distribution: Melodic Morenia 1.14.3
  - Rviz 1.12.16
  - MoveIt! 1.13.3
  - Gazebo 9.0.0

## インストール方法



- `Subversion`を使用して本パッケージをダウンロードします。

  ```bash
  cd ~/catkin_ws/src/crane_x7_ros
  svn export https://github.com/ShioriSugiyama/crane_x7_ros/trunk/crane_x7_robot_design3
  ```
  ダウンロードして、実行許可がない場合は次のコマンドで許可与えてください。
   ```bash
   chmod 777　ファイル名
    ```
-  追加したパッケージをコンパイルしてくれるように~/catkin_ws/src/crane_x7_ros/crane_xにあるpackage.xmlに記入

    以下の写真のように26行目に以下の文を追加
  
    ```
      <run_depend>crane_x7_robot_design3</run_depend>
    ```

![cmake](https://github.com/ShioriSugiyama/crane_x7_ros/blob/image/image/a3fe030e89c33e0a895bda411ab39625.png "cmake")
- `catkin_make`を使用して本パッケージをビルドします。

  ```bash
  cd ~/catkin_ws && catkin_make
  ```

# crane_x7_robot_design3

CRANE-X7のためのパッケージ、 `realsensD435i` を使って画像処理して`crane_x7`を動作させるためのパッケージです。

| 使用するパッケージ内プログラム名 | 機能説明 |
----|----
| opencv.cpp  | 赤色の物体を表示する |
| explore_move.py | cranex_7が物体を探すために探索動作を行う |
| pick_up_move.py | 物体が検知した場合、物体をつかみに行く |



## システムの起動方法

CRANE_X7の制御信号ケーブルを制御用パソコンへ接続します。
Terminalを開き、`crane_x7_moveit_config`の`demo.launch`を起動します。


### 実機

- 実機で動作を確認する場合、まず `初めにrealsenseD435iを起動` させます。
> realsenseD435に搭載されているIMUの影響により画面が反転したりしてしまうのでそれを防ぐための対処法です。
>（とりあえず、一回realsenseD435iを動かせばなんとかなります）

このパッケージにあるプログラムを動かし、反転をし続けないようにします。
次のコマンドを実行します。

```sh
roslaunch realsense2_camera rs_camera.launch 
rosrun crane_x7_robot_design3 cam 
```
以下のようなwindow[RGB_image]が表示されます。
window[RGB_image]は閉じずにそのままにしてOKです。
![RGB_image](https://github.com/ShioriSugiyama/crane_x7_ros/blob/image/image/1794985.jpg "RGB_image")
> これで、IMUの影響による画像反転は防がれます。


- 制御信号ケーブルを接続した状態で次のコマンドを実行します。

```sh
roslaunch crane_x7_moveit_config demo.launch 
```

ケーブルの接続ポート名はデフォルトで`/dev/ttyUSB0`です。
別のポート名(例: /dev/ttyUSB1)を使う場合は次のコマンドを実行します。

```sh
roslaunch crane_x7_moveit_config demo.launch port:=/dev/ttyUSB1
```

- 次にpick_up_move.pyを実行します。
> 物体が検知に完了時、動き始めるので実行させてもすぐにcrane_x7動きません。

```sh
rosrun crane_x7_robot_design3 pick_up_move.py
```
物体検知完了時の動き
![crane_x7_e](https://github.com/ShioriSugiyama/crane_x7_ros/blob/image/image/98a708fc4e160b0428ef70ab31432807.gif "crane_x7_e")
- 最後にexplore_move.pyを実行します。
> 物体検知するための、探索動作を行います。

```sh
rosrun crane_x7_robot_design3 explore_move.py
```
物体探索動作

![crane_x7_m](https://github.com/ShioriSugiyama/crane_x7_ros/blob/image/image/2163519889481d2ef15215dd37d131f6.gif "crane_x7_m")

実際の動作はこちらになります。
[YoutubeMovie](https://youtu.be/2-XMopff29E)

