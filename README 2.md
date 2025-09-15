[INFO] [1757827166.480113, 740.760000]: delta : -3.997965
STOP
STOP
STOP
STOP
STOP
[INFO] [1757827166.493232, 740.769000]: kp is : 7026.709727
[ERROR] [1757827166.493309, 740.769000]: bad callback: <function callback_feedback at 0x7f0ea6a17550>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/cyun/9.13/sim/src/Frenet_Planner_Simulation/tracking_control/src/scripts/tracking_Methods/pure_pursuit.py", line 89, in callback_feedback
    ep = min(distances)
ValueError: min() arg is an empty sequence

STOP
[INFO] [1757827166.494130, 740.769000]: ki is : 5401.404924
[INFO] [1757827166.494902, 740.770000]: kd is : 0.789792
[INFO] [1757827166.495492, 740.771000]: linear velocity : 0.012650
[INFO] [1757827166.496064, 740.771000]: target linear velocity : 0.006835
[INFO] [1757827166.496664, 740.772000]: delta : -3.997965
STOP
STOP
STOP
[INFO] [1757827166.503023, 740.779000]: kp is : 7026.714933
[ERROR] [1757827166.503052, 740.779000]: bad callback: <function callback_feedback at 0x7f0ea6a17550>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/cyun/9.13/sim/src/Frenet_Planner_Simulation/tracking_control/src/scripts/tracking_Methods/pure_pursuit.py", line 89, in callback_feedback
    ep = min(distances)
ValueError: min() arg is an empty sequence
到达终点后报错
跟踪效果并不好
既然是frenet规划,那应该是有多条轨迹的,但是现在只看到了一条.
全局规划的接口从哪里来的

这里的全局costmap和局部costmap如果构建的全局和局部地图也确实是我之前设想的那种形式...

全局地图还有一个传递的机制,局部地图传给了全局这个障碍物信息!!!
10s一次的更新频率???
cyun@cyun:~$ rostopic hz /move_base/local_costmap/costmap
subscribed to [/move_base/local_costmap/costmap]
WARNING: may be using simulated time
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
average rate: 0.110
	min: 9.080s max: 9.080s std dev: 0.00000s window: 2
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
average rate: 0.109
	min: 9.080s max: 9.298s std dev: 0.10900s window: 3
no new messages
no new messages
average rate: 0.149
	min: 1.800s max: 9.298s std dev: 3.48434s window: 4
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages

根本原理
costmap_2d 默认打开 latch + update_publish：
整张地图只发布一次（latch），之后只发 /costmap_updates。
只有当 地图内容变化 时才再次推送整张图。
你 rostopic hz 监听的是“整张图”话题，没人保证它一直发。
实际规划器订阅的是 /move_base/local_costmap/costmap_updates，10 Hz 级别的增量格子，足够实时。
2. 那障碍物突然出现，会不会来不及更新？
不会，因为：
更新线程频率由 update_frequency 决定（局部建议 10–20 Hz）。
增量更新在每次 updateBounds() → updateCosts() 后立即发出，延迟 < 1/update_frequency。
规划器（dwa/teb）在 controller_frequency 周期（通常 10–20 Hz）内拿到的就是最新代价



全局的应该是一直都没有更新过的:
cyun@cyun:~$ rostopic hz /move_base/global_costmap/costmap
subscribed to [/move_base/global_costmap/costmap]
WARNING: may be using simulated time
no new messages
no new messages
no new messages
no new messages
no new messages
no new messages
这个稍微正常一点,比较像是实际更新的costmap的所在
cyun@cyun:~$ rostopic hz /move_base/local_costmap/costmap_updates 
subscribed to [/move_base/local_costmap/costmap_updates]
WARNING: may be using simulated time
average rate: 3.328
	min: 0.300s max: 0.301s std dev: 0.00050s window: 3
average rate: 3.571
	min: 0.201s max: 0.301s std dev: 0.03951s window: 6
average rate: 3.750
	min: 0.201s max: 0.301s std dev: 0.04644s window: 10
average rate: 3.547
	min: 0.201s max: 0.500s std dev: 0.08291s window: 12
average rate: 3.589
	min: 0.200s max: 0.500s std dev: 0.07692s window: 15
^Caverage rate: 3.571
	min: 0.200s max: 0.500s std dev: 0.07448s window: 16
当然,它还不止这两个接口,其余接口也都是很丰富的:
cyun@cyun:~$ rostopic hz /move_base/local_costmap/
/move_base/local_costmap/costmap
/move_base/local_costmap/costmap_updates
/move_base/local_costmap/footprint
/move_base/local_costmap/inflation_layer/parameter_descriptions
/move_base/local_costmap/inflation_layer/parameter_updates
/move_base/local_costmap/obstacle_layer/parameter_descriptions
/move_base/local_costmap/obstacle_layer/parameter_updates
/move_base/local_costmap/parameter_descriptions
/move_base/local_costmap/parameter_updates
这个时候我们再回看launch文件,发现其实他也是非常容易看move_base在这里的作用的
	<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="use_sim_time" value="true" />
    <param name="base_global_planner" value="$(arg base_global_planner)" />
    <!-- <param name="base_local_planner" value="$(arg base_local_planner)" />    -->
    <!-- <rosparam file="$(find car_demo)/cfg/teb_local_planner_params.yaml" command="load"/>    -->
    <rosparam file="$(find frenet_planner)/cfg/costmap_common.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find frenet_planner)/cfg/costmap_common.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find frenet_planner)/cfg/costmap_local.yaml" command="load" />
    <rosparam file="$(find frenet_planner)/cfg/costmap_global.yaml" command="load" />
  </node>
全局规划应该给出了全局路径,不过竟然是这个节点发出来的???
cyun@cyun:~$ rostopic info /global_path 
Type: nav_msgs/Path
Publishers: 
 * /frenet_planner (http://cyun:41491/)
Subscribers: 
 * /rviz (http://cyun:43291/)
并且没有进行频率控制
cyun@cyun:~$ rostopic hz /global_path 
subscribed to [/global_path]
WARNING: may be using simulated time
average rate: 438.323
	min: 0.000s max: 0.004s std dev: 0.00100s window: 367
average rate: 431.750
	min: 0.000s max: 0.004s std dev: 0.00102s window: 719
^Caverage rate: 428.014
	min: 0.000s max: 0.004s std dev: 0.00102s window: 878
具体怎么发布的就需要再好好看看了.
然后是局部轨迹的信息
cyun@cyun:~$ rostopic info /frenet_path 
Type: nav_msgs/Path
Publishers: 
 * /frenet_planner (http://cyun:41491/)
Subscribers: 
 * /rviz (http://cyun:43291/)
 * /tracker (http://cyun:37641/)
最后给的是跟踪节点.
cyun@cyun:~$ rostopic hz /frenet_path 
subscribed to [/frenet_path]
WARNING: may be using simulated time
average rate: 505.376
	min: 0.000s max: 0.003s std dev: 0.00084s window: 424
average rate: 501.765
	min: 0.000s max: 0.004s std dev: 0.00086s window: 854
average rate: 501.758
	min: 0.000s max: 0.009s std dev: 0.00089s window: 1285
average rate: 500.584
	min: 0.000s max: 0.009s std dev: 0.00088s window: 1716
^Caverage rate: 500.391
	min: 0.000s max: 0.009s std dev: 0.00087s window: 1920
也是没有频率控制的,而且就像我之前说的
作为frenet的规划,路径应该是不止这一条的,其他的路径在哪里?怎么可视化出来?_---markerarray

轨迹跟踪和速度跟踪,输入输出都是什么,从哪里来的
  <!-- Run the tracker node -->
  <node pkg= "tracking_control" name="tracker" type="pure_pursuit.py" output="screen"/>
  
  <!-- Run the controller node --> 
  <node pkg= "tracking_control" name="controller" type="PID_MIT_velocity_controller.py" output="screen"/>



scan是30hz的.


任务1.基于rslidar_points的全局与局部costmap研究
任务2.基于frenet坐标的规划与可视化
任务3.基于frenet规划的速度与路径跟踪(轨迹跟踪)