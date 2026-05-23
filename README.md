# Week 1

## 上面是底盘

但是貌似joint_state_publisher和robot_state_publisher都不起作用,关系发布不出去
看小鱼的视频以及求助ai都没找到办法,先放上来了
只能手动在rviz2里打开看个静态的()
里面写了两个轮子,主体是长方体

## 容我再学学想想办法

# Week 2

可以显示模型并且能动
但是install的时候依旧不能复制urdf和display.launch.py
是手动复制过去的（）

# Week 3

原来做的莫名其妙打不开了。但是跟小鱼做的突然又能用了，因此使用这个继续完成
添加两全向轮起支撑作用
拼尽全力无法使得模型和map在rviz2里进行相对运动，查询ai说要建立base_link-->odom-->map的tf链
但是base_link-->odom一直无法进行tf
目前只能在gazebo里运动，是两轮差速控制
可以使用键盘控制
display_robot.launch.py以及gazebo_robot.launch.py求助ai制作（）
附赠一个操控小车在gazebo里跑的视频（）
另外，用turtlebot3在仿真环境中slam建图，每次开小车必定翻车，导致建出来一堆乱七八糟的玩意（

# 大作业

一直推不上去，把之前的分支删了还是不行，只能手动传文件了
上传了新增和修改的文件
map里面是建的图
referee两个包是用来定义消息和模拟裁判系统与决策，其中decsion_node是决策树
referee_executor来执行决策（虽然未能完成
referee_keyboard是键盘调试工具，可以手动控制比赛阶段，模拟扣血之类
对原本的启动文件bringup_sim.launch.py进行了一些修改，为了执行决策，但是没啥效果
以及对fake_vel_transform进行了一些修改，也是为了执行决策，也是没啥效果
虽然如此还是感谢deepseek的帮助（）