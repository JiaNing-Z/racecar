## Multi Goal ##
#### 包内有两个代码文件 .\src\getpoints.py .\src\goal_loop.py

##### getpoints.py
该文件的主要功能为记录途经点，并将途经点保存至同目录下的test.csv中，运行该脚本时需要在终端中cd到multi_goal\src文件夹，在rviz中可以使用goal point选点，需要注意顺序，结束选点时，在脚本的终端内输入 f 即可

##### goal_loop.py
该文件的主要功能是根据车辆的位置进行多点循环导航。导航点读取的是test.csv中的数据，切换点的时间会有一点的提前，为了保证比较好的路径跟踪效果。