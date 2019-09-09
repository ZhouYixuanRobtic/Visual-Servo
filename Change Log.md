## Change Log

- **Header** :  21cd2c7
  - **comments** : fix rescue from 100% CPU usage
  - **Actual Change** : 
    1. 解决了部分程序CPU占用率高达100%的情况。
    2. 使相机程序中标签检测部分动态运行
    3. 添加了机械臂IO翻转功能
    4. 添加了gazebo模拟launch文件
- **Header**: e012af1
  - **comments** : add dynamic parameter listener and makes some parameters could be set when node running
  - **Actual Change**:
    	1. 添加了一个在后台工作的参数服务器监视器并且给出了两种获取参数值的封装方法
     	2. 添加了机械臂软件急停功能
     	3. 更改了新的模型