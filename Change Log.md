## Change Log

- **Header** :  21cd2c7
  - **Branch** : master
  - **comments** : fix rescue from 100% CPU usage
  - **Actual Change** : 
    1. 解决了部分程序CPU占用率高达100%的情况。
    2. 使相机程序中标签检测部分动态运行
    3. 添加了机械臂IO翻转功能
    4. 添加了gazebo模拟launch文件
- **Header**: e012af1
  - **Branch** : master
  - **comments** : add dynamic parameter listener and makes some parameters could be set when node running
  - **Actual Change**:
     1. 添加了一个在后台工作的参数服务器监视器并且给出了两种获取参数值的封装方法
     2. 添加了机械臂软件急停功能
     3. 更改了新的模型
- **Header**: fcfb478
  - **comments** : fix go charge bugs and give a safe way to get parameters from listener
  - **Branch** : master
  - **Actual Change**:
    1. 修复了充电命令
    2. 给出了获取参数值的安全方法
    3. 注意：没有上传parameter.yaml
- **Header**: 4b155b6
  - **comments** : give a template function instead of function overloading and use refernce to improve performance
  - **Branch** : master
  - **Actual Change**:
    1. 修复了切割命令
    2. 给出了一个模板函数以取代反复重载的函数
    3. 使用引用取代了拷贝，以提升性能 
 - **Header**: ac770e0
   - **comments** : add two services and accomplish whole curve
   - **Branch** : master
   - **Actual Change**:
     1. 给出了便于调试的两个服务，分别使机械臂竖直和放下
     2. 使切割曲线变得完整
 - **Header**:  76c191
    - **comments** : update with scale speed function and linear move and fix some bugs
    - **Branch** : master
    - **Actual Change**:
        1. 修复了参数监视器析构的bug，并且使单线程监听多个参数，节约了资源
        2. 在相机节点和主节点中添加了看门狗机制以监视话题消失
        3. 在主函数中添加了两种线性运动，和轨迹速度缩放
        4. 使tag检测能居中
        5. 添加了依赖安装脚本
 - **Header**:  1ecf06
    - **comments** : update a new mechanic control strategy
    - **Branch** : mechanical-dev
    - **Actual Change**:
        1. 新增加了一个分支以适应新的机构
        2. 添加了一种简易的使末端线性运动的服务
        