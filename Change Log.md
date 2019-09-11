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
- **Header**: fcfb478
  - **comments** : fix go charge bugs and give a safe way to get parameters from listener
  - **Actual Change**:
    	1. 修复了充电命令
    	2. 给出了获取参数值的安全方法
     	3. 注意：没有上传parameter.yaml
- **Header**: 4b155b6
  - **comments** : give a template function instead of function overloading and use refernce to improve performance
  - **Actual Change**:
    	1. 修复了切割命令
    	2. 给出了一个模板函数以取代反复重载的函数
     	3. 使用应用取代了拷贝，以提升性能
     	