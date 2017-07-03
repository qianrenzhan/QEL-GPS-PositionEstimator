## QEL-GPS-PositionEstimator ##

PositionEstimator\geo文件夹是GPS经纬度数据转换为位置坐标的函数。

PositionEstimator\gyro是接收陀螺仪数据的相关文件

PositionEstimator\mathlib和PositionEstimator\matrix是数学库，包括欧拉角，旋转矩阵，四元数等之间相互转换的函数。矩阵，向量等数据类型的定义。矩阵，向量运算的函数。

PositionEstimator\LocalPositionEstimator文件夹下是进行卡尔曼滤波的具体实现方法。

PositionEstimator\main.cpp是程序总的执行流程，包括初始化和最终结果写入txt等操作。
