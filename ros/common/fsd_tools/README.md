# fsd_tools

BITFSD公共算法库，将工程中常用的函数进行封装。只需引用该 package 即可调用算法。

## 在ROS包中添加fsd_tools的方式

**步骤一：**在 ` CMakeLists.txt` 中的 ` find_package` 里添加 `fsd_tools` 。例如：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  fsd_common_msgs
  fsd_tools		# fsd_tools 添加位置
  )
```

**步骤2：**在 `package.xml` 中添加依赖。例如：

```xml
<?xml version="1.0"?>
<package format="2">
 	...
  <build_depend>fsd_tools</build_depend>
  <build_export_depend>fsd_tools</build_export_depend>
  <exec_depend>fsd_tools</exec_depend>
  ...
</package>
```

##  程序中的引用方法

在程序的头文件中，添加 include，例如：

```c++
#include "fsd_tools/cubic_spline.h"
#include "fsd_tools/fastindex_tool.h"
```

调用时，需加入fsd_tools的命名空间，例如：

```c++
fsd::Vec_f;
fsd::Spline2D spline(x, y);
fsd::ReflineFastIndexTool<cv::Point2f> g_fastindex_tool;
```



## 程序接口说明

- types

  ```c++
  using Vec_f=std::vector<float>;
  using Poi_f=std::array<float, 2>;
  using Vec_Poi=std::vector<Poi_f>;
  ```

- cubic_spline

  ```c++
  //spline 拟合初始化
  class Spline2D(Vec_f x, Vec_f y)
  //计算曲线长度s位置的(x,y)坐标
  Poi_f calc_postion(float s_t);
  //计算曲线长度s位置的曲率
  float calc_curvature(float s_t);
  //计算曲线长度s位置的倾角
  float calc_yaw(float s_t);
  ```

- fastindex_tool

  ```c++
  // 更新路径点数据
  bool UpdateData(const std::vector<CvPointType>& tj_pts);
  // 输入一个点pt，找到路径点距离pt最近的索引
  int FindClosestIndex(const CvPointType& pt);
  // 输入一个点pt，找到路径点距离pt最近的索引，以及对应frenet坐标系的s和d
  void FindClosestIndexAndFrenetCoord(const CvPointType& pt, int32_t *idx, double *s, double *d);
  ```

  

