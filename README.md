# ROLL

## 修改过的代码
修改代码实现在RVIZ中发布位姿，实现调整初始位姿  

在mapOptmization.cpp文件的void updateInitialGuess()函数中修改以下代码（大约在1927行，具体行数不确定，因为改过代码，主要看函数）
```C++
if(localizationMode && poseGuessFromRvizAvailable)
{
    Eigen::Affine3f tWrong = trans2Affine3f(transformTobeMapped);
    Eigen::Affine3f tCorrect = relocCorrection*tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f affine_body_to_map = tCorrect;//TODO:将RIVZ发布的位姿信息更新到当前位姿关系
    affine_imu_to_map = affine_body_to_map*affine_imu_to_body;
    affine_odom_to_map = affine_imu_to_map*affine_imu_to_odom.inverse();
    for(int i = 0;i < 6;i++){
        rvizGuess[i] = transformTobeMapped[i];
        transformBeforeMapped[i] = transformTobeMapped[i];
    }
    printTrans("The reloc pose given by rviz: ",transformTobeMapped);
    poseGuessFromRvizAvailable = false;
    tryReloc = true;
    return;
}
```

## 目前的工作

1. 关键帧构成的全局地图，做一个降采样，仅做显示使用
   
   ROLL只使用关键帧做全局地图的显示，以最近邻找到半径阈值内的关键帧，经过体素降采样后publish以显示

2. 只用一部分关键帧做定位，能否将后续的建图做出来
   
   当机器人访问到地图中没有的地方（以LOAM中的内点比率做阈值判断标准），ROLL可以实现构建临时地图，但是需要当临时地图和已有全局地图的内点比率大于一定阈值（类似回环），才会将临时地图融合进全局地图（此部分包含GTSAM的因子图优化）。
   
   下图是原始用于定位的关键帧地图的结束位置
   ![定位图](./pic/定位.png "定位图")

   下图是ROLL访问到新的地图构建出的地图，可见定位地图中后续缺少的地方，经过ROLL建图添加到了地图中。
   ![新建图局部](./pic/新建图局部.png "新建图局部")

   下图是当前数据包全局地图
   ![全局地图](./pic/全局地图.png "全局地图")

   下图是ROLL重新构建的全局地图，可见最上方的地图并没有构建出来，这正是由于没有和定位地图发生临时地图融合（LOAM计算的内点得分未达到阈值）导致的。
   ![新建图全局](./pic/新建图全局.png "新建图全局")

   视频：[nclt数据的重定位+地图更新](/ROLL/vedio/loc_1.mp4)
   
   <font color=OrangeRed>ps：</font>会考虑一个情况是，第一次构建临时地图融合到全局地图后，那第二次触发构建临时地图，这次匹配是否会将第一次的临时地图考虑进去。由于当前数据不满足该情况，无法验证。

3. 对LTAOM的关键帧做角点和面点的提取，用于ROLL定位
   
   将ROLL中提取关键帧中的角点和面点的代码单独写成一个[corner_surface.cpp](/ROLL/code/corner_surface.cpp)，其中将livox雷达的数据转为256线的velodyne数据，用于在每条线上提取角点和面点。同时由于LTAOM输出关键帧为基于建图原点坐标系，而ROLL使用关键帧基于当前雷达坐标系，所以在其中加入了pose的逆矩阵转换，解决了ROLL加载关键帧错误的问题。

   视频：[室内楼梯场景定位](/ROLL/vedio/loc_2.mp4)
   
4. ROLL中重定位的实现原理（rviz中的绿线和红线）
   
   绿线：/roll/mapping/path_fusion  
   红线：/roll/mapping/path_fusion_vins  

   绿线位姿为实时估计位姿（雷达坐标系基于该位姿显示），红线为经过GTSAM优化后的位姿。

5. 大型数据集测试
   
   使用LTAOM对nclt数据集(2012-02-02，包含约1.5h，5.5km的数据)建图并提取关键帧中的角点和面点，再使用nclt(2013-01-10)实现定位及临时建图融合。实验结果表明，在nclt(2013-01-10)数据的机器人运动轨迹不完全重和，且部分雷达数据不相同的情况下，依然可以实现定位和临时建图。但是定位过程中，机器人姿态会出现轻微抖动，怀疑是定位模式下的位姿优化导致。

   视频：[大型数据集下的建图与定位](/ROLL/vedio/nclt不同数据集的建图与定位.mp4)

6. 算法内存优化
   
   将原始算法在开始时一次性加载所有关键帧，改为在只加载当前位置附近的关键帧，随着位置的变动，加载不同的关键帧（每隔几米，加载半径内的关键帧）。

7. 想办法提高里程计的发布频率

   或者是后续使用imu做里程计插值，提高频率，用于高速场景下的导航。

8. 减小算法CPU占用率

   在改完关键帧读取策略后，内存降下去了，但是CPU占用率上去了，相比于之前的约70%，现在变成了200%。   
   经过多次比较实验，发现是关键帧密度问题，导致算法输入点云量太多，进而计算量增大。LTAOM采用等距采样子图，再以等数量子图采样获得关键帧，当前算法中阈值得到关键帧约每2m一帧，全局关键帧2155；但是经过ROLL采样过滤后，全局关键帧仅540，且体素栅格相比LTAOM更大，所以计算量相对更小，实测约50%。

## 算法框架

   ROLL为基于FAST-LIO2的一种建图+定位算法，同时，基于GTSAM的因子图优化，实现了当机器人运动至未知区域时，内点得分（计算方式类似LOAM）低于阈值，触发临时建图机制，当回环访问的内点得分高于一定阈值则将临时建图部分融合到全局地图，以更新全局地图。


