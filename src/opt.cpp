void GlobalOptimization::optimize()
{
    while (true)
    { // 无限循环，持续进行优化
        if (newGlobalLocPose)
        { // 如果有新的全局定位位姿
            if (reInitialize == true)
                reInitialize = false; // 如果需要重新初始化，取消重新初始化标志

            TicToc opt_time;                       // 计时器，用于测量优化时间
            newGlobalLocPose = false;              // 重置新全局定位位姿标志
            NonlinearFactorGraph gtSAMgraphTM;     // 创建一个非线性因子图
            Values initialEstimateTM;              // 创建初始估计值
            ISAM2 *isamTM;                         // 创建一个指向 ISAM2 的指针
            Values isamCurrentEstimateTM;          // 创建一个变量保存当前的ISAM2估计
            ISAM2Params parameters;                // 创建 ISAM2 参数
            parameters.relinearizeThreshold = 0.1; // 设置重线性化阈值
            parameters.relinearizeSkip = 1;        // 设置重线性化跳过步数
            isamTM = new ISAM2(parameters);        // 用参数初始化 ISAM2

            mPoseMap.lock(); // 锁定位姿地图，防止并发访问

            int length = localPoseMap.size(); // 获取局部位姿地图的大小
            map<double, vector<double>>::iterator iterIni, iterLIO, iterLIOnext, iterGlobalLoc;
            iterIni = globalPoseMap.begin(); // 初始化迭代器指向全局位姿地图的开头
            int i = 0;                       // 索引初始化为0
            int found = 0;                   // 初始化找到的全局定位位姿计数器为0

            // 遍历局部位姿地图
            for (iterLIO = localPoseMap.begin(); iterLIO != localPoseMap.end(); iterLIO++, i++, iterIni++)
            {
                iterLIOnext = iterLIO;
                iterLIOnext++; // 获取下一个局部位姿

                // 如果有下一个局部位姿
                if (iterLIOnext != localPoseMap.end())
                {
                    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances(
                        (Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());                         // 创建一个对角噪声模型，用于里程计因子
                    gtsam::Pose3 poseFrom = QT2gtsamPose(iterLIO->second);                                     // 将当前局部位姿转换为 GTSAM Pose3
                    gtsam::Pose3 poseTo = QT2gtsamPose(iterLIOnext->second);                                   // 将下一个局部位姿转换为 GTSAM Pose3
                    gtSAMgraphTM.add(BetweenFactor<Pose3>(i, i + 1, poseFrom.between(poseTo), odometryNoise)); // 添加里程计因子到因子图
                }

                double t = iterLIO->first;                // 获取当前局部位姿的时间戳
                iterGlobalLoc = globalLocPoseMap.find(t); // 查找相同时间戳的全局定位位姿

                // 如果找到对应的全局定位位姿
                if (iterGlobalLoc != globalLocPoseMap.end())
                {
                    gtsam::Pose3 poseGlobal = QT2gtsamPose(iterGlobalLoc->second); // 将全局定位位姿转换为 GTSAM Pose3
                    double tE = iterGlobalLoc->second[7];                          // 获取全局定位位姿的旋转精度
                    double tQ = iterGlobalLoc->second[8];                          // 获取全局定位位姿的平移精度
                    noiseModel::Diagonal::shared_ptr corrNoise = noiseModel::Diagonal::Variances(
                        (Vector(6) << tQ * tQ, tQ * tQ, tQ * tQ, tE * tE, tE * tE, tE * tE).finished()); // 创建一个对角噪声模型，用于全局定位因子
                    gtSAMgraphTM.add(PriorFactor<Pose3>(i, poseGlobal, corrNoise));                      // 添加全局定位因子到因子图
                    found++;                                                                             // 找到的全局定位位姿计数器加1

                    // 计算并备份全局到局部的变换矩阵
                    Eigen::Matrix4d local = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d global = Eigen::Matrix4d::Identity();
                    global.block<3, 3>(0, 0) = Eigen::Quaterniond(iterGlobalLoc->second[3], iterGlobalLoc->second[4],
                                                                  iterGlobalLoc->second[5], iterGlobalLoc->second[6])
                                                   .toRotationMatrix();
                    global.block<3, 1>(0, 3) = Eigen::Vector3d(iterGlobalLoc->second[0], iterGlobalLoc->second[1], iterGlobalLoc->second[2]);
                    local.block<3, 3>(0, 0) = Eigen::Quaterniond(iterLIO->second[3], iterLIO->second[4],
                                                                 iterLIO->second[5], iterLIO->second[6])
                                                  .toRotationMatrix();
                    local.block<3, 1>(0, 3) = Eigen::Vector3d(iterLIO->second[0], iterLIO->second[1], iterLIO->second[2]);
                    backupTgl = global * local.inverse(); // 计算备份的全局到局部的变换矩阵
                }

                gtsam::Pose3 poseGuess = QT2gtsamPose(iterIni->second); // 将初始全局位姿转换为 GTSAM Pose3
                initialEstimateTM.insert(i, poseGuess);                 // 插入初始估计值
            }

            if (found == 0)
            {                      // 如果没有找到任何全局定位位姿
                mPoseMap.unlock(); // 解锁位姿地图
                continue;          // 跳过本次循环
            }

            isamTM->update(gtSAMgraphTM, initialEstimateTM); // 使用因子图和初始估计更新 ISAM2
            isamTM->update();                                // 再次更新 ISAM2
            gtSAMgraphTM.resize(0);                          // 清空因子图
            initialEstimateTM.clear();                       // 清空初始估计值

            isamCurrentEstimateTM = isamTM->calculateEstimate(); // 计算当前的 ISAM2 估计

            iterIni = globalPoseMap.begin(); // 重置迭代器指向全局位姿地图的开头

            Eigen::Matrix4d start;                                     // 初始化起始变换矩阵
            Eigen::Matrix4d end;                                       // 初始化终止变换矩阵
            Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); // 初始化局部到机体的变换矩阵
            Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity(); // 初始化全局到机体的变换矩阵

            // 遍历局部位姿地图，更新全局位姿
            for (int i = 0; i < length; i++, iterIni++)
            {
                vector<double> globalPose(7, 0);                                  // 初始化全局位姿向量
                gtsamPose2Vector(isamCurrentEstimateTM.at<Pose3>(i), globalPose); // 将 GTSAM Pose3 转换为向量
                iterIni->second = globalPose;                                     // 更新全局位姿

                double t = iterIni->first; // 获取时间戳
                WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4],
                                                                   localPoseMap[t][5], localPoseMap[t][6])
                                                    .toRotationMatrix();
                WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
                WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4],
                                                                   globalPose[5], globalPose[6])
                                                    odometryNoise.toRotationMatrix();
                WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);

                WGlobal_T_WLocal = WGPS_T_body * WVIO_T_body.inverse(); // 计算并更新全局到局部的变换矩阵
            }
odometryNoise
            gtsamPose2Matrix4d(isamCurrentEstimateTM.at<Pose3>(length - 1), end); // 将最后一个 GTSAM Pose3 转换为矩阵
            gtsamPose2Matrix4d(isamCurrentEstimateTM.at<Pose3>(0), start);        // 将第一个 GTSAM Pose3 转换为矩阵
            lastP = Eigen::Vector3d(end(0, 3), end(1, 3), end(2, 3));             // 更新最后位置
            lastQ = Eigen::Quaterniond(end.block<3, 3>(0, 0));                    // 更新最后姿态

            // 计算起始和终止位置的位移
            double shift = sqrt((start(0, 3) - end(0, 3)) * (start(0, 3) - end(0, 3)) +
                                (start(1, 3) - end(1, 3)) * (start(1, 3) - end(1, 3)) +
                                (start(2, 3) - end(2, 3)) * (start(2, 3) - end(2, 3)));

            if (shift > 5)
            {                                 // 如果位移超过阈值
                resetOptimization(backupTgl); // 重置优化
            }

            mPoseMap.unlock(); // 解锁位姿地图
        }
        usleep(50); // 休眠50微秒
    }
}
