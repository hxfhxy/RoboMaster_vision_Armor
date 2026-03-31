    #include "cpp08_armor_detector/armor_detector.hpp"

    // 构造函数：初始化相机内参、畸变系数、装甲板物理坐标、卡尔曼滤波参数
    ArmorDetector::ArmorDetector()
    {
        // 相机内参 (fx, fy, cx, cy)
        cameraMatrix = (cv::Mat_<double>(3, 3) << 2374.54248, 0, 698.85288,
                                                0, 2377.53648, 520.8649,
                                                0, 0, 1);

        // 相机畸变系数 (k1 k2 p1 p2 k3)
        distCoeffs = (cv::Mat_<double>(1, 5) << -0.059743, 0.355479, -0.000625, 0.001595, 0);

        // 装甲板物理尺寸 (mm)，并构建 3D 角点顺序
        float half_x = 135.0f / 2;
        float half_y = 55.0f / 2;
        objectPoints.clear(); // 先清空，防止残留
        objectPoints.push_back(cv::Point3f(-half_x,  half_y, 0.0f)); // 1. 左上 (x负, y正)
        objectPoints.push_back(cv::Point3f(-half_x, -half_y, 0.0f)); // 2. 左下 (x负, y负)
        objectPoints.push_back(cv::Point3f( half_x, -half_y, 0.0f)); // 3. 右下 (x正, y负)
        objectPoints.push_back(cv::Point3f( half_x,  half_y, 0.0f)); // 4. 右上 (x正, y正)

        // 卡尔曼滤波器（yaw 角度+角速度），初始化状态转移矩阵 F 和观测矩阵 H
        float dt = 1.0f / 50.0f;
        kf.F = (cv::Mat_<float>(2, 2) << 1, dt,
                                        0, 1);
        kf.H = (cv::Mat_<float>(1, 2) << 1, 0);
    }