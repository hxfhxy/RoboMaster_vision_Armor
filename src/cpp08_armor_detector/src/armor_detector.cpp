    #include "cpp08_armor_detector/armor_detector.hpp"
    #include <opencv2/calib3d.hpp>
    #include <vector>

    // 构造函数：初始化相机内参、畸变系数、装甲板物理坐标、卡尔曼滤波参数
    ArmorDetector::ArmorDetector()
    {

        // 相机内参 (fx, fy, cx, cy)
        // camera_matrix 数据
        cameraMatrix = (cv::Mat_<double>(3, 3) << 
            1296.16167,          0.0,    643.60901,
                   0.0,   1296.23028,    509.49319,
                   0.0,          0.0,          1.0);

        // distortion_coefficients 数据
        // 畸变系数 (k1, k2, p1, p2, k3)
        distCoeffs = (cv::Mat_<double>(1, 5) << 
            -0.069992, 0.120254, -0.001661, -0.000788, 0.000000);

        // 装甲板物理尺寸 (mm)，并构建 3D 角点顺序
        float half_x = 135.0f / 2;
        float half_y = 55.0f / 2;
        objectPoints.clear(); // 先清空，防止残留
        objectPoints.push_back(cv::Point3f(-half_x,  half_y, 0.0f)); // 1. 左上 
        objectPoints.push_back(cv::Point3f(-half_x, -half_y, 0.0f)); // 2. 左下 
        objectPoints.push_back(cv::Point3f( half_x, -half_y, 0.0f)); // 3. 右下 
        objectPoints.push_back(cv::Point3f( half_x,  half_y, 0.0f)); // 4. 右上 

        
    }
        
    
