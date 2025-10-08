// #include <cmath>
// #include <time.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include "rclcpp/rclcpp.hpp"  

// #include <nav_msgs/msg/odometry.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <std_msgs/msg/empty.hpp>
// #include "std_msgs/msg/int32.hpp"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <opencv2/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

// #include <pcl_conversions/pcl_conversions.h>  
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>

// // Constants
// const double PI = 3.1415926;
// const double rad2deg = 180 / PI;
// const double deg2rad = PI / 180;

// // Global variables 
// double initTime;
// double timeStart;
// double timeLasted;
// bool systemInited = false;

// double timeScanCur = 0;
// double timeScanLast = 0;

// int laserRotDir = 1;
// float laserAngleLast = 0;
// float laserAngleCur = 0;

// int skipFrameNum = 0;
// int skipFrameCount = 0;

// pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
// pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLessExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());

// sensor_msgs::msg::PointCloud2 laserCloudExtreCur2;
// sensor_msgs::msg::PointCloud2 laserCloudLast2;

// // ros::Publisher* pubLaserCloudExtreCurPointer;
// // ros::Publisher* pubLaserCloudLastPointer;
// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudExtreCurPointer;
// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudLastPointer;

// int cloudSortInd[800];
// int cloudNeighborPicked[800];

// // IMU buffers
// int imuPointerFront = 0;
// int imuPointerLast = -1;
// const int imuQueLength = 400;
// bool imuInited = false;

// float imuRollStart, imuPitchStart, imuYawStart;
// float imuRollCur, imuPitchCur, imuYawCur;

// float imuVeloXStart, imuVeloYStart, imuVeloZStart;
// float imuShiftXStart, imuShiftYStart, imuShiftZStart;
// float imuVeloXCur, imuVeloYCur, imuVeloZCur;
// float imuShiftXCur, imuShiftYCur, imuShiftZCur;

// float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
// float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

// double imuTime[imuQueLength] = {0};
// float imuRoll[imuQueLength] = {0};
// float imuPitch[imuQueLength] = {0};
// float imuYaw[imuQueLength] = {0};

// float imuAccX[imuQueLength] = {0};
// float imuAccY[imuQueLength] = {0};
// float imuAccZ[imuQueLength] = {0};

// float imuVeloX[imuQueLength] = {0};
// float imuVeloY[imuQueLength] = {0};
// float imuVeloZ[imuQueLength] = {0};

// float imuShiftX[imuQueLength] = {0};
// float imuShiftY[imuQueLength] = {0};
// float imuShiftZ[imuQueLength] = {0};

// bool newSweep = false;

// void ShiftToStartIMU()
// {
//   float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
//   float y1 = imuShiftFromStartYCur;
//   float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

//   float x2 = x1;
//   float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
//   float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

//   imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
//   imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
//   imuShiftFromStartZCur = z2;
// }

// void VeloToStartIMU()
// {
//   float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
//   float y1 = imuVeloFromStartYCur;
//   float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

//   float x2 = x1;
//   float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
//   float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

//   imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
//   imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
//   imuVeloFromStartZCur = z2;
// }

// void TransformToStartIMU(pcl::PointXYZHSV *p)
// {
//   float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
//   float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
//   float z1 = p->z;

//   float x2 = x1;
//   float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
//   float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

//   float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
//   float y3 = y2;
//   float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

//   float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
//   float y4 = y3;
//   float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

//   float x5 = x4;
//   float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
//   float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

//   p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
//   p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
//   p->z = z5 + imuShiftFromStartZCur;
// }

// void AccumulateIMUShift()
// {
//   float roll = imuRoll[imuPointerLast];
//   float pitch = imuPitch[imuPointerLast];
//   float yaw = imuYaw[imuPointerLast];
//   float accX = imuAccX[imuPointerLast];
//   float accY = imuAccY[imuPointerLast];
//   float accZ = imuAccZ[imuPointerLast];

//   float x1 = cos(roll) * accX - sin(roll) * accY;
//   float y1 = sin(roll) * accX + cos(roll) * accY;
//   float z1 = accZ;

//   float x2 = x1;
//   float y2 = cos(pitch) * y1 - sin(pitch) * z1;
//   float z2 = sin(pitch) * y1 + cos(pitch) * z1;

//   accX = cos(yaw) * x2 + sin(yaw) * z2;
//   accY = y2;
//   accZ = -sin(yaw) * x2 + cos(yaw) * z2;

//   int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
//   double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
//   if (timeDiff < 0.1) {

//     imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
//                               + accX * timeDiff * timeDiff / 2;
//     imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
//                               + accY * timeDiff * timeDiff / 2;
//     imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
//                               + accZ * timeDiff * timeDiff / 2;

//     imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
//     imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
//     imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
//   }
// }

// //void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
// void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudIn2)
// {
//   // std::cout << "[LOAM] Handling laser cloud" << std::endl;

//   if (!systemInited) {
//     initTime = laserCloudIn2->header.stamp.sec;
//     imuPointerFront = (imuPointerLast + 1) % imuQueLength;
//     systemInited = true;
//   }

//   timeScanLast = timeScanCur;
//   timeScanCur = laserCloudIn2->header.stamp.sec;
//   //timeScanCur = rclcpp::Time(laserCloudIn2->header.stamp).seconds();
//   timeLasted = timeScanCur - initTime;

//   pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
//   pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);
//   int cloudSize = laserCloudIn->points.size();

//   pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZHSV>(cloudSize, 1));
//   for (int i = 0; i < cloudSize; i++) {
//     laserCloud->points[i].x = laserCloudIn->points[i].x;
//     laserCloud->points[i].y = laserCloudIn->points[i].y;
//     laserCloud->points[i].z = laserCloudIn->points[i].z;
//     laserCloud->points[i].h = timeLasted;
//     laserCloud->points[i].v = 0;
//     cloudSortInd[i] = i;
//     cloudNeighborPicked[i] = 0;
//   }
// /*
//   bool newSweep = false;
//   laserAngleLast = laserAngleCur;
//   laserAngleCur = atan2(laserCloud->points[cloudSize - 1].x - laserCloud->points[0].x, 
//                         laserCloud->points[cloudSize - 1].y - laserCloud->points[0].y);

//   if (laserAngleLast > 0 && laserRotDir == 1 && laserAngleCur < laserAngleLast) {
//     laserRotDir = -1;
//     newSweep = true;
//   } else if (laserAngleLast < 0 && laserRotDir == -1 && laserAngleCur > laserAngleLast) {
//     laserRotDir = 1;
//     newSweep = true;
//   }
// */
//   if (newSweep) {
//     newSweep = false;
//     std::cout << "[LOAM] New Sweep" << std::endl;
//     timeStart = timeScanLast - initTime;

//     pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
//     imuTrans->points[0].x = imuPitchStart;
//     imuTrans->points[0].y = imuYawStart;
//     imuTrans->points[0].z = imuRollStart;
//     imuTrans->points[0].v = 10;

//     imuTrans->points[1].x = imuPitchCur;
//     imuTrans->points[1].y = imuYawCur;
//     imuTrans->points[1].z = imuRollCur;
//     imuTrans->points[1].v = 11;

//     imuTrans->points[2].x = imuShiftFromStartXCur;
//     imuTrans->points[2].y = imuShiftFromStartYCur;
//     imuTrans->points[2].z = imuShiftFromStartZCur;
//     imuTrans->points[2].v = 12;

//     imuTrans->points[3].x = imuVeloFromStartXCur;
//     imuTrans->points[3].y = imuVeloFromStartYCur;
//     imuTrans->points[3].z = imuVeloFromStartZCur;
//     imuTrans->points[3].v = 13;

//     *laserCloudExtreCur += *laserCloudLessExtreCur;
//     pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudLast2);
//     //laserCloudLast2.header.stamp = ros::Time().fromSec(timeScanLast);
//     laserCloudLast2.header.stamp = rclcpp::Time(timeScanLast);
//     laserCloudLast2.header.frame_id = "camera";

//     laserCloudExtreCur->clear();
//     laserCloudLessExtreCur->clear();
//     imuTrans->clear();

//     imuRollStart = imuRollCur;
//     imuPitchStart = imuPitchCur;
//     imuYawStart = imuYawCur;

//     imuVeloXStart = imuVeloXCur;
//     imuVeloYStart = imuVeloYCur;
//     imuVeloZStart = imuVeloZCur;

//     imuShiftXStart = imuShiftXCur;
//     imuShiftYStart = imuShiftYCur;
//     imuShiftZStart = imuShiftZCur;
//   }

//   imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;
//   imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
//   imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

//   if (imuPointerLast >= 0) {
//     while (imuPointerFront != imuPointerLast) {
//       if (timeScanCur < imuTime[imuPointerFront]) break;
//       imuPointerFront = (imuPointerFront + 1) % imuQueLength;
//     }

//     if (timeScanCur > imuTime[imuPointerFront]) {
//       imuRollCur = imuRoll[imuPointerFront];
//       imuPitchCur = imuPitch[imuPointerFront];
//       imuYawCur = imuYaw[imuPointerFront];
//       imuVeloXCur = imuVeloX[imuPointerFront];
//       imuVeloYCur = imuVeloY[imuPointerFront];
//       imuVeloZCur = imuVeloZ[imuPointerFront];
//       imuShiftXCur = imuShiftX[imuPointerFront];
//       imuShiftYCur = imuShiftY[imuPointerFront];
//       imuShiftZCur = imuShiftZ[imuPointerFront];
//     } else {
//       int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
//       float ratioFront = (timeScanCur - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
//       float ratioBack = 1.0f - ratioFront;

//       imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
//       imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;

//       if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI)
//         imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
//       else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI)
//         imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
//       else
//         imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;

//       imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
//       imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
//       imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

//       imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
//       imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
//       imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
//     }
//   }

//   if (!imuInited) {
//     imuRollStart = imuRollCur;
//     imuPitchStart = imuPitchCur;
//     imuYawStart = imuYawCur;

//     imuVeloXStart = imuVeloXCur;
//     imuVeloYStart = imuVeloYCur;
//     imuVeloZStart = imuVeloZCur;

//     imuShiftXStart = imuShiftXCur;
//     imuShiftYStart = imuShiftYCur;
//     imuShiftZStart = imuShiftZCur;

//     imuInited = true;
//   }

//   imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * (timeLasted - timeStart);
//   imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * (timeLasted - timeStart);
//   imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * (timeLasted - timeStart);

//   ShiftToStartIMU();

//   imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
//   imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
//   imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

//   VeloToStartIMU();

//   for (int i = 0; i < cloudSize; i++) {
//     TransformToStartIMU(&laserCloud->points[i]);
//   }

//   for (int i = 5; i < cloudSize - 5; i++) {
//     float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x 
//                 + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x 
//                 + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x 
//                 + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
//                 + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
//                 + laserCloud->points[i + 5].x;
//     float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y 
//                 + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y 
//                 + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y 
//                 + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
//                 + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
//                 + laserCloud->points[i + 5].y;
//     float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z 
//                 + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z 
//                 + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z 
//                 + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
//                 + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
//                 + laserCloud->points[i + 5].z;
    
//     laserCloud->points[i].s = diffX * diffX + diffY * diffY + diffZ * diffZ;
//   }
  
//   for (int i = 5; i < cloudSize - 6; i++) {
//     float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
//     float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
//     float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
//     float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

//     if (diff > 0.05) {

//       float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
//                      laserCloud->points[i].y * laserCloud->points[i].y +
//                      laserCloud->points[i].z * laserCloud->points[i].z);

//       float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
//                      laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
//                      laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

//       if (depth1 > depth2) {
//         diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
//         diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
//         diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

//         if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
//           cloudNeighborPicked[i - 5] = 1;
//           cloudNeighborPicked[i - 4] = 1;
//           cloudNeighborPicked[i - 3] = 1;
//           cloudNeighborPicked[i - 2] = 1;
//           cloudNeighborPicked[i - 1] = 1;
//           cloudNeighborPicked[i] = 1;
//         }
//       } else {
//         diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
//         diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
//         diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

//         if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
//           cloudNeighborPicked[i + 1] = 1;
//           cloudNeighborPicked[i + 2] = 1;
//           cloudNeighborPicked[i + 3] = 1;
//           cloudNeighborPicked[i + 4] = 1;
//           cloudNeighborPicked[i + 5] = 1;
//           cloudNeighborPicked[i + 6] = 1;
//         }
//       }
//     }

//     float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
//     float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
//     float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
//     float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

//     float dis = laserCloud->points[i].x * laserCloud->points[i].x
//               + laserCloud->points[i].y * laserCloud->points[i].y
//               + laserCloud->points[i].z * laserCloud->points[i].z;

//     if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis) {
//       cloudNeighborPicked[i] = 1;
//     }
//   }

//   pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
//   pcl::PointCloud<pcl::PointXYZHSV>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZHSV>());
//   pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZHSV>());
//   pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZHSV>());

//   int startPoints[4] = {5, 6 + int((cloudSize - 10) / 4.0), 
//                         6 + int((cloudSize - 10) / 2.0), 6 + int(3 * (cloudSize - 10) / 4.0)};
//   int endPoints[4] = {5 + int((cloudSize - 10) / 4.0), 5 + int((cloudSize - 10) / 2.0), 
//                       5 + int(3 * (cloudSize - 10) / 4.0), cloudSize - 6};

//   for (int i = 0; i < 4; i++) {
//     int sp = startPoints[i];
//     int ep = endPoints[i];

//     for (int j = sp + 1; j <= ep; j++) {
//       for (int k = j; k >= sp + 1; k--) {
//         if (laserCloud->points[cloudSortInd[k]].s < laserCloud->points[cloudSortInd[k - 1]].s) {
//           int temp = cloudSortInd[k - 1];
//           cloudSortInd[k - 1] = cloudSortInd[k];
//           cloudSortInd[k] = temp;
//         }
//       }
//     }

//     int largestPickedNum = 0;
//     for (int j = ep; j >= sp; j--) {
//       if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
//           laserCloud->points[cloudSortInd[j]].s > 0.1 &&
//           (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
//           fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
//           fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
//           fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
//           fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
//           fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {
        
//         largestPickedNum++;
//         if (largestPickedNum <= 2) {
//           laserCloud->points[cloudSortInd[j]].v = 2;
//           cornerPointsSharp->push_back(laserCloud->points[cloudSortInd[j]]);
//         } else if (largestPickedNum <= 20) {
//           laserCloud->points[cloudSortInd[j]].v = 1;
//           cornerPointsLessSharp->push_back(laserCloud->points[cloudSortInd[j]]);
//         } else {
//           break;
//         }

//         cloudNeighborPicked[cloudSortInd[j]] = 1;
//         for (int k = 1; k <= 5; k++) {
//           float diffX = laserCloud->points[cloudSortInd[j] + k].x 
//                       - laserCloud->points[cloudSortInd[j] + k - 1].x;
//           float diffY = laserCloud->points[cloudSortInd[j] + k].y 
//                       - laserCloud->points[cloudSortInd[j] + k - 1].y;
//           float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
//                       - laserCloud->points[cloudSortInd[j] + k - 1].z;
//           if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
//             break;
//           }

//           cloudNeighborPicked[cloudSortInd[j] + k] = 1;
//         }
//         for (int k = -1; k >= -5; k--) {
//           float diffX = laserCloud->points[cloudSortInd[j] + k].x 
//                       - laserCloud->points[cloudSortInd[j] + k + 1].x;
//           float diffY = laserCloud->points[cloudSortInd[j] + k].y 
//                       - laserCloud->points[cloudSortInd[j] + k + 1].y;
//           float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
//                       - laserCloud->points[cloudSortInd[j] + k + 1].z;
//           if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
//             break;
//           }

//           cloudNeighborPicked[cloudSortInd[j] + k] = 1;
//         }
//       }
//     }

//     int smallestPickedNum = 0;
//     for (int j = sp; j <= ep; j++) {
//       if (cloudNeighborPicked[cloudSortInd[j]] == 0 &&
//           laserCloud->points[cloudSortInd[j]].s < 0.1 &&
//           (fabs(laserCloud->points[cloudSortInd[j]].x) > 0.3 || 
//           fabs(laserCloud->points[cloudSortInd[j]].y) > 0.3 || 
//           fabs(laserCloud->points[cloudSortInd[j]].z) > 0.3) && 
//           fabs(laserCloud->points[cloudSortInd[j]].x) < 30 && 
//           fabs(laserCloud->points[cloudSortInd[j]].y) < 30 && 
//           fabs(laserCloud->points[cloudSortInd[j]].z) < 30) {

//         laserCloud->points[cloudSortInd[j]].v = -1;
//         surfPointsFlat->push_back(laserCloud->points[cloudSortInd[j]]);

//         smallestPickedNum++;
//         if (smallestPickedNum >= 4) {
//           break;
//         }

//         cloudNeighborPicked[cloudSortInd[j]] = 1;
//         for (int k = 1; k <= 5; k++) {
//           float diffX = laserCloud->points[cloudSortInd[j] + k].x 
//                       - laserCloud->points[cloudSortInd[j] + k - 1].x;
//           float diffY = laserCloud->points[cloudSortInd[j] + k].y 
//                       - laserCloud->points[cloudSortInd[j] + k - 1].y;
//           float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
//                       - laserCloud->points[cloudSortInd[j] + k - 1].z;
//           if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
//             break;
//           }

//           cloudNeighborPicked[cloudSortInd[j] + k] = 1;
//         }
//         for (int k = -1; k >= -5; k--) {
//           float diffX = laserCloud->points[cloudSortInd[j] + k].x 
//                       - laserCloud->points[cloudSortInd[j] + k + 1].x;
//           float diffY = laserCloud->points[cloudSortInd[j] + k].y 
//                       - laserCloud->points[cloudSortInd[j] + k + 1].y;
//           float diffZ = laserCloud->points[cloudSortInd[j] + k].z 
//                       - laserCloud->points[cloudSortInd[j] + k + 1].z;
//           if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
//             break;
//           }

//           cloudNeighborPicked[cloudSortInd[j] + k] = 1;
//         }
//       }
//     }
//   }
// for (int i = 0; i < cloudSize; i++) {
//   if (laserCloud->points[i].v == 0) {
//     surfPointsLessFlat->push_back(laserCloud->points[i]);
//   }
// }

// pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatDS(new pcl::PointCloud<pcl::PointXYZHSV>());
// pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
// downSizeFilter.setInputCloud(surfPointsLessFlat);
// downSizeFilter.setLeafSize(0.1f, 0.1f, 0.1f);
// downSizeFilter.filter(*surfPointsLessFlatDS);

// // Merge feature points
// *laserCloudExtreCur += *cornerPointsSharp;
// *laserCloudExtreCur += *surfPointsFlat;
// *laserCloudLessExtreCur += *cornerPointsLessSharp;
// *laserCloudLessExtreCur += *surfPointsLessFlatDS;

// // Clear temporary clouds
// laserCloudIn->clear();
// laserCloud->clear();
// cornerPointsSharp->clear();
// cornerPointsLessSharp->clear();
// surfPointsFlat->clear();
// surfPointsLessFlat->clear();
// surfPointsLessFlatDS->clear();

// if (skipFrameCount >= skipFrameNum) {
//   skipFrameCount = 0;

//   pcl::PointCloud<pcl::PointXYZHSV>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZHSV>(4, 1));
//   imuTrans->points[0].x = imuPitchStart;
//   imuTrans->points[0].y = imuYawStart;
//   imuTrans->points[0].z = imuRollStart;
//   imuTrans->points[0].v = 10;

//   imuTrans->points[1].x = imuPitchCur;
//   imuTrans->points[1].y = imuYawCur;
//   imuTrans->points[1].z = imuRollCur;
//   imuTrans->points[1].v = 11;

//   imuTrans->points[2].x = imuShiftFromStartXCur;
//   imuTrans->points[2].y = imuShiftFromStartYCur;
//   imuTrans->points[2].z = imuShiftFromStartZCur;
//   imuTrans->points[2].v = 12;

//   imuTrans->points[3].x = imuVeloFromStartXCur;
//   imuTrans->points[3].y = imuVeloFromStartYCur;
//   imuTrans->points[3].z = imuVeloFromStartZCur;
//   imuTrans->points[3].v = 13;

//   sensor_msgs::msg::PointCloud2 laserCloudExtreCur2;
//   pcl::toROSMsg(*laserCloudExtreCur + *imuTrans, laserCloudExtreCur2);
//   //laserCloudExtreCur2.header.stamp = ros::Time().fromSec(timeScanCur);
//   laserCloudExtreCur2.header.stamp = rclcpp::Time(timeScanCur);
//   laserCloudExtreCur2.header.frame_id = "camera";

//   pubLaserCloudExtreCurPointer->publish(laserCloudExtreCur2);
//   imuTrans->clear();

//   pubLaserCloudLastPointer->publish(laserCloudLast2);
//   // std::cout << "[LOAM] publish laser cloud last" << std::endl;
//   // ROS_INFO("publish laser cloud last (%d, %d)", laserCloudLast2.width, laserCloudExtreCur2.width);
//   // std::cout << "publish laser cloud last (" << laserCloudLast2.width << ", " << laserCloudExtreCur2.width << ")" << std::endl;

//   }//ROS_INFO ("%d %d", laserCloudLast2.width, laserCloudExtreCur2.width);

//   skipFrameCount++;
    
// }

// //void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
// void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuIn)
// {
//   double roll, pitch, yaw;

//   // tf::Quaternion orientation;
//   // tf::quaternionMsgToTF(imuIn->orientation, orientation);
//   // tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
//   tf2::Quaternion orientation;
//   tf2::fromMsg(imuIn->orientation, orientation);
//   tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

//   float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
//   float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
//   float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

//   imuPointerLast = (imuPointerLast + 1) % imuQueLength;

//   imuTime[imuPointerLast] = imuIn->header.stamp.sec - 0.1068;
//   //imuTime[imuPointerLast] = rclcpp::Time(imuIn->header.stamp).seconds() - 0.1068;
//   imuRoll[imuPointerLast] = roll;
//   imuPitch[imuPointerLast] = pitch;
//   imuYaw[imuPointerLast] = yaw;
//   imuAccX[imuPointerLast] = accX;
//   imuAccY[imuPointerLast] = accY;
//   imuAccZ[imuPointerLast] = accZ;

//   // std::cout << "IMU handler (" << imuRoll[imuPointerLast] << ", " << imuPitch[imuPointerLast] << ", " << imuYaw[imuPointerLast] << ")" << std::endl;
//   AccumulateIMUShift();
// }

// // void sweepHandler(const std_msgs::msg::Empty::SharedPtr /*unused*/)
// // {
// //   newSweep = true;
// // }

// int  lastServoAngle    = -1;   // uninitialized
// int  currentServoAngle = -1;
// int  lastDirection     = -1;   // -1 = unknown, 0 = decreasing, 1 = increasing

// void servoSweepHandler(const std_msgs::msg::Int32::SharedPtr servoAngle)
// {
//   // std::cout << "[LOAM] Servo sweep: " << servoAngle->data << std::endl;
//   lastServoAngle = currentServoAngle;
//   currentServoAngle = servoAngle->data;
//   if (lastServoAngle < 0 || lastServoAngle == currentServoAngle) return;

//   int currentDirection = currentServoAngle > lastServoAngle ? 1 : 0;

//   // if servo starts moving in the opposite direction, a sweep just happened
//   if (currentDirection != lastDirection) {
//     std::cout << "[LOAM] Servo new sweep: " << servoAngle->data << std::endl;
//     newSweep = true;
//   }
  
//   lastDirection = currentDirection;
// }

// int main(int argc, char** argv)
// {
//   // ros::init(argc, argv, "scanRegistration");
//   // ros::NodeHandle nh;
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("scan_registration");
//   // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> 
//   //                                 ("/sync_scan_cloud_filtered", 2, laserCloudHandler);
//   auto subLaserCloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
//     "/point_cloud", 2, laserCloudHandler);

//   // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//   // ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> 
//   //                          ("/imu/daata", 5, imuHandler);
//   // auto subImu = node->create_subscription<sensor_msgs::msg::Imu>(
//   //   "/imu/data", 5, imuHandler);
//   // WE IGNORE IMU SINCE IT DOESN'T WORK, COMMENT THE NEXT LINE AND UNCOMMENT THE PREVIOUS ONE IF YOU WANT TO USE IT ANYWAY
//   auto subImu = node->create_subscription<sensor_msgs::msg::Imu>(
//     "/nothing_is_published_to_this_topic", 5, imuHandler);
//   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//   // // ros::Subscriber subSweep = nh.subscribe<std_msgs::Empty>
//   // //                            ("/new_sweep", 5, sweepHandler);
//   // auto subSweep = node->create_subscription<std_msgs::msg::Empty>(
//   //   "/new_sweep", 5, sweepHandler);

//   // we use directly the servo angle for determining the sweep
//   auto subServoAngle = node->create_subscription<std_msgs::msg::Int32>(
//     "/lidar_servo_angle", 5, servoSweepHandler);

//   // ros::Publisher pubLaserCloudExtreCur = nh.advertise<sensor_msgs::PointCloud2> 
//   //                                        ("/laser_cloud_extre_cur", 2);
//   auto pubLaserCloudExtreCur = node->create_publisher<sensor_msgs::msg::PointCloud2>(
//     "/laser_cloud_extre_cur", 2);

//   // ros::Publisher pubLaserCloudLast = nh.advertise<sensor_msgs::PointCloud2> 
//   //                                    ("/laser_cloud_last", 2);
//   auto pubLaserCloudLast = node->create_publisher<sensor_msgs::msg::PointCloud2>(
//     "/laser_cloud_last", 2);

//   // pubLaserCloudExtreCurPointer = &pubLaserCloudExtreCur;
//   // pubLaserCloudLastPointer = &pubLaserCloudLast;
//   pubLaserCloudExtreCurPointer = pubLaserCloudExtreCur;
//   pubLaserCloudLastPointer = pubLaserCloudLast;

//   //ros::spin();
//   rclcpp::spin(node);

//   return 0;
// }


#include <cmath>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"  
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/int32.hpp"

#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using std::placeholders::_1;

class SweepMergerNode : public rclcpp::Node
{
public:
  SweepMergerNode()
  : Node("sweep_merger_node"),
    last_servo_angle_(-1),
    current_servo_angle_(-1),
    last_direction_(-1),
    new_sweep_(false)
  {
    // Subscriber: each ring
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/point_cloud", rclcpp::QoS(2),
      std::bind(&SweepMergerNode::laserCloudHandler, this, _1));

    // Subscriber: servo angle to detect new sweeps
    sub_servo_ = create_subscription<std_msgs::msg::Int32>(
      "/lidar_servo_angle", rclcpp::QoS(5),
      std::bind(&SweepMergerNode::servoSweepHandler, this, _1));

    // Publisher: merged full scan
    pub_full_scan_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/full_scan", rclcpp::QoS(1));

    // Initialize accumulator
    accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

private:
  // Called on each incoming ring
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 1) Convert to PCL
    pcl::PointCloud<pcl::PointXYZ> ring;
    pcl::fromROSMsg(*msg, ring);

    // 2) (Optional) Transform ring from sensor frame → base_link here via TF2
    //    … omitted for brevity …

    // 3) Accumulate
    *accumulated_cloud_ += ring;

    // 4) If we just finished a sweep, publish and clear
    if (new_sweep_) {
      publishFullSweep(msg->header.stamp);
      new_sweep_ = false;
    }
  }

  // Servo updates → detect end of sweep on direction change
  void servoSweepHandler(const std_msgs::msg::Int32::SharedPtr angle)
  {
    last_servo_angle_    = current_servo_angle_;
    current_servo_angle_ = angle->data;

    // skip first or duplicate
    if (last_servo_angle_ < 0 || last_servo_angle_ == current_servo_angle_) {
      return;
    }

    int dir = (current_servo_angle_ > last_servo_angle_) ? 1 : 0;
    if (dir != last_direction_) {
      RCLCPP_INFO(get_logger(), "Detected end of sweep at angle %d", current_servo_angle_);
      new_sweep_ = true;
    }
    last_direction_ = dir;
  }

  // Publish the accumulated point cloud as one full scan
  void publishFullSweep(const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*accumulated_cloud_, out);

    out.header.stamp = stamp;
    out.header.frame_id = "base_link";
    pub_full_scan_->publish(out);

    RCLCPP_INFO(get_logger(),
                "Published full sweep with %zu points",
                accumulated_cloud_->size());

    accumulated_cloud_->clear();
  }

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   sub_servo_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_full_scan_;

  // Accumulator
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;

  // Sweep detection state
  int last_servo_angle_;
  int current_servo_angle_;
  int last_direction_;
  bool new_sweep_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SweepMergerNode>());
  rclcpp::shutdown();
  return 0;
}
