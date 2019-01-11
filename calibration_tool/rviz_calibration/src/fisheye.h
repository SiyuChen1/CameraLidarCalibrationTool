
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include "opencv2/core.hpp"
// #include "opencv2/features2d.hpp"
// #include "opencv2/core/affine.hpp"
// #include "opencv2/calib3d/calib3d_c.h"
// #include <opencv2/calib3d/calib3d.hpp>
// #include <cv_bridge/cv_bridge.h>  // opencv 常用头文件


// void cv::fisheye::undistortImage(InputArray distorted, OutputArray undistorted,
//         InputArray K, InputArray D, InputArray Knew, const Size& new_size)
// {
//     CV_INSTRUMENT_REGION();

//     Size size = !new_size.empty() ? new_size : distorted.size();

//     cv::Mat map1, map2;
//     fisheye::initUndistortRectifyMap(K, D, cv::Matx33d::eye(), Knew, size, CV_16SC2, map1, map2 );
//     cv::remap(distorted, undistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
// }