#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"


#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
	vector<cv::Point3f> ConP3DVec;
	Mat ConP3D;
	//lidar coordinate
	//back
	ConP3DVec.push_back(Point3f(-8.07247, -6.33671, -1.22931));
	ConP3DVec.push_back(Point3f(-8.0435, -6.3712, 0.134053));

	ConP3DVec.push_back(Point3f(-10.6773, -3.83326, -0.453425));
	ConP3DVec.push_back(Point3f(-10.2893, -4.40891, 0.168226));
	ConP3DVec.push_back(Point3f(-9.93396, -4.94149, -0.456872));

	ConP3DVec.push_back(Point3f(-10.7904, -1.64425, -1.10991));
	ConP3DVec.push_back(Point3f(-10.7884, -1.67572, 0.255577));
	ConP3DVec.push_back(Point3f(-10.6818, -2.32596, -0.471647));

	ConP3DVec.push_back(Point3f(-9.13832, 1.033, -1.1236));
	ConP3DVec.push_back(Point3f(-8.88196, 1.65024, -0.460116));

	//back2
	ConP3DVec.push_back(Point3f(-5.00092, -5.66173, -1.20854));
	ConP3DVec.push_back(Point3f(-5.52814, -5.28628, -0.502107));
	ConP3DVec.push_back(Point3f(-4.47532, -6.04, -0.530151));

	ConP3DVec.push_back(Point3f(-7.12925, -5.38207, -1.19103));
	ConP3DVec.push_back(Point3f(-7.14966, -5.36414, 0.163357));

	ConP3DVec.push_back(Point3f(-6.0397, 2.84085, -1.12525));
	ConP3DVec.push_back(Point3f(-5.79931, 3.48509, -0.41946));
	ConP3DVec.push_back(Point3f(-6.03761, 2.86693, 0.270821));
	ConP3DVec.push_back(Point3f(-6.27056, 2.24296, -0.390746));

	ConP3DVec.push_back(Point3f(-3.89891, 3.54764, -1.15317));
	ConP3DVec.push_back(Point3f(-3.46377, 4.01888, -0.397696));
	ConP3DVec.push_back(Point3f(-3.88561, 3.55068, 0.253932));
	ConP3DVec.push_back(Point3f(-4.3496, 3.04688, -0.390803));

	//back3
	ConP3DVec.push_back(Point3f(-3.30901, -3.68092, -1.1832));
	ConP3DVec.push_back(Point3f(-3.80626, -3.28583, -0.457667));
	ConP3DVec.push_back(Point3f(-3.25849, -3.70549, 0.189849));
	ConP3DVec.push_back(Point3f(-2.74768, -4.11066, -0.490179));

	ConP3DVec.push_back(Point3f(-5.59935, -4.49824, -1.21131));
	ConP3DVec.push_back(Point3f(-6.1151, -4.00141, -0.514894));
	ConP3DVec.push_back(Point3f(-5.62852, -4.45948, 0.149269));
	ConP3DVec.push_back(Point3f(-5.19535, -4.87632, -0.379222));

	ConP3DVec.push_back(Point3f(-5.1354, 2.27932, -1.11751));
	ConP3DVec.push_back(Point3f(-4.845, 2.88341, -0.426362));
	ConP3DVec.push_back(Point3f(-5.11723, 2.28373, 0.247546));
	ConP3DVec.push_back(Point3f(-5.41625, 1.66017, -0.402942));

	ConP3DVec.push_back(Point3f(-3.01542, 3.03572, -1.12483));
	ConP3DVec.push_back(Point3f(-2.65881, 3.52822, -0.419088));
	ConP3DVec.push_back(Point3f(-2.9877, 2.99253, 0.262359));
	ConP3DVec.push_back(Point3f(-3.4023, 2.41144, -0.419023));
	Mat(ConP3DVec).convertTo(ConP3D, CV_32F);




	double camD[9] = { 657.338179917347, 0, 1003.929171908614,
	0, 658.0477619158104, 539.6019466669126,
	0, 0, 1 };
	double distCoeffD[4] = { -0.0196289, 0.00021278, -0.00128872, 1.08719e-005 };
	Mat camera_matrix = Mat(3, 3, CV_64FC1, camD);
	Mat distortion_coefficients = Mat(4, 1, CV_64FC1, distCoeffD);

	Mat rvec;
	Mat tvec;
	Mat R_matrix = Mat(3,3,CV_32FC1, Scalar::all(0));



	vector<cv::Point2f> ConP2DVec;
	Mat ConP2D;
	//back
	ConP2DVec.push_back(Point2f(473.81, 584.33));
	ConP2DVec.push_back(Point2f(470.925, 473.585));

	ConP2DVec.push_back(Point2f(713.645, 526.095));
	ConP2DVec.push_back(Point2f(670.51, 475.207));
	ConP2DVec.push_back(Point2f(622.507, 522.895));

	ConP2DVec.push_back(Point2f(856.59, 575.603));
	ConP2DVec.push_back(Point2f(855.941, 477.109));
	ConP2DVec.push_back(Point2f(808.073, 526.492));

	ConP2DVec.push_back(Point2f(1059.54, 597.683));
	ConP2DVec.push_back(Point2f(1121.9, 544.296));

	//back2
	ConP2DVec.push_back(Point2f(321.048, 626.286));
	ConP2DVec.push_back(Point2f(381.566, 544.349));
	ConP2DVec.push_back(Point2f(254.409, 544.801));

	ConP2DVec.push_back(Point2f(474.654, 599.921));
	ConP2DVec.push_back(Point2f(478.483, 468.812));

	ConP2DVec.push_back(Point2f(1341.21, 650.872));
	ConP2DVec.push_back(Point2f(1427.27, 564.872));
	ConP2DVec.push_back(Point2f(1349.06, 473.031));
	ConP2DVec.push_back(Point2f(1261.18, 560.004));

	ConP2DVec.push_back(Point2f(1592.62, 712.008));
	ConP2DVec.push_back(Point2f(1703.29, 600.692));
	ConP2DVec.push_back(Point2f(1619.26, 470.31));
	ConP2DVec.push_back(Point2f(1509.25, 582.934));

	//back3
	ConP2DVec.push_back(Point2f(265.749, 707.132));
	ConP2DVec.push_back(Point2f(351.58, 568.961));
	ConP2DVec.push_back(Point2f(249.237, 432.348));
	ConP2DVec.push_back(Point2f(157.331, 572.934));

	ConP2DVec.push_back(Point2f(436.1, 630.427));
	ConP2DVec.push_back(Point2f(507.691, 547.802));
	ConP2DVec.push_back(Point2f(438.501, 459.985));
	ConP2DVec.push_back(Point2f(363.471, 541.312));

	ConP2DVec.push_back(Point2f(1341.3, 684.784));
	ConP2DVec.push_back(Point2f(1449.65, 575.364));
	ConP2DVec.push_back(Point2f(1349.65, 461.368));
	ConP2DVec.push_back(Point2f(1240.36, 569.846));

	ConP2DVec.push_back(Point2f(1662.16, 771.196));
	ConP2DVec.push_back(Point2f(1787.29, 621.189));
	ConP2DVec.push_back(Point2f(1701.24, 452.81));
	ConP2DVec.push_back(Point2f(1572.16, 608.631));

	Mat(ConP2DVec).convertTo(ConP2D, CV_32F);

	Mat ConP2D_undistort;
	fisheye::undistortPoints(ConP2D, ConP2D_undistort, camera_matrix, distortion_coefficients, camera_matrix);

	Mat fake_distCoeffs = (Mat_<double>(4, 1) << 0, 0, 0, 0);
	solvePnP(ConP3D, ConP2D_undistort, camera_matrix, fake_distCoeffs, rvec, tvec, false, SOLVEPNP_EPNP);  //SOLVEPNP_ITERATIVE    SOLVEPNP_EPNP
	Rodrigues(rvec, R_matrix);

	//transpose of R_matrix
	Mat R_Transpose = Mat(3, 3, CV_32FC1, Scalar::all(0));
	transpose(R_matrix, R_Transpose);

	//change of tvec
	Mat tvec_modify = Mat(3, 1, CV_64FC1, Scalar::all(0));
	tvec_modify.at<double>(0, 0) = -1.0 * tvec.at<double>(2, 0);
	tvec_modify.at<double>(1, 0) = tvec.at<double>(0, 0);
	tvec_modify.at<double>(2, 0) = tvec.at<double>(1, 0);


	ofstream fout("PP3_59_back_outter_param.txt");
	fout << "Rotation matrix:" << endl;
	fout << R_matrix << endl;
	fout << "\nTranslation vector:" << endl;
	fout << tvec << endl;
	fout << endl;


    return 0;
}