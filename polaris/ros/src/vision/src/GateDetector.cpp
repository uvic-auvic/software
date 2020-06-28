#include "GateDetector.hpp"
#include "Distance.hpp"
#include "opencv2/xfeatures2d.hpp"

#define GATE_WIDTH      120
#define GATE_HEIGHT     60

GateDetector::GateDetector(CameraInput& input, std::string cascade_name) : camera_input(input)
{
    if(cascade_name != "") cascade.load(cascade_name);
}

GateDetector::~GateDetector()
{
}

//Update to return Detector Result Struct
bool GateDetector::update()
{

    cv::Rect Gate;
    cv::Mat frame_gray;

    cv::cvtColor(camera_input.getFrameFront(), frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);
    // Detects objects and stores their location in object
    std::vector<cv::Rect> object;
    cascade.detectMultiScale(frame_gray, object, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    //if object is found in frame
    if(!object.empty()) {
        Gate = object[0];  
        distance_x_front = Distance::getDistanceX(Gate, GATE_WIDTH, camera_input.getFrameFront());
        distance_y_front = Distance::getDistanceY(Gate, GATE_HEIGHT, camera_input.getFrameFront());
        distance_z_front = Distance::getDistanceZ(Gate, GATE_WIDTH, 1);

    }
    return true;
}

cv::Point GateDetector::findGateDivider(cv::Mat frame){
    cv::Mat img1 = frame;
    cv::Mat img2 = cv::imread("img_matches/gate_divide.PNG");

    int minHessian = 400;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    detector->detectAndCompute( img1, cv::noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( img2, cv::noArray(), keypoints2, descriptors2 );
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<cv::DMatch> > knn_matches;
    matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> feature_points;

    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
            feature_points.push_back(keypoints2.at(knn_matches[i][0].queryIdx).pt);
        }
    }
    return avgPoint(feature_points);
}

cv::Point GateDetector::avgPoint(std::vector<cv::Point2f> list){
    int x = 0;
    int y = 0;
    int points = list.size();
    for (int i = 0; i < points; i++){
        x += list.at(i).x;
        y += list.at(i).y;
    }
    int x_avg = x/points;
    int y_avg = y/points;
    cv::Point avg = cv::Point(x_avg,y_avg);
    return avg;
}