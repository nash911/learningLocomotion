//#include "cv.h"
//#include "highgui.h"

#include <vector>

#include "/usr/local/include/opencv/cv.h"
#include "/usr/local/include/opencv/highgui.h"

#include "layer_utils.h"
#include "disjoint_sets2.h"


class VisualTracker
{
private:
    static const int DEFAULT_WIDTH = 640, DEFAULT_HEIGHT = 480;

    cv::VideoCapture camera; // From camera or video

public:
    /*!
     * Contructor.
     * \param w desired width of the camera frame (pixels)
     * \param h desired height of the camera frame (pixels)
     */
    VisualTracker(int w = DEFAULT_WIDTH, int h = DEFAULT_HEIGHT);
    ~VisualTracker() {
        camera.release();
    }

    //std::vector<int> get_robot_3D_position(void);
    //void get_robot_3D_position(void);
    //! return position of the robot in meters
    bool process_image(const cv::Mat&, int &robot_x_pixels, int &robot_y_pixels);
    void get_robot_3D_position(double &x, double &y, double& z);
    bool get_robot_3D_position_rectfied(const std::string&, double&, double&, double&);
    double euclidean_distance(cv::Point, cv::Point);
};
