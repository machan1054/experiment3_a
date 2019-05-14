// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    // Create a context object. This object owns the handles to all connected realsense devices
    rs::context ctx;

    // Access the first available RealSense device
    rs::device * dev = ctx.get_device(0);

    // Configure Infrared stream to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);
    
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
    float scale = dev->get_depth_scale();

    // Start streaming
    dev->start();
    

    // Camera warmup - Dropped several first frames to let auto-exposure stabilize
    while(waitKey(1) != 'q') {
       dev->wait_for_frames();

        // Creating OpenCV Matrix from a color image
        Mat color(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color), Mat::AUTO_STEP);
        
        Mat depth(Size(640, 480), CV_16UC1, (void*)dev->get_frame_data(rs::stream::depth_aligned_to_color)); //(*1)
        
        Mat gray_depth;
        depth.convertTo(gray_depth, CV_8UC1, 255*scale, 0);

        // Display in a GUI
        imshow("Display Image", color);
        imshow("Depth Image", gray_depth);
    }
    

    return 0;
}
