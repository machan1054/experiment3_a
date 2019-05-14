/*
  Deprojection

  Input:  color and depth image streams, mouse events
  Output: color image (window) and deprojection results (stdout)

  Compile me with:
  $ g++ -std=c++11 clickXYZ.cpp -lrealsense -lopencv_core -lopencv_highgui -o clickXYZ
  and run
  $ ./clickXYZ

  Written by tsakai@cis.nagasaki-u.ac.jp, 2017
*/

// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// obtained coordinates by on_mouse()
int imouse, jmouse;
bool clicked = false;

// camera parameter set
rs::intrinsics camera_param;
float scale;

// inline functions
inline float J_TO_U(int j){ return -(j - camera_param.ppx);}
inline float I_TO_V(int i){ return -(i - camera_param.ppy);}

// prototye declaration
void on_mouse( int, int, int, int, void *);
void Zuv_to_XY(float, float, float, float *, float *);


int main(void)
{
    // Create a context object. This object owns the handles to all connected realsense devices
    rs::context ctx;

    // Access the first available RealSense device
    rs::device * dev = ctx.get_device(0);

    // Configure BGR and depth streams to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);

    // Get depth camera parameters
    camera_param = dev->get_stream_intrinsics(rs::stream::color);
    scale = dev->get_depth_scale();

    // Create a window and watch every mouse event on it by on_mouse()
    const char *window_name = "color image";
    namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    setMouseCallback(window_name, on_mouse);

    // Start streaming
    dev->start();
    int i = 0;
    float POINT[3][3] = {};
    
    
    while(waitKey(1) != 'q'){
        dev->wait_for_frames();

        // Creating OpenCV Matrix from color and depth images
        Mat color(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color));
        Mat depth(Size(640, 480), CV_16UC1, (void*)dev->get_frame_data(rs::stream::depth_aligned_to_color));

        // Display the color image
        imshow(window_name, color);

        float u, v, X, Y, Z, XY[3]={}, XZ[3]={};
        if(clicked){                        // if clicked,
            clicked = false;                // set the flag to false

            u = J_TO_U(jmouse);             // set u from column index j
            v = I_TO_V(imouse);             // set v from row index i

            // get the depth value in meters
            Z = scale * depth.at<unsigned short>(imouse, jmouse);

            // if depth is available, compute and display the coordinates
            if(Z != 0.){
                Zuv_to_XY(Z, u, v, &X, &Y); // compute X and Y from Z, u and v
                printf("[%d] (%+6.1f,%+6.1f) => (%+6.3f, %+6.3f, %+6.3f)\n", i, u, v, X, Y, Z);
            }
            // display "NA" otherwise
            else{
                printf("(%+6.1f,%+6.1f), (NA, NA, NA)\n", u, v);
                i = 0;
           }
           POINT[i][0] = X;
           POINT[i][1] = Y;
           POINT[i][2] = Z;
           i++;
           if(i == 3) {
               //printf("(%+6.3f, %+6.3f, %+6.3f) (%+6.3f, %+6.3f, %+6.3f) (%+6.3f, %+6.3f, %+6.3f)\n", POINT[0][0], POINT[0][1], POINT[0][2], POINT[1][0], POINT[1][1], POINT[1][2], POINT[2][0], POINT[2][1], POINT[2][2]);
               XY[0] = POINT[1][0] - POINT[0][0];
               XY[1] = POINT[1][1] - POINT[0][1];
               XY[2] = POINT[1][2] - POINT[0][2];
               XZ[0] = POINT[2][0] - POINT[0][0];
               XZ[1] = POINT[2][1] - POINT[0][1];
               XZ[2] = POINT[2][2] - POINT[0][2];
               printf("n= (%+6.3f, %+6.3f, %+6.3f)\n", XY[1] * XZ[2] - XY[2] * XZ[1], XY[2] * XZ[0] - XY[0] * XZ[2], XY[0] * XZ[1] - XY[1] * XZ[0]);
               i = 0;
           }
               
       }
    }   // Repeat until 'q' is pressed

    return 0;
}


/*
    Mouse event callback function

    Input:  type of mouse event (event_type),
            mouse cursor position (i, j),
            mouse status (flag) unused,
            misc parameters (param) unused
    Output: none

    If left button is down, this function sets the flag and the global variables imouse and jmouse.
*/
void on_mouse(int event_type, int j, int i, int flags, void *param)
{
  if (event_type == CV_EVENT_LBUTTONDOWN){  // if the left mouse button is down
    clicked = true;                         // set the flag to true
    imouse = i;
    jmouse = j;
  }
}


/*
    Inverse perspective transformation
    computes X and Y from Z, u and v
*/
// COMPLETE THIS FUNCTION ON YOUR OWN. GOOD LUCK!!
void Zuv_to_XY(float Z, float u, float v, float *X, float *Y)
{
    *X = u * Z / camera_param.fx;// (available variables: Z, u, v, camera_param.fx)
    *Y = v * Z / camera_param.fy;// (available variables: Z, u, v, camera_param.fy)
    
}

