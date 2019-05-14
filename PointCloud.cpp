/*
  Point cloud

  Input:  color and depth images
  Output: color image (color00pc.png),
          depth image (depth00pc.png),
          point cloud data (xyzrgb00pc.ply)

  Compile me with:
  $ g++ -std=c++11 PointCloud.cpp -lrealsense -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -o PC
  and run
  $ ./PC

  Written by tsakai@cis.nagasaki-u.ac.jp, 2017
*/

// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// camera parameter set
rs::intrinsics camera_param;
float scale;

// inline functions
inline float J_TO_U(int j){ return -(j - camera_param.ppx);}
inline float I_TO_V(int i){ return -(i - camera_param.ppy);}

// prototye declaration
void Zuv_to_XY(float, float, float, float *, float *);
void ComputePointCloud(const Mat&, Mat&);
unsigned int SavePly(const char *, const Mat&, const Mat&);


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

    // Start streaming
    dev->start();

    // Camera warmup - Dropped several first frames to let auto-exposure stabilize
    for(int i = 0; i < 30; i++) dev->wait_for_frames();

    // Get color and depth images as OpenCV Matrices
    Mat color(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color));
    Mat depth(Size(640, 480), CV_16UC1, (void*)dev->get_frame_data(rs::stream::depth_aligned_to_color));
    Mat gray_depth;
    depth.convertTo(gray_depth, CV_8UC1, 255*scale, 0);
    //applyColorMap(gray_depth, gray_depth, COLORMAP_OCEAN);

    // Save images
    imwrite("color00pc.png", color);
    imwrite("depth00pc.png", gray_depth);

    // Compute the point cloud from the depth
    Mat pointCloud;
    ComputePointCloud(depth, pointCloud);

    // Save the point cloud with color in ply format
    SavePly("xyzrgb00pc.ply", pointCloud, color);

    return 0;
}


/*
    Compute point cloud from depth image

    Input:  depth image (depth),
    Output: matrix (PointCloud) of triplets (X,Y,Z)
*/
void ComputePointCloud(const Mat& Depth, Mat& PointCloud)
{
    // Define a OpenCV matrix PointCloud of the same size as Depth
    // with all entries (X,Y,Z) = (0.0,0.0,0.0)
    PointCloud = Mat(Depth.rows, Depth.cols, CV_32FC3, Scalar::all(0));

    float u, v, X, Y, Z;

    // for all depth pixels
    for(int i = 0; i != Depth.rows; i ++){
        for(int j = 0; j != Depth.cols; j ++){

            // get the depth value in meters
            Z = scale * Depth.at<unsigned short>(i, j);

            if( Z != 0. ){      // if depth is available
                u = J_TO_U(j);  // compute u from column index j
                v = I_TO_V(i);  // compute v from row index i
                Zuv_to_XY(Z, u, v, &X, &Y); // compute X and Y from Z, u and v

                // ij-th entry of the matrix PointCloud is a triplet (X, Y, Z)
                PointCloud.at<Point3f>(i, j) = Point3f(X, Y, Z);
            }
        }
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

/*
    Save point cloud with color in ply format, which can be visualized by MeshLab

    Input:  point cloud (xyz),
            color image (bgr)
    Output: ply file with given filename
            # of points (num)
*/
unsigned int SavePly(const char *filename, const Mat& xyz, const Mat& bgr)
{

    // Open an output file
    FILE *fp;
    fp = fopen(filename, "w");
    if( fp == NULL ) return 0;

    // count available points
    unsigned int num = 0;
    Point3f point;
    for(int i = 0; i != xyz.rows; i ++){
        for(int j = 0; j != xyz.cols; j ++){
            point = xyz.at<Point3f>(i, j);
            if(point.z != 0.0f) num ++;
        }
    }

    // Write header information
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", num);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    // Write available points and its colors
    Vec3b color3b;
    for(int i = 0; i != xyz.rows; i ++){
        for(int j = 0; j != xyz.cols; j ++){
            point = xyz.at<Point3f>(i, j);
            if(point.z != 0.0f){
                color3b = bgr.at<Vec3b>(i, j);
                // x y z r g b
                fprintf(fp, "%7.4f %7.4f %7.4f %3u %3u %3u\n", point.x, point.y, point.z,
                    unsigned(color3b.val[2]), unsigned(color3b.val[1]), unsigned(color3b.val[0]));
            }
        }
    }

    // Close the file
    fclose(fp);

    return num;
}

