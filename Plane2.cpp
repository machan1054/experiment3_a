/*
  Plane detection

  Input:  color and depth images
  Output: color and depth image (window)

  Compile me with:
  $ g++ -std=c++11 Plane.cpp -lrealsense -lopencv_core -lopencv_imgproc -lopencv_highgui -o Plane
  and run
  $ ./Plane

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

// (j, i) of three points, left, right and bottom, that determine the plane
Point posL(180, 380), posR(460, 380), posB(320, 440);

// Font faces
int face[] = {FONT_HERSHEY_SIMPLEX, FONT_HERSHEY_PLAIN, FONT_HERSHEY_DUPLEX, FONT_HERSHEY_COMPLEX, FONT_HERSHEY_TRIPLEX, FONT_HERSHEY_COMPLEX_SMALL, FONT_HERSHEY_SCRIPT_SIMPLEX, FONT_HERSHEY_SCRIPT_COMPLEX, FONT_ITALIC};

// prototye declaration
int ComputePlane(const Mat& Depth, Mat&, Mat&);
int ColorPlane(const Mat&, const Mat&, const Mat&, const float&, Mat&, Mat&, Point&, float&, bool maskc=false);
void Zuv_to_XY(float, float, float, float *, float *);

int main(void)
{
    // コンテキストオブジェクトを作成します。このオブジェクトは、接続されているすべてのRealsenseデバイスへのハンドルを所有しています。
    rs::context ctx;

    // 最初に利用可能なRealSenseデバイスにアクセスする
    rs::device * dev = ctx.get_device(0);

    // 毎秒30フレームのVGA解像度で実行するようにBGRと深度ストリームを設定する
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);

    // 深度カメラパラメータを取得する
    camera_param = dev->get_stream_intrinsics(rs::stream::color);
    scale = dev->get_depth_scale();

    // Parameters
    float eps = 0.10;
    Mat vecN, vecP, vecC; 
    Point posC;
    float height;

    // Start streaming
    dev->start();

    // カメラウォームアップ - 自動露出を安定させるためにいくつかの最初のフレームを落としました
    //for(int i = 0; i < 30; i++)

    char key;
    bool mask_nodepth = false;
    bool show_dist = false;
    bool show_3points = true;
    while((key = waitKey(1)) != 'q'){
        dev->wait_for_frames();

        // カラー画像と深度画像からOpenCV Matrixを作成する
        Mat color(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color));
        Mat depth(Size(640, 480), CV_16UC1, (void*)dev->get_frame_data(rs::stream::depth_aligned_to_color));

        // 平面パラメータを推定
        if(ComputePlane(depth, vecN, vecP) == 0){

            if(key == 'd') show_dist = !show_dist;

            if(key == 'm') mask_nodepth = !mask_nodepth;
            // 平面を検出して色付けする
            ColorPlane(vecN, vecP, depth, eps, color, vecC, posC, height, mask_nodepth);

            // Write the height by
            // putText(image, text, position(bottom left),
            //              font face, text scale, text color, thickness, type);
            if(show_dist){
                char cnumUser[13];
                sprintf(cnumUser, "%d [mm]", (int) (height*1000));
                putText(color, cnumUser, Point(50,50),
                                face[0], 1.2, Scalar(0,0,200), 2, CV_AA);

                circle(color, posC, 4, Scalar(0,255,0), -1);
           }
        }

        // 平面を決定する3点を描画する
        if(show_3points){
            circle(color, posL, 2, Scalar(255,128,128), -1);
            circle(color, posR, 2, Scalar(255,128,128), -1);
            circle(color, posB, 2, Scalar(255,128,128), -1);
        }

        Mat gray_depth;
        depth.convertTo(gray_depth, CV_8UC1, 255*scale, 0);
        applyColorMap(gray_depth, gray_depth, COLORMAP_OCEAN);

        // Display in a GUI
        imshow("BGR Image", color);
        imshow("Depth Image", gray_depth);
    }

    return 0;
}


int ComputePlane(const Mat& Depth, Mat& vecNormal, Mat& vecPoint)
{
    Point3f Pl, Pr, Pb;
    int i, j;

    j = posL.x; i = posL.y;
    Pl.z = scale * Depth.at<unsigned short>(i, j);
    if( Pl.z == 0. ) return -1;
    Zuv_to_XY(Pl.z, J_TO_U(j), I_TO_V(i), &(Pl.x), &(Pl.y));

    j = posR.x; i = posR.y;
    Pr.z = scale * Depth.at<unsigned short>(i, j);
    if( Pr.z == 0. ) return -1;
    Zuv_to_XY(Pr.z, J_TO_U(j), I_TO_V(i), &(Pr.x), &(Pr.y));

    j = posB.x; i = posB.y;
    Pb.z = scale * Depth.at<unsigned short>(i, j);
    if( Pb.z == 0. ) return -1;
    Zuv_to_XY(Pb.z, J_TO_U(j), I_TO_V(i), &(Pb.x), &(Pb.y));

    // a point on the plane
    vecPoint = Mat(Pb);

    Mat vecA, vecB;
    vecA = Mat(Pl) - Mat(Pb);
    vecB = Mat(Pr) - Mat(Pb);
    // normal vector
    vecNormal = vecB.cross(vecA);
    vecNormal = vecNormal / norm(vecNormal);

    return 0;
}


int ColorPlane(const Mat& vecN, const Mat& vecP, const Mat& Depth, const float& eps_mm, Mat& bgrImage,
                    Mat& vecC, Point& posC, float& maxheight, bool maskc)
{
    Point3f P;
    float h;

    maxheight = 0.0;
    posC = Point(0,0);

    // for all depth pixels
    for(int i = 0; i != Depth.rows; i ++){
        for(int j = 0; j != Depth.cols; j ++){

            // get the depth value in meters
            P.z = scale * Depth.at<unsigned short>(i, j);

            if( P.z != 0. ){      // if depth is available
                Zuv_to_XY(P.z, J_TO_U(j), I_TO_V(i), &(P.x), &(P.y)); // compute P.x and P.y from P.z, i and j

                // Pと平面の間の距離を計算する
//              h = fabs(vecN.dot(Mat(P) - vecP));
                h = vecN.dot(Mat(P) - vecP);

                // 現在最大の場合はそれを記録する
                if(h > maxheight){
                    maxheight = h;
                    vecC = Mat(P);
                    posC = Point(j, i);
                }

                // 距離がeps_mmより小さい場合はピクセルカラーを変更します
                if( fabs(h) < eps_mm ){
                    bgrImage.at<Vec3b>(i, j) += Vec3b(127,0,0);
                    // 地面に近いほど色を黒くする。地面が0では黒すぎるので地面を0.5m下げて、明るくした。
                    bgrImage.at<Vec3b>(i, j) *= (h < -0.5) ? 0: (h / eps_mm + 0.5);
                }
            }
            else if(maskc){
                bgrImage.at<Vec3b>(i, j) = Vec3b(0,0,0);
            }
        }
    }

    return 0;
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

