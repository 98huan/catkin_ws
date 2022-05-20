#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;

int main()
{
     cout << "----------读摄像机内外参数----------"<< endl;
    Mat img=imread("../1.jpg");     //当前路径在/test/build
    cv::imshow("image",img);
    cv::waitKey();

    return 0;
}


