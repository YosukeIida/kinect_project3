
//--------------------------------------------------------------------
//  2022/10/25作成
//  kinect_project2をまとめ直す
//
//
//  VC++ディレクトリ / インクルードディレクトリに
//      C:\opencv347\build\install\include
//      C:\Program Files\Azure Kinect SDK v1.3.0\sdk\include
//
//      C:\Program Files\Azure Kinect SDK v1.4.1\sdk\include        (自宅用 kinect sdk 1.4.1のとき使用)
//  を追加する．
//
//  VC++ディレクトリ / ライブラリディレクトリに
//      C:\opencv347\build\install\x64\vc16\lib
//      C:\Program Files\Azure Kinect SDK v1.3.0\sdk\windows-desktop\amd64\release\lib
//
//      C:\opencv347\build\install\lib      (自宅用 opencv347)
//      C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\lib          (自宅用 kinect sdk 1.4.1)
//
//---------------------------------------------------------------------

#include <iostream>
#include <string>       // to_string()で使用
#include <fstream>      // ofstream()で使用

#include <opencv2/opencv.hpp>       // opencv347
#include <k4a/k4a.h>                // azure kinect sdk

#include "kinect.h"     // 自作ヘッダ


//  Visual C++でコンパイルするときにリンクするライブラリファイル
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui3347.lib")



int main() {

    // 変数宣言
    k4a_capture_t capture;      // kinectのキャプチャーハンドル
                                // ほぼ同時にデバイスで記録したColor，Depth，Irなどの一連の画像を表す．
                                // 
        // デバイスの画像は k4a_device_get_capture() によって返される k4a_capture_t オブジェクトを通して取得
    k4a_image_t color_image;    // kinectのカラー画像のイメージハンドル
    k4a_image_t depth_image;    // kinectのデプス画像のイメージハンドル



}




