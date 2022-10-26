
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

    cv::Mat rgbaImg;        // rgba画像rgba画像に変換して表示
    cv::Mat depthImg;       // グレースケール画像 デプスデータをグレースケールに変換して表示
    cv::Mat depthcoloredImg;    // depthImgをカラー画像にして境界などを描画して表示

    // 変数宣言
    k4a_capture_t capture;      // kinectのキャプチャハンドル
                                // ほぼ同時にデバイスで記録したColor，Depth，Irなどの一連のデータを表す．


        // デバイスで取得した画像は k4a_device_get_capture() によって返される k4a_capture_t オブジェクトを通して取得
        // k4a_image_t は画像データと関連するメタデータを管理する
    k4a_image_t color_image;    // キャプチャのカラー画像のハンドル
    k4a_image_t depth_image;    // キャプチャのデプス画像のハンドル

        // カラー画像
    int32_t color_image_height;         // カラー画像の高さ
    int32_t color_image_width;          // 幅
    uint8_t* color_image_buffer;        // カラー画像のポインタ

        // デプス画像
    int32_t depth_image_height;         // デプス画像の高さ
    int32_t depth_image_width;          // 幅
    uint8_t* depth_image_buffer;        // デプス画像のポインタ


        // インスタンスの生成
        // インスタンスの生成時にコンストラクタが呼び出される
        // コンストラクタでデバイスのオープン, カメラ構成設定, カメラのスタートを行う
    Kinect kinect;


        // tryブロックの中で例外処理を throw() で記述する
        // 例外が発生した場合tryブロックの下にあるcatchブロックが実行される
    try
    {
        while (true) {
            switch (k4a_device_get_capture(kinect.device, &capture, 1000)) {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;      // メインループ抜ける
            case K4A_WAIT_RESULT_TIMEOUT:
                std::cout << "キャプチャタイムアウト" << std::endl;
                continue;   // もう一度
            case K4A_WAIT_RESULT_FAILED:
                throw std::runtime_error("キャプチャに失敗しました");
            }
        }

            // キャプチャハンドルからカラー画像のハンドルを取得する
        color_image = k4a_capture_get_color_image(capture);
            // キャプチャハンドルからデプス画像のハンドルを取得する
        depth_image = k4a_capture_get_depth_image(capture);


            // カラー画像のハンドルから画像のデータ



    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}

