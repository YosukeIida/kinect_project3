
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

#include "mykinect.h"     // 自作ヘッダ


//  Visual C++でコンパイルするときにリンクするライブラリファイル
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui3347.lib")



int main() {

    cv::Mat rgbaImg;        // カラーセンサのイメージハンドルの画像データをrgba画像に変換して表示
    cv::Mat depthImg;       // デプスセンサのイメージハンドルのデプスデータをグレースケールに変換して表示
    cv::Mat depthcoloredImg;    // depthImgをカラー画像に変換して境界などを描画して表示

    // 変数宣言
    k4a_capture_t capture;      // kinectのキャプチャハンドル
                                // ほぼ同時にデバイスで記録したColor，Depth，Irなどの一連のデータを表す．


        // デバイスで取得した画像は k4a_device_get_capture() によって返される k4a_capture_t オブジェクトを通して取得
        // k4a_image_t は画像データと関連するメタデータを管理する
    k4a_image_t color_image_handle;    // キャプチャのカラーセンサのハンドル
    k4a_image_t depth_image_handle;    // キャプチャのデプスセンサのハンドル

        // カラーイメージ
    int32_t color_image_height;         // カラーイメージの高さ
    int32_t color_image_width;          // 幅
    uint8_t* color_image_buffer;        // カラーイメージのデータのポインタ

        // デプスイメージ
    int32_t depth_image_height;         // デプスイメージの高さ
    int32_t depth_image_width;          // 幅
    uint8_t* depth_image_buffer;        // デプスイメージのデータのポインタ


        // インスタンスの生成
        // インスタンスの生成時にコンストラクタが呼び出される
        // コンストラクタでデバイスのオープン, カメラ構成設定, カメラのスタートを行う
    KinectDevice kinectdevice;


        // tryブロックの中で例外処理を throw() で記述する
        // 例外が発生した場合tryブロックの下にあるcatchブロックが実行される
    try
    {
            // 無限ループ
        while (true) {

                // キャプチャが成功しているかどうかを調べる
            switch (k4a_device_get_capture(kinectdevice.device, &capture, 1000)) {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;      // メインループ抜ける
            case K4A_WAIT_RESULT_TIMEOUT:
                std::cout << "キャプチャタイムアウト" << std::endl;
                continue;   // もう一度
            case K4A_WAIT_RESULT_FAILED:
                throw std::runtime_error("キャプチャに失敗しました");
            }

                // キャプチャハンドルからカラーイメージのハンドルを取得する
            color_image_handle = k4a_capture_get_color_image(capture);
                // キャプチャハンドルからデプスイメージのハンドルを取得する
            depth_image_handle = k4a_capture_get_depth_image(capture);


                // カラーイメージのハンドルから画像のデータ，高さ，幅を取得する
            if (color_image_handle) {
                color_image_height = k4a_image_get_height_pixels(color_image_handle);
                color_image_width = k4a_image_get_width_pixels(color_image_handle);

                color_image_buffer = k4a_image_get_buffer(color_image_handle);

                    // カラーセンサのデータをRGBA画像に変換する
                rgbaImg = cv::Mat(color_image_height, color_image_width, CV_8UC4);      //4ch RGBA画像
                rgbaImg.data = color_image_buffer;
            }


                // デプスイメージのハンドルから深度データ，高さ，幅を取得する
            if (depth_image_handle) {
                depth_image_height = k4a_image_get_height_pixels(depth_image_handle);
                depth_image_width = k4a_image_get_width_pixels(depth_image_handle);

                depth_image_buffer = k4a_image_get_buffer(depth_image_handle);

                    // 
            }




        }
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}

