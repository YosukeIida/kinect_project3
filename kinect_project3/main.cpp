
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
#pragma comment(lib, "opencv_highgui347.lib")



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
    uint16_t* depth_image_buffer;        // デプスイメージのデータのポインタ

        // 計測対象の上下左右の端の座標を格納
    int32_t depth_data_point_left;
    int32_t depth_data_point_right;
    int32_t depth_data_point_upper;
    int32_t depth_data_point_lower;

    int32_t key;

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
            switch (k4a_device_get_capture(kinectdevice.getdevice(), &capture, 1000)) {
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

                depth_image_buffer =(uint16_t*) k4a_image_get_buffer(depth_image_handle);

                    // デプスセンサのデータをグレースケール画像に変換する
                depthImg = cv::Mat(depth_image_height, depth_image_width, CV_8UC1);

                for (int y = 0; y < depth_image_height; y++) {
                    for (int x = 0; x < depth_image_width; x++) {
                        int address = y * depth_image_width + x;

                            // グレースケール画像作成 350mm:255, 605mm:0
                        if (depth_image_buffer[address] >= 350 && depth_image_buffer[address] < 350 + 255) {
                            depthImg.data[address] = 255 - (depth_image_buffer[address] - 350);
                        }
                        else if (depth_image_buffer[address] == 0) {
                            depthImg.data[address] = 255;
                        }
                        else {
                            depthImg.data[address] = 0;
                        }
                    }
                }

                    // デプス画像(グレースケール)からカラー画像を作成
                cv::cvtColor(depthImg, depthcoloredImg, cv::COLOR_GRAY2BGR);

                // 緑線を表示
                for (int x = 0; x < depth_image_width; x++) {
                    depthcoloredImg.at<cv::Vec3b>(288, x)[1] = 255;
                }
                for (int y = 0; y < depth_image_height; y++) {
                    depthcoloredImg.at<cv::Vec3b>(y, 320)[1] = 255;
                }


                // 左側を探索
                for (int x = 150; x < 550; x++) {
                    int address = 288 * depth_image_width + x;
                        // 計測対象の0mmの測定できない縁の内側を検出する
                    if (depth_image_buffer[address] < 600 && depth_image_buffer[address] != 0) {
                        for (int y = 0; y < depth_image_height; y++) {
                            depthcoloredImg.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                        }
                        depth_data_point_left = x;
                        break;
                    }
                }

                // 右側を探索
                for (int x = depth_data_point_left; x < 550; x++) {
                    int address = 288 * depth_image_width + x;
                        // 
                    if (depth_image_buffer[address] < 600 && depth_image_buffer[address + 1] == 0) {
                        for (int y = 0; y < depth_image_height; y++) {
                            depthcoloredImg.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                        }
                        depth_data_point_right = x;
                        break;
                    }
                }

                cv::imshow("depthcoloredImg", depthcoloredImg);
                cv::imshow("depth image", depthImg);


            }
            cv::waitKey(1);

            k4a_image_release(color_image_handle);
            k4a_image_release(depth_image_handle);
            k4a_capture_release(capture);

            key = cv::waitKey(1);
            if (key == 'q') {
                break;      // メインループ抜ける
            }
        }
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}

