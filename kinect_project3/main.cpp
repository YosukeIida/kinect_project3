
//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------

#include <iostream>
#include <string>       // to_string()で使用
#include <fstream>      // ofstream()で使用

#define _USE_MATH_DEFINES
#include <math.h>


#include <opencv2/opencv.hpp>       // opencv347
#include <k4a/k4a.h>                // azure kinect sdk

#include "kinect.h"     // 自作ヘッダ

#define DEPTH_SEARCH_BORDER  600            // 計測対象を探索する境界値 この値より手前を探索する
#define DEPTH_IMAGE_NEAR_LIMIT  350         // グレースケール画像にする最小距離
#define X_CENTER_COORD  320                 // NFOV Unbinnedの横の中央座標
#define Y_CENTER_COORD  288                 // NFOV Unbinnedの縦の中央座標
#define KINECT_ANGLE_X  75                  // NFOV Unbinnedの横の画角
#define KINECT_ANGLE_Y  65                  // NFOV Unbinnedの縦の画角



//  Visual C++でコンパイルするときにリンクするライブラリファイル
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui347.lib")


// capture から color_imageを取得
void get_color_image_data(k4a_image_t* color_image_handle, int32_t* color_image_height, int32_t* color_image_width, uint8_t** color_image_buffer) {
    *color_image_height = k4a_image_get_height_pixels(*color_image_handle);
    *color_image_width = k4a_image_get_width_pixels(*color_image_handle);

    *color_image_buffer = k4a_image_get_buffer(*color_image_handle);
}


// capture から depth_imageを取得
void get_depth_image_data(k4a_image_t* depth_image_handle, int32_t* depth_image_height, int32_t* depth_image_width, uint16_t** depth_image_buffer) {
    *depth_image_height = k4a_image_get_height_pixels(*depth_image_handle);
    *depth_image_width = k4a_image_get_width_pixels(*depth_image_handle);

    *depth_image_buffer = (uint16_t*)k4a_image_get_buffer(*depth_image_handle);
}


// グレースケールのデプス画像を作成
void make_depthImg(cv::Mat* depthImg, int32_t depth_image_height, int32_t depth_image_width, uint16_t* depth_image_buffer) {
    for (int y = 0; y < depth_image_height; y++) {
        for (int x = 0; x < depth_image_width; x++) {
            int address = y * depth_image_width + x;

            // グレースケール画像作成 350mmを白(255), 605mmを黒(0)で255段階
            if (depth_image_buffer[address] >= DEPTH_IMAGE_NEAR_LIMIT && depth_image_buffer[address] < DEPTH_IMAGE_NEAR_LIMIT + 255) {
                depthImg->data[address] = 255 - (depth_image_buffer[address] - DEPTH_IMAGE_NEAR_LIMIT);
            }
            else if (depth_image_buffer[address] == 0) {
                depthImg->data[address] = 255;
            }
            else {
                depthImg->data[address] = 0;
            }
        }
    }
}


//  whileループ depth_image 1キャプチャ分の bufferのデータをcsvに書き込み
void get_depth_surface_to_csv(int32_t depth_image_height, int32_t depth_image_width, uint16_t* depth_image_buffer) {
    const char* filename = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\depth_surface.csv";
    //const char* filename = "C:\\Users\\i1811402\\cpp_program\\kinect_project3\\data\\depth_surface.csv";
    std::ofstream file_depth_surface(filename);
    if (!file_depth_surface) {
        throw std::runtime_error("depth_surface.csv が開けませんでした");
    }

    for (int y = 0; y < depth_image_height; y++) {
        for (int x = 0; x < depth_image_width; x++) {
            int address = y * depth_image_width + x;
            file_depth_surface << depth_image_buffer[address] << ",";
        }
         file_depth_surface << std::endl;
    }
}



int main() {

    cv::Mat rgbaImg;                    // カラーセンサのイメージハンドルの画像データをrgba画像に変換して表示
    cv::Mat depthImg;                   // デプスセンサのイメージハンドルのデプスデータをグレースケールに変換して表示
    cv::Mat depthcoloredImg;            // depthImgをカラー画像に変換して境界などを描画して表示


        // デバイスで取得した画像は k4a_device_get_capture() によって返される k4a_capture_t オブジェクトを通して取得
        // k4a_image_t は画像データと関連するメタデータを管理する
    k4a_capture_t capture;              // kinectのキャプチャハンドル
                                        // ほぼ同時にデバイスで記録したColor，Depth，Irなどの一連のデータを表す．

        // カラーイメージ
    k4a_image_t color_image_handle;     // キャプチャのカラーセンサのハンドル

    int32_t color_image_height;         // カラーイメージの高さ
    int32_t color_image_width;          // カラーイメージの幅
    uint8_t* color_image_buffer;        // カラーイメージのデータのポインタ

        // デプスイメージ
    k4a_image_t depth_image_handle;     // キャプチャのデプスセンサのハンドル

    int32_t depth_image_height = 0;         // デプスイメージの高さ
    int32_t depth_image_width = 0;          // デプスイメージの幅
    uint16_t* depth_image_buffer = 0;        // デプスイメージのデータのポインタ


        // 計測対象の上下左右の端の深度を格納
    int32_t depth_data_point_left;          // 計測対象の左端の深度
    int32_t depth_data_point_right;         // 計測対象の右端の深度
    int32_t depth_data_point_upper;         // 計測対象の上端の深度
    int32_t depth_data_point_lower;         // 計測対象の下端の深度

        // デプス画像のそれぞれの点の深度
    int32_t depthimage_center_point;        // デプス画像の中央の深度
    int32_t depthimage_left_point;          // デプス画像の左端の深度
    int32_t depthimage_right_point;         // デプス画像の右端の深度
    int32_t depthimage_upper_point;         // デプス画像の上端の深度
    int32_t depthimage_lower_point;         // デプス画像の下端の深度

    double angle_x;                         // 中央から計測対象の中心までのx軸の角度
    cv::Point3d measure_target_coord;        // 計測対象の3次元座標

    cv::Point2i depth_coord_center;         // 計測対象の中心座標

    int32_t key;


        // インスタンスの生成 生成時にコンストラクタが呼び出される
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
                get_color_image_data(&color_image_handle, &color_image_height, &color_image_width, &color_image_buffer);
                
                    // カラーセンサのデータをRGBA画像に変換する
                rgbaImg = cv::Mat(color_image_height, color_image_width, CV_8UC4);      //4ch RGBA画像
                rgbaImg.data = color_image_buffer;
                cv::imshow("rgbaImg",rgbaImg);
            }


                // デプスイメージのハンドルから深度データ，高さ，幅を取得する
            if (depth_image_handle) {
                get_depth_image_data(&depth_image_handle, &depth_image_height, &depth_image_width, &depth_image_buffer);

                    // デプスセンサのデータをグレースケール画像に変換する
                depthImg = cv::Mat(depth_image_height, depth_image_width, CV_8UC1);
                make_depthImg(&depthImg, depth_image_height, depth_image_width, depth_image_buffer);

                    // デプス画像(グレースケール)からカラー画像を作成
                cv::cvtColor(depthImg, depthcoloredImg, cv::COLOR_GRAY2BGR);

                // 緑線を表示
                for (int x = 0; x < depth_image_width; x++) {
                    depthcoloredImg.at<cv::Vec3b>(288, x)[1] = 255;
                }
                for (int y = 0; y < depth_image_height; y++) {
                    depthcoloredImg.at<cv::Vec3b>(y, 320)[1] = 255;
                    
                }

                //// デプス画像の中央, 上下左右の深度を格納
                //depthimage_center_point = depth_image_buffer[depth_image_height / 2 * depth_image_width + depth_image_width / 2];
                //depthimage_left_point = depth_image_buffer[depth_image_height / 2 * depth_image_width + 20];
                //depthimage_right_point = depth_image_buffer[depth_image_height / 2 * depth_image_width + 620];
                //depthimage_upper_point = depth_image_buffer[20 * depth_image_width + depth_image_width / 2];
                //depthimage_lower_point = depth_image_buffer[(depth_image_height - 20) * depth_image_width + depth_image_width / 2];
                //// デプス画像の中央, 上下左右の深度を depthcoloredImg に表示
                //cv::putText(depthcoloredImg, std::to_string(depthimage_center_point), cv::Point(depth_image_width / 2, depth_image_height / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_left_point), cv::Point(20, depth_image_height / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_right_point), cv::Point(580, depth_image_height / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_upper_point), cv::Point(depth_image_width / 2, 20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_lower_point), cv::Point(depth_image_width / 2, depth_image_height-20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);

                //// 縦青線を表示
                //for (int y = 0; y < depth_image_height; y++) {
                //    depthcoloredImg.at<cv::Vec3b>(y, 20) = cv::Vec3b(255, 0, 0);
                //    depthcoloredImg.at<cv::Vec3b>(y, 620) = cv::Vec3b(255, 0, 0);
                //}

                //for (int x = 0; x < depth_image_width; x++) {
                //    depthcoloredImg.at<cv::Vec3b>(20, x) = cv::Vec3b(255, 0, 0);
                //    depthcoloredImg.at<cv::Vec3b>(depth_image_height - 20, x) = cv::Vec3b(255, 0, 0);
                //}

                //// 横 走査ライン探す
                //int scan_line_upper = -1;
                //int scan_line_lower = -1;
                //for (int y = 0; y < depth_image_height; y++) {
                //    for (int x = 0; x < depth_image_width; x++) {
                //        int address = y * depth_image_width + x;
                //        if (depth_image_buffer[address] < DEPTH_SEARCH_BORDER && depth_image_buffer[address] != 0) {
                //            if (scan_line_upper == -1) {
                //                scan_line_upper = y;
                //            }
                //            break;
                //        }
                //    }
                //}
                //std::cout << scan_line_upper << scan_line_lower << std::endl;

                //for (int x = 0; x < depth_image_width; x++) {
                //    depthcoloredImg.at<cv::Vec3b>(scan_line_upper, x) = cv::Vec3b(0, 255, 0);
                //    depthcoloredImg.at<cv::Vec3b>(scan_line_lower, x) = cv::Vec3b(0, 255, 0);
                //}

                // 左側を探索

                depth_data_point_left = 0;
                for (int x = 150; x < 550; x++) {
                    int address = Y_CENTER_COORD * depth_image_width + x;
                        // 計測対象の0mmの測定できない縁の内側を検出する
                    if (depth_image_buffer[address] < DEPTH_SEARCH_BORDER && depth_image_buffer[address] != 0) {
                        for (int y = 0; y < depth_image_height; y++) {
                            depthcoloredImg.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                        }
                        depth_data_point_left = x;
                        break;
                    }
                }

                // 右側を探索
                for (int x = depth_data_point_left; x < 550; x++) {
                    int address = Y_CENTER_COORD * depth_image_width + x;
                        // 
                    if (depth_image_buffer[address] < DEPTH_SEARCH_BORDER && depth_image_buffer[address + 1] == 0) {
                        for (int y = 0; y < depth_image_height; y++) {
                            depthcoloredImg.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                        }
                        depth_data_point_right = x;
                        break;
                    }
                }
                // 計測対象の左右の中間線を表示
                for (int y = 0; y < depth_image_height; y++) {
                    depthcoloredImg.at<cv::Vec3b>(y, (depth_data_point_right + depth_data_point_left) / 2) = cv::Vec3b(255, 0, 0);
                }


                // 上側を探索
                for (int y = 150; y < 500; y++) {
                    int address = y * depth_image_width + (depth_data_point_right + depth_data_point_left) / 2;

                    if (depth_image_buffer[address] < DEPTH_SEARCH_BORDER && depth_image_buffer[address] != 0) {
                        for ( int x = 0; x <depth_image_width; x++) {
                            depthcoloredImg.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                        }
                        depth_data_point_upper = y;
                        break;
                    }
                }


                // 下側を探索
                for (int y = depth_data_point_upper; y < 500; y++) {
                    int address = y * depth_image_width + (depth_data_point_right + depth_data_point_left) / 2;

                    if (depth_image_buffer[address] - depth_image_buffer[address + depth_image_width] > 5) {
                        for ( int x = 0; x <depth_image_width; x++) {
                            depthcoloredImg.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                        }
                        depth_data_point_lower = y;
                        break;
                    }
                }
                // 計測対象の上下の中間線を表示
                for (int x = 0; x < depth_image_width; x++) {
                    depthcoloredImg.at<cv::Vec3b>((depth_data_point_upper + depth_data_point_lower) / 2, x) = cv::Vec3b(255, 0, 0);
                }


                // 計測対象の中心座標を格納
                depth_coord_center.x = (depth_data_point_right + depth_data_point_left) / 2;
                depth_coord_center.y = (depth_data_point_upper + depth_data_point_lower) / 2;
                // 表示
                depthcoloredImg.at<cv::Vec3b>(depth_coord_center.y, depth_coord_center.x) = cv::Vec3b(0, 255, 255);


                // 計測対象の中心座標の深度を measure_target_coordに格納
                measure_target_coord.z = depth_image_buffer[depth_coord_center.y * depth_image_width + depth_coord_center.x];


                // カメラ中心から計測対象の中心の角度を求める(x座標)
                    angle_x = ((depth_coord_center.x - X_CENTER_COORD) / (double)X_CENTER_COORD) * KINECT_ANGLE_X / 2.0;
                    measure_target_coord.x = measure_target_coord.z * sin(angle_x/180.0 * M_PI);

                     std::cout << measure_target_coord << std::endl;


                cv::imshow("depthcoloredImg", depthcoloredImg);
                cv::imshow("depth image", depthImg);


            }
            cv::waitKey(1);


            key = cv::waitKey(1);
            if (key == 'q') {
                break;      // メインループ抜ける
            }

            if (key == 's') {
                std::cout << "get_depth_surface_to_csv 開始" << std::endl;
                get_depth_surface_to_csv(depth_image_height, depth_image_width, depth_image_buffer);
                std::cout << "get_depth_surface_to_csv 終了" << std::endl;
            }

            k4a_image_release(color_image_handle);
            k4a_image_release(depth_image_handle);
            k4a_capture_release(capture);
        }
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}


