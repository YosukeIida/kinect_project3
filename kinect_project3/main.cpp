
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

#define DEPTH_SEARCH_BORDER  720            // 計測対象を探索する境界値 この値より手前を探索する
#define DEPTH_IMAGE_NEAR_LIMIT  700         // グレースケール画像にする最小距離
#define DEPTH_IMAGE_FAR_LIMIT   750
         // グレースケール画像にする最大距離
#define X_CENTER_COORD  320                 // NFOV Unbinnedの横の中央座標
#define Y_CENTER_COORD  288                 // NFOV Unbinnedの縦の中央座標
#define NFOV_FOI_HOR  (75.0 / 2.0) / 180.0 * (double)M_PI                  // NFOV Unbinnedの水平視野角
#define NFOV_FOI_VERT  (65.0 / 2.0) / 180.0 * (double)M_PI                  // NFOV Unbinnedの垂直視野角



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

void make_depthrangeImg(cv::Mat* depthrangeImg, int32_t depth_image_height, int32_t depth_image_width, uint16_t* depth_image_buffer) {
    for (int y = 0; y < depth_image_height; y++) {
        for (int x = 0; x < depth_image_width; x++) {
            int address = y * depth_image_width + x;

            // グレースケール画像作成 350mmを白(255), 605mmを黒(0)で255段階
            if (depth_image_buffer[address] >= DEPTH_IMAGE_NEAR_LIMIT && depth_image_buffer[address] < DEPTH_IMAGE_FAR_LIMIT) {
                depthrangeImg->data[address] =(depth_image_buffer[address] - DEPTH_IMAGE_NEAR_LIMIT) * (255/(DEPTH_IMAGE_FAR_LIMIT - DEPTH_IMAGE_NEAR_LIMIT));
            }
            else if (depth_image_buffer[address] == 0) {
                depthrangeImg->data[address] = 0;
            }
            else {
                depthrangeImg->data[address] = 0;
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


int search_measuretarget_left(int32_t depth_image_height, int32_t depth_image_width, uint16_t* depth_image_buffer, int32_t scan_line) {
    int temp_val = -1;
    for (int x = 0; x < depth_image_width - 1; x++) {
        int address = scan_line * depth_image_width + x;
        // 計測対象の0mmになる部分の内側を検出する
        if (depth_image_buffer[address] < DEPTH_SEARCH_BORDER && depth_image_buffer[address] != 0) {
            temp_val = x;
            break;
        }
    }
    return temp_val;
}

int search_measuretarget_right(int32_t depth_image_height, int32_t depth_image_width, uint16_t* depth_image_buffer, int32_t depth_data_point_left, int32_t scan_line) {
    int temp_val = -1;
    for (int x = depth_data_point_left + 5; x < depth_image_width; x++) {
        int address = scan_line * depth_image_width + x;
        if (depth_image_buffer[address] < DEPTH_SEARCH_BORDER && std::sqrt(std::pow((depth_image_buffer[address] - depth_image_buffer[address + 1]), 2.0)) > 5.0) {
            temp_val = x;
            break;
        }
    }
    return temp_val;
}

int search_measuretarget_upper(int32_t depth_image_height, int32_t depth_image_width, uint16_t* depth_image_buffer, int32_t depth_data_point_left, int32_t depth_data_point_right) {
    int temp_val = -1;
    for (int y = 0; y < depth_image_width - 1; y++) {
        int address = y * depth_image_width + (depth_data_point_right + depth_data_point_left) / 2;
        if (depth_image_buffer[address] < DEPTH_SEARCH_BORDER && depth_image_buffer[address] != 0) {
            temp_val = y;
            break;
        }
    }
    return temp_val;
}

int search_measuretarget_lower(int32_t depth_image_height, int32_t depth_image_width, uint16_t* depth_image_buffer, int32_t depth_data_point_left, int32_t depth_data_point_right, int32_t depth_data_point_upper) {
    int temp_val = -1;
    for (int y = depth_data_point_upper + 5; y < depth_image_height; y++) {
        int address = y * depth_image_width + (depth_data_point_right + depth_data_point_left) / 2;
        if (depth_image_buffer[address] < DEPTH_SEARCH_BORDER && std::sqrt(std::pow((depth_image_buffer[address] - depth_image_buffer[address + depth_image_width]), 2.0)) > 3.0) {
            temp_val = y;
            break;
        }
    }
    return temp_val;
    //return depth_data_point_upper + (depth_data_point_left - depth_data_point_right);
}


cv::Point3d depth2world(int32_t depth_image_height, int32_t depth_image_width, cv::Point3d measure_target_coord, cv::Point2i depth_coord_center){
    double z_distance;      // 距離
    double x_max_distance;       // 距離がdistanceの時の最大視野(mm)
    double x_distance_pixcel;
    
    double y_max_distance;
    double y_distance_pixcel;
    z_distance = measure_target_coord.z;

    // x方向
    x_max_distance = sin(NFOV_FOI_HOR) / cos(NFOV_FOI_HOR) * z_distance;
    x_distance_pixcel = depth_coord_center.x - (depth_image_width / 2);
    measure_target_coord.x = x_distance_pixcel * x_max_distance / (depth_image_width / 2);

    // y方向
    y_max_distance = sin(NFOV_FOI_VERT) / cos(NFOV_FOI_VERT) * z_distance;
    y_distance_pixcel = depth_coord_center.y - (depth_image_height / 2);
    measure_target_coord.y = y_distance_pixcel * x_max_distance / (depth_image_height / 2);
    return measure_target_coord;

}



int main() {

    cv::Mat rgbaImg;                    // カラーセンサのイメージハンドルの画像データをrgba画像に変換して表示
    cv::Mat depthImg;                   // デプスセンサのイメージハンドルのデプスデータをグレースケールに変換して表示
    cv::Mat depthrangeImg;
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
    cv::Point3d measure_target_coord;        // 計測対象の3次元座標 [mm]

    cv::Point2i depth_coord_center;         // 計測対象の中心座標 [pixcel]

    double depth_data_center_5x5;           // 計測対象中心深度 25マス移動平均

    int32_t key;
    int32_t flag_measure_target_coord = -1;      // 3次元座標[mm]をcsvに記録を行うフラグ

    const char* filename_mtc = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\a.csv";
    std::ofstream fp_measure_target_coord(filename_mtc);
    if (!fp_measure_target_coord) {
        throw std::runtime_error("a.csv が開けませんでした");
    }


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

                depthrangeImg = cv::Mat(depth_image_height, depth_image_width, CV_8UC1);
                make_depthrangeImg(&depthrangeImg, depth_image_height, depth_image_width, depth_image_buffer);

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

                // 横 走査ライン探す
                int scan_line_upper = -1;
                int scan_line_lower = -1;
                for (int y = 0; y < depth_image_height; y++) {
                    for (int x = 0; x < depth_image_width; x++) {
                        int address = y * depth_image_width + x;
                        if (depth_image_buffer[address] < DEPTH_SEARCH_BORDER && depth_image_buffer[address] != 0) {
                            if (scan_line_upper == -1) {
                                scan_line_upper = y;
                            }
                            //scan_line_lower = y;
                            break;
                        }
                    }
                }
                //std::cout << scan_line_upper << scan_line_lower << std::endl;
                scan_line_lower = scan_line_upper + 20;
                int scan_line = (scan_line_upper + scan_line_lower) / 2;
                for (int x = 0; x < depth_image_width; x++) {
                    if (scan_line_upper != -1) {
                        depthcoloredImg.at<cv::Vec3b>(scan_line_upper, x) = cv::Vec3b(255, 255, 0);
                        depthcoloredImg.at<cv::Vec3b>(scan_line_lower, x) = cv::Vec3b(255, 255, 0);
                    }
                }

                // 左側を探索
                if (scan_line > 0) {
                    depth_data_point_left = search_measuretarget_left(depth_image_height, depth_image_width, depth_image_buffer, scan_line);
                    if (depth_data_point_left != -1) {
                        // 左側 赤線を表示
                        for (int y = 0; y < depth_image_height; y++) {
                            depthcoloredImg.at<cv::Vec3b>(y, depth_data_point_left) = cv::Vec3b(0, 0, 255);
                        }
                        // 右側を探索
                        depth_data_point_right = search_measuretarget_right(depth_image_height, depth_image_width, depth_image_buffer, depth_data_point_left, scan_line);
                        if (depth_data_point_right != -1) {
                            for (int y = 0; y < depth_image_height; y++) {
                                depthcoloredImg.at<cv::Vec3b>(y, depth_data_point_right) = cv::Vec3b(0, 0, 255);
                            }
                            // 計測対象の左右の中間線を表示
                            for (int y = 0; y < depth_image_height; y++) {
                                depthcoloredImg.at<cv::Vec3b>(y, (depth_data_point_right + depth_data_point_left) / 2) = cv::Vec3b(255, 0, 0);
                            }
                            // 上側を探索
                            depth_data_point_upper = search_measuretarget_upper(depth_image_height, depth_image_width, depth_image_buffer, depth_data_point_left, depth_data_point_right);
                            if (depth_data_point_upper != -1) {
                                for (int x = 0; x < depth_image_width; x++) {
                                    depthcoloredImg.at<cv::Vec3b>(depth_data_point_upper, x) = cv::Vec3b(0, 0, 255);
                                }
                                // 下側を探索
                                depth_data_point_lower = search_measuretarget_lower(depth_image_height, depth_image_width, depth_image_buffer, depth_data_point_left, depth_data_point_right, depth_data_point_upper);
                                if (depth_data_point_lower != -1) {
                                    for (int x = 0; x < depth_image_width; x++) {
                                        depthcoloredImg.at<cv::Vec3b>(depth_data_point_lower, x) = cv::Vec3b(0, 0, 255);
                                    }
                                    // 計測対象の上下の中間線を表示
                                    for (int x = 0; x < depth_image_width; x++) {
                                        depthcoloredImg.at<cv::Vec3b>((depth_data_point_upper + depth_data_point_lower) / 2, x) = cv::Vec3b(255, 0, 0);
                                    }
                                }
                                else {
                                    std::cout << "下側検出できない" << std::endl;
                                }
                            }
                            else {
                                std::cout << "上側検出できない" << std::endl;
                            }
                        }
                        else {
                            std::cout << "右側検出できない" << std::endl;
                        }
                    }
                    else {
                        std::cout << "左側検出できない" << std::endl;
                    }
                }



                // 計測対象の中心座標を格納
                depth_coord_center.x = (depth_data_point_right + depth_data_point_left) / 2;
                depth_coord_center.y = (depth_data_point_upper + depth_data_point_lower) / 2;
                // 表示
                depthcoloredImg.at<cv::Vec3b>(depth_coord_center.y, depth_coord_center.x) = cv::Vec3b(0, 255, 255);


                // 計測対象の中心座標の深度を measure_target_coordに格納
                //measure_target_coord.z = depth_image_buffer[depth_coord_center.y * depth_image_width + depth_coord_center.x];


                // 移動平均
                depth_data_center_5x5 = 0.0;
                for (int x = depth_coord_center.x - 2; x <= depth_coord_center.x + 2; x++) {
                    for (int y = depth_coord_center.y - 2; y <= depth_coord_center.y + 2; y++) {
                        depth_data_center_5x5 += depth_image_buffer[y * depth_image_width + x] / 25.0;
                    }
                }
                measure_target_coord.z = depth_data_center_5x5;


                // カメラ中心から計測対象の中心の角度を求める(x座標)
                //angle_x = ((depth_coord_center.x - X_CENTER_COORD) / (double)X_CENTER_COORD) * NFOV_FOI_HOR / 2.0;
                //measure_target_coord.x = ((depth_coord_center.x - X_CENTER_COORD) * measure_target_coord.z * tan(NFOV_FOI_HOR/2.0/180.0*M_PI)) / X_CENTER_COORD;
                measure_target_coord = depth2world(depth_image_height, depth_image_width, measure_target_coord, depth_coord_center);
                std::cout << measure_target_coord << std::endl;


                cv::imshow("depthcoloredImg", depthcoloredImg);
                cv::imshow("depthrangeImg", depthrangeImg);
                cv::imshow("depth image", depthImg);

                if (flag_measure_target_coord != -1 && flag_measure_target_coord < 100) {
                    fp_measure_target_coord << measure_target_coord.x <<"," << measure_target_coord.y <<"," <<measure_target_coord.z << std::endl;
                    flag_measure_target_coord++;
                }
                if (flag_measure_target_coord == 100) {
                    std::cout << "3次元データ記録終了" << std::endl;
                    flag_measure_target_coord = -1;
                }

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

            if (key == 'p' && flag_measure_target_coord == -1) {
                std::cout << "3次元データ記録開始" << std::endl;
                flag_measure_target_coord = 0;
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


