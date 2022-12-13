
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
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>


#include <opencv2/opencv.hpp>       // opencv347
#include <k4a/k4a.h>                // azure kinect sdk

#include "kinectdevice.h"     // 自作ヘッダ

#define DEPTH_WIDTH         640             // デプスセンサ NOFV Unbinnedの横幅
#define DEPTH_HEIGHT        576             // デプスセンサ NFOV Unbinnedの縦幅
#define COLOR_WIDTH         1920            // カラーセンサの横幅
#define COLOR_HEIGHT        1080            // カラーセンサの縦幅

#define DEPTH_SEARCH_BORDER  710            // 計測対象を探索する境界値 この値より手前を探索する
#define DEPTH_IMAGE_FAR_LIMIT   750
#define DEPTH_IMAGE_NEAR_LIMIT  650         // グレースケール画像にする最小距離
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


//関数宣言
void get_color_image_data(k4a_image_t* color_image_handle, uint8_t** color_image_buffer);
void get_depth_image_data(k4a_image_t* depth_image_handle, uint16_t** depth_image_buffer);
void set_distance(std::vector<std::vector<int32_t> >&depthdata, uint16_t * depth_image_buffer);
void make_depthImg(cv::Mat* depthImg, uint16_t* depth_image_buffer);
void make_depthImg_2darray(cv::Mat* depthImgtemp, std::vector<std::vector<int32_t> > depthdata);
void make_depthrangeImg(cv::Mat* depthrangeImg, std::vector<std::vector<int32_t> > depthdata);





// capture から color_imageを取得
void get_color_image_data(k4a_image_t* color_image_handle, uint8_t** color_image_buffer) {

    *color_image_buffer = k4a_image_get_buffer(*color_image_handle);
}


// capture から depth_imageを取得
void get_depth_image_data(k4a_image_t* depth_image_handle, uint16_t** depth_image_buffer) {
    
    *depth_image_buffer = (uint16_t*)k4a_image_get_buffer(*depth_image_handle);
}


// depth_image_buffer の深度データから2次元vector配列を作成  (2次元配列にする)
void set_distance(std::vector<std::vector<int32_t> >& depthdata, uint16_t* depth_image_buffer) {
    int ptr = 0;
    for (int y = 0; y < DEPTH_HEIGHT; y++) {
        for (int x = 0; x < DEPTH_WIDTH; x++) {
            depthdata[x][y] = depth_image_buffer[ptr];
            ptr++;
        }
    }
}


// グレースケールのデプス画像を作成
// depth_image_buffer をラスタ操作で depthImg.data に深度データを入れていく．
void make_depthImg(cv::Mat* depthImg, uint16_t* depth_image_buffer) {
    for (int y = 0; y < DEPTH_HEIGHT; y++) {
        for (int x = 0; x < DEPTH_WIDTH; x++) {
            int address = y * DEPTH_WIDTH + x;

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


// グレースケールのデプス画像を作成
// 2次元配列からdepthImgを作成する
void make_depthImg_2darray(cv::Mat* depthImgtemp, std::vector<std::vector<int32_t> > depthdata) {
    for (int y = 0; y < DEPTH_HEIGHT; y++) {
        for (int x = 0; x < DEPTH_WIDTH; x++) {

            // グレースケール画像作成 350mmを白(255), 605mmを黒(0)で255段階
            if (depthdata[x][y] >= DEPTH_IMAGE_NEAR_LIMIT && depthdata[x][y] < DEPTH_IMAGE_NEAR_LIMIT + 255) {
                depthImgtemp->at<uint8_t>(y, x) = 255 - (depthdata[x][y] - DEPTH_IMAGE_NEAR_LIMIT);
            }
            else if (depthdata[x][y] == 0) {
                depthImgtemp->at<uint8_t>(y, x) = 255;
            }
            else {
                depthImgtemp->at<uint8_t>(y, x) = 0;
            }
        }
    }
}


// グレースケールのデプス画像を作成
// DEPTH_IMAGE_NEAR_LIMIT から DEPTH_IMAGE_FAR_LIMIT までを255段階で表示
void make_depthrangeImg(cv::Mat* depthrangeImg, std::vector<std::vector<int32_t> > depthdata) {
    for (int y = 0; y < DEPTH_HEIGHT; y++) {
        for (int x = 0; x < DEPTH_WIDTH; x++) {

            if (depthdata[x][y] >= DEPTH_IMAGE_NEAR_LIMIT && depthdata[x][y] < DEPTH_IMAGE_FAR_LIMIT) {
                depthrangeImg->at<uint8_t>(y, x) = (depthdata[x][y] - DEPTH_IMAGE_NEAR_LIMIT) * (255/(DEPTH_IMAGE_FAR_LIMIT - DEPTH_IMAGE_NEAR_LIMIT));
            }
            else if (depthdata[x][y] == 0) {
                depthrangeImg->at<uint8_t>(y, x) = 0;
            }
            else {
                depthrangeImg->at<uint8_t>(y, x) = 0;
            }
        }
    }
}



////  whileループ depth_image 1キャプチャ分の bufferのデータをcsvに書き込み
//void get_depth_surface_to_csv(uint16_t* depth_image_buffer) {
//    const char* filename = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\depth_surface.csv";
//    //const char* filename = "C:\\Users\\i1811402\\cpp_program\\kinect_project3\\data\\depth_surface.csv";
//    std::ofstream file_depth_surface(filename);
//    if (!file_depth_surface) {
//        throw std::runtime_error("depth_surface.csv が開けませんでした");
//    }
//
//    for (int y = 0; y < DEPTH_HEIGHT; y++) {
//        for (int x = 0; x < DEPTH_WIDTH; x++) {
//            int address = y * DEPTH_WIDTH + x;
//            file_depth_surface << depth_image_buffer[address] << ",";
//        }
//        file_depth_surface << std::endl;
//    }
//}


int search_measuretarget_left(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_row) {
    int temp_val = -1;
    int y = scan_line_row;
    for (int x = 0; x < DEPTH_WIDTH; x++) {
        // 計測対象の0mmになる部分の内側を検出する
        if (y >= DEPTH_HEIGHT || x >= DEPTH_WIDTH) {
            break;
        }
        else if (depthdata[x][y] < DEPTH_SEARCH_BORDER && depthdata[x][y] != 0) {
            temp_val = x;
            break;
        }
    }
    return temp_val;
}

int search_measuretarget_right(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_row, int32_t depth_data_point_left) {
    int temp_val = -1;
    int y = scan_line_row;
    for (int x = depth_data_point_left + 5; x < DEPTH_WIDTH - 1; x++) {
        if (y >= DEPTH_HEIGHT || x >= DEPTH_WIDTH) {
            break;
        }
        else if (depthdata[x][y] < DEPTH_SEARCH_BORDER && std::sqrt(std::pow((depthdata[x][y] - depthdata[x+1][y]), 2.0)) > 3.0) {
            temp_val = x;
            break;
        }
    }
    return temp_val + 1;
}



int search_measuretarget_upper(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_col) {
    int temp_val = -1;
    int x = scan_line_col;
    for (int y = 0; y < DEPTH_WIDTH; y++) {
        if (depthdata[x][y] < DEPTH_SEARCH_BORDER && depthdata[x][y] != 0) {
            temp_val = y;
            break;
        }
    }
    return temp_val;
}

int search_measuretarget_lower(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_col, int32_t depth_data_point_upper) {
    int temp_val = -1;
    int x = scan_line_col;
    for (int y = depth_data_point_upper + 5; y < DEPTH_HEIGHT; y++) {
        if (depthdata[x][y] < DEPTH_SEARCH_BORDER && std::sqrt(std::pow((depthdata[x][y] - depthdata[x][y+1]), 2.0)) > 3.0) {
            temp_val = y;
            break;
        }
    }
    return temp_val + 1;
    //return depth_data_point_upper + (depth_data_point_left - depth_data_point_right);
}


cv::Point3d depth2world(cv::Point3d measure_target_coord, cv::Point2d depth_coord_center){
    double z_distance;      // 距離
    double x_max_distance;       // 距離がdistanceの時の最大視野(mm)
    double x_distance_pixcel;
    
    double y_max_distance;
    double y_distance_pixcel;
    z_distance = measure_target_coord.z;

    // x方向
    x_max_distance = sin(NFOV_FOI_HOR) / cos(NFOV_FOI_HOR) * z_distance;
    x_distance_pixcel = depth_coord_center.x - (double)X_CENTER_COORD;
    measure_target_coord.x = x_distance_pixcel * x_max_distance / (double)(X_CENTER_COORD);

    // y方向
    y_max_distance = sin(NFOV_FOI_VERT) / cos(NFOV_FOI_VERT) * z_distance;
    y_distance_pixcel = depth_coord_center.y - (double)Y_CENTER_COORD;
    measure_target_coord.y = y_distance_pixcel * -1.0 * y_max_distance / (double)Y_CENTER_COORD; // スクリーン座標とワールド座標を逆にする
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

    uint8_t* color_image_buffer;        // カラーイメージのデータのポインタ

        // デプスイメージ
    k4a_image_t depth_image_handle;     // キャプチャのデプスセンサのハンドル

    uint16_t* depth_image_buffer = 0;        // デプスイメージのデータのポインタ

    std::vector<std::vector< int32_t > > depthdata(DEPTH_WIDTH, std::vector<int32_t>(DEPTH_HEIGHT));


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

    cv::Point2d depth_coord_center;         // 計測対象の中心座標 [pixcel]

    double depth_data_center_5x5;           // 計測対象中心深度 25マス移動平均

    bool flg_measureblock = false;          // ３次元位置計測ブロックを実行するフラグ

    const char* filename_mtc = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\a.csv";
    //const char* filename_mtc = "C:\\Users\\yosuk\\cpp_program\\kinect_project3\\data\\a.csv";
    std::ofstream fp_measure_target_coord(filename_mtc);
    if (!fp_measure_target_coord) {
        throw std::runtime_error("a.csv が開けませんでした");
    }
    KinectDevice kinectdevice;




        // tryブロックの中で例外処理を throw() で記述する
        // 例外が発生した場合tryブロックの下にあるcatchブロックが実行される
    try
    {
        // メインループ
        while (true) {



            // インスタンスの生成 生成時にコンストラクタが呼び出される
            // コンストラクタでデバイスのオープン, カメラ構成設定, カメラのスタートを行う

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


            if (color_image_handle) {
                // カラーイメージのハンドルから画像のデータ，高さ，幅を取得する
                get_color_image_data(&color_image_handle, &color_image_buffer);

                // カラーセンサのデータをRGBA画像に変換する
                rgbaImg = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);      //4ch RGBA画像
                rgbaImg.data = color_image_buffer;
                cv::imshow("rgbaImg", rgbaImg);
            }


            if (depth_image_handle) {
                // デプスイメージのハンドルから深度データ，高さ，幅を取得する
                get_depth_image_data(&depth_image_handle, &depth_image_buffer);

                // uint8_t* depth_image_bufferの深度データを vector<vector<int32_t>> depthdataの2次元配列に格納する
                set_distance(depthdata, depth_image_buffer);

                // デプスセンサのデータをグレースケール画像に変換する
                depthImg = cv::Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
                // make_depthImg(&depthImg, depth_image_buffer);
                make_depthImg_2darray(&depthImg, depthdata);
                

                depthrangeImg = cv::Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
                make_depthrangeImg(&depthrangeImg, depthdata);

                // デプス画像(グレースケール)からカラー画像を作成
                cv::cvtColor(depthImg, depthcoloredImg, cv::COLOR_GRAY2BGR);


                // 縦横の中央(x=320, y=288)に緑線を表示 (厳密には中央線の右，下のピクセルに線を描画)
                for (int x = 0; x < DEPTH_WIDTH; x++) {
                    depthcoloredImg.at<cv::Vec3b>(288, x)[1] = 255;
                }
                for (int y = 0; y < DEPTH_HEIGHT; y++) {
                    depthcoloredImg.at<cv::Vec3b>(y, 320)[1] = 255;

                }

                //// デプス画像の中央, 上下左右の深度を格納
                //depthimage_center_point = depth_image_buffer[DEPTH_HEIGHT / 2 * DEPTH_WIDTH + DEPTH_WIDTH / 2];
                //depthimage_left_point = depth_image_buffer[DEPTH_HEIGHT / 2 * DEPTH_WIDTH + 20];
                //depthimage_right_point = depth_image_buffer[DEPTH_HEIGHT / 2 * DEPTH_WIDTH + 620];
                //depthimage_upper_point = depth_image_buffer[20 * DEPTH_WIDTH + DEPTH_WIDTH / 2];
                //depthimage_lower_point = depth_image_buffer[(DEPTH_HEIGHT - 20) * DEPTH_WIDTH + DEPTH_WIDTH / 2];
                //// デプス画像の中央, 上下左右の深度を depthcoloredImg に表示
                //cv::putText(depthcoloredImg, std::to_string(depthimage_center_point), cv::Point(DEPTH_WIDTH / 2, DEPTH_HEIGHT / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_left_point), cv::Point(20, DEPTH_HEIGHT / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_right_point), cv::Point(580, DEPTH_HEIGHT / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_upper_point), cv::Point(DEPTH_WIDTH / 2, 20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_lower_point), cv::Point(DEPTH_WIDTH / 2, DEPTH_HEIGHT-20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);

                //// 縦青線を表示
                //for (int y = 0; y < DEPTH_HEIGHT; y++) {
                //    depthcoloredImg.at<cv::Vec3b>(y, 20) = cv::Vec3b(255, 0, 0);
                //    depthcoloredImg.at<cv::Vec3b>(y, 620) = cv::Vec3b(255, 0, 0);
                //}

                //for (int x = 0; x < DEPTH_WIDTH; x++) {
                //    depthcoloredImg.at<cv::Vec3b>(20, x) = cv::Vec3b(255, 0, 0);
                //    depthcoloredImg.at<cv::Vec3b>(DEPTH_HEIGHT - 20, x) = cv::Vec3b(255, 0, 0);
                //}

                // 3次元位置計測ループ
                if (flg_measure == true) {

                    //// 横 走査ライン探す
                    //int scan_line_upper = -1;
                    //int scan_line_lower = -1;
                    //for (int y = 0; y < DEPTH_HEIGHT; y+=10) {
                    //    for (int x = 0; x < DEPTH_WIDTH; x++) {
                    //        if (depthdata[x][y] < DEPTH_SEARCH_BORDER && depthdata[x][y] != 0) {
                    //            if (scan_line_upper == -1) {
                    //                scan_line_upper = y;
                    //            }
                    //            //scan_line_lower = y;
                    //            break;
                    //        }
                    //    }
                    //}


                    // Kinectを立てて計測するときはこっち↓

                    // 横 走査ライン探す
                    int scan_line_upper = -1;
                    int scan_line_lower = -1;
                    for (int x = 0; x < DEPTH_WIDTH; x += 10) {
                        for (int y = 0; y < DEPTH_HEIGHT; y++) {
                            if (depthdata[x][y] < DEPTH_SEARCH_BORDER && depthdata[x][y] != 0) {
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
                    int scan_line_row = (int)(scan_line_upper + scan_line_lower) / 2;
                    for (int x = 0; x < DEPTH_WIDTH; x++) {
                        if (scan_line_upper != -1) {
                            depthcoloredImg.at<cv::Vec3b>(scan_line_upper, x) = cv::Vec3b(255, 255, 0);
                            depthcoloredImg.at<cv::Vec3b>(scan_line_lower, x) = cv::Vec3b(255, 255, 0);
                        }
                    }

                    // 左側を探索
                    if (scan_line_row > 0 && scan_line_row < DEPTH_HEIGHT) {
                        depth_data_point_left = search_measuretarget_left(depthdata, scan_line_row);
                        if (depth_data_point_left != -1) {
                            // 左側 赤線を表示
                            for (int y = 0; y < DEPTH_HEIGHT; y++) {
                                depthcoloredImg.at<cv::Vec3b>(y, depth_data_point_left) = cv::Vec3b(0, 0, 255);
                            }
                            // 右側を探索
                            depth_data_point_right = search_measuretarget_right(depthdata, scan_line_row, depth_data_point_left);
                            if (depth_data_point_right != -1) {
                                for (int y = 0; y < DEPTH_HEIGHT; y++) {
                                    depthcoloredImg.at<cv::Vec3b>(y, depth_data_point_right) = cv::Vec3b(0, 0, 255);
                                }
                                // 計測対象の左右の中間線を表示
                                for (int y = 0; y < DEPTH_HEIGHT; y++) {
                                    depthcoloredImg.at<cv::Vec3b>(y, (depth_data_point_right + depth_data_point_left) / 2) = cv::Vec3b(255, 0, 0);
                                }
                                int scan_line_col = (int)(depth_data_point_left + depth_data_point_right) / 2;
                                // 上側を探索
                                depth_data_point_upper = search_measuretarget_upper(depthdata, scan_line_col);
                                if (depth_data_point_upper != -1) {
                                    for (int x = 0; x < DEPTH_WIDTH; x++) {
                                       // depthcoloredImg.at<cv::Vec3b>(depth_data_point_upper, x) = cv::Vec3b(0, 0, 255);
                                    }
                                    // 下側を探索
                                    depth_data_point_lower = search_measuretarget_lower(depthdata, scan_line_col, depth_data_point_upper);
                                    if (depth_data_point_lower != -1) {
                                        for (int x = 0; x < DEPTH_WIDTH; x++) {
                                            depthcoloredImg.at<cv::Vec3b>(depth_data_point_lower, x) = cv::Vec3b(0, 0, 255);
                                        }
                                        // 計測対象の上下の中間線を表示
                                        for (int x = 0; x < DEPTH_WIDTH; x++) {
                                            depthcoloredImg.at<cv::Vec3b>((depth_data_point_upper + depth_data_point_lower) / 2, x) = cv::Vec3b(255, 0, 0);
                                        }



                                        // 計測対象の中心座標を格納
                                        depth_coord_center.x = (depth_data_point_right + depth_data_point_left) / 2.0;
                                        depth_coord_center.y = (depth_data_point_upper + depth_data_point_lower) / 2.0;
                                        // 表示
                                        depthcoloredImg.at<cv::Vec3b>(depth_coord_center.y, depth_coord_center.x) = cv::Vec3b(0, 255, 255);


                                        // 計測対象の中心座標の深度を measure_target_coordに格納
                                        //measure_target_coord.z = depth_image_buffer[depth_coord_center.y * DEPTH_WIDTH + depth_coord_center.x];


                                        // 移動平均
                                        depth_data_center_5x5 = 0.0;
                                        for (int y = depth_coord_center.y - 2; y <= depth_coord_center.y + 2; y++) {
                                            for (int x = depth_coord_center.x - 2; x <= depth_coord_center.x + 2; x++) {
                                                depth_data_center_5x5 += depthdata[x][y] / 25.0;
                                            }
                                        }
                                        measure_target_coord.z = depth_data_center_5x5;


                                        // カメラ中心から計測対象の中心の角度を求める(x座標)
                                        //angle_x = ((depth_coord_center.x - X_CENTER_COORD) / (double)X_CENTER_COORD) * NFOV_FOI_HOR / 2.0;
                                        //measure_target_coord.x = ((depth_coord_center.x - X_CENTER_COORD) * measure_target_coord.z * tan(NFOV_FOI_HOR/2.0/180.0*M_PI)) / X_CENTER_COORD;
                                        measure_target_coord = depth2world(measure_target_coord, depth_coord_center);
                                        std::cout << std::setw(10) <<std::setprecision(5) << measure_target_coord.x << "\t" << measure_target_coord.y << "\t" << measure_target_coord.z << std::endl;



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

                }   // ３次元位置計測  if (flg_measure)


                cv::imshow("depthcoloredImg", depthcoloredImg);
                cv::imshow("depthrangeImg", depthrangeImg);
                cv::imshow("depth image", depthImg);

                

                if (flag_measure_target_coord != -1 && flag_measure_target_coord < 100) {
                    fp_measure_target_coord << measure_target_coord.x << "," << measure_target_coord.y << "," << measure_target_coord.z << "," << depth_coord_center.x << "," << depth_coord_center.y << std::endl;
                    flag_measure_target_coord++;
                }
                if (flag_measure_target_coord == 100) {
                    std::cout << "3次元データ記録終了" << std::endl;
                    flag_measure_target_coord = -1;
                }
                k4a_image_release(color_image_handle);
                k4a_image_release(depth_image_handle);
                k4a_capture_release(capture);

            }   // depthimageのif文
            cv::Mat image = cv::Mat::zeros(500, 500, CV_8UC3);

            int cols = image.cols;
            int rows = image.rows;
            for (int j = 0; j < rows; j++) {
                for (int i = 0; i < cols; i++) {
                    image.at<cv::Vec3b>(j, i)[0] = 255; //青
                    image.at<cv::Vec3b>(j, i)[1] = 255; //緑
                    image.at<cv::Vec3b>(j, i)[2] = 255; //赤
                }
            }

            cv::imshow("image", image);


            int key = cv::waitKey(1);
            if (key == 'q') {
                break;      // 
            }

            //if (key == 's') {
            //    std::cout << "get_depth_surface_to_csv 開始" << std::endl;
            //    get_depth_surface_to_csv(depth_image_buffer);
            //    std::cout << "get_depth_surface_to_csv 終了" << std::endl;
            //}

            if (key == 'p' && flag_measure_target_coord == -1) {
                std::cout << "3次元データ記録開始" << std::endl;
                flag_measure_target_coord = 0;
            }
        }   // メインループ
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}


