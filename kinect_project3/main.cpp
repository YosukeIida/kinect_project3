
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


//  プログラム実行中のキー操作
//  "q" プログラム終了
//  "d" デプスカメラ記録開始  100フレーム計測後プログラム終了
//  "c" カラーカメラ記録開始  100フレーム計測後プログラム終了
//  "m" 両方記録
//  



#include <iostream>
#include <vector>
#include <string>       // to_string()で使用
#include <fstream>      // ofstream()で使用
#include <iomanip>      // 浮動小数点数の表示方法

#define _USE_MATH_DEFINES
#include <math.h>


#include <opencv2/opencv.hpp>       // opencv347
#include <opencv2/aruco.hpp>        // opencv contribution aruco
#include <k4a/k4a.h>                // azure kinect sdk

#include "kinectdevice.h"     // 自作ヘッダ

#define DEPTH_WIDTH         640             // デプスセンサ NOFV Unbinnedの横幅
#define DEPTH_HEIGHT        576             // デプスセンサ NFOV Unbinnedの縦幅
#define COLOR_WIDTH         1920            // カラーセンサの横幅
#define COLOR_HEIGHT        1080            // カラーセンサの縦幅

#define MARKER_LENGTH       0.0180           // ArUcoマーカー1辺の長さ [m]

#define DEPTH_SEARCH_BORDER  520            // 計測対象を探索する境界値 この値より手前を探索する
#define DEPTH_IMAGE_FAR_LIMIT   550
#define DEPTH_IMAGE_NEAR_LIMIT  450         // グレースケール画像にする最小距離
         // グレースケール画像にする最大距離
#define X_CENTER_COORD  320                 // NFOV Unbinnedの横の中央座標
#define Y_CENTER_COORD  288                 // NFOV Unbinnedの縦の中央座標
#define NFOV_FOI_HOR  (75.0 / 2.0) / 180.0 * (double)M_PI                  // NFOV Unbinnedの水平視野角
#define NFOV_FOI_VERT  (65.0 / 2.0) / 180.0 * (double)M_PI                  // NFOV Unbinnedの垂直視野角


//  Kinectを立てて撮影する時に切り替える．
#define KINECT_ATTITUDE     0               // 水平方向 : 0,    垂直方向 : 1



//  Visual C++でコンパイルするときにリンクするライブラリファイル
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui347.lib")
#pragma comment(lib, "opencv_aruco347.lib")


//関数宣言
void get_color_image_data(k4a_image_t* color_image_handle, uint8_t** color_image_buffer);
void get_depth_image_data(k4a_image_t* depth_image_handle, uint16_t** depth_image_buffer);
void set_distance(std::vector<std::vector<int32_t> >&depthdata, uint16_t * depth_image_buffer);
void make_depthImg(cv::Mat* depthImg, uint16_t* depth_image_buffer);
void make_depthImg_2darray(cv::Mat* depthImgtemp, std::vector<std::vector<int32_t> > depthdata);
void make_depthrangeImg(cv::Mat* depthrangeImg, std::vector<std::vector<int32_t> > depthdata);
int search_measuretarget_left(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_row);
int search_measuretarget_right(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_row, int32_t depth_data_point_left);
int search_measuretarget_upper(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_col);
int search_measuretarget_lower(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_col, int32_t depth_data_point_upper);
cv::Point3d depth2world(cv::Point3d measure_target_coord, cv::Point2d depth_coord_center);





int main() {

    // 表示する画像用Mat配列
    cv::Mat rgbaImg;                    // カラーセンサのイメージハンドルの画像データをrgba画像に変換して表示
    cv::Mat colorImg;                   // arucoで使用 4chRGBA -> 3chRGB

    cv::Mat depthImg;                   // デプスセンサのイメージハンドルのデプスデータをグレースケールに変換して表示
    cv::Mat depthrangeImg;
    cv::Mat depthcoloredImg;            // depthImgをカラー画像に変換して境界などを描画して表示


    // カラーカメラ行列 (カメラ内部行列)
    cv::Mat color_camera_matrix(3, 3, CV_64FC1);
    color_camera_matrix.at<double>(0, 0) = 909.98956298828125000000;      // fx
    color_camera_matrix.at<double>(0, 1) = 0.0;                           // 0.0
    color_camera_matrix.at<double>(0, 2) = 961.07861328125000000000;      // cx
    color_camera_matrix.at<double>(1, 0) = 0.0;                           // 0.0
    color_camera_matrix.at<double>(1, 1) = 909.63812255859375000000;      // fy
    color_camera_matrix.at<double>(1, 2) = 553.43408203125000000000;      // cy
    color_camera_matrix.at<double>(2, 0) = 0.0;                           // 0.0
    color_camera_matrix.at<double>(2, 1) = 0.0;                           // 0.0
    color_camera_matrix.at<double>(2, 2) = 1.0;                           // 1.0
    // カメラ行列 (歪み distortion)
    cv::Mat color_camera_dist_coeffs(1, 5, CV_64FC1);
    color_camera_dist_coeffs.at<double>(0, 0) = 0.46717128157615661621;          // k1
    color_camera_dist_coeffs.at<double>(0, 1) = -2.45866727828979492188;         // k2
    color_camera_dist_coeffs.at<double>(0, 2) = 0.00136364088393747807;          // p1
    color_camera_dist_coeffs.at<double>(0, 3) = -0.00006751885666744784;          // p2
    color_camera_dist_coeffs.at<double>(0, 4) = 1.37056386470794677734;         // k3


    // デプスカメラ行列 (カメラ内部行列)
    cv::Mat depth_camera_matrix(3, 3, CV_64FC1);
    depth_camera_matrix.at<double>(0, 0) = 503.22808837890625000000;      // fx
    depth_camera_matrix.at<double>(0, 1) = 0.0;                           // 0.0
    depth_camera_matrix.at<double>(0, 2) = 314.59005737304687500000;      // cx
    depth_camera_matrix.at<double>(1, 0) = 0.0;                           // 0.0
    depth_camera_matrix.at<double>(1, 1) = 503.35195922851562500000;      // fy
    depth_camera_matrix.at<double>(1, 2) = 332.22369384765625000000;      // cy
    depth_camera_matrix.at<double>(2, 0) = 0.0;                           // 0.0
    depth_camera_matrix.at<double>(2, 1) = 0.0;                           // 0.0
    depth_camera_matrix.at<double>(2, 2) = 1.0;                           // 1.0
    // カメラ行列 (歪み distortion)
    cv::Mat depth_camera_dist_coeffs(1, 5, CV_64FC1);
    depth_camera_dist_coeffs.at<double>(0, 0) = 2.48467683792114257812;          // k1
    depth_camera_dist_coeffs.at<double>(0, 1) = 1.70563745498657226562;         // k2
    depth_camera_dist_coeffs.at<double>(0, 2) = -0.00007825181819498539;          // p1
    depth_camera_dist_coeffs.at<double>(0, 3) = 0.00002669491004780866;          // p2
    depth_camera_dist_coeffs.at<double>(0, 4) = 0.09044291824102401733;         // k3



    // Create Marker Dictionary, Type of marker : 4x4, 1000
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_1000;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    // 検出したマーカーのIDを格納するベクター
    std::vector<int> marker_ids;

    //検出したマーカーのコーナー座標とリジェクト座標を格納する
    std::vector<std::vector<cv::Point2f>> marker_corners, rejectedCandidates;

    int sum_ids = 0;
    std::vector<cv::Vec3d> rvecs, tvecs;        // arucoマーカーのtvec, rvec

    cv::Point3d aruco_coord;


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

    //double angle_x;                         // 中央から計測対象の中心までのx軸の角度
    cv::Point3d measure_target_coord;        // 計測対象の3次元座標 [mm]

    cv::Point2d depth_coord_center;         // 計測対象の中心座標 [pixcel]

    double depth_data_center_5x5;           // 計測対象中心深度 25マス移動平均

    int key;
    int32_t flag_depth_measure_target_coord = -1;      // 3次元座標[mm]をcsvに記録を行うフラグ
    int32_t flag_aruco_coord_write = -1;    // arucoマーカーの3次元座標[mm]をcsvに書き込むフラグ

    bool flag_close = false;






    const char* filename_depth = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\depth.csv";
    //const char* filename_depth = "C:\\Users\\yosuk\\cpp_program\\kinect_project3\\data\\depth.csv";
    std::ofstream fp_depth_measure_target_coord(filename_depth);
    if (!fp_depth_measure_target_coord) {
        throw std::runtime_error("depth.csv が開けませんでした");
    }

    const char * filename_aruco = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\aruco.csv";
    //const char* filename_depth = "C:\\Users\\yosuk\\cpp_program\\kinect_project3\\data\\aruco.csv";
    std::ofstream fp_aruco_measure_target_coord(filename_aruco);
    if (!fp_aruco_measure_target_coord) {
        throw std::runtime_error("aruco.csv が開けませんでした");
    }


    KinectDevice kinectdevice;

    k4a_calibration_t device_calibration;
    k4a_device_get_calibration(kinectdevice.device, kinectdevice.device_configuration.depth_mode, kinectdevice.device_configuration.color_resolution, &device_calibration);
    /*
    k4a_calibration_camera_t depth_calib = device_calibration.depth_camera_calibration;
    std::cout << std::fixed << std::setprecision(20);
    std::cout << "resolution width: " << depth_calib.resolution_width << std::endl;
    std::cout << "resolution height: " << depth_calib.resolution_height << std::endl;
    std::cout << "principal point x: " << depth_calib.intrinsics.parameters.param.cx << std::endl;
    std::cout << "principal point y: " << depth_calib.intrinsics.parameters.param.cy << std::endl;
    std::cout << "focal length x: " << depth_calib.intrinsics.parameters.param.fx << std::endl;
    std::cout << "focal length y: " << depth_calib.intrinsics.parameters.param.fy << std::endl;
    std::cout << "radial distortion coefficients:" << std::endl;
    std::cout << "k1: " << depth_calib.intrinsics.parameters.param.k1 << std::endl;
    std::cout << "k2: " << depth_calib.intrinsics.parameters.param.k2 << std::endl;
    std::cout << "k3: " << depth_calib.intrinsics.parameters.param.k3 << std::endl;
    std::cout << "k4: " << depth_calib.intrinsics.parameters.param.k4 << std::endl;
    std::cout << "k5: " << depth_calib.intrinsics.parameters.param.k5 << std::endl;
    std::cout << "k6: " << depth_calib.intrinsics.parameters.param.k6 << std::endl;
    std::cout << "center of distortion in Z=1 plane, x: " << depth_calib.intrinsics.parameters.param.codx << std::endl;
    std::cout << "center of distortion in Z=1 plane, y: " << depth_calib.intrinsics.parameters.param.cody << std::endl;
    std::cout << "tangential distortion coefficient x: " << depth_calib.intrinsics.parameters.param.p1 << std::endl;
    std::cout << "tangential distortion coefficient y: " << depth_calib.intrinsics.parameters.param.p2 << std::endl;
    std::cout << "metric radius: " << depth_calib.intrinsics.parameters.param.metric_radius << std::endl;
    */



        // tryブロックの中で例外処理を throw() で記述する
        // 例外が発生した場合tryブロックの下にあるcatchブロックが実行される
    try
    {
        // 無限ループ
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
                //cv::imshow("rgbaImg", rgbaImg);

                colorImg = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC3);     // 3ch RGB画像
                cv::cvtColor(rgbaImg, colorImg, CV_RGBA2RGB);

                cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
                // サブピクセル化ON
                parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

                // arucoマーカを検出する
                cv::aruco::detectMarkers(colorImg, dictionary, marker_corners, marker_ids, parameters, rejectedCandidates);

                //// cv::outputarray を使って vector<int> を vector<cv::Mat>に変換して表示
                //cv::OutputArray marker_ids_outary = marker_ids;
                //std::vector<cv::Mat> show_marker_ids;
                //marker_ids_outary.getMatVector(show_marker_ids);

                //for (auto show_marker_ids : show_marker_ids) {
                //    std::cout << show_marker_ids << std::endl;
                //}

                //std::cout << "id:";
                //for (size_t num = 0; num < marker_ids.size(); num++) {
                //    std::cout << marker_ids[num] << ",";
                //}
                //std::cout << std::endl;

                //for (size_t num = 0; num < marker_ids.size(); num++) {
                //    std::cout << "[" << marker_ids[num] << "]";
                //    std::cout << "corner:";
                //    std::cout << marker_corners[num] << std::endl;
                //}

                //std::cout << std::endl << std::endl;

                if (marker_ids.size() > 0) {

                    // 検出したマーカを可視化
                    cv::aruco::drawDetectedMarkers(colorImg, marker_corners, marker_ids);

                    // マーカのrvec, tvecを求める
                    cv::aruco::estimatePoseSingleMarkers(marker_corners, MARKER_LENGTH, color_camera_matrix, color_camera_dist_coeffs, rvecs, tvecs);
                    for (int i = 0; i < marker_ids.size(); i++) {
                        std::cout << "tvecs:" << tvecs[i] * 1000 << std::endl;      // 単位[mm]
//                        std::cout << "rvecs:" << rvecs[i] << std::endl;
                        cv::aruco::drawAxis(colorImg, color_camera_matrix, color_camera_dist_coeffs, rvecs[i], tvecs[i], MARKER_LENGTH * 5);
                    }
                    aruco_coord = tvecs[0]*1000;
                    //aruco_coord.x = tvecs[0][0];
                    //aruco_coord.y = tvecs[0][1];
                    //aruco_coord.z = tvecs[0][2];
                }
                else {
                    aruco_coord = cv::Vec3d(9999.9999999, 9999.9999999, 9999.9999999);
                }
                //cv::putText(depthcoloredImg, std::to_string(depthimage_center_point), cv::Point(DEPTH_WIDTH / 2, DEPTH_HEIGHT / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                cv::putText(colorImg, "x:" + std::to_string(aruco_coord.x), cv::Point(100, 300), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255), 2, CV_AA);
                cv::putText(colorImg, "y:" + std::to_string(-aruco_coord.y), cv::Point(100, 400), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255), 2, CV_AA);
                cv::putText(colorImg, "z:" + std::to_string(aruco_coord.z), cv::Point(100, 500), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255), 2, CV_AA);

                cv::resize(colorImg, colorImg, cv::Size(), 0.5, 0.5);
                cv::imshow("colorImg", colorImg);


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

                int scan_line_upper = -1;
                int scan_line_lower = -1;
                if (KINECT_ATTITUDE == 0) {
                    // 横 走査ライン探す
                    for (int y = 0; y < DEPTH_HEIGHT; y += 10) {
                        for (int x = 0; x < DEPTH_WIDTH; x++) {
                            if (depthdata[x][y] < DEPTH_SEARCH_BORDER && depthdata[x][y] != 0) {
                                if (scan_line_upper == -1) {
                                    scan_line_upper = y;
                                }
                                //scan_line_lower = y;
                                break;
                            }
                        }
                    }
                }
                else if (KINECT_ATTITUDE == 1) {
                    // 横 走査ライン探す
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


                k4a_float2_t depth_point_2d;
                
                k4a_float3_t depth_mm_3d;

                int valid;







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
//                                    std::cout << std::setprecision(5) << std::setw(10) << measure_target_coord.x << std::setw(10) << measure_target_coord.y << std::setw(10) << measure_target_coord.z << std::setw(10) << depth_coord_center.x << std::setw(10) << depth_coord_center.y << std::setw(10) << std::endl;

                                    cv::putText(depthcoloredImg, "x:" + std::to_string(measure_target_coord.x), cv::Point(50, 200), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2, CV_AA);
                                    cv::putText(depthcoloredImg, "y:" + std::to_string(measure_target_coord.y), cv::Point(50, 300), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2, CV_AA);
                                    cv::putText(depthcoloredImg, "z:" + std::to_string(measure_target_coord.z), cv::Point(50, 400), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2, CV_AA);


                                    depth_point_2d.xy.x = depth_coord_center.x;
                                    depth_point_2d.xy.y = depth_coord_center.y;


                                    k4a_calibration_2d_to_3d(&device_calibration, &depth_point_2d, measure_target_coord.z, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &depth_mm_3d, &valid);
                                    cv::putText(depthcoloredImg, "x:" + std::to_string(depth_mm_3d.xyz.x), cv::Point(400, 200), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2, CV_AA);
                                    cv::putText(depthcoloredImg, "y:" + std::to_string(depth_mm_3d.xyz.y), cv::Point(400, 300), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2, CV_AA);
                                    cv::putText(depthcoloredImg, "z:" + std::to_string(depth_mm_3d.xyz.z), cv::Point(400, 400), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2, CV_AA);



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


                cv::imshow("depthcoloredImg", depthcoloredImg);
                cv::imshow("depthrangeImg", depthrangeImg);
                cv::imshow("depth image", depthImg);

                k4a_image_release(color_image_handle);
                k4a_image_release(depth_image_handle);
                k4a_capture_release(capture);

            }
            //cv::waitKey(1);


            key = cv::waitKey(10);
            if (key == 'q') {
                break;      // メインループ抜ける
            }

            //if (key == 's') {
            //    std::cout << "get_depth_surface_to_csv 開始" << std::endl;
            //    get_depth_surface_to_csv(depth_image_buffer);
            //    std::cout << "get_depth_surface_to_csv 終了" << std::endl;
            //}

            // 
            if (key == 'm' && flag_depth_measure_target_coord == -1 && flag_aruco_coord_write == -1) {
                std::cout << "\n\n両方記録開始\n\n";
                flag_depth_measure_target_coord = 0;
                flag_aruco_coord_write = 0;
            }

            if (key == 'd' && flag_depth_measure_target_coord == -1) {
                std::cout << "\n\n3次元データ記録開始\n\n" << std::endl;
                flag_depth_measure_target_coord = 0;
            }

            if (flag_depth_measure_target_coord != -1 && flag_depth_measure_target_coord < 100) {
                fp_depth_measure_target_coord << std::fixed << std::setprecision(10) << measure_target_coord.x << "," << measure_target_coord.y << "," << measure_target_coord.z << "," << depth_coord_center.x << "," << depth_coord_center.y << std::endl;
                flag_depth_measure_target_coord++;
            }
            if (flag_depth_measure_target_coord == 100) {
                std::cout << "\n\n3次元データ記録終了\n\n" << std::endl;
                flag_depth_measure_target_coord = -1;
                flag_close = true;
            }

            if (key == 'c' && flag_aruco_coord_write == -1) {
                std::cout << "\n\nArUco記録開始\n\n" << std::endl;
                flag_aruco_coord_write = 0;
            }
            if (flag_aruco_coord_write != -1 && flag_aruco_coord_write < 100) {
                fp_aruco_measure_target_coord << std::fixed << std::setprecision(10) << aruco_coord.x << "," << -aruco_coord.y << "," << aruco_coord.z << std::endl;
                flag_aruco_coord_write++;
            }
            if (flag_aruco_coord_write == 100) {
                std::cout << "\n\nArUco記録終了\n\n" << std::endl;
                flag_aruco_coord_write = -1;
                flag_close = true;
            }


            if (flag_close == true) {
                break;
            }
        }
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}



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
                depthrangeImg->at<uint8_t>(y, x) = (depthdata[x][y] - DEPTH_IMAGE_NEAR_LIMIT) * (255 / (DEPTH_IMAGE_FAR_LIMIT - DEPTH_IMAGE_NEAR_LIMIT));
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


// デプス画像から計測対象の左側を探し，左側の座標を返す
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


// デプス画像から計測対象の右側を探し，右側の座標を返す
int search_measuretarget_right(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_row, int32_t depth_data_point_left) {
    int temp_val = -1;
    int y = scan_line_row;
    for (int x = depth_data_point_left + 5; x < DEPTH_WIDTH - 1; x++) {
        if (y >= DEPTH_HEIGHT || x >= DEPTH_WIDTH) {
            break;
        }
        else if (depthdata[x][y] < DEPTH_SEARCH_BORDER && std::sqrt(std::pow((depthdata[x][y] - depthdata[x + 1][y]), 2.0)) > 3.0) {
            temp_val = x;
            break;
        }
    }
    return temp_val + 1;
}


// デプス画像から計測対象の上側を探し，上側の座標を返す
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


// デプス画像から計測対象の下側を探し，下側の座標を返す
int search_measuretarget_lower(std::vector<std::vector<int32_t> > depthdata, int32_t scan_line_col, int32_t depth_data_point_upper) {
    int temp_val = -1;
    int x = scan_line_col;
    for (int y = depth_data_point_upper + 5; y < DEPTH_HEIGHT; y++) {
        if (depthdata[x][y] < DEPTH_SEARCH_BORDER && std::sqrt(std::pow((depthdata[x][y] - depthdata[x][y + 1]), 2.0)) > 3.0) {
            temp_val = y;
            break;
        }
    }
    return temp_val + 1;
    //return depth_data_point_upper + (depth_data_point_left - depth_data_point_right);
}


// デプス画像から計測対象のxy方向の座標を計算して返す
cv::Point3d depth2world(cv::Point3d measure_target_coord, cv::Point2d depth_coord_center) {
    double z_distance;      // 距離
    double x_max_distance;       // 距離がdistanceの時の最大視野(mm)
    double x_distance_pixcel;

    double y_max_distance;
    double y_distance_pixcel;
    z_distance = measure_target_coord.z;

    // x方向
    x_max_distance = sin(NFOV_FOI_HOR) / cos(NFOV_FOI_HOR) * z_distance;
    x_distance_pixcel = depth_coord_center.x - (double)X_CENTER_COORD;
    measure_target_coord.x = x_distance_pixcel * x_max_distance / (double)X_CENTER_COORD;

    // y方向
    y_max_distance = sin(NFOV_FOI_VERT) / cos(NFOV_FOI_VERT) * z_distance;
    y_distance_pixcel = depth_coord_center.y - (double)Y_CENTER_COORD;
    measure_target_coord.y = y_distance_pixcel * -1.0 * y_max_distance / (double)Y_CENTER_COORD; // スクリーン座標とワールド座標を逆にする
    return measure_target_coord;

}

