//--------------------------------------------------------------------
//  2022/09/27作成
//  コンストラクタ( kinectのオープン, kinectのカメラ設定, カメラの起動)
//  デストラクタ( カメラのストップ, kinectのクローズ)
//
//
//--------------------------------------------------------------------

#include <iostream>
#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>

#include "mykinect.h"         // 自作ヘッダ

// コンストラクタ
KinectDevice::KinectDevice():
    device(NULL),
    device_configuration(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL),
    serial_size(0)
{
    // デバイスの接続の有無、接続数を調べる
    device_count = k4a_device_get_installed_count();
    if (device_count == 0) {
        throw std::runtime_error("Kinectが接続されていません");
    }

    // Azure Kinectデバイスのオープン
        // "K4A_DEVICE_DEFAULT" で最初のデバイスを開く
        // 終了時は"k4a_device_close()" を呼び出す   
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device))) {
        throw std::runtime_error("デバイスをオープンできません");
    }

    // シリアルナンバーの取得
        // 最初は "serial_number = NULL" で呼び出し、シリアルナンバーに必要なバッファサイズを取得する
        // 2回目の呼び出して完全なシリアルナンバーを返す
    k4a_device_get_serialnum(device, NULL, &serial_size);
    serial_number = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial_number, &serial_size);
    std::cout << "Opened device:" << serial_number << std::endl;
    free(serial_number);

    // カメラ構成の設定
    device_configuration.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;    // カラーデータはBGRA32bit
    device_configuration.color_resolution = K4A_COLOR_RESOLUTION_1080P;       // 画質1080p 1920x1080(width, height)
    device_configuration.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;     // 640x576(width, height)
    device_configuration.camera_fps = K4A_FRAMES_PER_SECOND_30;         // 30fps
    device_configuration.synchronized_images_only = true;
    device_configuration.depth_delay_off_color_usec = 0;
    device_configuration.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    device_configuration.subordinate_delay_off_master_usec = 0;
    device_configuration.disable_streaming_indicator = true;

    // カメラを起動する
        // 終了時は "k4a_device_stop_cameras()" を呼び出す
    if (K4A_FAILED(k4a_device_start_cameras(device, &device_configuration))) {
        throw std::runtime_error("カメラを起動できません");
    }
}

// デストラクタ
KinectDevice::~KinectDevice()
{
    k4a_device_stop_cameras(device);
    std::cout << "stop device" << std::endl;
    k4a_device_close(device);
    std::cout << "close device" << std::endl;
}



