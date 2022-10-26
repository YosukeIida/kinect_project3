//----------------------------------------------------------------------
//  2022/09/27作成
//  自作ヘッダ
//  
//  kinect.cppで呼び出すメンバ変数，メンバ関数を記述
//
//----------------------------------------------------------------------

#pragma once

#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>

// Kinectクラス
class Kinect {

    // メンバ変数
private:
    // デバイスについての変数
    uint32_t device_count;      // pcに接続しているkinectの数
    size_t serial_size;         // 接続しているkinectのシリアルナンバー
    char* serial_number;        // シリアルナンバー
    k4a_device_configuration_t device_configuration;   // kinectのカメラモード設定変数

public:
    k4a_device_t device;        // kinectのデバイスハンドル


// メンバ関数
public:
    // コンストラクタ
    Kinect();
    // デストラクタ
    ~Kinect();
};

