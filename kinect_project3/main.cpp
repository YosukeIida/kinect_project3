
//--------------------------------------------------------------------
//  2022/10/25�쐬
//  kinect_project2���܂Ƃߒ���
//
//
//  VC++�f�B���N�g�� / �C���N���[�h�f�B���N�g����
//      C:\opencv347\build\install\include
//      C:\Program Files\Azure Kinect SDK v1.3.0\sdk\include
//
//      C:\Program Files\Azure Kinect SDK v1.4.1\sdk\include        (����p kinect sdk 1.4.1�̂Ƃ��g�p)
//  ��ǉ�����D
//
//  VC++�f�B���N�g�� / ���C�u�����f�B���N�g����
//      C:\opencv347\build\install\x64\vc16\lib
//      C:\Program Files\Azure Kinect SDK v1.3.0\sdk\windows-desktop\amd64\release\lib
//
//      C:\opencv347\build\install\lib      (����p opencv347)
//      C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\lib          (����p kinect sdk 1.4.1)
//
//---------------------------------------------------------------------

#include <iostream>
#include <string>       // to_string()�Ŏg�p
#include <fstream>      // ofstream()�Ŏg�p

#include <opencv2/opencv.hpp>       // opencv347
#include <k4a/k4a.h>                // azure kinect sdk

#include "kinect.h"     // ����w�b�_


//  Visual C++�ŃR���p�C������Ƃ��Ƀ����N���郉�C�u�����t�@�C��
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui3347.lib")



int main() {

    // �ϐ��錾
    k4a_capture_t capture;      // kinect�̃L���v�`���[�n���h��
                                // �قړ����Ƀf�o�C�X�ŋL�^����Color�CDepth�CIr�Ȃǂ̈�A�̉摜��\���D
                                // 
        // �f�o�C�X�̉摜�� k4a_device_get_capture() �ɂ���ĕԂ���� k4a_capture_t �I�u�W�F�N�g��ʂ��Ď擾
    k4a_image_t color_image;    // kinect�̃J���[�摜�̃C���[�W�n���h��
    k4a_image_t depth_image;    // kinect�̃f�v�X�摜�̃C���[�W�n���h��



}




