
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

    cv::Mat rgbaImg;        // rgba�摜rgba�摜�ɕϊ����ĕ\��
    cv::Mat depthImg;       // �O���[�X�P�[���摜 �f�v�X�f�[�^���O���[�X�P�[���ɕϊ����ĕ\��
    cv::Mat depthcoloredImg;    // depthImg���J���[�摜�ɂ��ċ��E�Ȃǂ�`�悵�ĕ\��

    // �ϐ��錾
    k4a_capture_t capture;      // kinect�̃L���v�`���n���h��
                                // �قړ����Ƀf�o�C�X�ŋL�^����Color�CDepth�CIr�Ȃǂ̈�A�̃f�[�^��\���D


        // �f�o�C�X�Ŏ擾�����摜�� k4a_device_get_capture() �ɂ���ĕԂ���� k4a_capture_t �I�u�W�F�N�g��ʂ��Ď擾
        // k4a_image_t �͉摜�f�[�^�Ɗ֘A���郁�^�f�[�^���Ǘ�����
    k4a_image_t color_image;    // �L���v�`���̃J���[�摜�̃n���h��
    k4a_image_t depth_image;    // �L���v�`���̃f�v�X�摜�̃n���h��

        // �J���[�摜
    int32_t color_image_height;         // �J���[�摜�̍���
    int32_t color_image_width;          // ��
    uint8_t* color_image_buffer;        // �J���[�摜�̃|�C���^

        // �f�v�X�摜
    int32_t depth_image_height;         // �f�v�X�摜�̍���
    int32_t depth_image_width;          // ��
    uint8_t* depth_image_buffer;        // �f�v�X�摜�̃|�C���^


        // �C���X�^���X�̐���
        // �C���X�^���X�̐������ɃR���X�g���N�^���Ăяo�����
        // �R���X�g���N�^�Ńf�o�C�X�̃I�[�v��, �J�����\���ݒ�, �J�����̃X�^�[�g���s��
    Kinect kinect;


        // try�u���b�N�̒��ŗ�O������ throw() �ŋL�q����
        // ��O�����������ꍇtry�u���b�N�̉��ɂ���catch�u���b�N�����s�����
    try
    {
        while (true) {
            switch (k4a_device_get_capture(kinect.device, &capture, 1000)) {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;      // ���C�����[�v������
            case K4A_WAIT_RESULT_TIMEOUT:
                std::cout << "�L���v�`���^�C���A�E�g" << std::endl;
                continue;   // ������x
            case K4A_WAIT_RESULT_FAILED:
                throw std::runtime_error("�L���v�`���Ɏ��s���܂���");
            }
        }

            // �L���v�`���n���h������J���[�摜�̃n���h�����擾����
        color_image = k4a_capture_get_color_image(capture);
            // �L���v�`���n���h������f�v�X�摜�̃n���h�����擾����
        depth_image = k4a_capture_get_depth_image(capture);


            // �J���[�摜�̃n���h������摜�̃f�[�^



    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}

