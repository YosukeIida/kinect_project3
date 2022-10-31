
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

#include "mykinect.h"     // ����w�b�_


//  Visual C++�ŃR���p�C������Ƃ��Ƀ����N���郉�C�u�����t�@�C��
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui3347.lib")



int main() {

    cv::Mat rgbaImg;        // �J���[�Z���T�̃C���[�W�n���h���̉摜�f�[�^��rgba�摜�ɕϊ����ĕ\��
    cv::Mat depthImg;       // �f�v�X�Z���T�̃C���[�W�n���h���̃f�v�X�f�[�^���O���[�X�P�[���ɕϊ����ĕ\��
    cv::Mat depthcoloredImg;    // depthImg���J���[�摜�ɕϊ����ċ��E�Ȃǂ�`�悵�ĕ\��

    // �ϐ��錾
    k4a_capture_t capture;      // kinect�̃L���v�`���n���h��
                                // �قړ����Ƀf�o�C�X�ŋL�^����Color�CDepth�CIr�Ȃǂ̈�A�̃f�[�^��\���D


        // �f�o�C�X�Ŏ擾�����摜�� k4a_device_get_capture() �ɂ���ĕԂ���� k4a_capture_t �I�u�W�F�N�g��ʂ��Ď擾
        // k4a_image_t �͉摜�f�[�^�Ɗ֘A���郁�^�f�[�^���Ǘ�����
    k4a_image_t color_image_handle;    // �L���v�`���̃J���[�Z���T�̃n���h��
    k4a_image_t depth_image_handle;    // �L���v�`���̃f�v�X�Z���T�̃n���h��

        // �J���[�C���[�W
    int32_t color_image_height;         // �J���[�C���[�W�̍���
    int32_t color_image_width;          // ��
    uint8_t* color_image_buffer;        // �J���[�C���[�W�̃f�[�^�̃|�C���^

        // �f�v�X�C���[�W
    int32_t depth_image_height;         // �f�v�X�C���[�W�̍���
    int32_t depth_image_width;          // ��
    uint8_t* depth_image_buffer;        // �f�v�X�C���[�W�̃f�[�^�̃|�C���^


        // �C���X�^���X�̐���
        // �C���X�^���X�̐������ɃR���X�g���N�^���Ăяo�����
        // �R���X�g���N�^�Ńf�o�C�X�̃I�[�v��, �J�����\���ݒ�, �J�����̃X�^�[�g���s��
    KinectDevice kinectdevice;


        // try�u���b�N�̒��ŗ�O������ throw() �ŋL�q����
        // ��O�����������ꍇtry�u���b�N�̉��ɂ���catch�u���b�N�����s�����
    try
    {
            // �������[�v
        while (true) {

                // �L���v�`�����������Ă��邩�ǂ����𒲂ׂ�
            switch (k4a_device_get_capture(kinectdevice.device, &capture, 1000)) {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;      // ���C�����[�v������
            case K4A_WAIT_RESULT_TIMEOUT:
                std::cout << "�L���v�`���^�C���A�E�g" << std::endl;
                continue;   // ������x
            case K4A_WAIT_RESULT_FAILED:
                throw std::runtime_error("�L���v�`���Ɏ��s���܂���");
            }

                // �L���v�`���n���h������J���[�C���[�W�̃n���h�����擾����
            color_image_handle = k4a_capture_get_color_image(capture);
                // �L���v�`���n���h������f�v�X�C���[�W�̃n���h�����擾����
            depth_image_handle = k4a_capture_get_depth_image(capture);


                // �J���[�C���[�W�̃n���h������摜�̃f�[�^�C�����C�����擾����
            if (color_image_handle) {
                color_image_height = k4a_image_get_height_pixels(color_image_handle);
                color_image_width = k4a_image_get_width_pixels(color_image_handle);

                color_image_buffer = k4a_image_get_buffer(color_image_handle);

                    // �J���[�Z���T�̃f�[�^��RGBA�摜�ɕϊ�����
                rgbaImg = cv::Mat(color_image_height, color_image_width, CV_8UC4);      //4ch RGBA�摜
                rgbaImg.data = color_image_buffer;
            }


                // �f�v�X�C���[�W�̃n���h������[�x�f�[�^�C�����C�����擾����
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

