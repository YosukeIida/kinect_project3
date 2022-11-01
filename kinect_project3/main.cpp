
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
#pragma comment(lib, "opencv_highgui347.lib")



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
    uint16_t* depth_image_buffer;        // �f�v�X�C���[�W�̃f�[�^�̃|�C���^

        // �v���Ώۂ̏㉺���E�̒[�̍��W���i�[
    int32_t depth_data_point_left;
    int32_t depth_data_point_right;
    int32_t depth_data_point_upper;
    int32_t depth_data_point_lower;

    int32_t key;

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
            switch (k4a_device_get_capture(kinectdevice.getdevice(), &capture, 1000)) {
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

                depth_image_buffer =(uint16_t*) k4a_image_get_buffer(depth_image_handle);

                    // �f�v�X�Z���T�̃f�[�^���O���[�X�P�[���摜�ɕϊ�����
                depthImg = cv::Mat(depth_image_height, depth_image_width, CV_8UC1);

                for (int y = 0; y < depth_image_height; y++) {
                    for (int x = 0; x < depth_image_width; x++) {
                        int address = y * depth_image_width + x;

                            // �O���[�X�P�[���摜�쐬 350mm:255, 605mm:0
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

                    // �f�v�X�摜(�O���[�X�P�[��)����J���[�摜���쐬
                cv::cvtColor(depthImg, depthcoloredImg, cv::COLOR_GRAY2BGR);

                // �ΐ���\��
                for (int x = 0; x < depth_image_width; x++) {
                    depthcoloredImg.at<cv::Vec3b>(288, x)[1] = 255;
                }
                for (int y = 0; y < depth_image_height; y++) {
                    depthcoloredImg.at<cv::Vec3b>(y, 320)[1] = 255;
                }


                // ������T��
                for (int x = 150; x < 550; x++) {
                    int address = 288 * depth_image_width + x;
                        // �v���Ώۂ�0mm�̑���ł��Ȃ����̓��������o����
                    if (depth_image_buffer[address] < 600 && depth_image_buffer[address] != 0) {
                        for (int y = 0; y < depth_image_height; y++) {
                            depthcoloredImg.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                        }
                        depth_data_point_left = x;
                        break;
                    }
                }

                // �E����T��
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
                break;      // ���C�����[�v������
            }
        }
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}

