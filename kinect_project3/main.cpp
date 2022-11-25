
//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------

#include <iostream>
#include <string>       // to_string()�Ŏg�p
#include <fstream>      // ofstream()�Ŏg�p

#define _USE_MATH_DEFINES
#include <math.h>


#include <opencv2/opencv.hpp>       // opencv347
#include <k4a/k4a.h>                // azure kinect sdk

#include "kinect.h"     // ����w�b�_

#define DEPTH_SEARCH_BORDER  720            // �v���Ώۂ�T�����鋫�E�l ���̒l����O��T������
#define DEPTH_IMAGE_NEAR_LIMIT  700         // �O���[�X�P�[���摜�ɂ���ŏ�����
#define DEPTH_IMAGE_FAR_LIMIT   750
         // �O���[�X�P�[���摜�ɂ���ő勗��
#define X_CENTER_COORD  320                 // NFOV Unbinned�̉��̒������W
#define Y_CENTER_COORD  288                 // NFOV Unbinned�̏c�̒������W
#define NFOV_FOI_HOR  (75.0 / 2.0) / 180.0 * (double)M_PI                  // NFOV Unbinned�̐�������p
#define NFOV_FOI_VERT  (65.0 / 2.0) / 180.0 * (double)M_PI                  // NFOV Unbinned�̐�������p



//  Visual C++�ŃR���p�C������Ƃ��Ƀ����N���郉�C�u�����t�@�C��
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui347.lib")


// capture ���� color_image���擾
void get_color_image_data(k4a_image_t* color_image_handle, int32_t* color_image_height, int32_t* color_image_width, uint8_t** color_image_buffer) {
    *color_image_height = k4a_image_get_height_pixels(*color_image_handle);
    *color_image_width = k4a_image_get_width_pixels(*color_image_handle);

    *color_image_buffer = k4a_image_get_buffer(*color_image_handle);
}


// capture ���� depth_image���擾
void get_depth_image_data(k4a_image_t* depth_image_handle, int32_t* depth_image_height, int32_t* depth_image_width, uint16_t** depth_image_buffer) {
    *depth_image_height = k4a_image_get_height_pixels(*depth_image_handle);
    *depth_image_width = k4a_image_get_width_pixels(*depth_image_handle);

    *depth_image_buffer = (uint16_t*)k4a_image_get_buffer(*depth_image_handle);
}


// �O���[�X�P�[���̃f�v�X�摜���쐬
void make_depthImg(cv::Mat* depthImg, int32_t depth_image_height, int32_t depth_image_width, uint16_t* depth_image_buffer) {
    for (int y = 0; y < depth_image_height; y++) {
        for (int x = 0; x < depth_image_width; x++) {
            int address = y * depth_image_width + x;

            // �O���[�X�P�[���摜�쐬 350mm��(255), 605mm����(0)��255�i�K
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

            // �O���[�X�P�[���摜�쐬 350mm��(255), 605mm����(0)��255�i�K
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


//  while���[�v depth_image 1�L���v�`������ buffer�̃f�[�^��csv�ɏ�������
void get_depth_surface_to_csv(int32_t depth_image_height, int32_t depth_image_width, uint16_t* depth_image_buffer) {
    const char* filename = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\depth_surface.csv";
    //const char* filename = "C:\\Users\\i1811402\\cpp_program\\kinect_project3\\data\\depth_surface.csv";
    std::ofstream file_depth_surface(filename);
    if (!file_depth_surface) {
        throw std::runtime_error("depth_surface.csv ���J���܂���ł���");
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
        // �v���Ώۂ�0mm�ɂȂ镔���̓��������o����
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
    double z_distance;      // ����
    double x_max_distance;       // ������distance�̎��̍ő压��(mm)
    double x_distance_pixcel;
    
    double y_max_distance;
    double y_distance_pixcel;
    z_distance = measure_target_coord.z;

    // x����
    x_max_distance = sin(NFOV_FOI_HOR) / cos(NFOV_FOI_HOR) * z_distance;
    x_distance_pixcel = depth_coord_center.x - (depth_image_width / 2);
    measure_target_coord.x = x_distance_pixcel * x_max_distance / (depth_image_width / 2);

    // y����
    y_max_distance = sin(NFOV_FOI_VERT) / cos(NFOV_FOI_VERT) * z_distance;
    y_distance_pixcel = depth_coord_center.y - (depth_image_height / 2);
    measure_target_coord.y = y_distance_pixcel * x_max_distance / (depth_image_height / 2);
    return measure_target_coord;

}



int main() {

    cv::Mat rgbaImg;                    // �J���[�Z���T�̃C���[�W�n���h���̉摜�f�[�^��rgba�摜�ɕϊ����ĕ\��
    cv::Mat depthImg;                   // �f�v�X�Z���T�̃C���[�W�n���h���̃f�v�X�f�[�^���O���[�X�P�[���ɕϊ����ĕ\��
    cv::Mat depthrangeImg;
    cv::Mat depthcoloredImg;            // depthImg���J���[�摜�ɕϊ����ċ��E�Ȃǂ�`�悵�ĕ\��


        // �f�o�C�X�Ŏ擾�����摜�� k4a_device_get_capture() �ɂ���ĕԂ���� k4a_capture_t �I�u�W�F�N�g��ʂ��Ď擾
        // k4a_image_t �͉摜�f�[�^�Ɗ֘A���郁�^�f�[�^���Ǘ�����
    k4a_capture_t capture;              // kinect�̃L���v�`���n���h��
                                        // �قړ����Ƀf�o�C�X�ŋL�^����Color�CDepth�CIr�Ȃǂ̈�A�̃f�[�^��\���D

        // �J���[�C���[�W
    k4a_image_t color_image_handle;     // �L���v�`���̃J���[�Z���T�̃n���h��

    int32_t color_image_height;         // �J���[�C���[�W�̍���
    int32_t color_image_width;          // �J���[�C���[�W�̕�
    uint8_t* color_image_buffer;        // �J���[�C���[�W�̃f�[�^�̃|�C���^

        // �f�v�X�C���[�W
    k4a_image_t depth_image_handle;     // �L���v�`���̃f�v�X�Z���T�̃n���h��

    int32_t depth_image_height = 0;         // �f�v�X�C���[�W�̍���
    int32_t depth_image_width = 0;          // �f�v�X�C���[�W�̕�
    uint16_t* depth_image_buffer = 0;        // �f�v�X�C���[�W�̃f�[�^�̃|�C���^


        // �v���Ώۂ̏㉺���E�̒[�̐[�x���i�[
    int32_t depth_data_point_left;          // �v���Ώۂ̍��[�̐[�x
    int32_t depth_data_point_right;         // �v���Ώۂ̉E�[�̐[�x
    int32_t depth_data_point_upper;         // �v���Ώۂ̏�[�̐[�x
    int32_t depth_data_point_lower;         // �v���Ώۂ̉��[�̐[�x

        // �f�v�X�摜�̂��ꂼ��̓_�̐[�x
    int32_t depthimage_center_point;        // �f�v�X�摜�̒����̐[�x
    int32_t depthimage_left_point;          // �f�v�X�摜�̍��[�̐[�x
    int32_t depthimage_right_point;         // �f�v�X�摜�̉E�[�̐[�x
    int32_t depthimage_upper_point;         // �f�v�X�摜�̏�[�̐[�x
    int32_t depthimage_lower_point;         // �f�v�X�摜�̉��[�̐[�x

    double angle_x;                         // ��������v���Ώۂ̒��S�܂ł�x���̊p�x
    cv::Point3d measure_target_coord;        // �v���Ώۂ�3�������W [mm]

    cv::Point2i depth_coord_center;         // �v���Ώۂ̒��S���W [pixcel]

    double depth_data_center_5x5;           // �v���Ώے��S�[�x 25�}�X�ړ�����

    int32_t key;
    int32_t flag_measure_target_coord = -1;      // 3�������W[mm]��csv�ɋL�^���s���t���O

    const char* filename_mtc = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\a.csv";
    std::ofstream fp_measure_target_coord(filename_mtc);
    if (!fp_measure_target_coord) {
        throw std::runtime_error("a.csv ���J���܂���ł���");
    }


        // �C���X�^���X�̐��� �������ɃR���X�g���N�^���Ăяo�����
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
                get_color_image_data(&color_image_handle, &color_image_height, &color_image_width, &color_image_buffer);
                
                    // �J���[�Z���T�̃f�[�^��RGBA�摜�ɕϊ�����
                rgbaImg = cv::Mat(color_image_height, color_image_width, CV_8UC4);      //4ch RGBA�摜
                rgbaImg.data = color_image_buffer;
                cv::imshow("rgbaImg",rgbaImg);
            }


                // �f�v�X�C���[�W�̃n���h������[�x�f�[�^�C�����C�����擾����
            if (depth_image_handle) {
                get_depth_image_data(&depth_image_handle, &depth_image_height, &depth_image_width, &depth_image_buffer);

                    // �f�v�X�Z���T�̃f�[�^���O���[�X�P�[���摜�ɕϊ�����
                depthImg = cv::Mat(depth_image_height, depth_image_width, CV_8UC1);
                make_depthImg(&depthImg, depth_image_height, depth_image_width, depth_image_buffer);

                depthrangeImg = cv::Mat(depth_image_height, depth_image_width, CV_8UC1);
                make_depthrangeImg(&depthrangeImg, depth_image_height, depth_image_width, depth_image_buffer);

                    // �f�v�X�摜(�O���[�X�P�[��)����J���[�摜���쐬
                cv::cvtColor(depthImg, depthcoloredImg, cv::COLOR_GRAY2BGR);

                // �ΐ���\��
                for (int x = 0; x < depth_image_width; x++) {
                    depthcoloredImg.at<cv::Vec3b>(288, x)[1] = 255;
                }
                for (int y = 0; y < depth_image_height; y++) {
                    depthcoloredImg.at<cv::Vec3b>(y, 320)[1] = 255;
                    
                }

                //// �f�v�X�摜�̒���, �㉺���E�̐[�x���i�[
                //depthimage_center_point = depth_image_buffer[depth_image_height / 2 * depth_image_width + depth_image_width / 2];
                //depthimage_left_point = depth_image_buffer[depth_image_height / 2 * depth_image_width + 20];
                //depthimage_right_point = depth_image_buffer[depth_image_height / 2 * depth_image_width + 620];
                //depthimage_upper_point = depth_image_buffer[20 * depth_image_width + depth_image_width / 2];
                //depthimage_lower_point = depth_image_buffer[(depth_image_height - 20) * depth_image_width + depth_image_width / 2];
                //// �f�v�X�摜�̒���, �㉺���E�̐[�x�� depthcoloredImg �ɕ\��
                //cv::putText(depthcoloredImg, std::to_string(depthimage_center_point), cv::Point(depth_image_width / 2, depth_image_height / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_left_point), cv::Point(20, depth_image_height / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_right_point), cv::Point(580, depth_image_height / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_upper_point), cv::Point(depth_image_width / 2, 20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_lower_point), cv::Point(depth_image_width / 2, depth_image_height-20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);

                //// �c����\��
                //for (int y = 0; y < depth_image_height; y++) {
                //    depthcoloredImg.at<cv::Vec3b>(y, 20) = cv::Vec3b(255, 0, 0);
                //    depthcoloredImg.at<cv::Vec3b>(y, 620) = cv::Vec3b(255, 0, 0);
                //}

                //for (int x = 0; x < depth_image_width; x++) {
                //    depthcoloredImg.at<cv::Vec3b>(20, x) = cv::Vec3b(255, 0, 0);
                //    depthcoloredImg.at<cv::Vec3b>(depth_image_height - 20, x) = cv::Vec3b(255, 0, 0);
                //}

                // �� �������C���T��
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

                // ������T��
                if (scan_line > 0) {
                    depth_data_point_left = search_measuretarget_left(depth_image_height, depth_image_width, depth_image_buffer, scan_line);
                    if (depth_data_point_left != -1) {
                        // ���� �Ԑ���\��
                        for (int y = 0; y < depth_image_height; y++) {
                            depthcoloredImg.at<cv::Vec3b>(y, depth_data_point_left) = cv::Vec3b(0, 0, 255);
                        }
                        // �E����T��
                        depth_data_point_right = search_measuretarget_right(depth_image_height, depth_image_width, depth_image_buffer, depth_data_point_left, scan_line);
                        if (depth_data_point_right != -1) {
                            for (int y = 0; y < depth_image_height; y++) {
                                depthcoloredImg.at<cv::Vec3b>(y, depth_data_point_right) = cv::Vec3b(0, 0, 255);
                            }
                            // �v���Ώۂ̍��E�̒��Ԑ���\��
                            for (int y = 0; y < depth_image_height; y++) {
                                depthcoloredImg.at<cv::Vec3b>(y, (depth_data_point_right + depth_data_point_left) / 2) = cv::Vec3b(255, 0, 0);
                            }
                            // �㑤��T��
                            depth_data_point_upper = search_measuretarget_upper(depth_image_height, depth_image_width, depth_image_buffer, depth_data_point_left, depth_data_point_right);
                            if (depth_data_point_upper != -1) {
                                for (int x = 0; x < depth_image_width; x++) {
                                    depthcoloredImg.at<cv::Vec3b>(depth_data_point_upper, x) = cv::Vec3b(0, 0, 255);
                                }
                                // ������T��
                                depth_data_point_lower = search_measuretarget_lower(depth_image_height, depth_image_width, depth_image_buffer, depth_data_point_left, depth_data_point_right, depth_data_point_upper);
                                if (depth_data_point_lower != -1) {
                                    for (int x = 0; x < depth_image_width; x++) {
                                        depthcoloredImg.at<cv::Vec3b>(depth_data_point_lower, x) = cv::Vec3b(0, 0, 255);
                                    }
                                    // �v���Ώۂ̏㉺�̒��Ԑ���\��
                                    for (int x = 0; x < depth_image_width; x++) {
                                        depthcoloredImg.at<cv::Vec3b>((depth_data_point_upper + depth_data_point_lower) / 2, x) = cv::Vec3b(255, 0, 0);
                                    }
                                }
                                else {
                                    std::cout << "�������o�ł��Ȃ�" << std::endl;
                                }
                            }
                            else {
                                std::cout << "�㑤���o�ł��Ȃ�" << std::endl;
                            }
                        }
                        else {
                            std::cout << "�E�����o�ł��Ȃ�" << std::endl;
                        }
                    }
                    else {
                        std::cout << "�������o�ł��Ȃ�" << std::endl;
                    }
                }



                // �v���Ώۂ̒��S���W���i�[
                depth_coord_center.x = (depth_data_point_right + depth_data_point_left) / 2;
                depth_coord_center.y = (depth_data_point_upper + depth_data_point_lower) / 2;
                // �\��
                depthcoloredImg.at<cv::Vec3b>(depth_coord_center.y, depth_coord_center.x) = cv::Vec3b(0, 255, 255);


                // �v���Ώۂ̒��S���W�̐[�x�� measure_target_coord�Ɋi�[
                //measure_target_coord.z = depth_image_buffer[depth_coord_center.y * depth_image_width + depth_coord_center.x];


                // �ړ�����
                depth_data_center_5x5 = 0.0;
                for (int x = depth_coord_center.x - 2; x <= depth_coord_center.x + 2; x++) {
                    for (int y = depth_coord_center.y - 2; y <= depth_coord_center.y + 2; y++) {
                        depth_data_center_5x5 += depth_image_buffer[y * depth_image_width + x] / 25.0;
                    }
                }
                measure_target_coord.z = depth_data_center_5x5;


                // �J�������S����v���Ώۂ̒��S�̊p�x�����߂�(x���W)
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
                    std::cout << "3�����f�[�^�L�^�I��" << std::endl;
                    flag_measure_target_coord = -1;
                }

            }
            cv::waitKey(1);


            key = cv::waitKey(1);
            if (key == 'q') {
                break;      // ���C�����[�v������
            }

            if (key == 's') {
                std::cout << "get_depth_surface_to_csv �J�n" << std::endl;
                get_depth_surface_to_csv(depth_image_height, depth_image_width, depth_image_buffer);
                std::cout << "get_depth_surface_to_csv �I��" << std::endl;
            }

            if (key == 'p' && flag_measure_target_coord == -1) {
                std::cout << "3�����f�[�^�L�^�J�n" << std::endl;
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


