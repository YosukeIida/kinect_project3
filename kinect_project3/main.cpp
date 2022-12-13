
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
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>


#include <opencv2/opencv.hpp>       // opencv347
#include <k4a/k4a.h>                // azure kinect sdk

#include "kinectdevice.h"     // ����w�b�_

#define DEPTH_WIDTH         640             // �f�v�X�Z���T NOFV Unbinned�̉���
#define DEPTH_HEIGHT        576             // �f�v�X�Z���T NFOV Unbinned�̏c��
#define COLOR_WIDTH         1920            // �J���[�Z���T�̉���
#define COLOR_HEIGHT        1080            // �J���[�Z���T�̏c��

#define DEPTH_SEARCH_BORDER  710            // �v���Ώۂ�T�����鋫�E�l ���̒l����O��T������
#define DEPTH_IMAGE_FAR_LIMIT   750
#define DEPTH_IMAGE_NEAR_LIMIT  650         // �O���[�X�P�[���摜�ɂ���ŏ�����
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


//�֐��錾
void get_color_image_data(k4a_image_t* color_image_handle, uint8_t** color_image_buffer);
void get_depth_image_data(k4a_image_t* depth_image_handle, uint16_t** depth_image_buffer);
void set_distance(std::vector<std::vector<int32_t> >&depthdata, uint16_t * depth_image_buffer);
void make_depthImg(cv::Mat* depthImg, uint16_t* depth_image_buffer);
void make_depthImg_2darray(cv::Mat* depthImgtemp, std::vector<std::vector<int32_t> > depthdata);
void make_depthrangeImg(cv::Mat* depthrangeImg, std::vector<std::vector<int32_t> > depthdata);





// capture ���� color_image���擾
void get_color_image_data(k4a_image_t* color_image_handle, uint8_t** color_image_buffer) {

    *color_image_buffer = k4a_image_get_buffer(*color_image_handle);
}


// capture ���� depth_image���擾
void get_depth_image_data(k4a_image_t* depth_image_handle, uint16_t** depth_image_buffer) {
    
    *depth_image_buffer = (uint16_t*)k4a_image_get_buffer(*depth_image_handle);
}


// depth_image_buffer �̐[�x�f�[�^����2����vector�z����쐬  (2�����z��ɂ���)
void set_distance(std::vector<std::vector<int32_t> >& depthdata, uint16_t* depth_image_buffer) {
    int ptr = 0;
    for (int y = 0; y < DEPTH_HEIGHT; y++) {
        for (int x = 0; x < DEPTH_WIDTH; x++) {
            depthdata[x][y] = depth_image_buffer[ptr];
            ptr++;
        }
    }
}


// �O���[�X�P�[���̃f�v�X�摜���쐬
// depth_image_buffer �����X�^����� depthImg.data �ɐ[�x�f�[�^�����Ă����D
void make_depthImg(cv::Mat* depthImg, uint16_t* depth_image_buffer) {
    for (int y = 0; y < DEPTH_HEIGHT; y++) {
        for (int x = 0; x < DEPTH_WIDTH; x++) {
            int address = y * DEPTH_WIDTH + x;

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


// �O���[�X�P�[���̃f�v�X�摜���쐬
// 2�����z�񂩂�depthImg���쐬����
void make_depthImg_2darray(cv::Mat* depthImgtemp, std::vector<std::vector<int32_t> > depthdata) {
    for (int y = 0; y < DEPTH_HEIGHT; y++) {
        for (int x = 0; x < DEPTH_WIDTH; x++) {

            // �O���[�X�P�[���摜�쐬 350mm��(255), 605mm����(0)��255�i�K
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


// �O���[�X�P�[���̃f�v�X�摜���쐬
// DEPTH_IMAGE_NEAR_LIMIT ���� DEPTH_IMAGE_FAR_LIMIT �܂ł�255�i�K�ŕ\��
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



////  while���[�v depth_image 1�L���v�`������ buffer�̃f�[�^��csv�ɏ�������
//void get_depth_surface_to_csv(uint16_t* depth_image_buffer) {
//    const char* filename = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\depth_surface.csv";
//    //const char* filename = "C:\\Users\\i1811402\\cpp_program\\kinect_project3\\data\\depth_surface.csv";
//    std::ofstream file_depth_surface(filename);
//    if (!file_depth_surface) {
//        throw std::runtime_error("depth_surface.csv ���J���܂���ł���");
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
        // �v���Ώۂ�0mm�ɂȂ镔���̓��������o����
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
    double z_distance;      // ����
    double x_max_distance;       // ������distance�̎��̍ő压��(mm)
    double x_distance_pixcel;
    
    double y_max_distance;
    double y_distance_pixcel;
    z_distance = measure_target_coord.z;

    // x����
    x_max_distance = sin(NFOV_FOI_HOR) / cos(NFOV_FOI_HOR) * z_distance;
    x_distance_pixcel = depth_coord_center.x - (double)X_CENTER_COORD;
    measure_target_coord.x = x_distance_pixcel * x_max_distance / (double)(X_CENTER_COORD);

    // y����
    y_max_distance = sin(NFOV_FOI_VERT) / cos(NFOV_FOI_VERT) * z_distance;
    y_distance_pixcel = depth_coord_center.y - (double)Y_CENTER_COORD;
    measure_target_coord.y = y_distance_pixcel * -1.0 * y_max_distance / (double)Y_CENTER_COORD; // �X�N���[�����W�ƃ��[���h���W���t�ɂ���
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

    uint8_t* color_image_buffer;        // �J���[�C���[�W�̃f�[�^�̃|�C���^

        // �f�v�X�C���[�W
    k4a_image_t depth_image_handle;     // �L���v�`���̃f�v�X�Z���T�̃n���h��

    uint16_t* depth_image_buffer = 0;        // �f�v�X�C���[�W�̃f�[�^�̃|�C���^

    std::vector<std::vector< int32_t > > depthdata(DEPTH_WIDTH, std::vector<int32_t>(DEPTH_HEIGHT));


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

    cv::Point2d depth_coord_center;         // �v���Ώۂ̒��S���W [pixcel]

    double depth_data_center_5x5;           // �v���Ώے��S�[�x 25�}�X�ړ�����

    bool flg_measureblock = false;          // �R�����ʒu�v���u���b�N�����s����t���O

    const char* filename_mtc = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\a.csv";
    //const char* filename_mtc = "C:\\Users\\yosuk\\cpp_program\\kinect_project3\\data\\a.csv";
    std::ofstream fp_measure_target_coord(filename_mtc);
    if (!fp_measure_target_coord) {
        throw std::runtime_error("a.csv ���J���܂���ł���");
    }
    KinectDevice kinectdevice;




        // try�u���b�N�̒��ŗ�O������ throw() �ŋL�q����
        // ��O�����������ꍇtry�u���b�N�̉��ɂ���catch�u���b�N�����s�����
    try
    {
        // ���C�����[�v
        while (true) {



            // �C���X�^���X�̐��� �������ɃR���X�g���N�^���Ăяo�����
            // �R���X�g���N�^�Ńf�o�C�X�̃I�[�v��, �J�����\���ݒ�, �J�����̃X�^�[�g���s��

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


            if (color_image_handle) {
                // �J���[�C���[�W�̃n���h������摜�̃f�[�^�C�����C�����擾����
                get_color_image_data(&color_image_handle, &color_image_buffer);

                // �J���[�Z���T�̃f�[�^��RGBA�摜�ɕϊ�����
                rgbaImg = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);      //4ch RGBA�摜
                rgbaImg.data = color_image_buffer;
                cv::imshow("rgbaImg", rgbaImg);
            }


            if (depth_image_handle) {
                // �f�v�X�C���[�W�̃n���h������[�x�f�[�^�C�����C�����擾����
                get_depth_image_data(&depth_image_handle, &depth_image_buffer);

                // uint8_t* depth_image_buffer�̐[�x�f�[�^�� vector<vector<int32_t>> depthdata��2�����z��Ɋi�[����
                set_distance(depthdata, depth_image_buffer);

                // �f�v�X�Z���T�̃f�[�^���O���[�X�P�[���摜�ɕϊ�����
                depthImg = cv::Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
                // make_depthImg(&depthImg, depth_image_buffer);
                make_depthImg_2darray(&depthImg, depthdata);
                

                depthrangeImg = cv::Mat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
                make_depthrangeImg(&depthrangeImg, depthdata);

                // �f�v�X�摜(�O���[�X�P�[��)����J���[�摜���쐬
                cv::cvtColor(depthImg, depthcoloredImg, cv::COLOR_GRAY2BGR);


                // �c���̒���(x=320, y=288)�ɗΐ���\�� (�����ɂ͒������̉E�C���̃s�N�Z���ɐ���`��)
                for (int x = 0; x < DEPTH_WIDTH; x++) {
                    depthcoloredImg.at<cv::Vec3b>(288, x)[1] = 255;
                }
                for (int y = 0; y < DEPTH_HEIGHT; y++) {
                    depthcoloredImg.at<cv::Vec3b>(y, 320)[1] = 255;

                }

                //// �f�v�X�摜�̒���, �㉺���E�̐[�x���i�[
                //depthimage_center_point = depth_image_buffer[DEPTH_HEIGHT / 2 * DEPTH_WIDTH + DEPTH_WIDTH / 2];
                //depthimage_left_point = depth_image_buffer[DEPTH_HEIGHT / 2 * DEPTH_WIDTH + 20];
                //depthimage_right_point = depth_image_buffer[DEPTH_HEIGHT / 2 * DEPTH_WIDTH + 620];
                //depthimage_upper_point = depth_image_buffer[20 * DEPTH_WIDTH + DEPTH_WIDTH / 2];
                //depthimage_lower_point = depth_image_buffer[(DEPTH_HEIGHT - 20) * DEPTH_WIDTH + DEPTH_WIDTH / 2];
                //// �f�v�X�摜�̒���, �㉺���E�̐[�x�� depthcoloredImg �ɕ\��
                //cv::putText(depthcoloredImg, std::to_string(depthimage_center_point), cv::Point(DEPTH_WIDTH / 2, DEPTH_HEIGHT / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_left_point), cv::Point(20, DEPTH_HEIGHT / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_right_point), cv::Point(580, DEPTH_HEIGHT / 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_upper_point), cv::Point(DEPTH_WIDTH / 2, 20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);
                //cv::putText(depthcoloredImg, std::to_string(depthimage_lower_point), cv::Point(DEPTH_WIDTH / 2, DEPTH_HEIGHT-20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 255), 2, CV_AA);

                //// �c����\��
                //for (int y = 0; y < DEPTH_HEIGHT; y++) {
                //    depthcoloredImg.at<cv::Vec3b>(y, 20) = cv::Vec3b(255, 0, 0);
                //    depthcoloredImg.at<cv::Vec3b>(y, 620) = cv::Vec3b(255, 0, 0);
                //}

                //for (int x = 0; x < DEPTH_WIDTH; x++) {
                //    depthcoloredImg.at<cv::Vec3b>(20, x) = cv::Vec3b(255, 0, 0);
                //    depthcoloredImg.at<cv::Vec3b>(DEPTH_HEIGHT - 20, x) = cv::Vec3b(255, 0, 0);
                //}

                // 3�����ʒu�v�����[�v
                if (flg_measure == true) {

                    //// �� �������C���T��
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


                    // Kinect�𗧂ĂČv������Ƃ��͂�������

                    // �� �������C���T��
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

                    // ������T��
                    if (scan_line_row > 0 && scan_line_row < DEPTH_HEIGHT) {
                        depth_data_point_left = search_measuretarget_left(depthdata, scan_line_row);
                        if (depth_data_point_left != -1) {
                            // ���� �Ԑ���\��
                            for (int y = 0; y < DEPTH_HEIGHT; y++) {
                                depthcoloredImg.at<cv::Vec3b>(y, depth_data_point_left) = cv::Vec3b(0, 0, 255);
                            }
                            // �E����T��
                            depth_data_point_right = search_measuretarget_right(depthdata, scan_line_row, depth_data_point_left);
                            if (depth_data_point_right != -1) {
                                for (int y = 0; y < DEPTH_HEIGHT; y++) {
                                    depthcoloredImg.at<cv::Vec3b>(y, depth_data_point_right) = cv::Vec3b(0, 0, 255);
                                }
                                // �v���Ώۂ̍��E�̒��Ԑ���\��
                                for (int y = 0; y < DEPTH_HEIGHT; y++) {
                                    depthcoloredImg.at<cv::Vec3b>(y, (depth_data_point_right + depth_data_point_left) / 2) = cv::Vec3b(255, 0, 0);
                                }
                                int scan_line_col = (int)(depth_data_point_left + depth_data_point_right) / 2;
                                // �㑤��T��
                                depth_data_point_upper = search_measuretarget_upper(depthdata, scan_line_col);
                                if (depth_data_point_upper != -1) {
                                    for (int x = 0; x < DEPTH_WIDTH; x++) {
                                       // depthcoloredImg.at<cv::Vec3b>(depth_data_point_upper, x) = cv::Vec3b(0, 0, 255);
                                    }
                                    // ������T��
                                    depth_data_point_lower = search_measuretarget_lower(depthdata, scan_line_col, depth_data_point_upper);
                                    if (depth_data_point_lower != -1) {
                                        for (int x = 0; x < DEPTH_WIDTH; x++) {
                                            depthcoloredImg.at<cv::Vec3b>(depth_data_point_lower, x) = cv::Vec3b(0, 0, 255);
                                        }
                                        // �v���Ώۂ̏㉺�̒��Ԑ���\��
                                        for (int x = 0; x < DEPTH_WIDTH; x++) {
                                            depthcoloredImg.at<cv::Vec3b>((depth_data_point_upper + depth_data_point_lower) / 2, x) = cv::Vec3b(255, 0, 0);
                                        }



                                        // �v���Ώۂ̒��S���W���i�[
                                        depth_coord_center.x = (depth_data_point_right + depth_data_point_left) / 2.0;
                                        depth_coord_center.y = (depth_data_point_upper + depth_data_point_lower) / 2.0;
                                        // �\��
                                        depthcoloredImg.at<cv::Vec3b>(depth_coord_center.y, depth_coord_center.x) = cv::Vec3b(0, 255, 255);


                                        // �v���Ώۂ̒��S���W�̐[�x�� measure_target_coord�Ɋi�[
                                        //measure_target_coord.z = depth_image_buffer[depth_coord_center.y * DEPTH_WIDTH + depth_coord_center.x];


                                        // �ړ�����
                                        depth_data_center_5x5 = 0.0;
                                        for (int y = depth_coord_center.y - 2; y <= depth_coord_center.y + 2; y++) {
                                            for (int x = depth_coord_center.x - 2; x <= depth_coord_center.x + 2; x++) {
                                                depth_data_center_5x5 += depthdata[x][y] / 25.0;
                                            }
                                        }
                                        measure_target_coord.z = depth_data_center_5x5;


                                        // �J�������S����v���Ώۂ̒��S�̊p�x�����߂�(x���W)
                                        //angle_x = ((depth_coord_center.x - X_CENTER_COORD) / (double)X_CENTER_COORD) * NFOV_FOI_HOR / 2.0;
                                        //measure_target_coord.x = ((depth_coord_center.x - X_CENTER_COORD) * measure_target_coord.z * tan(NFOV_FOI_HOR/2.0/180.0*M_PI)) / X_CENTER_COORD;
                                        measure_target_coord = depth2world(measure_target_coord, depth_coord_center);
                                        std::cout << std::setw(10) <<std::setprecision(5) << measure_target_coord.x << "\t" << measure_target_coord.y << "\t" << measure_target_coord.z << std::endl;



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

                }   // �R�����ʒu�v��  if (flg_measure)


                cv::imshow("depthcoloredImg", depthcoloredImg);
                cv::imshow("depthrangeImg", depthrangeImg);
                cv::imshow("depth image", depthImg);

                

                if (flag_measure_target_coord != -1 && flag_measure_target_coord < 100) {
                    fp_measure_target_coord << measure_target_coord.x << "," << measure_target_coord.y << "," << measure_target_coord.z << "," << depth_coord_center.x << "," << depth_coord_center.y << std::endl;
                    flag_measure_target_coord++;
                }
                if (flag_measure_target_coord == 100) {
                    std::cout << "3�����f�[�^�L�^�I��" << std::endl;
                    flag_measure_target_coord = -1;
                }
                k4a_image_release(color_image_handle);
                k4a_image_release(depth_image_handle);
                k4a_capture_release(capture);

            }   // depthimage��if��
            cv::Mat image = cv::Mat::zeros(500, 500, CV_8UC3);

            int cols = image.cols;
            int rows = image.rows;
            for (int j = 0; j < rows; j++) {
                for (int i = 0; i < cols; i++) {
                    image.at<cv::Vec3b>(j, i)[0] = 255; //��
                    image.at<cv::Vec3b>(j, i)[1] = 255; //��
                    image.at<cv::Vec3b>(j, i)[2] = 255; //��
                }
            }

            cv::imshow("image", image);


            int key = cv::waitKey(1);
            if (key == 'q') {
                break;      // 
            }

            //if (key == 's') {
            //    std::cout << "get_depth_surface_to_csv �J�n" << std::endl;
            //    get_depth_surface_to_csv(depth_image_buffer);
            //    std::cout << "get_depth_surface_to_csv �I��" << std::endl;
            //}

            if (key == 'p' && flag_measure_target_coord == -1) {
                std::cout << "3�����f�[�^�L�^�J�n" << std::endl;
                flag_measure_target_coord = 0;
            }
        }   // ���C�����[�v
    }
    catch (const std::exception &ex)
    {
        std::cout << ex.what() << std::endl;
    }
    return 0;
}


