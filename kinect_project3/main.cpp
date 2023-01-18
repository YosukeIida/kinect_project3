
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


//  �v���O�������s���̃L�[����
//  "q" �v���O�����I��
//  "d" �f�v�X�J�����L�^�J�n  100�t���[���v����v���O�����I��
//  "c" �J���[�J�����L�^�J�n  100�t���[���v����v���O�����I��
//  "m" �����L�^
//  



#include <iostream>
#include <vector>
#include <string>       // to_string()�Ŏg�p
#include <fstream>      // ofstream()�Ŏg�p
#include <iomanip>      // ���������_���̕\�����@

#define _USE_MATH_DEFINES
#include <math.h>


#include <opencv2/opencv.hpp>       // opencv347
#include <opencv2/aruco.hpp>        // opencv contribution aruco
#include <k4a/k4a.h>                // azure kinect sdk

#include "kinectdevice.h"     // ����w�b�_

#define DEPTH_WIDTH         640             // �f�v�X�Z���T NOFV Unbinned�̉���
#define DEPTH_HEIGHT        576             // �f�v�X�Z���T NFOV Unbinned�̏c��
#define COLOR_WIDTH         1920            // �J���[�Z���T�̉���
#define COLOR_HEIGHT        1080            // �J���[�Z���T�̏c��

#define MARKER_LENGTH       0.0180           // ArUco�}�[�J�[1�ӂ̒��� [m]

#define DEPTH_SEARCH_BORDER  520            // �v���Ώۂ�T�����鋫�E�l ���̒l����O��T������
#define DEPTH_IMAGE_FAR_LIMIT   550
#define DEPTH_IMAGE_NEAR_LIMIT  450         // �O���[�X�P�[���摜�ɂ���ŏ�����
         // �O���[�X�P�[���摜�ɂ���ő勗��
#define X_CENTER_COORD  320                 // NFOV Unbinned�̉��̒������W
#define Y_CENTER_COORD  288                 // NFOV Unbinned�̏c�̒������W
#define NFOV_FOI_HOR  (75.0 / 2.0) / 180.0 * (double)M_PI                  // NFOV Unbinned�̐�������p
#define NFOV_FOI_VERT  (65.0 / 2.0) / 180.0 * (double)M_PI                  // NFOV Unbinned�̐�������p


//  Kinect�𗧂ĂĎB�e���鎞�ɐ؂�ւ���D
#define KINECT_ATTITUDE     0               // �������� : 0,    �������� : 1



//  Visual C++�ŃR���p�C������Ƃ��Ƀ����N���郉�C�u�����t�@�C��
#pragma comment(lib, "k4a.lib")
#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_videoio347.lib")
#pragma comment(lib, "opencv_highgui347.lib")
#pragma comment(lib, "opencv_aruco347.lib")


//�֐��錾
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

    // �\������摜�pMat�z��
    cv::Mat rgbaImg;                    // �J���[�Z���T�̃C���[�W�n���h���̉摜�f�[�^��rgba�摜�ɕϊ����ĕ\��
    cv::Mat colorImg;                   // aruco�Ŏg�p 4chRGBA -> 3chRGB

    cv::Mat depthImg;                   // �f�v�X�Z���T�̃C���[�W�n���h���̃f�v�X�f�[�^���O���[�X�P�[���ɕϊ����ĕ\��
    cv::Mat depthrangeImg;
    cv::Mat depthcoloredImg;            // depthImg���J���[�摜�ɕϊ����ċ��E�Ȃǂ�`�悵�ĕ\��


    // �J���[�J�����s�� (�J���������s��)
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
    // �J�����s�� (�c�� distortion)
    cv::Mat color_camera_dist_coeffs(1, 5, CV_64FC1);
    color_camera_dist_coeffs.at<double>(0, 0) = 0.46717128157615661621;          // k1
    color_camera_dist_coeffs.at<double>(0, 1) = -2.45866727828979492188;         // k2
    color_camera_dist_coeffs.at<double>(0, 2) = 0.00136364088393747807;          // p1
    color_camera_dist_coeffs.at<double>(0, 3) = -0.00006751885666744784;          // p2
    color_camera_dist_coeffs.at<double>(0, 4) = 1.37056386470794677734;         // k3


    // �f�v�X�J�����s�� (�J���������s��)
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
    // �J�����s�� (�c�� distortion)
    cv::Mat depth_camera_dist_coeffs(1, 5, CV_64FC1);
    depth_camera_dist_coeffs.at<double>(0, 0) = 2.48467683792114257812;          // k1
    depth_camera_dist_coeffs.at<double>(0, 1) = 1.70563745498657226562;         // k2
    depth_camera_dist_coeffs.at<double>(0, 2) = -0.00007825181819498539;          // p1
    depth_camera_dist_coeffs.at<double>(0, 3) = 0.00002669491004780866;          // p2
    depth_camera_dist_coeffs.at<double>(0, 4) = 0.09044291824102401733;         // k3



    // Create Marker Dictionary, Type of marker : 4x4, 1000
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_1000;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    // ���o�����}�[�J�[��ID���i�[����x�N�^�[
    std::vector<int> marker_ids;

    //���o�����}�[�J�[�̃R�[�i�[���W�ƃ��W�F�N�g���W���i�[����
    std::vector<std::vector<cv::Point2f>> marker_corners, rejectedCandidates;

    int sum_ids = 0;
    std::vector<cv::Vec3d> rvecs, tvecs;        // aruco�}�[�J�[��tvec, rvec

    cv::Point3d aruco_coord;


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

    //double angle_x;                         // ��������v���Ώۂ̒��S�܂ł�x���̊p�x
    cv::Point3d measure_target_coord;        // �v���Ώۂ�3�������W [mm]

    cv::Point2d depth_coord_center;         // �v���Ώۂ̒��S���W [pixcel]

    double depth_data_center_5x5;           // �v���Ώے��S�[�x 25�}�X�ړ�����

    int key;
    int32_t flag_depth_measure_target_coord = -1;      // 3�������W[mm]��csv�ɋL�^���s���t���O
    int32_t flag_aruco_coord_write = -1;    // aruco�}�[�J�[��3�������W[mm]��csv�ɏ������ރt���O

    bool flag_close = false;






    const char* filename_depth = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\depth.csv";
    //const char* filename_depth = "C:\\Users\\yosuk\\cpp_program\\kinect_project3\\data\\depth.csv";
    std::ofstream fp_depth_measure_target_coord(filename_depth);
    if (!fp_depth_measure_target_coord) {
        throw std::runtime_error("depth.csv ���J���܂���ł���");
    }

    const char * filename_aruco = "C:\\Users\\student\\cpp_program\\kinect_project3\\data\\aruco.csv";
    //const char* filename_depth = "C:\\Users\\yosuk\\cpp_program\\kinect_project3\\data\\aruco.csv";
    std::ofstream fp_aruco_measure_target_coord(filename_aruco);
    if (!fp_aruco_measure_target_coord) {
        throw std::runtime_error("aruco.csv ���J���܂���ł���");
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



        // try�u���b�N�̒��ŗ�O������ throw() �ŋL�q����
        // ��O�����������ꍇtry�u���b�N�̉��ɂ���catch�u���b�N�����s�����
    try
    {
        // �������[�v
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
                //cv::imshow("rgbaImg", rgbaImg);

                colorImg = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC3);     // 3ch RGB�摜
                cv::cvtColor(rgbaImg, colorImg, CV_RGBA2RGB);

                cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
                // �T�u�s�N�Z����ON
                parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

                // aruco�}�[�J�����o����
                cv::aruco::detectMarkers(colorImg, dictionary, marker_corners, marker_ids, parameters, rejectedCandidates);

                //// cv::outputarray ���g���� vector<int> �� vector<cv::Mat>�ɕϊ����ĕ\��
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

                    // ���o�����}�[�J������
                    cv::aruco::drawDetectedMarkers(colorImg, marker_corners, marker_ids);

                    // �}�[�J��rvec, tvec�����߂�
                    cv::aruco::estimatePoseSingleMarkers(marker_corners, MARKER_LENGTH, color_camera_matrix, color_camera_dist_coeffs, rvecs, tvecs);
                    for (int i = 0; i < marker_ids.size(); i++) {
                        std::cout << "tvecs:" << tvecs[i] * 1000 << std::endl;      // �P��[mm]
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

                int scan_line_upper = -1;
                int scan_line_lower = -1;
                if (KINECT_ATTITUDE == 0) {
                    // �� �������C���T��
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
                    // �� �������C���T��
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
                break;      // ���C�����[�v������
            }

            //if (key == 's') {
            //    std::cout << "get_depth_surface_to_csv �J�n" << std::endl;
            //    get_depth_surface_to_csv(depth_image_buffer);
            //    std::cout << "get_depth_surface_to_csv �I��" << std::endl;
            //}

            // 
            if (key == 'm' && flag_depth_measure_target_coord == -1 && flag_aruco_coord_write == -1) {
                std::cout << "\n\n�����L�^�J�n\n\n";
                flag_depth_measure_target_coord = 0;
                flag_aruco_coord_write = 0;
            }

            if (key == 'd' && flag_depth_measure_target_coord == -1) {
                std::cout << "\n\n3�����f�[�^�L�^�J�n\n\n" << std::endl;
                flag_depth_measure_target_coord = 0;
            }

            if (flag_depth_measure_target_coord != -1 && flag_depth_measure_target_coord < 100) {
                fp_depth_measure_target_coord << std::fixed << std::setprecision(10) << measure_target_coord.x << "," << measure_target_coord.y << "," << measure_target_coord.z << "," << depth_coord_center.x << "," << depth_coord_center.y << std::endl;
                flag_depth_measure_target_coord++;
            }
            if (flag_depth_measure_target_coord == 100) {
                std::cout << "\n\n3�����f�[�^�L�^�I��\n\n" << std::endl;
                flag_depth_measure_target_coord = -1;
                flag_close = true;
            }

            if (key == 'c' && flag_aruco_coord_write == -1) {
                std::cout << "\n\nArUco�L�^�J�n\n\n" << std::endl;
                flag_aruco_coord_write = 0;
            }
            if (flag_aruco_coord_write != -1 && flag_aruco_coord_write < 100) {
                fp_aruco_measure_target_coord << std::fixed << std::setprecision(10) << aruco_coord.x << "," << -aruco_coord.y << "," << aruco_coord.z << std::endl;
                flag_aruco_coord_write++;
            }
            if (flag_aruco_coord_write == 100) {
                std::cout << "\n\nArUco�L�^�I��\n\n" << std::endl;
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


// �f�v�X�摜����v���Ώۂ̍�����T���C�����̍��W��Ԃ�
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


// �f�v�X�摜����v���Ώۂ̉E����T���C�E���̍��W��Ԃ�
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


// �f�v�X�摜����v���Ώۂ̏㑤��T���C�㑤�̍��W��Ԃ�
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


// �f�v�X�摜����v���Ώۂ̉�����T���C�����̍��W��Ԃ�
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


// �f�v�X�摜����v���Ώۂ�xy�����̍��W���v�Z���ĕԂ�
cv::Point3d depth2world(cv::Point3d measure_target_coord, cv::Point2d depth_coord_center) {
    double z_distance;      // ����
    double x_max_distance;       // ������distance�̎��̍ő压��(mm)
    double x_distance_pixcel;

    double y_max_distance;
    double y_distance_pixcel;
    z_distance = measure_target_coord.z;

    // x����
    x_max_distance = sin(NFOV_FOI_HOR) / cos(NFOV_FOI_HOR) * z_distance;
    x_distance_pixcel = depth_coord_center.x - (double)X_CENTER_COORD;
    measure_target_coord.x = x_distance_pixcel * x_max_distance / (double)X_CENTER_COORD;

    // y����
    y_max_distance = sin(NFOV_FOI_VERT) / cos(NFOV_FOI_VERT) * z_distance;
    y_distance_pixcel = depth_coord_center.y - (double)Y_CENTER_COORD;
    measure_target_coord.y = y_distance_pixcel * -1.0 * y_max_distance / (double)Y_CENTER_COORD; // �X�N���[�����W�ƃ��[���h���W���t�ɂ���
    return measure_target_coord;

}

