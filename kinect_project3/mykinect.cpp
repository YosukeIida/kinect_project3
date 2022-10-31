//--------------------------------------------------------------------
//  2022/09/27�쐬
//  �R���X�g���N�^( kinect�̃I�[�v��, kinect�̃J�����ݒ�, �J�����̋N��)
//  �f�X�g���N�^( �J�����̃X�g�b�v, kinect�̃N���[�Y)
//
//
//--------------------------------------------------------------------

#include <iostream>
#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>

#include "mykinect.h"         // ����w�b�_

// �R���X�g���N�^
KinectDevice::KinectDevice():
    device(NULL),
    device_configuration(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL),
    serial_size(0)
{
    // �f�o�C�X�̐ڑ��̗L���A�ڑ����𒲂ׂ�
    device_count = k4a_device_get_installed_count();
    if (device_count == 0) {
        throw std::runtime_error("Kinect���ڑ�����Ă��܂���");
    }

    // Azure Kinect�f�o�C�X�̃I�[�v��
        // "K4A_DEVICE_DEFAULT" �ōŏ��̃f�o�C�X���J��
        // �I������"k4a_device_close()" ���Ăяo��   
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device))) {
        throw std::runtime_error("�f�o�C�X���I�[�v���ł��܂���");
    }

    // �V���A���i���o�[�̎擾
        // �ŏ��� "serial_number = NULL" �ŌĂяo���A�V���A���i���o�[�ɕK�v�ȃo�b�t�@�T�C�Y���擾����
        // 2��ڂ̌Ăяo���Ċ��S�ȃV���A���i���o�[��Ԃ�
    k4a_device_get_serialnum(device, NULL, &serial_size);
    serial_number = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial_number, &serial_size);
    std::cout << "Opened device:" << serial_number << std::endl;
    free(serial_number);

    // �J�����\���̐ݒ�
    device_configuration.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;    // �J���[�f�[�^��BGRA32bit
    device_configuration.color_resolution = K4A_COLOR_RESOLUTION_1080P;       // �掿1080p 1920x1080(width, height)
    device_configuration.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;     // 640x576(width, height)
    device_configuration.camera_fps = K4A_FRAMES_PER_SECOND_30;         // 30fps
    device_configuration.synchronized_images_only = true;
    device_configuration.depth_delay_off_color_usec = 0;
    device_configuration.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    device_configuration.subordinate_delay_off_master_usec = 0;
    device_configuration.disable_streaming_indicator = true;

    // �J�������N������
        // �I������ "k4a_device_stop_cameras()" ���Ăяo��
    if (K4A_FAILED(k4a_device_start_cameras(device, &device_configuration))) {
        throw std::runtime_error("�J�������N���ł��܂���");
    }
}

// �f�X�g���N�^
KinectDevice::~KinectDevice()
{
    k4a_device_stop_cameras(device);
    std::cout << "stop device" << std::endl;
    k4a_device_close(device);
    std::cout << "close device" << std::endl;
}



