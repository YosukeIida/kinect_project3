//----------------------------------------------------------------------
//  2022/09/27�쐬
//  ����w�b�_
//  
//  kinect.cpp�ŌĂяo�������o�ϐ��C�����o�֐����L�q
//
//----------------------------------------------------------------------

#pragma once

#include <k4a/k4a.h>
#include <opencv2/opencv.hpp>

// Kinect�f�o�C�X�p�N���X
class KinectDevice {

// �f�o�C�X�ɂ��Ẵ����o�ϐ�
private:
    uint32_t device_count;      // pc�ɐڑ����Ă���kinect�̐�
    size_t serial_size;         // �ڑ����Ă���kinect�̃V���A���i���o�[
    char* serial_number;        // �V���A���i���o�[
    k4a_device_configuration_t device_configuration;   // �f�o�C�X�̃J�������[�h�ݒ�ϐ�
public:
    k4a_device_t device;        // kinect�̃f�o�C�X�n���h�� (main�̒��Œ��ڌĂяo������public)

// �����o�֐�
public:
    // �R���X�g���N�^
    KinectDevice();
    // �f�X�g���N�^
    ~KinectDevice();
};

