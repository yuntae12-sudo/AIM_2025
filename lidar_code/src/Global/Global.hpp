#ifndef GLOBAL_HPP
#define GLOBAL_HPP

// 1. 기본 표준 라이브러리
#include <vector>
#include <string>

// 2. PCL 기본 헤더 (포인트 타입 및 클라우드)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/common/common.h>

// 3. RANSAC 및 필터링 관련 (Ransac.cpp용)
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h> // SAC_RANSAC 정의용
#include <pcl/sample_consensus/model_types.h>

// 4. 클러스터링 및 트리 관련 (Clustering.cpp용)
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// 5. 기타 유틸리티 (Voxel, ROI 등)
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <pcl/filters/crop_box.h>


//칼만필터 행렬 계산용
#include <Eigen/Dense>

struct Point2D {
    double x;
    double y;
};

struct Rectangle {
    Eigen::Vector2d corners[4];
    double heading;
    double score;
};

struct Lbox {
    // 위치 (중심점)
    float f32_X_center;
    float f32_Y_center;
    float f32_Z_center;

    // 크기
    float f32_L; // 길이 (Length)
    float f32_W; // 폭 (Width)
    float f32_H; // 높이 (Height)

    // 방향 및 상태
    float f32_Heading; // 차가 바라보는 각도 (Radian)
    int s32_ID;        // 장애물 식별 번호
    
    // 생성자 (기본값 초기화)
    Lbox() : f32_X_center(0), f32_Y_center(0), f32_Z_center(0), f32_L(0), f32_W(0), f32_H(0), f32_Heading(0), s32_ID(-1) {
    }
};

// [추가] 개별 객체의 추적 상태를 관리하는 구조체
struct Track {
    int s32_ID;
    Eigen::Matrix<float, 6, 1> vec_X; // 상태 변수: [x, y, vx, vy, ax, ay]^T 칼만 예측 
    Eigen::Matrix<float, 6, 6> mat_P; // 오차 공분산 행렬 (추정치의 불확실성) 칼만업데이트
    
    int s32_LostFrames;               // 매칭에 실패한 연속 프레임 수
    uint8_t u8_R, u8_G, u8_B;         // 객체별 고유 색상 (RViz 시각화용)

    // 박스의 기하학적 중심 정보 (IoU 및 최종 출력용) 외부 인터페이스
    float f32_X_center;
    float f32_Y_center;
    float f32_Z_center;

    // 크기 정보 유지 (L-Shape 결과가 튀는 것을 방지하기 위해 추적 중인 크기 저장)
    float f32_L; 
    float f32_W;
    float f32_H;
    float f32_Heading;



    // 생성자: 초기화 및 행렬 크기 설정
    Track() : s32_ID(-1), s32_LostFrames(0), u8_R(255), u8_G(255), u8_B(255),
              f32_X_center(0.0f), f32_Y_center(0.0f), f32_Z_center(0.0f),
              f32_L(0), f32_W(0), f32_H(0), f32_Heading(0) {
        vec_X.setZero();
        mat_P = Eigen::Matrix<float, 6, 6>::Identity() * 1.0f; // 초기 불확실성 설정
    }
};

struct LiDAR {
    // 1. Raw Data: 센서에서 갓 들어온 원본 점구름
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_RawCloud;

    // 2. Preprocessed Data: ROI와 Voxel을 거친 깨끗한 점구름
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_RoiCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_VoxelCloud;

    // 3. Segmented Data: 바닥(Ground)과 물체(Non-ground) 분리 결과
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_GroundCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_NonGroundCloud;

    //클러스터링 결과를 저장할 변수
    std::vector<pcl::PointIndices> cluster_indices;
    // 4. Final Objects: 최종 인식된 물체들의 리스트
    // 우리가 만들었던 ObjectObservation이나 원본의 LiDARCluster 형태
    std::vector<Lbox> vec_Objects;
    // 현재 추적 중인 객체들의 최종 리스트 (ID와 속도가 포함됨)
    std::vector<Track> vec_Tracks;

    // 생성자: 메모리 공간을 미리 만들어둠 , 생성자가 없다면 포인터들은 처음에 아무것도 안가리키는 빈주고 상태, 그 상태로 돌리면 예를 들어 화살표를 쓰는 순간 컴퓨터는 빈 주소인데 어디로 들어가라는 거야라며 프로그램 즉시 종료,,, 일반 변수(vector)는 스스로 초기화 할줄 안다. 근데 포인터는 스스로 공간을 못만듬 그래서 해준느거
    LiDAR() {
        pcl_RawCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_RoiCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_VoxelCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_GroundCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_NonGroundCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
};

#endif