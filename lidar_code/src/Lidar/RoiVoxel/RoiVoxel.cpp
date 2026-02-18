#include <Lidar/RoiVoxel/RoiVoxel.hpp>

// void Roi(LiDAR& st_LiDAR) {
//     if (st_LiDAR.pcl_RawCloud->empty()) return;

//     // [최적화 1] 객체 재사용 (Static)
//     static pcl::CropBox<pcl::PointXYZ> crop_outer;
//     static pcl::CropBox<pcl::PointXYZ> crop_inner;

//     // 1. 큰 관심 영역 설정
//     crop_outer.setInputCloud(st_LiDAR.pcl_RawCloud);
//     crop_outer.setMin(Eigen::Vector4f(-5.0, -5.0, -2.0, 1.0));
//     crop_outer.setMax(Eigen::Vector4f(23.0, 5.0, 3.0, 1.0));
//     crop_outer.filter(*st_LiDAR.pcl_RoiCloud);

//     // 2. 내 차량 영역 제거 (이미 생성된 객체의 설정만 변경)
//     crop_inner.setInputCloud(st_LiDAR.pcl_RoiCloud);
//     crop_inner.setMin(Eigen::Vector4f(-2.0, -1.2, -2.0, 1.0)); // Z 범위를 Outer와 맞춤
//     crop_inner.setMax(Eigen::Vector4f(2.0, 1.2, 2.0, 1.0));
//     crop_inner.setNegative(true); 
//     crop_inner.filter(*st_LiDAR.pcl_RoiCloud);
// }

void Roi(LiDAR& st_LiDAR) {
    if (st_LiDAR.pcl_RawCloud->empty()) return;

    st_LiDAR.pcl_RoiCloud->clear();
    
    // 포인터 추출 (가독성과 성능을 위해)
    const auto& raw_points = st_LiDAR.pcl_RawCloud->points;
    int total_points = (int)raw_points.size();

    // 임시 바구니 (스레드 충돌 방지를 위해 일단 인덱스만 저장)
    std::vector<int> inlier_indices;
    inlier_indices.reserve(total_points);

    // 수동 루프에 OpenMP 적용
    #pragma omp parallel
    {
        // 각 스레드별로 개인 바구니를 만들어 push_back 충돌 방지
        std::vector<int> local_indices;
        
        #pragma omp for nowait
        for (int i = 0; i < total_points; ++i) {
            const auto& pt = raw_points[i];
            
            // 1. Outer Box 조건 (도로 범위)
            bool is_in_outer = (pt.x > -5.0 && pt.x < 29.0 && pt.y > -6.0 && pt.y < 6.0 && pt.z > -2.0 && pt.z < 3.0);
            
            // 2. Inner Box 조건 (내 차 범위 제거)
            bool is_in_inner = (pt.x > -4.0 && pt.x < 0.6 && pt.y > -1.0 && pt.y < 1.0 && pt.z > -2.0 && pt.z < 1.5);

            if (is_in_outer && !is_in_inner) {
                local_indices.push_back(i);
            }
        }

        // 각 스레드가 찾은 인덱스를 합칠 때만 잠시 잠금(critical)
        #pragma omp critical
        {
            inlier_indices.insert(inlier_indices.end(), local_indices.begin(), local_indices.end());
        }
    }

        st_LiDAR.pcl_RoiCloud->resize(inlier_indices.size());
        #pragma omp parallel for // 마지막 복사도 병렬로 처리
        for (int i = 0; i < (int)inlier_indices.size(); ++i) {
        st_LiDAR.pcl_RoiCloud->points[i] = raw_points[inlier_indices[i]];
        }
}

void Voxel(LiDAR& st_LiDAR) {
    if (st_LiDAR.pcl_RoiCloud->empty()) return;

    // [최적화 2] VoxelGrid 객체 재사용
    static pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(st_LiDAR.pcl_RoiCloud);
    // [개선] Voxel 사이즈 최적화
    // 원래: 0.13m (너무 큼 → 작은 장애물 손실)
    // 개선: 0.08m (세밀한 형태 인식, 강건성 향상)
    vg.setLeafSize(0.08f, 0.08f, 0.08f);
    vg.filter(*st_LiDAR.pcl_VoxelCloud);
}