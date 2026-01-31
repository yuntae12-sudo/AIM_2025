#include <Lidar/Ransac/Ransac.hpp>

void Ransac(LiDAR& st_LiDAR)
{
    if (st_LiDAR.pcl_VoxelCloud->empty()) return;

    static pcl::SACSegmentation<pcl::PointXYZ> seg;
    static pcl::ExtractIndices<pcl::PointXYZ> extract;
    static pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    static pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold(0.25f); // 바닥 두께 설정
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(15.0 * M_PI / 180.0); // 20도보다 15도가 더 엄격하고 안전함

    seg.setInputCloud(st_LiDAR.pcl_VoxelCloud);
    seg.segment(*inliers, *coefficients);

    if (!inliers->indices.empty())
    {
        extract.setInputCloud(st_LiDAR.pcl_VoxelCloud);
        extract.setIndices(inliers);

        extract.setNegative(false);
        extract.filter(*st_LiDAR.pcl_GroundCloud);

        extract.setNegative(true);
        extract.filter(*st_LiDAR.pcl_NonGroundCloud);

        static pcl::CropBox<pcl::PointXYZ> zcrop;

        zcrop.setInputCloud(st_LiDAR.pcl_NonGroundCloud);
        zcrop.setMin(Eigen::Vector4f(-100.0f, -100.0f, -1.25f, 1.0f)); 
        zcrop.setMax(Eigen::Vector4f( 100.0f,  100.0f,  100.0f, 1.0f));

        pcl::PointCloud<pcl::PointXYZ> tmp;
        zcrop.filter(tmp);
        st_LiDAR.pcl_NonGroundCloud->swap(tmp);

    }
}


        // ==========================================================
        // [추가] 최종 찌꺼기 제거: 물리적 높이 필터 (Z-Pass Filter)
        // ==========================================================
        // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clean_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // for (const auto& pt : st_LiDAR.pcl_NonGroundCloud->points) {
        //     // 지면(-0.67)보다 약 12cm 위(-0.55)인 점들만 최종 살아남음
        //     if (pt.z > -0.7f) { 
        //         tmp_clean_cloud->points.push_back(pt);
        //     }
        // }
        
        // // 필터링된 깨끗한 클라우드로 교체
        // st_LiDAR.pcl_NonGroundCloud->swap(*tmp_clean_cloud);
// #include <Lidar/Ransac/Ransac.hpp>
// #include <patchwork/patchworkpp.h> 

// void EstimateGroundPatchwork(LiDAR& st_LiDAR)
// {
//     if (st_LiDAR.pcl_VoxelCloud->empty()) return;

//     static patchwork::Params patchwork_params;
//     static bool is_initialized = false;
    
//     if (!is_initialized) {
//         // [수정] 에러가 났던 th_dist_v 라인을 삭제하거나 주석 처리합니다.
//         patchwork_params.sensor_height = 0.0;  // 라이다 설치 높이 (가장 중요!)
//         patchwork_params.th_dist = 0.3;         // RANSAC의 0.3f와 동일한 역할
//             // 3. 씨앗 점(Seed) 찾기 임계값 (음수 방향으로 더 깊게)
//         // 센서 아래쪽 1.2m까지는 다 바닥 후보로 보겠다는 뜻입니다.
//         patchwork_params.th_seeds = 0.2; 

//         // 4. 평탄도 기준 완화 (기존보다 2배 더 관대하게)
//         // 도로가 조금 울퉁불퉁해도 바닥으로 인정합니다.
//         // patchwork_params.max_flatness_threshold = 0.3;
//         // 5. 각 구역별 최소 점 개수 (낮출수록 바닥을 더 잘 찾음)
//         patchwork_params.num_lpr = 10;
//         // 만약 th_dist_v가 안 된다면 기본 설정을 따르도록 두는 것이 안전합니다.
//         patchwork_params.verbose = false; 
        
//         is_initialized = true;
//     }

//     static patchwork::PatchWorkpp patchwork_engine(patchwork_params);

//     // 2. 데이터 변환 (PCL -> Eigen)
//     int n_points = st_LiDAR.pcl_VoxelCloud->points.size();
//     Eigen::MatrixXf cloud_eigen(n_points, 3);
//     for (int i = 0; i < n_points; ++i) {
//         cloud_eigen(i, 0) = st_LiDAR.pcl_VoxelCloud->points[i].x;
//         cloud_eigen(i, 1) = st_LiDAR.pcl_VoxelCloud->points[i].y;
//         cloud_eigen(i, 2) = st_LiDAR.pcl_VoxelCloud->points[i].z;
//     }

//     // 3. 실행 및 결과 추출
//     patchwork_engine.estimateGround(cloud_eigen);

//     Eigen::VectorXi ground_idx_eigen = patchwork_engine.getGroundIndices();
//     std::vector<int> ground_idx(ground_idx_eigen.data(), ground_idx_eigen.data() + ground_idx_eigen.size());
    
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//     inliers->indices = ground_idx;

//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud(st_LiDAR.pcl_VoxelCloud);
//     extract.setIndices(inliers);

//     extract.setNegative(false);
//     extract.filter(*st_LiDAR.pcl_GroundCloud);
//     extract.setNegative(true);
//     extract.filter(*st_LiDAR.pcl_NonGroundCloud);
// }

