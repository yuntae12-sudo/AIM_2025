#include "Clustering.hpp"

void Clustering(LiDAR& st_LiDAR)
{
    if (st_LiDAR.pcl_NonGroundCloud->empty()) return;

    // [최적화 1] KdTree 객체를 static으로 선언하여 재사용 (메모리 할당 방지)
    static pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    
    // [최적화 2] 클러스터 추출 객체도 static으로 관리
    static pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    // 데이터가 바뀔 때마다 트리만 갱신 (훨씬 빠름)
    tree->setInputCloud(st_LiDAR.pcl_NonGroundCloud);

    ec.setClusterTolerance(1.0); // 이 값도 나중엔 거리에 따라 변하게 하면 완벽함 , 원래1.3
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(3000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(st_LiDAR.pcl_NonGroundCloud);
    
    // 이전 결과 초기화 후 추출
    st_LiDAR.cluster_indices.clear();
    ec.extract(st_LiDAR.cluster_indices);
}

// void Clustering(LiDAR& st_LiDAR)
// {
//     // [0] 입력 데이터 검사
//     if (st_LiDAR.pcl_NonGroundCloud->empty()) return;

//     // [1] 자원 재사용을 위한 static 선언 (Heap 할당 최소화)
//     static pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     static pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

//     // 결과 바구니 초기화
//     st_LiDAR.cluster_indices.clear();

//     // [2] 거리별 구간 인덱스 바구니 준비
//     // zone 0: 0~15m, zone 1: 15~30m, zone 2: 30m 이상
//     std::vector<pcl::PointIndices> zone_indices(3);
    
//     // 
    
//     // 모든 점을 단 한 번만 훑으며 거리 제곱값($dist^2$)으로 구간 분류
//     for (int i = 0; i < (int)st_LiDAR.pcl_NonGroundCloud->size(); ++i) {
//         const auto& p = st_LiDAR.pcl_NonGroundCloud->points[i];
//         float dist_sq = p.x * p.x + p.y * p.y; // sqrt 연산 제거 최적화

//         if (dist_sq < 225.0f)       zone_indices[0].indices.push_back(i); // 15^2
//         else if (dist_sq < 900.0f)  zone_indices[1].indices.push_back(i); // 30^2
//         else                        zone_indices[2].indices.push_back(i); 
//     }

//     // [3] Euclidean Cluster Extraction 공통 설정
//     tree->setInputCloud(st_LiDAR.pcl_NonGroundCloud);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(st_LiDAR.pcl_NonGroundCloud);
//     ec.setMinClusterSize(5);     // 최소 점 개수
//     ec.setMaxClusterSize(3000);  // 최대 점 개수

//     // [4] 구간별 가변 Tolerance 적용 및 추출
//     // 가까울수록 촘촘하게(0.45m), 멀수록 넉넉하게(1.1m)
//     float tolerances[3] = {0.45f, 0.75f, 1.1f};

//     for (int i = 0; i < 3; ++i) {
//         if (zone_indices[i].indices.empty()) continue;

//         // 해당 구간의 포인트 인덱스만 추출 타겟으로 설정
//         pcl::PointIndices::Ptr indices_ptr = boost::make_shared<pcl::PointIndices>(zone_indices[i]);
//         ec.setIndices(indices_ptr); 
//         ec.setClusterTolerance(tolerances[i]);
        
//         std::vector<pcl::PointIndices> current_zone_clusters;
//         ec.extract(current_zone_clusters);
        
//         // 최종 결과 리스트(`st_LiDAR.cluster_indices`)에 병합
//         st_LiDAR.cluster_indices.insert(st_LiDAR.cluster_indices.end(), 
//                                         current_zone_clusters.begin(), 
//                                         current_zone_clusters.end());
//     }
// }

