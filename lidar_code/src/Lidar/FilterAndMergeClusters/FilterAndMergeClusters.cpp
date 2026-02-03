#include <Lidar/FilterAndMergeClusters/FilterAndMergeClusters.hpp>
#include <algorithm>
#include <vector>

// 1. 클러스터의 범위를 담는 구조체 (내부 루프 최적화용)
struct ClusterBox {
    float x_min, x_max, y_min, y_max;
    int point_count;
};

// 헬퍼 함수: 특정 클러스터의 범위를 계산 (딱 한 번만 실행됨)
ClusterBox CalculateBox(const LiDAR& st_LiDAR, const pcl::PointIndices& cluster) {
    ClusterBox box = {1e6, -1e6, 1e6, -1e6, (int)cluster.indices.size()};
    for (int idx : cluster.indices) {
        const auto& p = st_LiDAR.pcl_NonGroundCloud->points[idx];
        box.x_min = std::min(box.x_min, p.x);
        box.x_max = std::max(box.x_max, p.x);
        box.y_min = std::min(box.y_min, p.y);
        box.y_max = std::max(box.y_max, p.y);
    }
    return box;
}

bool IsGuardrail(const ClusterBox& b)
{
    float len = b.x_max - b.x_min;
    float wid = b.y_max - b.y_min;

    if (len > 1.0f && wid < 0.9f)
        return true;

    return false;
}


void FilterAndMergeClusters(LiDAR& st_LiDAR) {
    if (st_LiDAR.cluster_indices.empty()) return;

    // ===============================
    // [1] 가드레일 클러스터 제거
    // ===============================
    std::vector<pcl::PointIndices> filtered_clusters;

    for (const auto& cluster : st_LiDAR.cluster_indices)
    {
        ClusterBox box = CalculateBox(st_LiDAR, cluster);

        if (IsGuardrail(box))
            continue;   // ★ 가드레일은 아예 버림

        filtered_clusters.push_back(cluster);
    }

    // 가드레일 제거 후 결과로 교체
    st_LiDAR.cluster_indices.swap(filtered_clusters);

    // 여기까지 하고 일단 종료 (머지는 OFF)
    // return;



    // ===============================
    // [2] (옵션) 가까운 클러스터 머지 - 지금은 사용 X
    // ===============================
    
    if (st_LiDAR.cluster_indices.empty()) return;

    size_t n = st_LiDAR.cluster_indices.size();
    std::vector<pcl::PointIndices> merged_indices = st_LiDAR.cluster_indices;
    std::vector<bool> cluster_merged(n, false);
    std::vector<pcl::PointIndices> final_indices;

    std::vector<ClusterBox> boxes(n);
    for (size_t i = 0; i < n; ++i) {
        boxes[i] = CalculateBox(st_LiDAR, merged_indices[i]);
    }

    for (size_t i = 0; i < n; ++i) {
        if (cluster_merged[i]) continue;

        for (size_t j = i + 1; j < n; ++j) {
            if (cluster_merged[j]) continue;

            float dist_x = std::max(0.0f, std::max(boxes[i].x_min, boxes[j].x_min) - 
                                          std::min(boxes[i].x_max, boxes[j].x_max));
            float dist_y = std::max(0.0f, std::max(boxes[i].y_min, boxes[j].y_min) - 
                                          std::min(boxes[i].y_max, boxes[j].y_max));
            
            if (dist_x < 1.0f && dist_y < 1.5f) {

                merged_indices[i].indices.reserve(
                    merged_indices[i].indices.size() + merged_indices[j].indices.size()
                );
                merged_indices[i].indices.insert(
                    merged_indices[i].indices.end(),
                    merged_indices[j].indices.begin(),
                    merged_indices[j].indices.end()
                );
                cluster_merged[j] = true;

                boxes[i].x_min = std::min(boxes[i].x_min, boxes[j].x_min);
                boxes[i].x_max = std::max(boxes[i].x_max, boxes[j].x_max);
                boxes[i].y_min = std::min(boxes[i].y_min, boxes[j].y_min);
                boxes[i].y_max = std::max(boxes[i].y_max, boxes[j].y_max);
            }
        }

        final_indices.push_back(merged_indices[i]);
    }

    st_LiDAR.cluster_indices = final_indices;
}