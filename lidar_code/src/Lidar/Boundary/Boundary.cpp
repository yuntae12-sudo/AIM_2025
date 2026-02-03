#include <Lidar/Boundary/Boundary.hpp>
#include <Global/Global.hpp>
#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>

struct PointENU { double x, y; };

// 두 개의 바운더리 저장용
std::vector<PointENU> IN_LANE_BOUNDARY;
std::vector<PointENU> OUT_LANE_BOUNDARY;

// 파일 로드 함수 (재사용 가능하게 수정)
void loadBoundaryFile(const std::string& file_path, std::vector<PointENU>& target_vec) {
    target_vec.clear();
    std::ifstream infile(file_path);
    if (!infile.is_open()) {
        std::cerr << "파일 실패: " << file_path << std::endl;
        return;
    }
    double x, y, z;
    while (infile >> x >> y >> z) {
        target_vec.push_back({x, y});
    }
    std::cout << file_path << " 로드 완료: " << target_vec.size() << std::endl;
}

// 다각형 내부 판별 (기존과 동일)
bool isInsideBoundary(float x, float y, const std::vector<Point2D>& boundary) {
    bool inside = false;
    int n = (int)boundary.size();
    if (n < 3) return false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((boundary[i].y > y) != (boundary[j].y > y)) &&
            (x < (boundary[j].x - boundary[i].x) * (y - boundary[i].y) / (boundary[j].y - boundary[i].y) + boundary[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

void filterPointsByBoundary(LiDAR& st_LiDAR, double ego_x, double ego_y, double ego_heading) {
    // 1. 최초 실행 시 두 파일 모두 로드
    if (IN_LANE_BOUNDARY.empty()) {
        loadBoundaryFile("/home/autonav/AIM_2025/src/lidar_code/src/data/in__boundary.txt", IN_LANE_BOUNDARY);
        loadBoundaryFile("/home/autonav/AIM_2025/src/lidar_code/src/data/out__boundary.txt", OUT_LANE_BOUNDARY);
    }

    // if (st_LiDAR.pcl_VoxelCloud->empty()) return;
    if (st_LiDAR.pcl_NonGroundCloud->empty()) return;

    // 시각화 코드와 동일한 각도 보정
    // MORAI Heading(Degree) -> ROS Yaw(Radian)
    // 만약 시각화 코드의 egoPose.current_yaw를 직접 쓸 수 있다면 그게 베스트
    double ros_yaw = ego_heading;
    double cos_yaw = std::cos(ros_yaw);
    double sin_yaw = std::sin(ros_yaw);

    auto transform = [&](const std::vector<PointENU>& enu_vec) {
        std::vector<Point2D> local_vec;
        for (const auto& enu : enu_vec) {
            // 시각화 코드와 똑같은 상대 거리 계산
            double dx = enu.x - ego_x;
            double dy = enu.y - ego_y;

            // [시각화 코드 정답 수식 적용]
            local_vec.push_back({
                (float)( dx * cos_yaw + dy * sin_yaw), // pt.x
                (float)(-dx * sin_yaw + dy * cos_yaw)  // pt.y
            });
        }
        return local_vec;
    };

    std::vector<Point2D> local_in = transform(IN_LANE_BOUNDARY);
    std::vector<Point2D> local_out = transform(OUT_LANE_BOUNDARY);

    if (!local_out.empty()) {
        ROS_INFO_THROTTLE(1.0, "===========================================");
        ROS_INFO_THROTTLE(1.0, "[Ego Pose] X: %.2f, Y: %.2f, Heading: %.2f", ego_x, ego_y, ego_heading);
        ROS_INFO_THROTTLE(1.0, "[Local Boundary Check] X(Front): %.2f m, Y(Left): %.2f m", local_out[0].x, local_out[0].y);
        ROS_INFO_THROTTLE(1.0, "===========================================");
    }



// void filterPointsByBoundary(LiDAR& st_LiDAR, double ego_x, double ego_y, double ego_heading) {
//     // 1. 최초 실행 시 두 파일 모두 로드
//     if (IN_LANE_BOUNDARY.empty()) {
//         loadBoundaryFile("/home/autonav/AIM_2025/src/lidar_code/src/data/in__boundary.txt", IN_LANE_BOUNDARY);
//         loadBoundaryFile("/home/autonav/AIM_2025/src/lidar_code/src/data/out__boundary.txt", OUT_LANE_BOUNDARY);
//     }

//     // if (st_LiDAR.pcl_VoxelCloud->empty()) return;
//     if (st_LiDAR.pcl_NonGroundCloud->empty()) return;

//     // 시각화 코드와 동일한 각도 보정
//     // MORAI Heading(Degree) -> ROS Yaw(Radian)
//     // 만약 시각화 코드의 egoPose.current_yaw를 직접 쓸 수 있다면 그게 베스트
//     double ros_yaw = ego_heading;
//     double cos_yaw = std::cos(ros_yaw);
//     double sin_yaw = std::sin(ros_yaw);

//     // 🔸 추가: 맵 포인트 중 "너무 먼 것"은 아예 변환하지 않기 위한 최대 거리 (Ego 기준 ENU)
//     const double MAX_MAP_DIST = 120.0;           // [m]
//     const double max_dist_sq  = MAX_MAP_DIST * MAX_MAP_DIST;

//     auto transform = [&](const std::vector<PointENU>& enu_vec) {
//         std::vector<Point2D> local_vec;
//         local_vec.reserve(enu_vec.size());

//         for (const auto& enu : enu_vec) {
//             // 시각화 코드와 똑같은 상대 거리 계산
//             double dx = enu.x - ego_x;
//             double dy = enu.y - ego_y;

//             // 🔸 추가 1: Ego 기준으로 너무 먼 맵 포인트는 스킵 (성능 최적화)
//             double dist_sq = dx * dx + dy * dy;
//             if (dist_sq > max_dist_sq) {
//                 continue;
//             }

//             // [시각화 코드 정답 수식 적용]
//             float local_x = static_cast<float>( dx * cos_yaw + dy * sin_yaw); // pt.x
//             float local_y = static_cast<float>(-dx * sin_yaw + dy * cos_yaw); // pt.y

//             local_vec.push_back({ local_x, local_y });
//         }
//         return local_vec;
//     };

//     std::vector<Point2D> local_in = transform(IN_LANE_BOUNDARY);
//     std::vector<Point2D> local_out = transform(OUT_LANE_BOUNDARY);

//     if (!local_out.empty()) {
//         ROS_INFO_THROTTLE(1.0, "===========================================");
//         ROS_INFO_THROTTLE(1.0, "[Ego Pose] X: %.2f, Y: %.2f, Heading: %.2f", ego_x, ego_y, ego_heading);
//         ROS_INFO_THROTTLE(1.0, "[Local Boundary Check] X(Front): %.2f m, Y(Left): %.2f m", local_out[0].x, local_out[0].y);
//         ROS_INFO_THROTTLE(1.0, "===========================================");
//     }








    // 2. 필터링 로직 (바운더리 안에 있는 점들만 남기기)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // filterPointsByBoundary 함수 내부
    // for (const auto& pt : st_LiDAR.pcl_VoxelCloud->points) {
    for (const auto& pt : st_LiDAR.pcl_NonGroundCloud->points) {
        // 1. 바깥쪽 바운더리(Outer) 안에 있는지만 검사합니다.
        bool in_outer = isInsideBoundary(pt.x, pt.y, local_out);
        bool in_inner = isInsideBoundary(pt.x, pt.y, local_in);
        
        // 2. 안쪽 바운더리(in_inner)는 데이터가 이상하므로 일단 체크하지 않습니다.
        if (!in_outer && in_inner) { 
            cloud_filtered->points.push_back(pt);
        }
    }

    // ROS_INFO_THROTTLE(1.0, "Voxel: %zu -> Filtered: %zu", st_LiDAR.pcl_VoxelCloud->size(), cloud_filtered->size());
    // st_LiDAR.pcl_VoxelCloud = cloud_filtered;
    ROS_INFO_THROTTLE(1.0, "NonGround: %zu -> Filtered: %zu",
                  st_LiDAR.pcl_NonGroundCloud->size(), cloud_filtered->size());
    st_LiDAR.pcl_NonGroundCloud = cloud_filtered;
}