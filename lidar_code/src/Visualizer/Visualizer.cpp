#include "Visualizer.hpp"
#include <tf2/LinearMath/Quaternion.h> // 헤딩을 쿼터니언으로 바꾸기 위해 필요

void PublishObjectBoundingBox(ros::Publisher& pub, const std::vector<Lbox>& vec_Objects, const std::string& frame_id) {
    visualization_msgs::MarkerArray marker_array;
    
    // [최적화 1] 메모리 예약 (객체 개수 + 삭제 마커 1개)
    marker_array.markers.reserve(vec_Objects.size() + 1);

    // [최적화 2] 공통 타임스탬프 (단 한 번의 시스템 콜)
    ros::Time now = ros::Time::now();

    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = now;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (size_t i = 0; i < vec_Objects.size(); ++i) {
        const auto& obj = vec_Objects[i];
        visualization_msgs::Marker marker;
        
        marker.header.frame_id = frame_id;
        marker.header.stamp = now; // 공통 시간 사용
        marker.ns = "l_shape";
        marker.id = (int)i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // ... (나머지 포즈 및 스케일 설정은 동일) ...
        marker.pose.position.x = obj.f32_X_center;
        marker.pose.position.y = obj.f32_Y_center;
        marker.pose.position.z = obj.f32_Z_center;

        tf2::Quaternion q;
        q.setRPY(0, 0, obj.f32_Heading);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = obj.f32_L;
        marker.scale.y = obj.f32_W;
        marker.scale.z = obj.f32_H;

        marker.color.r = 0.0f; 
        marker.color.g = 1.0f; 
        marker.color.b = 0.0f; 
        marker.color.a = 0.5f;

        marker_array.markers.push_back(marker);
    }
    pub.publish(marker_array);
}

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

void PublishClusters(ros::Publisher& pub, const LiDAR& st_LiDAR, const std::string& frame_id) {
    if (st_LiDAR.cluster_indices.empty()) return;

    // [최적화 1] 객체 재사용 (static)
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->clear(); // 메모리 해제 없이 바구니만 비움
    
    // [최적화 2] 공통 타임스탬프 사용
    ros::Time now = ros::Time::now();

    int cluster_id = 0;
    for (const auto& indices : st_LiDAR.cluster_indices) {
        uint8_t r = (cluster_id * 70) % 255;
        uint8_t g = (cluster_id * 120) % 255;
        uint8_t b = (cluster_id * 180) % 255;

        for (const auto& idx : indices.indices) {
            pcl::PointXYZRGB p;
            const auto& p_in = st_LiDAR.pcl_NonGroundCloud->points[idx];
            p.x = p_in.x; p.y = p_in.y; p.z = p_in.z;
            p.r = r; p.g = g; p.b = b;
            colored_cloud->points.push_back(p);
        }
        cluster_id++;
    }

    // [최적화 3] 메시지 객체 재사용 고려 (선택사항)
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*colored_cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = now;

    pub.publish(output);
}

void PublishPointCloud(ros::Publisher& pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& frame_id) {
    if (cloud->empty()) return;

    // [최적화 1] 메시지 객체 재사용 (Capacity 유지)
    static sensor_msgs::PointCloud2::Ptr output_ptr(new sensor_msgs::PointCloud2);
    
    // [최적화 2] 공통 시간 사용
    ros::Time now = ros::Time::now();

    // PCL 데이터를 메시지로 변환 (필연적인 복사 구간)
    pcl::toROSMsg(*cloud, *output_ptr);
    
    output_ptr->header.frame_id = frame_id;
    output_ptr->header.stamp = now;

    // [최적화 3] 포인터를 발행하여 ROS 내부 복사 최소화
    pub.publish(output_ptr);
}

void PublishTracks(ros::Publisher& pub, const std::vector<Track>& vec_Tracks, const std::string& frame_id) {
    visualization_msgs::MarkerArray marker_array;
    
    // [최적화 1] 메모리 예약 (트랙당 마커 2개: 박스+텍스트, 그리고 삭제 마커 1개)
    // 루프 돌기 전 미리 공간을 확보하여 벡터 재할당(이사) 비용을 0으로 만듭니다.
    marker_array.markers.reserve(vec_Tracks.size() * 2 + 1);

    // [최적화 2] 공통 타임스탬프 (단 한 번의 시스템 콜)
    // 루프 안에서 매번 시간을 묻지 않고, 모든 마커가 동일한 시간을 공유하게 합니다.
    ros::Time now = ros::Time::now();

    // 이전 마커 삭제 (필수)
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = now;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (const auto& t : vec_Tracks) {
        // 0프레임 이상 실종된 트랙은 그리지 않음 (추적의 확실성 보장)
        if (t.s32_LostFrames > 0) continue;

        // --- 1. 바운딩 박스 마커 (CUBE) ---
        visualization_msgs::Marker box_marker;
        box_marker.header.frame_id = frame_id;
        box_marker.header.stamp = now; // 공통 시간 사용
        box_marker.ns = "track_boxes";
        box_marker.id = t.s32_ID; 
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.action = visualization_msgs::Marker::ADD;

        // 위치 (칼만 필터가 예측/보정한 위치)
        box_marker.pose.position.x = t.f32_X_center;
        box_marker.pose.position.y = t.f32_Y_center;
        box_marker.pose.position.z = t.f32_Z_center;

        // 회전 (헤딩값 적용)
        tf2::Quaternion q;
        q.setRPY(0, 0, t.f32_Heading);
        box_marker.pose.orientation.x = q.x();
        box_marker.pose.orientation.y = q.y();
        box_marker.pose.orientation.z = q.z();
        box_marker.pose.orientation.w = q.w();

        // 크기 (저주파 필터가 적용된 크기)
        box_marker.scale.x = t.f32_L;
        box_marker.scale.y = t.f32_W;
        box_marker.scale.z = t.f32_H;

        // 색상 (트래킹 생성 시 부여된 고유 색상)
        box_marker.color.r = t.u8_R / 255.0f;
        box_marker.color.g = t.u8_G / 255.0f;
        box_marker.color.b = t.u8_B / 255.0f;
        box_marker.color.a = 0.6f;

        marker_array.markers.push_back(box_marker);

        // --- 2. ID 및 속도 텍스트 마커 (TEXT_VIEW_FACING) ---
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = now; // 공통 시간 사용
        text_marker.ns = "track_info";
        text_marker.id = t.s32_ID + 10000; // ID 중복 방지
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        
        // 속도 계산 (m/s -> km/h 변환)
        float velocity = std::sqrt(std::pow(t.vec_X[2], 2) + std::pow(t.vec_X[3], 2)) * 3.6f;
        
        // 표시할 텍스트 구성
        char buf[50];
        sprintf(buf, "ID: %d | %.1f km/h", t.s32_ID, velocity);
        text_marker.text = buf;

        // 위치: 박스의 머리 위 0.5m 지점
        text_marker.pose.position.x = t.f32_X_center;
        text_marker.pose.position.y = t.f32_Y_center;
        text_marker.pose.position.z = t.f32_Z_center + (t.f32_H / 2.0f) + 0.5f;

        text_marker.scale.z = 0.7f; // 글자 크기
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0f;

        marker_array.markers.push_back(text_marker);
    }

    pub.publish(marker_array);
}