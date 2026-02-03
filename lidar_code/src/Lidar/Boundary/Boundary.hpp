#ifndef BOUNDARY_HPP
#define BOUNDARY_HPP

#include <Global/Global.hpp> // LiDAR 구조체나 공통 변수를 사용하기 위함

/**
 * @brief Voxel 처리된 점들을 ENU 바운더리 기준으로 필터링함
 * @param st_LiDAR LiDAR 데이터 구조체
 * @param ego_x 차량 현재 X (ENU)
 * @param ego_y 차량 현재 Y (ENU)
 * @param ego_heading 차량 현재 Heading (Degree)
 */
void filterPointsByBoundary(LiDAR& st_LiDAR, double ego_x, double ego_y, double ego_heading);

#endif