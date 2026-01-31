#include "LshapeFitting.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <Eigen/Geometry>

tf2_ros::Buffer tfBuffer;
static std::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;

class LShapeClosenessFitter {
public:
    LShapeClosenessFitter(double d0 = 0.15); //생성자

    // [원본 성능 핵심] CH-AWC 기반 L-Shape 피팅 함수
    bool fit(const std::vector<Point2D>& cluster, Rectangle& rect);
//Point2D는 점 하나하나(x=1.2, y=3.4)의미, std::vector<Point2D>이점들을 일렬로 세워놓은 점들의 묶음
private:
    double d0_;

    void project(const std::vector<Point2D>& pts, const Eigen::Vector2d& e1, const Eigen::Vector2d& e2,
                 std::vector<double>& C1, std::vector<double>& C2);

    void computeRectangle(const std::vector<Point2D>& pts, double theta, double score, Rectangle& rect);

    // Convex Hull 및 CH-AWC 관련 보조 함수
    std::vector<Point2D> findConvexHull(const std::vector<Point2D>& pts);
    double calculateArea(const std::vector<Point2D>& hull);
    double calculateCHAWCScore(const std::vector<Point2D>& cluster, double theta, double a_hull);

    static std::vector<Point2D> pruneHull(const std::vector<Point2D>& hull, double ratio);
};

// TF2를 한 번만 안전하게 초기화하는 헬퍼 함수
void ensureTF2Initialized() {
    if (!tfListenerPtr) {
        tfListenerPtr = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
    }
}

LShapeClosenessFitter::LShapeClosenessFitter(double d0)
    : d0_(d0) {} // d0라는 값을 받아서 클래스의 변수인 d0_에다 바로 넣어주기 d0_=d0라고 함수안에 대입하는것보다 메모리 처리 속도가 빠름
// 점들을 정렬하기 위한 보조 함수 (x좌표 우선, 같으면 y좌표 기준)
bool comparePoints(const Point2D& a, const Point2D& b) { //데이터 정렬
    return (a.x < b.x) || (a.x == b.x && a.y < b.y); //
}
const double EPSILON = 1e-9;
// 세 점의 방향성을 확인하는 함수 (외적 이용) [cite: 56, 67]
// 결과가 0보다 크면 반시계 방향, 0이면 일직선, 0보다 작으면 시계 방향 
double crossProduct(const Point2D& a, const Point2D& b, const Point2D& c) {
    double val = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);

    // 절대값이 EPSILON보다 작으면 사실상 일직선(0)으로 간주
    if (std::abs(val) < EPSILON) return 0.0;
    return val;
}
//-------------------점들을 감싸는 외곽선 정점 뽑ㄱㅣ---findCondvexHull-----------------------------------------------------
std::vector<Point2D> LShapeClosenessFitter::findConvexHull(const std::vector<Point2D>& pts) {
    int n = pts.size();
    if (n <= 3) return pts; // 점이 3개 이하면 그 자체가 외곽선임

    std::vector<Point2D> sorted_pts = pts;
    std::sort(sorted_pts.begin(), sorted_pts.end(), comparePoints); // sort는 데이터를 빠르게 재배열하는 함수 어디부터 어디까지 어떤기준으로

    std::vector<Point2D> hull; //외곽선 정점으로 최종 선택된 점들만 따로 모아둘 빈바구니 만드는거

    // 2. 아래쪽 껍질(Lower Hull) 만들기 [cite: 52]
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull.back(), sorted_pts[i]) <= 0) {
            hull.pop_back(); // 안쪽으로 꺾이면 마지막 점 제거 
        }
        hull.push_back(sorted_pts[i]);
    }

    // 3. 위쪽 껍질(Upper Hull) 만들기 [cite: 52]
    int lower_hull_size = hull.size();
    for (int i = n - 2; i >= 0; --i) {
        while (hull.size() > lower_hull_size && crossProduct(hull[hull.size() - 2], hull.back(), sorted_pts[i]) <= 0) {
            hull.pop_back(); // 안쪽으로 꺾이면 제거 
        }
        hull.push_back(sorted_pts[i]);
    }

    hull.pop_back(); // 시작점이 중복되므로 마지막 점 하나 제거
    return hull;
}

//------------------------------------이거 뭐더라
std::vector<Point2D>
LShapeClosenessFitter::pruneHull(const std::vector<Point2D>& hull,
                                 double ratio)
{
    if (hull.size() <= 4) return hull;

    double cx = 0, cy = 0;
    for (auto& p : hull) { cx += p.x; cy += p.y; }
    cx /= hull.size();
    cy /= hull.size();

    double max_r = 0;
    for (auto& p : hull) {
        double r = hypot(p.x - cx, p.y - cy);
        if (r > max_r) max_r = r;
    }

    std::vector<Point2D> cleaned;
    for (auto& p : hull) {
        double r = hypot(p.x - cx, p.y - cy);
        if (r > max_r * ratio)
            cleaned.push_back(p);
    }
    return cleaned;
}

//-----------------------면적 뽑기-----calculateArea--------------------------------------------------
double LShapeClosenessFitter::calculateArea(const std::vector<Point2D>& hull) {  
    double area = 0.0;
    int n = hull.size();
    if (n < 3) return 0.0; // 점이 3개는 있어야 면적이 생김 [cite: 41, 42]

    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        area += hull[i].x * hull[j].y;
        area -= hull[j].x * hull[i].y;
    }
    return std::abs(area) / 2.0; // 절댓값을 취해 최종 면적 산출 [cite: 58, 72]
}
//------면적 가중치----calculateCHAWCScore------------------------------------------------------------------------------------------------------
double LShapeClosenessFitter::calculateCHAWCScore(const std::vector<Point2D>& cluster, double theta, double a_hull) {
    // (a) 현재 각도로 점들을 정렬(투영)하고 박스 크기(A_OBB) 구하기 [cite: 92, 95]
    Eigen::Vector2d e1(cos(theta), sin(theta));
    Eigen::Vector2d e2(-sin(theta), cos(theta)); 
    
    double c1_min = std::numeric_limits<double>::max();//세상에서 가장큰 값을 넣어두면 어떤 값이 오든 이것보단 작을테니까 업데이트 ㄱㄴ
    double c1_max = -std::numeric_limits<double>::max();//세ㅔ상에서 가장 작은 값을 넣어두면 어떤 값이 오든 이것보다클테니까 업데이트 ㄱㄴ
    double c2_min = std::numeric_limits<double>::max();
    double c2_max = -std::numeric_limits<double>::max();
    ///// 점수계산용 내적
    std::vector<double> C1, C2;
    for (const Point2D& p : cluster) {
        double p1 = p.x * e1.x() + p.y * e1.y(); //회전된 새로운 축위로 각 점을 투영했을때 좌표값을 계산, 내적을 한거(투영)
        double p2 = p.x * e2.x() + p.y * e2.y();
        C1.push_back(p1); C2.push_back(p2);//모든 점의 길이방향 좌표(p1)들을 모아놓은 리스트, 모든 점의 폭 방향 좌표(p2)를 모아놓은 리스트
        c1_min = std::min(c1_min, p1); c1_max = std::max(c1_max, p1); //처음에는 둘다 p1인데 점이 계속 들어올수록 c1_min은 왼쪽 끝, c1_max는 오른쪽 끝을 가리킴,이러다 보면 결국 두값의 차이는 우리가 찾던 물체의 길이가 됨
        c2_min = std::min(c2_min, p2); c2_max = std::max(c2_max, p2);
    }

    // (b) 테두리 근접도(Closeness Score, beta) 계산 [cite: 82, 95]
    double beta = 0.0;
    for (size_t i = 0; i < C1.size(); ++i) {
        double d1 = std::min(c1_max - C1[i], C1[i] - c1_min); //i번째 점이 박스의 오른쪽 끝에서 얼마나 떨어져있나, i번째 점이 박스의 왼쪽 끝에서 얼마나 떨어져있나
        double d2 = std::min(c2_max - C2[i], C2[i] - c2_min);
        double d = std::max(std::min(d1, d2), d0_); //앞뒤좌우중 가장 가까운 면까지의 거리구함, 만약 네 점이 너무 딱 붙어서 거리가 0이된다면 1/d가 무한대가 됨, 이를 방지하기위해 최소거리를 보장해주는 d0가있음
        beta += 1.0 / d;
    }

    // (c) 면적 효율성 가중치(rho) 계산 [cite: 89, 92]
    double a_obb = (c1_max - c1_min) * (c2_max - c2_min);
    double rho = a_hull / (a_obb + 1e-6); // 1e-6은 0으로 나누기 방지 [cite: 92, 95]

    // (d) 최종 점수 = 근접도 * 면적 가중치 [cite: 90, 95]
    return beta * rho;
}
// =====================
// Projection, 최종 박스 생성용, 그 각도 하나에 대해서만 다시 제대로 투영을 해서 진짜 모서리 좌표를 계산하는거
// =====================
void LShapeClosenessFitter::project(const std::vector<Point2D>& pts,
                                    const Eigen::Vector2d& e1,
                                    const Eigen::Vector2d& e2,
                                    std::vector<double>& C1,
                                    std::vector<double>& C2)
{
    C1.clear();
    C2.clear();//여러번 호출 되므로 이전 데이터 깨끗이 비우고 시작

    for (const Point2D& p : pts) {
        Eigen::Vector2d x(p.x, p.y); //p를 벡터 x로 변환
        C1.push_back(x.dot(e1)); //e1축 방향으로의 그림자 길이 저장
        C2.push_back(x.dot(e2)); //e2축 방향으로의 그림자 길이 저장, (내적) ,computerectangle에서 만든 C1,C2에 넣어줌
    }
}

// =====================
// Rectangle computation
// =====================
void LShapeClosenessFitter::computeRectangle(
    const std::vector<Point2D>& pts, //원본 점 구름 데이터 묶음
    double theta, //회전 각도
    double score,
    Rectangle& rect)
{
    Eigen::Vector2d e1(std::cos(theta), std::sin(theta));
    Eigen::Vector2d e2(-std::sin(theta), std::cos(theta));

    std::vector<double> C1, C2; //이거는 최종각도에 따른 정사영값들을 담음
    project(pts, e1, e2, C1, C2);

    double c1_min = *std::min_element(C1.begin(), C1.end());
    double c1_max = *std::max_element(C1.begin(), C1.end());
    double c2_min = *std::min_element(C2.begin(), C2.end());
    double c2_max = *std::max_element(C2.begin(), C2.end());

    rect.corners[0] = c1_min * e1 + c2_min * e2; //다시 원래의 월드 좌표계상의 점으로 변환
    rect.corners[1] = c1_min * e1 + c2_max * e2;
    rect.corners[2] = c1_max * e1 + c2_max * e2;
    rect.corners[3] = c1_max * e1 + c2_min * e2;

    rect.heading = theta;
    rect.score = score;
}
//-------------------------------------fit-----------------------------------------------
bool LShapeClosenessFitter::fit(const std::vector<Point2D>& cluster, Rectangle& rect) {

    if (cluster.size() < 3) return false;

    std::vector<Point2D> hull = findConvexHull(cluster);
    hull = pruneHull(hull, 0.1);
    if (hull.size() < 3) hull = cluster;

    double a_hull = calculateArea(hull);

    double best_score = -1.0;
    double best_theta = 0.0;

    std::vector<double> theta_candidates;

    // 1) Convex hull 기반 각도
    for (size_t i = 0; i < hull.size(); ++i) {
        int next = (i + 1) % hull.size();
        double dx = hull[next].x - hull[i].x;
        double dy = hull[next].y - hull[i].y;

        double theta = atan2(dy, dx);
        theta = fmod(theta, M_PI/2.0);
        if (theta < 0) theta += M_PI/2.0;

        theta_candidates.push_back(theta);
    }

    // 2) Uniform sampling
    for (double theta = 0.0; theta < M_PI/2.0; theta += (M_PI/180.0)) {
        theta_candidates.push_back(theta);
    }

    // 3) 중복 제거
    std::sort(theta_candidates.begin(), theta_candidates.end());
    theta_candidates.erase(std::unique(theta_candidates.begin(), theta_candidates.end()),
                           theta_candidates.end());

    // 4) 점수 계산
    for (double theta : theta_candidates) {
        double score = calculateCHAWCScore(cluster, theta, a_hull);

        if (score > best_score) {
            best_score = score;
            best_theta = theta;
        }
    }

    computeRectangle(cluster, best_theta, best_score, rect);
    return true;
}



void L_ShapeFitting(LiDAR& st_LiDAR)
{
    if (st_LiDAR.cluster_indices.empty()) return;

    ensureTF2Initialized();

    static LShapeClosenessFitter fitter(0.15); //프로그램 터지는거 방지인데 위에 설계도에 0.15 적었으니 그거 쓰면 됨

    // 벡터를 루프 밖에서 딱 한 번만 선언
    // 함수가 실행될 때 메모리를 한 번 확보하면, 루프가 돌아도 계속 재사용
    std::vector<Point2D> cluster_pts;
    // 평균적인 클러스터 점 개수가 100개 정도라면 미리 예약해두면 더 빠름
    cluster_pts.reserve(500);

    geometry_msgs::TransformStamped tf_stamped;
    try {
        // tfBuffer 사용은 동일
        tf_stamped = tfBuffer.lookupTransform("base_link", "Lidar3D-3", ros::Time(0)); //지금 당장 라이다 기준 데이터를 자동차 중심 기준으로 바꾸려면 어디로 얼마나 움직이고 회전해야 하는지 공식 알려줘
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF2 변환 실패: %s", ex.what());
        return; //이번 루프는 계산을 못하니까 그냥 여기서 끝내고 다음 데이터가 올때까지 기다리기
    }// 성공적으로 정보를 가져오면 tf_stamped라는 변수에 저장(여기에는 라이다가 자동차 중심으로부터 x,y,z로 얼마나 떨어져있는지)랑 얼마나 삐딱하게 달려있는지

    // lookupTransform 이후, 한 번만 변환행렬 만들기
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    {
        const auto& tr = tf_stamped.transform;

        Eigen::Quaterniond q(
            tr.rotation.w,
            tr.rotation.x,
            tr.rotation.y,
            tr.rotation.z
        );
        T.linear() = q.toRotationMatrix();
        T.translation() = Eigen::Vector3d(
            tr.translation.x,
            tr.translation.y,
            tr.translation.z
        );
    }

    st_LiDAR.vec_Objects.clear(); // 안비우면 rviz에 과거 물체 잔상 남으니까 결과 담을 바구니 비우기

    // Clustering 결과(indices)를 하나씩 꺼내서 루프 돌리기
    for (const pcl::PointIndices& group : st_LiDAR.cluster_indices)  //합격자 명단에 있는 그룹들만 하나씩 꺼내기
        {
            // [핵심] 루프가 시작될 때마다 새로 만들지 않고 비우기만 합니다.
            // clear()는 메모리 공간(Capacity)은 남겨두고 데이터만 지우기 때문에 
            // 다음 점들을 담을 때 새로 메모리를 할당(malloc)할 필요가 없습니다.
            cluster_pts.clear();

            float min_z = std::numeric_limits<float>::max();
            float max_z = -std::numeric_limits<float>::max(); //어떤값이든 무조건 첫번째에 바로 업데이트 하기 위해

            // 루프 안에서는 행렬 곱만
            for (const int& idx : group.indices) {
                const pcl::PointXYZ& p_pcl = st_LiDAR.pcl_NonGroundCloud->points[idx];

                Eigen::Vector3d p_lidar(p_pcl.x, p_pcl.y, p_pcl.z);
                Eigen::Vector3d p_base = T * p_lidar;
                // Eigen::Vector3d p_base = p_lidar;  // TF 제거


                Point2D p;
                p.x = p_base.x();
                p.y = p_base.y();
                cluster_pts.push_back(p);

                if (p_base.z() < min_z) min_z = p_base.z();
                if (p_base.z() > max_z) max_z = p_base.z();
            }


            Rectangle rect;
            if (fitter.fit(cluster_pts, rect)) {
                Lbox obj; //지역변수

                // 1. 중심점 계산: 대각선 방향 모서리(0번과 2번)의 평균
                obj.f32_X_center = (rect.corners[0].x() + rect.corners[1].x() + rect.corners[2].x() + rect.corners[3].x()) / 4.0;
                obj.f32_Y_center = (rect.corners[0].y() + rect.corners[1].y() + rect.corners[2].y() + rect.corners[3].y()) / 4.0;
                
                // 라이다가 기준점일 때, 점들이 찍힌 실제 Z 범위를 계산하여 그 중간을 중심점으로 잡습니다.
                obj.f32_H        = max_z - min_z;           // 물체의 실제 높이
                obj.f32_Z_center = (min_z + max_z) / 2.0;   // 상자의 수직 중심점

                double side1 = (rect.corners[0] - rect.corners[1]).norm();
                double side2 = (rect.corners[1] - rect.corners[2]).norm();

                // 2. 긴 쪽을 항상 L(Length)로, 짧은 쪽을 W(Width)로 매칭합니다.
                if (side2 >= side1) {
                    // side1이 더 긴 경우: 원래 헤딩을 유지
                    obj.f32_L = side2;
                    obj.f32_W = side1;
                    obj.f32_Heading = rect.heading;
                } else {
                    // side2가 더 긴 경우: 가로세로를 바꾸고 헤딩을 90도(PI/2) 회전
                    obj.f32_L = side1;
                    obj.f32_W = side2;
                    obj.f32_Heading = rect.heading + (M_PI / 2.0); //기준이 되는 축이 90도 돌아갔으므로 물체의 방향도 90도를 돌려줘야 이물체는 긴 방향으로 서있다라는게 정확하게 표현가능
                }

                // OBB 기반 가드레일 제거
                if (obj.f32_L > 12.0f) continue;
                if (obj.f32_L > 3.0f && obj.f32_W < 0.7f) continue; 
                // if (obj.f32_L < 2.5f && obj.f32_W < 1.0f && obj.f32_H > 0.7f) continue; 

                // std::cout << "[L-Shape] Object Detected! " 
                // << "X: " << obj.f32_X << ", Y: " << obj.f32_Y 
                // << ", L: " << obj.f32_L << ", W: " << obj.f32_W 
                // << ", Heading: " << obj.f32_Heading << std::endl;



                st_LiDAR.vec_Objects.push_back(obj);
            }
        }
}