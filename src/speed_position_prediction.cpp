#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <vector>
#include <map>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// ======== Kalman Tracker =========
class KalmanTracker {
public:
    Vector4f state;   // [x, y, vx, vy]
    Matrix4f P;
    bool initialized = false;
    int id = -1;
    float initial_x = 0.0;
    float total_distance = 0.0;
    float last_x = 0.0;
    bool first_update = true;

    void init(float x, float y, int tracker_id) {
        state << x, y, 0, 0;
        P = Matrix4f::Identity();
        initialized = true;
        id = tracker_id;
        initial_x = x;
        last_x = x;
        total_distance = 0.0;
        first_update = true;
    }

    void predict(float dt = 0.1) {
        Matrix4f F = Matrix4f::Identity();
        F(0, 2) = dt;
        F(1, 3) = dt;
        Matrix4f Q = Matrix4f::Identity() * 0.01;
        state = F * state;
        P = F * P * F.transpose() + Q;
    }

    void update(float x, float y) {
        Vector2f z(x, y);
        Matrix<float, 2, 4> H;
        H << 1, 0, 0, 0,
             0, 1, 0, 0;
        Vector2f yk = z - H * state;
        Matrix2f R = Matrix2f::Identity() * 0.05;
        Matrix2f S = H * P * H.transpose() + R;
        Matrix<float, 4, 2> K = P * H.transpose() * S.inverse();
        state = state + K * yk;
        P = (Matrix4f::Identity() - K * H) * P;

        // Toplam yol hesaplama
        if (!first_update) {
            float dx = state(0) - last_x;
            if (abs(dx) > 0.002)
                total_distance += dx;
            else
                total_distance += 0.0;    
        }
        last_x = state(0);
        first_update = false;
    }

    pair<float, float> position() const {
        return {state(0), state(1)};
    }

    float getTotalDistance() const {
        return total_distance;
    }
};

// ======== Scan Cluster Tracker + Hız =========
class ScanClusterVelocity {
private:
    ros::NodeHandle nh;

    // Scan & Marker
    ros::Subscriber scan_sub;
    ros::Publisher marker_pub;
    ros::Publisher center_pub;
    ros::Publisher gui_pub;  

    ros::Subscriber center_sub;
    double last_x = 0.0;
    double last_time = 0.0;
    bool first = true;
    const double noise_threshold = 0.02;
    const double dt_min = 0.05;
    double current_vx = 0.0;

    vector<KalmanTracker> trackers;
    int next_tracker_id = 0;
    const float cluster_dist_threshold = 1.5;
    const float match_dist_threshold = 1.95;
    const int max_missed_frames = 5;
    map<int, int> tracker_missed_frames;
    vector<vector<pair<float, float>>> last_clusters;

    const double box_width = 1.0;
    const double box_length = 1.0;

public:
    ScanClusterVelocity() {
        scan_sub = nh.subscribe("/scan_filtered", 1, &ScanClusterVelocity::scanCallback, this);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1);
        center_pub = nh.advertise<geometry_msgs::PoseArray>("/merkez_point", 1);
        gui_pub = nh.advertise<std_msgs::Float64MultiArray>("/tracking_data", 1);

        center_sub = nh.subscribe("/merkez_point", 1, &ScanClusterVelocity::velocityCallback, this);

        ROS_INFO("Cluster ve velocity node baslatildi.");
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<pair<float, float>> points;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float r = scan->ranges[i];
            if (std::isfinite(r)) {
                float angle = scan->angle_min + i * scan->angle_increment;
                points.emplace_back(r * cos(angle), r * sin(angle));
            }
        }
        if (points.empty()) return;

        vector<vector<pair<float, float>>> clusters;
        vector<bool> visited(points.size(), false);

        for (size_t i = 0; i < points.size(); ++i) {
            if (visited[i]) continue;
            vector<pair<float, float>> cluster = {points[i]};
            visited[i] = true;
            for (size_t j = i + 1; j < points.size(); ++j) {
                float dx = points[j].first - cluster.back().first;
                float dy = points[j].second - cluster.back().second;
                if (sqrt(dx*dx + dy*dy) < cluster_dist_threshold) {
                    cluster.push_back(points[j]);
                    visited[j] = true;
                }
            }
            clusters.push_back(cluster);
        }

        vector<pair<float, float>> centers;
        for (auto &c : clusters) {
            float mx=0, my=0;
            for (auto &p : c) { mx+=p.first; my+=p.second; }
            mx/=c.size(); my/=c.size();
            centers.emplace_back(mx,my);
        }

        last_clusters = clusters;
        trackClusters(centers);
        publishMarkersAndCenters();
        publishGUIData();
    }

    void trackClusters(const vector<pair<float, float>>& centers) {
        float dt = 0.1;
        for (auto &t : trackers) if (t.initialized) t.predict(dt);

        vector<bool> matched(centers.size(), false);
        vector<bool> tracker_matched(trackers.size(), false);

        for (size_t t_idx = 0; t_idx < trackers.size(); ++t_idx) {
            auto &t = trackers[t_idx];
            auto [tx, ty] = t.position();
            float best_dist = 9999;
            int best_idx = -1;

            for (size_t i = 0; i < centers.size(); ++i) {
                if (matched[i]) continue;
                float dx = centers[i].first - tx;
                float dy = centers[i].second - ty;
                float d = sqrt(dx*dx + dy*dy);
                if (d < best_dist && d < match_dist_threshold) {
                    best_dist = d;
                    best_idx = i;
                }
            }

            if (best_idx != -1) {
                t.update(centers[best_idx].first, centers[best_idx].second);
                matched[best_idx] = true;
                tracker_matched[t_idx] = true;
                tracker_missed_frames[t.id] = 0;
            } else {
                tracker_missed_frames[t.id]++;
            }
        }

        // Kayıp tracker'ları sil
        trackers.erase(
            remove_if(trackers.begin(), trackers.end(), [&](const KalmanTracker& t) {
                return tracker_missed_frames[t.id] > max_missed_frames;
            }),
            trackers.end()
        );

        for (size_t i = 0; i < centers.size(); ++i) {
            if (!matched[i]) {
                KalmanTracker kf;
                kf.init(centers[i].first, centers[i].second, next_tracker_id++);
                tracker_missed_frames[kf.id] = 0;
                trackers.push_back(kf);
            }
        }
    }

    // ===== Publish Marker & PoseArray =====
    void publishMarkersAndCenters() {
        visualization_msgs::MarkerArray markers;
        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "laser_link";
        pose_array.header.stamp = ros::Time::now();
        int id = 0;

        for (auto &c : last_clusters) {
            if (c.empty()) continue;
            float cx = 0, cy = 0;
            for (auto &p : c) { cx += p.first; cy += p.second; }
            cx /= c.size(); cy /= c.size();

            // Merkez noktasi
            visualization_msgs::Marker center;
            center.header.frame_id = "laser_link";
            center.header.stamp = ros::Time::now();
            center.ns = "cluster_center";
            center.id = id++;
            center.type = visualization_msgs::Marker::SPHERE;
            center.action = visualization_msgs::Marker::ADD;
            center.pose.position.x = cx;
            center.pose.position.y = cy;
            center.pose.position.z = 0.2;
            center.scale.x = 0.08; center.scale.y = 0.08; center.scale.z = 0.15;
            center.color.a = 1.0; center.color.r = 1.0; center.color.g = 0.4; center.color.b = 0.2;
            markers.markers.push_back(center);

            visualization_msgs::Marker box;
            box.header.frame_id = "laser_link";
            box.header.stamp = ros::Time::now();
            box.ns = "stable_box";
            box.id = id++;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.pose.position.x = cx;
            box.pose.position.y = cy;
            box.pose.position.z = 0.0;
            box.pose.orientation.w = 1.0;
            box.scale.x = box_length / 1.8;
            box.scale.y = box_width;
            box.scale.z = 1.0;
            box.color.a = 0.3;
            box.color.r = 0.0;
            box.color.g = 2.0;
            box.color.b = 0.0;
            markers.markers.push_back(box);

            // Tracker'dan toplam yol bilgisini al
            float total_dist = 0.0;
            for (auto &t : trackers) {
                auto [tx, ty] = t.position();
                float dx = tx - cx;
                float dy = ty - cy;
                if (sqrt(dx*dx + dy*dy) < 0.3) {
                    total_dist = t.getTotalDistance();
                    break;
                }
            }

            visualization_msgs::Marker velocity_text;
            velocity_text.header.frame_id = "laser_link";
            velocity_text.header.stamp = ros::Time::now();
            velocity_text.ns = "velocity_text";
            velocity_text.id = id++;
            velocity_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            velocity_text.action = visualization_msgs::Marker::ADD;
            velocity_text.pose.position.x = cx;
            velocity_text.pose.position.y = cy;
            velocity_text.pose.position.z = 1.5;
            velocity_text.scale.z = 0.2;
            velocity_text.color.a = 1.0;
            velocity_text.color.r = 1.0;
            velocity_text.color.g = 1.0;
            velocity_text.color.b = 1.0;

            char vel_str[100];
            snprintf(vel_str, sizeof(vel_str), "Vx: %.2f m/s | Yol: %.2f m", current_vx, total_dist);
            velocity_text.text = vel_str;
            markers.markers.push_back(velocity_text);

            geometry_msgs::Pose pose;
            pose.position.x = cx;
            pose.position.y = cy;
            pose.position.z = 0.0;
            pose_array.poses.push_back(pose);
        }

        marker_pub.publish(markers);
        center_pub.publish(pose_array);
    }

    // ===== GUI Data Publisher =====
    void publishGUIData() {
        std_msgs::Float64MultiArray msg;
        float total_dist = 0.0;
        float pos_x = 0.0;

        if (!trackers.empty() && trackers[0].initialized) {
            total_dist = trackers[0].getTotalDistance();
            auto [x, y] = trackers[0].position();
            pos_x = x;
        }

        msg.data = {current_vx, total_dist, pos_x};
        gui_pub.publish(msg);
    }

    // ===== Velocity Callback =====
    void velocityCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        if (msg->poses.empty()) return;

        double x = msg->poses[0].position.x;
        double t = msg->header.stamp.toSec();

        if (first) { last_x = x; last_time = t; first = false; return; }

        double dt = t - last_time;
        if (dt < dt_min) return;

        double dx = x - last_x;
        double vx = dx / dt;

        if (fabs(vx) < noise_threshold) vx = 0.0;

        current_vx = vx;

        ROS_INFO_STREAM("Merkez x: " << x << "  |  vx: " << vx << " m/s");

        last_x = x;
        last_time = t;
    }
};

// ===== Main =====
int main(int argc, char** argv) {
    ros::init(argc, argv, "cluster_track_velocity");
    ScanClusterVelocity node;
    ros::spin();
    return 0;
}