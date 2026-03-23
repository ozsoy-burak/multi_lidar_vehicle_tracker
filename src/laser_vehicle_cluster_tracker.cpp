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
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace Eigen;

struct OBB {
    pair<float, float> center;
    pair<float, float> dimensions;
    geometry_msgs::Quaternion orientation;
};

class KalmanTracker {
public:
    Vector4f state;
    Matrix4f P;
    bool initialized = false;
    int id = -1;
    float total_distance = 0.0;
    float last_x = 0.0;
    bool first_update = true;

    float smoothed_length = 0.0;
    float smoothed_width = 0.0;
    bool dimensions_initialized = false;
    float smoothing_factor = 0.2f; 

    int matched_cluster_idx = -1;

    void init(float x, float y, int tracker_id) {
        state << x, y, 0, 0;
        P = Matrix4f::Identity();
        initialized = true;
        id = tracker_id;
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

        if (!first_update) {
            float dx = state(0) - last_x;
            if (abs(dx) > 0.002) total_distance += dx;
        }
        last_x = state(0);
        first_update = false;
    }

    void updateAppearance(const OBB& obb) {
        float new_length = obb.dimensions.first;
        float new_width = obb.dimensions.second;

        if (!dimensions_initialized) {
            smoothed_length = new_length;
            smoothed_width = new_width;
            dimensions_initialized = true;
        } else {
            smoothed_length = smoothing_factor * new_length + (1.0 - smoothing_factor) * smoothed_length;
            smoothed_width = smoothing_factor * new_width + (1.0 - smoothing_factor) * smoothed_width;
        }
    }

    pair<float, float> position() const {
        return {state(0), state(1)};
    }

    float getTotalDistance() const {
        return total_distance;
    }
};

class ScanClusterVelocity {
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher marker_pub;
    ros::Publisher center_pub;
    ros::Publisher gui_pub;
    ros::Subscriber center_sub;
    ros::Publisher filtered_points_pub;

    double last_x = 0.0, last_time = 0.0;
    bool first = true;
    const double noise_threshold = 0.02;
    const double dt_min = 0.05;
    double current_vx = 0.0;

    vector<KalmanTracker> trackers;
    int next_tracker_id = 0;
    const float cluster_dist_threshold = 0.5;
    const float match_dist_threshold = 1.0;
    const int max_missed_frames = 5;
    map<int, int> tracker_missed_frames;
    vector<vector<pair<float, float>>> last_clusters;

public:
    ScanClusterVelocity() {
        scan_sub = nh.subscribe("/scan_filtered", 1, &ScanClusterVelocity::scanCallback, this);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1);
        center_pub = nh.advertise<geometry_msgs::PoseArray>("/merkez_point", 1);
        gui_pub = nh.advertise<std_msgs::Float64MultiArray>("/tracking_data", 1);
        center_sub = nh.subscribe("/merkez_point", 1, &ScanClusterVelocity::velocityCallback, this);
        filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>("tracked_points", 1);

        ROS_INFO("Cluster ve velocity node baslatildi.");
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        vector<pair<float, float>> points;
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            if (std::isfinite(scan->ranges[i])) {
                float angle = scan->angle_min + i * scan->angle_increment;
                points.emplace_back(scan->ranges[i] * cos(angle), scan->ranges[i] * sin(angle));
            }
        }
        if (points.empty()) return;

        vector<vector<pair<float, float>>> clusters;
        vector<bool> visited(points.size(), false);
        for (size_t i = 0; i < points.size(); ++i) {
            if (visited[i]) continue;
            vector<pair<float, float>> current_cluster;
            vector<size_t> queue;
            queue.push_back(i);
            visited[i] = true;
            while(!queue.empty()){
                size_t current_idx = queue.front();
                queue.erase(queue.begin());
                current_cluster.push_back(points[current_idx]);
                for(size_t j = 0; j < points.size(); ++j){
                    if(visited[j]) continue;
                    float dx = points[current_idx].first - points[j].first;
                    float dy = points[current_idx].second - points[j].second;
                    if(sqrt(dx*dx + dy*dy) < cluster_dist_threshold){
                        visited[j] = true;
                        queue.push_back(j);
                    }
                }
            }
            if(current_cluster.size() > 5) {
                 clusters.push_back(current_cluster);
            }
        }

        vector<pair<float, float>> centers;
        for (const auto &c : clusters) {
            float mx = 0, my = 0;
            for (const auto &p : c) { mx += p.first; my += p.second; }
            centers.emplace_back(mx / c.size(), my / c.size());
        }

        last_clusters = clusters;
        trackClusters(centers);
        publishMarkersAndCenters();
        publishGUIData();
    }

    void trackClusters(const vector<pair<float, float>>& centers) {
        for (auto &t : trackers) if (t.initialized) t.predict(0.1);

        vector<bool> matched(centers.size(), false);
        vector<bool> tracker_matched(trackers.size(), false);

        for (size_t t_idx = 0; t_idx < trackers.size(); ++t_idx) {
            auto &t = trackers[t_idx];
            
            pair<float, float> pos = t.position();
            float tx = pos.first;
            float ty = pos.second;
            
            float best_dist = 9999;
            int best_idx = -1;
            for (size_t i = 0; i < centers.size(); ++i) {
                if (matched[i]) continue;
                float d = sqrt(pow(centers[i].first - tx, 2) + pow(centers[i].second - ty, 2));
                if (d < best_dist && d < match_dist_threshold) {
                    best_dist = d;
                    best_idx = i;
                }
            }
            if (best_idx != -1) {
                t.update(centers[best_idx].first, centers[best_idx].second);
                OBB obb = calculateOrientedBoundingBox(last_clusters[best_idx]);
                t.updateAppearance(obb);
                t.matched_cluster_idx = best_idx;
                matched[best_idx] = true;
                tracker_matched[t_idx] = true;
                tracker_missed_frames[t.id] = 0;
            } else {
                tracker_missed_frames[t.id]++;
                t.matched_cluster_idx = -1;
            }
        }

        trackers.erase(
            remove_if(trackers.begin(), trackers.end(), [&](const KalmanTracker& t) {
                return tracker_missed_frames.count(t.id) && tracker_missed_frames[t.id] > max_missed_frames;
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

    OBB calculateOrientedBoundingBox(const vector<pair<float, float>>& cluster) {
        OBB result;
        if (cluster.size() < 2) return result;
        MatrixXf points(cluster.size(), 2);
        for (size_t i = 0; i < cluster.size(); ++i) {
            points(i, 0) = cluster[i].first;
            points(i, 1) = cluster[i].second;
        }
        Vector2f center = points.colwise().mean();
        result.center = {center(0), center(1)};
        MatrixXf centered_points = points.rowwise() - center.transpose();
        Matrix2f cov = (centered_points.transpose() * centered_points) / float(cluster.size());
        SelfAdjointEigenSolver<Matrix2f> eigensolver(cov);
        VectorXf proj1 = centered_points * eigensolver.eigenvectors().col(1);
        VectorXf proj2 = centered_points * eigensolver.eigenvectors().col(0);
        float padding = 0.1;
        result.dimensions.first = proj1.maxCoeff() - proj1.minCoeff() + padding;
        result.dimensions.second = proj2.maxCoeff() - proj2.minCoeff() + padding;
        
        // Yönelimi sıfırla (kutu düz olsun)
        result.orientation.x = 0.0;
        result.orientation.y = 0.0;
        result.orientation.z = 0.0;
        result.orientation.w = 1.0;
        
        return result;
    }

    void publishMarkersAndCenters() {
        visualization_msgs::MarkerArray markers;
        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "rplidar_link";
        pose_array.header.stamp = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZ> tracked_cloud;
        tracked_cloud.header.frame_id = "rplidar_link";

        for (const auto &t : trackers) {
            if (!t.initialized || !t.dimensions_initialized) continue;

            pair<float, float> pos = t.position();
            float cx = pos.first;
            float cy = pos.second;
            
            // Merkez noktası
            visualization_msgs::Marker center_marker;
            center_marker.header.frame_id = "rplidar_link";
            center_marker.header.stamp = ros::Time::now();
            center_marker.ns = "cluster_center";
            center_marker.id = t.id * 4;
            center_marker.type = visualization_msgs::Marker::SPHERE;
            center_marker.action = visualization_msgs::Marker::ADD;
            center_marker.pose.position.x = cx;
            center_marker.pose.position.y = cy;
            center_marker.pose.position.z = 0.2;
            center_marker.scale.x = 0.08; center_marker.scale.y = 0.08; center_marker.scale.z = 0.15;
            center_marker.color.r = 1.0; center_marker.color.g = 0.4; center_marker.color.b = 0.2; center_marker.color.a = 1.0;
            markers.markers.push_back(center_marker);

            // Düz kutu (yönelim sıfır)
            visualization_msgs::Marker box;
            box.header = center_marker.header;
            box.ns = "smoothed_dynamic_box";
            box.id = t.id * 4 + 1;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.pose.position.x = cx;
            box.pose.position.y = cy;
            box.pose.position.z = 0.0;
            // Kutuyu DÜZ tut (eğim yok)
            box.pose.orientation.x = 0.0;
            box.pose.orientation.y = 0.0;
            box.pose.orientation.z = 0.0;
            box.pose.orientation.w = 1.0;
            box.scale.x = t.smoothed_length;
            box.scale.y = t.smoothed_width;
            box.scale.z = 1.0;
            box.color.r = 0.0; box.color.g = 1.0; box.color.b = 0.5; box.color.a = 0.3;
            markers.markers.push_back(box);
            
            // Kutu içindeki noktaları filtreleme
            if (t.matched_cluster_idx != -1) {
                const auto& raw_cluster = last_clusters[t.matched_cluster_idx];
                float half_len = t.smoothed_length / 2.0f;
                float half_wid = t.smoothed_width / 2.0f;

                for (const auto& point : raw_cluster) {
                    float translated_x = point.first - cx;
                    float translated_y = point.second - cy;

                    if (abs(translated_x) <= half_len && abs(translated_y) <= half_wid) {
                        tracked_cloud.points.push_back(pcl::PointXYZ(point.first, point.second, 0.0));
                    }
                }
            }

            // ID gösterimi
            visualization_msgs::Marker id_text;
            id_text.header = center_marker.header;
            id_text.ns = "tracker_id";
            id_text.id = t.id * 4 + 2;
            id_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            id_text.action = visualization_msgs::Marker::ADD;
            id_text.pose.position.x = cx;
            id_text.pose.position.y = cy;
            id_text.pose.position.z = 1.8;
            id_text.scale.z = 0.25;
            id_text.color.r = 1.0; id_text.color.g = 1.0; id_text.color.b = 0.0; id_text.color.a = 1.0;
            char id_str[50];
            snprintf(id_str, sizeof(id_str), "ID: %d", t.id);
            id_text.text = id_str;
            markers.markers.push_back(id_text);

            // Hız ve yol gösterimi
            visualization_msgs::Marker velocity_text;
            velocity_text.header = center_marker.header;
            velocity_text.ns = "velocity_text";
            velocity_text.id = t.id * 4 + 3;
            velocity_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            velocity_text.action = visualization_msgs::Marker::ADD;
            velocity_text.pose.position.x = cx;
            velocity_text.pose.position.y = cy;
            velocity_text.pose.position.z = 1.5;
            velocity_text.scale.z = 0.2;
            velocity_text.color.r = 1.0; velocity_text.color.g = 1.0; velocity_text.color.b = 1.0; velocity_text.color.a = 1.0;
            char vel_str[100];
            snprintf(vel_str, sizeof(vel_str), "Vx: %.2f m/s | Yol: %.2f m", current_vx, t.getTotalDistance());
            velocity_text.text = vel_str;
            markers.markers.push_back(velocity_text);

            // PoseArray
            geometry_msgs::Pose pose;
            pose.position.x = cx;
            pose.position.y = cy;
            pose.orientation = box.pose.orientation;
            pose_array.poses.push_back(pose);
        }

        marker_pub.publish(markers);
        center_pub.publish(pose_array);
        
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(tracked_cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        filtered_points_pub.publish(cloud_msg);
    }
    
    void publishGUIData() {
        std_msgs::Float64MultiArray msg;
        float total_dist = 0.0;
        float pos_y = 0.0;
        if (!trackers.empty() && trackers[0].initialized) {
            total_dist = trackers[0].getTotalDistance();
            pair<float, float> pos = trackers[0].position();
            pos_y = pos.second;
        }
        msg.data.push_back(current_vx);
        msg.data.push_back(total_dist);
        msg.data.push_back(pos_y);
        gui_pub.publish(msg);
    }

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
        last_x = x;
        last_time = t;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cluster_track_velocity");
    ScanClusterVelocity node;
    ros::spin();
    return 0;
}