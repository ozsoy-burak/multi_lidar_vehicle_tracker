/*
 * =====================================================================================
 * @file      multi_laser_filter.cpp
 * @brief     Multi-LiDAR filtering node with temporal and geometric constraints.
 * @details   Consolidates multiple LaserScan streams, applies geometric bounding boxes,
 * and utilizes temporal exponential smoothing to eliminate sensor noise.
 * =====================================================================================
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <limits>

const float INF = std::numeric_limits<float>::infinity();
const float GLOBAL_LATERAL_JUMP_THR = 10.0f;

enum ScanType { HORIZONTAL, VERTICAL };

struct FilterConfig {
    ScanType type;
    float max_height;
    float min_height;
    float max_lateral;
    float min_lateral;
    float temporal_height_thr;
    float alpha;
};

// {Tarama Tipi, Max Yükseklik, Min Yükseklik, Max Yanal, Min Yanal, Sıçrama Eşiği, EMA Düzeltme Katsayısı}
const FilterConfig CFG_SCAN1     = {HORIZONTAL, 4.72f, -INF, -INF, INF, 2.4f, 0.5f};
const FilterConfig CFG_SCAN2_SAG = {VERTICAL, 3.6f, 0.1f, 2.35f, 0.0f, 1.6f, 0.1f};
const FilterConfig CFG_SCAN2_SOL = {VERTICAL, 3.6f, 0.1f, 0.0f, -2.20f, 1.6f, 0.1f};
const FilterConfig CFG_SCAN3     = {HORIZONTAL, 4.72f, -INF, -INF, INF, 2.4f, 0.5f};
const FilterConfig CFG_SCAN4     = {VERTICAL, 3.6f, 0.1f, 3.0f, 0.3f, 1.6f, 0.1f};
const FilterConfig CFG_SCAN5     = {VERTICAL, 3.6f, 0.1f, -0.3f, -3.0f, 1.6f, 0.1f};

class MultiLaserFilter {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub1_, sub2_, sub3_, sub4_, sub5_;
    ros::Publisher pub1_, pub2_sag_, pub2_sol_, pub3_, pub4_, pub5_;
    
    std::vector<float> prev_ranges1_, prev_ranges2_sag_, prev_ranges2_sol_, prev_ranges3_, prev_ranges4_, prev_ranges5_;
    bool first1_ = true, first2_sag_ = true, first2_sol_ = true, first3_ = true, first4_ = true, first5_ = true;

    void processAndPublish(const sensor_msgs::LaserScan::ConstPtr& msg, ros::Publisher& pub, 
                           std::vector<float>& prev_ranges, bool& first_scan, const FilterConfig& cfg) {
        sensor_msgs::LaserScan filtered = *msg;
        size_t size = filtered.ranges.size();

        if (first_scan || prev_ranges.size() != size) {
            prev_ranges = filtered.ranges;
            first_scan = false;
        }

        for (size_t i = 0; i < size; ++i) {
            float curr = filtered.ranges[i];
            
            if (!std::isfinite(curr)) {
                filtered.ranges[i] = INF;
                prev_ranges[i] = curr;
                continue;
            }

            float angle = filtered.angle_min + i * filtered.angle_increment;
            float height = 0.0f;
            float lateral = 0.0f;

            if (cfg.type == HORIZONTAL) {
                height = curr * std::sin(angle);
                if (height > cfg.max_height) {
                    filtered.ranges[i] = INF;
                    prev_ranges[i] = curr;
                    continue;
                }
            } else {
                height = curr * std::cos(angle);
                lateral = curr * std::sin(angle);
                if (height > cfg.max_height || height < cfg.min_height || 
                    lateral > cfg.max_lateral || lateral < cfg.min_lateral) {
                    filtered.ranges[i] = INF;
                    prev_ranges[i] = curr;
                    continue;
                }
            }

            float prev = prev_ranges[i];
            if (!std::isfinite(prev)) {
                prev_ranges[i] = curr;
                continue;
            }

            float prev_height = (cfg.type == HORIZONTAL) ? (prev * std::sin(angle)) : (prev * std::cos(angle));
            float height_diff = std::abs(height - prev_height);
            bool jump_detected = (height_diff > cfg.temporal_height_thr);

            if (cfg.type == VERTICAL) {
                float prev_lateral = prev * std::sin(angle);
                float lateral_diff = std::abs(lateral - prev_lateral);
                if (lateral_diff > GLOBAL_LATERAL_JUMP_THR) {
                    jump_detected = true;
                }
            }

            if (jump_detected) {
                filtered.ranges[i] = prev;
            } else {
                filtered.ranges[i] = cfg.alpha * curr + (1.0f - cfg.alpha) * prev;
            }

            prev_ranges[i] = filtered.ranges[i];
        }

        pub.publish(filtered);
    }

    void cb1(const sensor_msgs::LaserScan::ConstPtr& msg) { processAndPublish(msg, pub1_, prev_ranges1_, first1_, CFG_SCAN1); }
    void cb2(const sensor_msgs::LaserScan::ConstPtr& msg) { 
        processAndPublish(msg, pub2_sag_, prev_ranges2_sag_, first2_sag_, CFG_SCAN2_SAG); 
        processAndPublish(msg, pub2_sol_, prev_ranges2_sol_, first2_sol_, CFG_SCAN2_SOL); 
    }
    void cb3(const sensor_msgs::LaserScan::ConstPtr& msg) { processAndPublish(msg, pub3_, prev_ranges3_, first3_, CFG_SCAN3); }
    void cb4(const sensor_msgs::LaserScan::ConstPtr& msg) { processAndPublish(msg, pub4_, prev_ranges4_, first4_, CFG_SCAN4); }
    void cb5(const sensor_msgs::LaserScan::ConstPtr& msg) { processAndPublish(msg, pub5_, prev_ranges5_, first5_, CFG_SCAN5); }

public:
    MultiLaserFilter() {
        sub1_ = nh_.subscribe("/scan", 10, &MultiLaserFilter::cb1, this);
        sub2_ = nh_.subscribe("/scan2", 10, &MultiLaserFilter::cb2, this);
        sub3_ = nh_.subscribe("/scan3", 10, &MultiLaserFilter::cb3, this);
        sub4_ = nh_.subscribe("/scan4", 10, &MultiLaserFilter::cb4, this);
        sub5_ = nh_.subscribe("/scan5", 10, &MultiLaserFilter::cb5, this);

        pub1_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 10);
        pub2_sag_ = nh_.advertise<sensor_msgs::LaserScan>("/scan2_sag_filtered", 10);
        pub2_sol_ = nh_.advertise<sensor_msgs::LaserScan>("/scan2_sol_filtered", 10);
        pub3_ = nh_.advertise<sensor_msgs::LaserScan>("/scan3_filtered", 10);
        pub4_ = nh_.advertise<sensor_msgs::LaserScan>("/scan4_filtered", 10);
        pub5_ = nh_.advertise<sensor_msgs::LaserScan>("/scan5_filtered", 10);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_laser_filter");
    MultiLaserFilter filter;
    ros::spin();
    return 0;
}