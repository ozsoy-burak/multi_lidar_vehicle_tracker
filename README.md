# Gazebo LiDAR Vehicle Detection & Tracking

A ROS + Gazebo simulation environment for overhead 2D LiDAR-based vehicle detection, multi-object tracking, and velocity estimation. Five 2D LiDAR sensors are mounted on a fixed station 5 meters above a road scene containing controllable vehicle models (hatchback, SUV, etc.). Detected vehicles are tracked in real time using a Kalman filter with oriented bounding box estimation.

---

## Simulation Environment

The Gazebo world contains a structured road layout with lane markings and multiple vehicle models placed as dynamic actors in the rear scene. Each vehicle is controlled via `cmd_vel` through a dedicated ROS node. The LiDAR station is defined in URDF and consists of 5 planar (2D) laser scanners arranged to cover the road area from above.

```
Gazebo World
├── Road scene with lane markings
├── Vehicle models (hatchback, SUV, ...)  ← cmd_vel controlled
└── LiDAR station @ +5m height (URDF)
    ├── /scan   (horizontal)
    ├── /scan2  (vertical — right & left split)
    ├── /scan3  (horizontal)
    ├── /scan4  (vertical — right)
    └── /scan5  (vertical — left)
```

---

## Software Architecture

```
├── multi_laser_filter.cpp           # Multi-LiDAR filtering node
└── laser_vehicle_cluster_tracker.cpp  # Clustering, tracking, and velocity estimation
```

---

## Nodes

### `multi_laser_filter`

Subscribes to all 5 raw scan topics and publishes filtered versions. Each scan is processed through a two-stage pipeline:

**Stage 1 — Geometric filtering:**
Each scanner has a `FilterConfig` struct defining its type (`HORIZONTAL` or `VERTICAL`) and spatial bounding limits. Points falling outside the height and lateral bounds are discarded. This removes ground returns, ceiling hits, and out-of-range reflections.

| Topic | Type | Height range | Lateral range |
|---|---|---|---|
| `/scan` | Horizontal | max 4.72 m | — |
| `/scan2` | Vertical | 0.1 – 3.6 m | right: 0.0 – 2.35 m / left: −2.20 – 0.0 m |
| `/scan3` | Horizontal | max 4.72 m | — |
| `/scan4` | Vertical | 0.1 – 3.6 m | 0.3 – 3.0 m |
| `/scan5` | Vertical | 0.1 – 3.6 m | −3.0 – −0.3 m |

**Stage 2 — Temporal filtering:**
After geometric filtering, each range reading is compared to the previous frame. If the height difference exceeds the configured threshold (2.4 m for horizontal, 1.6 m for vertical scans), the measurement is classified as a spurious jump and the previous value is kept. Valid readings are smoothed with an Exponential Moving Average (EMA):

- Horizontal scans: α = 0.5
- Vertical scans: α = 0.1

`/scan2` is split into two independent outputs (`/scan2_sag_filtered`, `/scan2_sol_filtered`), each with its own previous-frame state to avoid cross-contamination.

**Published topics:**
`/scan_filtered`, `/scan2_sag_filtered`, `/scan2_sol_filtered`, `/scan3_filtered`, `/scan4_filtered`, `/scan5_filtered`

---

### `laser_vehicle_cluster_tracker`

Subscribes to `/scan_filtered` and performs clustering, bounding box estimation, multi-object tracking, and velocity estimation.

**Clustering (BFS-based DBSCAN):**
Finite range readings are converted to Cartesian (x, y) points. A BFS expansion groups nearby points into clusters using a distance threshold of 0.5 m. Clusters with fewer than 5 points are discarded as noise.

**Oriented Bounding Box (OBB):**
For each cluster, PCA is applied via Eigen's `SelfAdjointEigenSolver` on the 2D covariance matrix. The cluster is projected onto the principal axes to compute length and width. Box dimensions are smoothed over time using EMA (factor = 0.2) to reduce per-frame jitter.

**Kalman Tracking (`KalmanTracker`):**
Each cluster centroid is matched to an existing tracker using nearest-neighbour association (threshold: 1.0 m). The state vector is `[x, y, vx, vy]` with a constant-velocity motion model:

- Process noise: Q = 0.01 · I
- Measurement noise: R = 0.05 · I

Trackers that go unmatched for more than 5 consecutive frames are removed. New clusters spawn new trackers with incrementing IDs. Each tracker accumulates total travel distance along the x-axis.

**Velocity estimation:**
A derivative-based velocity estimate is computed from the published `/merkez_point` PoseArray, with a minimum dt of 0.05 s and a noise gate of 0.02 m/s.

**Published topics:**

| Topic | Type | Content |
|---|---|---|
| `cluster_markers` | `MarkerArray` | Bounding boxes, center spheres, ID and velocity text |
| `/merkez_point` | `PoseArray` | Cluster centroids in `rplidar_link` frame |
| `tracked_points` | `PointCloud2` | Points inside active bounding boxes |
| `/tracking_data` | `Float64MultiArray` | `[vx, total_distance, pos_y]` for first tracker |

---

## Key Parameters

| Parameter | Value | Description |
|---|---|---|
| `cluster_dist_threshold` | 0.5 m | BFS neighborhood radius |
| `match_dist_threshold` | 1.0 m | Max distance for tracker–cluster association |
| `max_missed_frames` | 5 | Frames before a tracker is dropped |
| `smoothing_factor` (OBB) | 0.2 | EMA factor for bounding box dimensions |
| `alpha` (horizontal) | 0.5 | EMA factor for scan smoothing |
| `alpha` (vertical) | 0.1 | EMA factor for scan smoothing |
| Temporal jump threshold | 2.4 m / 1.6 m | Height spike rejection (horiz. / vert.) |

---

## Dependencies

- ROS Noetic
- Gazebo
- Eigen3
- PCL (pcl_ros)
- sensor_msgs, visualization_msgs, geometry_msgs, std_msgs
