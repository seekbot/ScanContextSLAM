#pragma once
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

using SCPointType = pcl::PointXYZI;
using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

class SCManager {
public:
    SCManager() = default;
    Eigen::MatrixXd makeScancontext(const pcl::PointCloud<SCPointType>& scan);
    Eigen::MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd& desc);
    Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd& desc);
    int fastAlignUsingVkey(Eigen::MatrixXd& v1, Eigen::MatrixXd& v2);
    double distDirectSC(Eigen::MatrixXd& sc1, Eigen::MatrixXd& sc2);
    std::pair<double,int> distanceBtnScanContext(Eigen::MatrixXd& a, Eigen::MatrixXd& b);
    void makeAndSaveScancontextAndKeys(const pcl::PointCloud<SCPointType>& scan);
    std::pair<int,float> detectLoopClosureIDGivenScan(const pcl::PointCloud<SCPointType>& scan);

    /* == Hyperparameters == */
    const double LIDAR_HEIGHT = 2.0;    // lift so all z > 0
    // add this to directly use the lidar scan in the local lidar coord (not robot base coord)
    // otherwise, set it directly to 0 if in robot-coord-transformed lidar scans.

    const int PC_NUM_RING = 20;                 // range bins (rows)
    const int PC_NUM_SECTOR = 60;               // azimuth bins (cols)
    double PC_MAX_RADIUS = 80.0;                // meters; TUNE accordingly to sensor (80.0 in Velodyne/Ouster)
    const double PC_UNIT_SECTOR_ANGLE = 360.0/double(PC_NUM_SECTOR);
    const double PC_UNIT_RING_GAP = 360.0/double(PC_NUM_RING);

    // Tree
   const int NUM_EXCLUDE_RECENT = 30;       // never match last N keyframes
    // simply just keyframe gap, but node position distance-based exclusion is ok.
    // const int NUM_CANDIDATES_FROM_TREE = 10;
    const int NUM_CANDIDATES_FROM_TREE = 50;

    // Loop thres.
    const double SEARCH_RATIO = 0.1;    // fine-search bandwidth for fast comparison, no Brute-Force, but search 10% is ok
    double SC_DIST_THRES = 0.2;         // loop accept threshold
    // empirically 0.1~0.2 is fine (rare false-alarms) for 20x60 polar context (but for >0.15, DCS or ICP fit score check)
    // E.g., should be required for robustness in LeGO-LOAM
    // 0.4~0.6 is good for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold. If not, 0.1~0.15 recommended.

    // K-d tree generation config
    const int TREE_MAKING_PERIOD_ = 10;      // rebuild kd-tree every Nth call to avoid non-mandatory every remaking (save time cost)
    // to find recent visits, use small value of it (it is fast enough ~5-5.5ms wrt N.)
    int tree_making_period_counter = 0;

    // Storage: Parallel arrays, index = keyframe id
    std::vector<double> polarContexts_timestamp_;           // Optional
    std::vector<Eigen::MatrixXd> polarContexts_;            // 20x60 descriptor
    std::vector<Eigen::MatrixXd> polarContexts_invKeys_;
    KeyMat polarContext_invKeys_Mat_;                        // ring keys for tree
    KeyMat polarContext_invKeys_to_Search_;
    std::unique_ptr<InvKeyTree> polarContext_Tree_;

};