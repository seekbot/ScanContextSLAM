#include "Scancontext.h"
/* === Auxiliary functions (Helpers) === */
float deg2rad(float deg) {
    return (M_PI / 180.0) * deg;
}

float rad2deg(float rad) {
    return (180.0 / M_PI) * rad;
}

// (x,y) -> bearing in degrees [0, 360). Quadrant-correct
float xy2theta(const float& _x, const float& _y) {
    // Quadrant I
    if (_x >= 0 & _y >= 0) 
        return (180.0 / M_PI) * atan(_y / _x);
        
    // Quadrant II
    if (_x < 0 & _y >= 0)
        return 180.0 - ((180.0 / M_PI) * atan(_y / _x));

    // Quadrant III
    if (_x < 0 & _y < 0)
        return 180.0 + ((180.0 / M_PI) * atan(_y / _x));

    // Quadrant IV
    if (_x >= 0 & _y < 0)
        return 360.0 - ((180.0 / M_PI) * atan(_y / _x));
}

// Column-rotate matrix to the righ by N - simulates robot yaw rotation
Eigen::MatrixXd circshift(Eigen::MatrixXd& mat, int n_shift) {
    // Shift cols to the right
    assert (n_shift >= 0);
    if (n_shift == 0) return Eigen::MatrixXd(mat);

    Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(mat.rows(), mat.cols());
    for (int c_idx = 0; c_idx < mat.cols(); ++c_idx) {
        shifted_mat.col((c_idx + n_shift) % mat.cols()) = mat.col(c_idx);
    }
    return shifted_mat;
}

// Convert Eigen::Matrix to float vec.
std::vector<float> eig2stdvec(Eigen::MatrixXd eigenMatrix) {
    return std::vector<float>(eigenMatrix.data(), eigenMatrix.data() + eigenMatrix.size());
}

/* === Core Functions/Algorithms === */
Eigen::MatrixXd SCManager::makeScancontext(const pcl::PointCloud<SCPointType>& scan) {
    const int NO_POINT = -1000;     // sentinel for "empty bin"
    Eigen::MatrixXd desc = NO_POINT * 
        Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);  // 20x60, all -1000

    SCPointType pt;
    float azim_range, azim_angle;   // within 2d plane (Ring=y, Sector=x)
    int ring_idx, sector_idx;

    for (int i = 0; i < (int)scan.points.size(); ++i){
        pt.x = scan.points[i].x;
        pt.y = scan.points[i].y;
        pt.z = scan.points[i].z + LIDAR_HEIGHT;         // (1) lift to z > 0

        azim_range = std::sqrt(pt.x*pt.x + pt.y*pt.y);  // (2) radial dist.
        azim_angle = xy2theta(pt.x, pt.y);

        if (azim_range > PC_MAX_RADIUS) continue;   // (3) clip far pts

        // (4) bin: which ring (row), which sector (col). 1-indexed then -1
        ring_idx = std::max(std::min(PC_NUM_RING, 
                    (int)std::ceil((azim_range/PC_MAX_RADIUS)*PC_NUM_RING)), 1);
        sector_idx = std::max(std::min(PC_NUM_SECTOR,
                    (int)std::ceil((azim_angle/360.0)*PC_NUM_SECTOR)), 1);
        // Ring grows with distance; sector grows with heading

        // (5) the encoding: keep the TALLEST (max-z) pt in each bin
        if (desc(ring_idx-1, sector_idx-1) < pt.z) 
            desc(ring_idx-1, sector_idx-1) = pt.z;
    }

    // (6) empty bins: sentinel -> 0 (so cos dist treats them as nil)
    for (int r_idx = 0; r_idx < desc.rows(); ++r_idx)
        for (int c_idx = 0; c_idx < desc.cols(); ++c_idx)
            if (desc(r_idx, c_idx) == NO_POINT)
                desc(r_idx, c_idx) = 0;

    return desc;
}

// RING KEY - row-wise mean -> 20 vec. (ROTATION INVARIANT)
// Goes into kd-tree. Used to FIND candidates regardless of heading
Eigen::MatrixXd SCManager::makeRingkeyFromScancontext(Eigen::MatrixXd& desc){
    Eigen::MatrixXd invariant_key(desc.rows(), 1);
    for (int r_idx = 0; r_idx < desc.rows(); ++r_idx) {
        invariant_key(r_idx, 0) = desc.row(r_idx).mean();  // mean of all sectors per ring
    }

    return invariant_key;
}

// SECTOR KEY - col wise mean -> 60 vec. (NOT INVARIANT by design)
// Guesses the yaw shift quickly before the fine search
Eigen::MatrixXd SCManager::makeSectorkeyFromScancontext(Eigen::MatrixXd& desc) {
    Eigen::MatrixXd variant_key(1, desc.cols());

    for (int c_idx = 0; c_idx < desc.cols(); ++c_idx) {
        variant_key(0, c_idx) = desc.col(c_idx).mean(); // mean of all rings per sector
    }

    return variant_key;
}

// Column-wise cosine distance 
// Given two already-aligned descriptors, how different are they?
double SCManager::distDirectSC(Eigen::MatrixXd& sc1, Eigen::MatrixXd& sc2) {
    int num_eff_cols = 0;
    double sum_sector_sim = 0;
    for (int c_idx = 0; c_idx < sc1.cols(); ++c_idx) {
        Eigen::VectorXd col_sc1 = sc1.col(c_idx);
        Eigen::VectorXd col_sc2 = sc2.col(c_idx);

        if (col_sc1.norm() == 0 || col_sc2.norm() == 0) continue;   // skip empty sectors
        sum_sector_sim += col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());    // cosine per col
        ++num_eff_cols;
    }

    return 1.0 - (sum_sector_sim / num_eff_cols);   // 0 = identical
}

// Coarse yaw guess
int SCManager::fastAlignUsingVkey(Eigen::MatrixXd& vkey1, Eigen::MatrixXd& vkey2) {
    int argmin_vkey_shift = 0;
    double min_vkey_diff_norm = 1e11;

    // Try all 60 shifts
    for (int shift_idx = 0; shift_idx < vkey1.cols(); ++shift_idx){
        Eigen::MatrixXd vkey2_shifted = circshift(vkey2, shift_idx);
        double current_diff_norm = (vkey1 - vkey2_shifted).norm();   // L2 of sector-key diff
        if (current_diff_norm < min_vkey_diff_norm) {  
            min_vkey_diff_norm = current_diff_norm; 
            argmin_vkey_shift = shift_idx;
        }
    }

    return argmin_vkey_shift;    // rough col offset
}

// Coarse-to-fine
std::pair<double,int> SCManager::distanceBtnScanContext(
    Eigen::MatrixXd& sc1, Eigen::MatrixXd& sc2) 
{
    // 1. Coarse: one yaw guess from sector keys (fast align using variant key)
    Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(sc1);
    Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(sc2);
    int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

    // 2. Build small search band AROUND that guess (+/- 10%)
    const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * sc1.cols());   // 1/2 of search range
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for (int i = 1; i < SEARCH_RADIUS + 1; ++i) {
        shift_idx_search_space.push_back((argmin_vkey_shift + i + sc1.cols()) % sc1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift - i + sc1.cols()) % sc1.cols());    // +cols!
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 3. Fine: full cosine distance only at those few shifts
    int argmin_shift = 0;
    double min_sc_dist = 1e11;
    for (int n_shift:shift_idx_search_space) {
        Eigen::MatrixXd sc2_shifted = circshift(sc2, n_shift);
        double current_sc_dist = distDirectSC(sc1, sc2_shifted);
        if (current_sc_dist < min_sc_dist) {
            argmin_shift = n_shift;
            min_sc_dist = current_sc_dist;
        }
    }

    return { min_sc_dist, argmin_shift };   // {distance, yaw-as-col-shift}

}

// Called on every keyframe - registers descriptor
void SCManager::makeAndSaveScancontextAndKeys(
        const pcl::PointCloud<SCPointType>& scan) 
{
    Eigen::MatrixXd sc = makeScancontext(scan);
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
    std::vector<float> key =eig2stdvec(ringkey);

    polarContexts_.push_back(sc);   // index == keyframe id
    polarContexts_invKeys_.push_back(ringkey);  
    polarContext_invKeys_Mat_.push_back(key);
}

// Called by loop thread - queries for a loop given a new scan
std::pair<int,float> SCManager::detectLoopClosureIDGivenScan(
    const pcl::PointCloud<SCPointType>& scan)
{
    int loop_id = -1;   // init with -1, meaning no loop (== LeGO-LOAM's variable "closestHistoryFrameID")
    Eigen::MatrixXd curr_desc = makeScancontext(scan);
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(curr_desc);
    std::vector<float> curr_key = eig2stdvec(ringkey);

    // Not enough history - early return
    if ((int)polarContext_invKeys_Mat_.size() < NUM_EXCLUDE_RECENT + 1)
        return {-1, 0.0};

    // Rebuild kd-tree only every Nth call ($$$)
    if (tree_making_period_counter % TREE_MAKING_PERIOD_ == 0) {
        polarContext_invKeys_to_Search_.clear();
        polarContext_invKeys_to_Search_.assign(
            polarContext_invKeys_Mat_.begin(),
            polarContext_invKeys_Mat_.end() - NUM_EXCLUDE_RECENT);  // exclude recent
        polarContext_Tree_.reset();
        polarContext_Tree_ = std::make_unique<InvKeyTree>(
            // PC_NUM_RING: dim, 10: max leaf
            PC_NUM_RING, polarContext_invKeys_to_Search_, 10);
            // tree_ptr_->index->buildIndex(); 
            // internally called in the constructor of InvKeyTree (refer to nanoflann and KDtreeVectorOfVectorsAdaptor)
    }
    ++tree_making_period_counter;

    // kNN search in ring-key tree -> few candidates
    std::vector<size_t> cand_idx(NUM_CANDIDATES_FROM_TREE);
    std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);
    nanoflann::KNNResultSet<float> knn_search_res(NUM_CANDIDATES_FROM_TREE);
    knn_search_res.init(&cand_idx[0], &out_dists_sqr[0]);
    // &curr_key[0]: query
    polarContext_Tree_->index->findNeighbors(knn_search_res, &curr_key[0], 
                                            nanoflann::SearchParams(10));

    // Full SC dist against each candidate; keep best
    double min_dist = 1e11; int nn_align = 0, nn_idx = 0;
    for (int cand_iter_idx = 0; cand_iter_idx < NUM_CANDIDATES_FROM_TREE; ++cand_iter_idx) {
        Eigen::MatrixXd polarContext_cand = polarContexts_[cand_idx[cand_iter_idx]];
        auto [cand_dist, cand_align] = distanceBtnScanContext(curr_desc, polarContext_cand);
        if (cand_dist < min_dist) {
            min_dist = cand_dist;
            nn_align = cand_align;
            nn_idx = cand_idx[cand_iter_idx];
        }
    }

    if (min_dist < SC_DIST_THRES) {
        loop_id = nn_idx;    // accept or reject
        std::cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarContexts_.size()-1 << " and " << nn_idx << "." << std::endl;
        std::cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTOR_ANGLE << " deg." << std::endl;
    }
    else {
        std::cout.precision(3); 
        std::cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarContexts_.size()-1 << " and " << nn_idx << "." << std::endl;
        std::cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTOR_ANGLE << " deg." << std::endl;
    }

    float yaw = deg2rad(nn_align * PC_UNIT_SECTOR_ANGLE);
    
    return { loop_id, yaw };
}