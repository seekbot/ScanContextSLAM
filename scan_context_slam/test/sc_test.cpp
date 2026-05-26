#include "Scancontext.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

int main(int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZI>scan_a, scan_b;
    // From DiTer++ dataset
    pcl::io::loadPCDFile("/home/dev/ros2_ws/src/scan_context_slam/data/Forest_GT.pcd", scan_a);

    SCManager sc;
    Eigen::MatrixXd sc_a = sc.makeScancontext(scan_a);

    // rotate cloud by +90 deg about z and rebuild
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float th = -M_PI/2; // CW in ScanContext
    T(0,0) = cos(th);   T(0,1) = -sin(th);
    T(1,0) = sin(th);   T(1,1) = cos(th);
    pcl::transformPointCloud(scan_a, scan_b, T);
    Eigen::MatrixXd sc_b = sc.makeScancontext(scan_b);

    auto [dist, shift] = sc.distanceBtnScanContext(sc_a, sc_b);
    std::cout << "dist = " << dist << " shift = " << shift << std::endl;
    std::cout << "Expect ~15 for 90 deg per 60 sectors" << std::endl;

    return 0;
}