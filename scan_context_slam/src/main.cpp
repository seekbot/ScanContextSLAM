#include "fast_lio_sam_sc_qn.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLioSamScQn>();
    /** IMPORTANT: By default, rclcpp::spin() uses a single-threaded executor
     * So the loop timer and the data callback run on 
     * the same thread & the loop work blocks data.
     */
    rclcpp::executors::MultiThreadedExecutor exec; 

    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();

    return 0;
}