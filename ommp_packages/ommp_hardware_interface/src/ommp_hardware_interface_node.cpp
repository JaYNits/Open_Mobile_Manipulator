#include <ommp_hardware_interface/ommp_hardware_interface.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ommp_hardware_interface");
    // ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    // nh.setCallbackQueue(&ros_queue);
    //
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ommp_hardware_interface::OmmpHardwareInterface rhi(nh);

    // ros::spin();
    ros::waitForShutdown();
    // ros::MultiThreadedSpinner spinner(0);
    //spinner.spin(&ros_queue);
    return 0;
}