#include "amr_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "amr_base_node");
    AmrBase amr;
    ros::spin();
    return 0;
}