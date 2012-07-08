#include "TorpedoTask.hpp"

int main(int argc, char** pArgv)
{
    ros::init(argc, pArgv, "TorpedoTask");

    TorpedoTask task;
    ros::spin();

    return 0;
}
