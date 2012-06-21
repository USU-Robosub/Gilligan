#include "BuoysTask.hpp"

int main(int argc, char** pArgv)
{
    BuoyColors first = RED;
    BuoyColors second = GREEN;
    ros::init(argc, pArgv, "BuoysTask");

    if(argc >= 3)
    {
        first = BuoyColors(atoi(pArgv[1]));
        second = BuoyColors(atoi(pArgv[2]));
    }

    BuoysTask task(first, second);
    ros::spin();

    return 0;
}
