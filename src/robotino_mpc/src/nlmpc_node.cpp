//
// Created by ebonetto on 8/26/20.
//

#include "nlmpc.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nlmpc_node");

    ros::NodeHandle nh, private_nh("~");

    std::shared_ptr<nlmpc> mpc(new nlmpc(nh, private_nh));

    ros::waitForShutdown();

    return 0;
}