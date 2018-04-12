#include "go_dest/go_action.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "go_dest");

    Go go_dest("go_dest");
    ros::spin();

    return 0;
}
