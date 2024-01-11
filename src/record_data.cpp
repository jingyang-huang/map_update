#include "record_data.h"


using namespace std;


int main(int argc, char** argv) {
    ros::init(argc,argv,"record_data_node");
    cout<<" hello ws"<<endl;
  
    Record recorder;
    recorder.start();

    ros::spin();
    
    return 0;
}
