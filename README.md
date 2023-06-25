## ROS Noetic Installation(Full)
```bash
sudo apt install ros-noetic-desktop-full
```

## QGroundControl Installation 
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html 참고하여 설치 진행

## PX4 installation
1. Download PX4 Source Code:
```git
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
2. Run the ubuntu.sh with no arguments (in a bash shell) to install everything:
```git
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
실행 시 `HTTP request sent, awaiting response...` 에러 발생하면서 진행안됨.
#### Sol) 
`~/PX4-AutoPilot/Tools/setup/ubuntu.sh` 파일 235번째 줄 

`wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -` 부분 commend처리 후 

chrome에서 `http://packages.osrfoundation.org/gazebo.key` 입력하면 key가 자동으로 다운로드 됨

key받아진 경로로 가서 `sudo apt-key add gazebo.key` 이후 다시
```bash ./PX4-Autopilot/Tools/setup/ubuntu.sh```


## MAVROS Binary Installation (Debian / Ubuntu)
Install MAVROS

```bash
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
```

Then install GeographicLib
```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
```

## Launching Gazebo Classic with ROS Wrappers
The Gazebo Classic simulation can be modified to integrate sensors publishing directly to ROS topics e.g. the Gazebo Classic ROS laser plugin. To support this feature, Gazebo Classic must be launched with the appropriate ROS wrappers.

There are ROS launch scripts available to run the simulation wrapped in ROS:

* posix_sitl.launch : plain SITL launch
* mavros_posix_sitl.launch : SITL and MAVROS
```bash
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo-classic
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 posix_sitl.launch
```
이후 terminal 창에 `roslaunch px4 posix_sitl.launch` 입력 하면 드론 호버링 확인

터미널 창 킬 때마다 위처럼 환경변수 설정해주기 귀찮으니 bashrc 파일에 추가
```bash
source ~/drone_ws/devel/setup.bash    # (optional)
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```
우리는 앞으로 drone_ws 폴더에서 빌드할 예정이므로 변경
새로운 터미널 열고 제대로 실행되는지 확인
```bash
roslaunch px4 posix_sitl.launch
```
`PX4_SITL`과 `MAVROS` 함께 실행
```
roslaunch px4 mavros_posix_sitl.launch 
```
실행 후 rqt_graph로 mavros 노드 확인, `rostopic echo /mavros/state`로 `connected: True`인지 확인

## Run MAVROS Offboard control example
* ### Create a ROS package
```bash
cd ~/drone_ws/src
catkin_create_pkg mavros_offboard_control_example geometry_msgs mavros_msgs roscpp
```
* ### Create source code
```bash
roscd mavros_offboard_control_example
cd src
code offb_node.cpp
```
```cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```
코드의 자세한 설명은 https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html에서 확인가능

Edit the `CMakeLists.txt`
```
cd ..
code CMakeLists.txt
```
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(mavros_offboard_control_example)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_offb_node src/offb_node.cpp)
target_link_libraries(${PROJECT_NAME}_offb_node ${catkin_LIBRARIES})
```

* ### Build and Run
```bash
cd ~/drone_ws
catkin_make
source devel/setup.bash
rosrun mavros_offboard_control_example mavros_offboard_control_example_offb_node
[ INFO] [1605190344.010748305, 50.352000000]: Offboard enabled
[ INFO] [1605190349.075704903, 55.408000000]: Vehicle armed
```
rqt_graph에서 offb_node 노드 생성된 것 확인

offb_node가 `mavros/setpoint_position/local`에 publish해줌으로써 drone을 원하는 위치로 비행시킴

소스코드에서 `pose.pose.position.x = 0;pose.pose.position.y = 0;pose.pose.position.z = 2;` 를 조절하며 드론위치 변경

## Example
src 폴더에 `offboard_node_traj.cpp`파일 생성
```cpp
/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "math.h"

double r=2;
double theta;
double count=0.0;
double wn=0.5;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
	

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

	theta = wn*count*0.05;

    	pose.pose.position.x = r*sin(theta);
    	pose.pose.position.y = r*cos(theta);
    	pose.pose.position.z = 2;

	count++;

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```
CMakeLists.txt 파일 수정
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(mavros_offboard_control_example)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_offb_node src/offb_node.cpp)
target_link_libraries(${PROJECT_NAME}_offb_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_offb_node_traj src/offb_node_traj.cpp)
target_link_libraries(${PROJECT_NAME}_offb_node_traj ${catkin_LIBRARIES})
```
빌드 후 실행
rqt_graph로 offb_node 노드가 rostopic을 publish하는 것을 확인.
 하지만 전과 다르게 드론이 움직이지 않는데 왜냐하면 자동으로 offboard 모드로 바꾸는 것을 주석처리 했기 때문
QGC 실행 후 왼쪽 상단의 `takeoff`클릭 후 `slide to confirm` 실행하면 드론 이륙 후 Hold 모드로 진입
이후 QGC에서 Hold모드를 Offboard모드로 변경하면 드론이 원운동을 함
