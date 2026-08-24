

#include <opencv2/core.hpp>
#include <sys/time.h>
#include <dlfcn.h>
#include <csignal>
#include <thread>
#include "serial/serial.h"
#include <functional>
#include <mutex>
#include <semaphore.h>
#include <opencv2/highgui.hpp>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <ecal/msg/string/publisher.h>
#include "BaseMsgs.pb.h"
#include "LocalizationMsgs.pb.h"
#include "NavigationMsgs.pb.h"
#include <random>
#include <fstream>
struct RemoteData
{
    int ch1;
    int ch2;
    int ch3;
    int ch4;
    int ch5;
    int ch6;
    int ch7;
    int ch8;
    int ch9;
    int ch10;
    int ch11;
    int ch12;
    int ch13;
    int ch14;
    int ch15;
    int ch16;
};
// 保持随机引擎全局避免每次调用都重复种子
std::default_random_engine rng(std::random_device{}());
std::uniform_real_distribution<double> noise_pos(-0.03, 0.03);  // -5cm~5cm
std::uniform_real_distribution<double> noise_yaw(-4.0 * M_PI / 180.0, 4.0 * M_PI / 180.0);  // -2°~2°
bool found_device_ = false;
serial::Serial serial_io_;
std::mutex serial_mtx_;
std::string port_name_ = "/dev/ttyUSB2";
std::vector<uint8_t> data;
bool thread_run_ = true;
std::thread serial_receive_thread_;
struct RemoteData remote = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
struct CmdMsg
{
    ChassisCmdProto cmd;
    int watchdog_cnt;
    bool online;
};





    CmdMsg nav_cmd;
    std::mutex cmd_mtx_;
     double px_, py_ ,yaw_, v_, w_,steer_angle_ = 0;
     double pub_px_, pub_py_, pub_yaw_;
std::vector<cv::Point2d> dynamic_obstacles_world_;
std::string dynamic_obstacle_file_ = "dynamic_obstacles.txt";
bool capture_mode_ = false;
bool has_last_capture_point_ = false;
cv::Point2d last_capture_point_(0.0, 0.0);

void LoadDynamicObstacles(const std::string& file_path) {
    dynamic_obstacles_world_.clear();
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        return;
    }
    double x = 0.0;
    double y = 0.0;
    while (ifs >> x >> y) {
        dynamic_obstacles_world_.emplace_back(x, y);
    }
    std::cout << "[dynamic_obstacle] loaded " << dynamic_obstacles_world_.size() << " points from " << file_path << std::endl;
}

void SaveDynamicObstacles(const std::string& file_path) {
    std::ofstream ofs(file_path, std::ios::trunc);
    if (!ofs.is_open()) {
        std::cout << "[dynamic_obstacle] failed to save " << file_path << std::endl;
        return;
    }
    for (const auto& p : dynamic_obstacles_world_) {
        ofs << p.x << " " << p.y << "\n";
    }
    std::cout << "[dynamic_obstacle] saved " << dynamic_obstacles_world_.size() << " points to " << file_path << std::endl;
}

bool find_device(std::string port_name) {
    try {
        serial_io_.setPort(port_name);
        serial_io_.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_io_.setTimeout(to);
        serial_io_.open();
        std::cout << "found " << port_name << std::endl;
    }
    catch(serial::SerialException& e){
        serial_io_.close();
        std::cout << "serial init error: " << e.what() << std::endl;
        return false;
    }
    return true;
}

void update(double dt, double v ,double w) {

        double delta_x, delta_y;
        if(fabs(w) < 1e-5) {
            delta_x = v * dt;
            delta_y = 0;
        }
        else {
            double r = v / w;
            delta_x = r * std::sin(w * dt);
            delta_y = r - r * std::cos(w * dt);
        }

        px_ += std::cos(yaw_) * delta_x - std::sin(yaw_) * delta_y;
        py_ += std::sin(yaw_) * delta_x + std::cos(yaw_) * delta_y;
        yaw_ += w * dt;
        while(yaw_ > M_PI) yaw_ -= 2*M_PI;
        while(yaw_ < -M_PI) yaw_ += 2*M_PI;

        pub_px_ = px_;// + noise_pos(rng);
        pub_py_ = py_;// + noise_pos(rng);
        pub_yaw_ = yaw_;// + noise_yaw(rng);
    }


    // 更新车辆状态 (简化版)
    void Ackerman_update(double velocity,double steer_angle,  double dt) {
        // 1. 更新转向角 (带限幅)
      //  steer_angle_ += steer_rate * dt;
      //  max_steer = 1.0;
      //  steer_angle_ = std::clamp(steer_angle, -max_steer, max_steer);

        // 2. 计算车辆角速度 (简化的运动学模型)
        double angular_velocity = 0.0;
        double L = -1.0;
        if (std::abs(steer_angle) > 1e-5) {
            angular_velocity = velocity * std::tan(steer_angle) / L;
        }



        // 4. 直接分解速度更新位置
        px_ += velocity * std::cos(yaw_) * dt;
        py_ += velocity * std::sin(yaw_) * dt;
        // 3. 更新车辆朝向
        yaw_ += angular_velocity * dt;
        while(yaw_ > M_PI) yaw_ -= 2*M_PI;
        while(yaw_ < -M_PI) yaw_ += 2*M_PI;

        pub_px_ = px_;// + noise_pos(rng);
        pub_py_ = py_;// + noise_pos(rng);
        pub_yaw_ = yaw_;// + noise_yaw(rng);
    }




void NavChassisCmdCallback(const ChassisCmdProto& cmd) {
    std::lock_guard<std::mutex> lock(cmd_mtx_);
    nav_cmd.cmd = cmd;
    nav_cmd.watchdog_cnt = 0;
    nav_cmd.online = true;
}


void serial_receive_thread()
{
    while(thread_run_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(found_device_ == false) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if(find_device(port_name_)) {
                found_device_ = true;
            }
            continue;
        }
        size_t read_len = 0;
        try {
            read_len = serial_io_.available();
        } catch(serial::SerialException& e) {
            std::cout << "[serial][available] exception: " << e.what() << std::endl;
            found_device_ = false;
        }
        if(read_len >0) {
            try {
                std::vector<uint8_t> tmp;
                tmp.resize(read_len);
                serial_io_.read(tmp.data(), read_len);
                if (tmp.size() == 25){
                    for(int i = 0; i < tmp.size(); i++) {
                        if(tmp[0] == 0x0F) {
                            remote.ch1 = ((int16_t)tmp[ 1] >> 0 | ((int16_t)tmp[ 2] << 8 )) & 0x07FF;
                            remote.ch2 = ((int16_t)tmp[ 2] >> 3 | ((int16_t)tmp[ 3] << 5 )) & 0x07FF;
                            remote.ch3 = ((int16_t)tmp[ 3] >> 6 | ((int16_t)tmp[ 4] << 2 )  | (int16_t)tmp[ 5] << 10 ) & 0x07FF;
                            remote.ch4 = ((int16_t)tmp[ 5] >> 1 | ((int16_t)tmp[ 6] << 7 )) & 0x07FF;
                            remote.ch5 = ((int16_t)tmp[ 6] >> 4 | ((int16_t)tmp[ 7] << 4 )) & 0x07FF;
                            remote.ch6 = ((int16_t)tmp[ 7] >> 7 | ((int16_t)tmp[ 8] << 1 )  | (int16_t)tmp[ 9] <<  9 ) & 0x07FF;
                            remote.ch7 = ((int16_t)tmp[ 9] >> 2 | ((int16_t)tmp[10] << 6 )) & 0x07FF;
                            remote.ch8 = ((int16_t)tmp[10] >> 5 | ((int16_t)tmp[11] << 3 )) & 0x07FF;
                            remote.ch9 = ((int16_t)tmp[12] << 0 | ((int16_t)tmp[13] << 8 )) & 0x07FF;
                            remote.ch10 = ((int16_t)tmp[13] >> 3 | ((int16_t)tmp[14] << 5 )) & 0x07FF;
                            remote.ch11 = ((int16_t)tmp[14] >> 6 | ((int16_t)tmp[15] << 2 )  | (int16_t)tmp[16] << 10 ) & 0x07FF;
                            remote.ch12 = ((int16_t)tmp[16] >> 1 | ((int16_t)tmp[17] << 7 )) & 0x07FF;
                            remote.ch13 = ((int16_t)tmp[17] >> 4 | ((int16_t)tmp[18] << 4 )) & 0x07FF;
                            remote.ch14 = ((int16_t)tmp[18] >> 7 | ((int16_t)tmp[19] << 1 )  | (int16_t)tmp[20] <<  9 ) & 0x07FF;
                            remote.ch15 = ((int16_t)tmp[20] >> 2 | ((int16_t)tmp[21] << 6 )) & 0x07FF;
                            remote.ch16 = ((int16_t)tmp[21] >> 5 | ((int16_t)tmp[22] << 3 )) & 0x07FF;

                        }
                    }
                }

            } catch(serial::SerialException& e) {
                std::cout << "[serial][read] exception: " << e.what() << std::endl;
            }
        }
    //    std::cout<<"read_len "<<read_len <<std::endl;
      /*  if(data.size() == 0) continue;
        for(int i = 0; i < data.size(); i++) {
            if(tmp[0] == 0x0F) {
                remote.ch1 = ((int16_t)tmp[ 1] >> 0 | ((int16_t)tmp[ 2] << 8 )) & 0x07FF;
                remote.ch2 = ((int16_t)tmp[ 2] >> 3 | ((int16_t)tmp[ 3] << 5 )) & 0x07FF;
                remote.ch3 = ((int16_t)tmp[ 3] >> 6 | ((int16_t)tmp[ 4] << 2 )  | (int16_t)tmp[ 5] << 10 ) & 0x07FF;
                remote.ch4 = ((int16_t)tmp[ 5] >> 1 | ((int16_t)tmp[ 6] << 7 )) & 0x07FF;
                remote.ch5 = ((int16_t)tmp[ 6] >> 4 | ((int16_t)tmp[ 7] << 4 )) & 0x07FF;
                remote.ch6 = ((int16_t)tmp[ 7] >> 7 | ((int16_t)tmp[ 8] << 1 )  | (int16_t)tmp[ 9] <<  9 ) & 0x07FF;
                remote.ch7 = ((int16_t)tmp[ 9] >> 2 | ((int16_t)tmp[10] << 6 )) & 0x07FF;
                remote.ch8 = ((int16_t)tmp[10] >> 5 | ((int16_t)tmp[11] << 3 )) & 0x07FF;
                remote.ch9 = ((int16_t)tmp[12] << 0 | ((int16_t)tmp[13] << 8 )) & 0x07FF;
                remote.ch10 = ((int16_t)tmp[13] >> 3 | ((int16_t)tmp[14] << 5 )) & 0x07FF;
                remote.ch11 = ((int16_t)tmp[14] >> 6 | ((int16_t)tmp[15] << 2 )  | (int16_t)tmp[16] << 10 ) & 0x07FF;
                remote.ch12 = ((int16_t)tmp[16] >> 1 | ((int16_t)tmp[17] << 7 )) & 0x07FF;
                remote.ch13 = ((int16_t)tmp[17] >> 4 | ((int16_t)tmp[18] << 4 )) & 0x07FF;
                remote.ch14 = ((int16_t)tmp[18] >> 7 | ((int16_t)tmp[19] << 1 )  | (int16_t)tmp[20] <<  9 ) & 0x07FF;
                remote.ch15 = ((int16_t)tmp[20] >> 2 | ((int16_t)tmp[21] << 6 )) & 0x07FF;
                remote.ch16 = ((int16_t)tmp[21] >> 5 | ((int16_t)tmp[22] << 3 )) & 0x07FF;
                std::cout<<"test "<<remote.ch3 <<" "<<remote.ch1<<std::endl;
            }
        }*/
    }
    printf("serial thread end\n");
}
double mapValueClamped(double input, double out_min, double out_max) {
    const double in_min = 282.0;
    const double in_max = 1722.0;

    // 钳制输入值在有效范围内
    if(input < in_min) input = in_min;
    if(input > in_max) input = in_max;

    return (input - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}

int main(int argc, char** argv)
{

    eCAL::Initialize(argc, argv, "mower_base");

    double t1 = (double)cv::getTickCount() / cv::getTickFrequency();
    double t2 = (double)cv::getTickCount() / cv::getTickFrequency();
    CmdMsg remote_cmd;
    remote_cmd.online = false;
    remote_cmd.watchdog_cnt = 0;
    std::shared_ptr<eCAL::protobuf::CPublisher<BaseInfoProto>> base_info_pub_ = std::make_shared<eCAL::protobuf::CPublisher<BaseInfoProto>>("/mower_base/base_info");
    std::shared_ptr<eCAL::protobuf::CSubscriber<ChassisCmdProto>>  nav_cmd_sub_ = std::make_shared<eCAL::protobuf::CSubscriber<ChassisCmdProto>>("/mower_navigation/chassis_cmd");

    std::shared_ptr<eCAL::protobuf::CPublisher<PoseInfoProto>> pos_pub_ = std::make_shared<eCAL::protobuf::CPublisher<PoseInfoProto>>("/mower_localization/pos_info");
    std::shared_ptr<eCAL::protobuf::CPublisher<GnssPoseInfoProto>> gnss_pos_pub_ = std::make_shared<eCAL::protobuf::CPublisher<GnssPoseInfoProto>>("/mower_localization/gnss_pos_info");
    std::shared_ptr<eCAL::protobuf::CPublisher<DoubleProto>> lidar_heartbeat_pub_ = std::make_shared<eCAL::protobuf::CPublisher<DoubleProto>>("/lidar/heartbeat");
    std::shared_ptr<eCAL::string::CPublisher<std::string>> rtk_dtu_state_pub_ = std::make_shared<eCAL::string::CPublisher<std::string>>("/rtk_dtu/state");
    std::shared_ptr<eCAL::protobuf::CPublisher<PathMsgProto>> dynamic_obstacle_pub_ = std::make_shared<eCAL::protobuf::CPublisher<PathMsgProto>>("/mower_localization/cloud");
    v_= 0,w_ = 0;
     nav_cmd_sub_->AddReceiveCallback(std::bind(&NavChassisCmdCallback, std::placeholders::_2));
    LoadDynamicObstacles(dynamic_obstacle_file_);
    cv::namedWindow("mower_base", cv::WINDOW_NORMAL);
    bool flag = find_device(port_name_);
    if(flag == false)
    {
        std::cout << "Unable to open port " << port_name_ << std::endl;
        printf("try again");
        while(flag == false) {
            printf(".");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            flag = find_device(port_name_);
        }
        printf("\n");
    }
    found_device_ = true;
    serial_receive_thread_= std::thread(&serial_receive_thread);
    while(true)
    {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            t1 = t2;
            cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);  // 500x500 像素，3通道（RGB），每个通道8位

            // 显示空白图像
         //   cv::imshow("mower_base", img);

            t2 = (double)cv::getTickCount() / cv::getTickFrequency();
          char keyboard = static_cast<char>(cv::waitKey(1) & 0xFF);
            if (keyboard == 'c') {
                capture_mode_ = true;
                dynamic_obstacles_world_.emplace_back(pub_px_, pub_py_);
                last_capture_point_ = cv::Point2d(pub_px_, pub_py_);
                has_last_capture_point_ = true;
                std::cout << "[dynamic_obstacle] capture mode on, append start point: "
                          << pub_px_ << ", " << pub_py_ << std::endl;
            } else if (keyboard == 'q') {
                if (capture_mode_) {
                    SaveDynamicObstacles(dynamic_obstacle_file_);
                    capture_mode_ = false;
                    has_last_capture_point_ = false;
                    std::cout << "[dynamic_obstacle] saved and exit capture mode, kept memory points" << std::endl;
                }
            } else if (keyboard == 'x') {
                capture_mode_ = false;
                dynamic_obstacles_world_.clear();
                has_last_capture_point_ = false;
                SaveDynamicObstacles(dynamic_obstacle_file_);
                std::cout << "[dynamic_obstacle] cleared all dynamic obstacles" << std::endl;
            }
            if (remote.ch6 == 1722){
                v_= mapValueClamped(remote.ch3 ,-1.5, 1.5);
                w_= mapValueClamped(remote.ch1 ,1.5, -1.5);
                steer_angle_+= mapValueClamped(remote.ch1 ,-1, 1) * 0.01;
                if (steer_angle_ > 30 * M_PI/180.0)
                   steer_angle_ = 30 * M_PI/180.0;
                if (steer_angle_ < -30 * M_PI/180.0)
                   steer_angle_ = -30 * M_PI/180.0;
                remote_cmd.watchdog_cnt = 0;
                remote_cmd.online = true;
              //  std::cout<<"manual  "<<v_ <<" "<<steer_angle_<<std::endl;
            }


         /*  switch (keyboard) {
            case 'w':  // 向前
                    v_ = 1.0;
                    remote_cmd.watchdog_cnt = 0;
                    remote_cmd.online = true;

                break;
            case 's':  // 向后
                    v_ = -1.0;
                    remote_cmd.watchdog_cnt = 0;
                    remote_cmd.online = true;

                break;
            case 'a':  // 向左
                    w_ = 1;
                    remote_cmd.watchdog_cnt = 0;
                    remote_cmd.online = true;

                break;
            case 'd':  // 向右
                    w_ = -1;
                    remote_cmd.watchdog_cnt = 0;
                    remote_cmd.online = true;

                break;
            default:
                if (manual_mode_ && remote_cmd.online){
                    v_ = 0;
                    w_ = 0;
                }
                break;
            }*/
            if(remote_cmd.online) {
                remote_cmd.watchdog_cnt++;
                if(remote_cmd.watchdog_cnt > 50) {
                    remote_cmd.online = false;
                }
            }
            if(nav_cmd.online) {
                    nav_cmd.watchdog_cnt++;
                    if(nav_cmd.watchdog_cnt > 10) {
                        nav_cmd.online = false;
                    }
            }
            if (remote_cmd.online == false){
                std::lock_guard<std::mutex> lock(cmd_mtx_);
                if (nav_cmd.online){
                    v_ = nav_cmd.cmd.vel();
                    w_ = nav_cmd.cmd.angular();
                steer_angle_ = nav_cmd.cmd.steer_angle();
                if (steer_angle_ > nav_cmd.cmd.steer_angle()+0.02)
                   steer_angle_-= 0.02;
                else if (steer_angle_ < nav_cmd.cmd.steer_angle()-0.02)
                   steer_angle_+= 0.02;
                if (steer_angle_ > 30 * M_PI/180.0)
                   steer_angle_ = 30 * M_PI/180.0;
                if (steer_angle_ < -30 * M_PI/180.0)
                   steer_angle_ = -30 * M_PI/180.0;
              //  std::cout << "online "<< v_<<" "<<w_<<std::endl;
                }
                else{
                    v_ = 0;
                    w_ = 0;
                }
            }
            BaseInfoProto base_info_proto;
            if (remote_cmd.online)
               base_info_proto.set_is_manual(true);
            else
               base_info_proto.set_is_manual(false);
            base_info_proto.set_linear_vel(v_);
            base_info_proto.set_angular_vel(w_);
            base_info_proto.set_steer_angle(steer_angle_);
            base_info_proto.set_mcu_online(true);
            base_info_proto.set_left_battery_remain(50);
            base_info_proto.set_right_battery_remain(50);
            base_info_proto.set_is_left_battery_output(true);
            base_info_proto.set_is_right_battery_output(false);
            base_info_proto.set_left_battery_connected(true);
            base_info_proto.set_right_battery_connected(true);
            base_info_proto.set_left_battery_current(0);
            base_info_proto.set_right_battery_current(0);
            base_info_proto.set_left_battery_voltage(48000);
            base_info_proto.set_right_battery_voltage(48000);
            base_info_proto.set_left_battery_protection_state(0);
            base_info_proto.set_right_battery_protection_state(0);
            base_info_proto.set_walk_motor_connected(true);
            base_info_proto.set_left_walk_motor_error_code(0);
            base_info_proto.set_right_walk_motor_error_code(0);
            base_info_pub_->Send(base_info_proto);

            update(t2-t1,v_,w_);
         //   Ackerman_update(v_,steer_angle_, t2-t1);
            if (capture_mode_) {
                if (!has_last_capture_point_) {
                    last_capture_point_ = cv::Point2d(pub_px_, pub_py_);
                    dynamic_obstacles_world_.emplace_back(pub_px_, pub_py_);
                    has_last_capture_point_ = true;
                    std::cout << "[dynamic_obstacle] capture point: " << pub_px_ << ", " << pub_py_ << std::endl;
                } else {
                    const double dx = pub_px_ - last_capture_point_.x;
                    const double dy = pub_py_ - last_capture_point_.y;
                    const double dist = std::sqrt(dx * dx + dy * dy);
                    if (dist >= 0.05) {
                        dynamic_obstacles_world_.emplace_back(pub_px_, pub_py_);
                        last_capture_point_ = cv::Point2d(pub_px_, pub_py_);
                        std::cout << "[dynamic_obstacle] capture point: " << pub_px_ << ", " << pub_py_ << std::endl;
                    }
                }
            }
            PoseInfoProto pos_msg;
            pos_msg.set_x(pub_px_);
            pos_msg.set_y(pub_py_);
            pos_msg.set_yaw(pub_yaw_);
            pos_msg.set_time((double)cv::getTickCount() / cv::getTickFrequency());
            pos_msg.set_state(std::string("normal"));
            pos_msg.set_cur_map(std::string("default"));
            pos_pub_->Send(pos_msg);

            GnssPoseInfoProto gnss_pos_msg;
            gnss_pos_msg.mutable_pose()->CopyFrom(pos_msg);
            gnss_pos_msg.set_lat(22.52291);
            gnss_pos_msg.set_lon(114.05454);
            gnss_pos_msg.set_hei(2.0);
            gnss_pos_msg.set_offset_lat(22.52291);
            gnss_pos_msg.set_offset_lon(114.05454);
            gnss_pos_msg.set_offset_hei(2.0);
            gnss_pos_msg.set_rtk_status(4);
            gnss_pos_msg.set_inertial_status(5);
            gnss_pos_msg.set_main_satellite(20);
            gnss_pos_msg.set_aux_satellite(20);
       //     gnss_pos_pub_->Send(gnss_pos_msg);

            DoubleProto lidar_heartbeat_msg;
            lidar_heartbeat_msg.set_value(t2);
            lidar_heartbeat_pub_->Send(lidar_heartbeat_msg);
            rtk_dtu_state_pub_->Send("connected");

            PathMsgProto dynamic_msg;
            dynamic_msg.set_time((double)cv::getTickCount() / cv::getTickFrequency());
            for (const auto& obs_w : dynamic_obstacles_world_) {
                const double dx = obs_w.x - pub_px_;
                const double dy = obs_w.y - pub_py_;
                const double x_robot = std::cos(pub_yaw_) * dx + std::sin(pub_yaw_) * dy;
                const double y_robot = -std::sin(pub_yaw_) * dx + std::cos(pub_yaw_) * dy;
                // 前方 2m x 2m 框: x in [0, 2], y in [-1, 1]
                if (x_robot >= 0.0 && x_robot <= 8.0 && std::fabs(y_robot) <= 4.0) {
                    PointMsgProto* pt = dynamic_msg.add_path();
                    pt->set_x(static_cast<float>(x_robot));
                    pt->set_y(static_cast<float>(y_robot));
                    pt->set_z(0.0f);
                }
            }
            dynamic_msg.set_len(dynamic_msg.path_size());
    dynamic_obstacle_pub_->Send(dynamic_msg);

    }
    eCAL::Finalize();

    thread_run_ = false;
    serial_receive_thread_.join();
    try {
        serial_io_.close();
        printf("serial closed\n");
    }
    catch(serial::SerialException& e){
        std::cout << "[serial][close] exception: " << e.what() << std::endl;
        found_device_ = false;
    }
    return 0;
}
