#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using std::string; //std名前空間のstringというメソッドを使う

namespace robot_planner{ //global planner名前空間を定義

class RobotPlanner : public nav_core::BaseGlobalPlanner { //nav_coreからbgpのインタフェースを継承させる
  public:
    RobotPlanner(); //デフォルトコンストラクター
    RobotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);//引数を受け取る移譲コンストラクター（コストマップの初期化）
    ~RobotPlanner();//デストラクター（動的確保したメモリの開放をする）

     /** nav_core::BaseGlobalPlannerからインターフェースとしてオーバーライドする **/
     void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);//nav_coreから持ってきたメソッドたち　色んな変数の初期化
     bool makePlan(const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal,
                   std::vector<geometry_msgs::PoseStamped>& plan
     ); //動作を記述するメソッド

   //共通に使う処理部分をprivateにまとめている
   private:
     costmap_2d::Costmap2DROS* costmap_ros_; //二次元の障害物のある範囲を示す変数
     costmap_2d::Costmap2D* costmap_; //二次元コストマップ情報のデータが入っている変数
     double step_size_, min_dist_from_robot_; //最小の移動できる距離
     base_local_planner::WorldModel* world_model_; //ロボットのfootprint（大きさ？）を格納

     double footprintCost(double x_i, double y_i, double theta_i);
     bool initialized_;
};

PLUGINLIB_EXPORT_CLASS(robot_planner::RobotPlanner, nav_core::BaseGlobalPlanner)//move_baseでのnav_coreのプラグインになる

  //Default Constructor
  RobotPlanner::RobotPlanner()//コンストラクターの定義
    : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){} //メンバー変数の初期化

  RobotPlanner::RobotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) //移譲コンストラクター
    : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  RobotPlanner::~RobotPlanner(){ //デストラクター
    delete world_model_; //動的メモリの破棄
  }

  void RobotPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
      if(!initialized_){
          costmap_ros_ = costmap_ros; //ポインター変数
          costmap_ = costmap_ros_->getCostmap();

          ros::NodeHandle private_nh("~/" + name);
          private_nh.param("step_size", step_size_, costmap_->getResolution());
          private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
          world_model_ = new base_local_planner::CostmapModel(*costmap_); //newで動的にメモリを確保

          initialized_ = true; //boolで定義
      }
      else{ROS_WARN("This planner has already been initialized... doing nothing");}
  }
  
  double RobotPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){ //初期化されてなければ初期化する
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint(); //geometry_msgsのfootprintパラメーターを取得
    //もしfootprintが無ければ...　何もしない
    if(footprint.size() < 3)
      return -1.0;
      
    //今までの確認でfootprintが正しい値ならlocal plannerから得たfootprint_costに代入して返す
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint); //local_costmapの値？
    
    return footprint_cost;
  }
  
  bool RobotPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
  const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
    //初期位置、目的地、経路を引数として入力される
    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap(); //layeard_costmap_->costmap_から取得

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    const double start_yaw = tf2::getYaw(start.pose.orientation);
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);

    //正しいセルを見つけるまでロボットの自己位置と目標姿勢で作成されたベクトルに沿って後退させる
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x; //ゴールまでの差
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;

    bool done = false;
    double scale = 1.0;
    double dScale = 0.01;

    while(!done) //while Trueと同じ, 実行する部分
    {

      target_x = start_x + scale * diff_x; //ゴール座標にxがずれた座標 赤い方向に行く
      target_y = start_y + scale * diff_y;
      target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

      double footprint_cost = footprintCost(target_x, target_y, target_yaw); //ここでロボットのfootprintを使っている

      if(footprint_cost >= 0 ){ done = true; } //footprint costは負になっている
      
      scale -=dScale;
    }

    plan.push_back(start); //経路のベクトルが開始

    geometry_msgs::PoseStamped new_goal = goal;
    tf2::Quaternion goal_quat;//ゴールでの向きのtf
    goal_quat.setRPY(0, 0, target_yaw);

    new_goal.pose.position.x = target_x;//新しく設定したゴール地点
    new_goal.pose.position.y = target_y;

    new_goal.pose.orientation.x = goal_quat.x();//ゴールでの姿勢
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal); //新しいゴールを取ります

   return done;
  }
}; //global plannerのnamespace用のかっこ
