//
// Created by luomao on 24-12-12.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

// 定义按键与移动命令之间的映射关系
std::map<char, std::vector<float>> moveBindings = {
  {'w', {1, 0, 0, 0}},
  {'a', {0, 1, 0 ,0}},
  {'s', {-1, 0, 0, 0}},
  {'d', {0, -1, 0, 0}},
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
};

// 定义按键与速度调整之间的映射关系
std::map<char, std::vector<float>> speedBindings = {
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}},
};

// 获取键盘输入的函数
char getKey()
{
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(STDIN_FILENO, &old) < 0)
    perror("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &old) < 0)
    perror("tcsetattr ICANON");
  if (read(STDIN_FILENO, &buf, 1) < 0)
    perror("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(STDIN_FILENO, TCSADRAIN, &old) < 0)
    perror("tcsetattr ~ICANON");
  return buf;
}

// 发布线程类，用于发布Twist消息
class PublishThread
{
public:
  // 构造函数，初始化成员变量并启动发布线程
  PublishThread(double rate)
      : rate_(rate), done_(false), x_(0), y_(0), z_(0), th_(0), speed_(0), turn_(0)
  {
    pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    thread_ = std::thread(&PublishThread::run, this);
  }

  // 析构函数，停止发布线程并等待其结束
  ~PublishThread()
  {
    stop();
    thread_.join();
  }

  // 更新运动命令并通知发布线程
  void update(float x, float y, float z, float th, float speed, float turn)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = x;
    y_ = y;
    z_ = z;
    th_ = th;
    speed_ = speed;
    turn_ = turn;
    cv_.notify_one();  // 通知发布线程有新的运动命令
  }

  // 停止发布线程
  void stop()
  {
    done_ = true;
    update(0, 0, 0, 0, 0, 0);  // 确保发布线程退出等待状态
  }

private:
  // 发布线程的主循环
  void run()
  {
    geometry_msgs::Twist twist;
    ros::Rate rate(rate_);
    while (ros::ok() && !done_)
    {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock);  // 等待新的运动命令
        twist.linear.x = x_ * speed_;
        twist.linear.y = y_ * speed_;
        twist.linear.z = z_ * speed_;
        twist.angular.z = th_ * turn_;
      }
      pub_.publish(twist);  // 发布Twist消息
      rate.sleep();
    }
  }

  ros::NodeHandle nh_;  // ROS节点句柄
  ros::Publisher pub_;  // ROS发布者
  std::thread thread_;  // 发布线程
  std::mutex mutex_;  // 互斥锁，用于保护共享数据
  std::condition_variable cv_;  // 条件变量，用于线程间同步
  double rate_;  // 发布频率
  bool done_;  // 标志位，指示发布线程是否应该停止
  float x_, y_, z_, th_, speed_, turn_;  // 运动命令变量
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_twist_keyboard");

  double speed = 0.5;
  double turn = 1.0;
  double speed_limit = 1000;
  double turn_limit = 1000;
  double repeat = 0.0;
  double key_timeout = 0.5;

  ros::NodeHandle nh("~");
  nh.param("speed", speed, speed);
  nh.param("turn", turn, turn);
  nh.param("speed_limit", speed_limit, speed_limit);
  nh.param("turn_limit", turn_limit, turn_limit);
  nh.param("repeat_rate", repeat, repeat);
  nh.param("key_timeout", key_timeout, key_timeout);

  PublishThread pub_thread(repeat);

  float x = 0, y = 0, z = 0, th = 0;
  int status = 0;

  try
  {
    while (ros::ok())
    {
      char key = getKey();  // 获取键盘输入
      if (moveBindings.count(key))
      {
        x = moveBindings[key][0];
        y = moveBindings[key][1];
        z = moveBindings[key][2];
        th = moveBindings[key][3];
      }
      else if (speedBindings.count(key))
      {
        speed = std::min(speed_limit, speed * speedBindings[key][0]);
        turn = std::min(turn_limit, turn * speedBindings[key][1]);
        std::cout << "currently:\tspeed " << speed << "\tturn " << turn << std::endl;
      }
      else
      {
        if (key == '\x03')  // 检查是否按下了Ctrl+C
          break;
        x = 0;
        y = 0;
        z = 0;
        th = 0;
      }
      pub_thread.update(x, y, z, th, speed, turn);  // 更新运动命令
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }

  pub_thread.stop();  // 停止发布线程
  return 0;
}