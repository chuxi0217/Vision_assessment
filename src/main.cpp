#include <iostream>
#include <opencv2/opencv.hpp>

#include "yolov5.hpp"
#include "solver.hpp"
#include "aimer.hpp"

using namespace auto_aim;
using namespace std;

int main(int argc, char** argv) {
  // 1. 配置文件和视频路径
  string config_path = "configs/infantry.yaml";
  string video_path = "assets/infantry.avi";
  std::deque<std::pair<double,double>> yaw_history;
  const size_t History_Maxsize = 5;
  // 2. 初始化 YOLOv5 类
  //    第一个参数：yaml 配置文件路径
  //    第二个参数：是否开启 yolo 自带的 debug 显示（这里关掉了，我们自己画）
  YOLOV5 yolo(config_path, false);

  // 3. 初始化 Solver 类（用于 PnP 解算）
  Solver solver(config_path);
  Aimer aimer;

  // 4. 打开视频
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    cerr << "Failed to open video: " << video_path << endl;
    return -1;
  }

  cv::Mat frame;
  int frame_count = 0;
  bool pause = false;
  bool hasFrame = false;

  // 5. 逐帧处理
  while (true) {
    if(!pause){
      hasFrame = cap.read(frame);
      if(!hasFrame) break;
    }
    frame_count++;

    // 调用 YOLO 模型检测装甲板，返回一个装甲板列表
    auto armors = yolo.detect(frame, frame_count);

    // 遍历每一个识别到的装甲板
    for (auto & armor : armors) {
      // TODO: PnP 解算（solver 目前还是空的，所以距离显示为 0）
      solver.solve(armor);
      aimer.update(armor);
      double yaw_now = aimer.yaw();
      double t_now = (double)cv::getTickCount() / cv::getTickFrequency();
      yaw_history.push_back({yaw_now , t_now});
      if(yaw_history.size() > History_Maxsize) yaw_history.pop_front();
      double yaw_predict = yaw_now;
      if(yaw_history.size() >= 2){
        double dy = yaw_history.back().first - yaw_history.front().first;
        double dt = yaw_history.back().second - yaw_history.front().second;
        if(dt > 0){
          double v_yaw = dy / dt;
          yaw_predict = yaw_now + v_yaw*0.1;
        }
      }
      // 6. 在图像上画出装甲板的四个角
      //    points 里有 4 个点，按顺序连成四边形
      for (size_t i = 0; i < armor.points.size(); i++) {
        cv::circle(frame, armor.points[i], 4, cv::Scalar(0, 255, 0), -1);
        cv::line(
          frame,
          armor.points[i],
          armor.points[(i + 1) % armor.points.size()],
          cv::Scalar(0, 255, 0), 2);
      }

      // 7. 在装甲板中心显示距离（当前为 0，因为 PnP 没写）
      double dist = armor.xyz_in_gimbal.norm();
      string text = cv::format("%.2f m", dist);
      cv::putText(
        frame, text, armor.center,
        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
      // 新增：显示 yaw/pitch（角度制，方便人读）
    double yaw_deg = aimer.yaw() * 180.0 / CV_PI;
    double pitch_deg = aimer.pitch() * 180.0 / CV_PI;
    std::string angle_text = cv::format("Y:%.1f P:%.1f", yaw_deg, pitch_deg);
    cv::putText(frame, angle_text,
      cv::Point(armor.center.x, armor.center.y - 20),
      cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(255, 255, 0), 2);
      // 白字：当前值
      std::string text1 = cv::format("Y:%.1f", yaw_now * 180.0 / CV_PI);
      cv::putText(frame, text1, cv::Point(10, 30), 
            cv::FONT_HERSHEY_SIMPLEX, 0.7,
            cv::Scalar(255, 255, 0), 2);

      // 红字：预测值
      std::string text2 = cv::format("Yp:%.1f", yaw_predict * 180.0 / CV_PI);
      cv::putText(frame, text2, cv::Point(10, 60), 
            cv::FONT_HERSHEY_SIMPLEX, 0.7,
            cv::Scalar(0, 0, 255), 2);
    }

    // 8. 显示画面
    cv::imshow("Vision Assessment", frame);

    // 按 ESC 退出
    int key = cv::waitKey(30);
    if (key == 27) {
      break;
    }
    else if(key == 32){
      pause = !pause;
    }
  }
  cap.release();
  cv::destroyAllWindows();
  return 0;
}
