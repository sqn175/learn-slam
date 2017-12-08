/*
 * @Author: Shi Qin 
 * @Date: 2017-12-05 17:26:32 
 * @Brief: Evaluate SLAM result, compare with ground truth
 */

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <glog/logging.h>
#include <eigen3/Eigen/Core>


namespace lslam{

class Evaluation {
public:
  void Evaluate() {
    std::string file_name = "/home/sqn/Documents/learn-slam/build/trajectory.csv";
    std::string gt_name = "/home/sqn/SLAM_Data/EuroC_dataset_MH_3_medium/state_groundtruth_estimate0/data.csv";
    std::string out1 = "/home/sqn/repo-clone/evaluate_ate_scale/gt.txt";
    std::string out2 = "/home/sqn/repo-clone/evaluate_ate_scale/est.txt";
    LoadSlamPoseFromFile(file_name, out2);
    LoadGroundTruthFromFile(gt_name, out1);
    //PlotTrajectory();
  }
  void LoadGroundTruthFromFile(const std::string& file_name);

  void LoadSlamPoseFromFile(const std::string& file_in, const std::string& file_out) {
    // FILE* fp = fopen(file_in.c_str(), "r");
    // FILE* fp_path = fopen(file_out.c_str(), "w");
    // CHECK(fp) << "Invalid pose result file: " << file_in;
    // while (feof(fp) == 0) {
    //   double px, py, pz, qx, qy, qz, qw;
    //   long double t;
    //   int n = fscanf(fp, "%Lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf%*s\n", &t, 
    //             &px, &py, &pz, &qx, &qy, &qz, &qw);
    //   fprintf(fp_path, "%f %f %f\n", px, py, pz);
    
    // }
    // fclose(fp);
    // fclose(fp_path);
    std::ifstream in(file_in);
    CHECK(in.is_open()) << "Open file failed: " << file_in;

    std::ofstream out(file_out);
    CHECK(out.is_open()) << "Open file failed: " << file_out;

    std::string line;
    double t, px, py, pz, qx, qy, qz, qw;
    while (getline(in, line)) {
      for (auto& c : line) {
        c = c == ',' ? ' ' : c;
      }
      std::istringstream iss(line);
      
      bool t1 = iss >> t >> px >> py >> pz >> qx >> qy >> qz >> qw;
      CHECK(t1) << "Parsing string failed.";

      out << std::fixed << t << " " << px << " " << py << " " << pz << " "
          << qx << " " << qy << " " << qz << " " << qw << "\n";
    }

    in.close();
    out.close();
  }

  void LoadGroundTruthFromFile(const std::string& file_in, const std::string& file_out) {
    std::ifstream in(file_in);
    CHECK(in.is_open()) << "Open file failed: " << file_in;

    std::ofstream out(file_out);
    CHECK(out.is_open()) << "Open file failed: " << file_out;

    std::string line;
    double t, px, py, pz, qx, qy, qz, qw;
    while (getline(in, line)) {
      // This line is a comment
      if (line[0] == '#') continue;
      for (auto& c : line) {
        c = c == ',' ? ' ' : c;
      }
      std::istringstream iss(line);
      
      bool t1 = iss >> t >> px >> py >> pz >> qw >> qx >> qy >> qz ;
      t = t / 1e9;
      CHECK(t1) << "Parsing string failed.";

      out << std::fixed << t << " " << px << " " << py << " " << pz << " "
          << qx << " " << qy << " " << qz << " " << qw << "\n";
    }

    in.close();
    out.close();
  }

  void PlotTrajectory() {
    std::string trajectory_path = "/home/sqn/01.gp";
    std::string tra_path = "/home/sqn/01.txt";
    char command[1024];
    FILE *fp = fopen(trajectory_path.c_str(), "w");

    fprintf(fp, "set term png size 900,900\n");
    fprintf(fp, "set output \"01.png\"\n");
    fprintf(fp, "splot \"/home/sqn/01.txt\"\n");

    fclose(fp);
    sprintf(command, "cd /home/sqn; gnuplot %s", trajectory_path.c_str());
    system(command);

    // Save pdf and crop
    //sprintf
  }
private:
  using PoseMap = std::map<double, Eigen::Matrix4d>;
  PoseMap pose_est_, pose_gt_;

};

} // namespace lslam

int main(int argc, char **argv) {
  lslam::Evaluation eval;
  eval.Evaluate();
  return 0;
}