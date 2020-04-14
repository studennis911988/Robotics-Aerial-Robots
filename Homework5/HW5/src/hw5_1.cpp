#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <fstream>
#include <string>
#include <ctime>
#include <sys/stat.h>

using namespace std;

// loss function
struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x1, double x2, double y) : _x1(x1), _x2(x2), _y(y) {}

  template<typename T>
  bool operator()(
    const T *const ab,
    T *residual) const {
    residual[0] = T(_y) - (ab[0] * T(_x1) + ab[1] * T(_x2)); // y- (ax1 + bx2)
    return true;
  }

  const double _x1, _x2, _y;    // x,y数据
};

int main(int argc, char **argv) {
  double a = 1.0;
  double b = 1.0;
  int N = 50;  // num of data

 // get data from file
  std::string filename = argv[1];
  std::ifstream ifs(filename.c_str());
  std::string line;
  vector<double> input_data1, input_data2, out_data;
  for (int i = 0; i < N; i++) {
      std::getline(ifs, line);
      double in1, in2, out;

      if(sscanf(line.c_str(), "%lf,%lf,%lf", &in1, &in2, &out) == 3)
      {
        input_data1.push_back(in1);
        input_data2.push_back(in2);
        out_data.push_back(out);
      }
  }

  // params to be estimated
  double ab[2] = {a, b};


  // construct a problem
  ceres::Problem problem;
  for (int i = 0; i < N; i++) {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 2>(
        new CURVE_FITTING_COST(input_data1[i], input_data2[i], out_data[i])
      ),
      nullptr,
      ab
    );
  }


  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  cout << summary.BriefReport() << endl;
  cout << "estimated a,b = ";
  for (auto a:ab) cout << a << " ";
  cout << endl;

  return 0;
}
