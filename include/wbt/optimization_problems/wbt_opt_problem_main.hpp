#ifndef WBT_OPTIMIZATION_PROBLEM_MAIN_H
#define WBT_OPTIMIZATION_PROBLEM_MAIN_H
#include <Utils/wrap_eigen.hpp>
#include <wbt/optimization_constants.hpp>
#include <string>

class Optimization_Problem_Main{
public:
  Optimization_Problem_Main(){}
  virtual ~Optimization_Problem_Main(){}

  std::string problem_name = "Undefined Optimization Problem";
};

#endif