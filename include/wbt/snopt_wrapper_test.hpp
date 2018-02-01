#ifndef SNOPT_WRAPPER_TEST_H
#define SNOPT_WRAPPER_TEST_H

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

#include <Optimizer/snopt/include/snoptProblem.hpp>

namespace snopt_wrapper_test{
  void usrFG_sample(int    *Status, int *n,    double x[],
     int    *needF,  int *lenF,  double F[],
     int    *needG,  int *lenG,  double G[],
     char      *cu,  int *lencu,
     int    iu[],    int *leniu,
     double ru[],    int *lenru);

  void solve_sample_problem();
}


#endif