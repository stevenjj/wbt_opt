#ifndef SNOPT_OPTIMIZATION_H
#define SNOPT_OPTIMIZATION_H

#include <stdio.h>
#include <string.h>
#include <iostream>

#include "wbt_optimization.h"
#include <Optimizer/snopt/include/snoptProblem.hpp>

void snopt_user_fun(int    *Status, int *n,    double x[],
       int    *needF,  int *neF,  double F[],
       int    *needG,  int *neG,  double G[],
       char      *cu,  int *lencu,
       int    iu[],    int *leniu,
       double ru[],    int *lenru);
void snopt_solve_opt_problem();

#endif