//==============================================================================
//         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
//
//          Distributed under the Boost Software License, Version 1.0.
//                 See accompanying file LICENSE.txt or copy at
//                     http://www.boost.org/LICENSE_1_0.txt
//==============================================================================

#ifdef WIN32
#include <random>
#include <vector>
#include <cmath>
#endif

#include <libv/lma/lma.hpp>
#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>
#include <slam/se3_spline.h>
#include <slam/readsensor.h>
#include <slam/constraint.h>

struct splineSegmentObj
{
    typedef SE3Group<double> SE3Type;
    double** parameters;
    int num_knot = 20;
    int num_parameters = SE3Type::num_parameters;
    int total_parameters_num = num_knot * num_parameters;
    
}

namespace lma
{
    template<> struct Size<double**> {enum {value = splineSegmentObj.total_parameters_num};};

    template<int I> void apply_small_increment(double **obj, double d, v::numeric_tag<I>, const Adl&){
        obj[I/7*7][I-I/7*7-1] += d;
    } 

    void apply_increment(double** obj, const double increment[splineSegmentObj.total_parameters_num], const Adl&){
        for (size_t i=0; i<splineSegmentObj.total_parameters_num;  ++i)
            obj[i/7*7][i-i/7*7-1] += increment[i]; 
    }
}

int main(int , char **)
{
  std::string filename("loop_20fps.bag");
  std::vector<gps_coordinate> coord;
  double x[3] = {4, 2, 2};

  std::cout << "x,y,r = " << x[0] << "," << x[1] << "," << x[2] << "\n";

  readsensor(filename, coord);

  // Parameterize r as m^2 so that it can't be negative.
  x[0] = x[1] = x[2] = 3;
  x[2] = sqrt(x[2]);

  std::cout << "x,y,r = " << x[0] << "," << x[1] << "," << x[2] << "\n";

  // Initialize spline
  size_t num_knots = coord.size()-splineSegmentObj.num_knot;
  size_t num_eval = 5;

  UniformSpline<double> spline = create_zero_spline(num_knots);
  auto eval_times = create_eval_times(spline, num_eval);
  GPSSplineConstraint* constraint = new GPSSplineConstraint(spline, eval_times, coord, 1.0);
   
  std::vector<double*>parameter_blocks;
  for (size_t i=0; i < spline.num_knots(); ++i){
      parameter_blocks.push_back(spline.get_knot_data(i));

  }

  // Initialize the solver
  double lambda = 0.001;
  double iteration_max = 5;
  lma::Solver<GPSSplineConstraint> solver(lambda, iteration_max);
  
  // Add constraint
  for(size_t i = 0 ; i < coord.size() ; ++i){
    //std::vector<double*> *temp = &parameter_blocks;
    double j = i/10;
    if (j*10+9 < coord.size())
        solver.add(GPSSplineConstraint(spline, eval_times,coord,1.0),&parameter_blocks[j*10:j*10+9]); 
    else
        solver.add(GPSSplineConstraint(spline, eval_times,coord,1.0),&parameter_blocks[j*10:coord.size()-1]); 
  }
 
  solver.solve(lma::DENSE,lma::enable_verbose_output());

//  for (size_t i = 0; i < parameter_blocks.size(); ++i){
//    std::cout <<paramter_blocks[i] << endl;
//  }

  return 0;
}



#if 0

#include <libv/lma/lma.hpp>

using namespace lma;
using namespace Eigen;

double distance(const Vector3d& circle, const Vector2d& point)
{
  return (circle.head<2>() - point).norm() - circle[2];
};

struct Error
{
  bool operator()(const Vector3d& circle, const Vector2d& point, double& residual) const
  {
    residual = distance(circle,point);
    return true;
  }
};

int main()
{
  std::cout << "Error before = " << distance(circle,point) << std::endl;

  Vector3d circle(0,0,100);
  Vector2d point(60.,10.);

  Solver<Error> solver;

  solver.add(Error{},&circle,&point);

  solver.solve(DENSE_SCHUR);

  std::cout << "Error after  = " << distance(circle,point) << std::endl;
}

#endif
