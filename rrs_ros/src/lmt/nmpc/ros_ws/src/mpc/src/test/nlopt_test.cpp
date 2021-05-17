#include <nlopt.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <ros/ros.h>

using namespace std;

typedef struct {
    double a, b;
} my_constraint_data;


//class myOpt
//{
//private:
//  double mysqrt(double my_x)
//  {
//    return sqrt(my_x);
//  }
////  static double myfunc(const std::vector<double> &x, std::vector<double> &grad,void *data)
////  {
////      myOpt* my_obj = reinterpret_cast<myOpt*>(data);

////          grad[0] = 0.0;
////          grad[1] = 0.5 / my_obj->mysqrt(x[1]);


////      return  my_obj->mysqrt(x[1]);
////  }

//  static double myfunc(unsigned n, const double *x, double* grad, void *data)
//  {
////      myOpt* my_obj = reinterpret_cast<myOpt*>(data);
//      if(grad)
//      {
//        grad[0] = 0.0;
//        grad[1] = 0.5 / sqrt(x[1]);
//      }
////      double value = my_obj->mysqrt(x[1]);
////      cout << "cost" << endl;
//      cout << "x1: " << x[0] << ", x2: " << x[1] << endl;
////      cout << "value: " << sqrt(x[1]) << ", grad1: " << grad[0] << ", grad2: " << grad[1] << endl;
//      return sqrt(x[1]);
//  }



//  static double myconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
//  {
//      my_constraint_data *d = reinterpret_cast<my_constraint_data*>(data);
//      double a = d->a, b = d->b;

//          grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
//          grad[1] = -1.0;
//      return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
//  }

//  static void multiconstraint(unsigned m, double* result,unsigned n,const double* x, double* grad, void* data)
//  {
//      vector<my_constraint_data>* d = reinterpret_cast<vector<my_constraint_data>*>(data);
//      //double a1=(*d)[0].a,b1 = (*d)[0].b, a2=(*d)[1].a,b2 = (*d)[1].b;
//      for(int i=0;i<m;i++)
//      {
//        double a=(*d)[i].a,b=(*d)[i].b;
//        result[i] = (a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1];

//        grad[i*n] =  3 * a * (a*x[0] + b) * (a*x[0] + b);
//        grad[i*n+1] = -1.0;
//      }
////      cout << "constraint" << endl;
////      cout << "grad1: " << grad[0] << ", grad2: " << grad[1] << ", grad3: " << grad[2] << ", grad4: " << grad[3] << endl;
////      cout << "==================" << endl;
//  }

//  nlopt::opt opt;
//  vector<double> lb;
////  vector<double> x;
//  vector<my_constraint_data> data;
//public:
//  myOpt()
//  {
//    opt = nlopt::opt(nlopt::LD_SLSQP, 2);
//    lb.resize(2);
//    lb[0] = -HUGE_VAL;
//    lb[1] = 0;

//    opt.set_lower_bounds(lb);
//    opt.set_min_objective(myfunc,NULL);

//    data.resize(2);
//    data[0].a=2;data[0].b=0;
//    data[1].a=-1;data[1].b=1;

////    opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
////    opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);
//    vector<double> tol(2,1e-8);
//    opt.add_inequality_mconstraint(multiconstraint,&data,tol);
//    opt.set_xtol_rel(1e-4);

//  }

//  void optimize()
//  {
//    vector<double> x(2);
//    x[0]=0;x[1]=2;
////    x[0] = 1; x[1] = 1;
//    double minf;
////    data[0].a=2;data[0].b=0;
////    data[1].a=-1;data[1].b=1;

//    try{
//        nlopt::result result = opt.optimize(x, minf);
//        std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
//            << std::setprecision(10) << minf << std::endl;
//    }
//    catch(std::exception &e) {
//        std::cout << "nlopt failed: " << e.what() << std::endl;
//    }
//  }

//};



double myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
//    cout << "cost" << endl;
    cout << "x1: " << x[0] << ", x2: " << x[1] << endl;
    return sqrt(x[1]);
}

void multiconstraint(unsigned m, double* result,unsigned n,const double* x, double* grad, void* data)
{
    vector<my_constraint_data>* d = reinterpret_cast<vector<my_constraint_data>*>(data);
    //double a1=(*d)[0].a,b1 = (*d)[0].b, a2=(*d)[1].a,b2 = (*d)[1].b;
    for(int i=0;i<m;i++)
    {
      double a=(*d)[i].a,b=(*d)[i].b;
      result[i] = (a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1];

      grad[i*n] =  3 * a * (a*x[0] + b) * (a*x[0] + b);
      grad[i*n+1] = -1.0;
    }
//    cout << "constraint" << endl;
//      cout << "grad1: " << grad[0] << ", grad2: " << grad[1] << ", grad3: " << grad[2] << ", grad4: " << grad[3] << endl;
}

int main(int argc, char** argv)
{
  nlopt::opt opt(nlopt::LD_SLSQP, 2);
  std::vector<double> lb = {-HUGE_VAL,0};
  opt.set_lower_bounds(lb);
  opt.set_min_objective(myfunc, NULL);
  vector<my_constraint_data> data = { {2,0}, {-1,1} };
  vector<double> tol(2,1e-8);
  opt.add_inequality_mconstraint(multiconstraint,&data,tol);
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(2);
  std::vector<double> x(2);
  x[0] = 1.234; x[1] = 2.345;
  double minf;

  for(int i=0;i<10;i++)
  {
    try{
        nlopt::result result = opt.optimize(x, minf);
        std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
            << std::setprecision(10) << minf << std::endl;
        cout << result << endl;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }
  }
//  myOpt my_opt;
//  ros::init(argc, argv, "mytest");
//  ros::NodeHandle n;
//  ros::Time start_time = ros::Time::now();
//  my_opt.optimize();
//  ros::Duration dur = ros::Time::now()-start_time;
//  cout << dur.toSec() << endl;

}
