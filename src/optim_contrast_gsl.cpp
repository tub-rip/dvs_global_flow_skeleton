#include "dvs_global_flow/image_warped_events.h"
#include "dvs_global_flow/global_flow_estimator.h"
#include "dvs_global_flow/numerical_deriv.h"

#include <glog/logging.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_blas.h>

#include <opencv2/highgui/highgui.hpp>


double contrast_MeanSquare(const cv::Mat& image)
{
  // Compute mean square value of the image
  double contrast = cv::norm(image,cv::NORM_L2SQR) / static_cast<double>(image.rows*image.cols);
  //LOG(INFO) << "mean square = " << contrast;
  return contrast;
}


double contrast_Variance(const cv::Mat& image)
{
  // Compute variance of the image
  double contrast;
  // FILL IN ...
  // hint: use OpenCV functions
  return contrast;
}


double computeContrast(
    const cv::Mat& image,
    const int contrast_measure
    )
{
  // Branch according to contrast measure
  double contrast;
  switch (contrast_measure)
  {
    case MEAN_SQUARE_CONTRAST:
      contrast = contrast_MeanSquare(image);
      break;
    default:
      contrast = contrast_Variance(image);
      break;
  }

  return contrast;
}


/**
 * @brief Auxiliary data structure for optimization algorithm
 */
typedef struct {
  std::vector<dvs_msgs::Event> *poEvents_subset;
  cv::Size * img_size;
  dvs_global_flow::OptionsMethod * opts;
} AuxdataBestFlow;


/**
 * @brief Main function used by optimization algorithm.
 * Maximize contrast, or equivalently, minimize (-contrast)
 */
double contrast_f_numerical (const gsl_vector *v, void *adata)
{
  // Extract auxiliary data of cost function
  AuxdataBestFlow *poAux_data = (AuxdataBestFlow *) adata;

  // Parameter vector (from GSL to OpenCV)
  cv::Point2d vel( gsl_vector_get(v,0), gsl_vector_get(v,1) );

  // Compute cost
  double contrast;
  // FILL IN ...
  // hint: Call computeImageOfWarpedEvents() and  computeContrast()

  return -contrast;
}


void contrast_fdf_numerical (const gsl_vector *v, void *adata, double *f, gsl_vector *df)
{
  // Finite difference approximation
  *f = vs_gsl_Gradient_ForwardDiff (v, adata, contrast_f_numerical, df, 1e0);
}


void contrast_df_numerical (const gsl_vector *v, void *adata, gsl_vector *df)
{
  double cost;
  contrast_fdf_numerical (v, adata, &cost, df);
}



namespace dvs_global_flow {

cv::Point2d GlobalFlowEstimator::findBestFlowInRangeBruteForce(const cv::Vec4d& vel_range,
                                                                const cv::Vec2d& vel_step)
{
  //Auxiliary data for the cost function
  AuxdataBestFlow oAuxdata;
  oAuxdata.poEvents_subset = &events_subset_;
  oAuxdata.img_size = &img_size_;
  oAuxdata.opts = &opts_;

  gsl_vector *vx_gsl = gsl_vector_alloc (2);

  // Brute-force search over a grid of possible global flows
  double vmin_x = vel_range(0), vmax_x = vel_range(1),
         vmin_y = vel_range(2), vmax_y = vel_range(3);
  double step_vx = vel_step(0), step_vy = vel_step(1);

  if(opts_.verbose_)
  {
    LOG(INFO) << "Flow search:";
    LOG(INFO) << "vx: " << vmin_x << " -> " << vmax_x << " step: " << step_vx;
    LOG(INFO) << "vy: " << vmin_y << " -> " << vmax_y << " step: " << step_vy;
  }

  double minimum_cost = 0.0;
  double opt_vx = 0., opt_vy = 0.;
  for(double vx = vmin_x; vx <= vmax_x; vx += step_vx)
  {
    for(double vy = vmin_y; vy <= vmax_y; vy += step_vy)
    {
      gsl_vector_set (vx_gsl, 0, vx);
      gsl_vector_set (vx_gsl, 1, vy);
      const double cost = contrast_f_numerical(vx_gsl, &oAuxdata);

      if(cost < minimum_cost)
      {
        minimum_cost = cost;
        opt_vx = vx;
        opt_vy = vy;
      }
    }
  }

  if(opts_.verbose_)
    LOG(INFO) << "Best flow: " << opt_vx << " , " << opt_vy;

  return cv::Point2d(opt_vx, opt_vy);
}

void GlobalFlowEstimator::findInitialFlow()
{
  // Hierarchical search to find a good initial flow
  cv::Point2d v_opt(0.,0.);
  std::vector<double> steps = {1000., 500., 250., 125., 50., 25.};

  for(size_t i=1; i<steps.size(); ++i)
  {
    const double prev_step = steps[i-1];
    const double step = steps[i];
    cv::Vec4d vel_range(v_opt.x-prev_step, v_opt.x+prev_step,
                        v_opt.y-prev_step, v_opt.y+prev_step);
    v_opt = findBestFlowInRangeBruteForce(vel_range, cv::Vec2d(step,step));
  }
  LOG(INFO) << "Found initial flow: " << v_opt;
  vel_.x = v_opt.x;
  vel_.y = v_opt.y;
}


// Maximize the contrast with respect to the global flow
double GlobalFlowEstimator::maximizeContrast()
{
  //MINIMIZATION ALGORITHM

  //PREPARE SOLVER
  //Solver/minimizer type (algorithm):
  const gsl_multimin_fdfminimizer_type *solver_type;
  solver_type = gsl_multimin_fdfminimizer_conjugate_fr; // Fletcher-Reeves conjugate gradient algorithm

  //Auxiliary data for the cost function
  AuxdataBestFlow oAuxdata;
  oAuxdata.poEvents_subset = &events_subset_;
  oAuxdata.img_size = &img_size_;
  oAuxdata.opts = &opts_;

  //Routines to compute the cost function and its derivatives
  gsl_multimin_function_fdf solver_info;

  const int num_params = 2; // Size of global flow
  solver_info.n = num_params; // Size of the parameter vector
  solver_info.f = contrast_f_numerical; // Cost function
  solver_info.df = contrast_df_numerical; // Gradient of cost function
  solver_info.fdf = contrast_fdf_numerical; // Cost and gradient functions
  solver_info.params = &oAuxdata; // Auxiliary data

  //Initial parameter vector
  gsl_vector *vx = gsl_vector_alloc (num_params);

  // FILL IN ...
  // gsl_vector_set (vx, ...


  //Initialize solver
  gsl_multimin_fdfminimizer *solver = gsl_multimin_fdfminimizer_alloc (solver_type, num_params);
  const double initial_step_size = 10;
  double tol = 0.05;

  gsl_multimin_fdfminimizer_set (solver, &solver_info, vx, initial_step_size, tol);

  const double initial_cost = solver->f;

  //ITERATE

  const int num_max_line_searches = 50;
  int status;
  const double epsabs_grad = 1e-3, tolfun=1e-2;
  double cost_new = 1e9, cost_old = 1e9;
  size_t iter = 0;
  if (opts_.verbose_ >= 2)
  {
    LOG(INFO) << "Optimization. Solver type = " << solver_type->name;
    LOG(INFO) << "iter=" << std::setw(3) << iter << "  vel=["
              << gsl_vector_get(solver->x, 0) << " "
              << gsl_vector_get(solver->x, 1) << "]  cost=" << std::setprecision(8) << solver->f;
  }

  do
  {
    iter++;
    cost_old = cost_new;
    status = gsl_multimin_fdfminimizer_iterate (solver);
    //status == GLS_SUCCESS (0) means that the iteration reduced the function value

    if (opts_.verbose_ >= 2)
    {
      LOG(INFO) << "iter=" << std::setw(3) << iter << "  vel=["
                << gsl_vector_get(solver->x, 0) << " "
                << gsl_vector_get(solver->x, 1) << "]  cost=" << std::setprecision(8) << solver->f;
    }

    if (status == GSL_SUCCESS)
    {
      //Test convergence due to stagnation in the value of the function
      cost_new = gsl_multimin_fdfminimizer_minimum(solver);
      if ( fabs( 1-cost_new/(cost_old+1e-7) ) < tolfun )
      {
        if (opts_.verbose_ >= 3)
          LOG(INFO) << "progress tolerance reached.";
        break;
      }
      else
        status = GSL_CONTINUE;
    }

    //Test convergence due to absolute norm of the gradient
    if (GSL_SUCCESS == gsl_multimin_test_gradient (solver->gradient, epsabs_grad))
    {
      if (opts_.verbose_ >= 3)
        LOG(INFO) << "gradient tolerance reached.";
      break;
    }

    if (status != GSL_CONTINUE)
    {
      // The iteration was not successful (did not reduce the function value)
      if (opts_.verbose_ >= 3)
        LOG(INFO) << "stopped iteration; status = " << status;
      break;
    }
  }
  while (status == GSL_CONTINUE && iter < num_max_line_searches);

  //SAVE RESULTS (best global flow velocity)

  //Convert from GSL to OpenCV format
  gsl_vector *final_x = gsl_multimin_fdfminimizer_x(solver);

  // FILL IN ...  the return value of vel_ using  final_x


  const double final_cost = gsl_multimin_fdfminimizer_minimum(solver);

  if (opts_.verbose_ >= 1)
  {
    LOG(INFO) << "--- Initial cost = " << std::setprecision(8) << initial_cost;
    LOG(INFO) << "--- Final cost   = " << std::setprecision(8) << final_cost;
    LOG(INFO) << "--- iter=" << std::setw(3) << iter << "  vel=[" << vel_.x << " " << vel_.y << "]";
  }

  //Release memory used during optimization
  gsl_multimin_fdfminimizer_free (solver);
  gsl_vector_free (vx);

  return final_cost;
}

} // namespace
