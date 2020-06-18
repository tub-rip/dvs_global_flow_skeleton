#pragma once

#include "dvs_global_flow/util.h"
#include <glog/logging.h>


/**
 * \brief Concatenate two matrices horizontally
 * \param[in] Mat A and B
 * \param[out] Mat C = [A, B]
*/
void concatHorizontal(const cv::Mat& A, const cv::Mat& B, cv::Mat* C)
{
  CHECK_EQ(A.rows, B.rows) << "Input arguments must have same number of rows";
  CHECK_EQ(A.type(), B.type()) << "Input arguments must have the same type";

  // FILL IN ...
  // *C = ... etc
}
