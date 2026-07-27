#include "options.h"

namespace akf_lio::options
{
  float LIDAR_COV = 0.001;
  int NUM_MAX_ITERATIONS = 0;
  int MAX_FEA_NUM = 10000;
  double TIME_TO_DELETE_LOCAL_MAP = 1000;
  double init_free_ratio = 0.5;
  float T_MAL = 3; // mal threshold
  bool FLAG_EXIT = false;
} // namespace akf_lio::options