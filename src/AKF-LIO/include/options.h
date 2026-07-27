
#ifndef AKF_LIO_OPTIONS_H
#define AKF_LIO_OPTIONS_H

namespace akf_lio::options
{

  /// fixed params
  constexpr double INIT_TIME = 0.1;
  constexpr double LASER_POINT_COV = 0.001;
  constexpr int PUBFRAME_PERIOD = 20;
  constexpr int NUM_MATCH_POINTS = 100;   // required matched points in current
  constexpr int MIN_NUM_MATCH_POINTS = 5; // minimum matched points in current

  /// configurable params
  extern int NUM_MAX_ITERATIONS; // max iterations of ekf
  extern int MAX_FEA_NUM;
  extern double TIME_TO_DELETE_LOCAL_MAP;
  extern double init_free_ratio;
  extern float T_MAL; // mal threshold
  extern float LIDAR_COV;
  extern bool FLAG_EXIT; // flag for exitting

} // namespace akf_lio::options

#endif // AKF_LIO_OPTIONS_H
