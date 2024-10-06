#include "jetEngineInterp.hpp"
namespace discrete{

const float sw431_wrpm[JETENGINE_DATA_LEN] = {3.60,4.20,4.70,5.20,5.70,6.20,6.50,7   ,7.30,7.60,7.90,8.20,8.40,8.70,8.90,9   ,9.20,9.30,9.40,9.50,9.70};
const float sw430_wrpm[JETENGINE_DATA_LEN] = {3.60,4.60,5.10,5.50,6   ,6.40,6.90,7.10,7.40,7.80,8   ,8.30,8.60,8.70,8.90,9   ,9.20,9.30,9.40,9.50,9.70};
const float sw429_wrpm[JETENGINE_DATA_LEN] = {3.60,4.20,4.80,5.30,5.90,6.40,6.80,7.20,7.50,7.80,8.10,8.40,8.70,8.90,9   ,9.20,9.30,9.40,9.50,9.60,9.70};
const float sw428_wrpm[JETENGINE_DATA_LEN] = {3.58,4.30,4.90,5.40,6   ,6.40,6.90,7.20,7.50,7.90,8.20,8.50,8.80,8.90,9   ,9.20,9.30,9.40,9.50,9.60,9.70};
const float sw427_wrpm[JETENGINE_DATA_LEN] = {3.50,4.10,4.70,5.30,5.80,6.30,6.80,7.10,7.50,7.80,8.20,8.40,8.70,8.80,9   ,9.10,9.30,9.40,9.50,9.60,9.70};
const float sw426_wrpm[JETENGINE_DATA_LEN] = {3.60,4.20,4.70,5.20,5.70,6.10,6.50,6.90,7.20,7.50,7.80,8.10,8.30,8.50,8.70,8.90,9   ,9.20,9.30,9.40,9.70};
const float sw015_wrpm[JETENGINE_DATA_LEN] = {3.60,4.70,5.20,5.70,6.20,6.60,7   ,7.30,7.60,7.90,8.10,8.40,8.60,8.80,8.90,9   ,9.20,9.30,9.40,9.50,9.70};
const float sw013_wrpm[JETENGINE_DATA_LEN] = {3.50,4   ,4.50,5.10,5.60,6.10,6.50,7   ,7.30,7.80,8   ,8.30,8.50,8.70,8.90,9.10,9.30,9.40,9.50,9.60,9.70};

const float sw_all_throttle[JETENGINE_DATA_LEN] = {0,0.05,0.100,0.150,0.200,0.250,0.300,0.350,0.400,0.450,0.500,0.550,0.600,0.650,0.700,0.750,0.800,0.850,0.900,0.950,1};

const float wrpm_test[71] = {3,3.10,3.20,3.30,3.40,3.50,3.60,3.70,3.80,3.90,4,4.10,4.20,4.30,4.40,4.50,4.60,4.70,4.80,4.90,5,5.10,5.20,5.30,5.40,5.50,5.60,5.70,5.80,5.90,6,6.10,6.20,6.30,6.40,6.50,6.60,6.70,6.80,6.90,7,7.10,7.20,7.30,7.40,7.50,7.60,7.70,7.80,7.90,8,8.10,8.20,8.30,8.40,8.50,8.60,8.70,8.80,8.90,9,9.10,9.20,9.30,9.40,9.50,9.60,9.70,9.80,9.90,10};

JetInterp<float,JETENGINE_DATA_LEN> sw_interp_wt[8] = { JetInterp<float,JETENGINE_DATA_LEN>(sw431_wrpm,sw_all_throttle),
							JetInterp<float,JETENGINE_DATA_LEN>(sw430_wrpm,sw_all_throttle),
							JetInterp<float,JETENGINE_DATA_LEN>(sw429_wrpm,sw_all_throttle),
							JetInterp<float,JETENGINE_DATA_LEN>(sw426_wrpm,sw_all_throttle),
							JetInterp<float,JETENGINE_DATA_LEN>(sw015_wrpm,sw_all_throttle),
							JetInterp<float,JETENGINE_DATA_LEN>(sw013_wrpm,sw_all_throttle),
							JetInterp<float,JETENGINE_DATA_LEN>(sw428_wrpm,sw_all_throttle),
							JetInterp<float,JETENGINE_DATA_LEN>(sw427_wrpm,sw_all_throttle)};

JetInterp<float,JETENGINE_DATA_LEN> sw_interp_tw[8] = { JetInterp<float,JETENGINE_DATA_LEN>(sw_all_throttle,sw431_wrpm),
							JetInterp<float,JETENGINE_DATA_LEN>(sw_all_throttle,sw430_wrpm),
							JetInterp<float,JETENGINE_DATA_LEN>(sw_all_throttle,sw429_wrpm),
							JetInterp<float,JETENGINE_DATA_LEN>(sw_all_throttle,sw426_wrpm),
							JetInterp<float,JETENGINE_DATA_LEN>(sw_all_throttle,sw015_wrpm),
							JetInterp<float,JETENGINE_DATA_LEN>(sw_all_throttle,sw013_wrpm),
							JetInterp<float,JETENGINE_DATA_LEN>(sw_all_throttle,sw428_wrpm),
							JetInterp<float,JETENGINE_DATA_LEN>(sw_all_throttle,sw427_wrpm)};


}
