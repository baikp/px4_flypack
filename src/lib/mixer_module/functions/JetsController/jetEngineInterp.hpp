#pragma once

#include "matrix/math.hpp"

namespace discrete{

#define JETENGINE_DATA_LEN 21u

extern const float sw431_wrpm[JETENGINE_DATA_LEN];
extern const float sw430_wrpm[JETENGINE_DATA_LEN];
extern const float sw429_wrpm[JETENGINE_DATA_LEN];
extern const float sw428_wrpm[JETENGINE_DATA_LEN];
extern const float sw427_wrpm[JETENGINE_DATA_LEN];
extern const float sw426_wrpm[JETENGINE_DATA_LEN];
extern const float sw015_wrpm[JETENGINE_DATA_LEN];
extern const float sw013_wrpm[JETENGINE_DATA_LEN];

extern const float sw_all_throttle[JETENGINE_DATA_LEN];

extern const float wrpm_test[71];


template<typename Type, size_t M>
class JetInterp
{
private:
	void binarySearch(Type x, u_char &li, u_char &ri){
		int start = 0;
		int end = M - 1;
		int mid = (start + end)/2;
		uint32_t count = 0;

		if(x <= interp_x(0)){
			li = 0;
			ri = 0;
			return;
		}else if(x >= interp_x(M-1)){
			li = M-1;
			ri = M-1;
			return;
		}

		while ( (end - start ) > 1 && count < M)
		{
			mid = (start + end)/2;
			if(x < interp_x(mid) ){
				end = mid;
			}else{
				start = mid;
			}

			count++;
		}

		li = start;
		ri = end;

		return;

	}
	/* data */
	matrix::Vector<Type,M> interp_x;
	matrix::Vector<Type,M> interp_y;

	matrix::Vector<Type,M-1> k_x2y;
public:
	JetInterp(const Type x[M], const Type y[M])
	{
		interp_x.copyFrom(x);
		interp_y.copyFrom(y);

		for(u_char i = 0;i < M - 1;i++){
			k_x2y(i) = ( interp_y(i+1) - interp_y(i) )/( interp_x(i+1) - interp_x(i) );
		}
	}
	~JetInterp(){};

	int getValue(Type x,Type &output)
	{
		u_char li = 0;
		u_char ri = 0;

		binarySearch(x, li, ri);
		// printf("li: %d,ri: %d \n",li,ri);

		if(li == ri){
			output = interp_y(li);
			return li;
		}

		output = k_x2y(li)*( x - interp_x(li) ) + interp_y(li);

		return li;
	}
};

extern JetInterp<float,JETENGINE_DATA_LEN> sw_interp_wt[8];
extern JetInterp<float,JETENGINE_DATA_LEN> sw_interp_tw[8];


}
