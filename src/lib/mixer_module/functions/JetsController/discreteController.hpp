#pragma once

#include "discreteTransferFun.hpp"
#include "jetEngineInterp.hpp"
#include <mathlib/math/filter/LowPassFilter2p.hpp>

// #include "stdio.h"

namespace discrete{

class discrete2DofPID
{
private:
    /* data */
    float pid_kp = 2.652f;
    float pid_ki = 0.0f;
    float pid_kd = 0.0f;
    float filter_coff = 50.0f; // rad/s filter cutoff freq
    float anti_integral_saturation_Tt = 1.0f/0.2236f; //  1/Tt
    float upper_bound = 300.0f;
    float lower_bound = -300.0f;
    float last_output = 0.0f;
    DiscreteIntegrator discrete_integ_;
    DiscreteDerivative discrete_deriv_;
    DiscreteFirstOrderInertialLink discrete_feedforward_;
    DiscreteFirstOrderInertialLink discrete_lowpass_;

    float ff_output = 0.0f;
    float p_output = 0.f;
    float i_output = 0.0f;
    float d_output = 0.0f;
    float error = 0.f;
	math::LowPassFilter2p<float> _d_lpf{100.f, 3.f};

    float clip(float input, float up,float low){
        return (input > up) ? (up) : ( (input < low) ? (low) : (input) );
    }

public:
    discrete2DofPID(float kp = 4.5f,
                    float ki = 6.5f,
                    float kd = 0.3f,
                    float coff = 20.0f,
                    float ff_timeconstant = 0.15f,
                    float Tt = 1.f/0.2774f*0.6f,
                    float up_bound = 300.0f,
                    float low_bound = -300.0f):
                    pid_kp(kp),
                    pid_ki(ki),
                    pid_kd(kd),
                    filter_coff(coff),
                    anti_integral_saturation_Tt(Tt),
                    upper_bound(up_bound),
                    lower_bound(low_bound),
                    discrete_deriv_(coff),
                    discrete_feedforward_(ff_timeconstant)
    {
        last_output = 0.0f;
        discrete_lowpass_.setConstant(1/(25.f*3.14f));
    }
    ~discrete2DofPID(){};

    void resetControllerState(float ref, float feedback){
        error = ref - feedback;
        discrete_integ_.resetHist(0.f);
        discrete_deriv_.resetHist(error*pid_kd);
        discrete_feedforward_.resetHist(ref);
        discrete_lowpass_.resetHist(error*pid_kp);
        _d_lpf.reset(error*pid_kd);
    }

    void setGain(float kp,float ki,float kd, float Tt){
        pid_kp = kp;
        pid_ki = ki;
        pid_kd = kd;
        anti_integral_saturation_Tt = Tt;
    }

    void getSeparateOut(float &_p_out, float &_i_out, float &_d_out, float &_ff_out, float &_error){
        _p_out = p_output;
        _i_out = i_output;
        _d_out = d_output;
        _ff_out = ff_output;
        _error = error;
    }

    bool update(float ref,float feedback, float dt,float &output){
        error = ref - feedback;
        ff_output = 0.0f;
        discrete_lowpass_.update(error*pid_kp,p_output,dt);
        // p_output = error*pid_kp;
        i_output = 0.0f;
        d_output = 0.0f;

        //calculate feedforward term
        if( !discrete_feedforward_.update(ref,ff_output,dt) ) ff_output = 0.0f;
        //calculate derivative term
        if( !discrete_deriv_.update( _d_lpf.apply(error*pid_kd) ,d_output,dt) ) d_output = 0.0f;

        //calculate integrator term
        if( !discrete_integ_.update( ( ( clip(last_output,upper_bound,lower_bound) - last_output)*anti_integral_saturation_Tt + error*pid_ki) ,
                                     i_output ,dt ) ) i_output = 0.0f;

        last_output = p_output + i_output + d_output;

        output = clip(clip(last_output,upper_bound,lower_bound) + ff_output,
                 1000.0f,
                 0.0f);

        return true;
    }

};


} //namespace discrete
