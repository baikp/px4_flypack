#pragma once

#include "matrix/math.hpp"
namespace discrete{

/*
 *DEN must larger than zero
 *NUM can not larger than DEN
 */
template<typename Type, size_t NUM, size_t DEN>
class DiscreteTransferFunction
{

public:
	DiscreteTransferFunction()
	{
		outputHistory_.setZero();
		inputHistory_.setZero();
		numerator_.setZero();
		denominator_.setZero();
	}
	~DiscreteTransferFunction(){};

	virtual void update_param(Type dt){	}

	void resetHist(Type sample){
		outputHistory_.setAll(sample);
		inputHistory_.setAll(sample);
	}

	bool update(Type input,Type &output,Type dt){
		//计算分子、分母系数
		update_param(dt);
		//确认是否初始化成功
		if( ( (denominator_(0) < 1e-4f) && (denominator_(0) > -1e-4f) ) || (NUM > DEN) ) return false;
		else {
			numerator_ /= denominator_(0);
			denominator_ /= denominator_(0);
		}
		//输入输出保存队列中赋值
		inputHistory_(0) = input;
		outputHistory_(0) = 0; //确保y(now)=0，

		//计算
		outputHistory_(0) = numerator_.dot( inputHistory_.template slice<NUM, 1>((DEN - NUM),0) ) - denominator_.dot(outputHistory_);
		output = outputHistory_(0);
		//输入输出历史移位保存
		for(int i = DEN-1;i > 0;i--){
			outputHistory_(i) = outputHistory_(i - 1);
			inputHistory_(i) = inputHistory_(i - 1);
		}
		return true;
	}

protected:
	matrix::Vector<Type,NUM> numerator_; // 分子系数
	matrix::Vector<Type,DEN> denominator_; // 分母系数
	matrix::Vector<Type,DEN> outputHistory_; // 输出历史
	matrix::Vector<Type,DEN> inputHistory_; // 输入历史
};


/* tustin */
class DiscreteIntegrator : public DiscreteTransferFunction<float, 2, 2> {
public:
	DiscreteIntegrator(){};
	~DiscreteIntegrator(){};

	void update_param(float dt) override {
		float nums[2] = {dt   , dt};
		float dens[2] = {2.0f , -2.0f};
		numerator_.copyFrom(nums);
		denominator_.copyFrom(dens);
	}
};

/* tustin */
class DiscreteDerivative : public DiscreteTransferFunction<float, 2, 2> {
private:
	float filter_coff = 50.f;//rad/s
public:
	DiscreteDerivative(float coff = 50.f):filter_coff(coff){};
	~DiscreteDerivative(){};
	void setCoff(float coff){
		filter_coff = coff;
	}
	void update_param(float dt) override {
		float nums[2] = {2.0f*filter_coff         , -2.0f*filter_coff};
		float dens[2] = {(dt*filter_coff + 2.0f)  , (dt*filter_coff - 2.0f)};
		numerator_.copyFrom(nums);
		denominator_.copyFrom(dens);
	}
};

/* tustin */
class DiscreteFirstOrderInertialLink : public DiscreteTransferFunction<float, 2, 2> {
private:
	float time_constant = 0.15f;//s
public:
	DiscreteFirstOrderInertialLink(float constant = 0.15f):time_constant(constant){};
	~DiscreteFirstOrderInertialLink(){};
	void setConstant(float constant){
		time_constant = constant;
	}
	void update_param(float dt) override {
		float nums[2] = {dt  , dt};
		float dens[2] = {(dt + 2.0f*time_constant)  , (dt - 2.0f*time_constant)};
		numerator_.copyFrom(nums);
		denominator_.copyFrom(dens);
	}
};

} //namespace discrete

