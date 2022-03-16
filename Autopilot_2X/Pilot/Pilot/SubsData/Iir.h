/**
*                                                                 
* @class IIR filter
*
* Infinite Impulse Response filter implementation
* Structure: Direct Form II, second order sections
* The max number of sections defined by IIR_MAX_SECTION
* Section transmitance:
* Hn(z) = gainInput * (b01 + b11*z^-1 + b21*z^-2)/(a01 + a11*z^-1 + a21*z^-2)
*
* Filter transmitance:
* H(z) = H1(z)*H2(z)*...*Hk(z) * gainOutput
* Assumption: a01 == 1
*
* 2009 Grzegorz Tyma @ Flytronic
*/

#ifndef IIR_H
#define IIR_H

class Iir
{

public:
	Iir(void);
	float iirProcessSample(float input);
	void iirReadCoef(int numberOfSections);


//max number of sections
	static const int IIR_MAX_SECTION=4;

	struct iirSectionStateT{
		float d1;
		float d2;
	};

	struct iirStateT{
		struct iirSectionStateT sectionState[IIR_MAX_SECTION];	
		int numberOfSections;
	};

	struct iirSectionCoefT{
		float b01;	//numerator
		float b11;
		float b21;
	
		float a01;	//denominator
		float a11;
		float a21;

		float gainInput;
	};

	struct iirCoefT{
		struct iirSectionCoefT sectionCoef[IIR_MAX_SECTION];
		float gainOutput;
	};


	struct iirCoefT iirCoef;
	struct iirStateT iirState;
	
private:
	void iirInitialize(struct iirStateT *state, int noOfSections);
	float iirProcessSampleInternal(float input, struct iirStateT *state, const struct iirCoefT& coeff);
	void iirSectionInitialize(struct iirSectionStateT *state);
	float iirSectionProcessSample(float input, struct iirSectionStateT *state, const struct iirSectionCoefT& coeff);
	void iirReadCoefInternal(struct iirCoefT *coeff, int numberOfSections);

};   

#endif  // IIR_H
