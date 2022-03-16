/**
*                                                                 
* @class IIR filter                                                        
*                                                                   
* 2009 Grzegorz Tyma @ Flytronic   								 
*/

#include "PilotIncludes.h"


/**
* Constructor
*/
Iir::Iir(void)
{
	iirInitialize(&iirState, IIR_MAX_SECTION);

	// 1 section, LP filter	
	iirState.numberOfSections = 1;
	iirReadCoefInternal(&iirCoef, iirState.numberOfSections);
}

/**
* initialize delay line of section
*/
void Iir::iirSectionInitialize( struct iirSectionStateT *state )
{
	state->d1 = 0.0f;
	state->d2 = 0.0f;
}

/**
* initialize all sections
*/
void Iir::iirInitialize( struct iirStateT *state, int noOfSections)
{
	int i;
	for(i=0;i<IIR_MAX_SECTION;i++)
	{
		iirSectionInitialize(&state->sectionState[i]);
	}

	state->numberOfSections = noOfSections;
}

void Iir::iirReadCoef(int numberOfSections)
{
	iirReadCoefInternal(&iirCoef, numberOfSections);
}

/**
* skeleton for coeffictient reading
*/
void Iir::iirReadCoefInternal(struct iirCoefT *coeff, int numberOfSections )
{
	int i, nos;

	nos = numberOfSections;
	if(nos>IIR_MAX_SECTION) nos = IIR_MAX_SECTION;

	for(i=0;i<nos;i++)
	{
		coeff->sectionCoef[i].a01 = 1.0f;
		coeff->sectionCoef[i].a11 = 0.0f;
		coeff->sectionCoef[i].a21 = 0.0f;

		coeff->sectionCoef[i].b01 = 1.0f;
		coeff->sectionCoef[i].b11 = 0.0f;
		coeff->sectionCoef[i].b21 = 0.0f;

		coeff->sectionCoef[i].gainInput = 1.0f;
	}

	coeff->gainOutput = 1.0f;

	//Example of IIR filter
	//Chebyshev, LP, fs=24Hz, fp=0.5Hz, fs=3Hz, Ap=1dB, As=30dB;
		
	coeff->sectionCoef[0].a01 = 1.0f;
	coeff->sectionCoef[0].a11 = -1.7115955335931829f;
	coeff->sectionCoef[0].a21 = 0.74914883700233958f;

	coeff->sectionCoef[0].b01 = 1.0f;
	coeff->sectionCoef[0].b11 = -0.97808335282173664f;
	coeff->sectionCoef[0].b21 = 1.0f;

	coeff->sectionCoef[0].gainInput = 0.036747912379008173f;
}

/**
* Process input data through section 
* Each section has the following transmitance:
* H(z) = gainInput * (b01 + b11*z^-1 + b21*z^-2)/(a01 + a11*z^-1 + a21*z^-2)
*/
float Iir::iirSectionProcessSample(float input, struct iirSectionStateT *state, const struct iirSectionCoefT& coeff)
{
	float output;
	float d;
	
	//assumption: a01 is equal 1
	d = coeff.gainInput*input - coeff.a11*state->d1 - coeff.a21*state->d2;
	output = d*coeff.b01 + coeff.b11*state->d1 + coeff.b21*state->d2;
	state->d2 = state->d1;
	state->d1 = d;

	return output;
}

/**
* Process input data 
*/
float Iir::iirProcessSample(float input)
{
	return iirProcessSampleInternal(input, &iirState, iirCoef);
}

float Iir::iirProcessSampleInternal(float input, struct iirStateT *state, const struct iirCoefT& coeff)
{
	float out=0.0f, in;
	int i, nos;

	nos = state->numberOfSections;
	if(nos>IIR_MAX_SECTION) nos = IIR_MAX_SECTION;

	
	in = input;

	for(i=0;i<nos;i++)
	{
		out = iirSectionProcessSample(in, &state->sectionState[i], coeff.sectionCoef[i]);
		in = out;
	}

	return out;
}
