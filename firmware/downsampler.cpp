#include "downsampler.h"
#include <cmath>

//Constructor
Downsampler::Downsampler(int ratio)
{
		//Float vectors for storing coefficients
		vector<vector<float>> coeffs;	 
		vector<float> sos(6);	 

		if (ratio==5)
		{
			sos={(float)0.24523728,  0.24523728, 0.000000e+00,  1.000000e+00, -0.50952545,  0.000000e+00};
		}
		else if(ratio==10){
			sos={(float)0.02008337,  0.04016673,  0.02008337,  1.0, -1.56101808,0.64135154};
		}
		else if(ratio==20){
			sos={(float)0.00554272,  0.01108543,  0.00554272,  1.0, -1.77863178,0.80080265};
		}
		else if(ratio==50){
			sos={(float)9.44691844e-04,   1.88938369e-03,   9.44691844e-04, 1.00000000e+00,  -1.91119707e+00,   9.14975835e-01};
		}
		else if(ratio==100){
			sos={(float)2.41359049e-04,   4.82718098e-04,   2.41359049e-04,1.00000000e+00,  -1.95557824e+00,   9.56543677e-01};
		}
		
		coeffs.push_back(sos);
		f= new Filter(coeffs, 1.0);;
		this->ratio=ratio;
		this->ready=false;
		this->lastSample=0;
		this->sampleCount=ratio-2;
}

//Update feature extractor with new sample

void Downsampler::update (float value)
{
	lastSample=f->filter(value);

	sampleCount+=1;
	//If sample count has reached window length, normalize, set flag as ready and reset sample count
	if(sampleCount>=ratio)
	{
		ready=true;
		sampleCount=0;
	}
  
}

//Return ready state
bool Downsampler::isReady()
{
	return ready;
}


//Return sample
float Downsampler::getSample()
{
	ready=false;
	return lastSample;
}

