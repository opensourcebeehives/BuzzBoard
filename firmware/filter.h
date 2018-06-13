//#include <iostream>
#include <vector>
using namespace std;

 #ifndef _FILTER_
 #define _FILTER_


//Simple bandpass filter

class Filter
{
	
	//Number of second order sections
	int size;
	

    //Filter coefficients, organized in second order sections:
	//b01 b11 b21 1 a11 a21
	//b02 b12 b22 1 a12 a22
    vector<vector<float> > coeffs;

    //Filter gain
	float g;

    //Buffers
    vector<vector<float> > z;

  public:
    Filter (vector<vector<float> > coeffs, float g);
    float filter (float);

};

 #endif
