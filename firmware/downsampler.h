//#include <iostream>
#include <vector>
#include "filter.h"
using namespace std;

//feature Extractor class. Computes energy for each stored filter

class Downsampler {
	
	//Filter
	Filter* f;
	
    //downsampling ratio
    int ratio;
	
	//updated sample count
    int sampleCount;

	//ready flag
	bool ready;

	//last sample
	float lastSample;

  public:
    Downsampler (int ratio);
	void update(float value);
    bool isReady();
	float getSample();

};

