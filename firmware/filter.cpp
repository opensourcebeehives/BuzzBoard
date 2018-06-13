#include "filter.h"


//Constructor
Filter::Filter (vector<vector<float> > coeffs, float g){
	    this->g=g;
		this->coeffs=coeffs;
		this->size=coeffs.size();
		this->z.resize(size);

		//Set z to 0
		for (int i=0; i<size; i++)
		{
			z[i].resize(2);
			std::fill(this->z[i].begin(), this->z[i].end(), 0);
		}
}

	
//Filter function based on sequential 2nd order section filtering (Direct Form II)
//coeffs structure:
//b01 b11 b21 1 a11 a21
//b02 b12 b22 1 a12 a22
//...
float Filter::filter (float x)
{
  float y=g*x;

  float v=0;

  //for each 2nd order section
  for (int f=0; f<size; f++){

	  //a coefficients

	  y=y-coeffs[f][4]*z[f][0];
	  y=y-coeffs[f][5]*z[f][1];

	  v=y;

	  y=y*coeffs[f][0];

	  //b coefficients
	  y=y+coeffs[f][1]*z[f][0];
	  y=y+coeffs[f][2]*z[f][1];

	  //update buffer
	  z[f][1]=z[f][0];
	  z[f][0]=v;

  }

  return y;
}
