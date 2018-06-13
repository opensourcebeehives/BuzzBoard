/******************************************************************************
Photon analog input in DMA mode

ADCDMA.h
ADCDMA Library Header File
Modified by Peter Naylor: Jan 5, 2018

derived from: Rickkas7 @ Particle.io
https://github.com/rickkas7/photonAudio

This file prototypes the ADCDMA class, implemented in ADCDMA.cpp. 

Development environment specifics:
ParticleCLI
Particle Photon

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef ADCDMA_h
#define ADCDMA_h


#include "Particle.h"
#include "adc_hal.h"
#include "gpio_hal.h"
#include "pinmap_hal.h"
#include "pinmap_impl.h"


// ADCDMA - Class to use Photon ADC in DMA Mode
class ADCDMA {
public:
	ADCDMA(int pin, uint16_t *buf, size_t bufSize);
	virtual ~ADCDMA();

	void start(size_t freqHZ);
	void stop();

private:
	int pin;
	uint16_t *buf;
	size_t bufSize;
};

#endif