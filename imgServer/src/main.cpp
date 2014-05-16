#include "LedsFinder.h"

int main(){
	int threshold = 100;
	float shutter =2.f;
	int brightness = 50;
	int exposure = 25;
	float gain = 0.1f;

	    
	printf("[SERVER] Start image server...\n");
	LedsFinder ledsFinder(5005, threshold, shutter, brightness, exposure, gain);
	//ledsFinder.takeRawPicture(100,threshold);
	ledsFinder.startProcessingLoop();
	return 0;
}
