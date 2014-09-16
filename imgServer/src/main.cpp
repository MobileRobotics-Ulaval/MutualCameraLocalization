#include "Camera.h"
#include "LedsFinder.h"

int main(int argc, char* argv[]){
	int threshold = 100;
	float shutter =2.f;
	int brightness = 50;
	int exposure = 25;
	float gain = 0.1f;


	img_server::LedsFinder ledsFinder(5005, threshold, shutter, brightness, exposure, gain);
	//ledsFinder.takeRawPicture(1,threshold);
	ledsFinder.startServerLoop();
	return 0;
}
