#include <iostream>
#include <math.h>

// Perceptaul haptic data reduction class
class DeadbandDataReduction
{

public:
	DeadbandDataReduction(double db); // initializes a deadband class for a 7 DoF haptic signal
	~DeadbandDataReduction(); // kills the deadband class


	double DeadbandParameter; // variable holding the deadband parameter 0<DB<1

	void GetCurrentSample(double* Sample); // copies a new sample for deadband computation
	void ApplyZOHDeadband(double* updatedSample, bool* TransmitFlag, double* value_out); // performs the deadband data reduction, decides the transmission state


private:

	double CurrentSample[7]; // variable holding the current haptic sample
	double PreviousSample[7]; // variable holding the previosly transmitted haptic sample
	double Difference[7];
	double DifferenceMag;
	double PreviousMag;

};

class KalmanFilter {

public:
	KalmanFilter();
	~KalmanFilter();

	double CurrentEstimation[3];
	double PreviousEstimation[3];
	double PredictionErrorVar[3];

	void ApplyKalmanFilter(double* CurrentSample);

private:

	double Innovation[3]; // I
	double InnovationVar[3]; // S
	double NoiseVar[3]; // R
	double Gain[3]; // K
	double ProcNoiseVar[3]; // Q

};
