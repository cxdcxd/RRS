#include <robot_groundstation/DataReduction.h>



DeadbandDataReduction::DeadbandDataReduction(double db) {

	DeadbandParameter = db;
	PreviousSample[0] = 10.0;
	PreviousSample[1] = 10.0;
	PreviousSample[2] = 10.0;
	PreviousSample[3] = 10.0;
	PreviousSample[4] = 10.0;
	PreviousSample[5] = 10.0;
	PreviousSample[6] = 10.0;



}


DeadbandDataReduction::~DeadbandDataReduction() {

	printf("closing deadband class\n");

}

void DeadbandDataReduction::GetCurrentSample(double* Sample) {

	CurrentSample[0] = Sample[0];
	CurrentSample[1] = Sample[1];
	CurrentSample[2] = Sample[2];
	CurrentSample[3] = Sample[3];
	CurrentSample[4] = Sample[4];
	CurrentSample[5] = Sample[5];
	CurrentSample[6] = Sample[6];

}


void DeadbandDataReduction::ApplyZOHDeadband(double* updatedSample, bool* TransmitFlag, double* value_out) {

	// Compute the difference between recently transmitted signal and the current signal
	Difference[0] = fabs(CurrentSample[0] - PreviousSample[0]);
	Difference[1] = fabs(CurrentSample[1] - PreviousSample[1]);
	Difference[2] = fabs(CurrentSample[2] - PreviousSample[2]);
	Difference[3] = fabs(CurrentSample[3] - PreviousSample[3]);
	Difference[4] = fabs(CurrentSample[4] - PreviousSample[4]);
	Difference[5] = fabs(CurrentSample[5] - PreviousSample[5]);
	Difference[6] = fabs(CurrentSample[6] - PreviousSample[6]);

	DifferenceMag = sqrt(pow(Difference[0], 2) + pow(Difference[1], 2) + pow(Difference[2], 2) + pow(Difference[3], 2) + pow(Difference[4], 2)
		+ pow(Difference[5], 2) + pow(Difference[6], 2));

	// Compute the magnitude of the recently transmitted signal
	PreviousMag = sqrt(pow(PreviousSample[0], 2) + pow(PreviousSample[1], 2) + pow(PreviousSample[2], 2)
		+ pow(PreviousSample[3], 2) + pow(PreviousSample[4], 2) + pow(PreviousSample[5], 2) + pow(PreviousSample[6], 2));

	// Check whether the difference is above the perceptual threshold
	*value_out = DifferenceMag / (PreviousMag + 0.00001);

	//std::cout << "Value : " << value << std::endl;

	if (*value_out >= DeadbandParameter)
	{
		// Transmit the current signal
		*TransmitFlag = true;

		updatedSample[0] = CurrentSample[0];
		updatedSample[1] = CurrentSample[1];
		updatedSample[2] = CurrentSample[2];
		updatedSample[3] = CurrentSample[3];
		updatedSample[4] = CurrentSample[4];
		updatedSample[5] = CurrentSample[5];
		updatedSample[6] = CurrentSample[6];

		// Update the previous sample for next iteration
		PreviousSample[0] = CurrentSample[0];
		PreviousSample[1] = CurrentSample[1];
		PreviousSample[2] = CurrentSample[2];
		PreviousSample[3] = CurrentSample[3];
		PreviousSample[4] = CurrentSample[4];
		PreviousSample[5] = CurrentSample[5];
		PreviousSample[6] = CurrentSample[6];
	}
	else
	{
		// Do not transmit the current signal
		*TransmitFlag = false;
		// Replicate the recently received signal (ZOH)
		updatedSample[0] = PreviousSample[0];
		updatedSample[1] = PreviousSample[1];
		updatedSample[2] = PreviousSample[2];
		updatedSample[3] = PreviousSample[3];
		updatedSample[4] = PreviousSample[4];
		updatedSample[5] = PreviousSample[5];
		updatedSample[6] = PreviousSample[6];

	}

}


KalmanFilter::KalmanFilter() {

	NoiseVar[0] = 300.0;
	NoiseVar[1] = 300.0;
	NoiseVar[2] = 300.0;

	ProcNoiseVar[0] = 1.0;
	ProcNoiseVar[1] = 1.0;
	ProcNoiseVar[2] = 1.0;

	PreviousEstimation[0] = 0.0;
	PreviousEstimation[1] = 0.0;
	PreviousEstimation[2] = 0.0;


}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::ApplyKalmanFilter(double* CurrentSample) {


	for (int i = 0; i < 3; i++) {

		Innovation[i] = CurrentSample[i] - PreviousEstimation[i];
		InnovationVar[i] = PredictionErrorVar[i] + NoiseVar[i];
		Gain[i] = PredictionErrorVar[i] / InnovationVar[i];

		CurrentEstimation[i] = PreviousEstimation[i] + Gain[i] * Innovation[i];

		// update values for next iteration
		PredictionErrorVar[i] = PredictionErrorVar[i] + ProcNoiseVar[i] - Gain[i] * PredictionErrorVar[i];
		PreviousEstimation[i] = CurrentEstimation[i];


	}

}
