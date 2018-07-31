#include "FusionEKF.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
	// R_<objects> moved to kalman_filter class
  //R_laser_ = MatrixXd(2, 2);
  //R_radar_ = MatrixXd(3, 3);


  /**   TODO: * Finish initializing the FusionEKF.
		* Set the process and measurement noises */

}

/*** Destructor. */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

	//get state measurement
	VectorXd z_in = measurement_pack.raw_measurements_; 


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /** TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    // first measurement
    cout << "Init EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;
 
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /** Convert radar from polar to cartesian coordinates and initialize state.
					bh: Matrices are init in constructor
					converting radar polar to cartesian and init x_ state. 7/25/2018
      */

			// if Radar is first measurement convert to Cartesian and set x_
			ekf_.update_x_polar_to_Cart(z_in); 
		
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /** Initialize state.   */

			// set x_ to first measurement
			ekf_.x_ << z_in[0], z_in[1], 0, 0;		

		}

		// set previous timestamp to curr timestamp and return 
		previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /** TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

	float dt = (float)( measurement_pack.timestamp_ - previous_timestamp_)/1000000; 

	ekf_.Update_F(dt); 
	ekf_.Update_Q(dt); 

  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /** TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

		// Calculate hj_ from previous state x_ prediction
		ekf_.update_hj(); 

		ekf_.UpdateEKF(z_in); 

  } else {
    // Laser updates

		ekf_.Update(z_in); 

  }
	
	previous_timestamp_ = measurement_pack.timestamp_; 
  // print the output
  cout << "x_ = \n" << ekf_.x_ << endl;
  cout << "P_ = \n" << ekf_.P_ << endl;
}


	
