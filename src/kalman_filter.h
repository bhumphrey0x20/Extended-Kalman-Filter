#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include <math.h>

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

	Eigen::VectorXd hj_; 

	Eigen::MatrixXd Hj_; 

  // measurement covariance matrices
	Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;

	// Identity Matrix
	Eigen::MatrixXd I; 

  /**   * Constructor    */
  KalmanFilter();

  /** * Destructor   */
  virtual ~KalmanFilter();

	// Update state Transition function with new delta t
	void Update_F(float dt); 

	// Update Process Noise with new delta t
	void Update_Q(float dt);

	// calc new h(x')-> hj to polar coordinates given current new state vectors
	void update_hj();

	// Calculates Jacobian Matrix from current state vector
	void CalculateJacobian();


  /**
   * Prediction: Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);


	// converts Radar measurements from polar to Cartesian Coordinatess
	void update_x_polar_to_Cart(const Eigen::VectorXd z_in);
};


#endif /* KALMAN_FILTER_H_ */


