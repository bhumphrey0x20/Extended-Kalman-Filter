#include <iostream>
#include "kalman_filter.h"

#define PI 3.14159265
#define TWO_PI (2*PI )

int points = 0; 
		// numerical value of PI taken from http://www.cplusplus.com/reference/cmath/atan2/
using namespace std; 
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.


// Constructor
KalmanFilter::KalmanFilter() {
	// Initiallizing State Transition functions, Covar Matrices, etc

	//Process Cov Matrixs
	P_ = MatrixXd(4,4); 
	P_ << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;

	// init F_ as Identity Matrix
	F_= MatrixXd::Identity(4,4); 

	//Process Noise
	Q_ = MatrixXd(4,4);

	// Init H_, hj_ and Hj_ 
	H_ = MatrixXd(2,4); 
	H_ << 1, 0, 0, 0,
				0, 1, 0, 0;

	hj_ = VectorXd(3); 

	Hj_ = MatrixXd(4,4); 

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2,2); 
	R_laser_ << 0.0225, 0,
        			0, 0.0225;

  //measurement covariance matrix - radar
	R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
        			0, 0.0009, 0,
        			0, 0, 0.09;

	I = MatrixXd::Identity(4,4); 
}

KalmanFilter::~KalmanFilter() {}

	void KalmanFilter::Update_F(float dt){
		F_(0,2) = dt; 
		F_(1,3) = dt; 
}

void KalmanFilter::Update_Q(float dt){
	static float noise_ax = 9.0; 
	static float noise_ay = 9.0;
	static float ax_2 = noise_ax * noise_ax; 
	static float ay_2 = noise_ay * noise_ay; 
	float dt_2 = pow(dt,2);
	float dt_3 = pow(dt,3);  
	float dt_4 = pow(dt,4);
	
	float qx_2 = (dt_2 * ax_2);
	float qx_3 = (dt_3 * ax_2)/2; 
	float qx_4 = (dt_4 * ax_2)/4;
	
	float qy_2 = (dt_2 * ay_2);
	float qy_3 = (dt_3 * ay_2)/2; 
	float qy_4 = (dt_4 * ay_2)/4; 
	 
	// Set Q matrix to new dt values
	Q_ << 	qx_4, 0, 		qx_3, 0, 
					0, 		qy_4, 0, 		qy_3, 
					qx_3, 0, 		qx_2, 0,
					0, 		qy_3, 0, 		qy_2; 
}


 // convert x' cartesian to polar coord to get an h(x') => hj_
	void KalmanFilter::update_hj(){

		float px = x_[0]; 
		float py = x_[1]; 
		float vx = x_[2]; 
		float vy = x_[3]; 
		float px2 = px*px; 
		float py2 = py*py; 
		float phi = 0; 

		//calculate rho
		float rho	= px2 + py2;
		rho = sqrt(rho);  

		// calculate the angle phi
		//phi = atan2(py, px); 
		if( (px < 0.000001) && (px > -0.000001) ){ //if px is essentially zero, angle is 90 deg
			rho = PI; 
		}else{
			
			phi = atan2(py, px); //phi); 
		}
				
		// Calculate radial velocity rho dot
		float rho_dot = ((px*vx) + (py*vy))/sqrt(px2+py2); 

	hj_ << rho, phi, rho_dot;  		
	}


void KalmanFilter::CalculateJacobian() {
  /**  TODO:
    * Calculate a Jacobian here.*/

	Hj_ = MatrixXd(3,4);
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];

	// check divide by zero
	if( (px == 0) & (py == 0) ){
		cout << "Jacobian Error: px/py divide by zero " << endl;
		return; 		 // no change: error
	}else{
	// compute Jacobian Matrix
		float denom1 = px*px + py* py; 
		float denom2 = sqrt(denom1); 
		float denom3 = pow(denom1,3) * denom2;  
	
		Hj_ << px/denom2, py/denom2 , 0, 0, 
					-py/denom1, px/denom1, 0, 0,
					py* (vx*py-vy*px)/denom3, px * (vy*px - vx*py)/denom3, px/denom2, py/denom2; 
	}
}
	 

void KalmanFilter::Predict() {
  /**  TODO:
    * predict the state  */

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**  TODO:
    * update the state by using Kalman Filter equations  */
	points++; 

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_laser_;


	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**  TODO:
    * update the state by using Extended Kalman Filter equations  */
	float y_phi; 

	// Compute Hj_
	CalculateJacobian(); 

 
	static float Neg_PI = (-1*PI); 
	//calc error
	VectorXd y = z - hj_; //+ R_radar_; 

	//normalize phi in error to between -PI and PI
	y_phi = y[1];
	if( y_phi > PI){
		while(y_phi > PI){
			y_phi = y_phi - (TWO_PI); 
		}
		y[1] = y_phi;		
	}else if(y_phi < Neg_PI){
		while(y_phi < Neg_PI){
			y_phi = y_phi + (TWO_PI);
		}
		y[1] = y_phi; 
	}

	MatrixXd Hjt_ = Hj_.transpose();
	MatrixXd S = Hj_ * P_ * Hjt_ + R_radar_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Hjt_ * Si; 

	// Calculate New Estimate
	x_ = x_ + (K * y); 
	long x_size = x_.size(); 
	
	P_ = (I - K*Hj_) * P_; 


}



void KalmanFilter::update_x_polar_to_Cart(const Eigen::VectorXd z_in){
		float px, py, vx, vy; 
		float rho = z_in[0];
		float phi = z_in[1];
		float rho_dot = z_in[2];
		float cos_p = cos(phi); 
		float sin_p = sin(phi); 

		px = rho * cos_p; 
		py = rho * sin_p; 
		vx = rho_dot * cos_p;
 		vy = rho_dot * sin_p;  
		
		x_ << px, py, vx, vy; 
}


