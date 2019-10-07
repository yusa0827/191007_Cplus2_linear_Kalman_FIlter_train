// Cplus_linear_kalman_filter_.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include "pch.h"
#include <iostream>
#include <Eigen/Core>

using namespace Eigen;

int main()
{
	Matrix3d A_(3, 3);//係数行列A
	A_ << 1, 1, 0.5,
		0, 1, 1,
		0, 0, 1;
	std::cout << "A_ = \n" << A_ << std::endl;

	Matrix3d A_T(3, 3);//A_の転置行列
	A_T = A_.transpose();
	std::cout << "A_T = \n" << A_T << std::endl;

	Matrix3d B_(3, 3);//係数行列B
	B_ << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	std::cout << "B_ = \n" << B_ << std::endl;

	Matrix3d B_T(3, 3); //B_の転置行列
	B_T = B_.transpose();
	std::cout << "B_T = \n" << B_T << std::endl;

	Vector3d Ct_; //係数行列C  列ベクトル
	Ct_ << 1, 0, 0;
	std::cout << "Ct_  = \n" << Ct_ << std::endl;

	MatrixXd C_ = Ct_.transpose();//係数行列C  行ベクトル
	std::cout << "C_  = \n" << C_ << std::endl;

	Matrix3d Q_(3, 3);//状態ノイズ
	Q_ << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	std::cout << "Q_ = \n" << Q_ << std::endl;

	double R_ = 1.0;//システムノイズ

	Matrix3d I_(3, 3);//単位行列
	I_ << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	Vector3d x_hat_k_k;//状態推定値に初期値を代入
	x_hat_k_k << 1, 1, 1;
	std::cout << "x_hat_k_k = \n" << x_hat_k_k << std::endl;

	Matrix3d P_k_k(3, 3);//誤差共分散行列に初期値を代入
	P_k_k << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	std::cout << "P_k_k = \n" << P_k_k << std::endl;

	Vector3d x_hat_k_k1;//事前状態推定値
	Matrix3d P_k_k1;//事前誤差共分散行列
	Vector3d G_;//カルマンゲイン行列

	double y[10] = { 2,4,6,8,10,12,14,16 };

	/* k=1 */
	// (1_1)
	x_hat_k_k1 = A_ * x_hat_k_k;
	std::cout << "x_hat_k_k1 = \n" << x_hat_k_k1 << std::endl;
	//(1_2)
	P_k_k1 = A_ * P_k_k * A_T + B_ * Q_ * B_T;
	std::cout << "P_k_k1_  = \n" << P_k_k1 << std::endl;
	//(2_1) 分母が1次元スカラーだったので、このように表現
	G_ = P_k_k1 * Ct_ / ((C_ * P_k_k1 * Ct_)(0, 0) + R_);
	std::cout << "G_  = \n" << G_ << std::endl;
	//(2_2) 観測値 y[0] = 2
	x_hat_k_k = x_hat_k_k1 + G_ * (y[0] - (C_ * x_hat_k_k1)(0, 0));
	std::cout << "x_hat_k_k  = \n" << x_hat_k_k << std::endl;
	//(2_3) 
	P_k_k = (I_ - G_ * C_) * P_k_k1;
	std::cout << "P_k_k  = \n" << P_k_k << std::endl;

	/* k=2 */
// (1_1)
	x_hat_k_k1 = A_ * x_hat_k_k;
	std::cout << "x_hat_k_k1 = \n" << x_hat_k_k1 << std::endl;
	//(1_2)
	P_k_k1 = A_ * P_k_k * A_T + B_ * Q_ * B_T;
	std::cout << "P_k_k1_  = \n" << P_k_k1 << std::endl;
	//(2_1) 分母が1次元スカラーだったので、このように表現
	G_ = P_k_k1 * Ct_ / ((C_ * P_k_k1 * Ct_)(0, 0) + R_);
	std::cout << "G_  = \n" << G_ << std::endl;
	//(2_2) 観測値 y[0] = 2
	x_hat_k_k = x_hat_k_k1 + G_ * (y[1] - (C_ * x_hat_k_k1)(0, 0));
	std::cout << "x_hat_k_k  = \n" << x_hat_k_k << std::endl;
	//(2_3) 
	P_k_k = (I_ - G_ * C_) * P_k_k1;
	std::cout << "P_k_k  = \n" << P_k_k << std::endl;


}

/*
C:\eigen-eigen-b3f3d4950030


	Vector2d x_0_average;//状態xの初期値　平均
	Vector2d x0_variarance;//状態xの初期値　分散
	Matrix2d P0;//事前誤差共分散行列　P0 平均ー分散



*/