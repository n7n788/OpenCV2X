#ifndef QUINTIC_POLYNOMIAL_H
#define QUINTIC_POLYNOMIAL_H

#include <Eigen/Dense>
#include <cmath>

namespace artery {

class QuinticPolynomial {

public:
    
	/*
     *  始点と終点から5次元多項式の係数を計算するクラス
     *
     * @param xs  [double] 始点位置
     * @param vxs [double] 始点速度
     * @param axs [double] 始点加速度
     * @param xe  [double] 終点位置
     * @param vxe [double] 終点速度
     * @param axe [double] 終点加速度
     * @param time [double] 始点から終点までの時間
     */
    QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double time);

    /*
     * 任意の時刻tにおける位置を計算
     * @param t [double] 時刻t
     * @return double 位置
     */
    double calc_point(double t) const;
    
    /*
     * 任意の時刻tにおける速度を計算
     * @param t [double] 時刻t
     * @return double 速度
     */
    double calc_first_derivative(double t) const;
    
    /*
     * 任意の時刻tにおける加速度を計算
     * @param t [double] 時刻t
     * @return double 加速度
     */
    double calc_second_derivative(double t) const;
    
    /*
     * 任意の時刻tにおける躍度を計算
     * @param t [double] 時刻t
     * @return double 躍度
     */
    double calc_third_derivative(double t) const;

    // getter
private:

    // 5次元多項式の係数
    double a0, a1, a2, a3, a4, a5;
};

} // namespace artery

#endif // QUINTIC_POLYNOMIAL_H

