#ifndef QUARTIC_POLYNOMIAL_H
#define QUARTIC_POLYNOMIAL_H

#include <Eigen/Dense>
#include <cmath>

namespace artery {

class QuarticPolynomial
{
public:

	/*
     * コンストラクタ
     *
     * @param xs  [double] 始点位置
     * @param vxs [double] 始点速度
     * @param axs [double] 始点加速度
     * @param vxe [double] 終点速度
     * @param axe [double] 終点加速度
     * @param time [double] 始点から終点までの時間
     */
    QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double time);

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

private:
	double a0, a1, a2, a3, a4; // 多項式の係数
};

} // namespace artery

#endif // QUARTIC_POLYNOMIAL_H
