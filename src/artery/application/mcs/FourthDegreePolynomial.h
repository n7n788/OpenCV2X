#ifndef FOURTH_DEGREE_POLYNOMIAL_H
#define FOURTH_DEGREE_POLYNOMIAL_H

#include "Polynomial.h"
#include <Eigen/Dense>
#include <cmath>

namespace artery {

class FourthDegreePolynomial : public Polynomial
{
public:

	/*
     * コンストラクタ
     *
     * @param xInit  [double] 始点位置
     * @param vInit [double] 始点位置の1次微分
     * @param aInit [double] 始点位置の2次微分
     * @param vFinal [double] 終点位置の1次微分
     * @param aFinal [double] 終点位置の2次微分
     * @param douration [double] 始点と終点までの時間
     */
    FourthDegreePolynomial(double xInit, double vInit, double aInit, double vFinal, double aFinal, double duration);

    /*
     * 任意の時刻tにおけるを計算
     * @param t [double] 時刻t
     * @return double 位置
     */
    double calc_x(double t) const;

    /*
     * 任意の時刻tにおける速度を計算
     * @param t [double] 時刻t
     * @return double 速度
     */
    double calc_v(double t) const;

    /*
     * 任意の時刻tにおける加速度を計算
     * @param t [double] 時刻t
     * @return double 加速度
     */
    double calc_a(double t) const;

    /*
     * 任意の時刻tにおける躍度を計算
     * @param t [double] 時刻t
     * @return double 躍度
     */
    double calc_j(double t) const;

private:
	double a0, a1, a2, a3, a4; // 多項式の係数
};

} // namespace artery

#endif // FOURTH_DEGREE_POLYNOMIAL_H
