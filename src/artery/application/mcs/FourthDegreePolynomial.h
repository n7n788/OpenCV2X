#ifndef FOURTH_DEGREE_POLYNOMIAL_H
#define FOURTH_DEGREE_POLYNOMIAL_H

#include "Polynomial.h"
#include <Eigen/Dense>
#include <cmath>

namespace artery {

/**
 * @brief 4 次多項式（位置・速度・加速度を境界条件に持つ）
 * 
 * 始点の位置・速度・加速度, 終点の速度・加速度を与え，
 * 区間長 @p duration で滑らかな 4 次曲線を生成する。
*/
class FourthDegreePolynomial : public Polynomial
{
public:

	/**
     * @param xInit  [double] 始点位置
     * @param vInit  [double] 始点速度
     * @param aInit  [double] 始点加速度
     * @param vFinal [double] 終点速度
     * @param aFinal [double] 終点加速度
     * @param douration [double] 0 ≤ t ≤ duration の区間長
     */
    FourthDegreePolynomial(double xInit, double vInit, double aInit,
                           double vFinal, double aFinal,
                           double duration);

    /// 位置 x(t)
    double calc_x(double t) const;
    /// 速度 ẋ(t)
    double calc_v(double t) const;
    /// 加速度 ẍ(t)
    double calc_a(double t) const;
    /// 躍度 x⃛(t)
    double calc_j(double t) const;

private:
    // 多項式係数 x(t) = Σ_{i=0}^{4} a_i t^i
	double a0{}, a1{}, a2{}, a3{}, a4{}; // 多項式の係数
};

} // namespace artery

#endif // FOURTH_DEGREE_POLYNOMIAL_H
