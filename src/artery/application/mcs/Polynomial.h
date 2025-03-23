#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

namespace artery
{

class Polynomial
{
public:
    virtual double calc_point(double t) const = 0;
    virtual double calc_first_derivative(double t) const = 0;
    virtual double calc_second_derivative(double t) const = 0;
    virtual double calc_third_derivative(double t) const = 0;
};
}
#endif // POLYNOMIAL_H