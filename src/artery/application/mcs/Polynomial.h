#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

namespace artery
{

class Polynomial
{
public:
    virtual double calc_x(double t) const = 0;
    virtual double calc_v(double t) const = 0;
    virtual double calc_a(double t) const = 0;
    virtual double calc_j(double t) const = 0;
};
}
#endif // POLYNOMIAL_H