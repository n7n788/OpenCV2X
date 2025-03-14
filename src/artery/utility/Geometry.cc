#include "artery/utility/Geometry.h"
#include <boost/geometry/algorithms/distance.hpp>

namespace artery
{

// // 追加
// Angle Angle::from_radian(double rad)
// {
//     return Angle { rad };
// }

// Angle Angle::from_degree(double deg)
// {
//     boost::units::quantity<boost::units::degree::plane_angle> tmp = deg * boost::units::degree::degree;
//     return Angle::value_type { tmp };
// }


double Angle::radian() const
{
    return value / boost::units::si::radian;
}

double Angle::degree() const
{
    boost::units::quantity<boost::units::degree::plane_angle> tmp { value };
    return tmp / boost::units::degree::degree;
}


// 追加
// Angle::value_type Angle::getTrueNorth() const
// {
//     static const auto pi = boost::math::constants::pi<double>();
//     using boost::units::si::radians;

//     value_type heading = value;
//     // change rotation ccw -> cw
//     heading *= -1.0;
//     // rotate zero from east to north
//     heading += 0.5 * pi * radians;
//     // normalize angle to [0; 2*pi[
//     static const auto circle = 2.0 * pi * radians;
//     while (heading >= circle || heading < 0.0 * radians) {
//         heading -= boost::units::copysign(circle, heading);
//     }

//     assert(heading >= 0.0 * radians);
//     assert(heading < circle);
//     return heading;
// }

Position::value_type distance(const Position& a, const Position& b)
{
    auto d = boost::geometry::distance(a, b);
    return d * boost::units::si::meter;
}

bool operator==(const Position& a, const Position& b)
{
    return !(a != b);
}

bool operator!=(const Position& a, const Position& b)
{
    return a.x != b.x || a.y != b.y;
}

} // namespace artery
