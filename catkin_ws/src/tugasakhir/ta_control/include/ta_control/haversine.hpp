// REFERENCE
// Gade, Kenneth (2010). "A Non-singular Horizontal Position Representation".
// Journal of Navigation. 63 (3): 395–417. doi:10.1017/S0373463309990415. ISSN 0373-4633

#pragma once

#include <math.h>
#include <sensor_msgs/NavSatFix.h>

namespace hav
{
    struct north_east {
        double north;
        double east;
    };

    double distance(double lat1, double lon1, double lat2, double lon2)
    {
        // Convert to radians
        double phi_1     = M_PI / 180.0 * lat1;
        double lambda_1  = M_PI / 180.0 * lon1;
        double phi_2     = M_PI / 180.0 * lat2;
        double lambda_2  = M_PI / 180.0 * lon2;

        // Earth Radius (KM)
        const double R = 6378.137;

        // Distance
        double d = 2 * R * asin(sqrt(
            pow(sin((phi_2 - phi_1) / 2.0), 2)

            +

            cos(phi_1) * cos(phi_2) * pow(sin((lambda_2 - lambda_1) / 2.0), 2)
        ));

        // Distance (Meter)
        return d * 1000.0f;
    }

    double distance(sensor_msgs::NavSatFix src, sensor_msgs::NavSatFix dst) {
        return distance(src.latitude, src.longitude, dst.latitude, dst.longitude);
    }

    north_east ne_distance(double lat1, double lon1, double lat2, double lon2)
    {
        // Convert to radians
        double phi_1     = M_PI / 180.0 * lat1;
        double lambda_1  = M_PI / 180.0 * lon1;
        double phi_2     = M_PI / 180.0 * lat2;
        double lambda_2  = M_PI / 180.0 * lon2;

        // Earth Radius
        const double R = 6378.137;

        north_east dist;

        // North Distance (Latitude 1 = Latitude 2)
        dist.north = 2 * R * asin(cos(phi_1) * sin((lambda_2 - lambda_1) / 2.0)) * 1000.0f;

        // East Distance (Longitude 1 = Longitude 2)
        dist.east = 2 * R * asin(sin((phi_2 - phi_1) / 2.0)) * 1000.0f;

        return dist;
    }

    north_east ne_distance(sensor_msgs::NavSatFix src, sensor_msgs::NavSatFix dst) {
        return ne_distance(src.latitude, src.longitude, dst.latitude, dst.longitude);
    }

    double distance(double north, double east)
    {
        return sqrt(pow(north, 2) + pow(east, 2));
    }

    double distance(north_east ne)
    {
        return distance(ne.north, ne.east);
    }

} // namespace hav
