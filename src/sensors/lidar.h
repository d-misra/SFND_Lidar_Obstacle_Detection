#ifndef LIDAR_H
#define LIDAR_H

#include "../render/render.h"
#include <ctime>
#include <chrono>
#include <random>
#include <utility>

constexpr double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062; // ... approximately.
constexpr double pi2 = 2.0 * pi;
constexpr double deg2Rad = pi / 180.0;

struct Ray {

    Vect3 origin;
    double resolution;
    Vect3 direction;
    Vect3 castPosition;
    double castDistance;

    std::mt19937 gen{(std::random_device{})()};

    // parameters:
    // setOrigin: the starting position from where the ray is cast
    // horizontalAngle: the angle of direction the ray travels on the xy plane
    // verticalAngle: the angle of direction between xy plane and ray
    //                 for example 0 radians is along xy plane and pi/2 radians is stright up
    // resoultion: the magnitude of the ray's step, used for ray casting, the smaller the more accurate but the more expensive

    Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution)
            : origin(setOrigin), resolution(setResolution),
              direction(resolution * cos(verticalAngle) * cos(horizontalAngle),
                        resolution * cos(verticalAngle) * sin(horizontalAngle), resolution * sin(verticalAngle)),
              castPosition(origin), castDistance(0) {}

    void rayCast(const std::vector<Car> &cars, double minDistance, double maxDistance,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double slopeAngle, double sderr) {
        // reset ray
        castPosition = origin;
        castDistance = 0;

        bool collision = false;

        while (!collision && castDistance < maxDistance) {

            castPosition = castPosition + direction;
            castDistance += resolution;

            // check if there is any collisions with ground slope
            collision = (castPosition.z <= castPosition.x * tan(slopeAngle));

            // check if there is any collisions with cars
            if (!collision && castDistance < maxDistance) {
                for (Car car : cars) {
                    collision |= car.checkCollision(castPosition);
                    if (collision)
                        break;
                }
            }
        }

        std::normal_distribution<> distanceNoise {0, sderr};

        if ((castDistance >= minDistance) && (castDistance <= maxDistance)) {
            const auto px = castPosition.x + distanceNoise(gen);
            const auto py = castPosition.y + distanceNoise(gen);
            const auto pz = castPosition.z + distanceNoise(gen);
            cloud->points.emplace_back(px, py, pz);
        }
    }

};

struct Lidar {

    std::vector<Ray> rays;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<Car> cars;
    Vect3 position;
    double groundSlope;
    double minDistance;
    double maxDistance;
    double resolution;
    double sderr;

    Lidar(std::vector<Car> setCars, double setGroundSlope)
            : cloud(new pcl::PointCloud<pcl::PointXYZ>()), position(0, 0, 2.6) {

        minDistance = 5;    // we'd like to ignore points on the roof of the ego car
        maxDistance = 50;
        resolution = 0.2;
        sderr = 0.1;        // error standard deviation (in m)
        cars = std::move(setCars);
        groundSlope = setGroundSlope;

        // It is generally wished for to have big field of view (in order to capture more of the environment)
        // and a high number of layers. The lower the number of layers, the more laser rays will spread out.
        // Since the distance between rays increases with the distance from the sensor, eventually even large
        // objects will fall "between" two rays and become invisible to the sensor.
        const int numLayers = 8;

        // the steepest vertical angle
        const auto steepestAngle = -30.0 * deg2Rad;
        const auto angleRange = 26.0 * deg2Rad;

        // The smaller the increments, the more fine-grained our measurements will be horizontally.
        // The same logic as above applies: As the rays spread out, objects between them become "invisible".
        const auto horizontalAngleInc = pi / 64;

        const auto angleIncrement = angleRange / numLayers;
        for (double angleVertical = steepestAngle; angleVertical < steepestAngle + angleRange; angleVertical += angleIncrement) {
            for (double angle = 0; angle <= pi2; angle += horizontalAngleInc) {
                Ray ray {position, angle, angleVertical, resolution};
                rays.push_back(ray);
            }
        }
    }

    // pcl uses boost smart pointers for cloud pointer so we don't have to worry about manually freeing the memory
    ~Lidar() = default;

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan() {
        cloud->points.clear();
        auto startTime = std::chrono::steady_clock::now();
        for (auto& ray : rays) {
            ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope, sderr);
        }
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        cout << "ray casting took " << elapsedTime.count() << " milliseconds" << endl;
        cloud->width = cloud->points.size();
        cloud->height = 1; // one dimensional unorganized point cloud dataset
        return cloud;
    }

};

#endif
