#ifndef __RTL_LINE__
#define __RTL_LINE__

#include "Base.hpp"
#include <cmath>
#include <limits>
#include <random>


class Line
{
public:
    Line() : a(0), b(0), c(0) { }

    Line(float _a, float _b, float _c) : a(_a), b(_b), c(_c) { }

    friend std::ostream& operator<<(std::ostream& out, const Line& l) { return out << l.a << ", " << l.b << ", " << l.c; }

    float a, b, c;
};

class LineEstimator : virtual public RTL::Estimator<Line, Point, std::vector<Point> >
{
public:
    virtual Line ComputeModel(const std::vector<Point>& data, const std::set<int>& samples)
    {
        float meanX = 0, meanY = 0, meanXX = 0, meanYY = 0, meanXY = 0;
        for (auto itr = samples.begin(); itr != samples.end(); itr++)
        {
            const Point& p = data[*itr];
            meanX += p.x;
            meanY += p.y;
            meanXX += p.x * p.x;
            meanYY += p.y * p.y;
            meanXY += p.x * p.y;
        }
        size_t M = samples.size();
        meanX /= M;
        meanY /= M;
        meanXX /= M;
        meanYY /= M;
        meanXY /= M;
        float a = meanXX - meanX * meanX;
        float b = meanXY - meanX * meanY;
        float d = meanYY - meanY * meanY;

        Line line;
        if (fabs(b) > 2.2204460492503131e-16)
        {
            // Calculate the first eigen vector of A = [a, b; b, d]
            // Ref. http://www.math.harvard.edu/archive/21b_fall_04/exhibits/2dmatrices/index.html
            float T2 = (a + d) / 2;
            float lambda = T2 - sqrt(T2 * T2 - (a * d - b * b));
            float v1 = lambda - d, v2 = b;
            float norm = sqrt(v1 * v1 + v2 * v2);
            line.a = v1 / norm;
            line.b = v2 / norm;
        }
        else
        {
            line.a = 1;
            line.b = 0;
        }
        line.c = -line.a * meanX - line.b * meanY;
        return line;
    }

    virtual float ComputeError(const Line& line, const Point& point)
    {
        return line.a * point.x + line.b * point.y + line.c;
    }
}; // End of 'LineEstimator'

class LineObserver : virtual public RTL::Observer<Line, Point, std::vector<Point> >
{
public:
    LineObserver(Point _max = Point(640, 480), Point _min = Point(0, 0)) : RANGE_MAX(_max), RANGE_MIN(_min) { }

    virtual std::vector<Point> GenerateData(const Line& line, int N, std::vector<int>& inliers, float noise = 0, float ratio = 1)
    {
        std::mt19937 generator;
        std::uniform_real_distribution<float> uniform(0, 1);
        std::normal_distribution<float> normal(0, 1);

        std::vector<Point> data;
        if (fabs(line.b) > fabs(line.a))
        {
            for (int i = 0; i < N; i++)
            {
                Point point;
                point.x = (RANGE_MAX.x - RANGE_MIN.x) * uniform(generator) + RANGE_MIN.x;
                float vote = uniform(generator);
                if (vote > ratio)
                {
                    // Generate an outlier
                    point.y = (RANGE_MAX.y - RANGE_MIN.y) * uniform(generator) + RANGE_MIN.y;
                }
                else
                {
                    // Generate an inlier
                    point.y = (line.a * point.x + line.c) / -line.b;
                    point.x += noise * normal(generator);
                    point.y += noise * normal(generator);
                    inliers.push_back(i);
                }
                data.push_back(point);
            }
        }
        else
        {
            for (int i = 0; i < N; i++)
            {
                Point point;
                point.y = (RANGE_MAX.y - RANGE_MIN.y) * uniform(generator) + RANGE_MIN.y;
                float vote = uniform(generator);
                if (vote > ratio)
                {
                    // Generate an outlier
                    point.x = (RANGE_MAX.x - RANGE_MIN.x) * uniform(generator) + RANGE_MIN.x;
                }
                else
                {
                    // Generate an inlier
                    point.x = (line.b * point.y + line.c) / -line.a;
                    point.x += noise * normal(generator);
                    point.y += noise * normal(generator);
                    inliers.push_back(i);
                }
                data.push_back(point);
            }
        }
        return data;
    }

    const Point RANGE_MIN;

    const Point RANGE_MAX;
};

#endif // End of '__RTL_LINE__'
