#ifndef __RTL_BASE__
#define __RTL_BASE__

#include <set>
#include <vector>
#include <iostream>

namespace RTL
{

template <class Model, class Datum, class Data>
class Estimator
{
public:
    virtual Model ComputeModel(const Data& data, const std::set<int>& samples) = 0;

    virtual float ComputeError(const Model& model, const Datum& datum) = 0;
};

template <class Model, class Datum, class Data>
class Observer
{
public:
    virtual Data GenerateData(const Model& model, int N, std::vector<int>& inliers, float noise = 0, float ratio = 1) = 0;
};

} // End of 'RTL'

#endif // End of '__RTL_BASE__'
