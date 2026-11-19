#pragma once

#define _SECURE_SCL 0 // Disable bound checks (a bit faster)
#include <algorithm> // std::max, etc.
#include <cstring> // memcpy
#include <vector>
#include <simMath/mathDefines.h>

class CVolumePlanes
{
public:
    CVolumePlanes(const double* planes,size_t size)
    {
        s=size;
        if (s==0)
            p=nullptr;
        else
            p=planes;
    }
    const double* ptr() const
    {
        return(p);
    }
    size_t size() const
    {
        return(s);
    }
    double at(size_t pos) const
    {
        return(p[pos]);
    }

private:
    const double* p;
    size_t s;
};

// Fixed-capacity vector providing the small subset of std::vector used by the
// node-sorting code. Replaces a per-call heap allocation (plus its push_back
// growth reallocations) with stack storage: same element type, same insertion
// order, same std::sort call, so the resulting order is identical.
template<typename T,size_t N>
class CFixedVector
{
public:
    CFixedVector() : _n(0) {}
    void push_back(const T& v) { _d[_n]=v; _n++; }
    size_t size() const { return(_n); }
    bool empty() const { return(_n==0); }
    T& operator[](size_t i) { return(_d[i]); }
    const T& operator[](size_t i) const { return(_d[i]); }
    T* begin() { return(_d); }
    T* end() { return(_d+_n); }
    const T* begin() const { return(_d); }
    const T* end() const { return(_d+_n); }
private:
    T _d[N];
    size_t _n;
};
static void pushData(std::vector<unsigned char>& data,const void* v,size_t vSize)
{
    for (size_t i=0;i<vSize;i++)
        data.push_back((reinterpret_cast<const unsigned char*>(v))[i]);
};
