#pragma once

#define _SECURE_SCL 0 // Disable bound checks (a bit faster)
#include <algorithm> // std::max, etc.
#include <vector>
#include "mathDefines.h"

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

static void pushData(std::vector<unsigned char>& data,const void* v,size_t vSize)
{
    for (size_t i=0;i<vSize;i++)
        data.push_back((reinterpret_cast<const unsigned char*>(v))[i]);
};
