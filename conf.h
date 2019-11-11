#pragma once

#define _SECURE_SCL 0 // Disable bound checks (a bit faster)
#include <algorithm> // std::max, etc.
#include <vector>

class CVolumePlanes
{
public:
    CVolumePlanes(const float* planes,size_t size)
    {
        s=size;
        if (s==0)
            p=nullptr;
        else
            p=planes;
    }
    const float* ptr() const
    {
        return(p);
    }
    size_t size() const
    {
        return(s);
    }
    float at(size_t pos) const
    {
        return(p[pos]);
    }

private:
    const float* p;
    size_t s;
};

static void pushData(std::vector<unsigned char>& data,const void* v,size_t vSize)
{
    for (size_t i=0;i<vSize;i++)
        data.push_back(((unsigned char*)v)[i]);
};
