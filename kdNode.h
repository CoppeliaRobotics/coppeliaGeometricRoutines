#pragma once

#include "conf.h"
#include <vector>
#include "3Vector.h"

struct SKdPt {
    C3Vector pt;
    bool ignorePt;
    unsigned char rgb[3];
};

class CKdNode
{
public:
    CKdNode();
    CKdNode(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,double proximityTol,int axis);

    virtual ~CKdNode();

    static CKdNode* buildKdTree(const double* pts,size_t ptCnt,const unsigned char* rgbData,bool rgbForEachPt,double proximityTol);

    void getPts(std::vector<double>& pts,std::vector<unsigned char>& rgbs);

    void _populateNode(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,double proximityTol,int axis);
    void _disableClosePts(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,double proximityTol,int axis);

    CKdNode* kdNodes[2];
    C3Vector pt;
    bool ptIsValid;
    unsigned char rgb[3];
};
