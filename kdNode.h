#pragma once

#include <conf.h>
#include <vector>
#include <simMath/3Vector.h>

struct SKdPt {
    C3Vector pt;
    bool ignorePt;
    unsigned char rgba[4];
};

class CKdNode
{
public:
    CKdNode();
    CKdNode(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,double proximityTol,int axis);

    virtual ~CKdNode();

    static CKdNode* buildKdTree(const double* pts,size_t ptCnt,const unsigned char* rgbaData,bool rgbForEachPt,double proximityTol);

    void getPts(std::vector<double>& pts,std::vector<unsigned char>& rgbas);

    void _populateNode(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,double proximityTol,int axis);
    void _disableClosePts(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,double proximityTol,int axis);

    CKdNode* kdNodes[2];
    C3Vector pt;
    bool ptIsValid;
    unsigned char rgba[4];
};
