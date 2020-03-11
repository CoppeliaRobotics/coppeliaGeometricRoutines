#pragma once

#include "conf.h"
#include <vector>
#include "3Vector.h"
#include "4X4Matrix.h"
#include "obbStruct.h"

class COcNode;

class CPcNode
{
    friend class COcNode;
public:
    CPcNode();
    CPcNode(simReal boxS,const C3Vector& boxCenter,simReal cellS,int cellPts,const std::vector<simReal>& points,std::vector<size_t>& ptsOriginalIndices,std::vector<bool>& ptsInvalidityIndicators,const std::vector<unsigned char>& rgbData,bool rgbForEachPt);
    virtual ~CPcNode();

    CPcNode* copyYourself() const;
    void scaleYourself(simReal f);
    void serialize(std::vector<unsigned char>& data) const;
    void deserialize(const unsigned char* data,int& pos);

    size_t countCellsWithContent() const;
    void getPointsPosAndRgb_all(simReal boxS,const C3Vector& boxCenter,std::vector<simReal>& data) const;
    void getPointsPosAndRgb_subset(simReal boxS,const C3Vector& boxCenter,simReal prop,std::vector<simReal>& data) const;
    void getOctreeCorners(simReal boxS,const C3Vector& boxCenter,std::vector<simReal>& data) const;

    const simReal* getPoints(simReal boxS,const C3Vector& boxCenter,unsigned long long int pcCaching,size_t* ptCnt,C3Vector& totalTransl) const;

    void add_pts(simReal boxS,const C3Vector& boxCenter,simReal cellS,int cellPts,const std::vector<simReal>& points,std::vector<size_t>& ptsOriginalIndices,std::vector<bool>& ptsInvalidityIndicators,const std::vector<unsigned char>& rgbData,bool rgbForEachPt);

    bool delete_pts(simReal boxS,const C3Vector& boxCenter,const std::vector<simReal>& points,simReal proximityTol,int* count);
    bool delete_octree(simReal pcBoxS,const C3Vector& pcBoxCenter,const C4X4Matrix& pcM,simReal ocBoxS,const C3Vector& ocBoxCenter,const COcNode* ocNode,const C4X4Matrix& ocM,int* count);

    bool intersect_pts(simReal boxS,const C3Vector& boxCenter,const std::vector<simReal>& points,simReal proximityTol);

    void flagDuplicates(simReal boxS,const C3Vector& boxCenter,const std::vector<simReal>& points,const std::vector<size_t>& ptsOriginalIndices,std::vector<bool>& duplicateIndicators,simReal proximityTol) const;

    bool getDistance_pt(simReal boxS,const C3Vector& boxCenter,const C3Vector& point,simReal& dist,C3Vector* pcMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const;
    bool getDistance_seg(simReal boxS,const C3Vector& boxCenter,const C3Vector& segMiddle,const C3Vector& segHs,simReal& dist,C3Vector* pcMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const;
    bool getDistance_tri(simReal boxS,const C3Vector& boxCenter,const C3Vector& p,const C3Vector& v,const C3Vector& w,simReal& dist,C3Vector* pcMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const;
    bool getDistance_shape(simReal boxS,const C3Vector& boxCenter,const C4X4Matrix& pcM,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,simReal& dist,C3Vector* pcMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos,int* obbCaching) const;
    bool getDistance_ptcloud(simReal box1Size,const C3Vector& box1Center,const C4X4Matrix& pc1M,const CPcNode* pc2Node,simReal box2Size,const C3Vector& box2Center,const C4X4Matrix& pc2M,simReal& dist,C3Vector* pc1MinDistSegPt,C3Vector* pc2MinDistSegPt,unsigned long long int* pc1Caching,unsigned long long int pc1CachePos,unsigned long long int* pc2Caching,unsigned long long int pc2CachePos) const;

    bool getSensorDistance(simReal boxS,const C3Vector& boxCenter,const C4X4Matrix& pcM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,bool fast,simReal& dist,C3Vector* detectPt) const;

    CPcNode** pcNodes;
    std::vector<simReal> pts;
    std::vector<unsigned char> rgbs;
};
