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
    CPcNode(float boxS,const C3Vector& boxCenter,float cellS,int cellPts,const std::vector<float>& points,std::vector<int>& ptsOriginalIndices,std::vector<bool>& ptsInvalidityIndicators,const std::vector<unsigned char>& rgbData,bool rgbForEachPt);
    virtual ~CPcNode();

    CPcNode* copyYourself() const;
    void scaleYourself(float f);
    void serialize(std::vector<unsigned char>& data) const;
    void deserialize(const unsigned char* data,int& pos);

    size_t countCellsWithContent() const;
    void getPointsPosAndRgb_all(float boxS,const C3Vector& boxCenter,std::vector<float>& data) const;
    void getPointsPosAndRgb_subset(float boxS,const C3Vector& boxCenter,float prop,std::vector<float>& data) const;
    void getOctreeCorners(float boxS,const C3Vector& boxCenter,std::vector<float>& data) const;

    const float* getPoints(float boxS,const C3Vector& boxCenter,unsigned long long int pcCaching,size_t* ptCnt,C3Vector& totalTransl) const;

    void add_pts(float boxS,const C3Vector& boxCenter,float cellS,int cellPts,const std::vector<float>& points,std::vector<int>& ptsOriginalIndices,std::vector<bool>& ptsInvalidityIndicators,const std::vector<unsigned char>& rgbData,bool rgbForEachPt);

    bool delete_pts(float boxS,const C3Vector& boxCenter,const std::vector<float>& points,float proximityTol,int* count);
    bool delete_octree(float pcBoxS,const C3Vector& pcBoxCenter,const C4X4Matrix& pcM,float ocBoxS,const C3Vector& ocBoxCenter,const COcNode* ocNode,const C4X4Matrix& ocM,int* count);

    bool intersect_pts(float boxS,const C3Vector& boxCenter,const std::vector<float>& points,float proximityTol);

    void flagDuplicates(float boxS,const C3Vector& boxCenter,const std::vector<float>& points,const std::vector<int>& ptsOriginalIndices,std::vector<bool>& duplicateIndicators,float proximityTol) const;

    bool getDistance_pt(float boxS,const C3Vector& boxCenter,const C3Vector& point,float& dist,C3Vector* pcMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const;
    bool getDistance_seg(float boxS,const C3Vector& boxCenter,const C3Vector& segMiddle,const C3Vector& segHs,float& dist,C3Vector* pcMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const;
    bool getDistance_tri(float boxS,const C3Vector& boxCenter,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* pcMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const;
    bool getDistance_shape(float boxS,const C3Vector& boxCenter,const C4X4Matrix& pcM,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,float& dist,C3Vector* pcMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos,int* obbCaching) const;
    bool getDistance_ptcloud(float box1Size,const C3Vector& box1Center,const C4X4Matrix& pc1M,const CPcNode* pc2Node,float box2Size,const C3Vector& box2Center,const C4X4Matrix& pc2M,float& dist,C3Vector* pc1MinDistSegPt,C3Vector* pc2MinDistSegPt,unsigned long long int* pc1Caching,unsigned long long int pc1CachePos,unsigned long long int* pc2Caching,unsigned long long int pc2CachePos) const;

    bool getSensorDistance(float boxS,const C3Vector& boxCenter,const C4X4Matrix& pcM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,bool fast,float& dist,C3Vector* detectPt) const;

    CPcNode** pcNodes;
    std::vector<float> pts;
    std::vector<unsigned char> rgbs;
};
