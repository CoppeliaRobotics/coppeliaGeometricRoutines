#pragma once

#include "conf.h"
#include <vector>
#include "3Vector.h"
#include "4X4Matrix.h"
#include "obbStruct.h"
#include "pcNode.h"

class COcStruct;

class CPcStruct
{
    friend class COcStruct;

public:
    CPcStruct();
    CPcStruct(float cellS,int cellPts,const float* points,size_t pointCnt,const unsigned char* rgbData,bool rgbForEachPt,float proximityTol);
    virtual ~CPcStruct();

    CPcStruct* copyYourself() const;
    void scaleYourself(float f);
    unsigned char* serialize(int& dataSize) const;
    void deserialize(const unsigned char* data);

    size_t countCellsWithContent() const;
    void getPointsPosAndRgb_all(std::vector<float>& pointsPosAndRgb) const;
    void getPointsPosAndRgb_subset(std::vector<float>& pointsPosAndRgb,float prop) const;
    void getOctreeCorners(std::vector<float>& points) const;

    const float* getPoints(const C4X4Matrix& pcM,unsigned long long int pcCaching,size_t* ptCnt,C4X4Matrix& transf) const;

    void add_pts(const float* points,size_t pointCnt,const unsigned char* rgbData,bool dataForEachPt,float proximityTol);

    bool delete_pts(const float* points,size_t pointCnt,float proximityTol,int* count);
    bool delete_octree(const C4X4Matrix& pcM,const COcStruct* ocStruct,const C4X4Matrix& ocM,int* count);

    bool intersect_pts(const float* points,size_t pointCnt,float proximityTol);

    bool getDistance_pt(const C4X4Matrix& pcM,const C3Vector& point,float& dist,C3Vector* pcMinDistSegAbsPt,C3Vector* ptMinDistSegAbsPt,unsigned long long int* pcCaching) const;
    bool getDistance_seg(const C4X4Matrix& pcM,const C3Vector& segMiddle,const C3Vector& segHs,float& dist,C3Vector* pcMinDistSegAbsPt,C3Vector* segMinDistSegAbsPt,unsigned long long int* pcCaching) const;
    bool getDistance_tri(const C4X4Matrix& pcM,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* pcMinDistSegAbsPt,C3Vector* triMinDistSegAbsPt,unsigned long long int* pcCaching) const;
    bool getDistance_shape(const CObbStruct* obbStruct,const C4X4Matrix& pcM,const C4X4Matrix& shapeM,float& dist,C3Vector* pcMinDistSegAbsPt,C3Vector* shapeMinDistSegAbsPt,unsigned long long int* pcCaching,int* obbCaching) const;
    bool getDistance_ptcloud(const CPcStruct* pc2Struct,const C4X4Matrix& pc1M,const C4X4Matrix& pc2M,float& dist,C3Vector* pc1MinDistSegAbsPt,C3Vector* pc2MinDistSegAbsPt,unsigned long long int* pc1Caching,unsigned long long int* pc2Caching) const;

    bool getSensorDistance(const C4X4Matrix& pcM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,bool fast,float& dist,C3Vector* detectPt) const;

private:
    void _create(float cellS,int cellPts,const float* points,size_t pointCnt,const unsigned char* rgbData,bool rgbForEachPt,float distanceTolerance);
    void _extendPointCloudOctreeIfNeeded(const float* points,size_t pointCnt);

    float cellSize;
    int maxPtCnt;
    float boxSize;
    C3Vector boxPos;
    CPcNode* pcNode;
};
