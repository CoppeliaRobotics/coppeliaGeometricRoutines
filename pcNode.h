#pragma once

#include "conf.h"
#include <vector>
#include <simMath/3Vector.h>
#include <simMath/4X4Matrix.h>
#include "obbStruct.h"

class COcNode;

class CPcNode
{
    friend class COcNode;
public:
    CPcNode();
    CPcNode(double boxS,const C3Vector& boxCenter,double cellS,int cellPts,const std::vector<double>& points,std::vector<size_t>& ptsOriginalIndices,std::vector<bool>& ptsInvalidityIndicators,const std::vector<unsigned char>& rgbData,bool rgbForEachPt);
    virtual ~CPcNode();

    CPcNode* copyYourself() const;
    void scaleYourself(double f);
    void serialize(std::vector<unsigned char>& data) const;
    void deserialize(const unsigned char* data,int& pos);
    void serializeOld(std::vector<unsigned char>& data) const;
    void deserializeOld(const unsigned char* data,int& pos);

    size_t countCellsWithContent() const;
    void getPointsPosAndRgb_all(double boxS,const C3Vector& boxCenter,std::vector<double>& data) const;
    void getPointsPosAndRgb_subset(double boxS,const C3Vector& boxCenter,double prop,std::vector<double>& data) const;
    void getOctreeCorners(double boxS,const C3Vector& boxCenter,std::vector<double>& data) const;

    const double* getPoints(double boxS,const C3Vector& boxCenter,unsigned long long int pcCaching,size_t* ptCnt,C3Vector& totalTransl) const;

    void add_pts(double boxS,const C3Vector& boxCenter,double cellS,int cellPts,const std::vector<double>& points,std::vector<size_t>& ptsOriginalIndices,std::vector<bool>& ptsInvalidityIndicators,const std::vector<unsigned char>& rgbData,bool rgbForEachPt);

    bool delete_pts(double boxS,const C3Vector& boxCenter,const std::vector<double>& points,double proximityTol,int* count);
    bool delete_octree(double pcBoxS,const C3Vector& pcBoxCenter,const C4X4Matrix& pcM,double ocBoxS,const C3Vector& ocBoxCenter,const COcNode* ocNode,const C4X4Matrix& ocM,int* count);

    bool intersect_pts(double boxS,const C3Vector& boxCenter,const std::vector<double>& points,double proximityTol);

    void flagDuplicates(double boxS,const C3Vector& boxCenter,const std::vector<double>& points,const std::vector<size_t>& ptsOriginalIndices,std::vector<bool>& duplicateIndicators,double proximityTol) const;

    bool getDistance_pt(double boxS,const C3Vector& boxCenter,const C3Vector& point,double& dist,C3Vector* pcMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const;
    bool getDistance_seg(double boxS,const C3Vector& boxCenter,const C3Vector& segMiddle,const C3Vector& segHs,double& dist,C3Vector* pcMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const;
    bool getDistance_tri(double boxS,const C3Vector& boxCenter,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* pcMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const;
    bool getDistance_shape(double boxS,const C3Vector& boxCenter,const C4X4Matrix& pcM,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,double& dist,C3Vector* pcMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos,int* obbCaching) const;
    bool getDistance_ptcloud(double box1Size,const C3Vector& box1Center,const C4X4Matrix& pc1M,const CPcNode* pc2Node,double box2Size,const C3Vector& box2Center,const C4X4Matrix& pc2M,double& dist,C3Vector* pc1MinDistSegPt,C3Vector* pc2MinDistSegPt,unsigned long long int* pc1Caching,unsigned long long int pc1CachePos,unsigned long long int* pc2Caching,unsigned long long int pc2CachePos) const;

    bool getSensorDistance(double boxS,const C3Vector& boxCenter,const C4X4Matrix& pcM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,bool fast,double& dist,C3Vector* detectPt) const;

    CPcNode** pcNodes;
    std::vector<double> pts;
    std::vector<unsigned char> rgbs;
};
