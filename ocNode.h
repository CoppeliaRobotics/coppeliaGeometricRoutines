#pragma once

#include "conf.h"
#include <vector>
#include "3Vector.h"
#include "pcNode.h"

class COcNode
{
public:
    COcNode();
    COcNode(float boxS,const C3Vector& boxCenter,float cellS,const std::vector<float>& points,const std::vector<unsigned char>& rgbData,const std::vector<unsigned int>& usrData,bool dataForEachPt);
    virtual ~COcNode();

    COcNode* copyYourself() const;
    void serialize(std::vector<unsigned char>& data) const;
    void deserialize(const unsigned char* data,int& pos);

    bool getCell(const C3Vector& boxCenter,float boxSize,unsigned long long int ocCaching,C3Vector& totalTranslation,unsigned int* usrData) const;

    void getVoxelsPosAndRgb(std::vector<float>& voxelsPosAndRgb,float pBoxSize,const C3Vector& boxCenter,std::vector<unsigned int>* usrData=nullptr) const;
    void getVoxelsCorners(std::vector<float>& points,float pBoxSize,const C3Vector& boxCenter) const;
    void getOctreeCorners(std::vector<float>& points,float pBoxSize,const C3Vector& boxCenter) const;

    bool deleteVoxels_pts(float boxS,const C3Vector& boxCenter,const std::vector<float>& points);
    bool deleteVoxels_shape(const C4X4Matrix& ocM,float boxS,const C3Vector& boxCenter,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM);
    bool deleteVoxels_octree(const C4X4Matrix& oc1M,float box1S,const C3Vector& box1Center,const COcNode* oc2Node,const C4X4Matrix& oc2M,float box2S,const C3Vector& box2Center);

    void add_pts(float cellS,float boxS,const C3Vector& boxCenter,const std::vector<float>& points,const std::vector<unsigned char>& rgbData,const std::vector<unsigned int>& usrData,bool dataForEachPt);
    bool add_shape(const C4X4Matrix& ocM,float cellS,float boxS,const C3Vector& boxCenter,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,const unsigned char* rgbData,unsigned int usrData);
    bool add_octree(const C4X4Matrix& oc1M,float cell1S,float box1S,const C3Vector& box1Center,const COcNode* oc2Node,const C4X4Matrix& oc2M,float box2S,const C3Vector& box2Center,const unsigned char* rgbData,unsigned int usrData);

    bool doCollide_pt(const C3Vector& point,float boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_pts(const std::vector<float>& points,const C3Vector& boxPosRelToParent,float boxS) const;
    bool doCollide_seg(const C3Vector& segMiddle,const C3Vector& segHs,float boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,float boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_shape(const C4X4Matrix& ocM,float cellS,float boxS,const C3Vector& boxCenter,unsigned long long int ocCacheValueHere,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,unsigned long long int* ocCaching,int* meshCaching) const;
    bool doCollide_octree(const C4X4Matrix& oc1M,float cell1S,float box1S,const C3Vector& box1Center,unsigned long long int oc1CacheValueHere,const COcNode* oc2Node,const C4X4Matrix& oc2M,float box2S,const C3Vector& box2Center,unsigned long long int oc2CacheValueHere,unsigned long long* oc1Caching,unsigned long long* oc2Caching) const;
    bool doCollide_ptcloud(const C4X4Matrix& ocM,float ocCellS,float ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CPcNode* pcNode,const C4X4Matrix& pcM,float pcBoxS,const C3Vector& pcBoxCenter,unsigned long long int pcCacheValueHere,unsigned long long* ocCaching,unsigned long long* pcCaching) const;

    bool getDistance_pt(const C3Vector& point,float boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,float& dist,C3Vector* minDistSegPt,unsigned long long int* caching) const;
    bool getDistance_seg(const C3Vector& segMiddle,const C3Vector& segHs,float boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,float& dist,C3Vector* ocMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* caching) const;
    bool getDistance_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,float boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,float& dist,C3Vector* ocMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* caching) const;
    bool getDistance_shape(const C4X4Matrix& ocM,float ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,float& dist,C3Vector* ocMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* ocCaching,int* obbCaching) const;
    bool getDistance_octree(const C4X4Matrix& oc1M,float ocBox1S,const C3Vector& ocBox1Center,unsigned long long int oc1CacheValueHere,const COcNode* oc2Node,const C4X4Matrix& oc2M,float ocBox2S,const C3Vector& ocBox2Center,unsigned long long int oc2CacheValueHere,float& dist,C3Vector* oc1MinDistSegPt,C3Vector* oc2MinDistSegPt,unsigned long long int* oc1Caching,unsigned long long int* oc2Caching) const;
    bool getDistance_ptcloud(const C4X4Matrix& ocM,float ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CPcNode* pcNode,const C4X4Matrix& pcM,const C3Vector& pcBoxCenter,float pcBoxS,unsigned long long int pcCacheValueHere,float& dist,C3Vector* ocMinDistSegPt,C3Vector* pcMinDistSegPt,unsigned long long int* ocCaching,unsigned long long int* pcCaching) const;

    bool getSensorDistance(const C4X4Matrix& ocM,float boxS,const C3Vector& boxCenter,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,float cosAngle,bool frontDetection,bool backDetection,bool fast,float& dist,C3Vector* detectPt,C3Vector* triN) const;
    bool getRaySensorDistance(const C4X4Matrix& ocM,float boxS,const C3Vector& boxCenter,const C3Vector& raySegP,const C3Vector& raySegL,float forbiddenDist,float cosAngle,bool frontDetection,bool backDetection,bool fast,float& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched) const;

    COcNode** ocNodes;
    bool empty;
    unsigned char rgb[3];
    unsigned int userData;
};
