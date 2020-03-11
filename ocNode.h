#pragma once

#include "conf.h"
#include <vector>
#include "3Vector.h"
#include "pcNode.h"

class COcNode
{
public:
    COcNode();
    COcNode(simReal boxS,const C3Vector& boxCenter,simReal cellS,const std::vector<simReal>& points,const std::vector<unsigned char>& rgbData,const std::vector<unsigned int>& usrData,bool dataForEachPt);
    virtual ~COcNode();

    COcNode* copyYourself() const;
    void serialize(std::vector<unsigned char>& data) const;
    void deserialize(const unsigned char* data,int& pos);

    bool getCell(const C3Vector& boxCenter,simReal boxSize,unsigned long long int ocCaching,C3Vector& totalTranslation,unsigned int* usrData) const;

    void getVoxelsPosAndRgb(std::vector<simReal>& voxelsPosAndRgb,simReal pBoxSize,const C3Vector& boxCenter,std::vector<unsigned int>* usrData=nullptr) const;
    void getVoxelsCorners(std::vector<simReal>& points,simReal pBoxSize,const C3Vector& boxCenter) const;
    void getOctreeCorners(std::vector<simReal>& points,simReal pBoxSize,const C3Vector& boxCenter) const;

    bool deleteVoxels_pts(simReal boxS,const C3Vector& boxCenter,const std::vector<simReal>& points);
    bool deleteVoxels_shape(const C4X4Matrix& ocM,simReal boxS,const C3Vector& boxCenter,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM);
    bool deleteVoxels_octree(const C4X4Matrix& oc1M,simReal box1S,const C3Vector& box1Center,const COcNode* oc2Node,const C4X4Matrix& oc2M,simReal box2S,const C3Vector& box2Center);

    void add_pts(simReal cellS,simReal boxS,const C3Vector& boxCenter,const std::vector<simReal>& points,const std::vector<unsigned char>& rgbData,const std::vector<unsigned int>& usrData,bool dataForEachPt);
    bool add_shape(const C4X4Matrix& ocM,simReal cellS,simReal boxS,const C3Vector& boxCenter,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,const unsigned char* rgbData,unsigned int usrData);
    bool add_octree(const C4X4Matrix& oc1M,simReal cell1S,simReal box1S,const C3Vector& box1Center,const COcNode* oc2Node,const C4X4Matrix& oc2M,simReal box2S,const C3Vector& box2Center,const unsigned char* rgbData,unsigned int usrData);

    bool doCollide_pt(const C3Vector& point,simReal boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_pts(const std::vector<simReal>& points,const C3Vector& boxPosRelToParent,simReal boxS) const;
    bool doCollide_seg(const C3Vector& segMiddle,const C3Vector& segHs,simReal boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,simReal boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_shape(const C4X4Matrix& ocM,simReal cellS,simReal boxS,const C3Vector& boxCenter,unsigned long long int ocCacheValueHere,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,unsigned long long int* ocCaching,int* meshCaching) const;
    bool doCollide_octree(const C4X4Matrix& oc1M,simReal cell1S,simReal box1S,const C3Vector& box1Center,unsigned long long int oc1CacheValueHere,const COcNode* oc2Node,const C4X4Matrix& oc2M,simReal box2S,const C3Vector& box2Center,unsigned long long int oc2CacheValueHere,unsigned long long* oc1Caching,unsigned long long* oc2Caching) const;
    bool doCollide_ptcloud(const C4X4Matrix& ocM,simReal ocCellS,simReal ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CPcNode* pcNode,const C4X4Matrix& pcM,simReal pcBoxS,const C3Vector& pcBoxCenter,unsigned long long int pcCacheValueHere,unsigned long long* ocCaching,unsigned long long* pcCaching) const;

    bool getDistance_pt(const C3Vector& point,simReal boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,simReal& dist,C3Vector* minDistSegPt,unsigned long long int* caching) const;
    bool getDistance_seg(const C3Vector& segMiddle,const C3Vector& segHs,simReal boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,simReal& dist,C3Vector* ocMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* caching) const;
    bool getDistance_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,simReal boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,simReal& dist,C3Vector* ocMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* caching) const;
    bool getDistance_shape(const C4X4Matrix& ocM,simReal ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,simReal& dist,C3Vector* ocMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* ocCaching,int* obbCaching) const;
    bool getDistance_octree(const C4X4Matrix& oc1M,simReal ocBox1S,const C3Vector& ocBox1Center,unsigned long long int oc1CacheValueHere,const COcNode* oc2Node,const C4X4Matrix& oc2M,simReal ocBox2S,const C3Vector& ocBox2Center,unsigned long long int oc2CacheValueHere,simReal& dist,C3Vector* oc1MinDistSegPt,C3Vector* oc2MinDistSegPt,unsigned long long int* oc1Caching,unsigned long long int* oc2Caching) const;
    bool getDistance_ptcloud(const C4X4Matrix& ocM,simReal ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CPcNode* pcNode,const C4X4Matrix& pcM,const C3Vector& pcBoxCenter,simReal pcBoxS,unsigned long long int pcCacheValueHere,simReal& dist,C3Vector* ocMinDistSegPt,C3Vector* pcMinDistSegPt,unsigned long long int* ocCaching,unsigned long long int* pcCaching) const;

    bool getSensorDistance(const C4X4Matrix& ocM,simReal boxS,const C3Vector& boxCenter,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,simReal cosAngle,bool frontDetection,bool backDetection,bool fast,simReal& dist,C3Vector* detectPt,C3Vector* triN) const;
    bool getRaySensorDistance(const C4X4Matrix& ocM,simReal boxS,const C3Vector& boxCenter,const C3Vector& raySegP,const C3Vector& raySegL,simReal forbiddenDist,simReal cosAngle,bool frontDetection,bool backDetection,bool fast,simReal& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched) const;

    COcNode** ocNodes;
    bool empty;
    unsigned char rgb[3];
    unsigned int userData;
};
