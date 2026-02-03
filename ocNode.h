#pragma once

#include <conf.h>
#include <vector>
#include <simMath/3Vector.h>
#include <pcNode.h>

class COcStruct;

class COcNode
{
public:
    COcNode();
    COcNode(COcStruct* oct, double boxS,const C3Vector& boxCenter,double cellS,const std::vector<double>& points,const std::vector<unsigned char>& rgbaData,const std::vector<unsigned int>& usrData,bool dataForEachPt);
    virtual ~COcNode();

    COcNode* copyYourself() const;
    void serialize(std::vector<unsigned char>& data) const;
    void deserialize(COcStruct* oct, const unsigned char* data,int& pos);
    void serialize_ver2(std::vector<unsigned char>& data) const;

    bool getCell(const C3Vector& boxCenter,double boxSize,unsigned long long int ocCaching,C3Vector& totalTranslation,unsigned int* usrData) const;

    void resetAllIds(COcStruct* oct);
    void getDisplayVoxelsColorsAndIds(COcStruct* oct, double pBoxSize,const C3Vector& boxCenter, std::vector<float>& thePts, std::vector<unsigned char>& theRgbas, std::vector<unsigned int>& theIds) const;
    void getVoxelsPosAndRgba(std::vector<double>& voxelsPosAndRgb,double pBoxSize,const C3Vector& boxCenter,std::vector<unsigned int>* usrData=nullptr) const;
    void getVoxelsCorners(std::vector<double>& points,double pBoxSize,const C3Vector& boxCenter) const;
    void getOctreeCorners(std::vector<double>& points,double pBoxSize,const C3Vector& boxCenter) const;

    bool deleteVoxels_pts(COcStruct* oct, double boxS,const C3Vector& boxCenter,const std::vector<double>& points);
    bool deleteVoxels_shape(COcStruct* oct, const C4X4Matrix& ocM,double boxS,const C3Vector& boxCenter,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM);
    bool deleteVoxels_octree(COcStruct* oct, const C4X4Matrix& oc1M,double box1S,const C3Vector& box1Center,const COcNode* oc2Node,const C4X4Matrix& oc2M,double box2S,const C3Vector& box2Center);

    void add_pts(COcStruct* oct, double cellS,double boxS,const C3Vector& boxCenter,const std::vector<double>& points,const std::vector<unsigned char>& rgbaData,const std::vector<unsigned int>& usrData,bool dataForEachPt);
    bool add_shape(COcStruct* oct, const C4X4Matrix& ocM,double cellS,double boxS,const C3Vector& boxCenter,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,const unsigned char* rgbData,unsigned int usrData);
    bool add_octree(COcStruct* oct, const C4X4Matrix& oc1M,double cell1S,double box1S,const C3Vector& box1Center,const COcNode* oc2Node,const C4X4Matrix& oc2M,double box2S,const C3Vector& box2Center,const unsigned char* rgbData,unsigned int usrData);

    bool doCollide_pt(const C3Vector& point,double boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_pts(const std::vector<double>& points,const C3Vector& boxPosRelToParent,double boxS) const;
    bool doCollide_seg(const C3Vector& segMiddle,const C3Vector& segHs,double boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,double boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_shape(const C4X4Matrix& ocM,double cellS,double boxS,const C3Vector& boxCenter,unsigned long long int ocCacheValueHere,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,unsigned long long int* ocCaching,int* meshCaching) const;
    bool doCollide_octree(const C4X4Matrix& oc1M,double cell1S,double box1S,const C3Vector& box1Center,unsigned long long int oc1CacheValueHere,const COcNode* oc2Node,const C4X4Matrix& oc2M,double box2S,const C3Vector& box2Center,unsigned long long int oc2CacheValueHere,unsigned long long* oc1Caching,unsigned long long* oc2Caching) const;
    bool doCollide_ptcloud(const C4X4Matrix& ocM,double ocCellS,double ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CPcNode* pcNode,const C4X4Matrix& pcM,double pcBoxS,const C3Vector& pcBoxCenter,unsigned long long int pcCacheValueHere,unsigned long long* ocCaching,unsigned long long* pcCaching) const;

    bool getDistance_pt(const C3Vector& point,double boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,double& dist,C3Vector* minDistSegPt,unsigned long long int* caching) const;
    bool getDistance_seg(const C3Vector& segMiddle,const C3Vector& segHs,double boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,double& dist,C3Vector* ocMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* caching) const;
    bool getDistance_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,double boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,double& dist,C3Vector* ocMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* caching) const;
    bool getDistance_shape(const C4X4Matrix& ocM,double ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,double& dist,C3Vector* ocMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* ocCaching,int* obbCaching) const;
    bool getDistance_octree(const C4X4Matrix& oc1M,double ocBox1S,const C3Vector& ocBox1Center,unsigned long long int oc1CacheValueHere,const COcNode* oc2Node,const C4X4Matrix& oc2M,double ocBox2S,const C3Vector& ocBox2Center,unsigned long long int oc2CacheValueHere,double& dist,C3Vector* oc1MinDistSegPt,C3Vector* oc2MinDistSegPt,unsigned long long int* oc1Caching,unsigned long long int* oc2Caching) const;
    bool getDistance_ptcloud(const C4X4Matrix& ocM,double ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CPcNode* pcNode,const C4X4Matrix& pcM,const C3Vector& pcBoxCenter,double pcBoxS,unsigned long long int pcCacheValueHere,double& dist,C3Vector* ocMinDistSegPt,C3Vector* pcMinDistSegPt,unsigned long long int* ocCaching,unsigned long long int* pcCaching) const;

    bool getSensorDistance(const C4X4Matrix& ocM,double boxS,const C3Vector& boxCenter,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,double cosAngle,bool frontDetection,bool backDetection,bool fast,double& dist,C3Vector* detectPt,C3Vector* triN) const;
    bool getRaySensorDistance(const C4X4Matrix& ocM,double boxS,const C3Vector& boxCenter,const C3Vector& raySegP,const C3Vector& raySegL,double forbiddenDist,double cosAngle,bool frontDetection,bool backDetection,bool fast,double& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched) const;

    COcNode** ocNodes;
    bool empty;
    unsigned char rgba[4];
    unsigned int userData;
    unsigned int id;
};
