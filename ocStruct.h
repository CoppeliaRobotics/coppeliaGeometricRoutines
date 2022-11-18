#pragma once

#include "conf.h"
#include <vector>
#include "3Vector.h"
#include "obbStruct.h"
#include "ocNode.h"

class CPcStruct;

class COcStruct
{
    friend class CPcStruct;
public:
    COcStruct();
    COcStruct(simReal cellS,const simReal* points,size_t pointCnt,const unsigned char* rgbData,const unsigned int* usrData,bool dataForEachPt);
    COcStruct(const C4X4Matrix& ocM,simReal cellS,const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const unsigned char* rgbData,unsigned int usrData);
    COcStruct(const C4X4Matrix& oc1M,simReal cell1S,const COcStruct* oc2Struct,const C4X4Matrix& oc2M,const unsigned char* rgbData,unsigned int usrData);
    virtual ~COcStruct();

    COcStruct* copyYourself() const;
    void scaleYourself(simReal f);
    unsigned char* serialize(int& dataSize) const;
    bool deserialize(const unsigned char* data);
    unsigned char* serializeOld(int& dataSize) const;
    bool deserializeOld(const unsigned char* data);

    void getVoxelsPosAndRgb(std::vector<simReal>& voxelsPosAndRgb,std::vector<unsigned int>* userData=nullptr) const;
    void getVoxelsCorners(std::vector<simReal>& points) const;
    void getOctreeCorners(std::vector<simReal>& points) const;

    C3Vector getBoxPos() const;
    bool getCell(const C4X4Matrix& ocM,const unsigned long long int ocCaching,simReal& cellS,C4X4Matrix& cellPos,unsigned int* usrData) const;

    bool deleteVoxels_pts(const simReal* points,size_t pointCnt);
    bool deleteVoxels_shape(const C4X4Matrix& ocM,const CObbStruct* obbStruct,const C4X4Matrix& shapeM);
    bool deleteVoxels_octree(const C4X4Matrix& oc1M,const COcStruct* oc2Struct,const C4X4Matrix& oc2M);

    void add_pts(const simReal* points,size_t pointCnt,const unsigned char* rgbData,const unsigned int* usrData,bool dataForEachPt);
    void add_shape(const C4X4Matrix& ocM,const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const unsigned char* rgbData,unsigned int usrData);
    void add_octree(const C4X4Matrix& oc1M,const COcStruct* oc2Struct,const C4X4Matrix& oc2M,const unsigned char* rgbData,unsigned int usrData);

    bool doCollide_pt(const C3Vector& point,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_pts(const std::vector<simReal>& points) const;
    bool doCollide_seg(const C3Vector& segMiddle,const C3Vector& segHs,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,unsigned int* usrData,unsigned long long int* ocCaching) const;
    bool doCollide_shape(const C4X4Matrix& ocM,const CObbStruct* obbStruct,const C4X4Matrix& shapeM,unsigned long long int* ocCaching,int* meshCaching) const;
    bool doCollide_octree(const C4X4Matrix& oc1M,const COcStruct* oc2Struct,const C4X4Matrix& oc2M,unsigned long long int* oc1Caching,unsigned long long int* oc2Caching) const;
    bool doCollide_ptcloud(const C4X4Matrix& ocM,const CPcStruct* pcStruct,const C4X4Matrix& pcM,unsigned long long int* ocCaching,unsigned long long int* pcCaching) const;

    bool getDistance_pt(const C3Vector& point,simReal& dist,C3Vector* minDistSegPt1,unsigned long long int* ocCaching) const;
    bool getDistance_seg(const C3Vector& segMiddle,const C3Vector& segHs,simReal& dist,C3Vector* ocMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* ocCaching) const;
    bool getDistance_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,simReal& dist,C3Vector* ocMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* ocCaching) const;
    bool getDistance_shape(const C4X4Matrix& ocM,const CObbStruct* obbStruct,const C4X4Matrix& shapeM,simReal& dist,C3Vector* ocMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* ocCaching,int* obbCaching) const;
    bool getDistance_octree(const C4X4Matrix& oc1M,const COcStruct* oc2Struct,const C4X4Matrix& oc2M,simReal& dist,C3Vector* oc1MinDistSegPt,C3Vector* oc2MinDistSegPt,unsigned long long int* oc1Caching,unsigned long long int* oc2Caching) const;
    bool getDistance_ptcloud(const C4X4Matrix& ocM,const CPcStruct* pcStruct,const C4X4Matrix& pcM,simReal& dist,C3Vector* ocMinDistSegPt,C3Vector* pcMinDistSegPt,unsigned long long int* ocCaching,unsigned long long int* pcCaching) const;

    bool getSensorDistance(const C4X4Matrix& ocM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,simReal cosAngle,bool frontDetection,bool backDetection,bool fast,simReal& dist,C3Vector* detectPt,C3Vector* triN) const;
    bool getRaySensorDistance(const C4X4Matrix& ocM,const C3Vector& raySegP,const C3Vector& raySegL,simReal forbiddenDist,simReal cosAngle,bool frontDetection,bool backDetection,bool fast,simReal& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched) const;

private:
    void _create(simReal cellS,const simReal* points,size_t pointCnt,const unsigned char* rgbData,const unsigned int* usrData,bool dataForEachPt);
    void _extendOctreeIfNeeded(const simReal* points,size_t pointCnt);

    simReal cellSize;
    simReal boxSize;
    C3Vector boxPos;
    COcNode* ocNode;
};
