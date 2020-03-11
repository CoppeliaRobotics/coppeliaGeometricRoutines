#pragma once

#include "conf.h"
#include <vector>
#include "3Vector.h"
#include "4X4Matrix.h"

class CObbStruct;

class CObbNode
{
public:
    CObbNode();
    CObbNode(const std::vector<simReal>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& triIndices,size_t maxTriCnt);
    virtual ~CObbNode();

    CObbNode* copyYourself() const;
    void scaleYourself(simReal f);

    void serialize(std::vector<unsigned char>& data) const;
    void deserialize(const unsigned char* data,int& pos);

    bool checkCollision_obb(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const CObbNode* obb2,const CObbStruct* obbStruct2,const C4X4Matrix& shape2M,std::vector<simReal>* intersections,int* cachingTri1,int* cachingTri2) const;
    bool checkCollision_tri(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,std::vector<simReal>* intersections,int* cachingTri1,int* cachingTri2) const;
    bool checkCollision_segp(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,std::vector<simReal>* intersections,int* cachingTri1) const;
    bool checkCollision_segp_withTriInfo(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,C3Vector& intersection,int& triangleIndex) const;

    bool checkDistance_obb(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const CObbNode* obb2,const CObbStruct* obbStruct2,const C4X4Matrix& shape2M,simReal& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1,int* cachingTri2) const;
    bool checkDistance_tri(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,simReal& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1,int* cachingTri2) const;
    bool checkDistance_segp(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,simReal& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1) const;
    bool checkDistance_pt(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const C3Vector& pt,simReal& dist,C3Vector* minDistSegPt,int* cachingTri) const;

    bool checkSensorDistance_obb(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,simReal cosAngle,bool frontDetection,bool backDetection,bool fast,simReal& dist,C3Vector* detectPt,C3Vector* triN);
    bool checkRaySensorDistance_obb(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const C3Vector& raySegP,const C3Vector& raySegL,simReal cosAngle,bool frontDetection,bool backDetection,simReal forbiddenDist,bool fast,simReal& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched);

    CObbNode** obbNodes;
    C3Vector boxHs;
    C4X4Matrix boxM;
    std::vector<int>* leafTris;

private:
    void splitTriangles(const std::vector<simReal>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& triIndices,std::vector<int>& triIndicesGroup1,std::vector<int>& triIndicesGroup2);
    static C4X4Matrix getNaturalFrame(const std::vector<simReal>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& trianglesIndices,C3Vector& boxHs);
};
