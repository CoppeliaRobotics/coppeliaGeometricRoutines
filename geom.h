#ifndef GEOM_H
#define GEOM_H

#include "conf.h"
#include "obbStruct.h"
#include "pcStruct.h"
#include "ocStruct.h"

// Mesh creation/destruction/manipulation/info
CObbStruct* geom_createMesh(const double* vertices,int verticesSize,const int* indices,int indicesSize,const C7Vector* meshOrigin=nullptr,double triangleEdgeMaxLength=double(0.3),int maxTrianglesInBoundingBox=8);
CObbStruct* geom_copyMesh(const CObbStruct* meshObbStruct);
CObbStruct* geom_getMeshFromSerializationData(const unsigned char* serializationData);
void geom_getMeshSerializationData(const CObbStruct* meshObbStruct,std::vector<unsigned char>& serializationData);
void geom_scaleMesh(CObbStruct* meshObbStruct,double scalingFactor);
void geom_destroyMesh(CObbStruct* meshObbStruct);
double geom_getMeshRootObbVolume(const CObbStruct* meshObbStruct);

// OC tree creation/destruction/manipulation/info
COcStruct* geom_createOctreeFromPoints(const double* points,int pointCnt,const C7Vector* octreeOrigin=nullptr,double cellS=double(0.05),const unsigned char rgbData[3]=nullptr,unsigned int usrData=0);
COcStruct* geom_createOctreeFromColorPoints(const double* points,int pointCnt,const C7Vector* octreeOrigin=nullptr,double cellS=double(0.05),const unsigned char* rgbData=nullptr,const unsigned int* usrData=nullptr);
COcStruct* geom_createOctreeFromMesh(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C7Vector* octreeOrigin=nullptr,double cellS=double(0.05),const unsigned char rgbData[3]=nullptr,unsigned int usrData=0);
COcStruct* geom_createOctreeFromOctree(const COcStruct* otherOctreeStruct,const C7Vector& otherOctreeTransformation,const C7Vector* newOctreeOrigin=nullptr,double newOctreeCellS=double(0.05),const unsigned char rgbData[3]=nullptr,unsigned int usrData=0);
COcStruct* geom_copyOctree(const COcStruct* ocStruct);
COcStruct* geom_getOctreeFromSerializationData(const unsigned char* serializationData);
void geom_getOctreeSerializationData(const COcStruct* ocStruct,std::vector<unsigned char>& serializationData);
COcStruct* geom_getOctreeFromSerializationData_float(const unsigned char* serializationData);
void geom_getOctreeSerializationData_float(const COcStruct* ocStruct,std::vector<unsigned char>& serializationData);
void geom_scaleOctree(COcStruct* ocStruct,double scalingFactor);
void geom_destroyOctree(COcStruct* ocStruct);
void geom_getOctreeVoxelData(const COcStruct* ocStruct,std::vector<double>& voxelData,std::vector<unsigned int>* userData=nullptr);
void geom_getOctreeCornersFromOctree(const COcStruct* ocStruct,std::vector<double>& points);

void geom_insertPointsIntoOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const double* points,int pointCnt,const unsigned char rgbData[3]=nullptr,unsigned int usrData=0);
void geom_insertColorPointsIntoOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const double* points,int pointCnt,const unsigned char* rgbData=nullptr,const unsigned int* usrData=nullptr);
void geom_insertMeshIntoOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const CObbStruct* obbStruct,const C7Vector& meshTransformation,const unsigned char rgbData[3]=nullptr,unsigned int usrData=0);
void geom_insertOctreeIntoOctree(COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation,const unsigned char rgbData[3]=nullptr,unsigned int usrData=0);
bool geom_removePointsFromOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const double* points,int pointCnt);
bool geom_removeMeshFromOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const CObbStruct* obbStruct,const C7Vector& meshTransformation);
bool geom_removeOctreeFromOctree(COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation);

// Point cloud creation/destruction/manipulation/info
CPcStruct* geom_createPtcloudFromPoints(const double* points,int pointCnt,const C7Vector* ptcloudOrigin=nullptr,double cellS=double(0.05),int maxPointCnt=20,const unsigned char rgbData[3]=nullptr,double proximityTol=double(0.005));
CPcStruct* geom_createPtcloudFromColorPoints(const double* points,int pointCnt,const C7Vector* ptcloudOrigin=nullptr,double cellS=double(0.05),int maxPointCnt=20,const unsigned char* rgbData=nullptr,double proximityTol=double(0.005));
CPcStruct* geom_copyPtcloud(const CPcStruct* pcStruct);
CPcStruct* geom_getPtcloudFromSerializationData(const unsigned char* serializationData);
void geom_getPtcloudSerializationData(const CPcStruct* pcStruct,std::vector<unsigned char>& serializationData);
CPcStruct* geom_getPtcloudFromSerializationData_float(const unsigned char* serializationData);
void geom_getPtcloudSerializationData_float(const CPcStruct* pcStruct,std::vector<unsigned char>& serializationData);
void geom_scalePtcloud(CPcStruct* pcStruct,double scalingFactor);
void geom_destroyPtcloud(CPcStruct* pcStruct);
bool geom_getDisplayPtcloudData(CPcStruct* pcStruct, bool forceFresh, std::vector<float>& points, std::vector<unsigned char>& cols, std::vector<unsigned int>& ids);
void geom_getPtcloudPoints(const CPcStruct* pcStruct,std::vector<double>& pointData,double prop=1.0);
void geom_getPtcloudOctreeCorners(const CPcStruct* pcStruct,std::vector<double>& points);
int geom_getPtcloudNonEmptyCellCount(const CPcStruct* pcStruct);

void geom_insertPointsIntoPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const double* points,int pointCnt,const unsigned char rgbData[3]=nullptr,double proximityTol=double(0.001));
void geom_insertColorPointsIntoPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const double* points,int pointCnt,const unsigned char* rgbData=nullptr,double proximityTol=double(0.001));
bool geom_removePointsFromPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const double* points,int pointCnt,double proximityTol,int* countRemoved=nullptr);
bool geom_removeOctreeFromPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const COcStruct* ocStruct,const C7Vector& octreeTransformation,int* countRemoved=nullptr);
bool geom_intersectPointsWithPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const double* points,int pointCnt,double proximityTol=double(0.001));


// Collision detection
bool geom_getMeshMeshCollision(const CObbStruct* mesh1ObbStruct,const C7Vector& mesh1Transformation,const CObbStruct* mesh2ObbStruct,const C7Vector& mesh2Transformation,std::vector<double>* intersections=nullptr,int* mesh1Caching=nullptr,int* mesh2Caching=nullptr);
bool geom_getMeshOctreeCollision(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const COcStruct* ocStruct,const C7Vector& octreeTransformation,int* meshCaching=nullptr,unsigned long long int* ocCaching=nullptr);
bool geom_getMeshTriangleCollision(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,std::vector<double>* intersections=nullptr,int* caching=nullptr);
bool geom_getMeshSegmentCollision(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& segmentExtremity,const C3Vector& segmentVector,std::vector<double>* intersections=nullptr,int* caching=nullptr);
bool geom_getMeshSegmentCollision_withTriInfo(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& segmentExtremity,const C3Vector& segmentVector,C3Vector& intersection,int& triangleIndex);

bool geom_getOctreeOctreeCollision(const COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation,unsigned long long int* oc1Caching=nullptr,unsigned long long int* oc2Caching=nullptr);
bool geom_getOctreePtcloudCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,unsigned long long int* ocCaching=nullptr,unsigned long long int* pcCaching=nullptr);
bool geom_getOctreeTriangleCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,unsigned long long int* caching=nullptr);
bool geom_getOctreeSegmentCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& segmentExtremity,const C3Vector& segmentVector,unsigned long long int* caching=nullptr);
bool geom_getOctreePointsCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const double* points,int pointCount);
bool geom_getOctreePointCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& point,unsigned int* usrData=nullptr,unsigned long long int* caching=nullptr);

bool geom_getBoxBoxCollision(const C7Vector& box1Transformation,const C3Vector& box1HalfSize,const C7Vector& box2Transformation,const C3Vector& box2HalfSize,bool boxesAreSolid);
bool geom_getBoxTriangleCollision(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& p,const C3Vector& v,const C3Vector& w);
bool geom_getBoxSegmentCollision(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& segmentEndPoint,const C3Vector& segmentVector);
bool geom_getBoxPointCollision(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,const C3Vector& point);

bool geom_getTriangleTriangleCollision(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,std::vector<double>* intersections=nullptr);
bool geom_getTriangleSegmentCollision(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,std::vector<double>* intersections=nullptr);

// Distance calculation
bool geom_getMeshMeshDistanceIfSmaller(const CObbStruct* mesh1ObbStruct,const C7Vector& mesh1Transformation,const CObbStruct* mesh2ObbStruct,const C7Vector& mesh2Transformation,double& dist,C3Vector* minDistSegPt1=nullptr,C3Vector* minDistSegPt2=nullptr,int* mesh1Caching=nullptr,int* mesh2Caching=nullptr);
double geom_getMeshMeshDistance(const CObbStruct* mesh1ObbStruct,const C7Vector& mesh1Transformation,const CObbStruct* mesh2ObbStruct,const C7Vector& mesh2Transformation,C3Vector* minDistSegPt1=nullptr,C3Vector* minDistSegPt2=nullptr,int* mesh1Caching=nullptr,int* mesh2Caching=nullptr);
bool geom_getMeshOctreeDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const COcStruct* ocStruct,const C7Vector& octreeTransformation,double& dist,C3Vector* meshMinDistPt=nullptr,C3Vector* ocMinDistPt=nullptr,int* meshCaching=nullptr,unsigned long long int* ocCaching=nullptr);
double geom_getMeshOctreeDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const COcStruct* ocStruct,const C7Vector& octreeTransformation,C3Vector* meshMinDistPt=nullptr,C3Vector* ocMinDistPt=nullptr,int* meshCaching=nullptr,unsigned long long int* ocCaching=nullptr);
bool geom_getMeshPtcloudDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const CPcStruct* pcStruct,const C7Vector& pcTransformation,double& dist,C3Vector* meshMinDistPt=nullptr,C3Vector* pcMinDistPt=nullptr,int* meshCaching=nullptr,unsigned long long int* pcCaching=nullptr);
double geom_getMeshPtcloudDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const CPcStruct* pcStruct,const C7Vector& pcTransformation,C3Vector* meshMinDistPt=nullptr,C3Vector* pcMinDistPt=nullptr,int* meshCaching=nullptr,unsigned long long int* pcCaching=nullptr);
bool geom_getMeshTriangleDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* meshDistSegPt=nullptr,C3Vector* triangleDistSegPt=nullptr,int* caching=nullptr);
double geom_getMeshTriangleDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,C3Vector* meshDistSegPt=nullptr,C3Vector* triangleDistSegPt=nullptr,int* caching=nullptr);
bool geom_getMeshSegmentDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* meshDistSegPt=nullptr,C3Vector* segmentDistSegPt=nullptr,int* caching=nullptr);
double geom_getMeshSegmentDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* meshDistSegPt=nullptr,C3Vector* segmentDistSegPt=nullptr,int* caching=nullptr);
bool geom_getMeshPointDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& point,double& dist,C3Vector* meshMinDistPt=nullptr,int* caching=nullptr);
double geom_getMeshPointDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& point,C3Vector* meshMinDistPt=nullptr,int* caching=nullptr);

bool geom_getOctreeOctreeDistanceIfSmaller(const COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation,double& dist,C3Vector* oc1MinDistPt=nullptr,C3Vector* oc2MinDistPt=nullptr,unsigned long long int* oc1Caching=nullptr,unsigned long long int* oc2Caching=nullptr);
double geom_getOctreeOctreeDistance(const COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation,C3Vector* oc1MinDistPt=nullptr,C3Vector* oc2MinDistPt=nullptr,unsigned long long int* oc1Caching=nullptr,unsigned long long int* oc2Caching=nullptr);
bool geom_getOctreePtcloudDistanceIfSmaller(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const CPcStruct* pcStruct,const C7Vector& pcTransformation,double& dist,C3Vector* ocMinDistPt=nullptr,C3Vector* pcMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr,unsigned long long int* pcCaching=nullptr);
double geom_getOctreePtcloudDistance(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const CPcStruct* pcStruct,const C7Vector& pcTransformation,C3Vector* ocMinDistPt=nullptr,C3Vector* pcMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr,unsigned long long int* pcCaching=nullptr);
bool geom_getOctreeTriangleDistanceIfSmaller(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* ocMinDistPt=nullptr,C3Vector* triMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr);
double geom_getOctreeTriangleDistance(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& p,const C3Vector& v,const C3Vector&w,C3Vector* ocMinDistPt=nullptr,C3Vector* triMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr);
bool geom_getOctreeSegmentDistanceIfSmaller(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* ocMinDistPt=nullptr,C3Vector* segMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr);
double geom_getOctreeSegmentDistance(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* ocMinDistPt=nullptr,C3Vector* segMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr);
bool geom_getOctreePointDistanceIfSmaller(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& point,double& dist,C3Vector* ocMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr);
double geom_getOctreePointDistance(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& point,C3Vector* ocMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr);

bool geom_getPtcloudPtcloudDistanceIfSmaller(const CPcStruct* pc1Struct,const C7Vector& pc1Transformation,const CPcStruct* pc2Struct,const C7Vector& pc2Transformation,double& dist,C3Vector* pc1MinDistPt=nullptr,C3Vector* pc2MinDistPt=nullptr,unsigned long long int* pc1Caching=nullptr,unsigned long long int* pc2Caching=nullptr);
double geom_getPtcloudPtcloudDistance(const CPcStruct* pc1Struct,const C7Vector& pc1Transformation,const CPcStruct* pc2Struct,const C7Vector& pc2Transformation,C3Vector* pc1MinDistPt=nullptr,C3Vector* pc2MinDistPt=nullptr,unsigned long long int* pc1Caching=nullptr,unsigned long long int* pc2Caching=nullptr);
bool geom_getPtcloudTriangleDistanceIfSmaller(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* pcMinDistPt=nullptr,C3Vector* triMinDistPt=nullptr,unsigned long long int* pcCaching=nullptr);
double geom_getPtcloudTriangleDistance(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,C3Vector* pcMinDistPt=nullptr,C3Vector* triMinDistPt=nullptr,unsigned long long int* pcCaching=nullptr);
bool geom_getPtcloudSegmentDistanceIfSmaller(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* pcMinDistPt=nullptr,C3Vector* segMinDistPt=nullptr,unsigned long long int* pcCaching=nullptr);
double geom_getPtcloudSegmentDistance(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* pcMinDistPt=nullptr,C3Vector* segMinDistPt=nullptr,unsigned long long int* pcCaching=nullptr);
bool geom_getPtcloudPointDistanceIfSmaller(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& point,double& dist,C3Vector* pcMinDistPt=nullptr,unsigned long long int* pcCaching=nullptr);
double geom_getPtcloudPointDistance(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& point,C3Vector* pcMinDistPt=nullptr,unsigned long long int* pcCaching=nullptr);

bool geom_getBoxBoxDistanceIfSmaller(const C7Vector& box1Transformation,const C3Vector& box1HalfSize,const C7Vector& box2Transformation,const C3Vector& box2HalfSize,bool boxesAreSolid,double& dist,C3Vector* distSegPt1=nullptr,C3Vector* distSegPt2=nullptr,bool altRoutine=false);
double geom_getBoxBoxDistance(const C7Vector& box1Transformation,const C3Vector& box1HalfSize,const C7Vector& box2Transformation,const C3Vector& box2HalfSize,bool boxesAreSolid,C3Vector* distSegPt1=nullptr,C3Vector* distSegPt2=nullptr,bool altRoutine=false);
double geom_getApproxBoxBoxDistance(const C7Vector& box1Transformation,const C3Vector& box1HalfSize,const C7Vector& box2Transformation,const C3Vector& box2HalfSize);
bool geom_getBoxCubeDistanceIfSmaller(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,const C7Vector& cubeTransformation,double cubeHalfSize,bool boxAndCubeAreSolid,double& dist,C3Vector* distSegPt1=nullptr,C3Vector* distSegPt2=nullptr);
double geom_getBoxCubeDistance(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,const C7Vector& cubeTransformation,double cubeHalfSize,bool boxAndCubeAreSolid,C3Vector* distSegPt1=nullptr,C3Vector* distSegPt2=nullptr);
bool geom_getBoxTriangleDistanceIfSmaller(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* boxDistSegPt=nullptr,C3Vector* triangleDistSegPt=nullptr,bool altRoutine=false);
double geom_getBoxTriangleDistance(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& p,const C3Vector& v,const C3Vector& w,C3Vector* boxDistSegPt=nullptr,C3Vector* triangleDistSegPt=nullptr,bool altRoutine=false);
bool geom_getBoxSegmentDistanceIfSmaller(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* boxDistSegPt=nullptr,C3Vector* segmentSegPt=nullptr,bool altRoutine=false);
double geom_getBoxSegmentDistance(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* boxDistSegPt=nullptr,C3Vector* segmentSegPt=nullptr,bool altRoutine=false);
bool geom_getBoxPointDistanceIfSmaller(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& point,double& dist,C3Vector* boxDistSegPt=nullptr);
double geom_getBoxPointDistance(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& point,C3Vector* boxDistSegPt=nullptr);

bool geom_getTriangleTriangleDistanceIfSmaller(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,double& dist,C3Vector* triangle1DistSegPt=nullptr,C3Vector* triangle2DistSegPt=nullptr);
double geom_getTriangleTriangleDistance(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,C3Vector* triangle1DistSegPt=nullptr,C3Vector* triangle2DistSegPt=nullptr);
bool geom_getTriangleSegmentDistanceIfSmaller(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* triangleDistSegPt=nullptr,C3Vector* segmentDistSegPt=nullptr);
double geom_getTriangleSegmentDistance(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* triangleDistSegPt=nullptr,C3Vector* segmentDistSegPt=nullptr);
bool geom_getTrianglePointDistanceIfSmaller(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& point,double& dist,C3Vector* triangleDistSegPt=nullptr);
double geom_getTrianglePointDistance(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& point,C3Vector* triangleDistSegPt=nullptr);

bool geom_getSegmentSegmentDistanceIfSmaller(const C3Vector& segment1EndPoint,const C3Vector& segment1Vector,const C3Vector& segment2EndPoint,const C3Vector& segment2Vector,double& dist,C3Vector* minDistSegPt1=nullptr,C3Vector* minDistSegPt2=nullptr);
double geom_getSegmentSegmentDistance(const C3Vector& segment1EndPoint,const C3Vector& segment1Vector,const C3Vector& segment2EndPoint,const C3Vector& segment2Vector,C3Vector* minDistSegPt1=nullptr,C3Vector* minDistSegPt2=nullptr);
bool geom_getSegmentPointDistanceIfSmaller(const C3Vector& segmentEndPoint,const C3Vector& segmentVector,const C3Vector& point,double& dist,C3Vector* segmentDistSegPt=nullptr);
double geom_getSegmentPointDistance(const C3Vector& segmentEndPoint,const C3Vector& segmentVector,const C3Vector& point,C3Vector* segmentDistSegPt=nullptr);

// Volume sensor
bool geom_volumeSensorDetectMeshIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const CObbStruct* obbStruct,const C7Vector& meshTransformation,double& dist,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr);
bool geom_volumeSensorDetectMeshIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const C7Vector& sensorTransformation,const CObbStruct* obbStruct,const C7Vector& meshTransformation,double& dist,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr);

bool geom_volumeSensorDetectOctreeIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const COcStruct* ocStruct,const C7Vector& octreeTransformation,double& dist,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr);
bool geom_volumeSensorDetectOctreeIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const C7Vector& sensorTransformation,const COcStruct* ocStruct,const C7Vector& octreeTransformation,double& dist,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr);

bool geom_volumeSensorDetectPtcloudIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,double& dist,bool fast=false,C3Vector* detectPt=nullptr);
bool geom_volumeSensorDetectPtcloudIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const C7Vector& sensorTransformation,const CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,double& dist,bool fast=false,C3Vector* detectPt=nullptr);

bool geom_volumeSensorDetectTriangleIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr);
bool geom_volumeSensorDetectTriangleIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const C7Vector& sensorTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr);

bool geom_volumeSensorDetectSegmentIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,double maxAngle=0.0,C3Vector* detectPt=nullptr);
bool geom_volumeSensorDetectSegmentIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const C7Vector& sensorTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,double maxAngle=0.0,C3Vector* detectPt=nullptr);

// Ray sensor
bool geom_raySensorDetectMeshIfSmaller(const C3Vector& rayStart,const C3Vector& rayVect,const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,double& dist,double forbiddenDist=0.0,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr,bool* forbiddenDistTouched=nullptr);
bool geom_raySensorDetectMeshIfSmaller(const C7Vector& sensorTransformation,double rayOffset,double rayLength,const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,double& dist,double forbiddenDist=0.0,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr,bool* forbiddenDistTouched=nullptr);

bool geom_raySensorDetectOctreeIfSmaller(const C3Vector& rayStart,const C3Vector& rayVect,const COcStruct* ocStruct,const C7Vector& octreeTransformation,double& dist,double forbiddenDist=0.0,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr,bool* forbiddenDistTouched=nullptr);
bool geom_raySensorDetectOctreeIfSmaller(const C7Vector& sensorTransformation,double rayOffset,double rayLength,const COcStruct* ocStruct,const C7Vector& octreeTransformation,double& dist,double forbiddenDist=0.0,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr,bool* forbiddenDistTouched=nullptr);

// Volume-pt test
bool geom_isPointInVolume(const double* planesIn,int planesInSize,const C3Vector& point);
bool geom_isPointInVolume(const double* planesIn,int planesInSize,const C7Vector& sensorTransformation,const C3Vector& point);

#endif
