#include "geom.h"
#include <iostream>
#include <cstdio>
#include "calcUtils.h"

bool geom_getMeshMeshCollision(const CObbStruct* mesh1ObbStruct,const C7Vector& mesh1Transformation,const CObbStruct* mesh2ObbStruct,const C7Vector& mesh2Transformation,std::vector<float>* intersections/*=nullptr*/,int* mesh1Caching/*=nullptr*/,int* mesh2Caching/*=nullptr*/)
{
    bool retVal=false;
    if ( (mesh1Caching!=nullptr)&&(mesh2Caching!=nullptr)&&(intersections==nullptr) )
    {
        if ( (mesh1Caching[0]>=0)&&(mesh1Caching[0]<mesh1ObbStruct->indices.size()/3)&&(mesh2Caching[0]>=0)&&(mesh2Caching[0]<mesh2ObbStruct->indices.size()/3) )
        {
            int tri1Ind[3]={mesh1ObbStruct->indices[3*mesh1Caching[0]+0],mesh1ObbStruct->indices[3*mesh1Caching[0]+1],mesh1ObbStruct->indices[3*mesh1Caching[0]+2]};
            int tri2Ind[3]={mesh2ObbStruct->indices[3*mesh2Caching[0]+0],mesh2ObbStruct->indices[3*mesh2Caching[0]+1],mesh2ObbStruct->indices[3*mesh2Caching[0]+2]};
            C3Vector p11(&mesh1ObbStruct->vertices[3*tri1Ind[0]]);
            C3Vector p12(&mesh1ObbStruct->vertices[3*tri1Ind[1]]);
            C3Vector p13(&mesh1ObbStruct->vertices[3*tri1Ind[2]]);
            C3Vector p21(&mesh2ObbStruct->vertices[3*tri2Ind[0]]);
            C3Vector p22(&mesh2ObbStruct->vertices[3*tri2Ind[1]]);
            C3Vector p23(&mesh2ObbStruct->vertices[3*tri2Ind[2]]);
            p11=mesh1Transformation*p11;
            p12=mesh1Transformation*p12;
            p13=mesh1Transformation*p13;
            p21=mesh2Transformation*p21;
            p22=mesh2Transformation*p22;
            p23=mesh2Transformation*p23;
            retVal=CCalcUtils::doCollide_tri_tri(p11,p12-p11,p13-p11,-1,p21,p22-p21,p23-p21,-1,nullptr,nullptr,nullptr);
        }
    }
    retVal=retVal||mesh1ObbStruct->obb->checkCollision_obb(mesh1ObbStruct,mesh1Transformation.getMatrix(),mesh2ObbStruct->obb,mesh2ObbStruct,mesh2Transformation.getMatrix(),intersections,mesh1Caching,mesh2Caching);
    return(retVal);
}

bool geom_getMeshOctreeCollision(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const COcStruct* ocStruct,const C7Vector& octreeTransformation,int* meshCaching/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/)
{
    bool retVal=false;
    if ( (meshCaching!=nullptr)&&(ocCaching!=nullptr) )
    {
        if ( (meshCaching[0]>=0)&&(meshCaching[0]<meshObbStruct->indices.size()/3) )
        {
            C4X4Matrix cellM;
            float cellS;
            if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)ocCaching[0],cellS,cellM,nullptr))
            {
                C4X4Matrix cellMi(cellM.getInverse());
                int triInd[3]={meshObbStruct->indices[3*meshCaching[0]+0],meshObbStruct->indices[3*meshCaching[0]+1],meshObbStruct->indices[3*meshCaching[0]+2]};
                C3Vector p1(&meshObbStruct->vertices[3*triInd[0]]);
                C3Vector p2(&meshObbStruct->vertices[3*triInd[1]]);
                C3Vector p3(&meshObbStruct->vertices[3*triInd[2]]);
                p1=cellMi*meshTransformation*p1;
                p2=cellMi*meshTransformation*p2;
                p3=cellMi*meshTransformation*p3;
                retVal=CCalcUtils::doCollide_cell_tri(cellS*0.5f,true,p1,p2-p1,p3-p1);
            }
        }
    }
    retVal=retVal||ocStruct->doCollide_shape(octreeTransformation.getMatrix(),meshObbStruct,meshTransformation.getMatrix(),ocCaching,meshCaching);
    return(retVal);
}


bool geom_getMeshTriangleCollision(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,std::vector<float>* intersections/*=nullptr*/,int* caching/*=nullptr*/)
{
    bool retVal=false;
    if ( (caching!=nullptr)&&(intersections==nullptr) )
    {
        if ( (caching[0]>=0)&&(caching[0]<meshObbStruct->indices.size()/3) )
        {
            int triInd[3]={meshObbStruct->indices[3*caching[0]+0],meshObbStruct->indices[3*caching[0]+1],meshObbStruct->indices[3*caching[0]+2]};
            C3Vector p1(&meshObbStruct->vertices[3*triInd[0]]);
            C3Vector p2(&meshObbStruct->vertices[3*triInd[1]]);
            C3Vector p3(&meshObbStruct->vertices[3*triInd[2]]);
            p1=meshTransformation*p1;
            p2=meshTransformation*p2;
            p3=meshTransformation*p3;
            retVal=CCalcUtils::doCollide_tri_tri(p1,p2-p1,p3-p1,-1,p,v,w,-1,nullptr,nullptr,nullptr);
        }
    }
    retVal=retVal||meshObbStruct->obb->checkCollision_tri(meshObbStruct,meshTransformation.getMatrix(),p,v,w,0,intersections,caching,nullptr);
    return(retVal);
}

bool geom_getMeshSegmentCollision(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& segmentExtremity,const C3Vector& segmentVector,std::vector<float>* intersections/*=nullptr*/,int* caching/*=nullptr*/)
{
    bool retVal=false;
    if ( (caching!=nullptr)&&(intersections==nullptr) )
    {
        if ( (caching[0]>=0)&&(caching[0]<meshObbStruct->indices.size()/3) )
        {
            int triInd[3]={meshObbStruct->indices[3*caching[0]+0],meshObbStruct->indices[3*caching[0]+1],meshObbStruct->indices[3*caching[0]+2]};
            C3Vector p1(&meshObbStruct->vertices[3*triInd[0]]);
            C3Vector p2(&meshObbStruct->vertices[3*triInd[1]]);
            C3Vector p3(&meshObbStruct->vertices[3*triInd[2]]);
            p1=meshTransformation*p1;
            p2=meshTransformation*p2;
            p3=meshTransformation*p3;
            retVal=CCalcUtils::doCollide_tri_segp(p1,p2-p1,p3-p1,-1,segmentExtremity,segmentVector,nullptr,nullptr);
        }
    }
    retVal=retVal||meshObbStruct->obb->checkCollision_segp(meshObbStruct,meshTransformation.getMatrix(),segmentExtremity,segmentVector,intersections,caching);
    return(retVal);
}

bool geom_getOctreeOctreeCollision(const COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation,unsigned  long long int* oc1Caching/*=nullptr*/,unsigned  long long int* oc2Caching/*=nullptr*/)
{
    bool retVal=false;
    if ( (oc1Caching!=nullptr)&&(oc2Caching!=nullptr) )
    {
        C4X4Matrix cell1M;
        float cell1S;
        if (oc1Struct->getCell(octree1Transformation.getMatrix(),(const unsigned long long int)oc1Caching[0],cell1S,cell1M,nullptr))
        {
            C3Vector _cell1S(cell1S*0.5f,cell1S*0.5f,cell1S*0.5f);
            C4X4Matrix cell2M;
            float cell2S;
            if (oc2Struct->getCell(octree2Transformation.getMatrix(),(const unsigned long long int)oc2Caching[0],cell2S,cell2M,nullptr))
            {
                C3Vector _cell2S(cell2S*0.5f,cell2S*0.5f,cell2S*0.5f);
                retVal=CCalcUtils::doCollide_box_box(cell1M,_cell1S,cell2M,_cell2S,true);
            }
        }
    }
    retVal=retVal||oc1Struct->doCollide_octree(octree1Transformation.getMatrix(),oc2Struct,octree2Transformation.getMatrix(),oc1Caching,oc2Caching);
    return(retVal);
}

bool geom_getOctreePtcloudCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,unsigned  long long int* ocCaching/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    bool retVal=false;
    if ( (ocCaching!=nullptr)&&(pcCaching!=nullptr) )
    {
        C4X4Matrix cellM;
        float cellS;
        if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)ocCaching[0],cellS,cellM,nullptr))
        {
            C4X4Matrix tr;
            size_t cnt=0;
            const float* pts=pcStruct->getPoints(ptcloudTransformation.getMatrix(),(const unsigned long long int)pcCaching[0],&cnt,tr);
            if ( (pts!=nullptr)&&(cnt>0) )
            {
                C4X4Matrix m(cellM.getInverse()*tr);
                for (size_t i=0;i<cnt;i++)
                {
                    C3Vector pt(pts+3*i);
                    pt*=m;
                    if (CCalcUtils::doCollide_cell_pt(cellS*0.5f,pt))
                    {
                        retVal=true;
                        break;
                    }
                }
            }
        }
    }
    retVal=retVal||ocStruct->doCollide_ptcloud(octreeTransformation.getMatrix(),pcStruct,ptcloudTransformation.getMatrix(),ocCaching,pcCaching);
    return(retVal);
}

bool geom_getOctreeTriangleCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,unsigned long long int* caching/*=nullptr*/)
{
    bool retVal=false;
    if (caching!=nullptr)
    {
        C4X4Matrix cellM;
        float cellS;
        if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)caching[0],cellS,cellM,nullptr))
        {
            C4X4Matrix cellMi(cellM.getInverse());
            C3Vector _p(cellMi*p);
            cellMi.X.clear();
            C3Vector _v(cellMi*v);
            C3Vector _w(cellMi*w);
            retVal=CCalcUtils::doCollide_cell_tri(cellS*0.5f,true,_p,_v,_w);
        }
    }
    if (!retVal)
    {
        C7Vector trInv(octreeTransformation.getInverse());
        C3Vector _p(trInv*p);
        trInv.X.clear();
        C3Vector _v(trInv*v);
        C3Vector _w(trInv*w);
        retVal=ocStruct->doCollide_tri(_p,_v,_w,nullptr,caching);
    }
    return(retVal);
}

bool geom_getOctreeSegmentCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& segmentExtremity,const C3Vector& segmentVector,unsigned long long int* caching/*=nullptr*/)
{
    bool retVal=false;
    if (caching!=nullptr)
    {
        C4X4Matrix cellM;
        float cellS;
        if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)caching[0],cellS,cellM,nullptr))
        {
            C4X4Matrix cellMi(cellM.getInverse());
            C3Vector _segP(cellMi*segmentExtremity);
            cellMi.X.clear();
            C3Vector _segL(cellMi*segmentVector);
            retVal=CCalcUtils::doCollide_cell_segp(cellS*0.5f,true,_segP,_segL);
        }
    }
    if (!retVal)
    {
        C7Vector trInv(octreeTransformation.getInverse());
        C3Vector _segP(trInv*segmentExtremity);
        trInv.X.clear();
        C3Vector _segL(trInv*segmentVector);
        retVal=ocStruct->doCollide_seg(_segP+_segL*0.5f,_segL*0.5f,nullptr,caching);
    }
    return(retVal);
}

bool geom_getOctreePointsCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const float* points,int pointCount)
{
    bool retVal=false;
    C7Vector trInv(octreeTransformation.getInverse());
    std::vector<float> _points;
    _points.resize(pointCount*3);
    for (int i=0;i<pointCount;i++)
    {
        C3Vector p(points+3*i);
        p*=trInv;
        _points[3*i+0]=p(0);
        _points[3*i+1]=p(1);
        _points[3*i+2]=p(2);
    }
    retVal=ocStruct->doCollide_pts(_points);
    return(retVal);
}

bool geom_getOctreePointCollision(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& point,unsigned int* usrData/*=nullptr*/,unsigned long long int* caching/*=nullptr*/)
{
    bool retVal=false;
    if (caching!=nullptr)
    {
        C4X4Matrix cellM;
        float cellS;
        if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)caching[0],cellS,cellM,usrData))
        {
            C4X4Matrix cellMi(cellM.getInverse());
            C3Vector _point(cellMi*point);
            retVal=CCalcUtils::doCollide_cell_pt(cellS*0.5f,_point);
        }
    }
    if (!retVal)
    {
        C7Vector trInv(octreeTransformation.getInverse());
        C3Vector _point(trInv*point);
        retVal=ocStruct->doCollide_pt(_point,usrData,caching);
    }
    return(retVal);
}

bool geom_getBoxBoxCollision(const C7Vector& box1Transformation,const C3Vector& box1HalfSize,const C7Vector& box2Transformation,const C3Vector& box2HalfSize,bool boxesAreSolid)
{
    bool retVal=CCalcUtils::doCollide_box_box(box1Transformation.getMatrix(),box1HalfSize,box2Transformation.getMatrix(),box2HalfSize,boxesAreSolid);
    return(retVal);
}

bool geom_getBoxTriangleCollision(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& p,const C3Vector& v,const C3Vector& w)
{
    bool retVal=CCalcUtils::doCollide_box_tri(boxTransformation.getMatrix(),boxHalfSize,boxIsSolid,p,v,w);
    return(retVal);
}

bool geom_getBoxSegmentCollision(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& segmentEndPoint,const C3Vector& segmentVector)
{
    bool retVal=CCalcUtils::doCollide_box_segp(boxTransformation.getMatrix(),boxHalfSize,boxIsSolid,segmentEndPoint,segmentVector);
    return(retVal);
}

bool geom_getBoxPointCollision(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,const C3Vector& point)
{
    C3Vector pt(point);
    pt*=boxTransformation.getInverse();
    if (fabsf(pt(0))>boxHalfSize(0))
        return(false);
    if (fabsf(pt(1))>boxHalfSize(1))
        return(false);
    if (fabsf(pt(2))>boxHalfSize(2))
        return(false);
    return(true);
}

bool geom_getTriangleTriangleCollision(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,std::vector<float>* intersections/*=nullptr*/)
{
    bool retVal=false;
    if (intersections!=nullptr)
    {
        size_t c=intersections->size();
        if (CCalcUtils::doCollide_tri_tri(p1,v1,w1,-1,p2,v2,w2,-1,intersections,nullptr,nullptr))
            retVal=(intersections->size()==c+6); // when intersections are computed, we might get different results for border-line cases
    }
    else
        retVal=CCalcUtils::doCollide_tri_tri(p1,v1,w1,-1,p2,v2,w2,-1,nullptr,nullptr,nullptr);
    return(retVal);
}

bool geom_getTriangleSegmentCollision(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,std::vector<float>* intersections/*=nullptr*/)
{
    bool retVal=CCalcUtils::doCollide_tri_segp(p,v,w,-1,segmentEndPoint,segmentVector,intersections,nullptr);
    return(retVal);
}

bool geom_getBoxBoxDistanceIfSmaller(const C7Vector& box1Transformation,const C3Vector& box1HalfSize,const C7Vector& box2Transformation,const C3Vector& box2HalfSize,bool boxesAreSolid,float& dist,C3Vector* distSegPt1/*=nullptr*/,C3Vector* distSegPt2/*=nullptr*/,bool altRoutine/*=false*/)
{
    bool retVal;
    if (altRoutine)
        retVal=CCalcUtils::getDistance_box_box_alt(box1Transformation.getMatrix(),box1HalfSize,box2Transformation.getMatrix(),box2HalfSize,boxesAreSolid,dist,distSegPt1,distSegPt2);
    else
        retVal=CCalcUtils::getDistance_box_box(box1Transformation.getMatrix(),box1HalfSize,box2Transformation.getMatrix(),box2HalfSize,boxesAreSolid,dist,distSegPt1,distSegPt2);
    return(retVal);
}

float geom_getBoxBoxDistance(const C7Vector& box1Transformation,const C3Vector& box1HalfSize,const C7Vector& box2Transformation,const C3Vector& box2HalfSize,bool boxesAreSolid,C3Vector* distSegPt1/*=nullptr*/,C3Vector* distSegPt2/*=nullptr*/,bool altRoutine/*=false*/)
{
    float dist=FLT_MAX;
    geom_getBoxBoxDistanceIfSmaller(box1Transformation.getMatrix(),box1HalfSize,box2Transformation.getMatrix(),box2HalfSize,boxesAreSolid,dist,distSegPt1,distSegPt2,altRoutine);
    return(dist);
}

float geom_getApproxBoxBoxDistance(const C7Vector& box1Transformation,const C3Vector& box1HalfSize,const C7Vector& box2Transformation,const C3Vector& box2HalfSize)
{
    float retVal=CCalcUtils::getApproxDistance_box_box(box1Transformation.getMatrix(),box1HalfSize,box2Transformation.getMatrix(),box2HalfSize);
    return(retVal);
}

bool geom_getBoxCubeDistanceIfSmaller(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,const C7Vector& cubeTransformation,float cubeHalfSize,bool boxAndCubeAreSolid,float& dist,C3Vector* distSegPt1/*=nullptr*/,C3Vector* distSegPt2/*=nullptr*/)
{
    C3Vector chs(cubeHalfSize,cubeHalfSize,cubeHalfSize);
    bool retVal=geom_getBoxBoxDistanceIfSmaller(boxTransformation.getMatrix(),boxHalfSize,cubeTransformation.getMatrix(),chs,boxAndCubeAreSolid,dist,distSegPt1,distSegPt2);
    return(retVal);
}

float geom_getBoxCubeDistance(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,const C7Vector& cubeTransformation,float cubeHalfSize,bool boxAndCubeAreSolid,C3Vector* distSegPt1/*=nullptr*/,C3Vector* distSegPt2/*=nullptr*/)
{
    C3Vector chs(cubeHalfSize,cubeHalfSize,cubeHalfSize);
    float dist=geom_getBoxBoxDistance(boxTransformation,boxHalfSize,cubeTransformation,chs,boxAndCubeAreSolid,distSegPt1,distSegPt2);
    return(dist);
}

bool geom_getBoxTriangleDistanceIfSmaller(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* distSegPt1/*=nullptr*/,C3Vector* distSegPt2/*=nullptr*/,bool altRoutine/*=false*/)
{
    bool retVal;
    if (altRoutine)
        retVal=CCalcUtils::getDistance_box_tri_alt(boxTransformation.getMatrix(),boxHalfSize,boxIsSolid,p,v,w,dist,distSegPt1,distSegPt2);
    else
        retVal=CCalcUtils::getDistance_box_tri(boxTransformation.getMatrix(),boxHalfSize,boxIsSolid,p,v,w,dist,distSegPt1,distSegPt2);
    return(retVal);
}

float geom_getBoxTriangleDistance(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& p,const C3Vector& v,const C3Vector& w,C3Vector* distSegPt1/*=nullptr*/,C3Vector* distSegPt2/*=nullptr*/,bool altRoutine/*=false*/)
{
    float dist=FLT_MAX;
    geom_getBoxTriangleDistanceIfSmaller(boxTransformation,boxHalfSize,boxIsSolid,p,v,w,dist,distSegPt1,distSegPt2,altRoutine);
    return(dist);
}

bool geom_getBoxSegmentDistanceIfSmaller(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,float& dist,C3Vector* distSegPt1/*=nullptr*/,C3Vector* distSegPt2/*=nullptr*/,bool altRoutine/*=false*/)
{
    bool retVal;
    if (altRoutine)
        retVal=CCalcUtils::getDistance_box_segp_alt(boxTransformation.getMatrix(),boxHalfSize,boxIsSolid,segmentEndPoint,segmentVector,dist,distSegPt1,distSegPt2);
    else
        retVal=CCalcUtils::getDistance_box_segp(boxTransformation.getMatrix(),boxHalfSize,boxIsSolid,segmentEndPoint,segmentVector,dist,distSegPt1,distSegPt2);
    return(retVal);
}

float geom_getBoxSegmentDistance(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* distSegPt1/*=nullptr*/,C3Vector* distSegPt2/*=nullptr*/,bool altRoutine/*=false*/)
{
    float dist=FLT_MAX;
    geom_getBoxSegmentDistanceIfSmaller(boxTransformation.getMatrix(),boxHalfSize,boxIsSolid,segmentEndPoint,segmentVector,dist,distSegPt1,distSegPt2,altRoutine);
    return(dist);
}

bool geom_getBoxPointDistanceIfSmaller(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& point,float& dist,C3Vector* distSegPt1/*=nullptr*/)
{
    bool retVal=CCalcUtils::getDistance_box_pt(boxTransformation.getMatrix(),boxHalfSize,boxIsSolid,point,dist,distSegPt1,nullptr);
    return(retVal);
}

float geom_getBoxPointDistance(const C7Vector& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& point,C3Vector* distSegPt1/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getBoxPointDistanceIfSmaller(boxTransformation,boxHalfSize,boxIsSolid,point,dist,distSegPt1);
    return(dist);
}

bool geom_getTriangleTriangleDistanceIfSmaller(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,float& dist,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/)
{
    bool retVal=CCalcUtils::getDistance_tri_tri(p1,v1,w1,p2,v2,w2,dist,minDistSegPt1,minDistSegPt2);
    return(retVal);
}

float geom_getTriangleTriangleDistance(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getTriangleTriangleDistanceIfSmaller(p1,v1,w1,p2,v2,w2,dist,minDistSegPt1,minDistSegPt2);
    return(dist);
}


bool geom_getTriangleSegmentDistanceIfSmaller(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,float& dist,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/)
{
    bool retVal=CCalcUtils::getDistance_tri_segp(p,v,w,segmentEndPoint,segmentVector,dist,minDistSegPt1,minDistSegPt2);
    return(retVal);
}

float geom_getTriangleSegmentDistance(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getTriangleSegmentDistanceIfSmaller(p,v,w,segmentEndPoint,segmentVector,dist,minDistSegPt1,minDistSegPt2);
    return(dist);
}

bool geom_getTrianglePointDistanceIfSmaller(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& point,float& dist,C3Vector* minDistSegPt/*=nullptr*/)
{
    bool retVal=CCalcUtils::getDistance_tri_pt(p,v,w,point,dist,minDistSegPt);
    return(retVal);
}

float geom_getTrianglePointDistance(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& point,C3Vector* minDistSegPt/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getTrianglePointDistanceIfSmaller(p,v,w,point,dist,minDistSegPt);
    return(dist);
}

bool geom_getSegmentSegmentDistanceIfSmaller(const C3Vector& segment1EndPoint,const C3Vector& segment1Vector,const C3Vector& segment2EndPoint,const C3Vector& segment2Vector,float& dist,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/)
{
    bool retVal=CCalcUtils::getDistance_segp_segp(segment1EndPoint,segment1Vector,segment2EndPoint,segment2Vector,dist,minDistSegPt1,minDistSegPt2);
    return(retVal);
}

float geom_getSegmentSegmentDistance(const C3Vector& segment1EndPoint,const C3Vector& segment1Vector,const C3Vector& segment2EndPoint,const C3Vector& segment2Vector,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getSegmentSegmentDistanceIfSmaller(segment1EndPoint,segment1Vector,segment2EndPoint,segment2Vector,dist,minDistSegPt1,minDistSegPt2);
    return(dist);
}

bool geom_getSegmentPointDistanceIfSmaller(const C3Vector& segmentEndPoint,const C3Vector& segmentVector,const C3Vector& point,float& dist,C3Vector* minDistSegPt/*=nullptr*/)
{
    bool retVal=CCalcUtils::getDistance_segp_pt(segmentEndPoint,segmentVector,point,dist,minDistSegPt);
    return(retVal);
}

float geom_getSegmentPointDistance(const C3Vector& segmentEndPoint,const C3Vector& segmentVector,const C3Vector& point,C3Vector* minDistSegPt/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getSegmentPointDistanceIfSmaller(segmentEndPoint,segmentVector,point,dist,minDistSegPt);
    return(dist);
}

CObbStruct* geom_createMesh(const float* vertices,int verticesSize,const int* indices,int indicesSize,const C7Vector* meshOrigin/*=nullptr*/,float triangleEdgeMaxLength/*=0.3f*/,int maxTrianglesInBoundingBox/*=8*/)
{
    std::vector<float> _vertices;
    _vertices.resize(size_t(verticesSize));
    C7Vector trInv;
    trInv.setIdentity();
    if (meshOrigin!=nullptr)
        trInv=meshOrigin->getInverse();
    for (size_t i=0;i<size_t(verticesSize)/3;i++)
    {
        C3Vector v(vertices+3*i);
        v*=trInv;
        _vertices[3*i+0]=v(0);
        _vertices[3*i+1]=v(1);
        _vertices[3*i+2]=v(2);
    }
    CObbStruct* newObbStruct=CObbStruct::copyObbStructFromExisting(&_vertices[0],verticesSize,indices,indicesSize,triangleEdgeMaxLength,maxTrianglesInBoundingBox);
    if (newObbStruct==nullptr)
        newObbStruct=new CObbStruct(&_vertices[0],verticesSize,indices,indicesSize,triangleEdgeMaxLength,maxTrianglesInBoundingBox);
    CObbStruct::addObbStruct(newObbStruct);
    return(newObbStruct);
}

CObbStruct* geom_copyMesh(const CObbStruct* meshObbStruct)
{
    CObbStruct* newObbStruct=meshObbStruct->copyYourself();
    CObbStruct::addObbStruct(newObbStruct);
    return(newObbStruct);
}

void geom_scaleMesh(CObbStruct* meshObbStruct,float scalingFactor)
{
    meshObbStruct->scaleYourself(scalingFactor);
}

void geom_getMeshSerializationData(const CObbStruct* meshObbStruct,std::vector<unsigned char>& serializationData)
{
    int s;
    unsigned char* data=meshObbStruct->serialize(s);
    serializationData.assign(data,data+s);
    delete[] data;
}

CObbStruct* geom_getMeshFromSerializationData(const unsigned char* serializationData)
{
    CObbStruct* newObbStruct=new CObbStruct();
    if (newObbStruct->deserialize(serializationData))
    {
        CObbStruct::addObbStruct(newObbStruct);
        return(newObbStruct);
    }
    else
    {
        delete newObbStruct;
        return(nullptr);
    }
}

void geom_destroyMesh(CObbStruct* meshObbStruct)
{
    CObbStruct::removeObbStruct(meshObbStruct);
}

float geom_getMeshRootObbVolume(const CObbStruct* meshObbStruct)
{
    return(meshObbStruct->obb->boxHs(0)*meshObbStruct->obb->boxHs(1)*meshObbStruct->obb->boxHs(2)*8);
}

bool geom_getMeshMeshDistanceIfSmaller(const CObbStruct* mesh1ObbStruct,const C7Vector& mesh1Transformation,const CObbStruct* mesh2ObbStruct,const C7Vector& mesh2Transformation,float& dist,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/,int* mesh1Caching/*=nullptr*/,int* mesh2Caching/*=nullptr*/)
{
    bool retVal=false;
    if ( (mesh1Caching!=nullptr)&&(mesh2Caching!=nullptr) )
    {
        if ( (mesh1Caching[0]>=0)&&(mesh1Caching[0]<mesh1ObbStruct->indices.size()/3)&&(mesh2Caching[0]>=0)&&(mesh2Caching[0]<mesh2ObbStruct->indices.size()/3) )
        {
            int tri1Ind[3]={mesh1ObbStruct->indices[3*mesh1Caching[0]+0],mesh1ObbStruct->indices[3*mesh1Caching[0]+1],mesh1ObbStruct->indices[3*mesh1Caching[0]+2]};
            int tri2Ind[3]={mesh2ObbStruct->indices[3*mesh2Caching[0]+0],mesh2ObbStruct->indices[3*mesh2Caching[0]+1],mesh2ObbStruct->indices[3*mesh2Caching[0]+2]};
            C3Vector p11(&mesh1ObbStruct->vertices[3*tri1Ind[0]]);
            C3Vector p12(&mesh1ObbStruct->vertices[3*tri1Ind[1]]);
            C3Vector p13(&mesh1ObbStruct->vertices[3*tri1Ind[2]]);
            C3Vector p21(&mesh2ObbStruct->vertices[3*tri2Ind[0]]);
            C3Vector p22(&mesh2ObbStruct->vertices[3*tri2Ind[1]]);
            C3Vector p23(&mesh2ObbStruct->vertices[3*tri2Ind[2]]);
            p11=mesh1Transformation*p11;
            p12=mesh1Transformation*p12;
            p13=mesh1Transformation*p13;
            p21=mesh2Transformation*p21;
            p22=mesh2Transformation*p22;
            p23=mesh2Transformation*p23;
            retVal=CCalcUtils::getDistance_tri_tri(p11,p12-p11,p13-p11,p21,p22-p21,p23-p21,dist,minDistSegPt1,minDistSegPt2);
        }
    }
    bool b=mesh1ObbStruct->obb->checkDistance_obb(mesh1ObbStruct,mesh1Transformation.getMatrix(),mesh2ObbStruct->obb,mesh2ObbStruct,mesh2Transformation.getMatrix(),dist,minDistSegPt1,minDistSegPt2,mesh1Caching,mesh2Caching);
    retVal=retVal||b;
    return(retVal);
}

bool geom_getMeshTriangleDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/,int* caching/*=nullptr*/)
{
    bool retVal=false;
    if (caching!=nullptr)
    {
        if ( (caching[0]>=0)&&(caching[0]<meshObbStruct->indices.size()/3) )
        {
            int triInd[3]={meshObbStruct->indices[3*caching[0]+0],meshObbStruct->indices[3*caching[0]+1],meshObbStruct->indices[3*caching[0]+2]};
            C3Vector p1(&meshObbStruct->vertices[3*triInd[0]]);
            C3Vector p2(&meshObbStruct->vertices[3*triInd[1]]);
            C3Vector p3(&meshObbStruct->vertices[3*triInd[2]]);
            p1=meshTransformation*p1;
            p2=meshTransformation*p2;
            p3=meshTransformation*p3;
            retVal=CCalcUtils::getDistance_tri_tri(p1,p2-p1,p3-p1,p,v,w,dist,minDistSegPt1,minDistSegPt2);
        }
    }
    bool b=meshObbStruct->obb->checkDistance_tri(meshObbStruct,meshTransformation.getMatrix(),p,v,w,-1,dist,minDistSegPt1,minDistSegPt2,caching,nullptr);
    retVal=retVal||b;
    return(retVal);
}

float geom_getMeshTriangleDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/,int* caching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getMeshTriangleDistanceIfSmaller(meshObbStruct,meshTransformation,p,v,w,dist,minDistSegPt1,minDistSegPt2,caching);
    return(dist);
}

float geom_getMeshMeshDistance(const CObbStruct* mesh1ObbStruct,const C7Vector& mesh1Transformation,const CObbStruct* mesh2ObbStruct,const C7Vector& mesh2Transformation,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/,int* mesh1Caching/*=nullptr*/,int* mesh2Caching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getMeshMeshDistanceIfSmaller(mesh1ObbStruct,mesh1Transformation,mesh2ObbStruct,mesh2Transformation,dist,minDistSegPt1,minDistSegPt2,mesh1Caching,mesh2Caching);
    return(dist);
}

bool geom_getMeshSegmentDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,float& dist,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/,int* caching/*=nullptr*/)
{
    bool retVal=false;
    if (caching!=nullptr)
    {
        if ( (caching[0]>=0)&&(caching[0]<meshObbStruct->indices.size()/3) )
        {
            int triInd[3]={meshObbStruct->indices[3*caching[0]+0],meshObbStruct->indices[3*caching[0]+1],meshObbStruct->indices[3*caching[0]+2]};
            C3Vector p1(&meshObbStruct->vertices[3*triInd[0]]);
            C3Vector p2(&meshObbStruct->vertices[3*triInd[1]]);
            C3Vector p3(&meshObbStruct->vertices[3*triInd[2]]);
            p1=meshTransformation*p1;
            p2=meshTransformation*p2;
            p3=meshTransformation*p3;
            retVal=CCalcUtils::getDistance_tri_segp(p1,p2-p1,p3-p1,segmentEndPoint,segmentVector,dist,minDistSegPt1,minDistSegPt2);
        }
    }
    bool b=meshObbStruct->obb->checkDistance_segp(meshObbStruct,meshTransformation.getMatrix(),segmentEndPoint,segmentVector,dist,minDistSegPt1,minDistSegPt2,caching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getMeshSegmentDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* minDistSegPt1/*=nullptr*/,C3Vector* minDistSegPt2/*=nullptr*/,int* caching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getMeshSegmentDistanceIfSmaller(meshObbStruct,meshTransformation,segmentEndPoint,segmentVector,dist,minDistSegPt1,minDistSegPt2,caching);
    return(dist);
}

bool geom_getMeshPointDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& point,float& dist,C3Vector* minDistSegPt/*=nullptr*/,int* caching/*=nullptr*/)
{
    bool retVal=false;
    if (caching!=nullptr)
    {
        if ( (caching[0]>=0)&&(caching[0]<meshObbStruct->indices.size()/3) )
        {
            int triInd[3]={meshObbStruct->indices[3*caching[0]+0],meshObbStruct->indices[3*caching[0]+1],meshObbStruct->indices[3*caching[0]+2]};
            C3Vector p1(&meshObbStruct->vertices[3*triInd[0]]);
            C3Vector p2(&meshObbStruct->vertices[3*triInd[1]]);
            C3Vector p3(&meshObbStruct->vertices[3*triInd[2]]);
            p1=meshTransformation*p1;
            p2=meshTransformation*p2;
            p3=meshTransformation*p3;
            retVal=CCalcUtils::getDistance_tri_pt(p1,p2-p1,p3-p1,point,dist,minDistSegPt);
        }
    }
    bool b=meshObbStruct->obb->checkDistance_pt(meshObbStruct,meshTransformation.getMatrix(),point,dist,minDistSegPt,caching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getMeshPointDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C3Vector& point,C3Vector* minDistSegPt/*=nullptr*/,int* caching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getMeshPointDistanceIfSmaller(meshObbStruct,meshTransformation,point,dist,minDistSegPt,caching);
    return(dist);
}

bool geom_getMeshOctreeDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const COcStruct* ocStruct,const C7Vector& octreeTransformation,float& dist,C3Vector* meshMinDistPt/*=nullptr*/,C3Vector* ocMinDistPt/*=nullptr*/,int* meshCaching/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/)
{
    bool retVal=false;
    if ( (meshCaching!=nullptr)&&(ocCaching!=nullptr) )
    {
        if ( (meshCaching[0]>=0)&&(meshCaching[0]<meshObbStruct->indices.size()/3) )
        {
            C4X4Matrix cellM;
            float cellS;
            if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)ocCaching[0],cellS,cellM,nullptr))
            {
                C4X4Matrix cellMi(cellM.getInverse());
                int triInd[3]={meshObbStruct->indices[3*meshCaching[0]+0],meshObbStruct->indices[3*meshCaching[0]+1],meshObbStruct->indices[3*meshCaching[0]+2]};
                C3Vector p1(&meshObbStruct->vertices[3*triInd[0]]);
                C3Vector p2(&meshObbStruct->vertices[3*triInd[1]]);
                C3Vector p3(&meshObbStruct->vertices[3*triInd[2]]);
                p1=cellMi*meshTransformation*p1;
                p2=cellMi*meshTransformation*p2;
                p3=cellMi*meshTransformation*p3;
                retVal=CCalcUtils::getDistance_cell_tri(cellS*0.5f,true,p1,p2-p1,p3-p1,dist,ocMinDistPt,meshMinDistPt);
                if (retVal)
                {
                    if (ocMinDistPt!=nullptr)
                        ocMinDistPt[0]=cellM*ocMinDistPt[0];
                    if (meshMinDistPt!=nullptr)
                        meshMinDistPt[0]=cellM*meshMinDistPt[0];
                }
            }
        }
    }
    bool b=ocStruct->getDistance_shape(octreeTransformation.getMatrix(),meshObbStruct,meshTransformation.getMatrix(),dist,ocMinDistPt,meshMinDistPt,ocCaching,meshCaching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getMeshOctreeDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const COcStruct* ocStruct,const C7Vector& octreeTransformation,C3Vector* meshMinDistPt/*=nullptr*/,C3Vector* ocMinDistPt/*=nullptr*/,int* meshCaching/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getMeshOctreeDistanceIfSmaller(meshObbStruct,meshTransformation,ocStruct,octreeTransformation,dist,meshMinDistPt,ocMinDistPt,meshCaching,ocCaching);
    return(dist);
}

bool geom_getMeshPtcloudDistanceIfSmaller(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const CPcStruct* pcStruct,const C7Vector& pcTransformation,float& dist,C3Vector* meshMinDistPt/*=nullptr*/,C3Vector* pcMinDistPt/*=nullptr*/,int* meshCaching/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    bool retVal=false;
    if ( (meshCaching!=nullptr)&&(pcCaching!=nullptr) )
    {
        if ( (meshCaching[0]>=0)&&(meshCaching[0]<meshObbStruct->indices.size()/3) )
        {
            C4X4Matrix tr;
            size_t cnt=0;
            const float* pts=pcStruct->getPoints(pcTransformation.getMatrix(),(const unsigned long long int)pcCaching[0],&cnt,tr);
            if ( (pts!=nullptr)&&(cnt>0) )
            {
                C4X4Matrix tri(tr.getInverse());
                int triInd[3]={meshObbStruct->indices[3*meshCaching[0]+0],meshObbStruct->indices[3*meshCaching[0]+1],meshObbStruct->indices[3*meshCaching[0]+2]};
                C3Vector p1(&meshObbStruct->vertices[3*triInd[0]]);
                C3Vector p2(&meshObbStruct->vertices[3*triInd[1]]);
                C3Vector p3(&meshObbStruct->vertices[3*triInd[2]]);
                p1=tri*meshTransformation*p1;
                p2=tri*meshTransformation*p2;
                p3=tri*meshTransformation*p3;
                for (size_t i=0;i<cnt;i++)
                {
                    C3Vector pt(pts+3*i);
                    if (CCalcUtils::getDistance_tri_pt(p1,p2-p1,p3-p1,pt,dist,meshMinDistPt))
                    {
                        retVal=true;
                        if (pcMinDistPt!=nullptr)
                            pcMinDistPt[0]=tr*pt;
                        if (meshMinDistPt!=nullptr)
                            meshMinDistPt[0]=tr*meshMinDistPt[0];
                    }
                }
            }
        }
    }
    bool b=pcStruct->getDistance_shape(meshObbStruct,pcTransformation.getMatrix(),meshTransformation.getMatrix(),dist,pcMinDistPt,meshMinDistPt,pcCaching,meshCaching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getMeshPtcloudDistance(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const CPcStruct* pcStruct,const C7Vector& pcTransformation,C3Vector* meshMinDistPt/*=nullptr*/,C3Vector* pcMinDistPt/*=nullptr*/,int* meshCaching/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getMeshPtcloudDistanceIfSmaller(meshObbStruct,meshTransformation,pcStruct,pcTransformation,dist,meshMinDistPt,pcMinDistPt,meshCaching,pcCaching);
    return(dist);
}

bool geom_getOctreePtcloudDistanceIfSmaller(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const CPcStruct* pcStruct,const C7Vector& pcTransformation,float& dist,C3Vector* ocMinDistPt/*=nullptr*/,C3Vector* pcMinDistPt/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    bool retVal=false;
    if ( (ocCaching!=nullptr)&&(pcCaching!=nullptr) )
    {
        C4X4Matrix cellM;
        float cellS;
        if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)ocCaching[0],cellS,cellM,nullptr))
        {
            C4X4Matrix tr;
            size_t cnt=0;
            const float* pts=pcStruct->getPoints(pcTransformation.getMatrix(),(const unsigned long long int)pcCaching[0],&cnt,tr);
            if ( (pts!=nullptr)&&(cnt>0) )
            {
                C4X4Matrix m(cellM.getInverse()*tr);
                for (size_t i=0;i<cnt;i++)
                {
                    C3Vector pt(pts+3*i);
                    pt*=m;
                    if (CCalcUtils::getDistance_cell_pt(cellS*0.5f,true,pt,dist,ocMinDistPt,nullptr))
                    {
                        retVal=true;
                        if (ocMinDistPt!=nullptr)
                            ocMinDistPt[0]=cellM*ocMinDistPt[0];
                        if (pcMinDistPt!=nullptr)
                            pcMinDistPt[0]=cellM*pt;
                    }
                }
            }
        }
    }
    bool b=ocStruct->getDistance_ptcloud(octreeTransformation.getMatrix(),pcStruct,pcTransformation.getMatrix(),dist,ocMinDistPt,pcMinDistPt,ocCaching,pcCaching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getOctreePtcloudDistance(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const CPcStruct* pcStruct,const C7Vector& pcTransformation,C3Vector* ocMinDistPt/*=nullptr*/,C3Vector* pcMinDistPt/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getOctreePtcloudDistanceIfSmaller(ocStruct,octreeTransformation,pcStruct,pcTransformation,dist,ocMinDistPt,pcMinDistPt,ocCaching,pcCaching);
    return(dist);
}

bool geom_getOctreeOctreeDistanceIfSmaller(const COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation,float& dist,C3Vector* oc1MinDistPt/*=nullptr*/,C3Vector* oc2MinDistPt/*=nullptr*/,unsigned  long long int* oc1Caching/*=nullptr*/,unsigned  long long int* oc2Caching/*=nullptr*/)
{
    bool retVal=false;
    if ( (oc1Caching!=nullptr)&&(oc2Caching!=nullptr) )
    {
        C4X4Matrix cell1M;
        float cell1S;
        if (oc1Struct->getCell(octree1Transformation.getMatrix(),(const unsigned long long int)oc1Caching[0],cell1S,cell1M,nullptr))
        {
            C4X4Matrix cell2M;
            float cell2S;
            if (oc2Struct->getCell(octree2Transformation.getMatrix(),(const unsigned long long int)oc2Caching[0],cell2S,cell2M,nullptr))
                retVal=CCalcUtils::getDistance_cell_cell(cell1M,cell1S*0.5f,cell2M,cell2S*0.5f,true,dist,oc1MinDistPt,oc2MinDistPt);
        }
    }
    bool b=oc1Struct->getDistance_octree(octree1Transformation.getMatrix(),oc2Struct,octree2Transformation.getMatrix(),dist,oc1MinDistPt,oc2MinDistPt,oc1Caching,oc2Caching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getOctreeOctreeDistance(const COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation,C3Vector* oc1MinDistPt/*=nullptr*/,C3Vector* oc2MinDistPt/*=nullptr*/,unsigned  long long int* oc1Caching/*=nullptr*/,unsigned  long long int* oc2Caching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getOctreeOctreeDistanceIfSmaller(oc1Struct,octree1Transformation,oc2Struct,octree2Transformation,dist,oc1MinDistPt,oc2MinDistPt,oc1Caching,oc2Caching);
    return(dist);
}

bool geom_getOctreeTriangleDistanceIfSmaller(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* ocMinDistPt/*=nullptr*/,C3Vector* triMinDistPt/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/)
{
    bool retVal=false;
    if (ocCaching!=nullptr)
    {
        C4X4Matrix cellM;
        float cellS;
        if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)ocCaching[0],cellS,cellM,nullptr))
        {
            C4X4Matrix cellMi(cellM.getInverse());
            C3Vector _p(cellMi*p);
            cellMi.X.clear();
            C3Vector _v(cellMi*v);
            C3Vector _w(cellMi*w);
            retVal=CCalcUtils::getDistance_cell_tri(cellS*0.5f,true,_p,_v,_w,dist,ocMinDistPt,triMinDistPt);
            if (retVal)
            {
                if (ocMinDistPt!=nullptr)
                    ocMinDistPt[0]=cellM*ocMinDistPt[0];
                if (triMinDistPt!=nullptr)
                    triMinDistPt[0]=cellM*triMinDistPt[0];
            }
        }
    }
    C7Vector trInv(octreeTransformation.getInverse());
    C3Vector _p(trInv*p);
    trInv.X.clear();
    C3Vector _v(trInv*v);
    C3Vector _w(trInv*w);
    bool b=ocStruct->getDistance_tri(_p,_v,_w,dist,ocMinDistPt,triMinDistPt,ocCaching);
    if (b)
    {
        if (ocMinDistPt!=nullptr)
            ocMinDistPt[0]=octreeTransformation*ocMinDistPt[0];
        if (triMinDistPt!=nullptr)
            triMinDistPt[0]=octreeTransformation*triMinDistPt[0];
    }
    retVal=retVal||b;
    return(retVal);
}

float geom_getOctreeTriangleDistance(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& p,const C3Vector& v,const C3Vector&w,C3Vector* ocMinDistPt/*=nullptr*/,C3Vector* triMinDistPt/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getOctreeTriangleDistanceIfSmaller(ocStruct,octreeTransformation,p,v,w,dist,ocMinDistPt,triMinDistPt,ocCaching);
    return(dist);
}

bool geom_getOctreeSegmentDistanceIfSmaller(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,float& dist,C3Vector* ocMinDistPt/*=nullptr*/,C3Vector* segMinDistPt/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/)
{
    bool retVal=false;
    if (ocCaching!=nullptr)
    {
        C4X4Matrix cellM;
        float cellS;
        if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)ocCaching[0],cellS,cellM,nullptr))
        {
            C4X4Matrix cellMi(cellM.getInverse());
            C3Vector _segP(cellMi*segmentEndPoint);
            cellMi.X.clear();
            C3Vector _segL(cellMi*segmentVector);
            retVal=CCalcUtils::getDistance_cell_segp(cellS*0.5f,true,_segP,_segL,dist,ocMinDistPt,segMinDistPt);
            if (retVal)
            {
                if (ocMinDistPt!=nullptr)
                    ocMinDistPt[0]=cellM*ocMinDistPt[0];
                if (segMinDistPt!=nullptr)
                    segMinDistPt[0]=cellM*segMinDistPt[0];
            }
        }
    }
    C7Vector trInv(octreeTransformation.getInverse());
    C3Vector _segP(trInv*segmentEndPoint);
    trInv.X.clear();
    C3Vector _segL(trInv*segmentVector);
    bool b=ocStruct->getDistance_seg(_segP+_segL*0.5f,_segL*0.5f,dist,ocMinDistPt,segMinDistPt,ocCaching);
    if (b)
    {
        if (ocMinDistPt!=nullptr)
            ocMinDistPt[0]=octreeTransformation*ocMinDistPt[0];
        if (segMinDistPt!=nullptr)
            segMinDistPt[0]=octreeTransformation*segMinDistPt[0];
    }
    retVal=retVal||b;
    return(retVal);
}

float geom_getOctreeSegmentDistance(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* ocMinDistPt/*=nullptr*/,C3Vector* segMinDistPt/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getOctreeSegmentDistanceIfSmaller(ocStruct,octreeTransformation,segmentEndPoint,segmentVector,dist,ocMinDistPt,segMinDistPt,ocCaching);
    return(dist);
}

bool geom_getOctreePointDistanceIfSmaller(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& point,float& dist,C3Vector* ocMinDistPt/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/)
{
    bool retVal=false;
    if (ocCaching!=nullptr)
    {
        C4X4Matrix cellM;
        float cellS;
        if (ocStruct->getCell(octreeTransformation.getMatrix(),(const unsigned long long int)ocCaching[0],cellS,cellM,nullptr))
        {
            C4X4Matrix cellMi(cellM.getInverse());
            C3Vector _point(cellMi*point);
            retVal=CCalcUtils::getDistance_cell_pt(cellS*0.5f,true,_point,dist,ocMinDistPt,nullptr);
            if (retVal)
            {
                if (ocMinDistPt!=nullptr)
                    ocMinDistPt[0]=cellM*ocMinDistPt[0];
            }
        }
    }
    C7Vector trInv(octreeTransformation.getInverse());
    C3Vector _point(trInv*point);
    bool b=ocStruct->getDistance_pt(_point,dist,ocMinDistPt,ocCaching);
    if (b)
    {
        if (ocMinDistPt!=nullptr)
            ocMinDistPt[0]=octreeTransformation*ocMinDistPt[0];
    }
    retVal=retVal||b;
    return(retVal);
}

float geom_getOctreePointDistance(const COcStruct* ocStruct,const C7Vector& octreeTransformation,const C3Vector& point,C3Vector* ocMinDistPt/*=nullptr*/,unsigned  long long int* ocCaching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getOctreePointDistanceIfSmaller(ocStruct,octreeTransformation,point,dist,ocMinDistPt,ocCaching);
    return(dist);
}

bool geom_getPtcloudPtcloudDistanceIfSmaller(const CPcStruct* pc1Struct,const C7Vector& pc1Transformation,const CPcStruct* pc2Struct,const C7Vector& pc2Transformation,float& dist,C3Vector* pc1MinDistPt/*=nullptr*/,C3Vector* pc2MinDistPt/*=nullptr*/,unsigned  long long int* pc1Caching/*=nullptr*/,unsigned  long long int* pc2Caching/*=nullptr*/)
{
    bool retVal=false;
    if ( (pc1Caching!=nullptr)&&(pc2Caching!=nullptr) )
    {
        C4X4Matrix tr1;
        size_t cnt1=0;
        const float* pts1=pc1Struct->getPoints(pc1Transformation.getMatrix(),(const unsigned long long int)pc1Caching[0],&cnt1,tr1);
        if ( (pts1!=nullptr)&&(cnt1>0) )
        {
            C4X4Matrix tr2;
            size_t cnt2=0;
            const float* pts2=pc2Struct->getPoints(pc2Transformation.getMatrix(),(const unsigned long long int)pc2Caching[0],&cnt2,tr2);
            if ( (pts2!=nullptr)&&(cnt2>0) )
            {
                C4X4Matrix tr1Rel(tr2.getInverse()*tr1);
                for (size_t i=0;i<cnt1;i++)
                {
                    C3Vector pt1(pts1+3*i);
                    pt1*=tr1Rel;
                    for (size_t j=0;j<cnt2;j++)
                    {
                        C3Vector pt2(pts2+3*j);
                        float d=(pt2-pt1).getLength();
                        if (d<dist)
                        {
                            dist=d;
                            retVal=true;
                            if (pc1MinDistPt!=nullptr)
                                pc1MinDistPt[0]=tr2*pt1;
                            if (pc2MinDistPt!=nullptr)
                                pc2MinDistPt[0]=tr2*pt2;
                        }
                    }
                }
            }
        }
    }
    bool b=pc1Struct->getDistance_ptcloud(pc2Struct,pc1Transformation.getMatrix(),pc2Transformation.getMatrix(),dist,pc1MinDistPt,pc2MinDistPt,pc1Caching,pc2Caching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getPtcloudPtcloudDistance(const CPcStruct* pc1Struct,const C7Vector& pc1Transformation,const CPcStruct* pc2Struct,const C7Vector& pc2Transformation,C3Vector* pc1MinDistPt/*=nullptr*/,C3Vector* pc2MinDistPt/*=nullptr*/,unsigned  long long int* pc1Caching/*=nullptr*/,unsigned  long long int* pc2Caching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getPtcloudPtcloudDistanceIfSmaller(pc1Struct,pc1Transformation,pc2Struct,pc2Transformation,dist,pc1MinDistPt,pc2MinDistPt,pc1Caching,pc2Caching);
    return(dist);
}

bool geom_getPtcloudTriangleDistanceIfSmaller(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* pcMinDistPt/*=nullptr*/,C3Vector* triMinDistPt/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    bool retVal=false;
    if (pcCaching!=nullptr)
    {
        C4X4Matrix tr;
        size_t cnt=0;
        const float* pts=pcStruct->getPoints(pcTransformation.getMatrix(),(const unsigned long long int)pcCaching[0],&cnt,tr);
        if ( (pts!=nullptr)&&(cnt>0) )
        {
            for (size_t i=0;i<cnt;i++)
            {
                C3Vector pt(pts+3*i);
                pt*=tr;
                if (CCalcUtils::getDistance_tri_pt(p,v,w,pt,dist,triMinDistPt))
                {
                    retVal=true;
                    if (pcMinDistPt!=nullptr)
                        pcMinDistPt[0]=pt;
                }
            }
        }
    }
    bool b=pcStruct->getDistance_tri(pcTransformation.getMatrix(),p,v,w,dist,pcMinDistPt,triMinDistPt,pcCaching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getPtcloudTriangleDistance(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,C3Vector* pcMinDistPt/*=nullptr*/,C3Vector* triMinDistPt/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getPtcloudTriangleDistanceIfSmaller(pcStruct,pcTransformation,p,v,w,dist,pcMinDistPt,triMinDistPt,pcCaching);
    return(dist);
}

bool geom_getPtcloudSegmentDistanceIfSmaller(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,float& dist,C3Vector* pcMinDistPt/*=nullptr*/,C3Vector* segMinDistPt/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    bool retVal=false;
    if (pcCaching!=nullptr)
    {
        C4X4Matrix tr;
        size_t cnt=0;
        const float* pts=pcStruct->getPoints(pcTransformation.getMatrix(),(const unsigned long long int)pcCaching[0],&cnt,tr);
        if ( (pts!=nullptr)&&(cnt>0) )
        {
            for (size_t i=0;i<cnt;i++)
            {
                C3Vector pt(pts+3*i);
                pt*=tr;
                if (CCalcUtils::getDistance_segp_pt(segmentEndPoint,segmentVector,pt,dist,segMinDistPt))
                {
                    retVal=true;
                    if (pcMinDistPt!=nullptr)
                        pcMinDistPt[0]=pt;
                }
            }
        }
    }
    bool b=pcStruct->getDistance_seg(pcTransformation.getMatrix(),segmentEndPoint+segmentVector*0.5f,segmentVector*0.5f,dist,pcMinDistPt,segMinDistPt,pcCaching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getPtcloudSegmentDistance(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,C3Vector* pcMinDistPt/*=nullptr*/,C3Vector* segMinDistPt/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getPtcloudSegmentDistanceIfSmaller(pcStruct,pcTransformation,segmentEndPoint,segmentVector,dist,pcMinDistPt,segMinDistPt,pcCaching);
    return(dist);
}

bool geom_getPtcloudPointDistanceIfSmaller(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& point,float& dist,C3Vector* pcMinDistPt/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    bool retVal=false;
    if (pcCaching!=nullptr)
    {
        C4X4Matrix tr;
        size_t cnt=0;
        const float* pts=pcStruct->getPoints(pcTransformation.getMatrix(),(const unsigned long long int)pcCaching[0],&cnt,tr);
        if ( (pts!=nullptr)&&(cnt>0) )
        {
            for (size_t i=0;i<cnt;i++)
            {
                C3Vector pt(pts+3*i);
                pt*=tr;
                float d=(pt-point).getLength();
                if (d<dist)
                {
                    dist=d;
                    retVal=true;
                    if (pcMinDistPt!=nullptr)
                        pcMinDistPt[0]=pt;
                }
            }
        }
    }
    bool b=pcStruct->getDistance_pt(pcTransformation.getMatrix(),point,dist,pcMinDistPt,nullptr,pcCaching);
    retVal=retVal||b;
    return(retVal);
}

float geom_getPtcloudPointDistance(const CPcStruct* pcStruct,const C7Vector& pcTransformation,const C3Vector& point,C3Vector* pcMinDistPt/*=nullptr*/,unsigned  long long int* pcCaching/*=nullptr*/)
{
    float dist=FLT_MAX;
    geom_getPtcloudPointDistanceIfSmaller(pcStruct,pcTransformation,point,dist,pcMinDistPt,pcCaching);
    return(dist);
}

void geom_getOctreeVoxelData(const COcStruct* ocStruct,std::vector<float>& voxelData,std::vector<unsigned int>* userData/*=nullptr*/)
{ // returns voxel positions and color
    ocStruct->getVoxelsPosAndRgb(voxelData,userData);
}

COcStruct* geom_createOctreeFromPoints(const float* points,int pointCnt,const C7Vector* octreeOrigin/*=nullptr*/,float cellS/*=0.05f*/,const unsigned char rgbData[3]/*=nullptr*/,unsigned int usrData/*=0*/)
{
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv;
    trInv.setIdentity();
    if (octreeOrigin!=nullptr)
        trInv=octreeOrigin->getInverse();
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    unsigned char _rgbData[3];
    if (rgbData!=nullptr)
    {
        _rgbData[0]=rgbData[0];
        _rgbData[1]=rgbData[1];
        _rgbData[2]=rgbData[2];
    }
    COcStruct* ocStruct=new COcStruct(cellS,&relPts[0],(size_t)pointCnt,_rgbData,&usrData,false);
    return(ocStruct);
}

COcStruct* geom_createOctreeFromColorPoints(const float* points,int pointCnt,const C7Vector* octreeOrigin/*=nullptr*/,float cellS/*=0.05f*/,const unsigned char* rgbData/*=nullptr*/,const unsigned int* usrData/*=nullptr*/)
{
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv;
    trInv.setIdentity();
    if (octreeOrigin!=nullptr)
        trInv=octreeOrigin->getInverse();
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    std::vector<unsigned char> _rgbData;
    _rgbData.resize(pointCnt*3,0);
    if (rgbData!=nullptr)
    {
        for (int i=0;i<pointCnt*3;i++)
            _rgbData[i]=rgbData[i];
    }
    std::vector<unsigned int> _usrData;
    _usrData.resize(pointCnt,0);
    if (usrData!=nullptr)
    {
        for (int i=0;i<pointCnt;i++)
            _usrData[i]=usrData[i];
    }
    COcStruct* ocStruct=new COcStruct(cellS,&relPts[0],(size_t)pointCnt,&_rgbData[0],&_usrData[0],true);
    return(ocStruct);
}

COcStruct* geom_createOctreeFromMesh(const CObbStruct* meshObbStruct,const C7Vector& meshTransformation,const C7Vector* octreeOrigin/*=nullptr*/,float cellS/*=0.05f*/,const unsigned char rgbData[3]/*=nullptr*/,unsigned int usrData/*=0*/)
{
    C7Vector _octreeOrigin;
    _octreeOrigin.setIdentity();
    if (octreeOrigin!=nullptr)
        _octreeOrigin=octreeOrigin[0];
    unsigned char _rgbData[3];
    if (rgbData!=nullptr)
    {
        _rgbData[0]=rgbData[0];
        _rgbData[1]=rgbData[1];
        _rgbData[2]=rgbData[2];
    }
    COcStruct* ocStruct=new COcStruct(_octreeOrigin.getMatrix(),cellS,meshObbStruct,meshTransformation.getMatrix(),_rgbData,usrData);
    return(ocStruct);
}

COcStruct* geom_createOctreeFromOctree(const COcStruct* otherOctreeStruct,const C7Vector& otherOctreeTransformation,const C7Vector* newOctreeOrigin/*=nullptr*/,float newOctreeCellS/*=0.05f*/,const unsigned char rgbData[3]/*=nullptr*/,unsigned int usrData/*=0*/)
{
    C7Vector _newOctreeOrigin;
    _newOctreeOrigin.setIdentity();
    if (newOctreeOrigin!=nullptr)
        _newOctreeOrigin=newOctreeOrigin[0];
    unsigned char _rgbData[3];
    if (rgbData!=nullptr)
    {
        _rgbData[0]=rgbData[0];
        _rgbData[1]=rgbData[1];
        _rgbData[2]=rgbData[2];
    }
    COcStruct* ocStruct=new COcStruct(_newOctreeOrigin.getMatrix(),newOctreeCellS,otherOctreeStruct,otherOctreeTransformation.getMatrix(),_rgbData,usrData);
    return(ocStruct);
}

COcStruct* geom_copyOctree(const COcStruct* ocStruct)
{
    COcStruct* newOcStruct=ocStruct->copyYourself();
    return(newOcStruct);
}

void geom_scaleOctree(COcStruct* ocStruct,float f)
{
    ocStruct->scaleYourself(f);
}

void geom_destroyOctree(COcStruct* ocStruct)
{
    delete ocStruct;
}

void geom_getOctreeSerializationData(const COcStruct* ocStruct,std::vector<unsigned char>& serializationData)
{
    int s;
    unsigned char* data=ocStruct->serialize(s);
    serializationData.assign(data,data+s);
    delete[] data;
}

COcStruct* geom_getOctreeFromSerializationData(const unsigned char* serializationData)
{
    COcStruct* newOctreeStruct=new COcStruct();
    if (newOctreeStruct->deserialize(serializationData))
        return(newOctreeStruct);
    else
    {
        delete newOctreeStruct;
        return(nullptr);
    }
}

CPcStruct* geom_createPtcloudFromPoints(const float* points,int pointCnt,const C7Vector* ptcloudOrigin/*=nullptr*/,float cellS/*=0.05f*/,int maxPointCnt/*=20*/,const unsigned char rgbData[3]/*=nullptr*/,float proximityTol/*=0.005f*/)
{
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv;
    trInv.setIdentity();
    if (ptcloudOrigin!=nullptr)
        trInv=ptcloudOrigin->getInverse();
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    unsigned char _rgbData[3];
    if (rgbData!=nullptr)
    {
        _rgbData[0]=rgbData[0];
        _rgbData[1]=rgbData[1];
        _rgbData[2]=rgbData[2];
    }
    CPcStruct* pcStruct=new CPcStruct(cellS,maxPointCnt,&relPts[0],(size_t)pointCnt,_rgbData,false,proximityTol);
    return(pcStruct);
}

CPcStruct* geom_createPtcloudFromColorPoints(const float* points,int pointCnt,const C7Vector* ptcloudOrigin/*=nullptr*/,float cellS/*=0.05f*/,int maxPointCnt/*=20*/,const unsigned char* rgbData/*=nullptr*/,float proximityTol/*=0.005f*/)
{
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv;
    trInv.setIdentity();
    if (ptcloudOrigin!=nullptr)
        trInv=ptcloudOrigin->getInverse();
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    std::vector<unsigned char> _rgbData;
    _rgbData.resize(pointCnt*3,0);
    if (rgbData!=nullptr)
    {
        for (int i=0;i<pointCnt*3;i++)
            _rgbData[i]=rgbData[i];
    }
    CPcStruct* pcStruct=new CPcStruct(cellS,maxPointCnt,&relPts[0],pointCnt,&_rgbData[0],true,proximityTol);
    return(pcStruct);
}

CPcStruct* geom_copyPtcloud(const CPcStruct* pcStruct)
{
    CPcStruct* newPcStruct=pcStruct->copyYourself();
    return(newPcStruct);
}

void geom_scalePtcloud(CPcStruct* pcStruct,float f)
{
    pcStruct->scaleYourself(f);
}

void geom_getPtcloudSerializationData(const CPcStruct* pcStruct,std::vector<unsigned char>& serializationData)
{
    int s;
    unsigned char* data=pcStruct->serialize(s);
    serializationData.assign(data,data+s);
    delete[] data;
}

CPcStruct* geom_getPtcloudFromSerializationData(const unsigned char* serializationData)
{
    CPcStruct* newPcStruct=new CPcStruct();
    if (newPcStruct->deserialize(serializationData))
        return(newPcStruct);
    else
    {
        delete newPcStruct;
        return(nullptr);
    }
}

void geom_destroyPtcloud(CPcStruct* pcStruct)
{
    delete pcStruct;
}

void geom_getPtcloudPoints(const CPcStruct* pcStruct,std::vector<float>& pointData,float prop/*=1.0f*/)
{
    if (prop==1.0f)
        pcStruct->getPointsPosAndRgb_all(pointData);
    else
        pcStruct->getPointsPosAndRgb_subset(pointData,prop);
}

bool geom_isPointInVolume(const float* planesIn,int planesInSize,const C3Vector& point)
{
    bool retVal=false;
    if (planesInSize>0)
    {
        CVolumePlanes _planesIn(planesIn,(size_t)planesInSize);
        retVal=CCalcUtils::isPointInVolume(_planesIn,point);
    }
    return(retVal);
}

bool geom_volumeSensorDetectMeshIfSmaller(const float* planesIn,int planesInSize,const float* planesOut,int planesOutSize,const CObbStruct* obbStruct,const C7Vector& meshTransformation,float& dist,bool fast/*=false*/,bool frontDetection/*=true*/,bool backDetection/*=true*/,float maxAngle/*=0.0f*/,C3Vector* detectPt/*=nullptr*/,C3Vector* triN/*=nullptr*/)
{ // planesOutSize can be 0. Sensor is at the origin. Mesh is relative to sensor
    CVolumePlanes _planesIn(planesIn,(size_t)planesInSize);
    CVolumePlanes _planesOut(planesOut,(size_t)planesOutSize);
    float cosAngle=2.0f; // means angle not taken into account
    if ( (maxAngle>0.0f)&&(maxAngle<=piValD2_f+0.001f) )
        cosAngle=cos(maxAngle);
    bool retVal=obbStruct->obb->checkSensorDistance_obb(obbStruct,meshTransformation.getMatrix(),_planesIn,_planesOut,cosAngle,frontDetection,backDetection,fast,dist,detectPt,triN);
    return(retVal);
}

bool geom_volumeSensorDetectOctreeIfSmaller(const float* planesIn,int planesInSize,const float* planesOut,int planesOutSize,const COcStruct* ocStruct,const C7Vector& octreeTransformation,float& dist,bool fast/*=false*/,bool frontDetection/*=true*/,bool backDetection/*=true*/,float maxAngle/*=0.0f*/,C3Vector* detectPt/*=nullptr*/,C3Vector* triN/*=nullptr*/)
{ // planesOutSize can be 0. Sensor is at the origin. Octree is relative to sensor
    CVolumePlanes _planesIn(planesIn,(size_t)planesInSize);
    CVolumePlanes _planesOut(planesOut,(size_t)planesOutSize);
    float cosAngle=2.0f; // means angle not taken into account
    if ( (maxAngle>0.0f)&&(maxAngle<=piValD2_f+0.001f) )
        cosAngle=cos(maxAngle);
    bool retVal=ocStruct->getSensorDistance(octreeTransformation.getMatrix(),_planesIn,_planesOut,cosAngle,frontDetection,backDetection,fast,dist,detectPt,triN);
    return(retVal);
}

bool geom_volumeSensorDetectPtcloudIfSmaller(const float* planesIn,int planesInSize,const float* planesOut,int planesOutSize,const CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,float& dist,bool fast/*=false*/,C3Vector* detectPt/*=nullptr*/)
{ // planesOutSize can be 0. Sensor is at the origin. PtCloud is relative to sensor
    CVolumePlanes _planesIn(planesIn,(size_t)planesInSize);
    CVolumePlanes _planesOut(planesOut,(size_t)planesOutSize);
    bool retVal=pcStruct->getSensorDistance(ptcloudTransformation.getMatrix(),_planesIn,_planesOut,fast,dist,detectPt);
    return(retVal);
}

bool geom_volumeSensorDetectTriangleIfSmaller(const float* planesIn,int planesInSize,const float* planesOut,int planesOutSize,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,bool frontDetection/*=true*/,bool backDetection/*=true*/,float maxAngle/*=0.0f*/,C3Vector* detectPt/*=nullptr*/,C3Vector* triN/*=nullptr*/)
{ // planesOutSize can be 0. Sensor is at the origin. Triangle is relative to sensor
    CVolumePlanes _planesIn(planesIn,(size_t)planesInSize);
    CVolumePlanes _planesOut(planesOut,(size_t)planesOutSize);
    float cosAngle=2.0f; // means angle not taken into account
    if ( (maxAngle>0.0f)&&(maxAngle<=piValD2_f+0.001f) )
        cosAngle=cos(maxAngle);
    bool retVal=CCalcUtils::getSensorDistance_tri(_planesIn,_planesOut,cosAngle,frontDetection,backDetection,p,v,w,dist,detectPt,triN);
    return(retVal);
}

bool geom_volumeSensorDetectSegmentIfSmaller(const float* planesIn,int planesInSize,const float* planesOut,int planesOutSize,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,float& dist,float maxAngle/*=0.0f*/,C3Vector* detectPt/*=nullptr*/)
{ // planesOutSize can be 0. Sensor is at the origin. Segment is relative to sensor
    CVolumePlanes _planesIn(planesIn,(size_t)planesInSize);
    CVolumePlanes _planesOut(planesOut,(size_t)planesOutSize);
    float cosAngle=2.0f; // means angle not taken into account
    if ( (maxAngle>0.0f)&&(maxAngle<=piValD2_f+0.001f) )
        cosAngle=cos(maxAngle);
    bool retVal=CCalcUtils::getSensorDistance_segp(_planesIn,_planesOut,cosAngle,segmentEndPoint,segmentVector,dist,detectPt);
    return(retVal);
}

bool geom_raySensorDetectMeshIfSmaller(const C3Vector& rayStart,const C3Vector& rayVect,const CObbStruct* obbStruct,const C7Vector& meshTransformation,float& dist,float forbiddenDist/*=0.0f*/,bool fast/*=false*/,bool frontDetection/*=true*/,bool backDetection/*=true*/,float maxAngle/*=0.0f*/,C3Vector* detectPt/*=nullptr*/,C3Vector* triN/*=nullptr*/,bool* forbiddenDistTouched/*=nullptr*/)
{ // Sensor is at the origin. Mesh is relative to sensor
    float cosAngle=2.0f; // means angle not taken into account
    if ( (maxAngle>0.0f)&&(maxAngle<=piValD2_f+0.001f) )
        cosAngle=cos(maxAngle);
    bool retVal=obbStruct->obb->checkRaySensorDistance_obb(obbStruct,meshTransformation.getMatrix(),rayStart,rayVect,cosAngle,frontDetection,backDetection,forbiddenDist,fast,dist,detectPt,triN,forbiddenDistTouched);
    return(retVal);
}

bool geom_raySensorDetectOctreeIfSmaller(const C3Vector& rayStart,const C3Vector& rayVect,const COcStruct* ocStruct,const C7Vector& octreeTransformation,float& dist,float forbiddenDist/*=0.0f*/,bool fast/*=false*/,bool frontDetection/*=true*/,bool backDetection/*=true*/,float maxAngle/*=0.0f*/,C3Vector* detectPt/*=nullptr*/,C3Vector* triN/*=nullptr*/,bool* forbiddenDistTouched/*=nullptr*/)
{ // Sensor is at the origin. octree is relative to sensor
    float cosAngle=2.0f; // means angle not taken into account
    if ( (maxAngle>0.0f)&&(maxAngle<=piValD2_f+0.001f) )
        cosAngle=cos(maxAngle);
    bool retVal=ocStruct->getRaySensorDistance(octreeTransformation.getMatrix(),rayStart,rayVect,forbiddenDist,cosAngle,frontDetection,backDetection,fast,dist,detectPt,triN,forbiddenDistTouched);
    return(retVal);
}

void geom_insertPointsIntoPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const float* points,int pointCnt,const unsigned char rgbData[3]/*=nullptr*/,float proximityTol/*=0.001*/)
{
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv(ptcloudTransformation.getInverse());
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    unsigned char _rgbData[3];
    if (rgbData!=nullptr)
    {
        _rgbData[0]=rgbData[0];
        _rgbData[1]=rgbData[1];
        _rgbData[2]=rgbData[2];
    }
    pcStruct->add_pts(&relPts[0],pointCnt,_rgbData,false,proximityTol);
}

void geom_insertColorPointsIntoPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const float* points,int pointCnt,const unsigned char* rgbData/*=nullptr*/,float proximityTol/*=0.001*/)
{
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv(ptcloudTransformation.getInverse());
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    std::vector<unsigned char> _rgbData;
    _rgbData.resize(pointCnt*3,0);
    if (rgbData!=nullptr)
    {
        for (int i=0;i<pointCnt*3;i++)
            _rgbData[i]=rgbData[i];
    }
    pcStruct->add_pts(&relPts[0],pointCnt,&_rgbData[0],true,proximityTol);
}

bool geom_removePointsFromPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const float* points,int pointCnt,float proximityTol/*=0.001*/,int* countRemoved/*=nullptr*/)
{ // returns true if the pt cloud is empty after the operation
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv(ptcloudTransformation.getInverse());
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    bool retVal=pcStruct->delete_pts(&relPts[0],(size_t)pointCnt,proximityTol,countRemoved);
    return(retVal);
}

bool geom_removeOctreeFromPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const COcStruct* ocStruct,const C7Vector& octreeTransformation,int* countRemoved/*=nullptr*/)
{ // returns true if the pt cloud is empty after the operation
   bool retVal=pcStruct->delete_octree(ptcloudTransformation.getMatrix(),ocStruct,octreeTransformation.getMatrix(),countRemoved);
   return(retVal);
}

bool geom_intersectPointsWithPtcloud(CPcStruct* pcStruct,const C7Vector& ptcloudTransformation,const float* points,int pointCnt,float proximityTol/*=0.001*/)
{ // returns true if the pt cloud is empty after the operation
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv(ptcloudTransformation.getInverse());
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    bool retVal=pcStruct->intersect_pts(&relPts[0],(size_t)pointCnt,proximityTol);
    return(retVal);
}

void geom_getPtcloudOctreeCorners(const CPcStruct* pcStruct,std::vector<float>& points)
{
    pcStruct->getOctreeCorners(points);
}

int geom_getPtcloudNonEmptyCellCount(const CPcStruct* pcStruct)
{
    return(int(pcStruct->countCellsWithContent()));
}

void geom_insertPointsIntoOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const float* points,int pointCnt,const unsigned char rgbData[3]/*=nullptr*/,unsigned int usrData/*=0*/)
{
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv(octreeTransformation.getInverse());
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    unsigned char _rgbData[3];
    if (rgbData!=nullptr)
    {
        _rgbData[0]=rgbData[0];
        _rgbData[1]=rgbData[1];
        _rgbData[2]=rgbData[2];
    }
    ocStruct->add_pts(&relPts[0],(size_t)pointCnt,_rgbData,&usrData,false);
}

void geom_insertColorPointsIntoOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const float* points,int pointCnt,const unsigned char* rgbData/*=nullptr*/,const unsigned int* usrData/*=nullptr*/)
{
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv(octreeTransformation.getInverse());
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    std::vector<unsigned char> _rgbData;
    _rgbData.resize(pointCnt*3,0);
    if (rgbData!=nullptr)
    {
        for (int i=0;i<pointCnt*3;i++)
            _rgbData[i]=rgbData[i];
    }
    std::vector<unsigned int> _usrData;
    _usrData.resize(pointCnt,0);
    if (usrData!=nullptr)
    {
        for (int i=0;i<pointCnt;i++)
            _usrData[i]=usrData[i];
    }
    ocStruct->add_pts(&relPts[0],(size_t)pointCnt,&_rgbData[0],&_usrData[0],true);
}

void geom_insertMeshIntoOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const CObbStruct* obbStruct,const C7Vector& meshTransformation,const unsigned char rgbData[3]/*=nullptr*/,unsigned int usrData/*=0*/)
{
    unsigned char _rgbData[3];
    if (rgbData!=nullptr)
    {
        _rgbData[0]=rgbData[0];
        _rgbData[1]=rgbData[1];
        _rgbData[2]=rgbData[2];
    }
    ocStruct->add_shape(octreeTransformation.getMatrix(),obbStruct,meshTransformation.getMatrix(),_rgbData,usrData);
}

void geom_insertOctreeIntoOctree(COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation,const unsigned char rgbData[3]/*=nullptr*/,unsigned int usrData/*=0*/)
{
    unsigned char _rgbData[3];
    if (rgbData!=nullptr)
    {
        _rgbData[0]=rgbData[0];
        _rgbData[1]=rgbData[1];
        _rgbData[2]=rgbData[2];
    }
    oc1Struct->add_octree(octree1Transformation.getMatrix(),oc2Struct,octree2Transformation.getMatrix(),_rgbData,usrData);
}

bool geom_removePointsFromOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const float* points,int pointCnt)
{ // returns true if the oc tree is empty after the operation
    std::vector<float> relPts;
    relPts.resize(size_t(pointCnt)*3);
    C7Vector trInv(octreeTransformation.getInverse());
    for (size_t i=0;i<size_t(pointCnt);i++)
    {
        C3Vector v(points+3*i);
        v*=trInv;
        relPts[3*i+0]=v(0);
        relPts[3*i+1]=v(1);
        relPts[3*i+2]=v(2);
    }
    bool retVal=ocStruct->deleteVoxels_pts(&relPts[0],pointCnt);
    return(retVal);
}

bool geom_removeMeshFromOctree(COcStruct* ocStruct,const C7Vector& octreeTransformation,const CObbStruct* obbStruct,const C7Vector& meshTransformation)
{ // returns true if the oc tree is empty after the operation
    bool retVal=ocStruct->deleteVoxels_shape(octreeTransformation.getMatrix(),obbStruct,meshTransformation.getMatrix());
    return(retVal);
}

bool geom_removeOctreeFromOctree(COcStruct* oc1Struct,const C7Vector& octree1Transformation,const COcStruct* oc2Struct,const C7Vector& octree2Transformation)
{ // returns true if the oc tree is empty after the operation
    bool retVal=oc1Struct->deleteVoxels_octree(octree1Transformation.getMatrix(),oc2Struct,octree2Transformation.getMatrix());
    return(retVal);
}

void geom_getOctreeCornersFromOctree(const COcStruct* ocStruct,std::vector<float>& points)
{
    ocStruct->getOctreeCorners(points);
}

