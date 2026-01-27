#include "calcUtils.h"
#include "ocStruct.h"
#include "pcStruct.h"

COcStruct::COcStruct()
{
    nextId = OCT_MAX_ID; // triggers a full data retrieval next time data is fetched
    allIds.resize(OCT_MAX_ID, false);
    ocNode=new COcNode();
}

COcStruct::COcStruct(double cellS,const double* points,size_t pointCnt,const unsigned char* rgbData,const unsigned int* usrData,bool dataForEachPt)
{
    // Create an OC tree from points
    nextId = OCT_MAX_ID; // triggers a full data retrieval next time data is fetched
    allIds.resize(OCT_MAX_ID, false);
    ocNode=new COcNode();
    _create(cellS,points,pointCnt,rgbData,usrData,dataForEachPt);
}

COcStruct::COcStruct(const C4X4Matrix& ocM,double cellS,const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const unsigned char* rgbData,unsigned int usrData)
{
    // Create an OC tree from a shape
    nextId = OCT_MAX_ID; // triggers a full data retrieval next time data is fetched
    allIds.resize(OCT_MAX_ID, false);
    ocNode=new COcNode();
    C4X4Matrix tr(ocM.getInverse()*shapeM);
    std::vector<double> points;
    for (size_t i=0;i<obbStruct->vertices.size()/3;i++)
    {
        C3Vector v(&obbStruct->vertices[3*i]);
        v*=tr;
        points.push_back(v(0));
        points.push_back(v(1));
        points.push_back(v(2));
    }
    _create(cellS,&points[0],points.size()/3,rgbData,&usrData,false);
    add_shape(ocM,obbStruct,shapeM,rgbData,usrData);
}

COcStruct::COcStruct(const C4X4Matrix& oc1M,double cell1S,const COcStruct* oc2Struct,const C4X4Matrix& oc2M,const unsigned char* rgbData,unsigned int usrData)
{
    // Create an OC tree from another OC tree
    nextId = OCT_MAX_ID; // triggers a full data retrieval next time data is fetched
    allIds.resize(OCT_MAX_ID, false);
    ocNode=new COcNode();
    C4X4Matrix tr(oc1M.getInverse()*oc2M);
    std::vector<double> points;
    oc2Struct->getVoxelsCorners(points);
    for (size_t i=0;i<points.size()/3;i++)
    {
        C3Vector v(&points[3*i]);
        v*=tr;
        points[3*i+0]=v(0);
        points[3*i+1]=v(1);
        points[3*i+2]=v(2);
    }
    _create(cell1S,&points[0],points.size()/3,rgbData,&usrData,false);
    add_octree(oc1M,oc2Struct,oc2M,rgbData,usrData);
}

void COcStruct::_create(double cellS,const double* points,size_t pointCnt,const unsigned char* rgbData,const unsigned int* usrData,bool dataForEachPt)
{
    // Create an OC tree from points
    cellSize=cellS;
    double ms=0.0;
    for (size_t i=0;i<pointCnt;i++)
    {
        C3Vector v(points+3*i);
        for (size_t j=0;j<3;j++)
        {
            if (fabs(v(j))>ms)
                ms=fabs(v(j));
        }
    }
    boxSize=ms*double(2.001);
    double c=cellS*2.0;
    while (c<boxSize)
        c*=2.0;
    boxSize=c;
    boxPos.clear();
    std::vector<double> pts;
    std::vector<unsigned char> rgbs;
    std::vector<unsigned int> usrs;
    if (!dataForEachPt)
    {
        rgbs.push_back(rgbData[0]);
        rgbs.push_back(rgbData[1]);
        rgbs.push_back(rgbData[2]);
        usrs.push_back(usrData[0]);
    }
    for (size_t i=0;i<pointCnt;i++)
    {
        pts.push_back(points[3*i+0]-boxPos(0));
        pts.push_back(points[3*i+1]-boxPos(1));
        pts.push_back(points[3*i+2]-boxPos(2));
        if (dataForEachPt)
        {
            rgbs.push_back(rgbData[3*i+0]);
            rgbs.push_back(rgbData[3*i+1]);
            rgbs.push_back(rgbData[3*i+2]);
            usrs.push_back(usrData[i]);
        }
    }
    ocNode->ocNodes=new COcNode* [8];
    for (size_t i=0;i<8;i++)
         ocNode->ocNodes[i]=new COcNode(this, boxSize*0.5,ocNodeTranslations[i]*boxSize,cellS,pts,rgbs,usrs,dataForEachPt);
}

COcStruct::~COcStruct()
{
    delete ocNode;
}

COcStruct* COcStruct::copyYourself() const
{
    COcStruct* newOcStruct=new COcStruct();
    newOcStruct->cellSize=cellSize;
    newOcStruct->boxSize=boxSize;
    newOcStruct->boxPos=boxPos;
    newOcStruct->ocNode=ocNode->copyYourself();
    return(newOcStruct);
}

void COcStruct::scaleYourself(double f)
{
    cellSize*=f;
    boxSize*=f;
    boxPos*=f;
}

unsigned char* COcStruct::serialize(int& dataSize) const
{
    std::vector<unsigned char> data;
    data.push_back(2); // ser ver
    pushData(data,&boxSize,sizeof(double));
    pushData(data,&cellSize,sizeof(double));
    pushData(data,&boxPos(0),sizeof(double));
    pushData(data,&boxPos(1),sizeof(double));
    pushData(data,&boxPos(2),sizeof(double));
    for (size_t i=0;i<8;i++)
        ocNode->ocNodes[i]->serialize(data);
    unsigned char* retVal=new unsigned char[data.size()];
    for (size_t i=0;i<data.size();i++)
        retVal[i]=data[i];
    dataSize=int(data.size());
    return(retVal);
}

bool COcStruct::deserialize(const unsigned char* data)
{
    int pos=0;
    unsigned char ver=data[pos++];
    if (ver<=2)
    {
        boxSize=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
        cellSize=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
        boxPos(0)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
        boxPos(1)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
        boxPos(2)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
        ocNode->ocNodes=new COcNode* [8];
        for (size_t i=0;i<8;i++)
        {
            ocNode->ocNodes[i]=new COcNode();
            ocNode->ocNodes[i]->deserialize(this, data,pos);
        }
        return(true);
    }
    return(false);
}

unsigned char* COcStruct::serializeOld(int& dataSize) const
{
    std::vector<unsigned char> data;
    data.push_back(2); // ser ver
    float a;
    a=(float)boxSize;
    pushData(data,&a,sizeof(float));
    a=(float)cellSize;
    pushData(data,&a,sizeof(float));
    a=(float)boxPos(0);
    pushData(data,&a,sizeof(float));
    a=(float)boxPos(1);
    pushData(data,&a,sizeof(float));
    a=(float)boxPos(2);
    pushData(data,&a,sizeof(float));
    for (size_t i=0;i<8;i++)
        ocNode->ocNodes[i]->serialize(data);
    unsigned char* retVal=new unsigned char[data.size()];
    for (size_t i=0;i<data.size();i++)
        retVal[i]=data[i];
    dataSize=int(data.size());
    return(retVal);
}

bool COcStruct::deserializeOld(const unsigned char* data)
{
    int pos=0;
    unsigned char ver=data[pos++];
    if (ver<=2)
    {
        boxSize=double((reinterpret_cast<const float*>(data+pos))[0]);pos+=sizeof(float);
        cellSize=double((reinterpret_cast<const float*>(data+pos))[0]);pos+=sizeof(float);
        boxPos(0)=double((reinterpret_cast<const float*>(data+pos))[0]);pos+=sizeof(float);
        boxPos(1)=double((reinterpret_cast<const float*>(data+pos))[0]);pos+=sizeof(float);
        boxPos(2)=double((reinterpret_cast<const float*>(data+pos))[0]);pos+=sizeof(float);
        ocNode->ocNodes=new COcNode* [8];
        for (size_t i=0;i<8;i++)
        {
            ocNode->ocNodes[i]=new COcNode();
            ocNode->ocNodes[i]->deserialize(this, data,pos);
        }
        return(true);
    }
    return(false);
}

void COcStruct::refreshDisplayData()
{
    nextId = OCT_MAX_ID;
}

bool COcStruct::getDisplayVoxelsColorsAndIds(std::vector<float>& thePts,std::vector<unsigned char>& theRgbs,std::vector<unsigned int>& theIds)
{
    bool retVal = false;
    if (nextId >= OCT_MAX_ID)
    {
        retVal = true;
        nextId = 1;
        allIds.clear();
        allIds.resize(OCT_MAX_ID, false);
        for (size_t i=0;i<8;i++)
            ocNode->ocNodes[i]->resetAllIds(this);
        removedIds.clear();
    }
    for (size_t i=0;i<8;i++)
        ocNode->ocNodes[i]->getDisplayVoxelsColorsAndIds(this, boxSize, boxPos + ocNodeTranslations[i] * boxSize, thePts, theRgbs, theIds);
    return retVal;
}

void COcStruct::getVoxelsPosAndRgb(std::vector<double>& voxelsPosAndRgb,std::vector<unsigned int>* userData/*=nullptr*/) const
{
    for (size_t i=0;i<8;i++)
        ocNode->ocNodes[i]->getVoxelsPosAndRgb(voxelsPosAndRgb,boxSize,boxPos+ocNodeTranslations[i]*boxSize,userData);
}

void COcStruct::getVoxelsCorners(std::vector<double>& points) const
{
    for (size_t i=0;i<8;i++)
        ocNode->ocNodes[i]->getVoxelsCorners(points,boxSize,boxPos+ocNodeTranslations[i]*boxSize);
}

void COcStruct::getOctreeCorners(std::vector<double>& points) const
{
    for (size_t i=0;i<8;i++)
        ocNode->ocNodes[i]->getOctreeCorners(points,boxSize,boxPos+ocNodeTranslations[i]*boxSize);
}

C3Vector COcStruct::getBoxPos() const
{
    return(boxPos);
}

bool COcStruct::deleteVoxels_pts(const double* points,size_t pointCnt)
{
    // Deletes voxels occupied by provided points
    bool retVal=true;
    std::vector<double> pts;
    for (size_t i=0;i<pointCnt;i++)
    {
        pts.push_back(points[3*i+0]-boxPos(0));
        pts.push_back(points[3*i+1]-boxPos(1));
        pts.push_back(points[3*i+2]-boxPos(2));
    }
    for (size_t i=0;i<8;i++)
    {
        bool bb=ocNode->ocNodes[i]->deleteVoxels_pts(this, boxSize*0.5,ocNodeTranslations[i]*boxSize,pts);
        retVal=retVal&&bb;
    }
    return(retVal); // true: this OC tree is empty
}

bool COcStruct::deleteVoxels_shape(const C4X4Matrix& ocM,const CObbStruct* obbStruct,const C4X4Matrix& shapeM)
{
    // Deletes voxels occupied by provided shape
    bool retVal=true;
    for (size_t i=0;i<8;i++)
    {
        C4X4Matrix _ocM(ocM);
        _ocM.X+=ocM.M*boxPos;
        bool bb=ocNode->ocNodes[i]->deleteVoxels_shape(this, _ocM,boxSize*0.5,ocNodeTranslations[i]*boxSize,obbStruct,obbStruct->obb,shapeM);
        retVal=retVal&&bb;
    }
    return(retVal); // true: this OC tree is empty
}

bool COcStruct::deleteVoxels_octree(const C4X4Matrix& oc1M,const COcStruct* oc2Struct,const C4X4Matrix& oc2M)
{
    // Deletes voxels occupied by provided OC tree
    bool retVal=true;
    C4X4Matrix _oc1M(oc1M);
    _oc1M.X+=oc1M.M*boxPos;
    C4X4Matrix _oc2M(oc2M);
    _oc2M.X+=oc2M.M*oc2Struct->boxPos;
    for (size_t i=0;i<8;i++)
    {
        bool bb=ocNode->ocNodes[i]->deleteVoxels_octree(this, _oc1M,boxSize*0.5,ocNodeTranslations[i]*boxSize,oc2Struct->ocNode,_oc2M,oc2Struct->boxSize,C3Vector::zeroVector);
        retVal=retVal&&bb;
    }
    return(retVal); // true: this OC tree is empty
}


void COcStruct::add_pts(const double* points,size_t pointCnt,const unsigned char* rgbData,const unsigned int* usrData,bool dataForEachPt)
{
    // Adds points to an OC tree (extends it if needed)
    _extendOctreeIfNeeded(points,pointCnt);
    // Now add the points:
    std::vector<double> pts;
    std::vector<unsigned char> rgbs;
    std::vector<unsigned int> usrs;
    for (size_t i=0;i<pointCnt;i++)
    {
        pts.push_back(points[3*i+0]-boxPos(0));
        pts.push_back(points[3*i+1]-boxPos(1));
        pts.push_back(points[3*i+2]-boxPos(2));
        if (dataForEachPt)
        {
            rgbs.push_back(rgbData[3*i+0]);
            rgbs.push_back(rgbData[3*i+1]);
            rgbs.push_back(rgbData[3*i+2]);
            usrs.push_back(usrData[i]);
        }
    }
    if (!dataForEachPt)
    {
        rgbs.push_back(rgbData[0]);
        rgbs.push_back(rgbData[1]);
        rgbs.push_back(rgbData[2]);
        usrs.push_back(usrData[0]);
    }
    for (size_t i=0;i<8;i++)
        ocNode->ocNodes[i]->add_pts(this, cellSize,boxSize*0.5,ocNodeTranslations[i]*boxSize,pts,rgbs,usrs,dataForEachPt);
}

void COcStruct::add_shape(const C4X4Matrix& ocM,const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const unsigned char* rgbData,unsigned int usrData)
{
    // First use the shape vertices to possibly extend the octree:
    C4X4Matrix tr(ocM.getInverse()*shapeM);
    std::vector<double> pts;
    for (size_t i=0;i<obbStruct->vertices.size()/3;i++)
    {
        C3Vector v(&obbStruct->vertices[3*i]);
        v*=tr;
        pts.push_back(v(0));
        pts.push_back(v(1));
        pts.push_back(v(2));
    }
    _extendOctreeIfNeeded(&pts[0],pts.size()/3);
    // Now add the shape:
    for (size_t i=0;i<8;i++)
    {
        C4X4Matrix m(ocM);
        m.X+=ocM.M*boxPos;
        ocNode->ocNodes[i]->add_shape(this, m,cellSize,boxSize*0.5,ocNodeTranslations[i]*boxSize,obbStruct,obbStruct->obb,shapeM,rgbData,usrData);
    }
}

void COcStruct::add_octree(const C4X4Matrix& oc1M,const COcStruct* oc2Struct,const C4X4Matrix& oc2M,const unsigned char* rgbData,unsigned int usrData)
{
    // First use the octree's corner vertices to possibly extend the octree:
    C4X4Matrix tr(oc1M.getInverse()*oc2M);
    std::vector<double> pts;
    oc2Struct->getVoxelsCorners(pts);
    for (size_t i=0;i<pts.size()/3;i++)
    {
        C3Vector v(&pts[3*i]);
        v*=tr;
        pts[3*i+0]=v(0);
        pts[3*i+1]=v(1);
        pts[3*i+2]=v(2);
    }
    _extendOctreeIfNeeded(&pts[0],pts.size()/3);
    // Now add the octree:
    C4X4Matrix m1(oc1M);
    m1.X+=oc1M.M*boxPos;
    C4X4Matrix m2(oc2M);
    m2.X+=oc2M.M*oc2Struct->boxPos;
    for (size_t i=0;i<8;i++)
    {
        for (size_t j=0;j<8;j++)
            ocNode->ocNodes[i]->add_octree(this, m1,cellSize,boxSize*0.5,ocNodeTranslations[i]*boxSize,oc2Struct->ocNode->ocNodes[j],m2,oc2Struct->boxSize*0.5,ocNodeTranslations[j]*oc2Struct->boxSize,rgbData,usrData);
    }
}

void COcStruct::_extendOctreeIfNeeded(const double* points,size_t pointCnt)
{
    // Get bb around points:
    C3Vector maxV,minV;
    for (size_t i=0;i<pointCnt;i++)
    {
        C3Vector v(points+3*i);
        if (i==0)
        {
            minV=v;
            maxV=v;
        }
        else
        {
            minV.keepMin(v);
            maxV.keepMax(v);
        }
    }

    while (true)
    {
        // Do some points lie outside?
        bool ptsOutside=false;
        int dir[3]={1,1,1};
        for (size_t i=0;i<3;i++)
        {
            if (maxV(i)>=boxPos(i)+boxSize*0.5)
                ptsOutside=true;
            if (minV(i)<=boxPos(i)-boxSize*0.5)
            {
                ptsOutside=true;
                dir[i]=-1;
            }
        }

        if (!ptsOutside)
            break;

        // we extend the octree by 7 boxes:
        int parentBoxIndex=0;
        const int boxIndexToBecomeParent[8]={0,1,2,3,4,5,6,7};
        const int extD[24]={1,1,1, -1,1,1, 1,-1,1, -1,-1,1, 1,1,-1, -1,1,-1, 1,-1,-1, -1,-1,-1};
        for (size_t i=0;i<8;i++)
        {
            if ( (dir[0]==extD[3*i+0])&&(dir[1]==extD[3*i+1])&&(dir[2]==extD[3*i+2]) )
            {
                parentBoxIndex=boxIndexToBecomeParent[i];
                break;
            }
        }

        boxPos+=C3Vector(double(dir[0]),double(dir[1]),double(dir[2]))*boxSize*0.5;
        boxSize*=2.0;
        COcNode* newParentNode=new COcNode();
        newParentNode->ocNodes=new COcNode* [8];
        for (int i=0;i<8;i++)
        {
            if (i==parentBoxIndex)
                newParentNode->ocNodes[i]=ocNode;
            else
                newParentNode->ocNodes[i]=new COcNode();
        }
        ocNode=newParentNode;
    }
}

bool COcStruct::doCollide_pt(const C3Vector& point,unsigned int* usrData,unsigned long long int* ocCaching) const
{
    for (size_t i=0;i<8;i++)
    {
        if (ocNode->ocNodes[i]->doCollide_pt(point-ocNodeTranslations[i]*boxSize,boxSize*0.5,(i<<6)|1,usrData,ocCaching))
            return(true);
    }
    return(false);
}

bool COcStruct::doCollide_pts(const std::vector<double>& points) const
{
    for (size_t i=0;i<8;i++)
    {
        if (ocNode->ocNodes[i]->doCollide_pts(points,ocNodeTranslations[i]*boxSize,boxSize*0.5))
            return(true);
    }
    return(false);
}

bool COcStruct::doCollide_seg(const C3Vector& segMiddle,const C3Vector& segHs,unsigned int* usrData,unsigned long long int* ocCaching) const
{
    for (size_t i=0;i<8;i++)
    {
        if (ocNode->ocNodes[i]->doCollide_seg(segMiddle-ocNodeTranslations[i]*boxSize,segHs,boxSize*0.5,(i<<6)|1,usrData,ocCaching))
            return(true);
    }
    return(false);
}

bool COcStruct::doCollide_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,unsigned int* usrData,unsigned long long int* ocCaching) const
{
    for (size_t i=0;i<8;i++)
    {
        if (ocNode->ocNodes[i]->doCollide_tri(p-ocNodeTranslations[i]*boxSize,v,w,boxSize*0.5,(i<<6)|1,usrData,ocCaching))
            return(true);
    }
    return(false);
}

bool COcStruct::doCollide_shape(const C4X4Matrix& ocM,const CObbStruct* obbStruct,const C4X4Matrix& shapeM,unsigned long long int* ocCaching,int* meshCaching) const
{
    for (size_t i=0;i<8;i++)
    {
        unsigned long long int ocCacheValueThere=(i<<6)|(1);
        C4X4Matrix m(ocM);
        m.X+=ocM.M*boxPos;
        if (ocNode->ocNodes[i]->doCollide_shape(m,cellSize,boxSize*0.5,ocNodeTranslations[i]*boxSize,ocCacheValueThere,obbStruct,obbStruct->obb,shapeM,ocCaching,meshCaching))
            return(true);
    }
    return(false);
}



bool COcStruct::doCollide_octree(const C4X4Matrix& oc1M,const COcStruct* oc2Struct,const C4X4Matrix& oc2M,unsigned long long int* oc1Caching,unsigned long long int* oc2Caching) const
{
    C4X4Matrix m1(oc1M);
    m1.X+=oc1M.M*boxPos;
    C4X4Matrix m2(oc2M);
    m2.X+=oc2M.M*oc2Struct->boxPos;
    for (size_t i=0;i<8;i++)
    {
        unsigned long long int oc1CacheValueThere=(i<<6)|(1);
        for (size_t j=0;j<8;j++)
        {
            unsigned long long int oc2CacheValueThere=(j<<6)|(1);
            if (ocNode->ocNodes[i]->doCollide_octree(m1,cellSize,boxSize*0.5,ocNodeTranslations[i]*boxSize,oc1CacheValueThere,oc2Struct->ocNode->ocNodes[j],m2,oc2Struct->boxSize*0.5,ocNodeTranslations[j]*oc2Struct->boxSize,oc2CacheValueThere,oc1Caching,oc2Caching))
                return(true);
        }
    }
    return(false);
}

bool COcStruct::doCollide_ptcloud(const C4X4Matrix& ocM,const CPcStruct* pcStruct,const C4X4Matrix& pcM,unsigned long long int* ocCaching,unsigned long long int* pcCaching) const
{
    C4X4Matrix m1(ocM);
    m1.X+=ocM.M*boxPos;
    C4X4Matrix m2(pcM);
    m2.X+=pcM.M*pcStruct->boxPos;
    for (size_t i=0;i<8;i++)
    {
        unsigned long long int ocCacheValueThere=(i<<6)|(1);
        for (size_t j=0;j<8;j++)
        {
            unsigned long long int pcCacheValueThere=(j<<6)|(1);
            if (ocNode->ocNodes[i]->doCollide_ptcloud(m1,cellSize,boxSize*0.5,ocNodeTranslations[i]*boxSize,ocCacheValueThere,pcStruct->pcNode->pcNodes[j],m2,pcStruct->boxSize*0.5,ocNodeTranslations[j]*pcStruct->boxSize,pcCacheValueThere,ocCaching,pcCaching))
                return(true);
        }
    }
    return(false);
}
//*

bool COcStruct::getDistance_pt(const C3Vector& point,double& dist,C3Vector* minDistSegPt1,unsigned long long int* ocCaching) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    // Check all 8 nodes, explore close nodes first:
    std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslation nodeTranslation;
        nodeTranslation.transl=ocNodeTranslations[i]*boxSize;
        nodeTranslation.index=i;
        nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-point).getLength(),nodeTranslation));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        size_t index=nodesToExplore[i].second.index;
        bool bb=ocNode->ocNodes[index]->getDistance_pt(point,boxSize*0.5,nodesToExplore[i].second.transl,(index<<6)|1,dist,minDistSegPt1,ocCaching);
        retVal=retVal||bb;
    }
    return(retVal);
}

bool COcStruct::getDistance_seg(const C3Vector& segMiddle,const C3Vector& segHs,double& dist,C3Vector* ocMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* ocCaching) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    // Check all 8 nodes, explore close nodes first:
    std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslation nodeTranslation;
        nodeTranslation.transl=ocNodeTranslations[i]*boxSize;
        nodeTranslation.index=i;
        nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-segMiddle).getLength(),nodeTranslation));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        size_t index=nodesToExplore[i].second.index;
        bool bb=ocNode->ocNodes[index]->getDistance_seg(segMiddle,segHs,boxSize*0.5,nodesToExplore[i].second.transl,(index<<6)|1,dist,ocMinDistSegPt,segMinDistSegPt,ocCaching);
        retVal=retVal||bb;
    }
    return(retVal);
}

bool COcStruct::getDistance_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* ocMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* ocCaching) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    // Check all 8 nodes, explore close nodes first:
    std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslation nodeTranslation;
        nodeTranslation.transl=ocNodeTranslations[i]*boxSize;
        nodeTranslation.index=i;
        nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-p).getLength(),nodeTranslation));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        size_t index=nodesToExplore[i].second.index;
        bool bb=ocNode->ocNodes[index]->getDistance_tri(p,v,w,boxSize*0.5,nodesToExplore[i].second.transl,(index<<6)|1,dist,ocMinDistSegPt,triMinDistSegPt,ocCaching);
        retVal=retVal||bb;
    }
    return(retVal);
}

bool COcStruct::getDistance_shape(const C4X4Matrix& ocM,const CObbStruct* obbStruct,const C4X4Matrix& shapeM,double& dist,C3Vector* ocMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* ocCaching,int* obbCaching) const
{
    if (dist==0.0)
        return(false);
    C4X4Matrix tr(ocM);
    tr.X+=ocM.M*boxPos;
    bool retVal=ocNode->getDistance_shape(tr,boxSize,C3Vector::zeroVector,0,obbStruct,obbStruct->obb,shapeM,dist,ocMinDistSegPt,shapeMinDistSegPt,ocCaching,obbCaching);
    return(retVal);
}

bool COcStruct::getCell(const C4X4Matrix& ocM,const unsigned long long int ocCaching,double& cellS,C4X4Matrix& cellPos,unsigned int* usrData) const
{
    bool retVal=false;
    unsigned long long int cellPath=(ocCaching>>6)<<6;
    unsigned long long int cellDepth=ocCaching&63;
    if (cellDepth>0)
    {
        int index=(cellPath>>(6+(cellDepth-1)*3))&7;
        unsigned long long int _ocCaching=cellPath|(cellDepth-1);
        C3Vector totalTranslation;
        retVal=ocNode->ocNodes[index]->getCell(ocNodeTranslations[index]*boxSize,boxSize*0.5,_ocCaching,totalTranslation,usrData);
        if (retVal)
        {
            cellPos=ocM;
            cellPos.X+=ocM.M*(boxPos+totalTranslation);
            cellS=cellSize;
        }
    }
    return(retVal);
}

bool COcStruct::getDistance_octree(const C4X4Matrix& oc1M,const COcStruct* oc2Struct,const C4X4Matrix& oc2M,double& dist,C3Vector* oc1MinDistSegPt,C3Vector* oc2MinDistSegPt,unsigned long long int* oc1Caching,unsigned long long int* oc2Caching) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    C4X4Matrix m1(oc1M);
    m1.X+=oc1M.M*boxPos;
    C4X4Matrix m2(oc2M);
    m2.X+=oc2M.M*oc2Struct->boxPos;
    // First check cached values to initially possibly reduce 'dist':
    double cS1;
    C4X4Matrix m1p;
    if (getCell(oc1M,oc1Caching[0],cS1,m1p,nullptr))
    {
        double cS2;
        C4X4Matrix m2p;
        if (getCell(oc2M,oc2Caching[0],cS2,m2p,nullptr))
        {
            if (CCalcUtils::getDistance_cell_cell(m1p,cS1*0.5,m2p,cS2*0.5,true,dist,oc1MinDistSegPt,oc2MinDistSegPt))
                retVal=true;
        }
    }
    // Now check all 8*8 node pairs, explore close node pairs first:
    std::vector<std::pair<double,SNodeTranslationPair>> nodePairsToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslationPair nodeTranslationPairs;
        nodeTranslationPairs.transl1=ocNodeTranslations[i]*boxSize;
        nodeTranslationPairs.index1=i;
        for (size_t j=0;j<8;j++)
        {
            nodeTranslationPairs.transl2=ocNodeTranslations[j]*oc2Struct->boxSize;
            nodeTranslationPairs.index2=j;
            double d=((m2*nodeTranslationPairs.transl2)-(m1*nodeTranslationPairs.transl1)).getLength();
            nodePairsToExplore.push_back(std::make_pair(d,nodeTranslationPairs));
        }
    }
    std::sort(nodePairsToExplore.begin(),nodePairsToExplore.end());
    for (size_t i=0;i<nodePairsToExplore.size();i++)
    {
        C3Vector transl1(nodePairsToExplore[i].second.transl1);
        size_t index1=nodePairsToExplore[i].second.index1;
        C3Vector transl2(nodePairsToExplore[i].second.transl2);
        size_t index2=nodePairsToExplore[i].second.index2;
        bool bb=ocNode->ocNodes[index1]->getDistance_octree(m1,boxSize*0.5,transl1,(index1<<6)|1,oc2Struct->ocNode->ocNodes[index2],m2,oc2Struct->boxSize*0.5,transl2,(index2<<6)|1,dist,oc1MinDistSegPt,oc2MinDistSegPt,oc1Caching,oc2Caching);
        retVal=retVal||bb;
    }
    return(retVal);
}

bool COcStruct::getDistance_ptcloud(const C4X4Matrix& ocM,const CPcStruct* pcStruct,const C4X4Matrix& pcM,double& dist,C3Vector* ocMinDistSegPt,C3Vector* pcMinDistSegPt,unsigned long long int* ocCaching,unsigned long long int* pcCaching) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    C4X4Matrix m1(ocM);
    m1.X+=ocM.M*boxPos;
    C4X4Matrix m2(pcM);
    m2.X+=pcM.M*pcStruct->boxPos;
    // Check all 8*8 node pairs, explore close node pairs first:
    std::vector<std::pair<double,SNodeTranslationPair>> nodePairsToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslationPair nodeTranslationPairs;
        nodeTranslationPairs.transl1=ocNodeTranslations[i]*boxSize;
        nodeTranslationPairs.index1=i;
        for (size_t j=0;j<8;j++)
        {
            nodeTranslationPairs.transl2=ocNodeTranslations[j]*pcStruct->boxSize;
            nodeTranslationPairs.index2=j;
            double d=((m2*nodeTranslationPairs.transl2)-(m1*nodeTranslationPairs.transl1)).getLength();
            nodePairsToExplore.push_back(std::make_pair(d,nodeTranslationPairs));
        }
    }
    std::sort(nodePairsToExplore.begin(),nodePairsToExplore.end());
    for (size_t i=0;i<nodePairsToExplore.size();i++)
    {
        C3Vector transl1(nodePairsToExplore[i].second.transl1);
        size_t index1=nodePairsToExplore[i].second.index1;
        C3Vector transl2(nodePairsToExplore[i].second.transl2);
        size_t index2=nodePairsToExplore[i].second.index2;
        bool bb=ocNode->ocNodes[index1]->getDistance_ptcloud(m1,boxSize*0.5,transl1,(index1<<6)|1,pcStruct->pcNode->pcNodes[index2],m2,transl2,pcStruct->boxSize*0.5,(index2<<6)|1,dist,ocMinDistSegPt,pcMinDistSegPt,ocCaching,pcCaching);
        retVal=retVal||bb;
    }
    return(retVal);
}

bool COcStruct::getSensorDistance(const C4X4Matrix& ocM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,double cosAngle,bool frontDetection,bool backDetection,bool fast,double& dist,C3Vector* detectPt,C3Vector* triN) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    C4X4Matrix m(ocM);
    m.X+=ocM.M*boxPos;
    // Check all 8 nodes, explore close nodes first (close to the sensor pt):
    std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslation nodeTranslation;
        nodeTranslation.transl=ocNodeTranslations[i]*boxSize;
        nodeTranslation.index=i;
        double d=(m*nodeTranslation.transl).getLength();
        nodesToExplore.push_back(std::make_pair(d,nodeTranslation));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        C3Vector transl(nodesToExplore[i].second.transl);
        size_t index=nodesToExplore[i].second.index;
        bool bb=ocNode->ocNodes[index]->getSensorDistance(m,boxSize*0.5,transl,planesIn,planesOut,cosAngle,frontDetection,backDetection,fast,dist,detectPt,triN);
        retVal=retVal||bb;
        if (retVal&&fast)
            break;
    }
    return(retVal);
}

bool COcStruct::getRaySensorDistance(const C4X4Matrix& ocM,const C3Vector& raySegP,const C3Vector& raySegL,double forbiddenDist,double cosAngle,bool frontDetection,bool backDetection,bool fast,double& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    C4X4Matrix m(ocM);
    m.X+=ocM.M*boxPos;
    // Check all 8 nodes, explore close nodes first (close to the sensor pt):
    std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslation nodeTranslation;
        nodeTranslation.transl=ocNodeTranslations[i]*boxSize;
        nodeTranslation.index=i;
        double d=(m*nodeTranslation.transl).getLength();
        nodesToExplore.push_back(std::make_pair(d,nodeTranslation));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        C3Vector transl(nodesToExplore[i].second.transl);
        size_t index=nodesToExplore[i].second.index;
        bool bb=ocNode->ocNodes[index]->getRaySensorDistance(m,boxSize*0.5,transl,raySegP,raySegL,forbiddenDist,cosAngle,frontDetection,backDetection,fast,dist,detectPt,triN,forbiddenDistTouched);
        retVal=retVal||bb;
        if (retVal&&fast)
            break;
    }
    return(retVal);
}

void COcStruct::genId(unsigned int& id)
{
    id = nextId;
    if (nextId < OCT_MAX_ID)
        nextId++;
    allIds[id] = true;
}

void COcStruct::remId(unsigned int& id)
{
    if (id != 0)
    {
        allIds[id] = false;
        removedIds.push_back(id);
        id = 0;
    }
}

