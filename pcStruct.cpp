#include "calcUtils.h"
#include "kdNode.h"
#include "pcStruct.h"
#include "ocStruct.h"

CPcStruct::CPcStruct()
{
    pcNode=new CPcNode();
}

CPcStruct::CPcStruct(float cellS,int cellPts,const float* points,size_t pointCnt,const unsigned char* rgbData,bool rgbForEachPt,float proximityTol)
{
    // Create an PC tree from points
    pcNode=new CPcNode();
    _create(cellS,cellPts,points,pointCnt,rgbData,rgbForEachPt,proximityTol);
}

void CPcStruct::_create(float cellS,int cellPts,const float* points,size_t pointCnt,const unsigned char* rgbData,bool rgbForEachPt,float proximityTol)
{
    // Create an OC tree from points
    cellSize=cellS;
    maxPtCnt=cellPts;
    // handle distance tolerance:
    std::vector<float> _points;
    std::vector<unsigned char> _rgbData;
    if (proximityTol==0.0f)
    {
        _points.assign(points,points+pointCnt*3);
        if (rgbForEachPt)
            _rgbData.assign(rgbData,rgbData+pointCnt*3);
        else
        {
            for (size_t i=0;i<pointCnt;i++)
            {
                _rgbData.push_back(rgbData[0]);
                _rgbData.push_back(rgbData[1]);
                _rgbData.push_back(rgbData[2]);
            }
        }
    }
    else
    {
        CKdNode* allKdTreePts=CKdNode::buildKdTree(points,pointCnt,rgbData,rgbForEachPt,proximityTol);
        allKdTreePts->getPts(_points,_rgbData);
        delete allKdTreePts;
    }
    // Set box size and pos:
    C3Vector maxV(-SIM_MAX_FLOAT,-SIM_MAX_FLOAT,-SIM_MAX_FLOAT);
    C3Vector minV(SIM_MAX_FLOAT,SIM_MAX_FLOAT,SIM_MAX_FLOAT);
    for (size_t i=0;i<_points.size()/3;i++)
    {
        C3Vector v(&_points[3*i+0]);
        maxV.keepMax(v);
        minV.keepMin(v);
    }
    C3Vector dim(maxV-minV);
    float sideSize=std::max<float>(std::max<float>(dim(0),dim(1)),dim(2))*1.001f; // a tiny bit larger
    float s=cellSize*2.0f;
    while (s<sideSize)
        s*=2.0f;
    boxSize=s;
    boxPos=(maxV+minV)*0.5f;
    // Now distribute the points in the OC tree of the PC:
    std::vector<float> pts;
    std::vector<int> ptsOriginalIndices;
    for (size_t i=0;i<_points.size()/3;i++)
    {
        ptsOriginalIndices.push_back(i);
        pts.push_back(_points[3*i+0]-boxPos(0));
        pts.push_back(_points[3*i+1]-boxPos(1));
        pts.push_back(_points[3*i+2]-boxPos(2));
    }
    std::vector<bool> ptsInvalidityIndicators(pts.size()/3,false);
    pcNode->pcNodes=new CPcNode* [8];
    for (size_t i=0;i<8;i++)
        pcNode->pcNodes[i]=new CPcNode(boxSize*0.5f,ocNodeTranslations[i]*boxSize,cellSize,maxPtCnt,pts,ptsOriginalIndices,ptsInvalidityIndicators,_rgbData,true);
}

CPcStruct::~CPcStruct()
{
    delete pcNode;
}

CPcStruct* CPcStruct::copyYourself() const
{
    CPcStruct* newPcStruct=new CPcStruct();
    newPcStruct->cellSize=cellSize;
    newPcStruct->maxPtCnt=maxPtCnt;
    newPcStruct->boxSize=boxSize;
    newPcStruct->boxPos=boxPos;
    newPcStruct->pcNode=pcNode->copyYourself();
    return(newPcStruct);
}

void CPcStruct::scaleYourself(float f)
{
    pcNode->scaleYourself(f);
    cellSize*=f;
    boxSize*=f;
    boxPos*=f;
}

unsigned char* CPcStruct::serialize(int& dataSize) const
{
    std::vector<unsigned char> data;
    data.push_back(2); // ser ver
    pushData(data,&boxSize,sizeof(boxSize));
    pushData(data,&cellSize,sizeof(cellSize));
    pushData(data,&boxPos(0),sizeof(boxPos(0)));
    pushData(data,&boxPos(1),sizeof(boxPos(1)));
    pushData(data,&boxPos(2),sizeof(boxPos(2)));
    pushData(data,&maxPtCnt,sizeof(maxPtCnt));
    for (size_t i=0;i<8;i++)
        pcNode->pcNodes[i]->serialize(data);
    unsigned char* retVal=new unsigned char[data.size()];
    for (size_t i=0;i<data.size();i++)
        retVal[i]=data[i];
    dataSize=(int)data.size();
    return(retVal);
}

bool CPcStruct::deserialize(const unsigned char* data)
{
    int pos=0;
    unsigned char ver=data[pos++];
    if (ver<=2)
    {
        boxSize=((float*)(data+pos))[0];pos+=sizeof(float);
        cellSize=((float*)(data+pos))[0];pos+=sizeof(float);
        boxPos(0)=((float*)(data+pos))[0];pos+=sizeof(float);
        boxPos(1)=((float*)(data+pos))[0];pos+=sizeof(float);
        boxPos(2)=((float*)(data+pos))[0];pos+=sizeof(float);
        maxPtCnt=((int*)(data+pos))[0];pos+=sizeof(int);
        pcNode->pcNodes=new CPcNode* [8];
        for (size_t i=0;i<8;i++)
        {
            pcNode->pcNodes[i]=new CPcNode();
            pcNode->pcNodes[i]->deserialize(data,pos);
        }
        return(true);
    }
    return(false);
}

size_t CPcStruct::countCellsWithContent() const
{
    size_t retVal=0;
    for (size_t i=0;i<8;i++)
        retVal+=pcNode->pcNodes[i]->countCellsWithContent();
    return(retVal);
}

void CPcStruct::getPointsPosAndRgb_all(std::vector<float>& pointsPosAndRgb) const
{
    for (size_t i=0;i<8;i++)
        pcNode->pcNodes[i]->getPointsPosAndRgb_all(boxSize*0.5f,boxPos+ocNodeTranslations[i]*boxSize,pointsPosAndRgb);
}

void CPcStruct::getPointsPosAndRgb_subset(std::vector<float>& pointsPosAndRgb,float prop) const
{
    for (size_t i=0;i<8;i++)
        pcNode->pcNodes[i]->getPointsPosAndRgb_subset(boxSize*0.5f,boxPos+ocNodeTranslations[i]*boxSize,prop,pointsPosAndRgb);
}

void CPcStruct::getOctreeCorners(std::vector<float>& points) const
{
    for (size_t i=0;i<8;i++)
        pcNode->pcNodes[i]->getOctreeCorners(boxSize*0.5f,boxPos+ocNodeTranslations[i]*boxSize,points);
}

void CPcStruct::_extendPointCloudOctreeIfNeeded(const float* points,size_t pointCnt)
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
            if (maxV(i)>=boxPos(i)+boxSize*0.5f)
                ptsOutside=true;
            if (minV(i)<=boxPos(i)-boxSize*0.5f)
            {
                ptsOutside=true;
                dir[i]=-1;
            }
        }

        if (!ptsOutside)
            break;

        // we extend the point cloud octree struct by 7 boxes:
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

        boxPos+=C3Vector((float)dir[0],(float)dir[1],(float)dir[2])*boxSize*0.5f;
        boxSize*=2.0f;
        CPcNode* newParentNode=new CPcNode();
        newParentNode->pcNodes=new CPcNode* [8];
        for (int i=0;i<8;i++)
        {
            if (i==parentBoxIndex)
                newParentNode->pcNodes[i]=pcNode;
            else
                newParentNode->pcNodes[i]=new CPcNode();
        }
        pcNode=newParentNode;
    }
}


void CPcStruct::add_pts(const float* points,size_t pointCnt,const unsigned char* rgbData,bool dataForEachPt,float proximityTol)
{
    if (proximityTol>0.0f)
    {
        // First discard duplicates with points in the point cloud:
        std::vector<float> pts;
        std::vector<int> ptsOriginalIndices;
        std::vector<bool> duplicateIndicators(pointCnt,false);
        for (size_t i=0;i<pointCnt;i++)
        {
            ptsOriginalIndices.push_back(int(i));
            pts.push_back(points[3*i+0]-boxPos(0));
            pts.push_back(points[3*i+1]-boxPos(1));
            pts.push_back(points[3*i+2]-boxPos(2));
        }
        for (size_t i=0;i<8;i++)
            pcNode->pcNodes[i]->flagDuplicates(boxSize*0.5f,ocNodeTranslations[i]*boxSize,pts,ptsOriginalIndices,duplicateIndicators,proximityTol);

        std::vector<float> points2;
        std::vector<unsigned char> rgbData2;
        for (size_t i=0;i<pointCnt;i++)
        {
            if (!duplicateIndicators[i])
            {
                points2.push_back(points[3*i+0]);
                points2.push_back(points[3*i+1]);
                points2.push_back(points[3*i+2]);
                if (dataForEachPt)
                {
                    rgbData2.push_back(rgbData[3*i+0]);
                    rgbData2.push_back(rgbData[3*i+1]);
                    rgbData2.push_back(rgbData[3*i+2]);
                }
            }
        }
        if (!dataForEachPt)
        {
            rgbData2.push_back(rgbData[0]);
            rgbData2.push_back(rgbData[1]);
            rgbData2.push_back(rgbData[2]);
        }

        if (points2.size()>0)
        {   // now discard duplicates among the new points:
            std::vector<float> points3;
            std::vector<unsigned char> rgbs3;
            CKdNode* allKdTreePts=CKdNode::buildKdTree(&points2[0],points2.size()/3,&rgbData2[0],dataForEachPt,proximityTol);
            allKdTreePts->getPts(points3,rgbs3);
            delete allKdTreePts;
            // Finally add the pts:
            if (points3.size()>0)
                add_pts(&points3[0],points3.size()/3,&rgbs3[0],dataForEachPt,0.0f);
        }
    }
    else
    { // we do not handle duplicates (i.e. they are not discarded)
        // Extend the OC tree if needed:
        _extendPointCloudOctreeIfNeeded(points,pointCnt);
        // Now add the points:
        std::vector<float> pts;
        std::vector<unsigned char> rgbs;
        std::vector<int> ptsOriginalIndices;
        std::vector<bool> ptsInvalidityIndicators(pointCnt,false);
        for (size_t i=0;i<pointCnt;i++)
        {
            ptsOriginalIndices.push_back(int(i));
            pts.push_back(points[3*i+0]-boxPos(0));
            pts.push_back(points[3*i+1]-boxPos(1));
            pts.push_back(points[3*i+2]-boxPos(2));
            if (dataForEachPt)
            {
                rgbs.push_back(rgbData[3*i+0]);
                rgbs.push_back(rgbData[3*i+1]);
                rgbs.push_back(rgbData[3*i+2]);
            }
        }
        if (!dataForEachPt)
        {
            rgbs.push_back(rgbData[0]);
            rgbs.push_back(rgbData[1]);
            rgbs.push_back(rgbData[2]);
        }
        for (size_t i=0;i<8;i++)
            pcNode->pcNodes[i]->add_pts(boxSize*0.5f,ocNodeTranslations[i]*boxSize,cellSize,maxPtCnt,pts,ptsOriginalIndices,ptsInvalidityIndicators,rgbs,dataForEachPt);
    }
}

bool CPcStruct::delete_pts(const float* points,size_t pointCnt,float proximityTol,int* count)
{
    if (count!=nullptr)
        count[0]=0;
    std::vector<float> pts;
    for (size_t i=0;i<pointCnt;i++)
    {
        pts.push_back(points[3*i+0]-boxPos(0));
        pts.push_back(points[3*i+1]-boxPos(1));
        pts.push_back(points[3*i+2]-boxPos(2));
    }
    bool retVal=true;
    for (size_t i=0;i<8;i++)
    {
        bool bb=pcNode->pcNodes[i]->delete_pts(boxSize*0.5f,ocNodeTranslations[i]*boxSize,pts,proximityTol,count);
        retVal=retVal&&bb;
    }
    return(retVal);
}

bool CPcStruct::delete_octree(const C4X4Matrix& pcM,const COcStruct* ocStruct,const C4X4Matrix& ocM,int* count)
{
    if (count!=nullptr)
        count[0]=0;
    C4X4Matrix m1(pcM);
    m1.X+=pcM.M*boxPos;
    C4X4Matrix m2(ocM);
    m2.X+=ocM.M*ocStruct->boxPos;
    bool retVal=true;
    for (size_t i=0;i<8;i++)
    {
        bool bb=false;
        for (size_t j=0;j<8;j++)
        {
            bb=pcNode->pcNodes[i]->delete_octree(boxSize*0.5f,ocNodeTranslations[i]*boxSize,m1,ocStruct->boxSize*0.5f,ocNodeTranslations[j]*ocStruct->boxSize,ocStruct->ocNode->ocNodes[j],m2,count);
            if (bb)
                break;
        }
        retVal=retVal&&bb;
    }
    return(retVal);
}

bool CPcStruct::intersect_pts(const float* points,size_t pointCnt,float proximityTol)
{
    std::vector<float> pts;
    for (size_t i=0;i<pointCnt;i++)
    {
        pts.push_back(points[3*i+0]-boxPos(0));
        pts.push_back(points[3*i+1]-boxPos(1));
        pts.push_back(points[3*i+2]-boxPos(2));
    }
    bool retVal=true;
    for (size_t i=0;i<8;i++)
    {
        bool bb=pcNode->pcNodes[i]->intersect_pts(boxSize*0.5f,ocNodeTranslations[i]*boxSize,pts,proximityTol);
        retVal=retVal&&bb;
    }
    return(retVal);
}

bool CPcStruct::getDistance_pt(const C4X4Matrix& pcM,const C3Vector& point,float& dist,C3Vector* pcMinDistSegAbsPt,C3Vector* ptMinDistSegAbsPt,unsigned long long int* pcCaching) const
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    C4X4Matrix m(pcM);
    m.X+=pcM.M*boxPos;
    C3Vector relPoint(m.getInverse()*point);

    // Check all 8 nodes, explore close nodes first:
    std::vector<std::pair<float,SNodeTranslation>> nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslation nodeTranslation;
        nodeTranslation.transl=ocNodeTranslations[i]*boxSize;
        nodeTranslation.index=i;
        nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-relPoint).getLength(),nodeTranslation));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        int index=nodesToExplore[i].second.index;
        bool bb=pcNode->pcNodes[index]->getDistance_pt(boxSize*0.5f,nodesToExplore[i].second.transl,relPoint,dist,pcMinDistSegAbsPt,pcCaching,(index<<6)|1);
        retVal=retVal||bb;
    }
    if (retVal)
    {
        if (pcMinDistSegAbsPt!=nullptr)
            pcMinDistSegAbsPt[0]*=m;
        if (ptMinDistSegAbsPt!=nullptr)
            ptMinDistSegAbsPt[0]=point;
    }
    return(retVal);
}

bool CPcStruct::getDistance_seg(const C4X4Matrix& pcM,const C3Vector& segMiddle,const C3Vector& segHs,float& dist,C3Vector* pcMinDistSegAbsPt,C3Vector* segMinDistSegAbsPt,unsigned long long int* pcCaching) const
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    C4X4Matrix m(pcM);
    m.X+=pcM.M*boxPos;
    C4X4Matrix mi(m.getInverse());
    C3Vector relSegMiddle(mi*segMiddle);
    mi.X.clear();
    C3Vector relSegHs(mi*segHs);

    // Check all 8 nodes, explore close nodes first:
    std::vector<std::pair<float,SNodeTranslation>> nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslation nodeTranslation;
        nodeTranslation.transl=ocNodeTranslations[i]*boxSize;
        nodeTranslation.index=i;
        nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-relSegMiddle).getLength(),nodeTranslation));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        int index=nodesToExplore[i].second.index;
        bool bb=pcNode->pcNodes[index]->getDistance_seg(boxSize*0.5f,nodesToExplore[i].second.transl,relSegMiddle,relSegHs,dist,pcMinDistSegAbsPt,segMinDistSegAbsPt,pcCaching,(index<<6)|1);
        retVal=retVal||bb;
    }
    if (retVal)
    {
        if (pcMinDistSegAbsPt!=nullptr)
            pcMinDistSegAbsPt[0]*=m;
        if (segMinDistSegAbsPt!=nullptr)
            segMinDistSegAbsPt[0]*=m;
    }
    return(retVal);
}

bool CPcStruct::getDistance_tri(const C4X4Matrix& pcM,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* pcMinDistSegAbsPt,C3Vector* triMinDistSegAbsPt,unsigned long long int* pcCaching) const
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    C4X4Matrix m(pcM);
    m.X+=pcM.M*boxPos;
    C4X4Matrix mi(m.getInverse());
    C3Vector relP(mi*p);
    mi.X.clear();
    C3Vector relV(mi*v);
    C3Vector relW(mi*w);
    C3Vector relC((relP*3.0f+relV+relW)*0.3333f);

    // Check all 8 nodes, explore close nodes first:
    std::vector<std::pair<float,SNodeTranslation>> nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslation nodeTranslation;
        nodeTranslation.transl=ocNodeTranslations[i]*boxSize;
        nodeTranslation.index=i;
        nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-relC).getLength(),nodeTranslation));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        int index=nodesToExplore[i].second.index;
        bool bb=pcNode->pcNodes[index]->getDistance_tri(boxSize*0.5f,nodesToExplore[i].second.transl,relP,relV,relW,dist,pcMinDistSegAbsPt,triMinDistSegAbsPt,pcCaching,(index<<6)|1);
        retVal=retVal||bb;
    }
    if (retVal)
    {
        if (pcMinDistSegAbsPt!=nullptr)
            pcMinDistSegAbsPt[0]*=m;
        if (triMinDistSegAbsPt!=nullptr)
            triMinDistSegAbsPt[0]*=m;
    }
    return(retVal);
}

bool CPcStruct::getDistance_shape(const CObbStruct* obbStruct,const C4X4Matrix& pcM,const C4X4Matrix& shapeM,float& dist,C3Vector* pcMinDistSegAbsPt,C3Vector* shapeMinDistSegAbsPt,unsigned long long int* pcCaching,int* obbCaching) const
{
    C4X4Matrix m(pcM);
    m.X+=pcM.M*boxPos;
    bool retVal=pcNode->getDistance_shape(boxSize,C3Vector::zeroVector,m,obbStruct,obbStruct->obb,shapeM,dist,pcMinDistSegAbsPt,shapeMinDistSegAbsPt,pcCaching,0,obbCaching);
    return(retVal);
}

bool CPcStruct::getDistance_ptcloud(const CPcStruct* pc2Struct,const C4X4Matrix& pc1M,const C4X4Matrix& pc2M,float& dist,C3Vector* pc1MinDistSegAbsPt,C3Vector* pc2MinDistSegAbsPt,unsigned long long int* pc1Caching,unsigned long long int* pc2Caching) const
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    C4X4Matrix m1(pc1M);
    m1.X+=pc1M.M*boxPos;
    C4X4Matrix m2(pc2M);
    m2.X+=pc2M.M*pc2Struct->boxPos;
    // Now check all 8*8 node pairs, explore close node pairs first:
    std::vector<std::pair<float,SNodeTranslationPair>> nodePairsToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslationPair nodeTranslationPairs;
        nodeTranslationPairs.transl1=ocNodeTranslations[i]*boxSize;
        nodeTranslationPairs.index1=i;
        for (size_t j=0;j<8;j++)
        {
            nodeTranslationPairs.transl2=ocNodeTranslations[j]*pc2Struct->boxSize;
            nodeTranslationPairs.index2=j;
            float d=((m2*nodeTranslationPairs.transl2)-(m1*nodeTranslationPairs.transl1)).getLength();
            nodePairsToExplore.push_back(std::make_pair(d,nodeTranslationPairs));
        }
    }
    std::sort(nodePairsToExplore.begin(),nodePairsToExplore.end());
    for (size_t i=0;i<nodePairsToExplore.size();i++)
    {
        C3Vector transl1(nodePairsToExplore[i].second.transl1);
        int index1=nodePairsToExplore[i].second.index1;
        C3Vector transl2(nodePairsToExplore[i].second.transl2);
        int index2=nodePairsToExplore[i].second.index2;
        bool bb=pcNode->pcNodes[index1]->getDistance_ptcloud(boxSize*0.5f,transl1,m1,pc2Struct->pcNode->pcNodes[index2],pc2Struct->boxSize*0.5f,transl2,m2,dist,pc1MinDistSegAbsPt,pc2MinDistSegAbsPt,pc1Caching,(index1<<6)|1,pc2Caching,(index2<<6)|1);
        retVal=retVal||bb;
    }
    return(retVal);
}

bool CPcStruct::getSensorDistance(const C4X4Matrix& pcM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,bool fast,float& dist,C3Vector* detectPt) const
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    C4X4Matrix m(pcM);
    m.X+=pcM.M*boxPos;
    // Check all 8 nodes, explore close nodes first (close to the sensor pt):
    std::vector<std::pair<float,SNodeTranslation>> nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SNodeTranslation nodeTranslation;
        nodeTranslation.transl=ocNodeTranslations[i]*boxSize;
        nodeTranslation.index=i;
        float d=(m*nodeTranslation.transl).getLength();
        nodesToExplore.push_back(std::make_pair(d,nodeTranslation));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        C3Vector transl(nodesToExplore[i].second.transl);
        int index=nodesToExplore[i].second.index;
        bool bb=pcNode->pcNodes[index]->getSensorDistance(boxSize*0.5f,transl,m,planesIn,planesOut,fast,dist,detectPt);
        retVal=retVal||bb;
        if (retVal&&fast)
            break;
    }
    return(retVal);
}

const float* CPcStruct::getPoints(const C4X4Matrix& pcM,unsigned long long int pcCaching,size_t* ptCnt,C4X4Matrix& transf) const
{
    const float* retVal=nullptr;
    unsigned long long int cellPath=(pcCaching>>6)<<6;
    unsigned long long int cellDepth=pcCaching&63;
    if (cellDepth>0)
    {
        int index=(cellPath>>(6+(cellDepth-1)*3))&7;
        unsigned long long int _pcCaching=cellPath|(cellDepth-1);
        C3Vector totalTransl;
        retVal=pcNode->pcNodes[index]->getPoints(boxSize*0.5f,ocNodeTranslations[index]*boxSize,_pcCaching,ptCnt,totalTransl);
        if (retVal!=nullptr)
        {
            transf=pcM;
            transf.X+=pcM.M*(boxPos+totalTransl);
        }
    }
    return(retVal);
}
