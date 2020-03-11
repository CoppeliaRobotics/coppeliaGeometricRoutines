#include "calcUtils.h"
#include "pcNode.h"
#include "ocNode.h"

CPcNode::CPcNode()
{
    pcNodes=nullptr;
}

CPcNode::CPcNode(simReal boxS,const C3Vector& boxCenter,simReal cellS,int cellPts,const std::vector<simReal>& points,std::vector<size_t>& ptsOriginalIndices,std::vector<bool>& ptsInvalidityIndicators,const std::vector<unsigned char>& rgbData,bool rgbForEachPt)
{
    simReal boxHsp=boxS*simReal(0.5001);
    pcNodes=nullptr;
    // Compute points relative to this box:
    std::vector<simReal> points2;
    std::vector<unsigned char> rgbData2;
    std::vector<size_t> ptsOriginalIndices2;
    for (size_t i=0;i<points.size()/3;i++)
    {
        C3Vector pt(&points[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
        {
            points2.push_back(pt(0));
            points2.push_back(pt(1));
            points2.push_back(pt(2));
            ptsOriginalIndices2.push_back(ptsOriginalIndices[i]);
            if (rgbForEachPt)
            {
                rgbData2.push_back(rgbData[3*i+0]);
                rgbData2.push_back(rgbData[3*i+1]);
                rgbData2.push_back(rgbData[3*i+2]);
            }
        }
    }
    if (!rgbForEachPt)
    {
        rgbData2.push_back(rgbData[0]);
        rgbData2.push_back(rgbData[1]);
        rgbData2.push_back(rgbData[2]);
    }
    if (points2.size()>0)
    {
        if ( (boxS>cellS*simReal(1.001))||(int(points2.size()/3)>cellPts) )
        { // subdivide
            pcNodes=new CPcNode* [8];
            for (size_t i=0;i<8;i++)
                pcNodes[i]=new CPcNode(boxS*simHalf,ocNodeTranslations[i]*boxS,cellS,cellPts,points2,ptsOriginalIndices2,ptsInvalidityIndicators,rgbData2,rgbForEachPt);
        }
        else
        { // we are at a leaf. Populate it
            for (size_t i=0;i<points2.size()/3;i++)
            {
                if (!ptsInvalidityIndicators[ptsOriginalIndices2[i]])
                {
                    ptsInvalidityIndicators[ptsOriginalIndices2[i]]=true;
                    pts.push_back(points2[3*i+0]);
                    pts.push_back(points2[3*i+1]);
                    pts.push_back(points2[3*i+2]);
                    if (!rgbForEachPt)
                    {
                        rgbs.push_back(rgbData2[0]);
                        rgbs.push_back(rgbData2[1]);
                        rgbs.push_back(rgbData2[2]);
                    }
                    else
                    {
                        rgbs.push_back(rgbData2[3*i+0]);
                        rgbs.push_back(rgbData2[3*i+1]);
                        rgbs.push_back(rgbData2[3*i+2]);
                    }
                }
            }
        }
    }
}

CPcNode::~CPcNode()
{
    if (pcNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            delete pcNodes[i];
        delete[] pcNodes;
    }
}

CPcNode* CPcNode::copyYourself() const
{
    CPcNode* newPcNode=new CPcNode();
    if (pts.size()>0)
    {
        newPcNode->pts.assign(pts.begin(),pts.end());
        newPcNode->rgbs.assign(rgbs.begin(),rgbs.end());
    }
    else
    {
        if (pcNodes!=nullptr)
        {
            newPcNode->pcNodes=new CPcNode* [8];
            for (size_t i=0;i<8;i++)
                newPcNode->pcNodes[i]=pcNodes[i]->copyYourself();
        }
    }
    return(newPcNode);
}

void CPcNode::scaleYourself(simReal f)
{
    if (pts.size()>0)
    {
        for (size_t i=0;i<pts.size();i++)
            pts[i]*=f;
    }
    else
    {
        if (pcNodes!=nullptr)
        {
            for (size_t i=0;i<8;i++)
                pcNodes[i]->scaleYourself(f);
        }
    }
}

void CPcNode::serialize(std::vector<unsigned char>& data) const
{
    int s=int(pts.size());
    pushData(data,&s,sizeof(int));
    for (size_t i=0;i<pts.size();i++)
        pushData(data,&pts[i],sizeof(simReal));
    for (size_t i=0;i<rgbs.size();i++)
        data.push_back(rgbs[i]);
    if (pcNodes!=nullptr)
    {
        data.push_back(1);
        for (size_t i=0;i<8;i++)
            pcNodes[i]->serialize(data);
    }
    else
        data.push_back(0);
}

void CPcNode::deserialize(const unsigned char* data,int& pos)
{
    int ptsize=(reinterpret_cast<const int*>(data+pos))[0];pos+=sizeof(int);
    for (int i=0;i<ptsize;i++)
    {
        pts.push_back((reinterpret_cast<const simReal*>(data+pos))[0]);
        pos+=sizeof(simReal);
    }
    for (int i=0;i<ptsize;i++)
        rgbs.push_back(data[pos++]);
    if (data[pos++]!=0)
    {
        pcNodes=new CPcNode* [8];
        for (size_t i=0;i<8;i++)
        {
            pcNodes[i]=new CPcNode();
            pcNodes[i]->deserialize(data,pos);
        }
    }
}

size_t CPcNode::countCellsWithContent() const
{
    size_t retVal=0;
    if (pcNodes==nullptr)
    {
        if (pts.size()>0)
            retVal=1;
    }
    else
    {
        for (size_t i=0;i<8;i++)
            retVal+=pcNodes[i]->countCellsWithContent();
    }
    return(retVal);
}

void CPcNode::getPointsPosAndRgb_all(simReal boxS,const C3Vector& boxCenter,std::vector<simReal>& data) const
{
    if (pcNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            pcNodes[i]->getPointsPosAndRgb_all(boxS*simHalf,boxCenter+ocNodeTranslations[i]*boxS,data);
    }
    for (size_t i=0;i<pts.size()/3;i++)
    {
        C3Vector pt(&pts[3*i]);
        pt+=boxCenter;
        data.push_back(pt(0));
        data.push_back(pt(1));
        data.push_back(pt(2));
        data.push_back(simReal(rgbs[3*i+0])/simReal(254.8));
        data.push_back(simReal(rgbs[3*i+1])/simReal(254.8));
        data.push_back(simReal(rgbs[3*i+2])/simReal(254.8));
    }
}

void CPcNode::getPointsPosAndRgb_subset(simReal boxS,const C3Vector& boxCenter,simReal prop,std::vector<simReal>& data) const
{
    if (pcNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            pcNodes[i]->getPointsPosAndRgb_subset(boxS*simHalf,boxCenter+ocNodeTranslations[i]*boxS,prop,data);
    }
    simReal step=(simOne/prop)+simReal(0.0001);
    for (simReal fi=simZero;size_t(fi)<pts.size()/3;fi+=step)
    {
        C3Vector pt(&pts[3*size_t(fi)]);
        pt+=boxCenter;
        data.push_back(pt(0));
        data.push_back(pt(1));
        data.push_back(pt(2));
        data.push_back(simReal(rgbs[3*size_t(fi)+0])/simReal(254.8));
        data.push_back(simReal(rgbs[3*size_t(fi)+1])/simReal(254.8));
        data.push_back(simReal(rgbs[3*size_t(fi)+2])/simReal(254.8));
    }
}

void CPcNode::getOctreeCorners(simReal boxS,const C3Vector& boxCenter,std::vector<simReal>& data) const
{
    if (pcNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            pcNodes[i]->getOctreeCorners(boxS*simHalf,boxCenter+ocNodeTranslations[i]*boxS,data);
    }
    for (size_t i=0;i<8;i++)
    {
        C3Vector shift(ocNodeTranslations[i]*boxS*simTwo);
        data.push_back(boxCenter(0)+shift(0));
        data.push_back(boxCenter(1)+shift(1));
        data.push_back(boxCenter(2)+shift(2));
    }
}

const simReal* CPcNode::getPoints(simReal boxS,const C3Vector& boxCenter,unsigned long long int pcCaching,size_t* ptCnt,C3Vector& totalTransl) const
{
    const simReal* retVal=nullptr;
    unsigned long long int cellPath=(pcCaching>>6)<<6;
    unsigned long long int cellDepth=pcCaching&63;
    if (cellDepth>0)
    {
        if (pcNodes!=nullptr)
        {
            int index=(cellPath>>(6+(cellDepth-1)*3))&7;
            unsigned long long int _pcCaching=cellPath|(cellDepth-1);
            retVal=pcNodes[index]->getPoints(boxS*simHalf,boxCenter+ocNodeTranslations[index]*boxS,_pcCaching,ptCnt,totalTransl);
        }
    }
    else
    {
        if (pts.size()>0)
        {
            totalTransl=boxCenter;
            ptCnt[0]=pts.size()/3;
            return(&pts[0]);
        }
    }
    return(retVal);
}

void CPcNode::add_pts(simReal boxS,const C3Vector& boxCenter,simReal cellS,int cellPts,const std::vector<simReal>& points,std::vector<size_t>& ptsOriginalIndices,std::vector<bool>& ptsInvalidityIndicators,const std::vector<unsigned char>& rgbData,bool rgbForEachPt)
{
    simReal boxHsp=boxS*simReal(0.5001);
    // Compute points relative to this box:
    std::vector<simReal> points2;
    std::vector<unsigned char> rgbData2;
    std::vector<size_t> ptsOriginalIndices2;
    for (size_t i=0;i<points.size()/3;i++)
    {
        C3Vector pt(&points[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
        {
            points2.push_back(pt(0));
            points2.push_back(pt(1));
            points2.push_back(pt(2));
            ptsOriginalIndices2.push_back(ptsOriginalIndices[i]);
            if (rgbForEachPt)
            {
                rgbData2.push_back(rgbData[3*i+0]);
                rgbData2.push_back(rgbData[3*i+1]);
                rgbData2.push_back(rgbData[3*i+2]);
            }
        }
    }
    if (!rgbForEachPt)
    {
        rgbData2.push_back(rgbData[0]);
        rgbData2.push_back(rgbData[1]);
        rgbData2.push_back(rgbData[2]);
    }
    if (points2.size()>0)
    {
        if (pts.size()==0)
        {
            if (pcNodes==nullptr)
            { // can we add the points here?
                if ( ((points2.size()/3)<=size_t(cellPts))&&(boxS<cellS*simReal(1.5)) )
                { // yes
                    for (size_t i=0;i<points2.size()/3;i++)
                    {
                        if (!ptsInvalidityIndicators[ptsOriginalIndices2[i]])
                        {
                            ptsInvalidityIndicators[ptsOriginalIndices2[i]]=true;
                            pts.push_back(points2[3*i+0]);
                            pts.push_back(points2[3*i+1]);
                            pts.push_back(points2[3*i+2]);
                            if (rgbForEachPt)
                            {
                                rgbs.push_back(rgbData2[3*i+0]);
                                rgbs.push_back(rgbData2[3*i+1]);
                                rgbs.push_back(rgbData2[3*i+2]);
                            }
                            else
                            {
                                rgbs.push_back(rgbData2[0]);
                                rgbs.push_back(rgbData2[1]);
                                rgbs.push_back(rgbData2[2]);
                            }
                        }
                    }
                }
                else
                { // we create new children..
                    pcNodes=new CPcNode* [8];
                    for (size_t i=0;i<8;i++)
                        pcNodes[i]=new CPcNode(boxS*simHalf,ocNodeTranslations[i]*boxS,cellS,cellPts,points2,ptsOriginalIndices2,ptsInvalidityIndicators,rgbData2,rgbForEachPt);
                }
            }
            else
            { // continue exploring...
                for (size_t i=0;i<8;i++)
                    pcNodes[i]->add_pts(boxS*simHalf,ocNodeTranslations[i]*boxS,cellS,cellPts,points2,ptsOriginalIndices2,ptsInvalidityIndicators,rgbData2,rgbForEachPt);
            }
        }
        else
        { // can we add more points to this cell?
            if ( (points2.size()+pts.size())/3<=size_t(cellPts) )
            { // yes, we do it
                for (size_t i=0;i<points2.size()/3;i++)
                {
                    if (!ptsInvalidityIndicators[ptsOriginalIndices2[i]])
                    {
                        ptsInvalidityIndicators[ptsOriginalIndices2[i]]=true;
                        pts.push_back(points2[3*i+0]);
                        pts.push_back(points2[3*i+1]);
                        pts.push_back(points2[3*i+2]);
                        if (rgbForEachPt)
                        {
                            rgbs.push_back(rgbData2[3*i+0]);
                            rgbs.push_back(rgbData2[3*i+1]);
                            rgbs.push_back(rgbData2[3*i+2]);
                        }
                        else
                        {
                            rgbs.push_back(rgbData2[0]);
                            rgbs.push_back(rgbData2[1]);
                            rgbs.push_back(rgbData2[2]);
                        }
                    }
                }
            }
            else
            { // we have to create children...
                points2.insert(points2.end(),pts.begin(),pts.end());
                for (size_t i=0;i<pts.size()/3;i++)
                {
                    ptsOriginalIndices2.push_back(ptsInvalidityIndicators.size());
                    ptsInvalidityIndicators.push_back(false);
                }
                pts.clear();
                if (rgbForEachPt)
                    rgbData2.insert(rgbData2.end(),rgbs.begin(),rgbs.end());
                rgbs.clear();
                pcNodes=new CPcNode* [8];
                for (size_t i=0;i<8;i++)
                    pcNodes[i]=new CPcNode(boxS*simHalf,ocNodeTranslations[i]*boxS,cellS,cellPts,points2,ptsOriginalIndices2,ptsInvalidityIndicators,rgbData2,true);
            }
        }
    }
}

bool CPcNode::delete_pts(simReal boxS,const C3Vector& boxCenter,const std::vector<simReal>& points,simReal proximityTol,int* count)
{
    if ( (pcNodes==nullptr)&&(pts.size()==0) )
        return(true); // nothing to remove, node is empty. Remove this node (maybe)
    simReal boxHsp=boxS*simReal(0.5001)+proximityTol;
    // Compute points relative to this box:
    std::vector<simReal> points2;
    for (size_t i=0;i<points.size()/3;i++)
    {
        C3Vector pt(&points[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
        {
            points2.push_back(pt(0));
            points2.push_back(pt(1));
            points2.push_back(pt(2));
        }
    }
    if (points2.size()>0)
    {
        if (pts.size()==0)
        { // no points here
            if (pcNodes==nullptr)
                return(true); // this node could be removed
            else
            { // explore further...
                bool removeChildNodes=true;
                for (size_t i=0;i<8;i++)
                {
                    bool bb=pcNodes[i]->delete_pts(boxS*simHalf,ocNodeTranslations[i]*boxS,points2,proximityTol,count);
                    removeChildNodes=removeChildNodes&&bb;
                }
                if (removeChildNodes)
                {
                    for (size_t i=0;i<8;i++)
                        delete pcNodes[i];
                    delete[] pcNodes;
                    pcNodes=nullptr;
                    return(true); // this node could be removed
                }
                else
                    return(false); // keep this node
            }
        }
        else
        { // we have points here that could be removed...
            simReal dTol=proximityTol*proximityTol;
            for (size_t i=0;i<points2.size()/3;i++)
            {
                C3Vector pt1(&points2[3*i]);
                for (size_t j=0;j<pts.size()/3;j++)
                {
                    C3Vector pt2(&pts[3*j]);
                    pt2-=pt1;
                    simReal d=pt2(0)*pt2(0)+pt2(1)*pt2(1)+pt2(2)*pt2(2);
                    if (d<dTol)
                    {
                        pts.erase(pts.begin()+3*j,pts.begin()+3*j+3);
                        rgbs.erase(rgbs.begin()+3*j,rgbs.begin()+3*j+3);
                        if (count!=nullptr)
                            count[0]++;
                        j--; // reprocess this pos
                    }
                }
                if (pts.size()==0)
                    return(true); // this node could be removed
            }
            return(false); // keep this node
        }
    }
    return(false); // keep this node
}

bool CPcNode::delete_octree(simReal pcBoxS,const C3Vector& pcBoxCenter,const C4X4Matrix& pcM,simReal ocBoxS,const C3Vector& ocBoxCenter,const COcNode* ocNode,const C4X4Matrix& ocM,int* count)
{
    if ( (pcNodes==nullptr)&&(pts.size()==0) )
        return(true); // nothing to remove, node is empty. Remove this node (maybe)
    simReal pcBoxHsp=pcBoxS*simReal(0.5001);
    simReal ocBoxHsp=ocBoxS*simReal(0.5001);
    C4X4Matrix m1(pcM);
    m1.X+=pcM.M*pcBoxCenter;
    C4X4Matrix m2(ocM);
    m2.X+=ocM.M*ocBoxCenter;
    C4X4Matrix relPcM(m2.getInverse()*m1);
    if (pts.size()==0)
    {
        if (pcNodes!=nullptr)
        {   // we have to continue exploring, maybe.
            if ( (ocNode->ocNodes!=nullptr)||(!ocNode->empty) )
            {
                // Fast check first:
                if (CCalcUtils::doCollide_box_cell(relPcM,C3Vector(pcBoxHsp,pcBoxHsp,pcBoxHsp),ocBoxHsp,true))
                { // now explore large volume first:
                    if ( (pcBoxHsp*pcBoxHsp*pcBoxHsp>ocBoxHsp*ocBoxHsp*ocBoxHsp)||(ocNode->ocNodes==nullptr) )
                    { // explore the point cloud...
                        bool removeChildNodes=true;
                        for (size_t i=0;i<8;i++)
                        {
                            bool bb=pcNodes[i]->delete_octree(pcBoxS*simHalf,pcBoxCenter+ocNodeTranslations[i]*pcBoxS,pcM,ocBoxS,ocBoxCenter,ocNode,ocM,count);
                            removeChildNodes=removeChildNodes&&bb;
                        }
                        if (removeChildNodes)
                        {
                            for (size_t i=0;i<8;i++)
                                delete pcNodes[i];
                            delete[] pcNodes;
                            pcNodes=nullptr;
                            return(true); // this node could be removed
                        }
                    }
                    else
                    { // explore the OC tree..
                        for (size_t i=0;i<8;i++)
                        {
                            if (delete_octree(pcBoxS,pcBoxCenter,pcM,ocBoxS*simHalf,ocBoxCenter+ocNodeTranslations[i]*ocBoxS,ocNode->ocNodes[i],ocM,count))
                                return(true); // this node could be removed
                        }
                    }
                }
            }
        }
        else
            return(true); // this node could be removed
    }
    else
    { // we have pts here..
        if ( (ocNode->ocNodes!=nullptr)||(!ocNode->empty) )
        {
            // fast check:
            if (CCalcUtils::doCollide_box_cell(relPcM,C3Vector(pcBoxHsp,pcBoxHsp,pcBoxHsp),ocBoxHsp,true))
            {
                if (ocNode->ocNodes==nullptr)
                { // check pts-cell collision and remove pts that collide:
                    for (size_t i=0;i<pts.size()/3;i++)
                    {
                        C3Vector pt(&pts[3*i]);
                        pt*=relPcM;
                        if ( (fabs(pt(0))<ocBoxHsp)&&(fabs(pt(1))<ocBoxHsp)&&(fabs(pt(2))<ocBoxHsp) )
                        {
                            pts.erase(pts.begin()+3*i,pts.begin()+3*i+3);
                            rgbs.erase(rgbs.begin()+3*i,rgbs.begin()+3*i+3);
                            if (count!=nullptr)
                                count[0]++;
                            i--; // reprocess this pos
                        }
                    }
                    if (pts.size()==0)
                        return(true); // this node could be removed
                }
                else
                { // continue exploring the oc tree...
                    for (size_t i=0;i<8;i++)
                    {
                        if (delete_octree(pcBoxS,pcBoxCenter,pcM,ocBoxS*simHalf,ocBoxCenter+ocNodeTranslations[i]*ocBoxS,ocNode->ocNodes[i],ocM,count))
                            return(true); // this node could be removed
                    }
                }
            }
        }
    }
    return(false); // keep this node
}

bool CPcNode::intersect_pts(simReal boxS,const C3Vector& boxCenter,const std::vector<simReal>& points,simReal proximityTol)
{
    if ( (pcNodes==nullptr)&&(pts.size()==0) )
        return(true); // nothing to intersect, node is empty. Remove this node (maybe)
    simReal boxHsp=boxS*simReal(0.5001)+proximityTol;
    // Compute points relative to this box:
    std::vector<simReal> points2;
    for (size_t i=0;i<points.size()/3;i++)
    {
        C3Vector pt(&points[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
        {
            points2.push_back(pt(0));
            points2.push_back(pt(1));
            points2.push_back(pt(2));
        }
    }
    if (points2.size()>0)
    {
        if (pts.size()==0)
        { // continue exploration... maybe
            if (pcNodes!=nullptr)
            {
                bool removeChildNodes=true;
                for (size_t i=0;i<8;i++)
                {
                    bool bb=pcNodes[i]->intersect_pts(boxS*simHalf,ocNodeTranslations[i]*boxS,points2,proximityTol);
                    removeChildNodes=removeChildNodes&&bb;
                }
                if (removeChildNodes)
                {
                    for (size_t i=0;i<8;i++)
                        delete pcNodes[i];
                    delete[] pcNodes;
                    pcNodes=nullptr;
                    return(true); // this node could be removed
                }
                else
                    return(false); // keep this node
            }
            else
                return(true); // this node could be removed
        }
        else
        { //check the points here (cell points vs points2):
            simReal dTol=proximityTol*proximityTol;
            size_t removableCnt=pts.size()/3;
            std::vector<bool> removableFlags(pts.size()/3,true);
            for (size_t i=0;i<points2.size()/3;i++)
            {
                C3Vector pt1(&points2[3*i]);
                for (size_t j=0;j<pts.size()/3;j++)
                {
                    if (removableFlags[j])
                    {
                        C3Vector pt2(&pts[3*j]);
                        pt2-=pt1;
                        simReal d=pt2(0)*pt2(0)+pt2(1)*pt2(1)+pt2(2)*pt2(2);
                        if (d<dTol)
                        {
                            removableFlags[j]=false;
                            removableCnt--;
                        }
                    }
                }
            }
            if (removableCnt==pts.size()/3)
            {
                pts.clear();
                rgbs.clear();
                return(true); // this node could be removed
            }
            std::vector<simReal> pts2(pts);
            std::vector<unsigned char> rgbs2(rgbs);
            pts.clear();
            rgbs.clear();
            for (size_t i=0;i<removableFlags.size();i++)
            {
                if (!removableFlags[i])
                {
                    pts.push_back(pts2[3*i+0]);
                    pts.push_back(pts2[3*i+1]);
                    pts.push_back(pts2[3*i+2]);
                    rgbs.push_back(rgbs2[3*i+0]);
                    rgbs.push_back(rgbs2[3*i+1]);
                    rgbs.push_back(rgbs2[3*i+2]);
                }
            }
            return(false); // keep this node
        }
    }
    return(true); // nothing to intersect, remove this node (maybe)
}

void CPcNode::flagDuplicates(simReal boxS,const C3Vector& boxCenter,const std::vector<simReal>& points,const std::vector<size_t>& ptsOriginalIndices,std::vector<bool>& duplicateIndicators,simReal proximityTol) const
{
    if ( (pcNodes==nullptr)&&(pts.size()==0) )
        return; // no duplicates here
    simReal boxHsp=boxS*simReal(0.5001)+proximityTol;
    // Compute points relative to this box:
    std::vector<simReal> points2;
    std::vector<size_t> ptsOriginalIndices2;
    for (size_t i=0;i<points.size()/3;i++)
    {
        C3Vector pt(&points[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
        {
            points2.push_back(pt(0));
            points2.push_back(pt(1));
            points2.push_back(pt(2));
            ptsOriginalIndices2.push_back(ptsOriginalIndices[i]);
        }
    }
    if (points2.size()>0)
    {
        if (pts.size()==0)
        {
            if (pcNodes!=nullptr)
            { // continue exploration..
                for (size_t i=0;i<8;i++)
                    pcNodes[i]->flagDuplicates(boxS*simHalf,ocNodeTranslations[i]*boxS,points2,ptsOriginalIndices2,duplicateIndicators,proximityTol);
            }
        }
        else
        { // check duplicates:
            simReal dTol=proximityTol*proximityTol;
            for (size_t i=0;i<points2.size()/3;i++)
            {
                C3Vector pt1(&points2[3*i]);
                for (size_t j=0;j<pts.size()/3;j++)
                {
                    C3Vector pt2(&pts[3*j]);
                    pt2-=pt1;
                    simReal d=pt2(0)*pt2(0)+pt2(1)*pt2(1)+pt2(2)*pt2(2);
                    if (d<dTol)
                    {
                        duplicateIndicators[ptsOriginalIndices2[i]]=true;
                        break;
                    }
                }
            }
        }
    }
}

bool CPcNode::getDistance_pt(simReal boxS,const C3Vector& boxCenter,const C3Vector& point,simReal& dist,C3Vector* pcMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const
{
    bool retVal=false;
    if (dist==simZero)
        return(retVal);
    simReal pcBoxHsp=boxS*simReal(0.5001);
    C3Vector relPoint(point-boxCenter);
    simReal l=relPoint.getLength();
    l-=sqrtf(3*pcBoxHsp*pcBoxHsp);
    if (l<dist)
    { // we could be in that box..
        if (pts.size()==0)
        {
            if (pcNodes!=nullptr)
            { // continue exploration...
                unsigned long long int cellPath=(pcCachePos>>6)<<(6+3);
                unsigned long long int cellDepth=(pcCachePos&63);
                for (size_t i=0;i<8;i++)
                {
                    bool bb=pcNodes[i]->getDistance_pt(boxS*simHalf,boxCenter+ocNodeTranslations[i]*boxS,point,dist,pcMinDistSegPt,pcCaching,cellPath|(i<<6)|(cellDepth+1));
                    retVal=retVal||bb;
                }
            }
        }
        else
        { // check the points in that cell:
            for (size_t i=0;i<pts.size()/3;i++)
            {
                C3Vector pt(&pts[3*i]);
                simReal d=(relPoint-pt).getLength();
                if (d<dist)
                {
                    dist=d;
                    if (pcMinDistSegPt!=nullptr)
                        pcMinDistSegPt[0]=boxCenter+pt;
                    retVal=true;
                    if (pcCaching!=nullptr)
                        pcCaching[0]=pcCachePos;
                }
            }
        }
    }
    return(retVal);
}

bool CPcNode::getDistance_seg(simReal boxS,const C3Vector& boxCenter,const C3Vector& segMiddle,const C3Vector& segHs,simReal& dist,C3Vector* pcMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const
{
    bool retVal=false;
    if (dist==simZero)
        return(retVal);
    simReal pcBoxHsp=boxS*simReal(0.5001);
    C3Vector relSegMiddle(segMiddle-boxCenter);
    simReal l=relSegMiddle.getLength();
    l=l-sqrtf(3*pcBoxHsp*pcBoxHsp)-segHs.getLength();
    if (l<dist)
    { // we could be in that box..
        if (pts.size()==0)
        {
            if (pcNodes!=nullptr)
            { // continue exploration...
                unsigned long long int cellPath=(pcCachePos>>6)<<(6+3);
                unsigned long long int cellDepth=(pcCachePos&63);
                for (size_t i=0;i<8;i++)
                {
                    bool bb=pcNodes[i]->getDistance_seg(boxS*simHalf,boxCenter+ocNodeTranslations[i]*boxS,segMiddle,segHs,dist,pcMinDistSegPt,segMinDistSegPt,pcCaching,cellPath|(i<<6)|(cellDepth+1));
                    retVal=retVal||bb;
                }
            }
        }
        else
        { // check the points in that cell:
            for (size_t i=0;i<pts.size()/3;i++)
            {
                C3Vector pt(&pts[3*i]);
                if (CCalcUtils::getDistance_segp_pt(relSegMiddle-segHs,segHs*simTwo,pt,dist,segMinDistSegPt))
                {
                    retVal=true;
                    if (pcMinDistSegPt!=nullptr)
                        pcMinDistSegPt[0]=boxCenter+pt;
                    if (segMinDistSegPt!=nullptr)
                        segMinDistSegPt[0]+=boxCenter;
                    if (pcCaching!=nullptr)
                        pcCaching[0]=pcCachePos;
                }
            }
        }
    }
    return(retVal);
}

bool CPcNode::getDistance_tri(simReal boxS,const C3Vector& boxCenter,const C3Vector& p,const C3Vector& v,const C3Vector& w,simReal& dist,C3Vector* pcMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos) const
{
    bool retVal=false;
    if (dist==simZero)
        return(retVal);
    simReal pcBoxHsp=boxS*simReal(0.5001);
    C3Vector relP(p-boxCenter);
    simReal l=relP.getLength();
    l=l-sqrtf(3*pcBoxHsp*pcBoxHsp)-std::max<simReal>(v.getLength(),w.getLength());
    if (l<dist)
    { // we could be in that box..
        if (pts.size()==0)
        {
            if (pcNodes!=nullptr)
            { // continue exploration...
                unsigned long long int cellPath=(pcCachePos>>6)<<(6+3);
                unsigned long long int cellDepth=(pcCachePos&63);
                for (size_t i=0;i<8;i++)
                {
                    bool bb=pcNodes[i]->getDistance_tri(boxS*simHalf,boxCenter+ocNodeTranslations[i]*boxS,p,v,w,dist,pcMinDistSegPt,triMinDistSegPt,pcCaching,cellPath|(i<<6)|(cellDepth+1));
                    retVal=retVal||bb;
                }
            }
        }
        else
        { // check the points in that cell:
            for (size_t i=0;i<pts.size()/3;i++)
            {
                C3Vector pt(&pts[3*i]);
                if (CCalcUtils::getDistance_tri_pt(relP,v,w,pt,dist,triMinDistSegPt))
                {
                    retVal=true;
                    if (pcMinDistSegPt!=nullptr)
                        pcMinDistSegPt[0]=boxCenter+pt;
                    if (triMinDistSegPt!=nullptr)
                        triMinDistSegPt[0]+=boxCenter;
                    if (pcCaching!=nullptr)
                        pcCaching[0]=pcCachePos;
                }
            }
        }
    }
    return(retVal);
}

bool CPcNode::getDistance_shape(simReal boxS,const C3Vector& boxCenter,const C4X4Matrix& pcM,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,simReal& dist,C3Vector* pcMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* pcCaching,unsigned long long int pcCachePos,int* obbCaching) const
{
    bool retVal=false;
    if (dist==simZero)
        return(retVal);
    C4X4Matrix pcTr(pcM);
    pcTr.X+=pcM.M*boxCenter;
    C4X4Matrix shapeTr(shapeM*obb->boxM);
    // Make a quick check:
    simReal d=CCalcUtils::getApproxDistance_box_box(pcTr,C3Vector(boxS*simHalf,boxS*simHalf,boxS*simHalf),shapeTr,obb->boxHs);
    if (d<dist)
    { // we may be closer than dist
        bool exploreObbNode=false;
        if (pcNodes!=nullptr)
        { // PC node can be explored further
            if ( (obb->leafTris!=nullptr)||(boxS*boxS*boxS>obb->boxHs(0)*obb->boxHs(1)*obb->boxHs(2)) )
            { // explore the PC node. Pick the box closer to the OBB box
                C4X4Matrix shapeTrRel(pcTr.getInverse()*shapeTr);
                std::vector<std::pair<simReal,SNodeTranslation>> nodesToExplore;
                for (size_t i=0;i<8;i++)
                {
                    SNodeTranslation nodeTranslation;
                    nodeTranslation.transl=ocNodeTranslations[i]*boxS;
                    nodeTranslation.index=i;
                    nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-shapeTrRel.X).getLength(),nodeTranslation));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                unsigned long long int cellPath=(pcCachePos>>6)<<(6+3);
                unsigned long long int cellDepth=(pcCachePos&63);
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    C3Vector transl(nodesToExplore[i].second.transl);
                    size_t index=nodesToExplore[i].second.index;
                    unsigned long long int _pcCacheValueHere=cellPath|(index<<6)|(cellDepth+1);
                    bool bb=pcNodes[index]->getDistance_shape(boxS*simHalf,boxCenter+transl,pcM,obbStruct,obb,shapeM,dist,pcMinDistSegPt,shapeMinDistSegPt,pcCaching,_pcCacheValueHere,obbCaching);
                    retVal=retVal||bb;
                }
            }
            else
                exploreObbNode=true;
        }
        else
        { // PC node can't be explored further
            if (pts.size()>0)
            { // there are points in the PC cell
                if (obb->leafTris==nullptr)
                    exploreObbNode=true;
                else
                { // Check triangles of OBB node vs this voxel's pts
                    C4X4Matrix shapeMRel(pcTr.getInverse()*shapeM);
                    for (size_t i=0;i<obb->leafTris->size();i++)
                    {
                        int ind=3*obb->leafTris[0][i];
                        C3Vector p(&obbStruct->vertices[3*size_t(obbStruct->indices[size_t(ind)+0])+0]);
                        C3Vector v(&obbStruct->vertices[3*size_t(obbStruct->indices[size_t(ind)+1])+0]);
                        C3Vector w(&obbStruct->vertices[3*size_t(obbStruct->indices[size_t(ind)+2])+0]);
                        p*=shapeMRel;
                        v*=shapeMRel;
                        w*=shapeMRel;
                        v-=p;
                        w-=p;
                        for (size_t j=0;j<pts.size()/3;j++)
                        {
                            C3Vector pcPt(&pts[3*j]);
                            if (CCalcUtils::getDistance_tri_pt(p,v,w,pcPt,dist,shapeMinDistSegPt))
                            {
                                if (pcMinDistSegPt!=nullptr)
                                    pcMinDistSegPt[0]=pcTr*pcPt;
                                if (shapeMinDistSegPt!=nullptr)
                                    shapeMinDistSegPt[0]*=pcTr;
                                if (pcCaching!=nullptr)
                                    pcCaching[0]=pcCachePos;
                                if (obbCaching!=nullptr)
                                    obbCaching[0]=ind/3;
                                retVal=true;
                            }
                        }
                    }
                }
            }
        }
        if (exploreObbNode)
        {
            const CObbNode* nodes[2];
            simReal d[2]={dist,dist};
            CCalcUtils::isApproxDistanceSmaller_box_box_fast(pcTr,C3Vector(boxS,boxS,boxS),shapeM*obb->obbNodes[0]->boxM,obb->obbNodes[0]->boxHs,d[0]);
            CCalcUtils::isApproxDistanceSmaller_box_box_fast(pcTr,C3Vector(boxS,boxS,boxS),shapeM*obb->obbNodes[1]->boxM,obb->obbNodes[1]->boxHs,d[1]);
            if (d[0]<=d[1])
            { // we first explore the closest box
                nodes[0]=obb->obbNodes[0];
                nodes[1]=obb->obbNodes[1];
            }
            else
            {
                nodes[0]=obb->obbNodes[1];
                nodes[1]=obb->obbNodes[0];
            }
            for (size_t i=0;i<2;i++)
            {
                bool bb=getDistance_shape(boxS,boxCenter,pcM,obbStruct,nodes[i],shapeM,dist,pcMinDistSegPt,shapeMinDistSegPt,pcCaching,pcCachePos,obbCaching);
                retVal=retVal||bb;
                if (dist<=d[1])
                    break;
            }
        }
    }
    return(retVal);
}

bool CPcNode::getDistance_ptcloud(simReal box1Size,const C3Vector& box1Center,const C4X4Matrix& pc1M,const CPcNode* pc2Node,simReal box2Size,const C3Vector& box2Center,const C4X4Matrix& pc2M,simReal& dist,C3Vector* pc1MinDistSegPt,C3Vector* pc2MinDistSegPt,unsigned long long int* pc1Caching,unsigned long long int pc1CachePos,unsigned long long int* pc2Caching,unsigned long long int pc2CachePos) const
{
    bool retVal=false;
    if (dist==simZero)
        return(retVal);
    C4X4Matrix m1(pc1M);
    m1.X+=pc1M.M*box1Center;
    C4X4Matrix m2(pc2M);
    m2.X+=pc2M.M*box2Center;
    simReal pc1BoxHsp=box1Size*simReal(0.5001);
    simReal pc2BoxHsp=box2Size*simReal(0.5001);
    simReal d=((pc1M.X+pc1M.M*box1Center)-(pc2M.X+pc2M.M*box2Center)).getLength()-sqrtf(3*pc1BoxHsp*pc1BoxHsp)-sqrtf(3*pc2BoxHsp*pc2BoxHsp);
    if (d<dist)
    {   // We might be closer...
        int nodeIndexToExplore=-1;
        if (pcNodes==nullptr)
        { // PC1 can't be explored. Does it have points?
            if (pts.size()>0)
            { // yes
                if (pc2Node->pcNodes==nullptr)
                { // PC2 can't be explored either. Does it have points?
                    if (pc2Node->pts.size()>0)
                    { // yes. Check pt-pt distances:
                        simReal dSquared=dist*dist;
                        for (size_t i=0;i<pts.size()/3;i++)
                        {
                            C3Vector pt1(&pts[3*i]);
                            pt1*=m1;
                            for (size_t j=0;j<pc2Node->pts.size()/3;j++)
                            {
                                C3Vector pt2(&pc2Node->pts[3*j]);
                                pt2*=m2;
                                C3Vector pt1Pt2(pt2(0)-pt1(0),pt2(1)-pt1(1),pt2(2)-pt1(2));
                                d=pt1Pt2(0)*pt1Pt2(0)+pt1Pt2(1)*pt1Pt2(1)+pt1Pt2(2)*pt1Pt2(2);
                                if (d<dSquared)
                                {
                                    dSquared=d;
                                    dist=sqrtf(d);
                                    if (pc1MinDistSegPt!=nullptr)
                                        pc1MinDistSegPt[0]=pt1;
                                    if (pc2MinDistSegPt!=nullptr)
                                        pc2MinDistSegPt[0]=pt2;
                                    if (pc1Caching!=nullptr)
                                        pc1Caching[0]=pc1CachePos;
                                    if (pc2Caching!=nullptr)
                                        pc2Caching[0]=pc2CachePos;
                                    retVal=true;
                                }
                            }
                        }
                    }
                }
                else
                    nodeIndexToExplore=1; // PC2 can be explored
            }
        }
        else
        { // we could explore PC1 nodes...
            if (pc2Node->pcNodes==nullptr)
            { // PC2 can't be explored. Does it have points?
                if (pc2Node->pts.size()>0)
                    nodeIndexToExplore=0; // yes, explore PC1
            }
            else
            { // we could also explore PC2 nodes...
                if (box1Size>box2Size)
                    nodeIndexToExplore=0;
                else
                    nodeIndexToExplore=1;
            }
        }
        if (nodeIndexToExplore>=0)
        { // continue exploration of PC with index nodeIndexToExplore
            int otherNodeIndex=0;
            if (nodeIndexToExplore==0)
                otherNodeIndex=1;
            C4X4Matrix m2Rel(m1.getInverse()*m2);
            C4X4Matrix m1Rel(m2.getInverse()*m1);
            C4X4Matrix* mRels[2]={&m2Rel,&m1Rel};
            const CPcNode* pcNodes[2]={this,pc2Node};
            simReal boxSizes[2]={box1Size,box2Size};
            const C3Vector* boxCenters[2]={&box1Center,&box2Center};
            const C4X4Matrix* pcMs[2]={&pc1M,&pc2M};
            C3Vector* pcMinDistSegPts[2]={pc1MinDistSegPt,pc2MinDistSegPt};
            unsigned long long int* pcCachings[2]={pc1Caching,pc2Caching};
            unsigned long long int pcCachePoss[2]={pc1CachePos,pc2CachePos};
            // Now check all 8 node pairs, explore closest node first:
            std::vector<std::pair<simReal,SNodeTranslation>> nodesToExplore;
            for (size_t i=0;i<8;i++)
            {
                SNodeTranslation nodeTranslation;
                nodeTranslation.transl=ocNodeTranslations[i]*boxSizes[nodeIndexToExplore];
                nodeTranslation.index=i;
                nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-mRels[nodeIndexToExplore][0].X).getLength(),nodeTranslation));
            }
            std::sort(nodesToExplore.begin(),nodesToExplore.end());
            unsigned long long int cellPath=(pcCachePoss[nodeIndexToExplore]>>6)<<(6+3);
            unsigned long long int cellDepth=(pcCachePoss[nodeIndexToExplore]&63);
            for (size_t i=0;i<nodesToExplore.size();i++)
            {
                C3Vector transl(nodesToExplore[i].second.transl);
                size_t index=nodesToExplore[i].second.index;
                unsigned long long int cacheValueHere=cellPath|(index<<6)|(cellDepth+1);
                bool bb=pcNodes[nodeIndexToExplore]->pcNodes[index]->getDistance_ptcloud(boxSizes[nodeIndexToExplore]*simHalf,boxCenters[nodeIndexToExplore][0]+transl,pcMs[nodeIndexToExplore][0],pcNodes[otherNodeIndex],boxSizes[otherNodeIndex],boxCenters[otherNodeIndex][0],pcMs[otherNodeIndex][0],dist,pcMinDistSegPts[nodeIndexToExplore],pcMinDistSegPts[otherNodeIndex],pcCachings[nodeIndexToExplore],cacheValueHere,pcCachings[otherNodeIndex],pcCachePoss[otherNodeIndex]);
                retVal=retVal||bb;
            }
        }
    }
    return(retVal);
}

bool CPcNode::getSensorDistance(simReal boxS,const C3Vector& boxCenter,const C4X4Matrix& pcM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,bool fast,simReal& dist,C3Vector* detectPt) const
{
    bool retVal=false;
    if (dist==simZero)
        return(retVal);
    C4X4Matrix m(pcM);
    m.X+=pcM.M*boxCenter;
    simReal d=dist;
    if (CCalcUtils::getDistance_box_pt(m,C3Vector(boxS*simHalf,boxS*simHalf,boxS*simHalf),true,C3Vector::zeroVector,d,nullptr,nullptr))
    { // we are closer than dist..
        if (CCalcUtils::isBoxMaybeInSensorVolume(planesIn,planesOut,m,C3Vector(boxS*simHalf,boxS*simHalf,boxS*simHalf)))
        { // the PC box touches the sensor volume
            if (pcNodes==nullptr)
            { // no more child nodes
                for (size_t i=0;i<pts.size()/3;i++)
                { // test the pts
                    C3Vector pt(&pts[3*i]);
                    pt*=m;
                    d=pt.getLength();
                    if (d<dist)
                    { // that pt is closer..
                        if (CCalcUtils::isPointInVolume(planesIn,pt))
                        { // ..and we are inside volume1..
                            if ( (planesOut.size()==0)||(!CCalcUtils::isPointInVolume(planesOut,pt)) )
                            { // and we are outside volume2. Register pt
                                dist=d;
                                if (detectPt!=nullptr)
                                    detectPt[0]=pt;
                                retVal=true;
                                if (fast)
                                    break;
                            }
                        }
                    }
                }
            }
            else
            { // continue exploration of the PC node. Explore closer nodes first:
                std::vector<std::pair<simReal,SNodeTranslation>> nodesToExplore;
                for (size_t i=0;i<8;i++)
                {
                    SNodeTranslation nodeTranslation;
                    nodeTranslation.transl=ocNodeTranslations[i]*boxS;
                    nodeTranslation.index=i;
                    d=(m.X+pcM.M*nodeTranslation.transl).getLength();
                    nodesToExplore.push_back(std::make_pair(d,nodeTranslation));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    C3Vector transl(nodesToExplore[i].second.transl);
                    size_t index=nodesToExplore[i].second.index;
                    bool bb=pcNodes[index]->getSensorDistance(boxS*simHalf,boxCenter+transl,pcM,planesIn,planesOut,fast,dist,detectPt);
                    retVal=retVal||bb;
                    if (retVal&&fast)
                        break;
                }
            }
        }
    }
    return(retVal);
}
