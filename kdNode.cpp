#include "kdNode.h"

CKdNode* CKdNode::buildKdTree(const simReal* pts,size_t ptCnt,const unsigned char* rgbData,bool rgbForEachPt,simReal proximityTol)
{
    std::vector<SKdPt> allPts;
    std::vector<int> selectedPts;
    for (size_t i=0;i<ptCnt;i++)
    {
        SKdPt kdpt;
        kdpt.pt=C3Vector(pts+3*i);
        kdpt.ignorePt=false;
        if (rgbForEachPt)
        {
            kdpt.rgb[0]=rgbData[3*i+0];
            kdpt.rgb[1]=rgbData[3*i+1];
            kdpt.rgb[2]=rgbData[3*i+2];
        }
        else
        {
            kdpt.rgb[0]=rgbData[0];
            kdpt.rgb[1]=rgbData[1];
            kdpt.rgb[2]=rgbData[2];
        }
        allPts.push_back(kdpt);
        selectedPts.push_back(int(i));
    }
    CKdNode* rootNode=new CKdNode();
    rootNode->_populateNode(allPts,selectedPts,proximityTol,0);
    return(rootNode);
}

CKdNode::CKdNode()
{
    kdNodes[0]=nullptr;
    kdNodes[1]=nullptr;
    ptIsValid=false;
}

CKdNode::CKdNode(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,simReal proximityTol,int axis)
{
    kdNodes[0]=nullptr;
    kdNodes[1]=nullptr;
    ptIsValid=false;
    _populateNode(pts,selectedPts,proximityTol,axis);
}

CKdNode::~CKdNode()
{
    delete kdNodes[0];
    delete kdNodes[1];
}

void CKdNode::_populateNode(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,simReal proximityTol,int axis)
{
    simReal proxTolPow2=proximityTol*proximityTol;
    for (size_t i=0;i<selectedPts.size();i++)
    {
        if (!pts[selectedPts[i]].ignorePt)
        {
            pt=pts[selectedPts[i]].pt;
            rgb[0]=pts[selectedPts[i]].rgb[0];
            rgb[1]=pts[selectedPts[i]].rgb[1];
            rgb[2]=pts[selectedPts[i]].rgb[2];
            ptIsValid=true;
            pts[selectedPts[i]].ignorePt=true;
            break;
        }
    }
    if (ptIsValid)
    {
        std::vector<int> selectedNegativeAxisPts;
        std::vector<int> selectedPositiveAxisPts;
        for (size_t i=0;i<selectedPts.size();i++)
        {
            if (!pts[selectedPts[i]].ignorePt)
            {
                C3Vector v(pts[selectedPts[i]].pt);
                C3Vector dv(v-pt);
                simReal d=dv(0)*dv(0)+dv(1)*dv(1)+dv(2)*dv(2);
                if (d>proxTolPow2)
                {
                    if (dv(axis)<0.0)
                        selectedNegativeAxisPts.push_back(selectedPts[i]);
                    else
                        selectedPositiveAxisPts.push_back(selectedPts[i]);
                }
            }
        }

        axis++;
        if (axis==3)
            axis=0;
        if (selectedPositiveAxisPts.size()>0)
            kdNodes[1]=new CKdNode(pts,selectedPositiveAxisPts,proximityTol,axis);
        if (selectedNegativeAxisPts.size()>0)
        {
            if (kdNodes[1]!=nullptr)
                kdNodes[1]->_disableClosePts(pts,selectedNegativeAxisPts,proximityTol,axis);
            kdNodes[0]=new CKdNode(pts,selectedNegativeAxisPts,proximityTol,axis);
        }
    }
}

void CKdNode::_disableClosePts(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,simReal proximityTol,int axis)
{
    if (ptIsValid)
    {
        simReal proxTolPow2=proximityTol*proximityTol;
        std::vector<int> selectedNegativeAxisPts;
        std::vector<int> selectedPositiveAxisPts;
        for (size_t i=0;i<selectedPts.size();i++)
        {
            if (!pts[selectedPts[i]].ignorePt)
            {
                C3Vector dv=pts[selectedPts[i]].pt-pt;
                simReal d=dv(0)*dv(0)+dv(1)*dv(1)+dv(2)*dv(2);
                if (d<proxTolPow2)
                    pts[selectedPts[i]].ignorePt=true;
                else
                {
                    if (dv(axis)<0.0)
                    {
                        selectedNegativeAxisPts.push_back(selectedPts[i]);
                        if (dv(axis)>-proximityTol) // do not forget points on the other side, close to the border!
                            selectedPositiveAxisPts.push_back(selectedPts[i]);
                    }
                    else
                    {
                        selectedPositiveAxisPts.push_back(selectedPts[i]);
                        if (dv(axis)<proximityTol) // do not forget points on the other side, close to the border!
                            selectedNegativeAxisPts.push_back(selectedPts[i]);
                    }
                }
            }
        }

        axis++;
        if (axis==3)
            axis=0;
        if ((kdNodes[0]!=nullptr)&&(selectedNegativeAxisPts.size()>0))
            kdNodes[0]->_disableClosePts(pts,selectedNegativeAxisPts,proximityTol,axis);
        if ((kdNodes[1]!=nullptr)&&(selectedPositiveAxisPts.size()>0))
            kdNodes[1]->_disableClosePts(pts,selectedPositiveAxisPts,proximityTol,axis);
    }
}

void CKdNode::getPts(std::vector<simReal>& pts,std::vector<unsigned char>& rgbs)
{
    if (ptIsValid)
    {
        pts.push_back(pt(0));
        pts.push_back(pt(1));
        pts.push_back(pt(2));
        rgbs.push_back(rgb[0]);
        rgbs.push_back(rgb[1]);
        rgbs.push_back(rgb[2]);
        if (kdNodes[0]!=nullptr)
            kdNodes[0]->getPts(pts,rgbs);
        if (kdNodes[1]!=nullptr)
            kdNodes[1]->getPts(pts,rgbs);
    }
}
