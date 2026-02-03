#include <kdNode.h>

CKdNode* CKdNode::buildKdTree(const double* pts,size_t ptCnt,const unsigned char* rgbaData,bool rgbForEachPt,double proximityTol)
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
            kdpt.rgba[0]=rgbaData[4*i+0];
            kdpt.rgba[1]=rgbaData[4*i+1];
            kdpt.rgba[2]=rgbaData[4*i+2];
            kdpt.rgba[3]=rgbaData[4*i+3];
        }
        else
        {
            kdpt.rgba[0]=rgbaData[0];
            kdpt.rgba[1]=rgbaData[1];
            kdpt.rgba[2]=rgbaData[2];
            kdpt.rgba[3]=rgbaData[3];
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

CKdNode::CKdNode(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,double proximityTol,int axis)
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

void CKdNode::_populateNode(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,double proximityTol,int axis)
{
    double proxTolPow2=proximityTol*proximityTol;
    for (size_t i=0;i<selectedPts.size();i++)
    {
        if (!pts[selectedPts[i]].ignorePt)
        {
            pt=pts[selectedPts[i]].pt;
            rgba[0]=pts[selectedPts[i]].rgba[0];
            rgba[1]=pts[selectedPts[i]].rgba[1];
            rgba[2]=pts[selectedPts[i]].rgba[2];
            rgba[3]=pts[selectedPts[i]].rgba[3];
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
                double d=dv(0)*dv(0)+dv(1)*dv(1)+dv(2)*dv(2);
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

void CKdNode::_disableClosePts(std::vector<SKdPt>& pts,const std::vector<int>& selectedPts,double proximityTol,int axis)
{
    if (ptIsValid)
    {
        double proxTolPow2=proximityTol*proximityTol;
        std::vector<int> selectedNegativeAxisPts;
        std::vector<int> selectedPositiveAxisPts;
        for (size_t i=0;i<selectedPts.size();i++)
        {
            if (!pts[selectedPts[i]].ignorePt)
            {
                C3Vector dv=pts[selectedPts[i]].pt-pt;
                double d=dv(0)*dv(0)+dv(1)*dv(1)+dv(2)*dv(2);
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

void CKdNode::getPts(std::vector<double>& pts,std::vector<unsigned char>& rgbas)
{
    if (ptIsValid)
    {
        pts.push_back(pt(0));
        pts.push_back(pt(1));
        pts.push_back(pt(2));
        rgbas.push_back(rgba[0]);
        rgbas.push_back(rgba[1]);
        rgbas.push_back(rgba[2]);
        rgbas.push_back(rgba[3]);
        if (kdNodes[0]!=nullptr)
            kdNodes[0]->getPts(pts,rgbas);
        if (kdNodes[1]!=nullptr)
            kdNodes[1]->getPts(pts,rgbas);
    }
}
