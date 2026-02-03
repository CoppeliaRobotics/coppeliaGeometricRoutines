#include <calcUtils.h>

unsigned long CCalcUtils::getDjb2Hash(const char* inputStr,size_t size)
{ // taken from http://www.cse.yorku.ca/~oz/hash.html
    unsigned long hash = 5381;
    for (size_t i=0;i<size;i++)
        hash = ((hash << 5) + hash) + int(inputStr[i]);
    return hash;
}

bool CCalcUtils::doCollide_tri_tri(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,int tri1Index,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,std::vector<double>* intersections,int* cachingTri1,int* cachingTri2)
{
    C3Vector n1((v1^w1).getNormalized());
    C3Vector n2((v2^w2).getNormalized());

    if (n1.isColinear(n2,double(0.99999)))
        return(false); // colinear!

    // Now check for separating axes:
    C3Vector u1(w1-v1);
    C3Vector u2(w2-v2);
    C3Vector tr1Edges[3]={v1,w1,u1};
    C3Vector tr2Edges[3]={v2,w2,u2};
    if (getProjectedDistance_tri_tri(n1,p1,v1,w1,p2,v2,w2)>0.0)
        return(false); // n1 is the separating axis!
    if (getProjectedDistance_tri_tri(n2,p1,v1,w1,p2,v2,w2)>0.0)
        return(false); // n2 is the separating axis!
    for (size_t i=0;i<3;i++)
    {
        for (size_t j=0;j<3;j++)
        {
            C3Vector axis=tr1Edges[i]^tr2Edges[j];
            if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
            { // Edges are not parallel
                if (getProjectedDistance_tri_tri(axis.getNormalized(),p1,v1,w1,p2,v2,w2)>0.0)
                    return(false); // that axis is separating!
            }
        }
    }
    if (intersections!=nullptr)
    {   // get the two intersections, i.e. tri1Surface vs tri2Edges + tri2Surface vs tri1Edges:
        const C3Vector* triP[2]={&p1,&p2};
        const C3Vector* triV[2]={&v1,&v2};
        const C3Vector* triW[2]={&w1,&w2};
        C3Vector intersection1;
        bool intersection1Done=false;
        C3Vector segP,segL;
        for (size_t j=0;j<2;j++)
        {
            C3Vector p(triP[j][0]);
            C3Vector v(triV[j][0]);
            C3Vector w(triW[j][0]);
            C3Vector n(v^w);
            size_t j2=1;
            if (j==1)
                j2=0;
            for (size_t i=0;i<3;i++)
            {
                if (i<2)
                    segP=triP[j2][0];
                else
                    segP=triP[j2][0]+triV[j2][0];
                if (i==0)
                    segL=triV[j2][0];
                if (i==1)
                    segL=triW[j2][0];
                if (i==2)
                    segL=triW[j2][0]-triV[j2][0];
                // Does the segment maybe collide with the triangle?
                double dd=-(p*n);
                double denom=n*segL;
                if (denom!=0.0)
                {   // Seg and tri are not parallel
                    double t=-((n*segP)+dd)/denom;
                    if ( (t>=0.0)&&(t<=1.0) )
                    {   // the segment collides with the triangle's plane
                        C3Vector intersection(segP+(segL*t));
                        // is intersection within triangle's borders?
                        C3Vector vect(intersection-p);
                        if ((vect^w)*n>0.0)
                        { // within border1
                            if ((v^vect)*n>0.0)
                            { // within border2
                                vect-=v;
                                C3Vector wv(w-v);
                                if ((wv^vect)*n>0.0)
                                { // within border3
                                    if (intersection1Done)
                                    {
                                        intersections->push_back(intersection1(0));
                                        intersections->push_back(intersection1(1));
                                        intersections->push_back(intersection1(2));
                                        intersections->push_back(intersection(0));
                                        intersections->push_back(intersection(1));
                                        intersections->push_back(intersection(2));
                                    }
                                    else
                                    {
                                        intersection1=intersection;
                                        intersection1Done=true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if (cachingTri1!=nullptr)
        cachingTri1[0]=tri1Index;
    if (cachingTri2!=nullptr)
        cachingTri2[0]=tri2Index;
    return(true);
}

bool CCalcUtils::doCollide_tri_segp(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,int tri1Index,const C3Vector& segP,const C3Vector& segL,C3Vector* intersection,int* cachingTri1)
{
    bool retVal=false;
    C3Vector n(v1^w1);
    // Does the segment maybe collide with the triangle?
    double dd=-(p1*n);
    double denom=n*segL;
    if (denom!=0.0)
    {   // Seg and tri are not parallel
        double t=-((n*segP)+dd)/denom;
        if ( (t>=0.0)&&(t<=1.0) )
        {   // the segment collides with the triangle's plane
            C3Vector intersect(segP+(segL*t));
            // is intersection within triangle's borders?
            C3Vector vect(intersect-p1);
            if ((vect^w1)*n>0.0)
            { // within border1
                if ((v1^vect)*n>0.0)
                { // within border2
                    vect-=v1;
                    C3Vector wv(w1-v1);
                    if ((wv^vect)*n>0.0)
                    { // within border3
                        retVal=true;
                        if (intersection!=nullptr)
                            intersection[0]=intersect;
                    }
                }
            }
        }
    }
    if (cachingTri1!=nullptr)
        cachingTri1[0]=tri1Index;
    return(retVal);
}

bool CCalcUtils::doCollide_box_cell(const C4X4Matrix& box,const C3Vector& boxHs,double cellHs,bool solidBoxAndCell)
{   // Cell is at the origin
    // optimize!!!
    C4X4Matrix box2;
    box2.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(doCollide_box_box(box2,sv,box,boxHs,solidBoxAndCell));
}

bool CCalcUtils::doCollide_box_box(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,bool solidBoxes)
{
    for (size_t i=0;i<3;i++)
    { // box 1's axes
        double box1Box2Proj=fabs((box2.X-box1.X)*box1.M.axis[i]);
        double box2Overlap=fabs(box2.M.axis[0]*box1.M.axis[i])*box2Hs(0)+fabs(box2.M.axis[1]*box1.M.axis[i])*box2Hs(1)+fabs(box2.M.axis[2]*box1.M.axis[i])*box2Hs(2);
        if (box1Hs(i)+box2Overlap<box1Box2Proj)
            return(false);
    }
    for (size_t i=0;i<3;i++)
    { // box 2's axes
        double box1Box2Proj=fabs((box2.X-box1.X)*box2.M.axis[i]);
        double box1Overlap=fabs(box1.M.axis[0]*box2.M.axis[i])*box1Hs(0)+fabs(box1.M.axis[1]*box2.M.axis[i])*box1Hs(1)+fabs(box1.M.axis[2]*box2.M.axis[i])*box1Hs(2);
        if (box2Hs(i)+box1Overlap<box1Box2Proj)
            return(false);
    }
    for (size_t i=0;i<3;i++)
    {
        for (size_t j=0;j<3;j++)
        {
            C3Vector a(box1.M.axis[i]^box2.M.axis[j]);
            if ( (a(0)!=0.0)||(a(1)!=0.0)||(a(2)!=0.0) )
            {
                a.normalize();
                double box1Box2Proj=fabs((box2.X-box1.X)*a);
                double box1Overlap=fabs(box1.M.axis[0]*a)*box1Hs(0)+fabs(box1.M.axis[1]*a)*box1Hs(1)+fabs(box1.M.axis[2]*a)*box1Hs(2);
                double box2Overlap=fabs(box2.M.axis[0]*a)*box2Hs(0)+fabs(box2.M.axis[1]*a)*box2Hs(1)+fabs(box2.M.axis[2]*a)*box2Hs(2);
                if (box1Overlap+box2Overlap<box1Box2Proj)
                    return(false);
            }
        }
    }
    if (!solidBoxes)
    {
        std::vector<C3Vector> b1pts;
        std::vector<C3Vector> b2pts;
        for (double z=-1.0;z<2.0;z+=2.0)
        {
            C3Vector a1;
            C3Vector a2;
            a1(2)=z*box1Hs(2);
            a2(2)=z*box2Hs(2);
            for (double y=-1.0;y<2.0;y+=2.0)
            {
                a1(1)=y*box1Hs(1);
                a2(1)=y*box2Hs(1);
                for (double x=-1.0;x<2.0;x+=2.0)
                {
                    a1(0)=x*box1Hs(0);
                    a2(0)=x*box2Hs(0);
                    b1pts.push_back(box1*a1);
                    b2pts.push_back(box2*a2);
                }
            }
        }
        if (doCollide_box_allpts(box1,box1Hs,b2pts))
            return(false); // box2 is contained in box1
        if (doCollide_box_allpts(box2,box2Hs,b1pts))
            return(false); // box1 is contained in box2
    }
    return(true);
}

bool CCalcUtils::doCollide_box_tri(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& p,const C3Vector& v,const C3Vector& w)
{
    C3Vector n((v^w).getNormalized());
    if (getProjectedDistance_box_tri(n,box,boxHs,p,v,w)>0.0)
        return(false); // tri normal vect. is separating axis
    for (size_t i=0;i<3;i++)
    {
        if (getProjectedDistance_box_tri(box.M.axis[i],box,boxHs,p,v,w)>0.0)
            return(false); // one of the box's edges is separating axis
    }
    // Now check the tri edges:
    C3Vector u(w-v);
    C3Vector trEdges[3]={v,w,u};
    for (size_t i=0;i<3;i++)
    {
        for (size_t j=0;j<3;j++)
        {
            C3Vector axis=box.M.axis[i]^trEdges[j];
            if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
            {
                if (getProjectedDistance_box_tri(axis.getNormalized(),box,boxHs,p,v,w)>0.0)
                    return(false);
            }
        }
    }
    if (!solidBox)
    {
        std::vector<C3Vector> pts;
        pts.push_back(p);
        pts.push_back(p+v);
        pts.push_back(p+w);
        if (doCollide_box_allpts(box,boxHs,pts))
            return(false); // triangle is contained in the box
    }
    return(true);
}

bool CCalcUtils::doCollide_box_segp(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& segP,const C3Vector& segL)
{   // Segment is defined as segP (end-point1) and segP+segL (end-point2)
    C3Vector segHs(segL*0.5);
    C3Vector segCenter(segP+segHs);
    return(doCollide_box_seg(box,boxHs,solidBox,segCenter,segHs));
}

bool CCalcUtils::doCollide_box_seg(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& segCenter,const C3Vector& segHs)
{   // Segment is defined as segCenter (center of segment) and segHs (segment vector half-size)
    // Box-segment segment:
    if (getProjectedDistance_box_seg(segCenter.getNormalized(),box,boxHs,segCenter,segHs)>0.0)
        return(false);
    // Segment axis:
    if (getProjectedDistance_box_seg(segHs.getNormalized(),box,boxHs,segCenter,segHs)>0.0)
        return(false);
    // Box axes:
    for (size_t i=0;i<3;i++)
    {
        if (getProjectedDistance_box_seg(box.M.axis[i],box,boxHs,segCenter,segHs)>0.0)
            return(false);
    }
    // Combination of Box and segment axes:
    for (size_t i=0;i<3;i++)
    {
        if (!segHs.isColinear(box.M.axis[i],double(0.99999)))
        {
            if (getProjectedDistance_box_seg((segHs^box.M.axis[i]).getNormalized(),box,boxHs,segCenter,segHs)>0.0)
                return(false);
        }
    }
    if (!solidBox)
    {
        std::vector<C3Vector> pts;
        pts.push_back(segCenter-segHs);
        pts.push_back(segCenter+segHs);
        if (doCollide_box_allpts(box,boxHs,pts))
            return(false); // segment is contained in the box
    }
    return(true);
}

bool CCalcUtils::doCollide_box_allpts(const C4X4Matrix& box,const C3Vector& boxHs,const std::vector<C3Vector>& points)
{ // true if all points are within the box
    C4X4Matrix boxInv(box.getInverse());
    for (size_t i=0;i<points.size();i++)
    {
        C3Vector transfPt(boxInv*points[i]);
        if (fabs(transfPt(0))>boxHs(0))
            return(false);
        if (fabs(transfPt(1))>boxHs(1))
            return(false);
        if (fabs(transfPt(2))>boxHs(2))
            return(false);
    }
    return(true);
}

bool CCalcUtils::doCollide_box_onept(const C4X4Matrix& box,const C3Vector& boxHs,const std::vector<C3Vector>& points,C3Vector* insidePt/*=nullptr*/)
{ // true is at least one point is within the box
    C4X4Matrix boxInv(box.getInverse());
    for (size_t i=0;i<points.size();i++)
    {
        C3Vector transfPt(boxInv*points[i]);
        if (fabs(transfPt(0))<=boxHs(0))
        {
            if (fabs(transfPt(1))<=boxHs(1))
            {
                if (fabs(transfPt(2))<=boxHs(2))
                {
                    if (insidePt!=nullptr)
                        insidePt[0]=points[i];
                    return(true);
                }
            }
        }
    }
    return(false);
}

bool CCalcUtils::doCollide_cell_segp(double cellHs,bool solidCell,const C3Vector& segP,const C3Vector& segL)
{   // Cell is at the origin
    // Segment is defined as segP (end-point1) and segP+segL (end-point2)
    C3Vector segHs(segL*0.5);
    C3Vector segCenter(segP+segHs);
    return(doCollide_cell_seg(cellHs,solidCell,segCenter,segHs));
}

bool CCalcUtils::doCollide_cell_pt(double cellHs,const C3Vector& pt)
{
    if (fabs(pt(0))>cellHs)
        return(false);
    if (fabs(pt(1))>cellHs)
        return(false);
    if (fabs(pt(2))>cellHs)
        return(false);
    return(true);
}

bool CCalcUtils::doCollide_cell_seg(double cellHs,bool solidCell,const C3Vector& segCenter,const C3Vector& segHs)
{   // Cell is at the origin
    // Segment is defined as segCenter (center of segment) and segHs (segment vector half-size)
    // optimize!!!
    C4X4Matrix box;
    box.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(doCollide_box_seg(box,sv,solidCell,segCenter,segHs));
}

bool CCalcUtils::doCollide_cell_tri(double cellHs,bool solidCell,const C3Vector& p,const C3Vector& v,const C3Vector& w)
{   // Cell is at the origin
    // optimize!!!
    C4X4Matrix box;
    box.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(doCollide_box_tri(box,sv,solidCell,p,v,w));
}

double CCalcUtils::getProjectedDistance_box_tri(const C3Vector& axis,const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& p,const C3Vector& v,const C3Vector& w)
{
    double boxPProjDist=axis*(p-box.X);
    C3Vector boxP(fabs(box.M.axis[0]*axis),fabs(box.M.axis[1]*axis),fabs(box.M.axis[2]*axis));
    double boxOverlap=boxP*boxHs;
    double triEdge1ProjDist=axis*v;
    double triEdge2ProjDist=axis*w;
    double overlapTriEdge1=0.0;
    double overlapTriEdge2=0.0;
    if (boxPProjDist*triEdge1ProjDist<0.0)
        overlapTriEdge1=fabs(triEdge1ProjDist);
    if (boxPProjDist*triEdge2ProjDist<0.0)
        overlapTriEdge2=fabs(triEdge2ProjDist);
    double retVal=fabs(boxPProjDist)-std::max<double>(overlapTriEdge1,overlapTriEdge2)-boxOverlap;
    if (retVal<0.0)
        retVal=0.0;
    return(retVal);
}

double CCalcUtils::getProjectedDistance_box_segp(const C3Vector& axis,const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segP,const C3Vector& segL)
{   // Segment is defined as segP (end-point1) and segP+segL (end-point2)
    C3Vector segHs(segL*0.5);
    C3Vector segCenter(segP+segHs);
    return(getProjectedDistance_box_seg(axis,box,boxHs,segCenter,segHs));
}

double CCalcUtils::getProjectedDistance_box_seg(const C3Vector& axis,const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segCenter,const C3Vector& segHs)
{   // Segment is defined as segCenter (center of segment) and segHs (segment vector half-size)
    double boxSegProjDist=fabs(axis*(segCenter-box.X));
    C3Vector boxP(fabs(box.M.axis[0]*axis),fabs(box.M.axis[1]*axis),fabs(box.M.axis[2]*axis));
    double boxOverlap=boxP*boxHs;
    double segOverlap=fabs(segHs*axis);
    double retVal=boxSegProjDist-boxOverlap-segOverlap;
    if (retVal<0.0)
        retVal=0.0;
    return(retVal);
}

double CCalcUtils::getProjectedDistance_box_pt(const C3Vector& axis,const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& pt)
{
    double boxSegProjDist=fabs(axis*(pt-box.X));
    C3Vector boxP(fabs(box.M.axis[0]*axis),fabs(box.M.axis[1]*axis),fabs(box.M.axis[2]*axis));
    double boxOverlap=boxP*boxHs;
    double retVal=boxSegProjDist-boxOverlap;
    if (retVal<0.0)
        retVal=0.0;
    return(retVal);
}

double CCalcUtils::getProjectedDistance_cell_tri(const C3Vector& axis,double cellHs,const C3Vector& p,const C3Vector& v,const C3Vector& w)
{   // Cell is at the origin
    // optimize!!!
    C4X4Matrix box;
    box.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(getProjectedDistance_box_tri(axis,box,sv,p,v,w));
}

double CCalcUtils::getProjectedDistance_cell_segp(const C3Vector& axis,double cellHs,const C3Vector& segP,const C3Vector& segL)
{   // Cell is at the origin
    // Segment is defined as segP (end-point1) and segP+segL (end-point2)
    C3Vector segHs(segL*0.5);
    C3Vector segCenter(segP+segHs);
    return(getProjectedDistance_cell_seg(axis,cellHs,segCenter,segHs));
}

double CCalcUtils::getProjectedDistance_cell_seg(const C3Vector& axis,double cellHs,const C3Vector& segCenter,const C3Vector& segHs)
{   // Cell is at the origin
    // Segment is defined as segCenter (center of segment) and segHs (segment vector half-size)
    // optimize!!!
    C4X4Matrix box;
    box.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(getProjectedDistance_box_seg(axis,box,sv,segCenter,segHs));
}

double CCalcUtils::getProjectedDistance_tri_tri(const C3Vector& axis,const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2)
{
    double p1p2ProjDist=axis*(p2-p1);
    double tri1Edge1ProjDist=axis*v1;
    double tri1Edge2ProjDist=axis*w1;
    double tri2Edge1ProjDist=axis*v2;
    double tri2Edge2ProjDist=axis*w2;
    double overlapTri1Edge1=0.0;
    double overlapTri1Edge2=0.0;
    if (p1p2ProjDist*tri1Edge1ProjDist>0.0)
        overlapTri1Edge1=fabs(tri1Edge1ProjDist);
    if (p1p2ProjDist*tri1Edge2ProjDist>0.0)
        overlapTri1Edge2=fabs(tri1Edge2ProjDist);
    double overlapTri2Edge1=0.0;
    double overlapTri2Edge2=0.0;
    if (p1p2ProjDist*tri2Edge1ProjDist<0.0)
        overlapTri2Edge1=fabs(tri2Edge1ProjDist);
    if (p1p2ProjDist*tri2Edge2ProjDist<0.0)
        overlapTri2Edge2=fabs(tri2Edge2ProjDist);
    double retVal=fabs(p1p2ProjDist)-std::max<double>(overlapTri1Edge1,overlapTri1Edge2)-std::max<double>(overlapTri2Edge1,overlapTri2Edge2);
    if (retVal<0.0)
        retVal=0.0;
    return(retVal);
}


bool CCalcUtils::isApproxDistanceSmaller_box_box(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    if (dist==0.0)
        return(false);
    double d=0.0;
    for (size_t i=0;i<3;i++)
    { // box 1's axes
        double box1Box2Proj=fabs((box2.X-box1.X)*box1.M.axis[i]);
        double box2Overlap=fabs(box2.M.axis[0]*box1.M.axis[i])*box2Hs(0)+fabs(box2.M.axis[1]*box1.M.axis[i])*box2Hs(1)+fabs(box2.M.axis[2]*box1.M.axis[i])*box2Hs(2);
        double dd=box1Box2Proj-box1Hs(i)-box2Overlap;
        if (dd>=dist)
            return(false);
        d=dd;
    }
    for (size_t i=0;i<3;i++)
    { // box 2's axes
        double box1Box2Proj=fabs((box2.X-box1.X)*box2.M.axis[i]);
        double box1Overlap=fabs(box1.M.axis[0]*box2.M.axis[i])*box1Hs(0)+fabs(box1.M.axis[1]*box2.M.axis[i])*box1Hs(1)+fabs(box1.M.axis[2]*box2.M.axis[i])*box1Hs(2);
        double dd=box1Box2Proj-box2Hs(i)-box1Overlap;
        if (dd>=dist)
            return(false);
        d=std::max<double>(d,dd);
    }
    for (size_t i=0;i<3;i++)
    {
        for (size_t j=0;j<3;j++)
        {
            C3Vector a(box1.M.axis[i]^box2.M.axis[j]);
            if ( (a(0)!=0.0)||(a(1)!=0.0)||(a(2)!=0.0) )
            {
                a.normalize();
                double box1Box2Proj=fabs((box2.X-box1.X)*a);
                double box1Overlap=fabs(box1.M.axis[0]*a)*box1Hs(0)+fabs(box1.M.axis[1]*a)*box1Hs(1)+fabs(box1.M.axis[2]*a)*box1Hs(2);
                double box2Overlap=fabs(box2.M.axis[0]*a)*box2Hs(0)+fabs(box2.M.axis[1]*a)*box2Hs(1)+fabs(box2.M.axis[2]*a)*box2Hs(2);
                double dd=box1Box2Proj-box1Overlap-box2Overlap;
                if (dd>=dist)
                    return(false);
                d=std::max<double>(d,dd);
            }
        }
    }
    if (d<0.0)
        d=0.0;
    dist=d;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_box_box_fast(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,double& dist)
{   // Similar to isApproxDistanceSmaller_box_box, but only one projection axis is tested (i.e. the axis between the two boxes' origin)
    // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    if (dist==0.0)
        return(false);
    C3Vector axis((box2.X-box1.X).getNormalized());
    double box1Box2Proj=fabs((box2.X-box1.X)*axis);
    double box1Overlap=fabs(box1.M.axis[0]*axis)*box1Hs(0)+fabs(box1.M.axis[1]*axis)*box1Hs(1)+fabs(box1.M.axis[2]*axis)*box1Hs(2);
    double box2Overlap=fabs(box2.M.axis[0]*axis)*box2Hs(0)+fabs(box2.M.axis[1]*axis)*box2Hs(1)+fabs(box2.M.axis[2]*axis)*box2Hs(2);
    double d=box1Box2Proj-box1Overlap-box2Overlap;
    if (d>=dist)
        return(false);
    if (d<0.0)
        d=0.0;
    dist=d;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_box_cell(const C4X4Matrix& box,const C3Vector& boxHs,double cellHs,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Cell is at the origin
    // optimize!!!
    if (dist==0.0)
        return(false);
    C4X4Matrix box2;
    box2.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(isApproxDistanceSmaller_box_box(box2,sv,box,boxHs,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_box_cell_fast(const C4X4Matrix& box,const C3Vector& boxHs,double cellHs,double& dist)
{   // Similar to isApproxDistanceSmaller_box_cell, but only one projection axis is tested (i.e. the axis between the two boxes' origin)
    // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Cell is at the origin
    // optimize!!!
    if (dist==0.0)
        return(false);
    C4X4Matrix box2;
    box2.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(isApproxDistanceSmaller_box_box_fast(box2,sv,box,boxHs,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_box_tri(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    if (dist==0.0)
        return(false);
    double d=0.0;
    // tri normal vect:
    C3Vector n((v^w).getNormalized());
    double dd=getProjectedDistance_box_tri(n,box,boxHs,p,v,w);
    if (dd>=dist)
        return(false);
    d=dd;
    // Box's edges:
    for (size_t i=0;i<3;i++)
    {
        double dd=getProjectedDistance_box_tri(box.M.axis[i],box,boxHs,p,v,w);
        if (dd>=dist)
            return(false);
        d=std::max<double>(d,dd);
    }
    // Now check the tri edges ^ box's axes:
    C3Vector u(w-v);
    C3Vector trEdges[3]={v,w,u};
    for (size_t i=0;i<3;i++)
    {
        for (size_t j=0;j<3;j++)
        {
            C3Vector axis=box.M.axis[i]^trEdges[j];
            if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
            {
                double dd=getProjectedDistance_box_tri(axis.getNormalized(),box,boxHs,p,v,w);
                if (dd>=dist)
                    return(false);
                d=std::max<double>(d,dd);
            }
        }
    }
    if (d<0.0)
        d=0.0;
    dist=d;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_box_tri_fast(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist)
{   // Similar to isApproxDistanceSmaller_box_tri, but only one projection axis is tested (i.e. the axis between the two boxes' origin)
    // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true

    if (dist==0.0)
        return(false);
    C3Vector axis((p-box.X).getNormalized());
    double d=getProjectedDistance_box_tri(axis,box,boxHs,p,v,w);
    if (d>=dist)
        return(false);
    dist=d;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_box_segp_fast(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segP,const C3Vector& segL,double& dist)
{   // Similar to isApproxDistanceSmaller_box_segp, but only one projection axis is tested (i.e. the axis between the two boxes' origin)
    // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Segment is defined as segP (end-point1) and segP+segL (end-point2)
    C3Vector segHs(segL*0.5);
    C3Vector segCenter(segP+segHs);
    return(isApproxDistanceSmaller_box_seg_fast(box,boxHs,segCenter,segHs,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_box_segp(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segP,const C3Vector& segL,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Segment is defined as segP (end-point1) and segP+segL (end-point2)
    C3Vector segHs(segL*0.5);
    C3Vector segCenter(segP+segHs);
    return(isApproxDistanceSmaller_box_seg(box,boxHs,segCenter,segHs,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_box_seg_fast(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segCenter,const C3Vector& segHs,double& dist)
{   // Similar to isApproxDistanceSmaller_box_seg, but only one projection axis is tested (i.e. the axis between the two boxes' origin)
    // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Segment is defined as segCenter (center of segment) and segHs (segment vector half-size)
    if (dist==0.0)
        return(false);
    double d=getProjectedDistance_box_seg((segCenter-box.X).getNormalized(),box,boxHs,segCenter,segHs);
    if (d>=dist)
        return(false);
    dist=d;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_box_pt_fast(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& pt,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    if (dist==0.0)
        return(false);
    double d=getProjectedDistance_box_pt((pt-box.X).getNormalized(),box,boxHs,pt);
    if (d>=dist)
        return(false);
    dist=d;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_box_seg(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segCenter,const C3Vector& segHs,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Segment is defined as segCenter (center of segment) and segHs (segment vector half-size)
    if (dist==0.0)
        return(false);
    double d=0.0;
    // Box-segment segment:
    double dd=getProjectedDistance_box_seg((segCenter-box.X).getNormalized(),box,boxHs,segCenter,segHs);
    if (dd>=dist)
        return(false);
    d=dd;
    // Segment axis:
    dd=getProjectedDistance_box_seg(segHs.getNormalized(),box,boxHs,segCenter,segHs);
    if (dd>=dist)
        return(false);
    d=std::max<double>(d,dd);
    // Box axes:
    for (size_t i=0;i<3;i++)
    {
        dd=getProjectedDistance_box_seg(box.M.axis[i],box,boxHs,segCenter,segHs);
        if (dd>=dist)
            return(false);
        d=std::max<double>(d,dd);
    }
    // Combination of Box and segment axes:
    for (size_t i=0;i<3;i++)
    {
        if (!segHs.isColinear(box.M.axis[i],double(0.99999)))
        {
            dd=getProjectedDistance_box_seg((segHs^box.M.axis[i]).getNormalized(),box,boxHs,segCenter,segHs);
            if (dd>=dist)
                return(false);
            d=std::max<double>(d,dd);
        }
    }
    if (d<0.0)
        d=0.0;
    dist=d;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_cell_tri(double cellHs,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Cell is at the origin
    // optimize!!!
    if (dist==0.0)
        return(false);
    C4X4Matrix box;
    box.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(isApproxDistanceSmaller_box_tri(box,sv,p,v,w,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_cell_tri_fast(double cellHs,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist)
{   // Similar to isApproxDistanceSmaller_cell_tri, but only one projection axis is tested (i.e. the axis between the two boxes' origin)
    // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Cell is at the origin
    // optimize!!!
    if (dist==0.0)
        return(false);
    C4X4Matrix box;
    box.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(isApproxDistanceSmaller_box_tri_fast(box,sv,p,v,w,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_cell_segp(double cellHs,const C3Vector& segP,const C3Vector& segL,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Cell is at the origin
    // Segment is defined as segP (end-point1) and segP+segL (end-point2)
    C3Vector segHs(segL*0.5);
    C3Vector segCenter(segP+segHs);
    return(isApproxDistanceSmaller_cell_seg(cellHs,segCenter,segHs,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_cell_segp_fast(double cellHs,const C3Vector& segP,const C3Vector& segL,double& dist)
{   // Similar to isApproxDistanceSmaller_cell_segp, but only one projection axis is tested (i.e. the axis between the two boxes' origin)
    // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Cell is at the origin
    // Segment is defined as segP (end-point1) and segP+segL (end-point2)
    C3Vector segHs(segL*0.5);
    C3Vector segCenter(segP+segHs);
    return(isApproxDistanceSmaller_cell_seg_fast(cellHs,segCenter,segHs,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_cell_seg(double cellHs,const C3Vector& segCenter,const C3Vector& segHs,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Cell is at the origin
    // Segment is defined as segCenter (center of segment) and segHs (segment vector half-size)
    // optimize!!!
    if (dist==0.0)
        return(false);
    C4X4Matrix box;
    box.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(isApproxDistanceSmaller_box_seg(box,sv,segCenter,segHs,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_cell_seg_fast(double cellHs,const C3Vector& segCenter,const C3Vector& segHs,double& dist)
{   // Similar to isApproxDistanceSmaller_cell_seg, but only one projection axis is tested (i.e. the axis between the two boxes' origin)
    // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    // Cell is at the origin
    // Segment is defined as segCenter (center of segment) and segHs (segment vector half-size)
    // optimize!!!
    if (dist==0.0)
        return(false);
    C4X4Matrix box;
    box.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(isApproxDistanceSmaller_box_seg_fast(box,sv,segCenter,segHs,dist));
}

bool CCalcUtils::isApproxDistanceSmaller_tri_tri(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    if (dist==0.0)
        return(false);

    double d=0.0;
    // Now check for separating axes:
    C3Vector u1(w1-v1);
    C3Vector u2(w2-v2);
    C3Vector tr1Edges[3]={v1,w1,u1};
    C3Vector tr2Edges[3]={v2,w2,u2};

    C3Vector n1((v1^w1).getNormalized());
    double dd=getProjectedDistance_tri_tri(n1,p1,v1,w1,p2,v2,w2);
    if (dd>=dist)
        return(false);
    d=dd;

    C3Vector n2((v2^w2).getNormalized());
    dd=getProjectedDistance_tri_tri(n2,p1,v1,w1,p2,v2,w2);
    if (dd>=dist)
        return(false);
    d=std::max<double>(d,dd);

    for (size_t i=0;i<3;i++)
    {
        for (size_t j=0;j<3;j++)
        {
            C3Vector axis=tr1Edges[i]^tr2Edges[j];
            if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
            { // Edges are not parallel
                dd=getProjectedDistance_tri_tri(axis.getNormalized(),p1,v1,w1,p2,v2,w2);
                if (dd>=dist)
                    return(false);
                d=std::max<double>(d,dd);
            }
        }
    }
    if (d<0.0)
        d=0.0;
    dist=d;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_tri_tri_fast(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,double& dist)
{   // Similar to isApproxDistanceSmaller_tri_tri, but only one projection axis is tested (i.e. the axis between the two boxes' origin)
    // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    if (dist==0.0)
        return(false);

    double d=getProjectedDistance_tri_tri((p2-p1).getNormalized(),p1,v1,w1,p2,v2,w2);
    if (d>=dist)
        return(false);
    dist=d;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_tri_pt_fast(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& pt,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    if (dist==0.0)
        return(false);

    C3Vector p_tri(p-pt);
    double l=p_tri.getLength();
    if (l>0.0)
    {
        p_tri/=l;
        double triS1=0.0;
        double triS1P=p_tri*v;
        if (triS1P<0.0)
            triS1=fabs(triS1P);
        double triS2=0.0;
        double triS2P=p_tri*w;
        if (triS2P<0.0)
            triS2=fabs(triS2P);
        double d=l-std::max<double>(triS1,triS2);
        if (d<dist)
        {
            if (d<0.0)
                d=0.0;
            dist=d;
            return(true);
        }
        return(false);
    }
    dist=0.0;
    return(true);
}

bool CCalcUtils::isApproxDistanceSmaller_segp_segp_fast(const C3Vector& seg1P,C3Vector seg1L,const C3Vector& seg2P,C3Vector seg2L,double& dist)
{   // The real distance is always larger or equal than the approximate distance
    // dist is modified if return is true
    if (dist==0.0)
        return(false);

    // check if projection on segCenter-segCenter axis is larger than dist
    seg1L*=0.5;
    seg2L*=0.5;
    C3Vector distS((seg1P+seg1L)-(seg2P+seg2L));
    double d=distS.getLength();
    if (d!=0.0)
    {
        distS=distS/d;
        d-=fabs(seg1L*distS);
        d-=fabs(seg2L*distS);
        if (d>=dist)
            return(false);
    }
    dist=d;
    return(true);
}

double CCalcUtils::getApproxDistance_box_box(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_box_box_fast(box1,box1Hs,box2,box2Hs,d);
    return(d);
}

double CCalcUtils::getApproxDistance_box_cell(const C4X4Matrix& box,const C3Vector& boxHs,double cellHs)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_box_cell_fast(box,boxHs,cellHs,d);
    return(d);
}

double CCalcUtils::getApproxDistance_box_tri(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& p,const C3Vector& v,const C3Vector& w)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_box_tri_fast(box,boxHs,p,v,w,d);
    return(d);
}

double CCalcUtils::getApproxDistance_box_seg(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segCenter,const C3Vector& segHs)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_box_seg_fast(box,boxHs,segCenter,segHs,d);
    return(d);
}

double CCalcUtils::getApproxDistance_box_segp(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segP,const C3Vector& segL)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_box_segp_fast(box,boxHs,segP,segL,d);
    return(d);
}

double CCalcUtils::getApproxDistance_box_pt(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& pt)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_box_pt_fast(box,boxHs,pt,d);
    return(d);
}

double CCalcUtils::getApproxDistance_cell_tri(double cellHs,const C3Vector& p,const C3Vector& v,const C3Vector& w)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_cell_tri_fast(cellHs,p,v,w,d);
    return(d);
}

double CCalcUtils::getApproxDistance_cell_seg(double cellHs,const C3Vector& segCenter,const C3Vector& segHs)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_cell_seg_fast(cellHs,segCenter,segHs,d);
    return(d);
}

double CCalcUtils::getApproxDistance_cell_segp(double cellHs,const C3Vector& segP,const C3Vector& segL)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_cell_segp_fast(cellHs,segP,segL,d);
    return(d);
}

double CCalcUtils::getApproxDistance_tri_tri(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2)
{   // Convenience function
    double d=DBL_MAX;
    isApproxDistanceSmaller_tri_tri_fast(p1,v1,w1,p2,v2,w2,d);
    return(d);
}

bool CCalcUtils::getDistance_box_box_alt(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,bool solidBoxes,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{   // optimize! Does simple triangle/triangle dist. calc. between boxes' triangles

    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double d=dist;
    if (!isApproxDistanceSmaller_box_box_fast(box1,box1Hs,box2,box2Hs,d))
        return(false);

    bool retVal=false;

    std::vector<C3Vector> b1pts;
    std::vector<C3Vector> b2pts;
    for (double z=-1.0;z<2.0;z+=2.0)
    {
        C3Vector a1;
        C3Vector a2;
        a1(2)=z*box1Hs(2);
        a2(2)=z*box2Hs(2);
        for (double y=-1.0;y<2.0;y+=2.0)
        {
            a1(1)=y*box1Hs(1);
            a2(1)=y*box2Hs(1);
            for (double x=-1.0;x<2.0;x+=2.0)
            {
                a1(0)=x*box1Hs(0);
                a2(0)=x*box2Hs(0);
                b1pts.push_back(box1*a1);
                b2pts.push_back(box2*a2);
            }
        }
    }

    bool intertweened=false;
    if (solidBoxes)
    {
        if (doCollide_box_onept(box1,box1Hs,b2pts,minDistSegPt1))
        {
            dist=0.0;
            if (minDistSegPt2!=nullptr)
                minDistSegPt2[0]=minDistSegPt1[0];
            return(true);
        }
        if (doCollide_box_onept(box2,box2Hs,b1pts,minDistSegPt2))
        {
            dist=0.0;
            if (minDistSegPt1!=nullptr)
                minDistSegPt1[0]=minDistSegPt2[0];
            return(true);
        }
    }
    else
    {
        if (doCollide_box_onept(box1,box1Hs,b2pts,nullptr))
            intertweened=true;
        else
        {
            if (doCollide_box_onept(box2,box2Hs,b1pts,nullptr))
                intertweened=true;
        }
    }

    // The triangle's indices are 'balanced':
    size_t triIndices[12*3]={0,1,5,4,5,7,7,2,6,0,3,1,1,3,7,0,6,2,0,5,4,4,7,6,7,3,2,0,2,3,1,7,5,0,4,6};

    C3Vector axis(box2.X-box1.X);
    for (size_t i=0;i<12;i++)
    {
        C3Vector p1(b1pts[triIndices[3*i+0]]);
        C3Vector v1(b1pts[triIndices[3*i+1]]-p1);
        C3Vector w1(b1pts[triIndices[3*i+2]]-p1);
        C3Vector n1=v1^w1;
        if ( (axis*n1>=0.0)||(intertweened) )
        { // only faces pointing towards box2
            for (size_t j=0;j<12;j++)
            {
                C3Vector p2(b2pts[triIndices[3*j+0]]);
                C3Vector v2(b2pts[triIndices[3*j+1]]-p2);
                C3Vector w2(b2pts[triIndices[3*j+2]]-p2);
                C3Vector n2=v2^w2;
                if ( (axis*n2<=0.0)||(intertweened) )
                { // only faces pointing towards box1
                    if ( (n1*n2<0.0)||(intertweened) )
                    { // only faces facing each other
                        if (getDistance_tri_tri(p1,v1,w1,p2,v2,w2,dist,minDistSegPt1,minDistSegPt2))
                        {
                            retVal=true;
                            if (dist<=0.0)
                                break;
                        }
                    }
                }
            }

        }
        if (retVal&&(dist<=0.0))
            break;
    }
    return(retVal);
}

bool CCalcUtils::getDistance_box_box(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,bool solidBoxes,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{ // computes the distance between two boxes
    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double d=dist;
    if (!isApproxDistanceSmaller_box_box_fast(box1,box1Hs,box2,box2Hs,d))
        return(false);

    bool retVal=false;
    C3Vector dirsA[3]={box1.M.axis[0]*box1Hs(0),box1.M.axis[1]*box1Hs(1),box1.M.axis[2]*box1Hs(2)};
    C3Vector dirsB[3]={box2.M.axis[0]*box2Hs(0),box2.M.axis[1]*box2Hs(1),box2.M.axis[2]*box2Hs(2)};
    C3Vector __p1A(box1.X-dirsA[0]);
    C3Vector __p2A(box1.X-dirsA[1]);
    C3Vector __p3A(box1.X-dirsA[2]);
    C3Vector __p1B(box2.X-dirsB[0]);
    C3Vector __p2B(box2.X-dirsB[1]);
    C3Vector __p3B(box2.X-dirsB[2]);
    for (double fi=-1.0;fi<2.0;fi+=2.0)
    {
        C3Vector _p1A(__p1A+(box1.M.axis[2]*(box1Hs(2)*fi)));
        C3Vector _p2A(__p2A+(box1.M.axis[2]*(box1Hs(2)*fi)));
        C3Vector _p3A(__p3A+(box1.M.axis[1]*(box1Hs(1)*fi)));
        C3Vector _p1B(__p1B+(box2.M.axis[2]*(box2Hs(2)*fi)));
        C3Vector _p2B(__p2B+(box2.M.axis[2]*(box2Hs(2)*fi)));
        C3Vector _p3B(__p3B+(box2.M.axis[1]*(box2Hs(1)*fi)));
        for (double fj=-1.0;fj<2.0;fj+=2.0)
        {
            C3Vector p1A(_p1A+(box1.M.axis[1]*(box1Hs(1)*fj)));
            C3Vector p2A(_p2A+(box1.M.axis[0]*(box1Hs(0)*fj)));
            C3Vector p3A(_p3A+(box1.M.axis[0]*(box1Hs(0)*fj)));
            C3Vector p1B(_p1B+(box2.M.axis[1]*(box2Hs(1)*fj)));
            C3Vector p2B(_p2B+(box2.M.axis[0]*(box2Hs(0)*fj)));
            C3Vector p3B(_p3B+(box2.M.axis[0]*(box2Hs(0)*fj)));
            bool b1=getDistance_box_segp(box2,box2Hs,solidBoxes,p1A,dirsA[0]*2.0,dist,minDistSegPt2,minDistSegPt1);
            bool b2=getDistance_box_segp(box2,box2Hs,solidBoxes,p2A,dirsA[1]*2.0,dist,minDistSegPt2,minDistSegPt1);
            bool b3=getDistance_box_segp(box2,box2Hs,solidBoxes,p3A,dirsA[2]*2.0,dist,minDistSegPt2,minDistSegPt1);
            bool b4=getDistance_box_segp(box1,box1Hs,solidBoxes,p1B,dirsB[0]*2.0,dist,minDistSegPt1,minDistSegPt2);
            bool b5=getDistance_box_segp(box1,box1Hs,solidBoxes,p2B,dirsB[1]*2.0,dist,minDistSegPt1,minDistSegPt2);
            bool b6=getDistance_box_segp(box1,box1Hs,solidBoxes,p3B,dirsB[2]*2.0,dist,minDistSegPt1,minDistSegPt2);
            retVal=retVal||b1||b2||b3||b4||b5||b6;
            if (dist==0.0)
                return(retVal);
        }
    }
    return(retVal);
}

bool CCalcUtils::getDistance_box_tri_alt(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{   // optimize! Does simple triangle/triangle dist. calc. between boxs' triangles and triangle

    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double d=dist;
    if (!isApproxDistanceSmaller_box_tri_fast(box,boxHs,p,v,w,d))
        return(false);

    bool retVal=false;
    if (solidBox)
    {
        std::vector<C3Vector> _pts;
        _pts.push_back(p);
        _pts.push_back(p+v);
        _pts.push_back(p+w);
        if (doCollide_box_onept(box,boxHs,_pts,minDistSegPt1))
        {
            dist=0.0;
            if (minDistSegPt2!=nullptr)
                minDistSegPt2[0]=minDistSegPt1[0];
            return(true);
        }
    }

    C3Vector bpts[8];
    size_t pos=0;
    for (double z=-1.0;z<2.0;z+=2.0)
    {
        C3Vector a;
        a(2)=z*boxHs(2);
        for (double y=-1.0;y<2.0;y+=2.0)
        {
            a(1)=y*boxHs(1);
            for (double x=-1.0;x<2.0;x+=2.0)
            {
                a(0)=x*boxHs(0);
                bpts[pos++]=C3Vector(box*a);
            }
        }
    }

    // The triangle's indices are 'balanced':
    size_t triIndices[12*3]={0,1,5,4,5,7,7,2,6,0,3,1,1,3,7,0,6,2,0,5,4,4,7,6,7,3,2,0,2,3,1,7,5,0,4,6};

    for (size_t i=0;i<12;i++)
    {
        C3Vector bp(bpts[triIndices[3*i+0]]);
        C3Vector bv(bpts[triIndices[3*i+1]]-bp);
        C3Vector bw(bpts[triIndices[3*i+2]]-bp);
        bool bb=getDistance_tri_tri(bp,bv,bw,p,v,w,dist,minDistSegPt1,minDistSegPt2);
        retVal=retVal||bb;
        if (dist==0.0)
            break;
    }
    return(retVal);
}

bool CCalcUtils::getDistance_box_tri(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{ // computes the distance between a box and a triangle.
    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double d=dist;
    if (!isApproxDistanceSmaller_box_tri_fast(box,boxHs,p,v,w,d))
        return(false);

    bool retVal=false;
    bool pt1Inside,pt2Inside,pt3Inside;
    if (getDistance_box_pt(box,boxHs,solidBox,p,dist,minDistSegPt1,&pt1Inside))
    {
        retVal=true;
        if (minDistSegPt2!=nullptr)
            minDistSegPt2[0]=p;
    }
    if (getDistance_box_pt(box,boxHs,solidBox,p+v,dist,minDistSegPt1,&pt2Inside))
    {
        retVal=true;
        if (minDistSegPt2!=nullptr)
            minDistSegPt2[0]=p+v;
    }
    if (getDistance_box_pt(box,boxHs,solidBox,p+w,dist,minDistSegPt1,&pt3Inside))
    {
        retVal=true;
        if (minDistSegPt2!=nullptr)
            minDistSegPt2[0]=p+w;
    }
    if (pt1Inside&&pt2Inside&&pt3Inside)
        return(retVal);
    if ( solidBox&&(pt1Inside||pt2Inside||pt3Inside) )
        return(retVal);

    C4X4Matrix boxInv(box.getInverse());
    C3Vector _p(boxInv*p);
    boxInv.X.clear();
    C3Vector _v(boxInv*v);
    C3Vector _w(boxInv*w);
    C3Vector _wv(_w-_v);

    C3Vector _segP[3]={_p,_p,_p+_v};
    C3Vector _segL[3]={_v,_w,_wv};

    // We now check box's faces vs tri segments:
    size_t axesInd[3];
    C3Vector firstAxis,secondAxis,thirdAxis;
    for (int i=0;i<3;i++)
    { // for each face orientation
        if (i==0)
        {
            axesInd[0]=0;
            axesInd[1]=1;
            axesInd[2]=2;
            firstAxis=C3Vector::unitXVector;
            secondAxis=C3Vector::unitYVector;
            thirdAxis=C3Vector::unitZVector;
        }
        if (i==1)
        {
            axesInd[0]=1;
            axesInd[1]=2;
            axesInd[2]=0;
            firstAxis=C3Vector::unitYVector;
            secondAxis=C3Vector::unitZVector;
            thirdAxis=C3Vector::unitXVector;
        }
        if (i==2)
        {
            axesInd[0]=2;
            axesInd[1]=0;
            axesInd[2]=1;
            firstAxis=C3Vector::unitZVector;
            secondAxis=C3Vector::unitXVector;
            thirdAxis=C3Vector::unitYVector;
        }

        for (size_t j=0;j<3;j++)
        { // for each triangle edge:
            C3Vector segPP(_segP[j]);
            C3Vector segLL(_segL[j]);
            // Does the segment maybe collide with that rect. face?
            if (segLL(axesInd[0])!=0.0)
            { // ok, segment is not parallel to that face
                for (double fi=-1.0;fi<2.0;fi+=2.0)
                { // for each of the two faces with the same orientation
                    double off=fi*boxHs(axesInd[0]);
                    double t=(off-segPP(axesInd[0]))/segLL(axesInd[0]);
                    if ( (t>=0.0)&&(t<=1.0) )
                    { // the segment collides with that plane. Now check the 2 other axes, if we are within bounds:
                        C3Vector intersection(segPP+(segLL*t));
                        if ( fabs(intersection(axesInd[1]))<=boxHs(axesInd[1]) )
                        {
                            if ( fabs(intersection(axesInd[2]))<=boxHs(axesInd[2]) )
                            { // yes, we are within bounds. The segment collides with one of the box's face:
                                retVal=true;//dist>0.0;
                                dist=0.0;
                                if (minDistSegPt1!=nullptr)
                                    minDistSegPt1[0]=box*intersection;
                                if (minDistSegPt2!=nullptr)
                                    minDistSegPt2[0]=box*intersection;
                                return(retVal);
                            }
                        }
                    }
                }
            }
            // Now check the 4 edges that link those two faces with the same orientation, VS the segment,
            // and the 4 edges vs tri surface:
            C3Vector edgeL(firstAxis*boxHs(axesInd[0])*2.0);
            for (double f1=-1.0;f1<2.0;f1+=2.0)
            {
                for (double f2=-1.0;f2<2.0;f2+=2.0)
                {
                    C3Vector edgeP(firstAxis*-boxHs(axesInd[0])+secondAxis*boxHs(axesInd[1])*f1+thirdAxis*boxHs(axesInd[2])*f2);
                    bool b1=getDistance_segp_segp(edgeP,edgeL,segPP,segLL,dist,minDistSegPt1,minDistSegPt2);
                    bool b2=getDistance_tris_segp(_p,_v,_w,edgeP,edgeL,dist,minDistSegPt2,minDistSegPt1);
                    if (b1||b2)
                    {
                        retVal=true;
                        if (minDistSegPt1!=nullptr)
                            minDistSegPt1[0]=box*minDistSegPt1[0];
                        if (minDistSegPt2!=nullptr)
                            minDistSegPt2[0]=box*minDistSegPt2[0];
                    }
                }
            }
        }
    }

    return(retVal);
}

bool CCalcUtils::getDistance_box_segp_alt(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& segP,const C3Vector& segL,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{   // optimize! Does simple triangle/seg dist. calc. between boxs' triangles and segment
    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double d=dist;
    if (!isApproxDistanceSmaller_box_segp_fast(box,boxHs,segP,segL,d))
        return(false);

    bool retVal=false;
    if (solidBox)
    {
        std::vector<C3Vector> _pts;
        _pts.push_back(segP);
        _pts.push_back(segP+segL);
        if (doCollide_box_onept(box,boxHs,_pts,minDistSegPt1))
        {
            dist=0.0;
            if (minDistSegPt2!=nullptr)
                minDistSegPt2[0]=minDistSegPt1[0];
            return(true);
        }
    }

    C3Vector bpts[8];
    size_t pos=0;
    for (double z=-1.0;z<2.0;z+=2.0)
    {
        C3Vector a;
        a(2)=z*boxHs(2);
        for (double y=-1.0;y<2.0;y+=2.0)
        {
            a(1)=y*boxHs(1);
            for (double x=-1.0;x<2.0;x+=2.0)
            {
                a(0)=x*boxHs(0);
                bpts[pos++]=C3Vector(box*a);
            }
        }
    }
    // The triangle's indices are 'balanced':
    size_t triIndices[12*3]={0,1,5,4,5,7,7,2,6,0,3,1,1,3,7,0,6,2,0,5,4,4,7,6,7,3,2,0,2,3,1,7,5,0,4,6};

    for (size_t i=0;i<12;i++)
    {
        C3Vector bp(bpts[triIndices[3*i+0]]);
        C3Vector bv(bpts[triIndices[3*i+1]]-bp);
        C3Vector bw(bpts[triIndices[3*i+2]]-bp);
        bool bb=getDistance_tri_segp(bp,bv,bw,segP,segL,dist,minDistSegPt1,minDistSegPt2);
        retVal=retVal||bb;
        if (dist==0.0)
            break;
    }
    return(retVal);
}

bool CCalcUtils::getDistance_box_segp(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& segP,const C3Vector& segL,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{ // computes the distance between a box and a segment.
    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double d=dist;
    if (!isApproxDistanceSmaller_box_segp_fast(box,boxHs,segP,segL,d))
        return(false);

    bool retVal=false;
    bool pt1Inside,pt2Inside;
    if (getDistance_box_pt(box,boxHs,solidBox,segP,dist,minDistSegPt1,&pt1Inside))
    {
        retVal=true;
        if (minDistSegPt2!=nullptr)
            minDistSegPt2[0]=segP;
    }
    if (getDistance_box_pt(box,boxHs,solidBox,segP+segL,dist,minDistSegPt1,&pt2Inside))
    {
        retVal=true;
        if (minDistSegPt2!=nullptr)
            minDistSegPt2[0]=segP+segL;
    }
    if (pt1Inside&&pt2Inside)
        return(retVal);
    if ( solidBox&&(pt1Inside||pt2Inside) )
        return(retVal);

    C4X4Matrix boxInv(box.getInverse());
    C3Vector segPP(boxInv*segP);
    boxInv.X.clear();
    C3Vector segLL(boxInv*segL);

    // We now check box's faces vs segment:
    size_t axesInd[3];
    C3Vector firstAxis,secondAxis,thirdAxis;
    for (int i=0;i<3;i++)
    { // for each face orientation
        if (i==0)
        {
            axesInd[0]=0;
            axesInd[1]=1;
            axesInd[2]=2;
            firstAxis=C3Vector::unitXVector;
            secondAxis=C3Vector::unitYVector;
            thirdAxis=C3Vector::unitZVector;
        }
        if (i==1)
        {
            axesInd[0]=1;
            axesInd[1]=2;
            axesInd[2]=0;
            firstAxis=C3Vector::unitYVector;
            secondAxis=C3Vector::unitZVector;
            thirdAxis=C3Vector::unitXVector;
        }
        if (i==2)
        {
            axesInd[0]=2;
            axesInd[1]=0;
            axesInd[2]=1;
            firstAxis=C3Vector::unitZVector;
            secondAxis=C3Vector::unitXVector;
            thirdAxis=C3Vector::unitYVector;
        }

        // Does the segment maybe collide with that rect. face?
        if (segLL(axesInd[0])!=0.0)
        { // ok, segment is not parallel to that face
            for (double fi=-1.0;fi<2.0;fi+=2.0)
            { // for each of the two faces with the same orientation
                double off=fi*boxHs(axesInd[0]);
                double t=(off-segPP(axesInd[0]))/segLL(axesInd[0]);
                if ( (t>=0.0)&&(t<=1.0) )
                { // the segment collides with that plane. Now check the 2 other axes, if we are within bounds:
                    C3Vector intersection(segPP+(segLL*t));
                    if ( fabs(intersection(axesInd[1]))<=boxHs(axesInd[1]) )
                    {
                        if ( fabs(intersection(axesInd[2]))<=boxHs(axesInd[2]) )
                        { // yes, we are within bounds. The segment collides with one of the box's face:
                            retVal=dist>0.0;
                            dist=0.0;
                            if (minDistSegPt1!=nullptr)
                                minDistSegPt1[0]=box*intersection;
                            if (minDistSegPt2!=nullptr)
                                minDistSegPt2[0]=box*intersection;
                            return(retVal);
                        }
                    }
                }
            }
        }
        // Now check the 4 edges that link those two faces with the same orientation, VS the segment:
        C3Vector edgeL(firstAxis*boxHs(axesInd[0])*2.0);
        for (double f1=-1.0;f1<2.0;f1+=2.0)
        {
            for (double f2=-1.0;f2<2.0;f2+=2.0)
            {
                C3Vector edgeP(firstAxis*-boxHs(axesInd[0])+secondAxis*boxHs(axesInd[1])*f1+thirdAxis*boxHs(axesInd[2])*f2);
                if (getDistance_segp_segp(edgeP,edgeL,segPP,segLL,dist,minDistSegPt1,minDistSegPt2))
                {
                    retVal=true;
                    if (minDistSegPt1!=nullptr)
                        minDistSegPt1[0]=box*minDistSegPt1[0];
                    if (minDistSegPt2!=nullptr)
                        minDistSegPt2[0]=box*minDistSegPt2[0];
                }
            }
        }
    }
    return(retVal);
}

bool CCalcUtils::getDistance_cell_segp(double cellHs,bool solidCell,const C3Vector& segP,const C3Vector& segL,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{ // computes the distance between a cube at the origine and a segment.
/*
    C4X4Matrix ident;
    ident.setIdentity();
    return(getDistance_box_segp_alt(ident,C3Vector(cellHs*0.5,cellHs*0.5,cellHs*0.5),solidCell,segP,segL,dist,minDistSegPt1,minDistSegPt2));
*/
    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double d=dist;
    if (!isApproxDistanceSmaller_cell_segp_fast(cellHs,segP,segL,d))
        return(false);

    bool retVal=false;
    bool pt1Inside,pt2Inside;
    if (getDistance_cell_pt(cellHs,solidCell,segP,dist,minDistSegPt1,&pt1Inside))
    {
        retVal=true;
        if (minDistSegPt2!=nullptr)
            minDistSegPt2[0]=segP;
    }
    if (getDistance_cell_pt(cellHs,solidCell,segP+segL,dist,minDistSegPt1,&pt2Inside))
    {
        retVal=true;
        if (minDistSegPt2!=nullptr)
            minDistSegPt2[0]=segP+segL;
    }
    if (pt1Inside&&pt2Inside)
        return(retVal);
    if ( solidCell&&(pt1Inside||pt2Inside) )
        return(retVal);

    // We now check box's faces vs segment:
    size_t axesInd[3];
    C3Vector firstAxis,secondAxis,thirdAxis;
    for (int i=0;i<3;i++)
    { // for each face orientation
        if (i==0)
        {
            axesInd[0]=0;
            axesInd[1]=1;
            axesInd[2]=2;
            firstAxis=C3Vector::unitXVector;
            secondAxis=C3Vector::unitYVector;
            thirdAxis=C3Vector::unitZVector;
        }
        if (i==1)
        {
            axesInd[0]=1;
            axesInd[1]=2;
            axesInd[2]=0;
            firstAxis=C3Vector::unitYVector;
            secondAxis=C3Vector::unitZVector;
            thirdAxis=C3Vector::unitXVector;
        }
        if (i==2)
        {
            axesInd[0]=2;
            axesInd[1]=0;
            axesInd[2]=1;
            firstAxis=C3Vector::unitZVector;
            secondAxis=C3Vector::unitXVector;
            thirdAxis=C3Vector::unitYVector;
        }

        // Does the segment maybe collide with that rect. face?
        if (segL(axesInd[0])!=0.0)
        { // ok, segment is not parallel to that face
            for (double fi=-1.0;fi<2.0;fi+=2.0)
            { // for each of the two faces with the same orientation
                double off=fi*cellHs;
                double t=(off-segP(axesInd[0]))/segL(axesInd[0]);
                if ( (t>=0.0)&&(t<=1.0) )
                { // the segment collides with that plane. Now check the 2 other axes, if we are within bounds:
                    C3Vector intersection(segP+(segL*t));
                    if ( fabs(intersection(axesInd[1]))<=cellHs )
                    {
                        if ( fabs(intersection(axesInd[2]))<=cellHs )
                        { // yes, we are within bounds. The segment collides with one of the box's face:
                            retVal=dist>0.0;
                            dist=0.0;
                            if (minDistSegPt1!=nullptr)
                                minDistSegPt1[0]=intersection;
                            if (minDistSegPt2!=nullptr)
                                minDistSegPt2[0]=intersection;
                            return(retVal);
                        }
                    }
                }
            }
        }
        // Now check the 4 edges that link those two faces with the same orientation, VS the segment:
        C3Vector edgeL(firstAxis*cellHs*2.0);
        for (double f1=-1.0;f1<2.0;f1+=2.0)
        {
            for (double f2=-1.0;f2<2.0;f2+=2.0)
            {
                C3Vector edgeP(firstAxis*-cellHs+secondAxis*cellHs*f1+thirdAxis*cellHs*f2);
                if (getDistance_segp_segp(edgeP,edgeL,segP,segL,dist,minDistSegPt1,minDistSegPt2))
                {
                    retVal=true;
                    if (minDistSegPt1!=nullptr)
                        minDistSegPt1[0]=minDistSegPt1[0];
                    if (minDistSegPt2!=nullptr)
                        minDistSegPt2[0]=minDistSegPt2[0];
                }
            }
        }
    }
    return(retVal);
}

bool CCalcUtils::getDistance_box_pt(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& pt,double& dist,C3Vector* minDistSegPt,bool* ptIsInside)
{
    if (dist==0.0)
        return(false);
    C3Vector transfPt(box.getInverse()*pt);
    C3Vector transfBoxPt(transfPt);
    if (fabs(transfBoxPt(0))>boxHs(0))
        transfBoxPt(0)*=boxHs(0)/fabs(transfBoxPt(0));
    if (fabs(transfBoxPt(1))>boxHs(1))
        transfBoxPt(1)*=boxHs(1)/fabs(transfBoxPt(1));
    if (fabs(transfBoxPt(2))>boxHs(2))
        transfBoxPt(2)*=boxHs(2)/fabs(transfBoxPt(2));
    double d=(transfBoxPt-transfPt).getLength();
    if (ptIsInside!=nullptr)
        ptIsInside[0]=(d==0.0);
    if ( (d==0.0)&&(!solidBox) )
    { // we are inside the box, and want a distance to the walls
        transfBoxPt=transfPt;
        C3Vector dd( boxHs(0)-fabs(transfBoxPt(0)),boxHs(1)-fabs(transfBoxPt(1)),boxHs(2)-fabs(transfBoxPt(2)) );
        size_t a=0;
        if (dd(1)<dd(0))
        {
            if (dd(2)<dd(1))
                a=2;
            else
                a=1;
        }
        else
        {
            if (dd(2)<dd(0))
                a=2;
        }
        if (transfBoxPt(a)>=0.0)
            transfBoxPt(a)=boxHs(a);
        else
            transfBoxPt(a)=-boxHs(a);
        d=(transfBoxPt-transfPt).getLength();
    }
    if (d<dist)
    {
        dist=d;
        if (minDistSegPt!=nullptr)
            minDistSegPt[0]=box*transfBoxPt;
        return(true);
    }
    return(false);
}

bool CCalcUtils::getDistance_cell_pt(double cellHs,bool solidCell,const C3Vector& pt,double& dist,C3Vector* minDistSegPt,bool* ptIsInside)
{
    if (dist==0.0)
        return(false);
    C3Vector transfBoxPt(pt);
    if (fabs(transfBoxPt(0))>cellHs)
        transfBoxPt(0)*=cellHs/fabs(transfBoxPt(0));
    if (fabs(transfBoxPt(1))>cellHs)
        transfBoxPt(1)*=cellHs/fabs(transfBoxPt(1));
    if (fabs(transfBoxPt(2))>cellHs)
        transfBoxPt(2)*=cellHs/fabs(transfBoxPt(2));
    double d=(transfBoxPt-pt).getLength();
    if (ptIsInside!=nullptr)
        ptIsInside[0]=(d==0.0);
    if ( (d==0.0)&&(!solidCell) )
    { // we are inside the cell, and want a distance to the walls
        transfBoxPt=pt;
        C3Vector dd( cellHs-fabs(transfBoxPt(0)),cellHs-fabs(transfBoxPt(1)),cellHs-fabs(transfBoxPt(2)) );
        size_t a=0;
        if (dd(1)<dd(0))
        {
            if (dd(2)<dd(1))
                a=2;
            else
                a=1;
        }
        else
        {
            if (dd(2)<dd(0))
                a=2;
        }
        if (transfBoxPt(a)>=0.0)
            transfBoxPt(a)=cellHs;
        else
            transfBoxPt(a)=-cellHs;
        d=(transfBoxPt-pt).getLength();
    }
    if (d<dist)
    {
        dist=d;
        if (minDistSegPt!=nullptr)
            minDistSegPt[0]=transfBoxPt;
        return(true);
    }
    return(false);
}

bool CCalcUtils::getDistance_cell_cell(const C4X4Matrix& cell1,double cell1Hs,const C4X4Matrix& cell2,double cell2Hs,bool solidCells,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{
    return(getDistance_box_box(cell1,C3Vector(cell1Hs,cell1Hs,cell1Hs),cell2,C3Vector(cell2Hs,cell2Hs,cell2Hs),solidCells,dist,minDistSegPt1,minDistSegPt2));
}

bool CCalcUtils::getDistance_cell_tri(double cellHs,bool solidCell,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{   // Cell is at the origin
    // optimize!!!
    C4X4Matrix box;
    box.setIdentity();
    C3Vector sv(cellHs,cellHs,cellHs);
    return(getDistance_box_tri(box,sv,solidCell,p,v,w,dist,minDistSegPt1,minDistSegPt2));
}


bool CCalcUtils::getDistance_tri_tri(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{
    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double projectionDistance=dist;
    if (!isApproxDistanceSmaller_tri_tri_fast(p1,v1,w1,p2,v2,w2,projectionDistance))
        return(false);

    bool retVal=false;
    C3Vector pv1(p1+v1);
    C3Vector wv1(w1-v1);
    C3Vector pv2(p2+v2);
    C3Vector wv2(w2-v2);

    if (projectionDistance==0.0)
    { // the two triangles could collide... do the slower calculations:
        // triangle surface vs triangle edges:
        bool bb1=getDistance_tris_segp(p1,v1,w1,p2,v2,dist,minDistSegPt1,minDistSegPt2);
        bool bb2=getDistance_tris_segp(p1,v1,w1,p2,w2,dist,minDistSegPt1,minDistSegPt2);
        bool bb3=getDistance_tris_segp(p1,v1,w1,pv2,wv2,dist,minDistSegPt1,minDistSegPt2);
        bool bb4=getDistance_tris_segp(p2,v2,w2,p1,v1,dist,minDistSegPt2,minDistSegPt1);
        bool bb5=getDistance_tris_segp(p2,v2,w2,p1,w1,dist,minDistSegPt2,minDistSegPt1);
        bool bb6=getDistance_tris_segp(p2,v2,w2,pv1,wv1,dist,minDistSegPt2,minDistSegPt1);
        retVal=retVal||bb1||bb2||bb3||bb4||bb5||bb6;
        if (dist==0.0)
            return(retVal);
    }
    else
    { // the two triangles can not collide... do the faster calculations:
        // triangle surface vs triangle points:
        if (getDistance_tris_pt(p1,v1,w1,p2,dist,minDistSegPt1))
        {
            retVal=true;
            if (minDistSegPt2!=nullptr)
                minDistSegPt2[0]=p2;
        }
        if (getDistance_tris_pt(p1,v1,w1,pv2,dist,minDistSegPt1))
        {
            retVal=true;
            if (minDistSegPt2!=nullptr)
                minDistSegPt2[0]=pv2;
        }
        if (getDistance_tris_pt(p1,v1,w1,p2+w2,dist,minDistSegPt1))
        {
            retVal=true;
            if (minDistSegPt2!=nullptr)
                minDistSegPt2[0]=p2+w2;
        }
        if (getDistance_tris_pt(p2,v2,w2,p1,dist,minDistSegPt2))
        {
            retVal=true;
            if (minDistSegPt1!=nullptr)
                minDistSegPt1[0]=p1;
        }
        if (getDistance_tris_pt(p2,v2,w2,pv1,dist,minDistSegPt2))
        {
            retVal=true;
            if (minDistSegPt1!=nullptr)
                minDistSegPt1[0]=pv1;
        }
        if (getDistance_tris_pt(p2,v2,w2,p1+w1,dist,minDistSegPt2))
        {
            retVal=true;
            if (minDistSegPt1!=nullptr)
                minDistSegPt1[0]=p1+w1;
        }
    }

    // triangle edges vs triangle edges:
    bool bb7=getDistance_segp_segp(p1,v1,p2,v2,dist,minDistSegPt1,minDistSegPt2);
    bool bb8=getDistance_segp_segp(p1,v1,p2,w2,dist,minDistSegPt1,minDistSegPt2);
    bool bb9=getDistance_segp_segp(p1,v1,pv2,wv2,dist,minDistSegPt1,minDistSegPt2);
    bool bb10=getDistance_segp_segp(p1,w1,p2,v2,dist,minDistSegPt1,minDistSegPt2);
    bool bb11=getDistance_segp_segp(p1,w1,p2,w2,dist,minDistSegPt1,minDistSegPt2);
    bool bb12=getDistance_segp_segp(p1,w1,pv2,wv2,dist,minDistSegPt1,minDistSegPt2);
    bool bb13=getDistance_segp_segp(pv1,wv1,p2,v2,dist,minDistSegPt1,minDistSegPt2);
    bool bb14=getDistance_segp_segp(pv1,wv1,p2,w2,dist,minDistSegPt1,minDistSegPt2);
    bool bb15=getDistance_segp_segp(pv1,wv1,pv2,wv2,dist,minDistSegPt1,minDistSegPt2);
    retVal=retVal||bb7||bb8||bb9||bb10||bb11||bb12||bb13||bb14||bb15;

    return(retVal);
}

bool CCalcUtils::getDistance_tri_segp(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segP,const C3Vector& segL,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{
    if (dist==0.0)
        return(false);

    // triangle surface vs segment:
    bool retVal=getDistance_tris_segp(p,v,w,segP,segL,dist,minDistSegPt1,minDistSegPt2);

    // triangle edges vs segment:
    bool b1=getDistance_segp_segp(p,v,segP,segL,dist,minDistSegPt1,minDistSegPt2);
    bool b2=getDistance_segp_segp(p,w,segP,segL,dist,minDistSegPt1,minDistSegPt2);
    bool b3=getDistance_segp_segp(p+v,w-v,segP,segL,dist,minDistSegPt1,minDistSegPt2);
    retVal=retVal||b1||b2||b3;
    return(retVal);
}

bool CCalcUtils::getDistance_tri_pt(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& pt,double& dist,C3Vector* minDistSegPt)
{
    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double d=dist;
    if (!isApproxDistanceSmaller_tri_pt_fast(p,v,w,pt,d))
        return(false);

    bool bb1=getDistance_tris_pt(p,v,w,pt,dist,minDistSegPt);
    bool bb2=getDistance_segp_pt(p,v,pt,dist,minDistSegPt);
    bool bb3=getDistance_segp_pt(p,w,pt,dist,minDistSegPt);
    bool bb4=getDistance_segp_pt(p+v,w-v,pt,dist,minDistSegPt);
    return(bb1||bb2||bb3||bb4);
}

bool CCalcUtils::getDistance_segp_segp(const C3Vector& seg1P,const C3Vector& seg1L,const C3Vector& seg2P,const C3Vector& seg2L,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{
    if (dist==0.0)
        return(false);

    // do first an approximate calculation:
    double d=dist;
    if (!isApproxDistanceSmaller_segp_segp_fast(seg1P,seg1L,seg2P,seg2L,d))
        return(false);

    double t1,t2;
    C3Vector c(seg1L^seg2L);

    if ( (c(0)==0.0)&&(c(1)==0.0)&&(c(2)==0.0) )
    {   // Segments are colinear
        t1=0.0;
        t2=(seg1P*seg1L)-(seg1L*seg2P)/(seg1L*seg2L);
        double l=seg2L.getLength()/seg1L.getLength();
        if ( (seg1L*seg2L)<0.0 )
            l*=-1.0;
        if (t2<0.0)
        {
            t1=-t2*l;
            t2=0.0;
        }
        else
        {
            if (t2>1.0)
            {
                t1=-(t2-1.0)*l;
                t2=1.0;
            }
        }
    }
    else
    {   // Segments not colinear
        // plane with line 1 intersection with line 2:
        C3Vector perp(c^seg1L);
        double l=seg1P*perp;
        t2=(l-(perp*seg2P))/(perp*seg2L);
        // plane with line 2 intersection with line 1:
        perp=c^seg2L;
        l=seg2P*perp;
        t1=(l-(perp*seg1P))/(perp*seg1L);
    }

    bool retVal=false;
    if ( (t1<0.0)||(t1>1.0)||(t2<0.0)||(t2>1.0) )
    {
        if (getDistance_segp_pt(seg2P,seg2L,seg1P,dist,minDistSegPt2))
        {
            retVal=true;
            if (minDistSegPt1!=nullptr)
                minDistSegPt1[0]=seg1P;
        }
        if (getDistance_segp_pt(seg2P,seg2L,seg1P+seg1L,dist,minDistSegPt2))
        {
            retVal=true;
            if (minDistSegPt1!=nullptr)
                minDistSegPt1[0]=seg1P+seg1L;
        }
        if (getDistance_segp_pt(seg1P,seg1L,seg2P,dist,minDistSegPt1))
        {
            retVal=true;
            if (minDistSegPt2!=nullptr)
                minDistSegPt2[0]=seg2P;
        }
        if (getDistance_segp_pt(seg1P,seg1L,seg2P+seg2L,dist,minDistSegPt1))
        {
            retVal=true;
            if (minDistSegPt2!=nullptr)
                minDistSegPt2[0]=seg2P+seg2L;
        }
    }
    else
    { // the min. dist pts are within the two segments bounds
        C3Vector p1(seg1P+seg1L*t1);
        C3Vector p2(seg2P+seg2L*t2);
        double d=(p1-p2).getLength();
        if (d<dist)
        {
            dist=d;
            retVal=true;
            if (minDistSegPt1!=nullptr)
                minDistSegPt1[0]=p1;
            if (minDistSegPt2!=nullptr)
                minDistSegPt2[0]=p2;
        }
    }
    return(retVal);
}

bool CCalcUtils::getDistance_segp_pt(const C3Vector& segP,const C3Vector& segL,const C3Vector& pt,double& dist,C3Vector* minDistSegPt)
{
    if (dist==0.0)
        return(false);

    double t=(pt*segL-segP*segL)/(segL*segL);
    if (t<0.0)
        t=0.0;
    if (t>1.0)
        t=1.0;
    C3Vector p(segP+segL*t);
    double d=(p-pt).getLength();
    if (d<dist)
    {
        dist=d;
        if (minDistSegPt!=nullptr)
            minDistSegPt[0]=p;
        return(true);
    }
    return(false);
}

bool CCalcUtils::getDistance_tris_segp(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segP,const C3Vector& segL,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{   // Calculates the distance between a triangle's surface and a segment
    // If the closest point on the triangle lies on an edge, return value is false

    C3Vector n(v^w);

    // Does the segment maybe collide with the triangle?
    double dd=-(p*n);
    double denom=n*segL;
    if (denom!=0.0)
    {   // Seg and tri are not parallel
        double t=-((n*segP)+dd)/denom;
        if ( (t>=0.0)&&(t<=1.0) )
        {   // the segment collides with the triangle's plane
            C3Vector intersection(segP+(segL*t));
            // is intersection within triangle's borders?
            C3Vector vect(intersection-p);
            if ((vect^w)*n>0.0)
            { // within border1
                if ((v^vect)*n>0.0)
                { // within border2
                    vect-=v;
                    C3Vector wv(w-v);
                    if ((wv^vect)*n>0.0)
                    { // within border3
                        dist=0.0;
                        if (minDistSegPt1!=nullptr)
                            minDistSegPt1[0]=intersection;
                        if (minDistSegPt2!=nullptr)
                            minDistSegPt2[0]=intersection;
                        return(true);
                    }
                }
            }
        }
    }

    bool retVal=false;
    // Now we check the segment endpoint1 projected onto the triangle's surface
    // If the projection lies within the triangle's borders, we have a distance
    denom=n*n;
    double t=-((n*segP)+dd)/denom;
    C3Vector projectedPt(segP+(n*t));
    // is projected pt within triangle's borders?
    C3Vector vect(projectedPt-p);
    if ((vect^w)*n>0.0)
    { // within border1
        if ((v^vect)*n>0.0)
        { // within border2
            vect-=v;
            C3Vector wv(w-v);
            if ((wv^vect)*n>0.0)
            { // within border3
                double d=(projectedPt-segP).getLength();
                if (d<dist)
                {
                    retVal=true;
                    dist=d;
                    if (minDistSegPt1!=nullptr)
                        minDistSegPt1[0]=projectedPt;
                    if (minDistSegPt2!=nullptr)
                        minDistSegPt2[0]=segP;
                }
            }
        }
    }

    // Now we check the segment endpoint2 projected onto the triangle's surface
    // If the projection lies within the triangle's borders, we have a distance
    t=-(n*(segP+segL)+dd)/denom;
    projectedPt=segP+segL+(n*t);
    // is projected pt within triangle's borders?
    vect=projectedPt-p;
    if ((vect^w)*n>0.0)
    { // within border1
        if ((v^vect)*n>0.0)
        { // within border2
            vect-=v;
            C3Vector wv(w-v);
            if ((wv^vect)*n>0.0)
            { // within border3
                double d=(projectedPt-(segP+segL)).getLength();
                if (d<dist)
                {
                    retVal=true;
                    dist=d;
                    if (minDistSegPt1!=nullptr)
                        minDistSegPt1[0]=projectedPt;
                    if (minDistSegPt2!=nullptr)
                        minDistSegPt2[0]=segP+segL;
                }
            }
        }
    }
    return(retVal);
}

bool CCalcUtils::getDistance_tris_pt(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& pt,double& dist,C3Vector* minDistSegPt)
{
    C3Vector n(v^w);
    double a=p*n;
    // Point projected onto the triangle's plane:
    double t=(a-(n*pt))/(n*n);
    C3Vector projection(pt+(n*t));
    // is projection within triangle's borders?
    C3Vector vect(projection-p);
    if ((vect^w)*n>0.0)
    { // within border1
        if ((v^vect)*n>0.0)
        { // within border2
            vect-=v;
            C3Vector wv(w-v);
            if ((wv^vect)*n>0.0)
            { // within border3
                double d=(projection-pt).getLength();
                if (d<dist)
                {
                    dist=d;
                    if (minDistSegPt!=nullptr)
                        minDistSegPt[0]=projection;
                    return(true);
                }
            }
        }
    }
    return(false);
}

bool CCalcUtils::getDistance_seg_seg(const C3Vector& seg1Center,const C3Vector& seg1Hs,const C3Vector& seg2Center,const C3Vector& seg2Hs,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2)
{
    C3Vector seg1P(seg1Center-seg1Hs);
    C3Vector seg1L(seg1Hs*2.0);
    C3Vector seg2P(seg2Center-seg2Hs);
    C3Vector seg2L(seg2Hs*2.0);
    return(getDistance_segp_segp(seg1P,seg1L,seg2P,seg2L,dist,minDistSegPt1,minDistSegPt2));
}


bool CCalcUtils::getSensorDistance_segp(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,double cosAngle,const C3Vector& segP,const C3Vector& segL,double& dist,C3Vector* detectPt)
{
    if (dist==0.0)
        return(false);

    // First we ignore the volumes:
    C3Vector tmpPt;
    double d=dist;
    if (!getDistance_segp_pt(segP,segL,C3Vector::zeroVector,d,&tmpPt))
        return(false);

    C3Vector p1(segP);
    C3Vector p2(segP+segL);
    C3Vector p1L(segL);

    bool volInTruncated=false;
    // is tmpPt in volIn?
    if (!isPointInVolume(planesIn,tmpPt))
    { // nope, we need to truncate the segment with volIn:
        if (truncateSegmentWithVolume(planesIn,p1,p2)<0)
            return(false); // segment was completely truncated away
        p1L=p2-p1;
        d=dist;
        // get the distance to the truncated segment:
        if (!getDistance_segp_pt(p1,p1L,C3Vector::zeroVector,d,&tmpPt))
            return(false); // distance is larger than dist
        volInTruncated=true;
    }

    // is the tmpPt in volOut?
    if (isPointInVolume(planesOut,tmpPt))
    { // yep, we need to truncate the segment with volOut and check the end pts:
        if (!volInTruncated)
        { // but first truncate the seg with volIn if not already done:
            if (truncateSegmentWithVolume(planesIn,p1,p2)<0)
                return(false); // segment was completely truncated away
        }
        if (truncateSegmentWithVolume(planesOut,p1,p2)<0)
            return(false); // segment was completely truncated away
        double d1=p1.getLength();
        double d2=p2.getLength();
        d=std::min<double>(d1,d2);
        if (d>=dist)
            return(false);
        if (d1<d2)
            tmpPt=p1;
        else
            tmpPt=p2;
    }
    // Volumes are respected. Check angle between detection ray and segment:
    bool retVal=(cosAngle>=piValue*0.5);
    if (!retVal)
    {   // we have an angular limitation
        C3Vector v(tmpPt.getNormalized());
        double a=acos(fabs(segL.getNormalized()*v));
        retVal=(((piValue*0.5)-a)<=cosAngle);
    }
    if (retVal)
    {
        dist=d;
        if (detectPt!=nullptr)
            detectPt[0]=tmpPt;
        return(true);
    }
    return(false);
}

bool CCalcUtils::getSensorDistance_tri(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,double cosAngle,bool frontDetection,bool backDetection,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* detectPt,C3Vector* triN)
{
    if (dist==0.0)
        return(false);

    C3Vector triangleN(v^w);
    if ( (!frontDetection)||(!backDetection) )
    {
        if (triangleN*p<0.0)
        { // showing front
            if (!frontDetection)
                return(false); // wrong side
        }
        else
        { // showing back
            if (!backDetection)
                return(false); // wrong side
        }
    }
    double d=dist;
    C3Vector tmpPt;
    if (!getDistance_tri_pt(p,v,w,C3Vector::zeroVector,d,&tmpPt))
        return(false); // nope, distance too large
    triangleN.normalize();
    if (cosAngle<double(1.1))
    { // do a first angle test
        if (fabs(triangleN*tmpPt.getNormalized())<cosAngle)
            return(false); // nope, angle too large
    }
    // Check volumeIn:
    CPolygonePt* polygone=nullptr;
    if (planesIn.size()!=0)
    {
        if (!isPointInVolume(planesIn,tmpPt))
        { // closest point is not in volumeIn. We need to truncate the triangle with volumeIn
            polygone=new CPolygonePt(p,p+v,p+w);
            polygone=truncatePolygonWithVolume(planesIn,polygone,nullptr);
            if (polygone==nullptr)
                return(false); // volumeIn removed the triangle entirely
            CPolygonePt* it1=polygone;
            CPolygonePt* it2=it1->next;
            d=dist;
            while (true)
            {
                getDistance_segp_pt(it1->pt,it2->pt-it1->pt,C3Vector::zeroVector,d,&tmpPt);
                it1=it2;
                if (it1==polygone)
                    break;
                it2=it1->next;
            }
            if (d>=dist)
            {
                polygone->removeAllOthers();
                delete polygone;
                return(false); // not closer
            }
        }
    }

    // Check volumeOut:
    if (planesOut.size()!=0)
    {
        if (isPointInVolume(planesOut,tmpPt))
        { // closest point is in volumeOut. We need to truncate the triangle with volumeOut
            if (polygone==nullptr)
            {
                polygone=new CPolygonePt(p,p+v,p+w);
                polygone=truncatePolygonWithVolume(planesIn,polygone,nullptr);
                if (polygone==nullptr)
                    return(false); // volumeIn removed the triangle entirely
            }
            std::vector<C3Vector> edges;
            polygone=truncatePolygonWithVolume(planesOut,polygone,&edges);
            if (polygone!=nullptr)
            {
                polygone->removeAllOthers();
                delete polygone;
                polygone=nullptr;
            }
            if (edges.size()==0)
                return(false); // polygone was entirely in volumeOut
            d=dist;
            for (size_t i=0;i<edges.size()/2;i++)
                getDistance_segp_pt(edges[2*i+0],edges[2*i+1],C3Vector::zeroVector,d,&tmpPt);
            if (d>=dist)
                return(false); // not closer
        }
    }

    if (polygone!=nullptr)
    {
        polygone->removeAllOthers();
        delete polygone;
    }

    // Point tmpPt is inside of volumeIn and outside of volumeOut, and is closer than 'dist'

    if (cosAngle<double(1.1))
    { // Check the angle
        if (fabs(triangleN*tmpPt.getNormalized())<cosAngle)
            return(false); // nope, angle too large
    }

    if (triN!=nullptr)
        triN[0]=triangleN;
    if (detectPt!=nullptr)
        detectPt[0]=tmpPt;
    dist=d;
    return(true); // point is ok
}

bool CCalcUtils::getSensorDistance_cell(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,double cosAngle,bool frontDetection,bool backDetection,const C4X4Matrix& cell,double cellHs,double& dist,C3Vector* detectPt,C3Vector* triN)
{   // optimize!!!
    C3Vector sv(cellHs,cellHs,cellHs);
    return(getSensorDistance_box(planesIn,planesOut,cosAngle,frontDetection,backDetection,cell,sv,dist,detectPt,triN));
}

bool CCalcUtils::getSensorDistance_box(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,double cosAngle,bool frontDetection,bool backDetection,const C4X4Matrix& box,const C3Vector& boxHs,double& dist,C3Vector* detectPt,C3Vector* triN)
{   // optimize! Does simple sensor/triangle dist. calc. between boxs' triangles and sensor

    if (dist==0.0)
        return(false);

    bool retVal=false;

    C3Vector bpts[8];
    size_t pos=0;
    for (double z=-1.0;z<2.0;z+=2.0)
    {
        C3Vector a;
        a(2)=z*boxHs(2);
        for (double y=-1.0;y<2.0;y+=2.0)
        {
            a(1)=y*boxHs(1);
            for (double x=-1.0;x<2.0;x+=2.0)
            {
                a(0)=x*boxHs(0);
                bpts[pos++]=C3Vector(box*a);
            }
        }
    }

    // The triangle's indices are 'balanced':
    size_t triIndices[12*3]={0,1,5,4,5,7,7,2,6,0,3,1,1,3,7,0,6,2,0,5,4,4,7,6,7,3,2,0,2,3,1,7,5,0,4,6};

    for (size_t i=0;i<12;i++)
    {
        C3Vector bp(bpts[triIndices[3*i+0]]);
        C3Vector bv(bpts[triIndices[3*i+1]]-bp);
        C3Vector bw(bpts[triIndices[3*i+2]]-bp);
        bool bb=getSensorDistance_tri(planesIn,planesOut,cosAngle,frontDetection,backDetection,bp,bv,bw,dist,detectPt,triN);
        retVal=retVal||bb;
        if (dist==0.0)
            break;
    }
    return(retVal);
}

bool CCalcUtils::getRaySensorDistance_tri(const C3Vector& raySegP,const C3Vector& raySegL,double cosAngle,bool frontDetection,bool backDetection,double forbiddenDist,const C3Vector& p,const C3Vector& v,const C3Vector& w,bool* forbiddenDistTouched,double& dist,C3Vector* detectPt,C3Vector* triN)
{
    if (dist==0.0)
        return(false);

    C3Vector triangleN(v^w);
    if ( (!frontDetection)||(!backDetection) )
    {
        if (triangleN*p<0.0)
        { // showing front
            if (!frontDetection)
                return(false); // wrong side
        }
        else
        { // showing back
            if (!backDetection)
                return(false); // wrong side
        }
    }
    triangleN.normalize();
    if (cosAngle<double(1.1))
    { // do the angle test
        if (fabs(triangleN*raySegL.getNormalized())<cosAngle)
            return(false); // nope, angle too large
    }

    double denom=triangleN*raySegL;
    if (denom!=0.0)
    {   // Ray and tri plane are not parallel
        double t=((p*triangleN)-(triangleN*raySegP))/denom;
        if ( (t>=0.0)&&(t<=1.0) )
        {
            C3Vector intersection(raySegP+(raySegL*t));
            // is intersection within triangle's borders?
            C3Vector vect(intersection-p);
            if ((vect^w)*triangleN>0.0)
            { // within border1
                if ((v^vect)*triangleN>0.0)
                { // within border2
                    vect-=v;
                    C3Vector wv(w-v);
                    if ((wv^vect)*triangleN>0.0)
                    { // within border3
                        double d=intersection.getLength();
                        if (d<dist)
                        {
                            dist=d;
                            if (detectPt!=nullptr)
                                detectPt[0]=intersection;
                            if (triN!=nullptr)
                                triN[0]=triangleN;
                            if (forbiddenDistTouched!=nullptr)
                            {
                                if (forbiddenDist>d)
                                    forbiddenDistTouched[0]=true; // we indicate that we are below the close limit
                            }
                            return(true);
                        }
                    }
                }
            }
        }
    }
    return(false);
}

bool CCalcUtils::getRaySensorDistance_cell(const C3Vector& raySegP,const C3Vector& raySegL,double cosAngle,bool frontDetection,bool backDetection,double forbiddenDist,const C4X4Matrix& cell,double cellHs,bool* forbiddenDistTouched,double& dist,C3Vector* detectPt,C3Vector* triN)
{
    return(getRaySensorDistance_box(raySegP,raySegL,cosAngle,frontDetection,backDetection,forbiddenDist,cell,C3Vector(cellHs,cellHs,cellHs),forbiddenDistTouched,dist,detectPt,triN));
}

bool CCalcUtils::getRaySensorDistance_box(const C3Vector& raySegP,const C3Vector& raySegL,double cosAngle,bool frontDetection,bool backDetection,double forbiddenDist,const C4X4Matrix& box,const C3Vector& boxHs,bool* forbiddenDistTouched,double& dist,C3Vector* detectPt,C3Vector* triN)
{   // optimize! Does simple triangle/triangle dist. calc. between boxs' triangles and triangle

    if (dist==0.0)
        return(false);

    bool retVal=false;

    C3Vector bpts[8];
    size_t pos=0;
    for (double z=-1.0;z<2.0;z+=2.0)
    {
        C3Vector a;
        a(2)=z*boxHs(2);
        for (double y=-1.0;y<2.0;y+=2.0)
        {
            a(1)=y*boxHs(1);
            for (double x=-1.0;x<2.0;x+=2.0)
            {
                a(0)=x*boxHs(0);
                bpts[pos++]=C3Vector(box*a);
            }
        }
    }

    // The triangle's indices are 'balanced':
    size_t triIndices[12*3]={0,1,5,4,5,7,7,2,6,0,3,1,1,3,7,0,6,2,0,5,4,4,7,6,7,3,2,0,2,3,1,7,5,0,4,6};

    for (size_t i=0;i<12;i++)
    {
        C3Vector bp(bpts[triIndices[3*i+0]]);
        C3Vector bv(bpts[triIndices[3*i+1]]-bp);
        C3Vector bw(bpts[triIndices[3*i+2]]-bp);
        bool bb=getRaySensorDistance_tri(raySegP,raySegL,cosAngle,frontDetection,backDetection,forbiddenDist,bp,bv,bw,forbiddenDistTouched,dist,detectPt,triN);
        retVal=retVal||bb;
        if (dist==0.0)
            break;
    }
    return(retVal);
}

bool CCalcUtils::isPointInVolume(const CVolumePlanes& planes,const C3Vector& pt)
{
    if (planes.size()>0)
    {
        for (size_t i=0;i<planes.size()/4;i++)
        {
            if ((C3Vector(planes.ptr()+4*i+0)*pt+planes.at(4*i+3))>=0.0)
                return(false);
        }
        return(true);
    }
    return(false);
}

int CCalcUtils::truncateSegmentWithVolume(const CVolumePlanes& planes,C3Vector& segP1,C3Vector& segP2)
{   // keeps inside of volume
    // return -1: segment was removed, 0: segment was modified, 1: segment is original
    int retVal=1;
    if (planes.size()!=0)
    {
        for (size_t i=0;i<planes.size()/4;i++)
        {
            C3Vector ppp(planes.ptr()+4*i+0);
            double d=planes.at(4*i+3);
            bool p1Inside=((ppp*segP1+d)<0.0);
            bool p2Inside=((ppp*segP2+d)<0.0);
            if (p1Inside!=p2Inside)
            { // points lie on opposite side of plane
                retVal=0;
                C3Vector v(segP2-segP1);
                double t=-(ppp*segP1+d)/(ppp*v);
                if (p1Inside)
                    segP2=segP1+(v*t);
                else
                    segP1=segP1+(v*t);
            }
            else if (!p1Inside)
                return(-1);
        }
    }
    return(retVal);
}


CPolygonePt* CCalcUtils::truncatePolygonWithVolume(const CVolumePlanes& planes,CPolygonePt* polygone,std::vector<C3Vector>* edgeSegments)
{   // keeps inside of volume
    if (planes.size()==0)
        return(polygone);
    for (size_t i=0;i<planes.size()/4;i++)
    {
        C3Vector xyz(planes.ptr()+4*i+0);
        double d=planes.at(4*i+3);
        CPolygonePt* it=polygone;
        C3Vector pt1(it->pt);
        bool p1Inside=((xyz*pt1+d)<0.0);
        it=it->next;
        bool cont=true;
        CPolygonePt* insideStart=nullptr;
        CPolygonePt* outsideStart=nullptr;
        bool hasPtsInside=false;
        while (cont)
        {
            if (it==polygone)
                cont=false;
            C3Vector pt2(it->pt);
            bool p2Inside=((xyz*pt2+d)<0.0);
            if (p2Inside!=p1Inside)
            { // pts lie on different side of the plane
                C3Vector pt1_pt2_vect(pt2-pt1);
                double t=-(xyz*pt1+d)/(xyz*pt1_pt2_vect);
                C3Vector newP(pt1+(pt1_pt2_vect*t));
                CPolygonePt* newPt=new CPolygonePt(newP);
                newPt->insertAfter(it->previous);
                if (edgeSegments!=nullptr)
                    edgeSegments->push_back(newP);
                if (insideStart!=nullptr)
                {
                    newPt->removePtsAfterThisUntilBefore(insideStart);
                    polygone=newPt;
                    break;
                }
                if (outsideStart!=nullptr)
                {
                    outsideStart->removePtsAfterThisUntilBefore(newPt);
                    polygone=newPt;
                    break;
                }
                if (p2Inside)
                    insideStart=newPt;
                else
                    outsideStart=newPt;
            }
            if (p1Inside)
                hasPtsInside=true;
            it=it->next;
            pt1=pt2;
            p1Inside=p2Inside;
        }
        if ( (!hasPtsInside)&&(insideStart==nullptr)&&(outsideStart==nullptr) )
        {
            polygone->removeAllOthers();
            delete polygone;
            return(nullptr);
        }
    }
    return(polygone);
}

bool CCalcUtils::isBoxMaybeInSensorVolume(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,const C4X4Matrix& box,const C3Vector& boxHs)
{
    C3Vector bpts[8];
    size_t pos=0;
    for (double z=-1.0;z<2.0;z+=2.0)
    {
        C3Vector a;
        a(2)=z*boxHs(2);
        for (double y=-1.0;y<2.0;y+=2.0)
        {
            a(1)=y*boxHs(1);
            for (double x=-1.0;x<2.0;x+=2.0)
            {
                a(0)=x*boxHs(0);
                bpts[pos++]=C3Vector(box*a);
            }
        }
    }

    for (size_t i=0;i<planesIn.size()/4;i++)
    {
        bool allCornersOutsideForThisPlane=true;
        C3Vector xyz(planesIn.ptr()+4*i+0);
        double d=planesIn.at(4*i+3);
        for (size_t j=0;j<8;j++)
            allCornersOutsideForThisPlane=allCornersOutsideForThisPlane&&((xyz*bpts[j]+d)>0.0);
        if (allCornersOutsideForThisPlane)
            return(false);
    }

    if (planesOut.size()==0)
        return(true);

    for (size_t i=0;i<planesOut.size()/4;i++)
    {
        bool oneCornerOutsideForThisPlane=false;
        C3Vector xyz(planesOut.ptr()+4*i+0);
        double d=planesOut.at(4*i+3);
        for (size_t j=0;j<8;j++)
            oneCornerOutsideForThisPlane=oneCornerOutsideForThisPlane||((xyz*bpts[j]+d)>0.0);
        if (oneCornerOutsideForThisPlane)
            return(true);
    }
    return(false);
}
