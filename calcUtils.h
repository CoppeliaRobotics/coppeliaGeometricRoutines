#pragma once

#include "conf.h"
#include <vector>
#include "3Vector.h"
#include "4Vector.h"
#include "4X4Matrix.h"

class CPolygonePt
{
public:
    CPolygonePt(const C3Vector& _pt)
    {
        createSingle(_pt);
    }
    virtual ~CPolygonePt()
    {
        if (next!=nullptr)
        {
            if (next==previous)
            { // two are left, then only one will be left
                next->next=nullptr;
                next->previous=nullptr;
            }
            else
            { // more than two are left
                next->previous=previous;
                previous->next=next;
            }
        }
    }
    CPolygonePt(const C3Vector& pt1,const C3Vector& pt2,const C3Vector& pt3)
    {
        createSingle(pt1);
        CPolygonePt* poly2=new CPolygonePt(pt2);
        CPolygonePt* poly3=new CPolygonePt(pt3);
        next=poly2;
        previous=poly3;
        poly2->next=poly3;
        poly2->previous=this;
        poly3->next=this;
        poly3->previous=poly2;
    }
    void createSingle(const C3Vector& _pt)
    {
        pt=_pt;
        next=nullptr;
        previous=nullptr;
    }
    void insertAfter(CPolygonePt* previousPolyPt)
    {
        next=previousPolyPt->next;
        previous=previousPolyPt;
        previousPolyPt->next=this;
        next->previous=this;
    }
    void removePtsAfterThisUntilBefore(CPolygonePt* here)
    {
        while (next!=here)
            delete next;
    }
    void removeAllOthers()
    {
        CPolygonePt* n=next;
        while ( (n!=nullptr)&&(n!=this) )
        {
            CPolygonePt* toDelete=n;
            n=n->next;
            delete toDelete;
        }
    }

    CPolygonePt* previous;
    CPolygonePt* next;
    C3Vector pt;
};

const C3Vector ocNodeTranslations[8]={
    C3Vector(-0.25f,-0.25f,-0.25f),
    C3Vector(0.25f,-0.25f,-0.25f),
    C3Vector(-0.25f,0.25f,-0.25f),
    C3Vector(0.25f,0.25f,-0.25f),
    C3Vector(-0.25f,-0.25f,0.25f),
    C3Vector(0.25f,-0.25f,0.25f),
    C3Vector(-0.25f,0.25f,0.25f),
    C3Vector(0.25f,0.25f,0.25f)
};

struct SNodeTranslation {
    C3Vector transl; int index;
    bool operator< (const SNodeTranslation& otherNodeTranslation) const {return false;}
};

struct SNodeTranslationPair {
    C3Vector transl1; int index1;
    C3Vector transl2; int index2;
    bool operator< (const SNodeTranslationPair& otherNodeTranslationPair) const {return false;}
};


class CCalcUtils
{
public:
    // Collision detection:
    static bool doCollide_box_box(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,bool solidBoxes);
    static bool doCollide_box_cell(const C4X4Matrix& box,const C3Vector& boxHs,float cellHs,bool solidBoxAndCell);
    static bool doCollide_box_tri(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& p,const C3Vector& v,const C3Vector& w);
    static bool doCollide_box_seg(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& segCenter,const C3Vector& segHs);
    static bool doCollide_box_segp(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& segP,const C3Vector& segL);
    static bool doCollide_box_allpts(const C4X4Matrix& box,const C3Vector& boxHs,const std::vector<C3Vector>& points);
    static bool doCollide_box_onept(const C4X4Matrix& box,const C3Vector& boxHs,const std::vector<C3Vector>& points,C3Vector* insidePt=nullptr);
    static bool doCollide_cell_tri(float cellHs,bool solidCell,const C3Vector& p,const C3Vector& v,const C3Vector& w);
    static bool doCollide_cell_seg(float cellHs,bool solidCell,const C3Vector& segCenter,const C3Vector& segHs);
    static bool doCollide_cell_segp(float cellHs,bool solidCell,const C3Vector& segP,const C3Vector& segL);
    static bool doCollide_cell_pt(float cellHs,const C3Vector& pt);
    static bool doCollide_tri_tri(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,int tri1Index,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,std::vector<float>* intersections,int* cachingTri1,int* cachingTri2);
    static bool doCollide_tri_segp(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,int tri1Index,const C3Vector& segP,const C3Vector& segL,std::vector<float>* intersections,int* cachingTri1);

    // Distance calculation:
    static bool getDistance_box_box_alt(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,bool solidBoxes,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_box_box(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,bool solidBoxes,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_box_tri_alt(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_box_tri(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_box_segp(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& segP,const C3Vector& segL,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_box_segp_alt(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& segP,const C3Vector& segL,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_box_pt(const C4X4Matrix& box,const C3Vector& boxHs,bool solidBox,const C3Vector& pt,float& dist,C3Vector* minDistSegPt,bool* ptIsInside);
    static bool getDistance_cell_cell(const C4X4Matrix& cell1,float cell1Hs,const C4X4Matrix& cell2,float cell2Hs,bool solidCells,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_cell_tri(float cellHs,bool solidCell,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_cell_segp(float cellHs,bool solidCell,const C3Vector& segP,const C3Vector& segL,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_cell_pt(float cellHs,bool solidCell,const C3Vector& pt,float& dist,C3Vector* minDistSegPt,bool* ptIsInside);
    static bool getDistance_tri_tri(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_tri_segp(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segP,const C3Vector& segL,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static bool getDistance_tri_pt(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& pt,float& dist,C3Vector* minDistSegPt);
    static inline bool getDistance_tris_segp(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segP,const C3Vector& segL,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static inline bool getDistance_tris_pt(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& pt,float& dist,C3Vector* minDistSegPt);
    static inline bool getDistance_seg_seg(const C3Vector& seg1Center,const C3Vector& seg1Hs,const C3Vector& seg2Center,const C3Vector& seg2Hs,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static inline bool getDistance_segp_segp(const C3Vector& seg1P,const C3Vector& seg1L,const C3Vector& seg2P,const C3Vector& seg2L,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2);
    static inline bool getDistance_segp_pt(const C3Vector& segP,const C3Vector& segL,const C3Vector& pt,float& dist,C3Vector* minDistSegPt);
    static bool isApproxDistanceSmaller_box_box(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,float& dist);
    static inline bool isApproxDistanceSmaller_box_box_fast(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs,float& dist);
    static bool isApproxDistanceSmaller_box_cell(const C4X4Matrix& box,const C3Vector& boxHs,float cellHs,float& dist);
    static inline bool isApproxDistanceSmaller_box_cell_fast(const C4X4Matrix& box,const C3Vector& boxHs,float cellHs,float& dist);
    static bool isApproxDistanceSmaller_box_tri(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist);
    static inline bool isApproxDistanceSmaller_box_tri_fast(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist);
    static bool isApproxDistanceSmaller_box_seg(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segCenter,const C3Vector& segHs,float& dist);
    static inline bool isApproxDistanceSmaller_box_seg_fast(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segCenter,const C3Vector& segHs,float& dist);
    static bool isApproxDistanceSmaller_box_segp(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segP,const C3Vector& segL,float& dist);
    static inline bool isApproxDistanceSmaller_box_segp_fast(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segP,const C3Vector& segL,float& dist);
    static inline bool isApproxDistanceSmaller_box_pt_fast(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& pt,float& dist);
    static bool isApproxDistanceSmaller_cell_tri(float cellHs,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist);
    static inline bool isApproxDistanceSmaller_cell_tri_fast(float cellHs,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist);
    static bool isApproxDistanceSmaller_cell_seg(float cellHs,const C3Vector& segCenter,const C3Vector& segHs,float& dist);
    static inline bool isApproxDistanceSmaller_cell_seg_fast(float cellHs,const C3Vector& segCenter,const C3Vector& segHs,float& dist);
    static bool isApproxDistanceSmaller_cell_segp(float cellHs,const C3Vector& segP,const C3Vector& segL,float& dist);
    static inline bool isApproxDistanceSmaller_cell_segp_fast(float cellHs,const C3Vector& segP,const C3Vector& segL,float& dist);
    static bool isApproxDistanceSmaller_tri_tri(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,float& dist);
    static inline bool isApproxDistanceSmaller_tri_tri_fast(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,float& dist);
    static bool isApproxDistanceSmaller_tri_pt_fast(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& pt,float& dist);
    static inline bool isApproxDistanceSmaller_segp_segp_fast(const C3Vector& seg1P,C3Vector seg1L,const C3Vector& seg2P,C3Vector seg2L,float& dist);
    static float getApproxDistance_box_box(const C4X4Matrix& box1,const C3Vector& box1Hs,const C4X4Matrix& box2,const C3Vector& box2Hs);
    static float getApproxDistance_box_cell(const C4X4Matrix& box,const C3Vector& boxHs,float cellHs);
    static float getApproxDistance_box_tri(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& p,const C3Vector& v,const C3Vector& w);
    static float getApproxDistance_box_seg(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segCenter,const C3Vector& segHs);
    static float getApproxDistance_box_segp(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segP,const C3Vector& segL);
    static float getApproxDistance_box_pt(const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& pt);
    static float getApproxDistance_cell_tri(float cellHs,const C3Vector& p,const C3Vector& v,const C3Vector& w);
    static float getApproxDistance_cell_seg(float cellHs,const C3Vector& segCenter,const C3Vector& segHs);
    static float getApproxDistance_cell_segp(float cellHs,const C3Vector& segP,const C3Vector& segL);
    static float getApproxDistance_tri_tri(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2);
    static float getProjectedDistance_box_tri(const C3Vector& axis,const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& p,const C3Vector& v,const C3Vector& w);
    static float getProjectedDistance_box_seg(const C3Vector& axis,const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segCenter,const C3Vector& segHs);
    static float getProjectedDistance_box_segp(const C3Vector& axis,const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& segP,const C3Vector& segL);
    static float getProjectedDistance_box_pt(const C3Vector& axis,const C4X4Matrix& box,const C3Vector& boxHs,const C3Vector& pt);
    static float getProjectedDistance_cell_tri(const C3Vector& axis,float cellHs,const C3Vector& p,const C3Vector& v,const C3Vector& w);
    static float getProjectedDistance_cell_seg(const C3Vector& axis,float cellHs,const C3Vector& segCenter,const C3Vector& segHs);
    static float getProjectedDistance_cell_segp(const C3Vector& axis,float cellHs,const C3Vector& segP,const C3Vector& segL);
    static float getProjectedDistance_tri_tri(const C3Vector& axis,const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2);

    // Proximity sensor detection:
    static bool getSensorDistance_box(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,float cosAngle,bool frontDetection,bool backDetection,const C4X4Matrix& box,const C3Vector& boxHs,float& dist,C3Vector* detectPt,C3Vector* triN);
    static bool getSensorDistance_cell(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,float cosAngle,bool frontDetection,bool backDetection,const C4X4Matrix& cell,float cellHs,float& dist,C3Vector* detectPt,C3Vector* triN);
    static bool getSensorDistance_tri(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,float cosAngle,bool frontDetection,bool backDetection,const C3Vector& p,const C3Vector& v,const C3Vector& w,float& dist,C3Vector* detectPt,C3Vector* triN);
    static bool getSensorDistance_segp(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,float cosAngle,const C3Vector& segP,const C3Vector& segL,float& dist,C3Vector* detectPt);
    static bool getRaySensorDistance_box(const C3Vector& raySegP,const C3Vector& raySegL,float cosAngle,bool frontDetection,bool backDetection,float forbiddenDist,const C4X4Matrix& box,const C3Vector& boxHs,bool* forbiddenDistTouched,float& dist,C3Vector* detectPt,C3Vector* triN);
    static bool getRaySensorDistance_cell(const C3Vector& raySegP,const C3Vector& raySegL,float cosAngle,bool frontDetection,bool backDetection,float forbiddenDist,const C4X4Matrix& cell,float cellHs,bool* forbiddenDistTouched,float& dist,C3Vector* detectPt,C3Vector* triN);
    static bool getRaySensorDistance_tri(const C3Vector& raySegP,const C3Vector& raySegL,float cosAngle,bool frontDetection,bool backDetection,float forbiddenDist,const C3Vector& p,const C3Vector& v,const C3Vector& w,bool* forbiddenDistTouched,float& dist,C3Vector* detectPt,C3Vector* triN);
    static bool isPointInVolume(const CVolumePlanes& planes,const C3Vector& pt);
    static bool isBoxMaybeInSensorVolume(const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,const C4X4Matrix& box,const C3Vector& boxHs);
    static int truncateSegmentWithVolume(const CVolumePlanes& planes,C3Vector& segP1,C3Vector& segP2);
    static CPolygonePt* truncatePolygonWithVolume(const CVolumePlanes& planes,CPolygonePt* polygone,std::vector<C3Vector>* edgeSegments);
};
