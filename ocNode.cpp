#include <calcUtils.h>
#include <ocNode.h>
#include <ocStruct.h>

COcNode::COcNode()
{
    id = 0;
    empty=true;
    ocNodes=nullptr;
}

COcNode::COcNode(COcStruct* oct, double boxS,const C3Vector& boxCenter,double cellS,const std::vector<double>& points,const std::vector<unsigned char>& rgbaData,const std::vector<unsigned int>& usrData,bool dataForEachPt)
{
    id = 0;
    empty=true;
    ocNodes=nullptr;
    double boxHsp=boxS*double(0.5001);
    if (boxS>cellS*double(1.5))
    {
        std::vector<double> pts;
        std::vector<unsigned char> rgbas;
        std::vector<unsigned int> usrs;
        for (size_t i=0;i<points.size()/3;i++)
        {
            C3Vector pt(&points[3*i]);
            pt-=boxCenter;
            if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
            {
                pts.push_back(pt(0));
                pts.push_back(pt(1));
                pts.push_back(pt(2));
                if (dataForEachPt)
                {
                    rgbas.push_back(rgbaData[4*i+0]);
                    rgbas.push_back(rgbaData[4*i+1]);
                    rgbas.push_back(rgbaData[4*i+2]);
                    rgbas.push_back(rgbaData[4*i+3]);
                    usrs.push_back(usrData[i]);
                }
            }
        }
        if (pts.size()>0)
        {
            if (!dataForEachPt)
            {
                rgbas.push_back(rgbaData[0]);
                rgbas.push_back(rgbaData[1]);
                rgbas.push_back(rgbaData[2]);
                rgbas.push_back(rgbaData[3]);
                usrs.push_back(usrData[0]);
            }
            ocNodes=new COcNode* [8];
            for (size_t i=0;i<8;i++)
                ocNodes[i]=new COcNode(oct, boxS*0.5,ocNodeTranslations[i]*boxS,cellS,pts,rgbas,usrs,dataForEachPt);
        }
    }
    else
    {
        for (size_t i=0;i<points.size()/3;i++)
        {
            C3Vector pt(&points[3*i]);
            pt-=boxCenter;
            if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
            {
                empty=false;
                oct->genId(id);
                if (dataForEachPt)
                {
                    rgba[0]=rgbaData[4*i+0];
                    rgba[1]=rgbaData[4*i+1];
                    rgba[2]=rgbaData[4*i+2];
                    rgba[3]=rgbaData[4*i+3];
                    userData=usrData[i];
                }
                else
                {
                    rgba[0]=rgbaData[0];
                    rgba[1]=rgbaData[1];
                    rgba[2]=rgbaData[2];
                    rgba[3]=rgbaData[3];
                    userData=usrData[0];
                }
            }
        }
    }
}

COcNode::~COcNode()
{
    if (ocNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            delete ocNodes[i];
        delete[] ocNodes;
    }
}

COcNode* COcNode::copyYourself() const
{
    COcNode* newOcNode=new COcNode();
    newOcNode->empty=empty;
    newOcNode->rgba[0]=rgba[0];
    newOcNode->rgba[1]=rgba[1];
    newOcNode->rgba[2]=rgba[2];
    newOcNode->rgba[3]=rgba[3];
    newOcNode->userData=userData;
    if (ocNodes!=nullptr)
    {
        newOcNode->ocNodes=new COcNode* [8];
        for (size_t i=0;i<8;i++)
            newOcNode->ocNodes[i]=ocNodes[i]->copyYourself();
    }
    return(newOcNode);
}

void COcNode::serialize(std::vector<unsigned char>& data) const
{
    const unsigned char ver = 3;
    if (!empty)
    {
        data.push_back(ver);
        data.push_back(rgba[0]);
        data.push_back(rgba[1]);
        data.push_back(rgba[2]);
        data.push_back(rgba[3]);
        pushData(data,&userData,sizeof(unsigned int));
    }
    else
        data.push_back(0);
    if (ocNodes!=nullptr)
    {
        data.push_back(ver);
        for (size_t i=0;i<8;i++)
            ocNodes[i]->serialize(data);
    }
    else
        data.push_back(0);
}

void COcNode::serialize_ver2(std::vector<unsigned char>& data) const
{ // for backw. compatibility. In ver 2 no rgbA, just rgb!
    if (!empty)
    {
        data.push_back(1);
        data.push_back(rgba[0]);
        data.push_back(rgba[1]);
        data.push_back(rgba[2]);
        pushData(data,&userData,sizeof(unsigned int));
    }
    else
        data.push_back(0);
    if (ocNodes!=nullptr)
    {
        data.push_back(1);
        for (size_t i=0;i<8;i++)
            ocNodes[i]->serialize_ver2(data);
    }
    else
        data.push_back(0);
}

void COcNode::deserialize(COcStruct* oct, const unsigned char* data,int& pos)
{
    unsigned char ver = data[pos++];
    empty=(ver==0);
    if (!empty)
    {
        oct->genId(id);
        rgba[0]=data[pos++];
        rgba[1]=data[pos++];
        rgba[2]=data[pos++];
        if (ver >= 3)
            rgba[3]=data[pos++];
        else
            rgba[3]=255;
        userData=(reinterpret_cast<const unsigned int*>(data+pos))[0];
        pos+=sizeof(userData);
    }
    else
        id = 0;
    if (data[pos++]!=0)
    {
        ocNodes=new COcNode* [8];
        for (size_t i=0;i<8;i++)
        {
            ocNodes[i]=new COcNode();
            ocNodes[i]->deserialize(oct, data,pos);
        }
    }
}

bool COcNode::getCell(const C3Vector& boxCenter,double boxSize,unsigned long long int ocCaching,C3Vector& totalTranslation,unsigned int* usrData) const
{
    bool retVal=false;
    unsigned long long int cellPath=(ocCaching>>6)<<6;
    unsigned long long int cellDepth=ocCaching&63;
    if (cellDepth>0)
    {
        if (ocNodes!=nullptr)
        {
            int index=(cellPath>>(6+(cellDepth-1)*3))&7;
            unsigned long long int _ocCaching=cellPath|(cellDepth-1);
            retVal=ocNodes[index]->getCell(boxCenter+ocNodeTranslations[index]*boxSize,boxSize*0.5,_ocCaching,totalTranslation,usrData);
        }
    }
    else
    {
        if (!empty)
        {
            totalTranslation=boxCenter;
            if (usrData!=nullptr)
                usrData[0]=userData;
            retVal=true;
        }
    }
    return(retVal);
}

void COcNode::resetAllIds(COcStruct* oct)
{
    if (ocNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            ocNodes[i]->resetAllIds(oct);
    }
    oct->genId(id);
}

void COcNode::getDisplayVoxelsColorsAndIds(COcStruct* oct, double pBoxSize,const C3Vector& boxCenter, std::vector<float>& thePts, std::vector<unsigned char>& theRgbas, std::vector<unsigned int>& theIds) const
{
    if (ocNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            ocNodes[i]->getDisplayVoxelsColorsAndIds(oct, pBoxSize * 0.5, boxCenter + ocNodeTranslations[i] * pBoxSize * 0.5, thePts, theRgbas, theIds);
    }
    if (!empty)
    {
        thePts.push_back(float(boxCenter(0)));
        thePts.push_back(float(boxCenter(1)));
        thePts.push_back(float(boxCenter(2)));
        theRgbas.push_back(rgba[0]);
        theRgbas.push_back(rgba[1]);
        theRgbas.push_back(rgba[2]);
        theRgbas.push_back(rgba[3]);
        theIds.push_back(id);
        oct->allIds[id] = false;
    }
}

void COcNode::getVoxelsPosAndRgba(std::vector<double>& voxelsPosAndRgba,double pBoxSize,const C3Vector& boxCenter,std::vector<unsigned int>* usrData/*=nullptr*/) const
{
    if (!empty)
    {
        voxelsPosAndRgba.push_back(boxCenter(0));
        voxelsPosAndRgba.push_back(boxCenter(1));
        voxelsPosAndRgba.push_back(boxCenter(2));
        voxelsPosAndRgba.push_back(double(rgba[0])/double(254.99));
        voxelsPosAndRgba.push_back(double(rgba[1])/double(254.99));
        voxelsPosAndRgba.push_back(double(rgba[2])/double(254.99));
        voxelsPosAndRgba.push_back(double(rgba[3])/double(254.99));
        if (usrData!=nullptr)
            usrData->push_back(userData);
    }
    if (ocNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            ocNodes[i]->getVoxelsPosAndRgba(voxelsPosAndRgba,pBoxSize*0.5,boxCenter+ocNodeTranslations[i]*pBoxSize*0.5,usrData);
    }
}

void COcNode::getVoxelsCorners(std::vector<double>& points,double pBoxSize,const C3Vector& boxCenter) const
{
    if (!empty)
    {
        for (size_t i=0;i<8;i++)
        {
            C3Vector translation(ocNodeTranslations[i]*pBoxSize);
            points.push_back(boxCenter(0)+translation(0));
            points.push_back(boxCenter(1)+translation(1));
            points.push_back(boxCenter(2)+translation(2));
        }
    }
    if (ocNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            ocNodes[i]->getVoxelsCorners(points,pBoxSize*0.5,boxCenter+ocNodeTranslations[i]*pBoxSize*0.5);
    }
}

void COcNode::getOctreeCorners(std::vector<double>& points,double pBoxSize,const C3Vector& boxCenter) const
{
    for (size_t i=0;i<8;i++)
    {
        C3Vector translation(ocNodeTranslations[i]*pBoxSize);
        points.push_back(boxCenter(0)+translation(0));
        points.push_back(boxCenter(1)+translation(1));
        points.push_back(boxCenter(2)+translation(2));
    }
    if (ocNodes!=nullptr)
    {
        for (size_t i=0;i<8;i++)
            ocNodes[i]->getOctreeCorners(points,pBoxSize*0.5,boxCenter+ocNodeTranslations[i]*pBoxSize*0.5);
    }
}

bool COcNode::deleteVoxels_pts(COcStruct* oct, double boxS,const C3Vector& boxCenter,const std::vector<double>& points)
{
    bool retVal=true;
    double boxHsp=boxS*double(0.5001);
    if (ocNodes!=nullptr)
    {
        std::vector<double> pts;
        for (size_t i=0;i<points.size()/3;i++)
        {
            C3Vector pt(&points[3*i]);
            pt-=boxCenter;
            if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
            {
                pts.push_back(pt(0));
                pts.push_back(pt(1));
                pts.push_back(pt(2));
            }
        }
        if (pts.size()>0)
        {
            for (size_t i=0;i<8;i++)
            {
                bool bb=ocNodes[i]->deleteVoxels_pts(oct, boxS*0.5,ocNodeTranslations[i]*boxS,pts);
                retVal=retVal&&bb;
            }
        }
        else
            retVal=false;
    }
    else
    {
        if (!empty)
        {
            for (size_t i=0;i<points.size()/3;i++)
            {
                C3Vector pt(&points[3*i]);
                pt-=boxCenter;
                if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
                {
                    empty=true;
                    oct->remId(id);
                    break;
                }
            }
            retVal=empty;
        }
    }
    if (retVal)
    {
        if (!empty)
        {
            empty=true;
            oct->remId(id);
        }
        if (ocNodes!=nullptr)
        {
            for (size_t i=0;i<8;i++)
                delete ocNodes[i];
            delete[] ocNodes;
            ocNodes=nullptr;
        }
    }
    return(retVal); // true: this OC node is empty
}

bool COcNode::deleteVoxels_shape(COcStruct* oct, const C4X4Matrix& ocM,double boxS,const C3Vector& boxCenter,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM)
{
    bool retVal=true;
    double boxHsp=boxS*double(0.5001);
    C4X4Matrix tr(ocM);
    tr.X+=ocM.M*boxCenter;
    tr.inverse();
    tr=tr*shapeM;
    if (ocNodes!=nullptr)
    {
        if (CCalcUtils::doCollide_box_cell(tr*obb->boxM,obb->boxHs,boxHsp,true))
        {
            double cellV=boxHsp*boxHsp*boxHsp;
            double nodeV=obb->boxHs(0)*obb->boxHs(1)*obb->boxHs(2);
            if ( (cellV<nodeV)&&(obb->leafTris==nullptr) )
            {
                retVal=false;
                for (size_t i=0;i<2;i++)
                    retVal=retVal||deleteVoxels_shape(oct, ocM,boxS,boxCenter,obbStruct,obb->obbNodes[i],shapeM);
            }
            else
            {
                for (size_t i=0;i<8;i++)
                {
                    bool bb=ocNodes[i]->deleteVoxels_shape(oct, ocM,boxS*0.5,boxCenter+ocNodeTranslations[i]*boxS,obbStruct,obb,shapeM);
                    retVal=retVal&&bb;
                }
            }
        }
        else
            retVal=false;
    }
    else
    {
        if (!empty)
        {
            if (CCalcUtils::doCollide_box_cell(tr*obb->boxM,obb->boxHs,boxHsp,true))
            {
                if (obb->leafTris==nullptr)
                {
                    retVal=deleteVoxels_shape(oct, ocM,boxS,boxCenter,obbStruct,obb->obbNodes[0],shapeM);
                    retVal=retVal||deleteVoxels_shape(oct, ocM,boxS,boxCenter,obbStruct,obb->obbNodes[1],shapeM);
                }
                else
                {
                    retVal=false;
                    for (size_t i=0;i<obb->leafTris->size();i++)
                    {
                        int ind=3*obb->leafTris[0][i];
                        C3Vector p(&obbStruct->vertices[3*size_t(obbStruct->indices[size_t(ind)+0])+0]);
                        C3Vector v(&obbStruct->vertices[3*size_t(obbStruct->indices[size_t(ind)+1])+0]);
                        C3Vector w(&obbStruct->vertices[3*size_t(obbStruct->indices[size_t(ind)+2])+0]);
                        p*=tr;
                        v*=tr;
                        w*=tr;
                        v-=p;
                        w-=p;
                        if (CCalcUtils::doCollide_cell_tri(boxHsp,true,p,v,w))
                        {
                            retVal=true;
                            break;
                        }
                    }
                }
            }
            else
                retVal=false;
        }
    }
    if (retVal)
    {
        if (!empty)
        {
            empty=true;
            oct->remId(id);
        }

        if (ocNodes!=nullptr)
        {
            for (size_t i=0;i<8;i++)
                delete ocNodes[i];
            delete[] ocNodes;
            ocNodes=nullptr;
        }
    }
    return(retVal); // true: this OC node is empty
}

bool COcNode::deleteVoxels_octree(COcStruct* oct, const C4X4Matrix& oc1M,double box1S,const C3Vector& box1Center,const COcNode* oc2Node,const C4X4Matrix& oc2M,double box2S,const C3Vector& box2Center)
{
    bool retVal=true;
    double box1Hsp=box1S*double(0.5001);
    C4X4Matrix tr1(oc1M);
    tr1.X+=oc1M.M*box1Center;
    double box2Hsp=box2S*double(0.5001);
    C4X4Matrix tr2(oc2M);
    tr2.X+=oc2M.M*box2Center;
    C4X4Matrix oc2MRel(tr1.getInverse()*tr2);
    if (ocNodes!=nullptr)
    {
        if (CCalcUtils::doCollide_box_cell(oc2MRel,C3Vector(box2Hsp,box2Hsp,box2Hsp),box1Hsp,true))
        {
            if ( (box1S*box1S*box1S<box2S*box2S*box2S)&&(oc2Node->ocNodes!=nullptr) )
            {
                retVal=false;
                for (size_t i=0;i<8;i++)
                    retVal=retVal||deleteVoxels_octree(oct, oc1M,box1S,box1Center,oc2Node->ocNodes[i],oc2M,box2S*0.5,box2Center+ocNodeTranslations[i]*box2S);
            }
            else
            {
                for (size_t i=0;i<8;i++)
                {
                    bool bb=ocNodes[i]->deleteVoxels_octree(oct, oc1M,box1S*0.5,box1Center+ocNodeTranslations[i]*box1S,oc2Node,oc2M,box2S,box2Center);
                    retVal=retVal&&bb;
                }
            }
        }
        else
            retVal=false;
    }
    else
    {
        if (!empty)
        {
            if (CCalcUtils::doCollide_box_cell(oc2MRel,C3Vector(box2Hsp,box2Hsp,box2Hsp),box1Hsp,true))
            {
                if (oc2Node->ocNodes!=nullptr)
                {
                    bool retVal=false;
                    for (size_t i=0;i<8;i++)
                        retVal=retVal||deleteVoxels_octree(oct, oc1M,box1S,box1Center,oc2Node->ocNodes[i],oc2M,box2S*0.5,box2Center+ocNodeTranslations[i]*box2S);
                }
            }
            else
                retVal=false;
        }
    }
    if (retVal)
    {
        if (!empty)
        {
            empty=true;
            oct->remId(id);
        }
        if (ocNodes!=nullptr)
        {
            for (size_t i=0;i<8;i++)
                delete ocNodes[i];
            delete[] ocNodes;
            ocNodes=nullptr;
        }
    }
    return(retVal); // true: this OC node is empty
}

void COcNode::add_pts(COcStruct* oct, double cellS,double boxS,const C3Vector& boxCenter,const std::vector<double>& points,const std::vector<unsigned char>& rgbaData,const std::vector<unsigned int>& usrData,bool dataForEachPt)
{
    double boxHsp=boxS*double(0.5001);
    if (boxS>cellS*double(1.5))
    { // not a leaf node yet..
        std::vector<double> pts;
        std::vector<unsigned char> rgbas;
        std::vector<unsigned int> usrs;
        for (size_t i=0;i<points.size()/3;i++)
        {
            C3Vector pt(&points[3*i]);
            pt-=boxCenter;
            if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
            {
                pts.push_back(pt(0));
                pts.push_back(pt(1));
                pts.push_back(pt(2));
                if (dataForEachPt)
                {
                    rgbas.push_back(rgbaData[4*i+0]);
                    rgbas.push_back(rgbaData[4*i+1]);
                    rgbas.push_back(rgbaData[4*i+2]);
                    rgbas.push_back(rgbaData[4*i+3]);
                    usrs.push_back(usrData[i]);
                }
            }
        }
        if (pts.size()>0)
        {
            if (!dataForEachPt)
            {
                rgbas.push_back(rgbaData[0]);
                rgbas.push_back(rgbaData[1]);
                rgbas.push_back(rgbaData[2]);
                rgbas.push_back(rgbaData[3]);
                usrs.push_back(usrData[0]);
            }
            if (ocNodes==nullptr)
            {
                ocNodes=new COcNode* [8];
                for (size_t i=0;i<8;i++)
                    ocNodes[i]=new COcNode(oct, boxS*0.5,ocNodeTranslations[i]*boxS,cellS,pts,rgbas,usrs,dataForEachPt);
            }
            else
            {
                for (size_t i=0;i<8;i++)
                    ocNodes[i]->add_pts(oct, cellS,boxS*0.5,ocNodeTranslations[i]*boxS,pts,rgbas,usrs,dataForEachPt);
            }
        }
    }
    else
    { // this is a leaf node
        if (empty)
        { // voxel is still not existent
            for (size_t i=0;i<points.size()/3;i++)
            {
                C3Vector pt(&points[3*i]);
                pt-=boxCenter;
                if ( (fabs(pt(0))<boxHsp)&&(fabs(pt(1))<boxHsp)&&(fabs(pt(2))<boxHsp) )
                { // create that voxel
                    empty=false;
                    oct->genId(id);
                    if (dataForEachPt)
                    {
                        rgba[0]=rgbaData[4*i+0];
                        rgba[1]=rgbaData[4*i+1];
                        rgba[2]=rgbaData[4*i+2];
                        rgba[3]=rgbaData[4*i+3];
                        userData=usrData[i];
                    }
                    else
                    {
                        rgba[0]=rgbaData[0];
                        rgba[1]=rgbaData[1];
                        rgba[2]=rgbaData[2];
                        rgba[3]=rgbaData[3];
                        userData=usrData[0];
                    }
                }
            }
        }
    }
}

bool COcNode::add_shape(COcStruct* oct, const C4X4Matrix& ocM,double cellS,double boxS,const C3Vector& boxCenter,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,const unsigned char* rgbaData,unsigned int usrData)
{
    bool retVal=false;
    double boxHsp=boxS*double(0.5001);
    C4X4Matrix shapeMRel(ocM);
    shapeMRel.X+=ocM.M*boxCenter;
    shapeMRel=shapeMRel.getInverse()*shapeM;
    if (boxS>cellS*double(1.5))
    { // not a leaf node yet..
        C4X4Matrix m(shapeMRel*obb->boxM);
        if (CCalcUtils::doCollide_box_cell(m,obb->boxHs,boxHsp,true))
        { // the OBB box and the ocnode box collide. Continue exploration by exploring the largest volume first:
            if ( (obb->boxHs(0)*obb->boxHs(1)*obb->boxHs(2)>boxHsp*boxHsp*boxHsp)&&(obb->leafTris==nullptr) )
            { // Explore the OBB node
                bool bb1=add_shape(oct, ocM,cellS,boxS,boxCenter,obbStruct,obb->obbNodes[0],shapeM,rgbaData,usrData);
                bool bb2=add_shape(oct, ocM,cellS,boxS,boxCenter,obbStruct,obb->obbNodes[1],shapeM,rgbaData,usrData);
                retVal=bb1||bb2;
            }
            else
            { // Explore the OC node
                if (ocNodes==nullptr)
                { // OC node children have to be created first
                    ocNodes=new COcNode* [8];
                    for (size_t i=0;i<8;i++)
                        ocNodes[i]=new COcNode();
                }
                for (size_t i=0;i<8;i++)
                {
                    bool bb=ocNodes[i]->add_shape(oct, ocM,cellS,boxS*0.5,boxCenter+ocNodeTranslations[i]*boxS,obbStruct,obb,shapeM,rgbaData,usrData);
                    retVal=retVal||bb;
                }
            }
        }
    }
    else
    { // this is a leaf node
        if (empty)
        { // voxel does not yet exist
            C4X4Matrix m(shapeMRel*obb->boxM);
            if (CCalcUtils::doCollide_box_cell(m,obb->boxHs,boxHsp,true))
            { // the OBB box and the voxel collide
                if (obb->leafTris==nullptr)
                { // explore the OBB node
                    retVal=add_shape(oct, ocM,cellS,boxS,boxCenter,obbStruct,obb->obbNodes[0],shapeM,rgbaData,usrData);
                    retVal=retVal||add_shape(oct, ocM,cellS,boxS,boxCenter,obbStruct,obb->obbNodes[1],shapeM,rgbaData,usrData);
                }
                else
                { // Check the triangles of the OBB leaf and the voxel
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
                        if (CCalcUtils::doCollide_cell_tri(boxHsp,true,p,v,w))
                        { // Create the voxel
                            empty=false;
                            oct->genId(id);
                            retVal=true;
                            rgba[0]=rgbaData[0];
                            rgba[1]=rgbaData[1];
                            rgba[2]=rgbaData[2];
                            rgba[3]=rgbaData[3];
                            userData=usrData;
                            break;
                        }
                    }
                }
            }
        }
    }
    return(retVal); // true: at least one new voxel was added
}

bool COcNode::add_octree(COcStruct* oct, const C4X4Matrix& oc1M,double cell1S,double box1S,const C3Vector& box1Center,const COcNode* oc2Node,const C4X4Matrix& oc2M,double box2S,const C3Vector& box2Center,const unsigned char* rgbaData,unsigned int usrData)
{
    bool retVal=false;
    double box1Hsp=box1S*double(0.5001);
    double box2Hsp=box2S*double(0.5001);
    C4X4Matrix m1(oc1M);
    m1.X+=oc1M.M*box1Center;
    C4X4Matrix m2(oc2M);
    m2.X+=oc2M.M*box2Center;
    C4X4Matrix oc2MRel(m1.getInverse()*m2);
    if (box1S>cell1S*double(1.5))
    { // not a leaf node yet..
        if (CCalcUtils::doCollide_box_cell(oc2MRel,C3Vector(box2Hsp,box2Hsp,box2Hsp),box1Hsp,true))
        { // the two ocnode boxes collide. Continue exploration by exploring the largest volume first:
            if ( (box1Hsp*box1Hsp*box1Hsp<box2Hsp*box2Hsp*box2Hsp)&&(oc2Node->ocNodes!=nullptr) )
            { // Explore the OC node 2
                for (size_t i=0;i<8;i++)
                {
                    bool bb=add_octree(oct, oc1M,cell1S,box1S,box1Center,oc2Node->ocNodes[i],oc2M,box2S*0.5,box2Center+ocNodeTranslations[i]*box2S,rgbaData,usrData);
                    retVal=retVal||bb;
                }
            }
            else
            { // Explore the OC node 1
                if (ocNodes==nullptr)
                { // OC node 1 children have to be created first
                    ocNodes=new COcNode* [8];
                    for (size_t i=0;i<8;i++)
                        ocNodes[i]=new COcNode();
                }
                for (size_t i=0;i<8;i++)
                {
                    bool bb=ocNodes[i]->add_octree(oct, oc1M,cell1S,box1S*0.5,box1Center+ocNodeTranslations[i]*box1S,oc2Node,oc2M,box2S,box2Center,rgbaData,usrData);
                    retVal=retVal||bb;
                }
            }
        }
    }
    else
    { // this is a leaf node
        if (empty)
        { // voxel does not yet exist
            if (CCalcUtils::doCollide_box_cell(oc2MRel,C3Vector(box2Hsp,box2Hsp,box2Hsp),box1Hsp,true))
            { // the voxel of node1 and box2 collide
                if (oc2Node->ocNodes!=nullptr)
                {  // explore the node 2 since we are not yet at its leafs
                    for (size_t i=0;i<8;i++)
                    {
                        retVal=add_octree(oct, oc1M,cell1S,box1S,box1Center,oc2Node->ocNodes[i],oc2M,box2S*0.5,box2Center+ocNodeTranslations[i]*box2S,rgbaData,usrData);
                        if (retVal)
                            break; // the voxel was added, leave
                    }
                }
                else
                { // if the box of node 2 is not empty, we can create a voxel since they collide
                    retVal=!oc2Node->empty;
                    if (retVal)
                    {
                        empty=false;
                        oct->genId(id);
                        rgba[0]=rgbaData[0];
                        rgba[1]=rgbaData[1];
                        rgba[2]=rgbaData[2];
                        rgba[3]=rgbaData[3];
                        userData=usrData;
                    }
                }
            }
        }
    }
    return(retVal); // true: at least one new voxel was added
}

bool COcNode::doCollide_pt(const C3Vector& point,double boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const
{
    bool retVal=false;
    double boxHsp=boxS*double(0.5001);
    if (empty)
    {
        if (ocNodes!=nullptr)
        {
            if ( (fabs(point(0))<boxHsp)&&(fabs(point(1))<boxHsp)&&(fabs(point(2))<boxHsp) )
            { // the point collides with this box. Check children
                unsigned long long int cellPath=(boxCacheLocation>>6)<<(6+3);
                unsigned long long int cellDepth=(boxCacheLocation&63);
                for (size_t i=0;i<8;i++)
                {
                    retVal=ocNodes[i]->doCollide_pt(point-ocNodeTranslations[i]*boxS,boxS*0.5,cellPath|(i<<6)|(cellDepth+1),usrData,ocCaching);
                    if (retVal)
                        break;
                }
            }
        }
    }
    else
    { // there is a voxel
        retVal=( (fabs(point(0))<boxHsp)&&(fabs(point(1))<boxHsp)&&(fabs(point(2))<boxHsp) );
        if (retVal)
        { // it collides with the point
            if (ocCaching!=nullptr)
                ocCaching[0]=boxCacheLocation;
            if (usrData!=nullptr)
                usrData[0]=userData;
        }
    }
    return(retVal);
}

bool COcNode::doCollide_pts(const std::vector<double>& points,const C3Vector& boxPosRelToParent,double boxS) const
{
    bool retVal=false;
    double boxHsp=boxS*double(0.5001);
    if ( (!empty)||(ocNodes!=nullptr) )
    {   // compute pts rel to this box:
        std::vector<double> pts;
        for (size_t i=0;i<points.size()/3;i++)
        {
            C3Vector v(&points[3*i]);
            v-=boxPosRelToParent;
            if ( (fabs(v(0))<boxHsp)&&(fabs(v(1))<boxHsp)&&(fabs(v(2))<boxHsp) )
            { // inside box
                pts.push_back(v(0));
                pts.push_back(v(1));
                pts.push_back(v(2));
            }
        }
        if (pts.size()>0)
        {
            if (empty)
            {
                if (ocNodes!=nullptr)
                { // check children
                    for (size_t i=0;i<8;i++)
                    {
                        retVal=ocNodes[i]->doCollide_pts(pts,ocNodeTranslations[i]*boxS,boxS*0.5);
                        if (retVal)
                            break;
                    }
                }
            }
            else
                retVal=true; // we collide with voxel
        }
    }
    return(retVal);
}

bool COcNode::doCollide_seg(const C3Vector& segMiddle,const C3Vector& segHs,double boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const
{
    bool retVal=false;
    double boxHsp=boxS*double(0.5001);
    if (empty)
    {
        if (ocNodes!=nullptr)
        {
            if (CCalcUtils::doCollide_cell_seg(boxHsp,true,segMiddle,segHs))
            { // the segment collides with this box. Check children
                unsigned long long int cellPath=(boxCacheLocation>>6)<<(6+3);
                unsigned long long int cellDepth=(boxCacheLocation&63);
                for (size_t i=0;i<8;i++)
                {
                    retVal=ocNodes[i]->doCollide_seg(segMiddle-ocNodeTranslations[i]*boxS,segHs,boxS*0.5,cellPath|(i<<6)|(cellDepth+1),usrData,ocCaching);
                    if (retVal)
                        break;
                }
            }
        }
    }
    else
    { // there is a voxel
        retVal=CCalcUtils::doCollide_cell_seg(boxHsp,true,segMiddle,segHs);
        if (retVal)
        { // it collides with the segment
            if (ocCaching!=nullptr)
                ocCaching[0]=boxCacheLocation;
            if (usrData!=nullptr)
                usrData[0]=userData;
        }
    }
    return(retVal);
}

bool COcNode::doCollide_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,double boxS,unsigned long long int boxCacheLocation,unsigned int* usrData,unsigned long long int* ocCaching) const
{
    bool retVal=false;
    double boxHsp=boxS*double(0.5001);
    if (empty)
    {
        if (ocNodes!=nullptr)
        {
            if (CCalcUtils::doCollide_cell_tri(boxHsp,true,p,v,w))
            { // the triangle collides with this box. Check children
                unsigned long long int cellPath=(boxCacheLocation>>6)<<(6+3);
                unsigned long long int cellDepth=(boxCacheLocation&63);
                for (size_t i=0;i<8;i++)
                {
                    retVal=ocNodes[i]->doCollide_tri(p-ocNodeTranslations[i]*boxS,v,w,boxS*0.5,cellPath|(i<<6)|(cellDepth+1),usrData,ocCaching);
                    if (retVal)
                        break;
                }
            }
        }
    }
    else
    { // there is a voxel
        retVal=CCalcUtils::doCollide_cell_tri(boxHsp,true,p,v,w);
        if (retVal)
        { // it collides with the triangle
            if (ocCaching!=nullptr)
                ocCaching[0]=boxCacheLocation;
            if (usrData!=nullptr)
                usrData[0]=userData;
        }
    }
    return(retVal);
}

bool COcNode::doCollide_shape(const C4X4Matrix& ocM,double cellS,double boxS,const C3Vector& boxCenter,unsigned long long int ocCacheValueHere,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,unsigned long long int* ocCaching,int* meshCaching) const
{
    bool retVal=false;
    double boxHsp=boxS*double(0.5001);
    C4X4Matrix shapeMRel(ocM);
    shapeMRel.X+=ocM.M*boxCenter;
    shapeMRel=shapeMRel.getInverse()*shapeM;
    if (boxS>cellS*double(1.5))
    { // not a leaf node yet..
        C4X4Matrix m(shapeMRel*obb->boxM);
        if (CCalcUtils::doCollide_box_cell(m,obb->boxHs,boxHsp,true))
        { // the OBB box and the ocnode box collide. Continue exploration by exploring the largest volume first:
            if ( (obb->boxHs(0)*obb->boxHs(1)*obb->boxHs(2)>boxHsp*boxHsp*boxHsp)&&(obb->leafTris==nullptr) )
            { // Explore the OBB node
                retVal=doCollide_shape(ocM,cellS,boxS,boxCenter,ocCacheValueHere,obbStruct,obb->obbNodes[0],shapeM,ocCaching,meshCaching);
                if (!retVal)
                {
                    bool b=doCollide_shape(ocM,cellS,boxS,boxCenter,ocCacheValueHere,obbStruct,obb->obbNodes[1],shapeM,ocCaching,meshCaching);
                    retVal=retVal||b;
                }
            }
            else
            { // Explore the OC node
                if (ocNodes!=nullptr)
                {
                    unsigned long long int cellPath=(ocCacheValueHere>>6)<<(6+3);
                    unsigned long long int cellDepth=ocCacheValueHere&63;
                    for (size_t i=0;i<8;i++)
                    {
                        unsigned long long int _ocCacheValueHere=cellPath|(i<<6)|(cellDepth+1);
                        bool b=ocNodes[i]->doCollide_shape(ocM,cellS,boxS*0.5,boxCenter+ocNodeTranslations[i]*boxS,_ocCacheValueHere,obbStruct,obb,shapeM,ocCaching,meshCaching);
                        retVal=retVal||b;
                        if (retVal)
                            break;
                    }
                }
            }
        }
    }
    else
    { // this is a leaf node
        if (!empty)
        { // voxel exists
            C4X4Matrix m(shapeMRel*obb->boxM);
            if (CCalcUtils::doCollide_box_cell(m,obb->boxHs,boxHsp,true))
            { // the OBB box and the voxel collide
                if (obb->leafTris==nullptr)
                { // explore the OBB node
                    retVal=doCollide_shape(ocM,cellS,boxS,boxCenter,ocCacheValueHere,obbStruct,obb->obbNodes[0],shapeM,ocCaching,meshCaching);
                    if (!retVal)
                    {
                        bool b=doCollide_shape(ocM,cellS,boxS,boxCenter,ocCacheValueHere,obbStruct,obb->obbNodes[1],shapeM,ocCaching,meshCaching);
                        retVal=retVal||b;
                    }
                }
                else
                { // Check the triangles of the OBB leaf and the voxel
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
                        bool b=CCalcUtils::doCollide_cell_tri(boxHsp,true,p,v,w);
                        retVal=retVal||b;
                        if (b)
                        {
                            if (ocCaching!=nullptr)
                                ocCaching[0]=ocCacheValueHere;
                            if (meshCaching!=nullptr)
                                meshCaching[0]=ind/3;
                        }
                        if (retVal)
                            break;
                    }
                }
            }
        }
    }
    return(retVal);
}

bool COcNode::doCollide_octree(const C4X4Matrix& oc1M,double cell1S,double box1S,const C3Vector& box1Center,unsigned long long int oc1CacheValueHere,const COcNode* oc2Node,const C4X4Matrix& oc2M,double box2S,const C3Vector& box2Center,unsigned long long int oc2CacheValueHere,unsigned long long* oc1Caching,unsigned long long* oc2Caching) const
{
    bool retVal=false;
    double box1Hsp=box1S*double(0.5001);
    double box2Hsp=box2S*double(0.5001);
    C4X4Matrix m1(oc1M);
    m1.X+=oc1M.M*box1Center;
    C4X4Matrix m2(oc2M);
    m2.X+=oc2M.M*box2Center;
    C4X4Matrix oc2MRel(m1.getInverse()*m2);
    if (box1S>cell1S*double(1.5))
    { // not a leaf node yet..
        if (CCalcUtils::doCollide_box_cell(oc2MRel,C3Vector(box2Hsp,box2Hsp,box2Hsp),box1Hsp,true))
        { // the two ocnode boxes collide. Continue exploration by exploring the largest volume first:
            if ( (box1Hsp*box1Hsp*box1Hsp<box2Hsp*box2Hsp*box2Hsp)&&(oc2Node->ocNodes!=nullptr) )
            { // Explore the OC node 2
                unsigned long long int cellPath=(oc2CacheValueHere>>6)<<(6+3);
                unsigned long long int cellDepth=oc2CacheValueHere&63;
                for (size_t i=0;i<8;i++)
                {
                    unsigned long long int _oc2CacheValueThere=cellPath|(i<<6)|(cellDepth+1);
                    retVal=doCollide_octree(oc1M,cell1S,box1S,box1Center,oc1CacheValueHere,oc2Node->ocNodes[i],oc2M,box2S*0.5,box2Center+ocNodeTranslations[i]*box2S,_oc2CacheValueThere,oc1Caching,oc2Caching);
                    if (retVal)
                        break;
                }
            }
            else
            { // Explore the OC node 1
                if (ocNodes!=nullptr)
                {
                    unsigned long long int cellPath=(oc1CacheValueHere>>6)<<(6+3);
                    unsigned long long int cellDepth=oc1CacheValueHere&63;
                    for (size_t i=0;i<8;i++)
                    {
                        unsigned long long int _oc1CacheValueThere=cellPath|(i<<6)|(cellDepth+1);
                        retVal=ocNodes[i]->doCollide_octree(oc1M,cell1S,box1S*0.5,box1Center+ocNodeTranslations[i]*box1S,_oc1CacheValueThere,oc2Node,oc2M,box2S,box2Center,oc2CacheValueHere,oc1Caching,oc2Caching);
                        if (retVal)
                            break;
                    }
                }
            }
        }
    }
    else
    { // this is a leaf node
        if (!empty)
        { // voxel exists
            if (CCalcUtils::doCollide_box_cell(oc2MRel,C3Vector(box2Hsp,box2Hsp,box2Hsp),box1Hsp,true))
            { // the voxel of node1 and box2 collide
                if (oc2Node->ocNodes!=nullptr)
                { // explore the node 2 since we are not yet at its leafs
                    unsigned long long int cellPath=(oc2CacheValueHere>>6)<<(6+3);
                    unsigned long long int cellDepth=oc2CacheValueHere&63;
                    for (size_t i=0;i<8;i++)
                    {
                        unsigned long long int _oc2CacheValueThere=cellPath|(i<<6)|(cellDepth+1);
                        retVal=doCollide_octree(oc1M,cell1S,box1S,box1Center,oc1CacheValueHere,oc2Node->ocNodes[i],oc2M,box2S*0.5,box2Center+ocNodeTranslations[i]*box2S,_oc2CacheValueThere,oc1Caching,oc2Caching);
                        if (retVal)
                            break;
                    }
                }
                else
                {
                    retVal=!oc2Node->empty;
                    if (retVal)
                    {
                        if (oc1Caching!=nullptr)
                            oc1Caching[0]=oc1CacheValueHere;
                        if (oc2Caching!=nullptr)
                            oc2Caching[0]=oc2CacheValueHere;
                    }
                }
            }
        }
    }
    return(retVal);
}

bool COcNode::doCollide_ptcloud(const C4X4Matrix& ocM,double ocCellS,double ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CPcNode* pcNode,const C4X4Matrix& pcM,double pcBoxS,const C3Vector& pcBoxCenter,unsigned long long int pcCacheValueHere,unsigned long long* ocCaching,unsigned long long* pcCaching) const
{
    bool retVal=false;
    double ocBoxHsp=ocBoxS*double(0.5001);
    double pcBoxHsp=pcBoxS*double(0.5001);
    C4X4Matrix m1(ocM);
    m1.X+=ocM.M*ocBoxCenter;
    C4X4Matrix m2(pcM);
    m2.X+=pcM.M*pcBoxCenter;
    C4X4Matrix pcMRel(m1.getInverse()*m2);
    if (ocBoxS>ocCellS*double(1.5))
    { // not a leaf node yet for the octree..
        if (CCalcUtils::doCollide_box_cell(pcMRel,C3Vector(pcBoxHsp,pcBoxHsp,pcBoxHsp),ocBoxHsp,true))
        { // the OC node box and the PC node box collide. Continue exploration by exploring the largest volume first:
            if ( (ocBoxHsp*ocBoxHsp*ocBoxHsp<pcBoxHsp*pcBoxHsp*pcBoxHsp)&&(pcNode->pcNodes!=nullptr) )
            { // Explore the PC node
                unsigned long long int cellPath=(pcCacheValueHere>>6)<<(6+3);
                unsigned long long int cellDepth=pcCacheValueHere&63;
                for (size_t i=0;i<8;i++)
                {
                    unsigned long long int _pcCacheValueThere=cellPath|(i<<6)|(cellDepth+1);
                    retVal=doCollide_ptcloud(ocM,ocCellS,ocBoxS,ocBoxCenter,ocCacheValueHere,pcNode->pcNodes[i],pcM,pcBoxS*0.5,pcBoxCenter+ocNodeTranslations[i]*pcBoxS,_pcCacheValueThere,ocCaching,pcCaching);
                    if (retVal)
                        break;
                }
            }
            else
            { // Explore the OC node
                if (ocNodes!=nullptr)
                {
                    unsigned long long int cellPath=(ocCacheValueHere>>6)<<(6+3);
                    unsigned long long int cellDepth=ocCacheValueHere&63;
                    for (size_t i=0;i<8;i++)
                    {
                        unsigned long long int _ocCacheValueThere=cellPath|(i<<6)|(cellDepth+1);
                        retVal=ocNodes[i]->doCollide_ptcloud(ocM,ocCellS,ocBoxS*0.5,ocBoxCenter+ocNodeTranslations[i]*ocBoxS,_ocCacheValueThere,pcNode,pcM,pcBoxS,pcBoxCenter,pcCacheValueHere,ocCaching,pcCaching);
                        if (retVal)
                            break;
                    }
                }
            }
        }
    }
    else
    { // this is a leaf node for the octree
        if (!empty)
        { // voxel exists
            if (CCalcUtils::doCollide_box_cell(pcMRel,C3Vector(pcBoxHsp,pcBoxHsp,pcBoxHsp),ocBoxHsp,true))
            {  // the voxel of the OC node and the PC box collide
                if (pcNode->pcNodes!=nullptr)
                { // explore the PC node since we are not yet at its leafs yet
                    unsigned long long int cellPath=(pcCacheValueHere>>6)<<(6+3);
                    unsigned long long int cellDepth=pcCacheValueHere&63;
                    for (size_t i=0;i<8;i++)
                    {
                        unsigned long long int _pcCacheValueThere=cellPath|(i<<6)|(cellDepth+1);
                        retVal=doCollide_ptcloud(ocM,ocCellS,ocBoxS,ocBoxCenter,ocCacheValueHere,pcNode->pcNodes[i],pcM,pcBoxS*0.5,pcBoxCenter+ocNodeTranslations[i]*pcBoxS,_pcCacheValueThere,ocCaching,pcCaching);
                        if (retVal)
                            break;
                    }
                }
                else
                { // Check if the OC voxel collides with some points in the PC voxel
                    if (pcNode->pts.size()>0)
                    {
                        for (size_t i=0;i<pcNode->pts.size()/3;i++)
                        {
                            C3Vector pt(&pcNode->pts[3*i]);
                            pt*=pcMRel;
                            retVal=( (fabs(pt(0))<ocBoxHsp)&&(fabs(pt(1))<ocBoxHsp)&&(fabs(pt(2))<ocBoxHsp) );
                            if (retVal)
                            {
                                if (ocCaching!=nullptr)
                                    ocCaching[0]=ocCacheValueHere;
                                if (pcCaching!=nullptr)
                                    pcCaching[0]=pcCacheValueHere;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    return(retVal);
}

bool COcNode::getDistance_shape(const C4X4Matrix& ocM,double ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CObbStruct* obbStruct,const CObbNode* obb,const C4X4Matrix& shapeM,double& dist,C3Vector* ocMinDistSegPt,C3Vector* shapeMinDistSegPt,unsigned long long int* ocCaching,int* obbCaching) const
{
    bool retVal=false;
    if (dist==0.0)
        return(retVal);
    C4X4Matrix ocTr(ocM);
    ocTr.X+=ocM.M*ocBoxCenter;
    C4X4Matrix shapeTr(shapeM*obb->boxM);
    double dd=CCalcUtils::getApproxDistance_box_box(ocTr,C3Vector(ocBoxS*0.5,ocBoxS*0.5,ocBoxS*0.5),shapeTr,obb->boxHs);
    if (dd<dist)
    { // we may be closer than dist
        bool exploreObbNode=false;
        if (ocNodes!=nullptr)
        { // OC node can be explored further
            if ( (obb->leafTris!=nullptr)||(ocBoxS*ocBoxS*ocBoxS>obb->boxHs(0)*obb->boxHs(1)*obb->boxHs(2)) )
            { // explore the OC node. Pick the box closer to the OBB box
                C4X4Matrix shapeTrRel(ocTr.getInverse()*shapeTr);
                std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
                for (size_t i=0;i<8;i++)
                {
                    SNodeTranslation nodeTranslation;
                    nodeTranslation.transl=ocNodeTranslations[i]*ocBoxS;
                    nodeTranslation.index=i;
                    nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-shapeTrRel.X).getLength(),nodeTranslation));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                unsigned long long int cellPath=(ocCacheValueHere>>6)<<(6+3);
                unsigned long long int cellDepth=ocCacheValueHere&63;
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    C3Vector transl(nodesToExplore[i].second.transl);
                    size_t index=nodesToExplore[i].second.index;
                    unsigned long long int _ocCacheValueHere=cellPath|(index<<6)|(cellDepth+1);
                    bool bb=ocNodes[index]->getDistance_shape(ocM,ocBoxS*0.5,ocBoxCenter+transl,_ocCacheValueHere,obbStruct,obb,shapeM,dist,ocMinDistSegPt,shapeMinDistSegPt,ocCaching,obbCaching);
                    retVal=retVal||bb;
                }
            }
            else
                exploreObbNode=true;
        }
        else
        { // OC node cannot be further explored
            if (!empty)
            { // voxel exists
                if (obb->leafTris==nullptr)
                    exploreObbNode=true;
                else
                { // Check triangles of OBB node vs this voxel
                    C4X4Matrix shapeMRel(ocTr.getInverse()*shapeM);
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
                        if (CCalcUtils::getDistance_cell_tri(ocBoxS*0.5,true,p,v,w,dist,ocMinDistSegPt,shapeMinDistSegPt))
                        {
                            retVal=true;
                            if (ocMinDistSegPt!=nullptr)
                                ocMinDistSegPt[0]*=ocTr;
                            if (shapeMinDistSegPt!=nullptr)
                                shapeMinDistSegPt[0]*=ocTr;
                            if (ocCaching!=nullptr)
                                ocCaching[0]=ocCacheValueHere;
                            if (obbCaching!=nullptr)
                                obbCaching[0]=ind/3;
                        }
                    }
                }
            }
        }
        if (exploreObbNode)
        {
            const CObbNode* nodes[2];
            double d[2]={dist,dist};
            CCalcUtils::isApproxDistanceSmaller_box_box_fast(ocTr,C3Vector(ocBoxS,ocBoxS,ocBoxS),shapeM*obb->obbNodes[0]->boxM,obb->obbNodes[0]->boxHs,d[0]);
            CCalcUtils::isApproxDistanceSmaller_box_box_fast(ocTr,C3Vector(ocBoxS,ocBoxS,ocBoxS),shapeM*obb->obbNodes[1]->boxM,obb->obbNodes[1]->boxHs,d[1]);
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
                bool bb=getDistance_shape(ocM,ocBoxS,ocBoxCenter,ocCacheValueHere,obbStruct,nodes[i],shapeM,dist,ocMinDistSegPt,shapeMinDistSegPt,ocCaching,obbCaching);
                retVal=retVal||bb;
                if (dist<=d[1])
                    break;
            }
        }
    }
    return(retVal);
}

bool COcNode::getDistance_pt(const C3Vector& point,double boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,double& dist,C3Vector* minDistSegPt,unsigned long long int* caching) const
{
    bool retVal=false;
    if (dist==0.0)
        return(retVal);
    C3Vector relPoint(point-boxCenter);
    double ocBoxHsp=boxS*double(0.5001);
    double l=relPoint.getLength()-sqrtf(3*ocBoxHsp*ocBoxHsp);
    if (l<dist)
    { // we might be closer..
        if (empty)
        {
            if (ocNodes!=nullptr)
            { // continue exploration
                unsigned long long int cellPath=(cacheValueHere>>6)<<(6+3);
                unsigned long long int cellDepth=(cacheValueHere&63);
                for (size_t i=0;i<8;i++)
                {
                    unsigned long long int _cacheValueHere=cellPath|(i<<6)|(cellDepth+1);
                    bool bb=ocNodes[i]->getDistance_pt(point,boxS*0.5,boxCenter+ocNodeTranslations[i]*boxS,_cacheValueHere,dist,minDistSegPt,caching);
                    retVal=retVal||bb;
                }
            }
        }
        else
        { // voxel exists
            C3Vector pt(relPoint);
            double ocBoxHs=boxS*0.5;
            if (fabs(pt(0))>ocBoxHs)
                pt(0)=ocBoxHs*pt(0)/fabs(pt(0));
            if (fabs(pt(1))>ocBoxHs)
                pt(1)=ocBoxHs*pt(1)/fabs(pt(1));
            if (fabs(pt(2))>ocBoxHs)
                pt(2)=ocBoxHs*pt(2)/fabs(pt(2));
            l=(relPoint-pt).getLength();
            if (l<dist)
            {
                retVal=true;
                dist=l;
                if (minDistSegPt!=nullptr)
                    minDistSegPt[0]=pt+boxCenter;
                if (caching!=nullptr)
                    caching[0]=cacheValueHere;
            }
        }
    }
    return(retVal);
}

bool COcNode::getDistance_seg(const C3Vector& segMiddle,const C3Vector& segHs,double boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,double& dist,C3Vector* ocMinDistSegPt,C3Vector* segMinDistSegPt,unsigned long long int* caching) const
{
    bool retVal=false;
    if (dist==0.0)
        return(retVal);
    C3Vector relSegMiddle(segMiddle-boxCenter);
    double ocBoxHsp=boxS*double(0.5001);
    double l=relSegMiddle.getLength()-sqrtf(3*ocBoxHsp*ocBoxHsp)-segHs.getLength();
    if (l<dist)
    { // we might be closer..
        if (empty)
        {
            if (ocNodes!=nullptr)
            { // continue exploration
                unsigned long long int cellPath=(cacheValueHere>>6)<<(6+3);
                unsigned long long int cellDepth=(cacheValueHere&63);
                for (size_t i=0;i<8;i++)
                {
                    unsigned long long int _cacheValueHere=cellPath|(i<<6)|(cellDepth+1);
                    bool bb=ocNodes[i]->getDistance_seg(segMiddle,segHs,boxS*0.5,boxCenter+ocNodeTranslations[i]*boxS,_cacheValueHere,dist,ocMinDistSegPt,segMinDistSegPt,caching);
                    retVal=retVal||bb;
                }
            }
        }
        else
        { // voxel exists
            if (CCalcUtils::getDistance_cell_segp(ocBoxHsp,true,relSegMiddle-segHs,segHs*2.0,dist,ocMinDistSegPt,segMinDistSegPt))
            {
                retVal=true;
                if (ocMinDistSegPt!=nullptr)
                    ocMinDistSegPt[0]+=boxCenter;
                if (segMinDistSegPt!=nullptr)
                    segMinDistSegPt[0]+=boxCenter;
                if (caching!=nullptr)
                    caching[0]=cacheValueHere;
            }
        }
    }
    return(retVal);
}

bool COcNode::getDistance_tri(const C3Vector& p,const C3Vector& v,const C3Vector& w,double boxS,const C3Vector& boxCenter,unsigned long long int cacheValueHere,double& dist,C3Vector* ocMinDistSegPt,C3Vector* triMinDistSegPt,unsigned long long int* caching) const
{
    bool retVal=false;
    if (dist==0.0)
        return(retVal);
    C3Vector relP(p-boxCenter);
    double ocBoxHsp=boxS*double(0.5001);
    double l=relP.getLength()-sqrtf(3*ocBoxHsp*ocBoxHsp)-std::max<double>(v.getLength(),w.getLength());
    if (l<dist)
    { // we might be closer..
        if (empty)
        {
            if (ocNodes!=nullptr)
            { // continue exploration
                unsigned long long int cellPath=(cacheValueHere>>6)<<(6+3);
                unsigned long long int cellDepth=(cacheValueHere&63);
                for (size_t i=0;i<8;i++)
                {
                    unsigned long long int _cacheValueHere=cellPath|(i<<6)|(cellDepth+1);
                    bool bb=ocNodes[i]->getDistance_tri(p,v,w,boxS*0.5,boxCenter+ocNodeTranslations[i]*boxS,_cacheValueHere,dist,ocMinDistSegPt,triMinDistSegPt,caching);
                    retVal=retVal||bb;
                }
            }
        }
        else
        { // voxel exists
            if (CCalcUtils::getDistance_cell_tri(ocBoxHsp,true,relP,v,w,dist,ocMinDistSegPt,triMinDistSegPt))
            {
                retVal=true;
                if (ocMinDistSegPt!=nullptr)
                    ocMinDistSegPt[0]+=boxCenter;
                if (triMinDistSegPt!=nullptr)
                    triMinDistSegPt[0]+=boxCenter;
                if (caching!=nullptr)
                    caching[0]=cacheValueHere;
            }
        }
    }
    return(retVal);
}

bool COcNode::getDistance_octree(const C4X4Matrix& oc1M,double ocBox1S,const C3Vector& ocBox1Center,unsigned long long int oc1CacheValueHere,const COcNode* oc2Node,const C4X4Matrix& oc2M,double ocBox2S,const C3Vector& ocBox2Center,unsigned long long int oc2CacheValueHere,double& dist,C3Vector* oc1MinDistSegPt,C3Vector* oc2MinDistSegPt,unsigned long long int* oc1Caching,unsigned long long int* oc2Caching) const
{
    bool retVal=false;
    if (dist==0.0)
        return(retVal);
    C4X4Matrix m1(oc1M);
    m1.X+=oc1M.M*ocBox1Center;
    C4X4Matrix m2(oc2M);
    m2.X+=oc2M.M*ocBox2Center;
    double dd=CCalcUtils::getApproxDistance_box_box(m1,C3Vector(ocBox1S*0.5,ocBox1S*0.5,ocBox1S*0.5),m2,C3Vector(ocBox2S*0.5,ocBox2S*0.5,ocBox2S*0.5));
    if (dd<dist)
    { // we might be closer
        int nodeToExplore=-1;
        if (ocNodes!=nullptr)
        {
            if ( (oc2Node->ocNodes!=nullptr)||(!oc2Node->empty) )
            {
                if ( (ocBox1S>ocBox2S)||(!oc2Node->empty) )
                    nodeToExplore=0;
                else
                    nodeToExplore=1;
            }
        }
        else
        {
            if (!empty)
            {
                if (oc2Node->ocNodes!=nullptr)
                    nodeToExplore=1;
                else
                {
                    if (!oc2Node->empty)
                    { // check the voxel vs voxel:
                        if (CCalcUtils::getDistance_cell_cell(m1,ocBox1S*0.5,m2,ocBox2S*0.5,true,dist,oc1MinDistSegPt,oc2MinDistSegPt))
                        {
                            retVal=true;
                            if (oc1Caching!=nullptr)
                                oc1Caching[0]=oc1CacheValueHere;
                            if (oc2Caching!=nullptr)
                                oc2Caching[0]=oc2CacheValueHere;
                        }
                    }
                }
            }
        }
        if (nodeToExplore!=-1)
        { // continue exploration
            int nodeNotToExplore=0;
            if (nodeToExplore==0)
                nodeNotToExplore=1;
            const COcNode* twoNodes[2]={this,oc2Node};
            const C4X4Matrix* twoOcMs[2]={&oc1M,&oc2M};
            const double twoOcBoxSizes[2]={ocBox1S,ocBox2S};
            const C3Vector* twoOcBoxCenters[2]={&ocBox1Center,&ocBox2Center};
            const unsigned long long int twoOcCacheValuesHere[2]={oc1CacheValueHere,oc2CacheValueHere};
            C3Vector* twoOcMinDistSegPts[2]={oc1MinDistSegPt,oc2MinDistSegPt};
            unsigned long long int* twoOcCaching[2]={oc1Caching,oc2Caching};
            const C4X4Matrix twoMRelBoxes[2]={m1.getInverse()*m2,m2.getInverse()*m1};
            // Now check all 8 node pairs, explore closest node first:
            std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
            for (size_t i=0;i<8;i++)
            {
                SNodeTranslation nodeTranslation;
                nodeTranslation.transl=ocNodeTranslations[i]*twoOcBoxSizes[nodeToExplore];
                nodeTranslation.index=i;
                nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-twoMRelBoxes[nodeToExplore].X).getLength(),nodeTranslation));
            }
            std::sort(nodesToExplore.begin(),nodesToExplore.end());
            unsigned long long int cellPath=(twoOcCacheValuesHere[nodeToExplore]>>6)<<(6+3);
            unsigned long long int cellDepth=(twoOcCacheValuesHere[nodeToExplore]&63);
            for (size_t i=0;i<nodesToExplore.size();i++)
            {
                C3Vector transl(nodesToExplore[i].second.transl);
                size_t index=nodesToExplore[i].second.index;
                unsigned long long int _cacheValueHere=cellPath|(index<<6)|(cellDepth+1);
                bool bb=twoNodes[nodeToExplore]->ocNodes[index]->getDistance_octree(twoOcMs[nodeToExplore][0],twoOcBoxSizes[nodeToExplore]*0.5,twoOcBoxCenters[nodeToExplore][0]+transl,_cacheValueHere,twoNodes[nodeNotToExplore],twoOcMs[nodeNotToExplore][0],twoOcBoxSizes[nodeNotToExplore],twoOcBoxCenters[nodeNotToExplore][0],twoOcCacheValuesHere[nodeNotToExplore],dist,twoOcMinDistSegPts[nodeToExplore],twoOcMinDistSegPts[nodeNotToExplore],twoOcCaching[nodeToExplore],twoOcCaching[nodeNotToExplore]);
                retVal=retVal||bb;
            }
        }
    }
    return(retVal);
}

bool COcNode::getDistance_ptcloud(const C4X4Matrix& ocM,double ocBoxS,const C3Vector& ocBoxCenter,unsigned long long int ocCacheValueHere,const CPcNode* pcNode,const C4X4Matrix& pcM,const C3Vector& pcBoxCenter,double pcBoxS,unsigned long long int pcCacheValueHere,double& dist,C3Vector* ocMinDistSegPt,C3Vector* pcMinDistSegPt,unsigned long long int* ocCaching,unsigned long long int* pcCaching) const
{
    bool retVal=false;
    if (dist==0.0)
        return(retVal);
    C4X4Matrix m1(ocM);
    m1.X+=ocM.M*ocBoxCenter;
    C4X4Matrix m2(pcM);
    m2.X+=pcM.M*pcBoxCenter;
    double dd=CCalcUtils::getApproxDistance_box_box(m1,C3Vector(ocBoxS*0.5,ocBoxS*0.5,ocBoxS*0.5),m2,C3Vector(pcBoxS*0.5,pcBoxS*0.5,pcBoxS*0.5));
    if (dd<dist)
    { // we might be closer
        bool explorePc=false;
        if (ocNodes!=nullptr)
        {
            if ( (pcNode->pcNodes!=nullptr)||(pcNode->pts.size()>0) )
            {
                if ( (ocBoxS>pcBoxS)||(pcNode->pts.size()>0) )
                {
                    std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
                    C4X4Matrix box2Rel(m1.getInverse()*m2);
                    for (size_t i=0;i<8;i++)
                    {
                        SNodeTranslation nodeTranslation;
                        nodeTranslation.transl=ocNodeTranslations[i]*ocBoxS;
                        nodeTranslation.index=i;
                        nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-box2Rel.X).getLength(),nodeTranslation));
                    }
                    std::sort(nodesToExplore.begin(),nodesToExplore.end());
                    unsigned long long int cellPath=(ocCacheValueHere>>6)<<(6+3);
                    unsigned long long int cellDepth=(ocCacheValueHere&63);
                    for (size_t i=0;i<nodesToExplore.size();i++)
                    {
                        C3Vector transl(nodesToExplore[i].second.transl);
                        size_t index=nodesToExplore[i].second.index;
                        unsigned long long int _ocCacheValueHere=cellPath|(index<<6)|(cellDepth+1);
                        bool bb=ocNodes[index]->getDistance_ptcloud(ocM,ocBoxS*0.5,ocBoxCenter+transl,_ocCacheValueHere,pcNode,pcM,pcBoxCenter,pcBoxS,pcCacheValueHere,dist,ocMinDistSegPt,pcMinDistSegPt,ocCaching,pcCaching);
                        retVal=retVal||bb;
                    }
                }
                else
                    explorePc=true;
            }
        }
        else
        {
            if (!empty)
            {
                if (pcNode->pcNodes==nullptr)
                {
                    if (pcNode->pts.size()>0)
                    {
                        C4X4Matrix box2Rel(m1.getInverse()*m2);
                        for (size_t i=0;i<pcNode->pts.size()/3;i++)
                        {
                            C3Vector pt(&pcNode->pts[3*i]);
                            pt*=box2Rel;
                            C3Vector ptVoxel(pt);
                            if (fabs(ptVoxel(0))>ocBoxS*0.5)
                                ptVoxel(0)=ocBoxS*0.5*fabs(ptVoxel(0))/ptVoxel(0);
                            if (fabs(ptVoxel(1))>ocBoxS*0.5)
                                ptVoxel(1)=ocBoxS*0.5*fabs(ptVoxel(1))/ptVoxel(1);
                            if (fabs(ptVoxel(2))>ocBoxS*0.5)
                                ptVoxel(2)=ocBoxS*0.5*fabs(ptVoxel(2))/ptVoxel(2);
                            dd=(pt-ptVoxel).getLength();
                            if (dd<dist)
                            {
                                retVal=true;
                                dist=dd;
                                if (ocMinDistSegPt!=nullptr)
                                    ocMinDistSegPt[0]=m1*ptVoxel;
                                if (pcMinDistSegPt!=nullptr)
                                    pcMinDistSegPt[0]=m1*pt;
                                if (ocCaching!=nullptr)
                                    ocCaching[0]=ocCacheValueHere;
                                if (pcCaching!=nullptr)
                                    pcCaching[0]=pcCacheValueHere;
                            }
                        }
                    }
                }
                else
                    explorePc=true;
            }
        }
        if (explorePc)
        {
            std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
            C4X4Matrix box1Rel(m2.getInverse()*m1);
            for (size_t i=0;i<8;i++)
            {
                SNodeTranslation nodeTranslation;
                nodeTranslation.transl=ocNodeTranslations[i]*pcBoxS;
                nodeTranslation.index=i;
                nodesToExplore.push_back(std::make_pair((nodeTranslation.transl-box1Rel.X).getLength(),nodeTranslation));
            }
            std::sort(nodesToExplore.begin(),nodesToExplore.end());
            unsigned long long int cellPath=(pcCacheValueHere>>6)<<(6+3);
            unsigned long long int cellDepth=(pcCacheValueHere&63);
            for (size_t i=0;i<nodesToExplore.size();i++)
            {
                C3Vector transl(nodesToExplore[i].second.transl);
                size_t index=nodesToExplore[i].second.index;
                unsigned long long int _pcCacheValueHere=cellPath|(index<<6)|(cellDepth+1);
                bool bb=getDistance_ptcloud(ocM,ocBoxS,ocBoxCenter,ocCacheValueHere,pcNode->pcNodes[index],pcM,pcBoxCenter+transl,pcBoxS*0.5,_pcCacheValueHere,dist,ocMinDistSegPt,pcMinDistSegPt,ocCaching,pcCaching);
                retVal=retVal||bb;
            }
        }
    }
    return(retVal);
}

bool COcNode::getSensorDistance(const C4X4Matrix& ocM,double boxS,const C3Vector& boxCenter,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,double cosAngle,bool frontDetection,bool backDetection,bool fast,double& dist,C3Vector* detectPt,C3Vector* triN) const
{
    bool retVal=false;
    if (dist==0.0)
        return(retVal);
    C4X4Matrix m(ocM);
    m.X+=ocM.M*boxCenter;
    double dd=DBL_MAX;
    CCalcUtils::getDistance_box_pt(m,C3Vector(boxS*0.5,boxS*0.5,boxS*0.5),true,C3Vector::zeroVector,dd,nullptr,nullptr);
    if (dd<dist)
    { // we are closer..
        if (CCalcUtils::isBoxMaybeInSensorVolume(planesIn,planesOut,m,C3Vector(boxS*0.5,boxS*0.5,boxS*0.5)))
        {
            if (ocNodes!=nullptr)
            { // continue exploration
                std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
                for (size_t i=0;i<8;i++)
                {
                    SNodeTranslation nodeTranslation;
                    nodeTranslation.transl=ocNodeTranslations[i]*boxS;
                    nodeTranslation.index=i;
                    nodesToExplore.push_back(std::make_pair((m.X+ocM.M*nodeTranslation.transl).getLength(),nodeTranslation));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    C3Vector transl(nodesToExplore[i].second.transl);
                    size_t index=nodesToExplore[i].second.index;
                    bool bb=ocNodes[index]->getSensorDistance(ocM,boxS*0.5,boxCenter+transl,planesIn,planesOut,cosAngle,frontDetection,backDetection,fast,dist,detectPt,triN);
                    retVal=retVal||bb;
                    if (retVal&&fast)
                        break;
                }
            }
            else
            {
                if (!empty)
                    retVal=CCalcUtils::getSensorDistance_cell(planesIn,planesOut,cosAngle,frontDetection,backDetection,m,boxS*0.5,dist,detectPt,triN);
            }
        }
    }
    return(retVal);
}

bool COcNode::getRaySensorDistance(const C4X4Matrix& ocM,double boxS,const C3Vector& boxCenter,const C3Vector& raySegP,const C3Vector& raySegL,double forbiddenDist,double cosAngle,bool frontDetection,bool backDetection,bool fast,double& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched) const
{
    bool retVal=false;
    if (dist==0.0)
        return(retVal);
    C4X4Matrix m(ocM);
    m.X+=ocM.M*boxCenter;
    double dd=DBL_MAX;
    CCalcUtils::getDistance_box_pt(m,C3Vector(boxS*0.5,boxS*0.5,boxS*0.5),true,C3Vector::zeroVector,dd,nullptr,nullptr);
    if (dd<dist)
    { // we are closer..
        if (CCalcUtils::doCollide_box_seg(m,C3Vector(boxS*0.5,boxS*0.5,boxS*0.5),true,raySegP+raySegL*0.5,raySegL*0.5))
        { // we collide..
            if (ocNodes!=nullptr)
            { // continue exploration
                std::vector<std::pair<double,SNodeTranslation>> nodesToExplore;
                for (size_t i=0;i<8;i++)
                {
                    SNodeTranslation nodeTranslation;
                    nodeTranslation.transl=ocNodeTranslations[i]*boxS;
                    nodeTranslation.index=i;
                    nodesToExplore.push_back(std::make_pair((m.X+ocM.M*nodeTranslation.transl).getLength(),nodeTranslation));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    C3Vector transl(nodesToExplore[i].second.transl);
                    size_t index=nodesToExplore[i].second.index;
                    bool bb=ocNodes[index]->getRaySensorDistance(ocM,boxS*0.5,boxCenter+transl,raySegP,raySegL,forbiddenDist,cosAngle,frontDetection,backDetection,fast,dist,detectPt,triN,forbiddenDistTouched);
                    retVal=retVal||bb;
                    if ( retVal&&(fast||((forbiddenDistTouched!=nullptr)&&forbiddenDistTouched[0])) )
                        break;
                }
            }
            else
            {
                if (!empty)
                    retVal=CCalcUtils::getRaySensorDistance_cell(raySegP,raySegL,cosAngle,frontDetection,backDetection,forbiddenDist,m,boxS*0.5,forbiddenDistTouched,dist,detectPt,triN);
            }
        }
    }
    return(retVal);
}
