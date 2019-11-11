#include "calcUtils.h"
#include "obbStruct.h"

std::vector<CObbStruct*> CObbStruct::_obbStructs;

CObbStruct::CObbStruct()
{
}

CObbStruct::CObbStruct(const float* ver,int verSize,const int* ind,int indSize,float triSize,int triCnt)
{
    _triCnt=triCnt;
    if (triSize<0.005f)
        triSize=0.005f;
    _triSize=triSize;
    vertices.assign(ver,ver+verSize);
    indices.assign(ind,ind+indSize);
    _originalVerticesSize=verSize;
    _originalIndicesSize=indSize;
    _originalVerticesSum=0.0f;
    for (int i=0;i<verSize;i++)
        _originalVerticesSum+=ver[i];
    _originalIndicesSum=0;
    for (int i=0;i<indSize;i++)
        _originalIndicesSum+=ind[i];

    reduceTriangleSizes(vertices,indices,_triSize);

    std::vector<int> usedTraingles;
    for (size_t i=0;i<indices.size()/3;i++)
        usedTraingles.push_back(i);

    obb=new CObbNode(vertices,indices,usedTraingles,_triCnt);
}

CObbStruct::~CObbStruct()
{
    delete obb;
}

CObbStruct* CObbStruct::copyYourself() const
{
    CObbStruct* newObbStruct=new CObbStruct();
    newObbStruct->obb=obb->copyYourself();
    newObbStruct->vertices.assign(vertices.begin(),vertices.end());
    newObbStruct->indices.assign(indices.begin(),indices.end());
    newObbStruct->_originalVerticesSize=_originalVerticesSize;
    newObbStruct->_originalVerticesSum=_originalVerticesSum;
    newObbStruct->_originalIndicesSize=_originalIndicesSize;
    newObbStruct->_originalIndicesSum=_originalIndicesSum;
    newObbStruct->_triCnt=_triCnt;
    newObbStruct->_triSize=_triSize;
    return(newObbStruct);
}

void CObbStruct::scaleYourself(float f)
{
    obb->scaleYourself(f);
    for (size_t i=0;i<vertices.size();i++)
        vertices[i]*=f;
}

bool CObbStruct::isSame(const float* v,int vSize,const int* ind,int indSize,float triSize,int triCnt)
{
    if (_originalVerticesSize!=vSize)
        return(false);
    if (_originalIndicesSize!=indSize)
        return(false);
    if (triSize!=_triSize)
        return(false);
    if (triCnt!=_triCnt)
        return(false);

    float s=0.0f;
    for (size_t i=0;i<vSize;i++)
        s+=v[i];
    if (s!=_originalVerticesSum)
        return(false);

    int is=0;
    for (size_t i=0;i<indSize;i++)
        is+=ind[i];
    if (is!=_originalIndicesSum)
        return(false);

    return(true);
}

unsigned char* CObbStruct::serialize(int& dataSize) const
{
    std::vector<unsigned char> data;

    data.push_back(2); // ser ver

    pushData(data,&_triSize,sizeof(int));
    pushData(data,&_triCnt,sizeof(int));
    pushData(data,&_originalVerticesSize,sizeof(int));
    pushData(data,&_originalVerticesSum,sizeof(float));
    pushData(data,&_originalIndicesSize,sizeof(int));
    pushData(data,&_originalIndicesSum,sizeof(int));

    int s=(int)vertices.size();
    pushData(data,&s,sizeof(int));
    for (size_t i=0;i<vertices.size();i++)
        pushData(data,&vertices[i],sizeof(float));

    s=(int)indices.size();
    pushData(data,&s,sizeof(int));
    for (size_t i=0;i<indices.size();i++)
        pushData(data,&indices[i],sizeof(int));

    obb->serialize(data);

    unsigned char* retVal=new unsigned char[data.size()];
    for (size_t i=0;i<data.size();i++)
        retVal[i]=data[i];
    dataSize=(int)data.size();

    return(retVal);
}

void CObbStruct::deserialize(const unsigned char* data)
{
    int pos=1;

    _triSize=((float*)(data+pos))[0];pos+=sizeof(float);
    _triCnt=((int*)(data+pos))[0];pos+=sizeof(int);
    _originalVerticesSize=((int*)(data+pos))[0];pos+=sizeof(int);
    _originalVerticesSum=((float*)(data+pos))[0];pos+=sizeof(float);
    _originalIndicesSize=((int*)(data+pos))[0];pos+=sizeof(int);
    _originalIndicesSum=((int*)(data+pos))[0];pos+=sizeof(int);

    int s;
    s=((int*)(data+pos))[0];pos+=sizeof(int);
    for (int i=0;i<s;i++)
    {
        vertices.push_back(((float*)(data+pos))[0]);
        pos+=sizeof(float);
    }

    s=((int*)(data+pos))[0];pos+=sizeof(int);
    for (int i=0;i<s;i++)
    {
        indices.push_back(((int*)(data+pos))[0]);
        pos+=sizeof(int);
    }

    obb=new CObbNode();
    obb->deserialize(data,pos);
}

CObbStruct* CObbStruct::copyObbStructFromExisting(const float* vert,int vertSize,const int* ind,int indSize,float triSize,int triCnt)
{
    for (size_t i=0;i<_obbStructs.size();i++)
    {
        if (_obbStructs[i]->isSame(vert,vertSize,ind,indSize,triSize,triCnt))
            return(_obbStructs[i]->copyYourself());
    }
    return(nullptr);
}

void CObbStruct::addObbStruct(CObbStruct* obbStruct)
{
    _obbStructs.push_back(obbStruct);
}

void CObbStruct::removeObbStruct(CObbStruct* obbStruct)
{
    for (size_t i=0;i<_obbStructs.size();i++)
    {
        if (_obbStructs[i]==obbStruct)
        {
            _obbStructs.erase(_obbStructs.begin()+i);
            delete obbStruct;
            break;
        }
    }
}

void CObbStruct::reduceTriangleSizes(std::vector<float>& vert,std::vector<int>& ind,float triSize)
{
    std::vector<int> t1(ind);
    for (size_t loop=0;loop<8;loop++)
    {
        std::vector<int> t2;
        for (size_t i=0;i<t1.size()/3;i++)
        {
            int indd[5];
            indd[0]=t1[3*i+0];
            indd[1]=t1[3*i+1];
            indd[2]=t1[3*i+2];
            indd[3]=indd[0];
            indd[4]=indd[1];
            C3Vector p1(&vert[3*indd[0]]);
            C3Vector p2(&vert[3*indd[1]]);
            C3Vector p3(&vert[3*indd[2]]);
            C3Vector pts[5]={p1,p2,p3,p1,p2};
            C3Vector edges[3];
            float ll[3];
            edges[0]=p2-p1;
            edges[1]=p3-p2;
            edges[2]=p1-p3;
            ll[0]=edges[0].getLength();
            ll[1]=edges[1].getLength();
            ll[2]=edges[2].getLength();
            int a=0;
            if (ll[1]>ll[0])
            {
                a=1;
                if (ll[2]>ll[1])
                    a=2;
            }
            else
            {
                if (ll[2]>ll[0])
                    a=2;
            }
            if (ll[a]>triSize)
            { // tri gets divided
                C3Vector np(pts[a]+edges[a]*0.5f);
                t2.push_back(indd[a]+0);
                t2.push_back(vert.size()/3);
                t2.push_back(indd[a+2]);
                t2.push_back(vert.size()/3);
                t2.push_back(indd[a+1]);
                t2.push_back(indd[a+2]);
                vert.push_back(np(0));
                vert.push_back(np(1));
                vert.push_back(np(2));
            }
            else
            { // tri remains same
                t2.push_back(t1[3*i+0]);
                t2.push_back(t1[3*i+1]);
                t2.push_back(t1[3*i+2]);
            }
        }

        if (t2.size()==t1.size())
            break;
        else
            t1.assign(t2.begin(),t2.end());
    }
    ind.assign(t1.begin(),t1.end());
}
