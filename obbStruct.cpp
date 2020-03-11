#include "calcUtils.h"
#include "obbStruct.h"

std::vector<CObbStruct*> CObbStruct::_obbStructs;

CObbStruct::CObbStruct()
{
}

CObbStruct::CObbStruct(const simReal* ver,int verSize,const int* ind,int indSize,simReal triSize,int triCnt)
{
    _triCnt=triCnt;
    if (triSize<=simZero)
        triSize=simZero;
    else
    {
        if (triSize<simReal(0.005))
            triSize=simReal(0.005);
    }
    _triSize=triSize;
    vertices.assign(ver,ver+verSize);
    indices.assign(ind,ind+indSize);
    _originalVerticesSize=verSize;
    _originalIndicesSize=indSize;

    _originalVerticesHash=CCalcUtils::getDjb2Hash(reinterpret_cast<const char*>(ver),size_t(verSize)*sizeof(simReal));
    _originalIndicesHash=CCalcUtils::getDjb2Hash(reinterpret_cast<const char*>(ind),size_t(indSize)*sizeof(int));

    reduceTriangleSizes(vertices,indices,_triSize);

    std::vector<int> usedTraingles;
    for (size_t i=0;i<indices.size()/3;i++)
        usedTraingles.push_back(int(i));

    obb=new CObbNode(vertices,indices,usedTraingles,size_t(_triCnt));
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
    newObbStruct->_originalVerticesHash=_originalVerticesHash;
    newObbStruct->_originalIndicesSize=_originalIndicesSize;
    newObbStruct->_originalIndicesHash=_originalIndicesHash;
    newObbStruct->_triCnt=_triCnt;
    newObbStruct->_triSize=_triSize;
    return(newObbStruct);
}

void CObbStruct::scaleYourself(simReal f)
{
    obb->scaleYourself(f);
    for (size_t i=0;i<vertices.size();i++)
        vertices[i]*=f;
}

bool CObbStruct::isSame(const simReal* v,int vSize,const int* ind,int indSize,simReal triSize,int triCnt)
{
    if (_originalVerticesSize!=vSize)
        return(false);
    if (_originalIndicesSize!=indSize)
        return(false);
    if (triSize!=_triSize)
        return(false);
    if (triCnt!=_triCnt)
        return(false);
    if (_originalVerticesHash==0)
        return(false);
    if (_originalIndicesHash==0)
        return(false);

    unsigned long hash=CCalcUtils::getDjb2Hash(reinterpret_cast<const char*>(v),size_t(vSize)*sizeof(simReal));
    if (hash!=_originalVerticesHash)
        return(false);

    hash=CCalcUtils::getDjb2Hash(reinterpret_cast<const char*>(ind),size_t(indSize)*sizeof(int));
    if (hash!=_originalIndicesHash)
        return(false);

    return(true);
}

unsigned char* CObbStruct::serialize(int& dataSize) const
{
    std::vector<unsigned char> data;

    data.push_back(3); // ser ver

    pushData(data,&_triSize,sizeof(int));
    pushData(data,&_triCnt,sizeof(int));
    pushData(data,&_originalVerticesSize,sizeof(int));
    pushData(data,&_originalVerticesHash,sizeof(unsigned long));
    pushData(data,&_originalIndicesSize,sizeof(int));
    pushData(data,&_originalIndicesHash,sizeof(unsigned long));

    int s=int(vertices.size());
    pushData(data,&s,sizeof(int));
    for (size_t i=0;i<vertices.size();i++)
        pushData(data,&vertices[i],sizeof(simReal));

    s=int(indices.size());
    pushData(data,&s,sizeof(int));
    for (size_t i=0;i<indices.size();i++)
        pushData(data,&indices[i],sizeof(int));

    obb->serialize(data);

    unsigned char* retVal=new unsigned char[data.size()];
    for (size_t i=0;i<data.size();i++)
        retVal[i]=data[i];
    dataSize=int(data.size());

    return(retVal);
}

bool CObbStruct::deserialize(const unsigned char* data)
{
    int pos=0;
    unsigned char ver=data[pos++];
    if (ver<=3)
    {
        _triSize=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
        _triCnt=(reinterpret_cast<const int*>(data+pos))[0];pos+=sizeof(int);
        _originalVerticesSize=(reinterpret_cast<const int*>(data+pos))[0];pos+=sizeof(int);
        if (ver==2)
        {
            _originalVerticesHash=0;
            pos+=sizeof(simReal);
        }
        else
        {
            _originalVerticesHash=(reinterpret_cast<const unsigned long*>(data+pos))[0];
            pos+=sizeof(unsigned long);
        }
        _originalIndicesSize=(reinterpret_cast<const int*>(data+pos))[0];pos+=sizeof(int);
        if (ver==2)
        {
            _originalIndicesHash=0;
            pos+=sizeof(int);
        }
        else
        {
            _originalIndicesHash=(reinterpret_cast<const unsigned long*>(data+pos))[0];
            pos+=sizeof(unsigned long);
        }

        int s;
        s=(reinterpret_cast<const int*>(data+pos))[0];pos+=sizeof(int);
        for (int i=0;i<s;i++)
        {
            vertices.push_back((reinterpret_cast<const simReal*>(data+pos))[0]);
            pos+=sizeof(simReal);
        }

        s=(reinterpret_cast<const int*>(data+pos))[0];pos+=sizeof(int);
        for (int i=0;i<s;i++)
        {
            indices.push_back((reinterpret_cast<const int*>(data+pos))[0]);
            pos+=sizeof(int);
        }

        obb=new CObbNode();
        obb->deserialize(data,pos);
        return(true);
    }
    return(false);
}

CObbStruct* CObbStruct::copyObbStructFromExisting(const simReal* vert,int vertSize,const int* ind,int indSize,simReal triSize,int triCnt)
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

void CObbStruct::reduceTriangleSizes(std::vector<simReal>& vert,std::vector<int>& ind,simReal triSize)
{
    if (triSize>simZero)
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
                C3Vector p1(&vert[3*size_t(indd[0])]);
                C3Vector p2(&vert[3*size_t(indd[1])]);
                C3Vector p3(&vert[3*size_t(indd[2])]);
                C3Vector pts[5]={p1,p2,p3,p1,p2};
                C3Vector edges[3];
                simReal ll[3];
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
                    C3Vector np(pts[a]+edges[a]*simHalf);
                    t2.push_back(indd[a]+0);
                    t2.push_back(int(vert.size()/3));
                    t2.push_back(indd[a+2]);
                    t2.push_back(int(vert.size()/3));
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
}
