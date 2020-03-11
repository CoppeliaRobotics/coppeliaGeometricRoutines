#pragma once

#include "conf.h"
#include "obbNode.h"

class CObbStruct
{
public:
    CObbStruct();
    CObbStruct(const simReal* ver,int verSize,const int* ind,int indSize,simReal triSize,int triCnt);
    virtual ~CObbStruct();

    CObbStruct* copyYourself() const;
    void scaleYourself(simReal f);
    bool isSame(const simReal* v,int vSize,const int* ind,int indSize,simReal triSize,int triCnt);
    unsigned char* serialize(int& dataSize) const;
    bool deserialize(const unsigned char* data);

    static void addObbStruct(CObbStruct* obbStruct);
    static void removeObbStruct(CObbStruct* obbStruct);
    static CObbStruct* copyObbStructFromExisting(const simReal* vert,int vertSize,const int* ind,int indSize,simReal triSize,int triCnt);
    static void reduceTriangleSizes(std::vector<simReal>& vert,std::vector<int>& ind,simReal triSize);

    CObbNode* obb;
    std::vector<simReal> vertices;
    std::vector<int> indices;

private:
    int _originalVerticesSize;
    unsigned long _originalVerticesHash;
    int _originalIndicesSize;
    unsigned long _originalIndicesHash;
    simReal _triSize;
    int _triCnt;

    static std::vector<CObbStruct*> _obbStructs;
};
