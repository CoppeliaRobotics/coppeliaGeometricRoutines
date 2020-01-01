#pragma once

#include "obbNode.h"

class CObbStruct
{
public:
    CObbStruct();
    CObbStruct(const float* ver,int verSize,const int* ind,int indSize,float triSize,int triCnt);
    virtual ~CObbStruct();

    CObbStruct* copyYourself() const;
    void scaleYourself(float f);
    bool isSame(const float* v,int vSize,const int* ind,int indSize,float triSize,int triCnt);
    unsigned char* serialize(int& dataSize) const;
    bool deserialize(const unsigned char* data);

    static void addObbStruct(CObbStruct* obbStruct);
    static void removeObbStruct(CObbStruct* obbStruct);
    static CObbStruct* copyObbStructFromExisting(const float* vert,int vertSize,const int* ind,int indSize,float triSize,int triCnt);
    static void reduceTriangleSizes(std::vector<float>& vert,std::vector<int>& ind,float triSize);

    CObbNode* obb;
    std::vector<float> vertices;
    std::vector<int> indices;

private:
    int _originalVerticesSize;
    unsigned long _originalVerticesHash;
    int _originalIndicesSize;
    unsigned long _originalIndicesHash;
    float _triSize;
    int _triCnt;

    static std::vector<CObbStruct*> _obbStructs;
};
