#pragma once

#include <conf.h>
#include <obbNode.h>

class CObbStruct
{
public:
    CObbStruct();
    CObbStruct(const double* ver,int verSize,const int* ind,int indSize,double triSize,int triCnt);
    virtual ~CObbStruct();

    CObbStruct* copyYourself() const;
    void scaleYourself(double f);
    bool isSame(const double* v,int vSize,const int* ind,int indSize,double triSize,int triCnt);
    unsigned char* serialize(int& dataSize) const;
    bool deserialize(const unsigned char* data);

    static void addObbStruct(CObbStruct* obbStruct);
    static void removeObbStruct(CObbStruct* obbStruct);
    static CObbStruct* copyObbStructFromExisting(const double* vert,int vertSize,const int* ind,int indSize,double triSize,int triCnt);
    static void reduceTriangleSizes(std::vector<double>& vert,std::vector<int>& ind,double triSize);

    CObbNode* obb;
    std::vector<double> vertices;
    std::vector<int> indices;

private:
    int _originalVerticesSize;
    unsigned long _originalVerticesHash;
    int _originalIndicesSize;
    unsigned long _originalIndicesHash;
    double _triSize;
    int _triCnt;

    static std::vector<CObbStruct*> _obbStructs;
};
