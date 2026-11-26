# Coppelia Geometric Routines, API reference

All units, unless otherwise indicated, are specified in meters and radians.

## Alphabetical list

- [geom_copyMesh](#geom_copyMesh)
- [geom_copyOctree](#geom_copyOctree)
- [geom_copyPtcloud](#geom_copyPtcloud)
- [geom_createMesh](#geom_createMesh)
- [geom_createOctreeFromColorPoints](#geom_createOctreeFromColorPoints)
- [geom_createOctreeFromMesh](#geom_createOctreeFromMesh)
- [geom_createOctreeFromOctree](#geom_createOctreeFromOctree)
- [geom_createOctreeFromPoints](#geom_createOctreeFromPoints)
- [geom_createPtcloudFromColorPoints](#geom_createPtcloudFromColorPoints)
- [geom_createPtcloudFromPoints](#geom_createPtcloudFromPoints)
- [geom_destroyMesh](#geom_destroyMesh)
- [geom_destroyOctree](#geom_destroyOctree)
- [geom_destroyPtcloud](#geom_destroyPtcloud)
- [geom_getApproxBoxBoxDistance](#geom_getApproxBoxBoxDistance)
- [geom_getBoxBoxCollision](#geom_getBoxBoxCollision)
- [geom_getBoxBoxDistanceIfSmaller](#geom_getBoxBoxDistanceIfSmaller)
- [geom_getBoxPointCollision](#geom_getBoxPointCollision)
- [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller)
- [geom_getBoxSegmentCollision](#geom_getBoxSegmentCollision)
- [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller)
- [geom_getBoxTriangleCollision](#geom_getBoxTriangleCollision)
- [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller)
- [geom_getMeshFromSerializationData](#geom_getMeshFromSerializationData)
- [geom_getMeshMeshCollision](#geom_getMeshMeshCollision)
- [geom_getMeshMeshDistanceIfSmaller](#geom_getMeshMeshDistanceIfSmaller)
- [geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision)
- [geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller)
- [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller)
- [geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller)
- [geom_getMeshSegmentCollision](#geom_getMeshSegmentCollision)
- [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller)
- [geom_getMeshSerializationData](#geom_getMeshSerializationData)
- [geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision)
- [geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller)
- [geom_getOctreeFromSerializationData](#geom_getOctreeFromSerializationData)
- [geom_getOctreeOctreeCollision](#geom_getOctreeOctreeCollision)
- [geom_getOctreeOctreeDistanceIfSmaller](#geom_getOctreeOctreeDistanceIfSmaller)
- [geom_getOctreePointCollision](#geom_getOctreePointCollision)
- [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller)
- [geom_getOctreePtcloudCollision](#geom_getOctreePtcloudCollision)
- [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller)
- [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision)
- [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller)
- [geom_getOctreeSerializationData](#geom_getOctreeSerializationData)
- [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision)
- [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller)
- [geom_getOctreeVoxelData](#geom_getOctreeVoxelData)
- [geom_getPtcloudFromSerializationData](#geom_getPtcloudFromSerializationData)
- [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller)
- [geom_getPtcloudPoints](#geom_getPtcloudPoints)
- [geom_getPtcloudPtcloudDistanceIfSmaller](#geom_getPtcloudPtcloudDistanceIfSmaller)
- [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller)
- [geom_getPtcloudSerializationData](#geom_getPtcloudSerializationData)
- [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller)
- [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)
- [geom_getSegmentSegmentDistanceIfSmaller](#geom_getSegmentSegmentDistanceIfSmaller)
- [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller)
- [geom_getTriangleSegmentCollision](#geom_getTriangleSegmentCollision)
- [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller)
- [geom_getTriangleTriangleCollision](#geom_getTriangleTriangleCollision)
- [geom_getTriangleTriangleDistanceIfSmaller](#geom_getTriangleTriangleDistanceIfSmaller)
- [geom_insertColorPointsIntoOctree](#geom_insertColorPointsIntoOctree)
- [geom_insertColorPointsIntoPtcloud](#geom_insertColorPointsIntoPtcloud)
- [geom_insertMeshIntoOctree](#geom_insertMeshIntoOctree)
- [geom_insertOctreeIntoOctree](#geom_insertOctreeIntoOctree)
- [geom_insertPointsIntoOctree](#geom_insertPointsIntoOctree)
- [geom_insertPointsIntoPtcloud](#geom_insertPointsIntoPtcloud)
- [geom_intersectPointsWithPtcloud](#geom_intersectPointsWithPtcloud)
- [geom_isPointInVolume](#geom_isPointInVolume)
- [geom_raySensorDetectMeshIfSmaller](#geom_raySensorDetectMeshIfSmaller)
- [geom_raySensorDetectOctreeIfSmaller](#geom_raySensorDetectOctreeIfSmaller)
- [geom_removeMeshFromOctree](#geom_removeMeshFromOctree)
- [geom_removeOctreeFromOctree](#geom_removeOctreeFromOctree)
- [geom_removeOctreeFromPtcloud](#geom_removeOctreeFromPtcloud)
- [geom_removePointsFromOctree](#geom_removePointsFromOctree)
- [geom_removePointsFromPtcloud](#geom_removePointsFromPtcloud)
- [geom_scaleMesh](#geom_scaleMesh)
- [geom_scaleOctree](#geom_scaleOctree)
- [geom_scalePtcloud](#geom_scalePtcloud)
- [geom_volumeSensorDetectMeshIfSmaller](#geom_volumeSensorDetectMeshIfSmaller)
- [geom_volumeSensorDetectOctreeIfSmaller](#geom_volumeSensorDetectOctreeIfSmaller)
- [geom_volumeSensorDetectPtcloudIfSmaller](#geom_volumeSensorDetectPtcloudIfSmaller)
- [geom_volumeSensorDetectSegmentIfSmaller](#geom_volumeSensorDetectSegmentIfSmaller)
- [geom_volumeSensorDetectTriangleIfSmaller](#geom_volumeSensorDetectTriangleIfSmaller)

## Mesh creation/destruction/manipulation/info

- [geom_createMesh](#geom_createMesh)
- [geom_destroyMesh](#geom_destroyMesh)
- [geom_copyMesh](#geom_copyMesh)
- [geom_scaleMesh](#geom_scaleMesh)
- [geom_getMeshSerializationData](#geom_getMeshSerializationData)
- [geom_getMeshFromSerializationData](#geom_getMeshFromSerializationData)
- [geom_createOctreeFromMesh](#geom_createOctreeFromMesh)
- [geom_insertMeshIntoOctree](#geom_insertMeshIntoOctree)
- [geom_removeMeshFromOctree](#geom_removeMeshFromOctree)

## OC-tree creation/destruction/manipulation/info

- [geom_createOctreeFromPoints](#geom_createOctreeFromPoints)
- [geom_createOctreeFromColorPoints](#geom_createOctreeFromColorPoints)
- [geom_createOctreeFromMesh](#geom_createOctreeFromMesh)
- [geom_createOctreeFromOctree](#geom_createOctreeFromOctree)
- [geom_destroyOctree](#geom_destroyOctree)
- [geom_copyOctree](#geom_copyOctree)
- [geom_scaleOctree](#geom_scaleOctree)
- [geom_getOctreeSerializationData](#geom_getOctreeSerializationData)
- [geom_getOctreeFromSerializationData](#geom_getOctreeFromSerializationData)
- [geom_getOctreeVoxelData](#geom_getOctreeVoxelData)
- [geom_insertPointsIntoOctree](#geom_insertPointsIntoOctree)
- [geom_insertColorPointsIntoOctree](#geom_insertColorPointsIntoOctree)
- [geom_insertMeshIntoOctree](#geom_insertMeshIntoOctree)
- [geom_insertOctreeIntoOctree](#geom_insertOctreeIntoOctree)
- [geom_removePointsFromOctree](#geom_removePointsFromOctree)
- [geom_removeMeshFromOctree](#geom_removeMeshFromOctree)
- [geom_removeOctreeFromOctree](#geom_removeOctreeFromOctree)

## Point-cloud creation/destruction/manipulation/info

- [geom_createPtcloudFromPoints](#geom_createPtcloudFromPoints)
- [geom_createPtcloudFromColorPoints](#geom_createPtcloudFromColorPoints)
- [geom_destroyPtcloud](#geom_destroyPtcloud)
- [geom_copyPtcloud](#geom_copyPtcloud)
- [geom_scalePtcloud](#geom_scalePtcloud)
- [geom_getPtcloudSerializationData](#geom_getPtcloudSerializationData)
- [geom_getPtcloudFromSerializationData](#geom_getPtcloudFromSerializationData)
- [geom_getPtcloudPoints](#geom_getPtcloudPoints)
- [geom_insertPointsIntoPtcloud](#geom_insertPointsIntoPtcloud)
- [geom_insertColorPointsIntoPtcloud](#geom_insertColorPointsIntoPtcloud)
- [geom_removePointsFromPtcloud](#geom_removePointsFromPtcloud)
- [geom_removeOctreeFromPtcloud](#geom_removeOctreeFromPtcloud)
- [geom_intersectPointsWithPtcloud](#geom_intersectPointsWithPtcloud)

## Mesh collision detection

- [geom_getMeshMeshCollision](#geom_getMeshMeshCollision)
- [geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision)
- [geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision)
- [geom_getMeshSegmentCollision](#geom_getMeshSegmentCollision)

## OC-tree collision detection

- [geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision)
- [geom_getOctreeOctreeCollision](#geom_getOctreeOctreeCollision)
- [geom_getOctreePtcloudCollision](#geom_getOctreePtcloudCollision)
- [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision)
- [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision)
- [geom_getOctreePointCollision](#geom_getOctreePointCollision)

## Point-cloud collision detection

- [geom_getOctreePtcloudCollision](#geom_getOctreePtcloudCollision)

## Primitives collision detection

- [geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision)
- [geom_getMeshSegmentCollision](#geom_getMeshSegmentCollision)
- [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision)
- [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision)
- [geom_getOctreePointCollision](#geom_getOctreePointCollision)
- [geom_getBoxBoxCollision](#geom_getBoxBoxCollision)
- [geom_getBoxTriangleCollision](#geom_getBoxTriangleCollision)
- [geom_getBoxSegmentCollision](#geom_getBoxSegmentCollision)
- [geom_getBoxPointCollision](#geom_getBoxPointCollision)
- [geom_getTriangleTriangleCollision](#geom_getTriangleTriangleCollision)
- [geom_getTriangleSegmentCollision](#geom_getTriangleSegmentCollision)

## Mesh minimum distance calculation

- [geom_getMeshMeshDistanceIfSmaller](#geom_getMeshMeshDistanceIfSmaller)
- [geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller)
- [geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller)
- [geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller)
- [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller)
- [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller)

## OC-tree minimum distance calculation

- [geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller)
- [geom_getOctreeOctreeDistanceIfSmaller](#geom_getOctreeOctreeDistanceIfSmaller)
- [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller)
- [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller)
- [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller)
- [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller)

## Point-cloud minimum distance calculation

- [geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller)
- [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller)
- [geom_getPtcloudPtcloudDistanceIfSmaller](#geom_getPtcloudPtcloudDistanceIfSmaller)
- [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller)
- [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller)
- [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller)

## Primitives minimum distance calculation

- [geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller)
- [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller)
- [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller)
- [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller)
- [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller)
- [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller)
- [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller)
- [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller)
- [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller)
- [geom_getBoxBoxDistanceIfSmaller](#geom_getBoxBoxDistanceIfSmaller)
- [geom_getApproxBoxBoxDistance](#geom_getApproxBoxBoxDistance)
- [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller)
- [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller)
- [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller)
- [geom_getTriangleTriangleDistanceIfSmaller](#geom_getTriangleTriangleDistanceIfSmaller)
- [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller)
- [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller)
- [geom_getSegmentSegmentDistanceIfSmaller](#geom_getSegmentSegmentDistanceIfSmaller)
- [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

## Proximity sensor simulation

- [geom_volumeSensorDetectMeshIfSmaller](#geom_volumeSensorDetectMeshIfSmaller)
- [geom_volumeSensorDetectOctreeIfSmaller](#geom_volumeSensorDetectOctreeIfSmaller)
- [geom_volumeSensorDetectPtcloudIfSmaller](#geom_volumeSensorDetectPtcloudIfSmaller)
- [geom_volumeSensorDetectTriangleIfSmaller](#geom_volumeSensorDetectTriangleIfSmaller)
- [geom_volumeSensorDetectSegmentIfSmaller](#geom_volumeSensorDetectSegmentIfSmaller)
- [geom_isPointInVolume](#geom_isPointInVolume)
- [geom_raySensorDetectMeshIfSmaller](#geom_raySensorDetectMeshIfSmaller)
- [geom_raySensorDetectOctreeIfSmaller](#geom_raySensorDetectOctreeIfSmaller)

## Other API functions

- [geom_isPointInVolume](#geom_isPointInVolume)

## Commands

### <a name="geom_copyMesh" id="geom_copyMesh"></a> geom_copyMesh

Duplicates a mesh object.

#### Synopsis:

```cpp
CObbStruct* geom_copyMesh(const CObbStruct* meshObbStruct)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): the mesh object to duplicate.

#### Return values:

A pointer to the duplicated mesh object in case of success, otherwise nullptr.

#### See also:

[geom_createMesh](#geom_createMesh)

### <a name="geom_copyOctree" id="geom_copyOctree"></a> geom_copyOctree

Duplicates an OC-tree object.

#### Synopsis:

```cpp
COcStruct* geom_copyOctree(const COcStruct* ocStruct)
```

#### Parameters:

- **ocStruct** (const COcStruct*): the OC-tree object to duplicate.

#### Return values:

A pointer to the duplicated OC-tree object in case of success, otherwise nullptr.

### <a name="geom_copyPtcloud" id="geom_copyPtcloud"></a> geom_copyPtcloud

Duplicates a point-cloud object.

#### Synopsis:

```cpp
CPcStruct* geom_copyPtcloud(const CPcStruct* pcStruct)
```

#### Parameters:

- **pcStruct** (const CPcStruct*): the point-cloud object to duplicate.

#### Return values:

A pointer to the duplicated point-cloud object in case of success, otherwise nullptr.

### <a name="geom_createMesh" id="geom_createMesh"></a> geom_createMesh

Creates a mesh object.

#### Synopsis:

```cpp
CObbStruct* geom_createMesh(const double* vertices,int verticesSize,const int* indices,int indicesSize,const CPose* meshOrigin=nullptr,double triangleEdgeMaxLength=0.3,int maxTrianglesInBoundingBox=8)
```

#### Parameters:

- **vertices** (const double*): an array with the vertices of the mesh, expressed relative to the world origin.
- **verticesSize** (int): the size of the vertex array, i.e. 3*vertexCount.
- **indices** (const int*): an array with the indices of the mesh.
- **indicesSize** (int): the size of the index array, i.e. each triangle takes 3 indices.
- **meshOrigin** (const CPose*, default: nullptr): the transformation frame of the mesh. Can be nullptr, in which case the frame of the mesh is located at the world origin.
- **triangleEdgeMaxLength** (double, default: 0.3): the maximum triangle size used internally for efficient calculations. A smaller size usually results in faster calculations, but larger calculation structures.
- **maxTrianglesInBoundingBox** (int, default: 8): the maximum number of triangles in leaf bounding boxes, used internally for efficient calculations. A smaller value usually results in faster calculations, but larger calculation structures.

#### Return values:

A pointer to the mesh object in case of success, nullptr otherwise

#### See also:

[geom_destroyMesh](#geom_destroyMesh), [geom_copyMesh](#geom_copyMesh), [geom_getMeshFromSerializationData](#geom_getMeshFromSerializationData)

### <a name="geom_createOctreeFromColorPoints" id="geom_createOctreeFromColorPoints"></a> geom_createOctreeFromColorPoints

Creates an OC-tree object from several points with individual colors and individual user data.

#### Synopsis:

```cpp
COcStruct* geom_createOctreeFromColorPoints(const double* points,int pointCnt,const CPose* octreeOrigin=nullptr,double cellS=0.05,const unsigned char* rgbaData=nullptr,const unsigned int* usrData=nullptr)
```

#### Parameters:

- **points** (const double*): an array with point data, expressed relative to the world origin.
- **pointCnt** (int): the number of provided points.
- **octreeOrigin** (const CPose*, default: nullptr): the transformation frame of the OC-tree. Can be nullptr, in which case the frame of the OC-tree is located at the world origin.
- **cellS** (double, default: 0.05): The desired OC-tree voxel size.
- **rgbaData** (const unsigned char*, default: nullptr): the RGBA colors (0-255) of the various points (i.e. 4 values per point: red, green, blue and alpha).
- **usrData** (const unsigned int*, default: nullptr): the user data of the various points (i.e. one user data per point).

#### Return values:

A pointer to an OC-tree object in case of success, otherwise nullptr.

#### See also:

[geom_createOctreeFromPoints](#geom_createOctreeFromPoints), [geom_createOctreeFromMesh](#geom_createOctreeFromMesh), [geom_createOctreeFromOctree](#geom_createOctreeFromOctree), [geom_destroyOctree](#geom_destroyOctree), [geom_copyOctree](#geom_copyOctree), [geom_getOctreeFromSerializationData](#geom_getOctreeFromSerializationData), [geom_insertColorPointsIntoOctree](#geom_insertColorPointsIntoOctree)

### <a name="geom_createOctreeFromMesh" id="geom_createOctreeFromMesh"></a> geom_createOctreeFromMesh

Creates an OC-tree object from a mesh object.

#### Synopsis:

```cpp
COcStruct* geom_createOctreeFromMesh(const CObbStruct* meshObbStruct,const CPose& meshTransformation,const CPose* octreeOrigin=nullptr,double cellS=0.05,const unsigned char rgbaData[4]=nullptr,unsigned int usrData=0)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh object.
- **octreeOrigin** (const CPose*, default: nullptr): the transformation frame of the OC-tree. Can be nullptr, in which case the frame of the OC-tree is located at the world origin.
- **cellS** (double, default: 0.05): The desired OC-tree voxel size.
- **rgbaData** (const unsigned char, default: nullptr): the RGBA color (0-255) of the generated voxels (4 values: red, green, blue and alpha).
- **usrData** (unsigned int, default: 0): the user data of the generated voxels.

#### Return values:

A pointer to an OC-tree object in case of success, otherwise nullptr.

#### See also:

[geom_createOctreeFromPoints](#geom_createOctreeFromPoints), [geom_createOctreeFromColorPoints](#geom_createOctreeFromColorPoints), [geom_createOctreeFromOctree](#geom_createOctreeFromOctree), [geom_destroyOctree](#geom_destroyOctree), [geom_copyOctree](#geom_copyOctree), [geom_getOctreeFromSerializationData](#geom_getOctreeFromSerializationData)

### <a name="geom_createOctreeFromOctree" id="geom_createOctreeFromOctree"></a> geom_createOctreeFromOctree

Creates an OC-tree object from another OC-tree object.

#### Synopsis:

```cpp
COcStruct* geom_createOctreeFromOctree(const COcStruct* otherOctreeStruct,const CPose& otherOctreeTransformation,const CPose* newOctreeOrigin=nullptr,double newOctreeCellS=0.05,const unsigned char rgbaData[4]=nullptr,unsigned int usrData=0)
```

#### Parameters:

- **otherOctreeStruct** (const COcStruct*): the other OC-tree object.
- **otherOctreeTransformation** (const CPose&): the transformation frame of the other OC-tree object.
- **newOctreeOrigin** (const CPose*, default: nullptr): the transformation frame of the new OC-tree. Can be nullptr, in which case the frame of the new OC-tree is located at the world origin.
- **newOctreeCellS** (double, default: 0.05): The desired OC-tree voxel size.
- **rgbaData** (const unsigned char, default: nullptr): the RGBA color (0-255) of the generated voxels (4 values: red, green, blue and alpha).
- **usrData** (unsigned int, default: 0): the user data of the generated voxels.

#### Return values:

A pointer to an OC-tree object in case of success, otherwise nullptr.

#### See also:

[geom_createOctreeFromPoints](#geom_createOctreeFromPoints), [geom_createOctreeFromColorPoints](#geom_createOctreeFromColorPoints), [geom_createOctreeFromMesh](#geom_createOctreeFromMesh), [geom_destroyOctree](#geom_destroyOctree), [geom_copyOctree](#geom_copyOctree), [geom_getOctreeFromSerializationData](#geom_getOctreeFromSerializationData)

### <a name="geom_createOctreeFromPoints" id="geom_createOctreeFromPoints"></a> geom_createOctreeFromPoints

Creates an OC-tree object from several points.

#### Synopsis:

```cpp
COcStruct* geom_createOctreeFromPoints(const double* points,int pointCnt,const CPose* octreeOrigin=nullptr,double cellS=0.05,const unsigned char rgbaData[4]=nullptr,unsigned int usrData=0)
```

#### Parameters:

- **points** (const double*): an array with point data, expressed relative to the world origin.
- **pointCnt** (int): the number of provided points.
- **octreeOrigin** (const CPose*, default: nullptr): the transformation frame of the OC-tree. Can be nullptr, in which case the frame of the OC-tree is located at the world origin.
- **cellS** (double, default: 0.05): The desired OC-tree voxel size.
- **rgbaData** (const unsigned char, default: nullptr): the RGBA color (0-255) of the generated voxels (4 values: red, green, blue and alpha).
- **usrData** (unsigned int, default: 0): the user data of the generated voxels.

#### Return values:

A pointer to an OC-tree object in case of success, otherwise nullptr.

#### See also:

[geom_createOctreeFromColorPoints](#geom_createOctreeFromColorPoints), [geom_createOctreeFromMesh](#geom_createOctreeFromMesh), [geom_createOctreeFromOctree](#geom_createOctreeFromOctree), [geom_destroyOctree](#geom_destroyOctree), [geom_copyOctree](#geom_copyOctree), [geom_getOctreeFromSerializationData](#geom_getOctreeFromSerializationData), [geom_insertPointsIntoOctree](#geom_insertPointsIntoOctree)

### <a name="geom_createPtcloudFromColorPoints" id="geom_createPtcloudFromColorPoints"></a> geom_createPtcloudFromColorPoints

Creates a point-cloud object from several points with individual colors.

#### Synopsis:

```cpp
CPcStruct* geom_createPtcloudFromColorPoints(const double* points,int pointCnt,const CPose* ptcloudOrigin=nullptr,double cellS=0.05,int maxPointCnt=20,const unsigned char* rgbaData=nullptr,double proximityTol=0.005)
```

#### Parameters:

- **points** (const double*): an array with point data, expressed relative to the world origin.
- **pointCnt** (int): the number of provided points.
- **ptcloudOrigin** (const CPose*, default: nullptr): the transformation frame of the point-cloud. Can be nullptr, in which case the frame of the point-cloud is located at the world origin.
- **cellS** (double, default: 0.05): The desired voxel size of the underlying OC-tree.
- **maxPointCnt** (int, default: 20): The maximum amount of points inside of a OC-tree voxel.
- **rgbaData** (const unsigned char*, default: nullptr): the RGBA color (0-255) of the generated points. Specify one color per point (4 values per point: red, green, blue and alpha).
- **proximityTol** (double, default: 0.005): the distance tolerance to other points (i.e. point-point distances below the specified value are not permitted and points will be omitted).

#### Return values:

A pointer to a point-cloud object in case of success, otherwise nullptr.

#### See also:

[geom_createPtcloudFromPoints](#geom_createPtcloudFromPoints), [geom_destroyPtcloud](#geom_destroyPtcloud), [geom_copyPtcloud](#geom_copyPtcloud), [geom_getPtcloudFromSerializationData](#geom_getPtcloudFromSerializationData), [geom_insertColorPointsIntoPtcloud](#geom_insertColorPointsIntoPtcloud)

### <a name="geom_createPtcloudFromPoints" id="geom_createPtcloudFromPoints"></a> geom_createPtcloudFromPoints

Creates a point-cloud object from several points.

#### Synopsis:

```cpp
CPcStruct* geom_createPtcloudFromPoints(const double* points,int pointCnt,const CPose* ptcloudOrigin=nullptr,double cellS=0.05,int maxPointCnt=20,const unsigned char rgbaData[4]=nullptr,double proximityTol=0.005)
```

#### Parameters:

- **points** (const double*): an array with point data, expressed relative to the world origin.
- **pointCnt** (int): the number of provided points.
- **ptcloudOrigin** (const CPose*, default: nullptr): the transformation frame of the point-cloud. Can be nullptr, in which case the frame of the point-cloud is located at the world origin.
- **cellS** (double, default: 0.05): The desired voxel size of the underlying OC-tree.
- **maxPointCnt** (int, default: 20): The maximum amount of points inside of a OC-tree voxel.
- **rgbaData** (const unsigned char, default: nullptr): the RGBA color (0-255) of the generated points (4 values: red, green, blue and alpha).
- **proximityTol** (double, default: 0.005): the distance tolerance to other points (i.e. point-point distances below the specified value are not permitted and points will be omitted).

#### Return values:

A pointer to a point-cloud object in case of success, otherwise nullptr.

#### See also:

[geom_createPtcloudFromColorPoints](#geom_createPtcloudFromColorPoints), [geom_destroyPtcloud](#geom_destroyPtcloud), [geom_copyPtcloud](#geom_copyPtcloud), [geom_getPtcloudFromSerializationData](#geom_getPtcloudFromSerializationData), [geom_insertPointsIntoPtcloud](#geom_insertPointsIntoPtcloud)

### <a name="geom_destroyMesh" id="geom_destroyMesh"></a> geom_destroyMesh

Destroys a mesh object.

#### Synopsis:

```cpp
void geom_destroyMesh(CObbStruct* meshObbStruct)
```

#### Parameters:

- **meshObbStruct** (CObbStruct*): a pointer to the mesh object.

#### Return values:

No return values

#### See also:

[geom_createMesh](#geom_createMesh)

### <a name="geom_destroyOctree" id="geom_destroyOctree"></a> geom_destroyOctree

Destroys an OC-tree object.

#### Synopsis:

```cpp
void geom_destroyOctree(COcStruct* ocStruct)
```

#### Parameters:

- **ocStruct** (COcStruct*): a pointer to the OC-tree object.

#### Return values:

No return values

#### See also:

[geom_createOctreeFromPoints](#geom_createOctreeFromPoints)

### <a name="geom_destroyPtcloud" id="geom_destroyPtcloud"></a> geom_destroyPtcloud

Destroys a point-cloud object.

#### Synopsis:

```cpp
void geom_destroyPtcloud(CPcStruct* pcStruct)
```

#### Parameters:

- **pcStruct** (CPcStruct*): a pointer to the point-cloud object.

#### Return values:

No return values

#### See also:

[geom_createPtcloudFromPoints](#geom_createPtcloudFromPoints)

### <a name="geom_getApproxBoxBoxDistance" id="geom_getApproxBoxBoxDistance"></a> geom_getApproxBoxBoxDistance

Computes a fast, approximate minimum distance between two cuboids. The approximate distance is always same or smaller than the exact distance.

#### Synopsis:

```cpp
double geom_getApproxBoxBoxDistance(const CPose& box1Transformation,const C3Vector& box1HalfSize,const CPose& box2Transformation,const C3Vector& box2HalfSize)
```

#### Parameters:

- **box1Transformation** (const CPose&): the transformation frame of the first cuboid.
- **box1HalfSize** (const C3Vector&): the half-size of the first cuboid.
- **box2Transformation** (const CPose&): the transformation frame of the second cuboid.
- **box2HalfSize** (const C3Vector&): the half-size of the second cuboid.

#### Return values:

the approximate minimum distance between the two cuboids. The approximate distance is always same or smaller than the exact distance.

#### See also:

[geom_getBoxBoxDistanceIfSmaller](#geom_getBoxBoxDistanceIfSmaller)

### <a name="geom_getBoxBoxCollision" id="geom_getBoxBoxCollision"></a> geom_getBoxBoxCollision

Checks for collision between two cuboids.

#### Synopsis:

```cpp
bool geom_getBoxBoxCollision(const CPose& box1Transformation,const C3Vector& box1HalfSize,const CPose& box2Transformation,const C3Vector& box2HalfSize,bool boxesAreSolid)
```

#### Parameters:

- **box1Transformation** (const CPose&): the transformation frame of the first cuboid.
- **box1HalfSize** (const C3Vector&): the half-size of the first cuboid.
- **box2Transformation** (const CPose&): the transformation frame of the second cuboid.
- **box2HalfSize** (const C3Vector&): the half-size of the second cuboid.
- **boxesAreSolid** (bool): whether the two cuboids are solid. If not, then cuboid A being fully encompassed by cuboid B will not lead to a collision detection.

#### Return values:

the collision status

#### See also:

[geom_getBoxTriangleCollision](#geom_getBoxTriangleCollision), [geom_getBoxSegmentCollision](#geom_getBoxSegmentCollision), [geom_getBoxPointCollision](#geom_getBoxPointCollision)

### <a name="geom_getBoxBoxDistanceIfSmaller" id="geom_getBoxBoxDistanceIfSmaller"></a> geom_getBoxBoxDistanceIfSmaller

Computes the minimum distance between two cuboids.

#### Synopsis:

```cpp
bool geom_getBoxBoxDistanceIfSmaller(const CPose& box1Transformation,const C3Vector& box1HalfSize,const CPose& box2Transformation,const C3Vector& box2HalfSize,bool boxesAreSolid,double& dist,C3Vector* distSegPt1=nullptr,C3Vector* distSegPt2=nullptr,bool altRoutine=false)
```

#### Parameters:

- **box1Transformation** (const CPose&): the transformation frame of the first cuboid.
- **box1HalfSize** (const C3Vector&): the half-size of the first cuboid.
- **box2Transformation** (const CPose&): the transformation frame of the second cuboid.
- **box2HalfSize** (const C3Vector&): the half-size of the second cuboid.
- **boxesAreSolid** (bool): whether the two cuboids are solid. If true, then cuboid A being fully encompassed by cuboid B will always lead to a distance of zero between the two cuboids.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **distSegPt1** (C3Vector*, default: nullptr): the minimum distance point on the first cuboid.
- **distSegPt2** (C3Vector*, default: nullptr): the minimum distance point on the second cuboid.
- **altRoutine** (bool, default: false): if true, an alternative calculation routine will be used.

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getApproxBoxBoxDistance](#geom_getApproxBoxBoxDistance), [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller), [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller), [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller)

### <a name="geom_getBoxPointCollision" id="geom_getBoxPointCollision"></a> geom_getBoxPointCollision

Checks for collision between a cuboid and a point.

#### Synopsis:

```cpp
bool geom_getBoxPointCollision(const CPose& boxTransformation,const C3Vector& boxHalfSize,const C3Vector& point)
```

#### Parameters:

- **boxTransformation** (const CPose&): the transformation frame of the cuboid.
- **boxHalfSize** (const C3Vector&): the half-size of the cuboid.
- **point** (const C3Vector&): point position.

#### Return values:

the collision status

#### See also:

[geom_getOctreePointCollision](#geom_getOctreePointCollision), [geom_getBoxBoxCollision](#geom_getBoxBoxCollision), [geom_getBoxTriangleCollision](#geom_getBoxTriangleCollision), [geom_getBoxSegmentCollision](#geom_getBoxSegmentCollision)

### <a name="geom_getBoxPointDistanceIfSmaller" id="geom_getBoxPointDistanceIfSmaller"></a> geom_getBoxPointDistanceIfSmaller

Computes the minimum distance between a cuboid and a point.

#### Synopsis:

```cpp
bool geom_getBoxPointDistanceIfSmaller(const CPose& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& point,double& dist,C3Vector* boxDistSegPt=nullptr)
```

#### Parameters:

- **boxTransformation** (const CPose&): the transformation frame of the cuboid.
- **boxHalfSize** (const C3Vector&): the half-size of the cuboid.
- **boxIsSolid** (bool): whether the cuboid is solid. If true, then the point being fully encompassed by the cuboid will always lead to a distance of zero.
- **point** (const C3Vector&): position of the point.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **boxDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the cuboid.

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller), [geom_getBoxBoxDistanceIfSmaller](#geom_getBoxBoxDistanceIfSmaller), [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller), [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getBoxSegmentCollision" id="geom_getBoxSegmentCollision"></a> geom_getBoxSegmentCollision

Checks for collision between a cuboid and a segment.

#### Synopsis:

```cpp
bool geom_getBoxSegmentCollision(const CPose& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& segmentEndPoint,const C3Vector& segmentVector)
```

#### Parameters:

- **boxTransformation** (const CPose&): the transformation frame of the cuboid.
- **boxHalfSize** (const C3Vector&): the half-size of the cuboid.
- **boxIsSolid** (bool): whether the cuboid is solid. If not, then the segment being fully encompassed by the cuboid will not lead to a collision detection.
- **segmentEndPoint** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.

#### Return values:

the collision status

#### See also:

[geom_getMeshSegmentCollision](#geom_getMeshSegmentCollision), [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision), [geom_getBoxBoxCollision](#geom_getBoxBoxCollision), [geom_getBoxTriangleCollision](#geom_getBoxTriangleCollision), [geom_getBoxPointCollision](#geom_getBoxPointCollision), [geom_getTriangleSegmentCollision](#geom_getTriangleSegmentCollision)

### <a name="geom_getBoxSegmentDistanceIfSmaller" id="geom_getBoxSegmentDistanceIfSmaller"></a> geom_getBoxSegmentDistanceIfSmaller

Computes the minimum distance between a cuboid and a segment.

#### Synopsis:

```cpp
bool geom_getBoxSegmentDistanceIfSmaller(const CPose& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* distSegPt1=nullptr,C3Vector* distSegPt2=nullptr,bool altRoutine=false)
```

#### Parameters:

- **boxTransformation** (const CPose&): the transformation frame of the cuboid.
- **boxHalfSize** (const C3Vector&): the half-size of the cuboid.
- **boxIsSolid** (bool): whether the cuboid is solid. If true, then the segment being fully encompassed by the cuboid will always lead to a distance of zero.
- **segmentEndPoint** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **boxDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the cuboid.
- **segmentSegPt** (C3Vector*, default: nullptr): the minimum distance point on the segment.
- **altRoutine** (bool, default: false): if true, an alternative calculation routine will be used.

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getBoxBoxDistanceIfSmaller](#geom_getBoxBoxDistanceIfSmaller), [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller), [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getSegmentSegmentDistanceIfSmaller](#geom_getSegmentSegmentDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getBoxTriangleCollision" id="geom_getBoxTriangleCollision"></a> geom_getBoxTriangleCollision

Checks for collision between a cuboid and a triangle.

#### Synopsis:

```cpp
bool geom_getBoxTriangleCollision(const CPose& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& p,const C3Vector& v,const C3Vector& w)
```

#### Parameters:

- **boxTransformation** (const CPose&): the transformation frame of the cuboid.
- **boxHalfSize** (const C3Vector&): the half-size of the cuboid.
- **boxIsSolid** (bool): whether the cuboid is solid. If not, then the triangle being fully encompassed by the cuboid will not lead to a collision detection.
- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.

#### Return values:

the collision status

#### See also:

[geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision), [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision), [geom_getBoxBoxCollision](#geom_getBoxBoxCollision), [geom_getBoxSegmentCollision](#geom_getBoxSegmentCollision), [geom_getBoxPointCollision](#geom_getBoxPointCollision), [geom_getTriangleTriangleCollision](#geom_getTriangleTriangleCollision), [geom_getTriangleSegmentCollision](#geom_getTriangleSegmentCollision)

### <a name="geom_getBoxTriangleDistanceIfSmaller" id="geom_getBoxTriangleDistanceIfSmaller"></a> geom_getBoxTriangleDistanceIfSmaller

Computes the minimum distance between a cuboid and a triangle.

#### Synopsis:

```cpp
bool geom_getBoxTriangleDistanceIfSmaller(const CPose& boxTransformation,const C3Vector& boxHalfSize,bool boxIsSolid,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* boxDistSegPt=nullptr,C3Vector* triangleDistSegPt=nullptr,bool altRoutine=false)
```

#### Parameters:

- **boxTransformation** (const CPose&): the transformation frame of the cuboid.
- **boxHalfSize** (const C3Vector&): the half-size of the cuboid.
- **boxIsSolid** (bool): whether the cuboid is solid. If true, then the triangle being fully encompassed by the cuboid will always lead to a distance of zero.
- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **boxDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the cuboid.
- **triangleDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the triangle.
- **altRoutine** (bool, default: false): if true, an alternative calculation routine will be used.

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getBoxBoxDistanceIfSmaller](#geom_getBoxBoxDistanceIfSmaller), [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller), [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller), [geom_getTriangleTriangleDistanceIfSmaller](#geom_getTriangleTriangleDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller)

### <a name="geom_getMeshFromSerializationData" id="geom_getMeshFromSerializationData"></a> geom_getMeshFromSerializationData

Creates a mesh object based on serialization data.

#### Synopsis:

```cpp
CObbStruct* geom_getMeshFromSerializationData(const unsigned char* serializationData)
```

#### Parameters:

- **serializationData** (const unsigned char*): the serialization data previously generated with [geom_getMeshSerializationData](#geom_getMeshSerializationData).

#### Return values:

A pointer to the created mesh object in case of success, nullptr otherwise.

#### See also:

[geom_getMeshSerializationData](#geom_getMeshSerializationData)

### <a name="geom_getMeshMeshCollision" id="geom_getMeshMeshCollision"></a> geom_getMeshMeshCollision

Checks for collision between two mesh objects.

#### Synopsis:

```cpp
bool geom_getMeshMeshCollision(const CObbStruct* mesh1ObbStruct,const CPose& mesh1Transformation,const CObbStruct* mesh2ObbStruct,const CPose& mesh2Transformation,std::vector* intersections=nullptr,int* mesh1Caching=nullptr,int* mesh2Caching=nullptr)
```

#### Parameters:

- **mesh1ObbStruct** (const CObbStruct*): a pointer to the first mesh object.
- **mesh1Transformation** (const CPose&): the transformation frame of the first mesh.
- **mesh2ObbStruct** (const CObbStruct*): a pointer to the second mesh object.
- **mesh2Transformation** (const CPose&): the transformation frame of the second mesh.
- **intersections** (std::vector*, default: nullptr): if not nullptr, then all collisions will be extracted as interference segments (set to nullptr for fast operation).
- **mesh1Caching** (int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding triangle in mesh 1).
- **mesh2Caching** (int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding triangle in mesh 2).

#### Return values:

the collision status

#### See also:

[geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision), [geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision), [geom_getMeshSegmentCollision](#geom_getMeshSegmentCollision)

### <a name="geom_getMeshMeshDistanceIfSmaller" id="geom_getMeshMeshDistanceIfSmaller"></a> geom_getMeshMeshDistanceIfSmaller

Computes the minimum distance between two mesh objects.

#### Synopsis:

```cpp
bool geom_getMeshMeshDistanceIfSmaller(const CObbStruct* mesh1ObbStruct,const CPose& mesh1Transformation,const CObbStruct* mesh2ObbStruct,const CPose& mesh2Transformation,double& dist,C3Vector* minDistSegPt1=nullptr,C3Vector* minDistSegPt2=nullptr,int* mesh1Caching=nullptr,int* mesh2Caching=nullptr)
```

#### Parameters:

- **mesh1ObbStruct** (const CObbStruct*): a pointer to the first mesh object.
- **mesh1Transformation** (const CPose&): the transformation frame of the first mesh.
- **mesh2ObbStruct** (const CObbStruct*): a pointer to the second mesh object.
- **mesh2Transformation** (const CPose&): the transformation frame of the second mesh.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **minDistSegPt1** (C3Vector*, default: nullptr): the minimum distance point on the first mesh.
- **minDistSegPt2** (C3Vector*, default: nullptr): the minimum distance point on the second mesh.
- **mesh1Caching** (int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the triangle that holds the minimum distance point on the first mesh).
- **mesh2Caching** (int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the triangle that holds the minimum distance point on the second mesh).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller), [geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller)

### <a name="geom_getMeshOctreeCollision" id="geom_getMeshOctreeCollision"></a> geom_getMeshOctreeCollision

Checks for collision between a mesh object and an OC-tree object.

#### Synopsis:

```cpp
bool geom_getMeshOctreeCollision(const CObbStruct* meshObbStruct,const CPose& meshTransformation,const COcStruct* ocStruct,const CPose& octreeTransformation,int* meshCaching=nullptr,unsigned long long int* ocCaching=nullptr)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **meshCaching** (int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding triangle in the mesh).
- **ocCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding voxel in the OC-tree).

#### Return values:

the collision status

#### See also:

[geom_getMeshMeshCollision](#geom_getMeshMeshCollision), [geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision), [geom_getMeshSegmentCollision](#geom_getMeshSegmentCollision), [geom_getOctreeOctreeCollision](#geom_getOctreeOctreeCollision), [geom_getOctreePtcloudCollision](#geom_getOctreePtcloudCollision), [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision), [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision), [geom_getOctreePointCollision](#geom_getOctreePointCollision)

### <a name="geom_getMeshOctreeDistanceIfSmaller" id="geom_getMeshOctreeDistanceIfSmaller"></a> geom_getMeshOctreeDistanceIfSmaller

Computes the minimum distance between a mesh object and an OC-tree object.

#### Synopsis:

```cpp
bool geom_getMeshOctreeDistanceIfSmaller(const CObbStruct* meshObbStruct,const CPose& meshTransformation,const COcStruct* ocStruct,const CPose& octreeTransformation,double& dist,C3Vector* meshMinDistPt=nullptr,C3Vector* ocMinDistPt=nullptr,int* meshCaching=nullptr,unsigned long long int* ocCaching=nullptr)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **meshMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the mesh.
- **ocMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the OC-tree.
- **meshCaching** (int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the triangle that holds the minimum distance point on the mesh).
- **ocCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the voxel that holds the minimum distance point on the OC-tree).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshMeshDistanceIfSmaller](#geom_getMeshMeshDistanceIfSmaller), [geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller), [geom_getOctreeOctreeDistanceIfSmaller](#geom_getOctreeOctreeDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller)

### <a name="geom_getMeshPointDistanceIfSmaller" id="geom_getMeshPointDistanceIfSmaller"></a> geom_getMeshPointDistanceIfSmaller

Computes the minimum distance between a mesh object and a point.

#### Synopsis:

```cpp
bool geom_getMeshPointDistanceIfSmaller(const CObbStruct* meshObbStruct,const CPose& meshTransformation,const C3Vector& point,double& dist,C3Vector* meshMinDistPt=nullptr,int* caching=nullptr)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **point** (const C3Vector&): the position of the point.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **meshMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the mesh.
- **caching** (int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the triangle that holds the minimum distance point on the mesh).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshMeshDistanceIfSmaller](#geom_getMeshMeshDistanceIfSmaller), [geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller), [geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller), [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getMeshPtcloudDistanceIfSmaller" id="geom_getMeshPtcloudDistanceIfSmaller"></a> geom_getMeshPtcloudDistanceIfSmaller

Computes the minimum distance between a mesh object and a point-cloud object.

#### Synopsis:

```cpp
bool geom_getMeshPtcloudDistanceIfSmaller(const CObbStruct* meshObbStruct,const CPose& meshTransformation,const CPcStruct* pcStruct,const CPose& pcTransformation,double& dist,C3Vector* meshMinDistPt=nullptr,C3Vector* pcMinDistPt=nullptr,int* meshCaching=nullptr,unsigned long long int* pcCaching=nullptr)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **pcStruct** (const CPcStruct*): a pointer to the point-cloud object.
- **pcTransformation** (const CPose&): the transformation frame of the point-cloud.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **meshMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the mesh.
- **pcMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the point-cloud.
- **meshCaching** (int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the triangle that holds the minimum distance point on the mesh).
- **pcCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the point that holds the minimum distance point on the point-cloud).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshMeshDistanceIfSmaller](#geom_getMeshMeshDistanceIfSmaller), [geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller), [geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getPtcloudPtcloudDistanceIfSmaller](#geom_getPtcloudPtcloudDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller)

### <a name="geom_getMeshSegmentCollision" id="geom_getMeshSegmentCollision"></a> geom_getMeshSegmentCollision

Checks for collision between a mesh object and a segment.

#### Synopsis:

```cpp
bool geom_getMeshSegmentCollision(const CObbStruct* meshObbStruct,const CPose& meshTransformation,const C3Vector& segmentExtremity,const C3Vector& segmentVector,std::vector* intersections=nullptr,int* caching=nullptr)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **segmentExtremity** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **intersections** (std::vector*, default: nullptr): if not nullptr, then all collisions will be extracted as interference segments (where both segment points are always coincident) (set to nullptr for fast operation).
- **caching** (int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding triangle in the mesh).

#### Return values:

the collision status

#### See also:

[geom_getMeshMeshCollision](#geom_getMeshMeshCollision), [geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision), [geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision), [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision), [geom_getBoxSegmentCollision](#geom_getBoxSegmentCollision), [geom_getTriangleSegmentCollision](#geom_getTriangleSegmentCollision)

### <a name="geom_getMeshSegmentDistanceIfSmaller" id="geom_getMeshSegmentDistanceIfSmaller"></a> geom_getMeshSegmentDistanceIfSmaller

Computes the minimum distance between a mesh object and a segment.

#### Synopsis:

```cpp
bool geom_getMeshSegmentDistanceIfSmaller(const CObbStruct* meshObbStruct,const CPose& meshTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* meshMinDistPt=nullptr,C3Vector* segmentDistSegPt=nullptr,int* caching=nullptr)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **segmentEndPoint** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **meshMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the mesh.
- **segmentDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the segment.
- **caching** (int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the triangle that holds the minimum distance point on the mesh).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshMeshDistanceIfSmaller](#geom_getMeshMeshDistanceIfSmaller), [geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller), [geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getSegmentSegmentDistanceIfSmaller](#geom_getSegmentSegmentDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getMeshSerializationData" id="geom_getMeshSerializationData"></a> geom_getMeshSerializationData

Retrieves the serialization data from a mesh object.

#### Synopsis:

```cpp
void geom_getMeshSerializationData(const CObbStruct* meshObbStruct,std::vector& serializationData)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **serializationData** (std::vector&): the serialization data.

#### Return values:

No return values

#### See also:

[geom_getMeshFromSerializationData](#geom_getMeshFromSerializationData)

### <a name="geom_getMeshTriangleCollision" id="geom_getMeshTriangleCollision"></a> geom_getMeshTriangleCollision

Checks for collision between a mesh object and a triangle.

#### Synopsis:

```cpp
bool geom_getMeshTriangleCollision(const CObbStruct* meshObbStruct,const CPose& meshTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,std::vector* intersections=nullptr,int* caching=nullptr)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **intersections** (std::vector*, default: nullptr): if not nullptr, then all collisions will be extracted as interference segments (set to nullptr for fast operation).
- **caching** (int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding triangle in the mesh).

#### Return values:

the collision status

#### See also:

[geom_getMeshMeshCollision](#geom_getMeshMeshCollision), [geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision), [geom_getMeshSegmentCollision](#geom_getMeshSegmentCollision), [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision), [geom_getBoxTriangleCollision](#geom_getBoxTriangleCollision), [geom_getTriangleTriangleCollision](#geom_getTriangleTriangleCollision), [geom_getTriangleSegmentCollision](#geom_getTriangleSegmentCollision)

### <a name="geom_getMeshTriangleDistanceIfSmaller" id="geom_getMeshTriangleDistanceIfSmaller"></a> geom_getMeshTriangleDistanceIfSmaller

Computes the minimum distance between a mesh object and a triangle.

#### Synopsis:

```cpp
bool geom_getMeshTriangleDistanceIfSmaller(const CObbStruct* meshObbStruct,const CPose& meshTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* meshMinDistPt=nullptr,C3Vector* triangleMinDistPt=nullptr,int* caching=nullptr)
```

#### Parameters:

- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **meshMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the mesh.
- **triangleMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the triangle.
- **caching** (int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the triangle that holds the minimum distance point on the mesh).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshMeshDistanceIfSmaller](#geom_getMeshMeshDistanceIfSmaller), [geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller), [geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller), [geom_getTriangleTriangleDistanceIfSmaller](#geom_getTriangleTriangleDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller)

### <a name="geom_getOctreeFromSerializationData" id="geom_getOctreeFromSerializationData"></a> geom_getOctreeFromSerializationData

Creates an OC-tree object based on serialization data.

#### Synopsis:

```cpp
COcStruct* geom_getOctreeFromSerializationData(const unsigned char* serializationData)
```

#### Parameters:

- **serializationData** (const unsigned char*): the serialization data previously generated with [geom_getOctreeSerializationData](#geom_getOctreeSerializationData).

#### Return values:

A pointer to the created OC-tree object in case of success, nullptr otherwise.

#### See also:

[geom_getOctreeSerializationData](#geom_getOctreeSerializationData)

### <a name="geom_getOctreeOctreeCollision" id="geom_getOctreeOctreeCollision"></a> geom_getOctreeOctreeCollision

Checks for collision between two OC-tree objects.

#### Synopsis:

```cpp
bool geom_getOctreeOctreeCollision(const COcStruct* oc1Struct,const CPose& octree1Transformation,const COcStruct* oc2Struct,const CPose& octree2Transformation,unsigned long long int* oc1Caching=nullptr,unsigned long long int* oc2Caching=nullptr)
```

#### Parameters:

- **oc1Struct** (const COcStruct*): a pointer to the first OC-tree object.
- **octree1Transformation** (const CPose&): the transformation frame of the first OC-tree.
- **oc2Struct** (const COcStruct*): a pointer to the second OC-tree object.
- **octree2Transformation** (const CPose&): the transformation frame of the second OC-tree.
- **oc1Caching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding voxel in OC-tree 1).
- **oc2Caching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding voxel in OC-tree 2).

#### Return values:

the collision status

#### See also:

[geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision), [geom_getOctreePtcloudCollision](#geom_getOctreePtcloudCollision), [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision), [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision), [geom_getOctreePointCollision](#geom_getOctreePointCollision)

### <a name="geom_getOctreeOctreeDistanceIfSmaller" id="geom_getOctreeOctreeDistanceIfSmaller"></a> geom_getOctreeOctreeDistanceIfSmaller

Computes the minimum distance between two OC-tree objects.

#### Synopsis:

```cpp
bool geom_getOctreeOctreeDistanceIfSmaller(const COcStruct* oc1Struct,const CPose& octree1Transformation,const COcStruct* oc2Struct,const CPose& octree2Transformation,double& dist,C3Vector* oc1MinDistPt=nullptr,C3Vector* oc2MinDistPt=nullptr,unsigned long long int* oc1Caching=nullptr,unsigned long long int* oc2Caching=nullptr)
```

#### Parameters:

- **oc1Struct** (const COcStruct*): a pointer to the first OC-tree object.
- **octree1Transformation** (const CPose&): the transformation frame of the first OC-tree.
- **oc2Struct** (const COcStruct*): a pointer to the second OC-tree object.
- **octree2Transformation** (const CPose&): the transformation frame of the second OC-tree.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **oc1MinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the first OC-tree.
- **oc2MinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the second OC-tree.
- **oc1Caching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the voxel that holds the minimum distance point on the first OC-tree).
- **oc2Caching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the voxel that holds the minimum distance point on the second OC-tree).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller)

### <a name="geom_getOctreePointCollision" id="geom_getOctreePointCollision"></a> geom_getOctreePointCollision

Checks for collision between an OC-tree object and a point.

#### Synopsis:

```cpp
bool geom_getOctreePointCollision(const COcStruct* ocStruct,const CPose& octreeTransformation,const C3Vector& point,unsigned int* usrData=nullptr,unsigned long long int* caching=nullptr)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **point** (const C3Vector&): a point expressed relative to the world reference frame.
- **usrData** (unsigned int*, default: nullptr): the user data of the colliding voxel.
- **caching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding voxel in the OC-tree).

#### Return values:

the collision status

#### See also:

[geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision), [geom_getOctreeOctreeCollision](#geom_getOctreeOctreeCollision), [geom_getOctreePtcloudCollision](#geom_getOctreePtcloudCollision), [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision), [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision), [geom_getBoxPointCollision](#geom_getBoxPointCollision)

### <a name="geom_getOctreePointDistanceIfSmaller" id="geom_getOctreePointDistanceIfSmaller"></a> geom_getOctreePointDistanceIfSmaller

Computes the minimum distance between an OC-tree object and a point.

#### Synopsis:

```cpp
bool geom_getOctreePointDistanceIfSmaller(const COcStruct* ocStruct,const CPose& octreeTransformation,const C3Vector& point,double& dist,C3Vector* ocMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **point** (const C3Vector&): the position of the point.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **ocMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the OC-tree.
- **ocCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the voxel that holds the minimum distance point on the OC-tree).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller), [geom_getOctreeOctreeDistanceIfSmaller](#geom_getOctreeOctreeDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller), [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getOctreePtcloudCollision" id="geom_getOctreePtcloudCollision"></a> geom_getOctreePtcloudCollision

Checks for collision between am OC-tree object and a point-cloud object.

#### Synopsis:

```cpp
bool geom_getOctreePtcloudCollision(const COcStruct* ocStruct,const CPose& octreeTransformation,const CPcStruct* pcStruct,const CPose& ptcloudTransformation,unsigned long long int* ocCaching=nullptr,unsigned long long int* pcCaching=nullptr)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **pcStruct** (const CPcStruct*): a pointer to the point-cloud object.
- **ptcloudTransformation** (const CPose&): the transformation frame of the point-cloud.
- **ocCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding voxel in the OC-tree).
- **pcCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding point in the point-cloud).

#### Return values:

the collision status

#### See also:

[geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision), [geom_getOctreeOctreeCollision](#geom_getOctreeOctreeCollision), [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision), [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision), [geom_getOctreePointCollision](#geom_getOctreePointCollision)

### <a name="geom_getOctreePtcloudDistanceIfSmaller" id="geom_getOctreePtcloudDistanceIfSmaller"></a> geom_getOctreePtcloudDistanceIfSmaller

Computes the minimum distance between an OC-tree object and a point-cloud object.

#### Synopsis:

```cpp
bool geom_getOctreePtcloudDistanceIfSmaller(const COcStruct* ocStruct,const CPose& octreeTransformation,const CPcStruct* pcStruct,const CPose& pcTransformation,double& dist,C3Vector* ocMinDistPt=nullptr,C3Vector* pcMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr,unsigned long long int* pcCaching=nullptr)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **pcStruct** (const CPcStruct*): a pointer to the point-cloud object.
- **pcTransformation** (const CPose&): the transformation frame of the point-cloud.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **ocMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the OC-tree.
- **pcMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the point-cloud.
- **ocCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the voxel that holds the minimum distance point on the OC-tree).
- **pcCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the point that holds the minimum distance on the point-cloud).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getOctreeOctreeDistanceIfSmaller](#geom_getOctreeOctreeDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller), [geom_getPtcloudPtcloudDistanceIfSmaller](#geom_getPtcloudPtcloudDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller)

### <a name="geom_getOctreeSegmentCollision" id="geom_getOctreeSegmentCollision"></a> geom_getOctreeSegmentCollision

Checks for collision between an OC-tree object and a segment.

#### Synopsis:

```cpp
bool geom_getOctreeSegmentCollision(const COcStruct* ocStruct,const CPose& octreeTransformation,const C3Vector& segmentExtremity,const C3Vector& segmentVector,unsigned long long int* caching=nullptr)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **segmentExtremity** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **caching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding voxel in the OC-tree).

#### Return values:

the collision status

#### See also:

[geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision), [geom_getMeshSegmentCollision](#geom_getMeshSegmentCollision), [geom_getOctreeOctreeCollision](#geom_getOctreeOctreeCollision), [geom_getOctreePtcloudCollision](#geom_getOctreePtcloudCollision), [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision), [geom_getOctreePointCollision](#geom_getOctreePointCollision), [geom_getBoxSegmentCollision](#geom_getBoxSegmentCollision), [geom_getTriangleSegmentCollision](#geom_getTriangleSegmentCollision)

### <a name="geom_getOctreeSegmentDistanceIfSmaller" id="geom_getOctreeSegmentDistanceIfSmaller"></a> geom_getOctreeSegmentDistanceIfSmaller

Computes the minimum distance between an OC-tree object and a segment.

#### Synopsis:

```cpp
bool geom_getOctreeSegmentDistanceIfSmaller(const COcStruct* ocStruct,const CPose& octreeTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* ocMinDistPt=nullptr,C3Vector* segmentDistSegPt=nullptr,unsigned long long int* ocCaching=nullptr)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **segmentEndPoint** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **ocMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the OC-tree.
- **segmentDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the segment.
- **ocCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the voxel that holds the minimum distance point on the OC-tree).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller), [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getOctreeOctreeDistanceIfSmaller](#geom_getOctreeOctreeDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getSegmentSegmentDistanceIfSmaller](#geom_getSegmentSegmentDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getOctreeSerializationData" id="geom_getOctreeSerializationData"></a> geom_getOctreeSerializationData

Retrieves the serialization data from an OC-tree object.

#### Synopsis:

```cpp
void geom_getOctreeSerializationData(const COcStruct* ocStruct,std::vector& serializationData)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **serializationData** (std::vector&): the serialization data.

#### Return values:

No return values

#### See also:

[geom_getOctreeFromSerializationData](#geom_getOctreeFromSerializationData)

### <a name="geom_getOctreeTriangleCollision" id="geom_getOctreeTriangleCollision"></a> geom_getOctreeTriangleCollision

Checks for collision between an OC-tree object and a triangle.

#### Synopsis:

```cpp
bool geom_getOctreeTriangleCollision(const COcStruct* ocStruct,const CPose& octreeTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,unsigned long long int* caching=nullptr)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **caching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next collision checking between the same two entities (memorizes the colliding voxel in the OC-tree).

#### Return values:

the collision status

#### See also:

[geom_getMeshOctreeCollision](#geom_getMeshOctreeCollision), [geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision), [geom_getOctreeOctreeCollision](#geom_getOctreeOctreeCollision), [geom_getOctreePtcloudCollision](#geom_getOctreePtcloudCollision), [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision), [geom_getOctreePointCollision](#geom_getOctreePointCollision), [geom_getBoxTriangleCollision](#geom_getBoxTriangleCollision), [geom_getTriangleTriangleCollision](#geom_getTriangleTriangleCollision), [geom_getTriangleSegmentCollision](#geom_getTriangleSegmentCollision)

### <a name="geom_getOctreeTriangleDistanceIfSmaller" id="geom_getOctreeTriangleDistanceIfSmaller"></a> geom_getOctreeTriangleDistanceIfSmaller

Computes the minimum distance between an OC-tree object and a triangle.

#### Synopsis:

```cpp
bool geom_getOctreeTriangleDistanceIfSmaller(const COcStruct* ocStruct,const CPose& octreeTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* ocMinDistPt=nullptr,C3Vector* triangleMinDistPt=nullptr,unsigned long long int* ocCaching=nullptr)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **ocMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the OC-tree.
- **triangleMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the triangle.
- **ocCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the voxel that holds the minimum distance point on the OC-tree).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshOctreeDistanceIfSmaller](#geom_getMeshOctreeDistanceIfSmaller), [geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getOctreeOctreeDistanceIfSmaller](#geom_getOctreeOctreeDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller), [geom_getTriangleTriangleDistanceIfSmaller](#geom_getTriangleTriangleDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller)

### <a name="geom_getOctreeVoxelData" id="geom_getOctreeVoxelData"></a> geom_getOctreeVoxelData

Retrieves voxel data and optionally user data from an OC-tree.

#### Synopsis:

```cpp
void geom_getOctreeVoxelData(const COcStruct* ocStruct,std::vector& voxelData,std::vector* userData=nullptr)
```

#### Parameters:

- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **voxelData** (std::vector&): the voxel data: for each voxel, there are 3 position values (XYZ) and 4 color values (RGBA, 0-1), i.e. 7 values per voxel.
- **userData** (std::vector*, default: nullptr): the user data: for each voxel, there is one user data value.

#### Return values:

No return values

### <a name="geom_getPtcloudFromSerializationData" id="geom_getPtcloudFromSerializationData"></a> geom_getPtcloudFromSerializationData

Creates a point-cloud object based on serialization data.

#### Synopsis:

```cpp
CPcStruct* geom_getPtcloudFromSerializationData(const unsigned char* serializationData)
```

#### Parameters:

- **serializationData** (const unsigned char*): the serialization data previously generated with [geom_getPtcloudSerializationData](#geom_getPtcloudSerializationData).

#### Return values:

A pointer to the created point-cloud object in case of success, nullptr otherwise.

#### See also:

[geom_getPtcloudSerializationData](#geom_getPtcloudSerializationData)

### <a name="geom_getPtcloudPointDistanceIfSmaller" id="geom_getPtcloudPointDistanceIfSmaller"></a> geom_getPtcloudPointDistanceIfSmaller

Computes the minimum distance between a point-cloud object and a point.

#### Synopsis:

```cpp
bool geom_getPtcloudPointDistanceIfSmaller(const CPcStruct* pcStruct,const CPose& pcTransformation,const C3Vector& point,double& dist,C3Vector* pcMinDistPt=nullptr,unsigned long long int* pcCaching=nullptr)
```

#### Parameters:

- **pcStruct** (const CPcStruct*): a pointer to the point-cloud object.
- **pcTransformation** (const CPose&): the transformation frame of the point-cloud.
- **point** (const C3Vector&): the position of the point.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **pcMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the point-cloud.
- **pcCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the point that holds the minimum distance point on the point-cloud).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller), [geom_getPtcloudPtcloudDistanceIfSmaller](#geom_getPtcloudPtcloudDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getPtcloudPoints" id="geom_getPtcloudPoints"></a> geom_getPtcloudPoints

Retrieves point data from a point-cloud.

#### Synopsis:

```cpp
void geom_getPtcloudPoints(const CPcStruct* pcStruct,std::vector& pointData,double prop=1.0)
```

#### Parameters:

- **pcStruct** (const CPcStruct*): a pointer to the point-cloud object.
- **pointData** (std::vector&): the point data: for each point, there are 3 position values (XYZ) and 4 color values (RGBA, 0-1), i.e. 7 values per point.
- **prop** (double, default: 1.0): the proportion of data to retrieve, on a voxel-basis. 1.0 retrieves all data, 0.25 retrieves 25% of the data.

#### Return values:

No return values

### <a name="geom_getPtcloudPtcloudDistanceIfSmaller" id="geom_getPtcloudPtcloudDistanceIfSmaller"></a> geom_getPtcloudPtcloudDistanceIfSmaller

Computes the minimum distance between two point-cloud objects.

#### Synopsis:

```cpp
bool geom_getPtcloudPtcloudDistanceIfSmaller(const CPcStruct* pc1Struct,const CPose& pc1Transformation,const CPcStruct* pc2Struct,const CPose& pc2Transformation,double& dist,C3Vector* pc1MinDistPt=nullptr,C3Vector* pc2MinDistPt=nullptr,unsigned long long int* pc1Caching=nullptr,unsigned long long int* pc2Caching=nullptr)
```

#### Parameters:

- **pc1Struct** (const CPcStruct*): a pointer to the first point-cloud object.
- **pc1Transformation** (const CPose&): the transformation frame of the first point-cloud.
- **pc2Struct** (const CPcStruct*): a pointer to the second point-cloud object.
- **pc2Transformation** (const CPose&): the transformation frame of the second point-cloud.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **pc1MinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the first point-cloud.
- **pc2MinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the second point-cloud.
- **pc1Caching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the point that holds the minimum distance point on the first point-cloud).
- **pc2Caching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the point that holds the minimum distance point on the second point-cloud).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller)

### <a name="geom_getPtcloudSegmentDistanceIfSmaller" id="geom_getPtcloudSegmentDistanceIfSmaller"></a> geom_getPtcloudSegmentDistanceIfSmaller

Computes the minimum distance between a point-cloud and a segment.

#### Synopsis:

```cpp
bool geom_getPtcloudSegmentDistanceIfSmaller(const CPcStruct* pcStruct,const CPose& pcTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* pcMinDistPt=nullptr,C3Vector* segmentDistSegPt=nullptr,unsigned long long int* pcCaching=nullptr)
```

#### Parameters:

- **pcStruct** (const CPcStruct*): a pointer to the point-cloud object.
- **pcTransformation** (const CPose&): the transformation frame of the point-cloud.
- **segmentEndPoint** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **pcMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the point-cloud.
- **segmentDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the segment.
- **pcCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the point that holds the minimum distance point on the point-cloud).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getPtcloudPtcloudDistanceIfSmaller](#geom_getPtcloudPtcloudDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller), [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getSegmentSegmentDistanceIfSmaller](#geom_getSegmentSegmentDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getPtcloudSerializationData" id="geom_getPtcloudSerializationData"></a> geom_getPtcloudSerializationData

Retrieves the serialization data from a point-cloud object.

#### Synopsis:

```cpp
void geom_getPtcloudSerializationData(const CPcStruct* pcStruct,std::vector& serializationData)
```

#### Parameters:

- **pcStruct** (const CPcStruct*): a pointer to the point-cloud object.
- **serializationData** (std::vector&): the serialization data.

#### Return values:

No return values

#### See also:

[geom_getPtcloudFromSerializationData](#geom_getPtcloudFromSerializationData)

### <a name="geom_getPtcloudTriangleDistanceIfSmaller" id="geom_getPtcloudTriangleDistanceIfSmaller"></a> geom_getPtcloudTriangleDistanceIfSmaller

Computes the minimum distance between a point-cloud object and a triangle.

#### Synopsis:

```cpp
bool geom_getPtcloudTriangleDistanceIfSmaller(const CPcStruct* pcStruct,const CPose& pcTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,C3Vector* pcMinDistPt=nullptr,C3Vector* triangleMinDistPt=nullptr,unsigned long long int* pcCaching=nullptr)
```

#### Parameters:

- **pcStruct** (const CPcStruct*): a pointer to the point-cloud object.
- **pcTransformation** (const CPose&): the transformation frame of the point-cloud.
- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **pcMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the point-cloud.
- **triangleMinDistPt** (C3Vector*, default: nullptr): the minimum distance point on the triangle.
- **pcCaching** (unsigned long long int*, default: nullptr): a value that can be used to speed-up next distance calculation between the same two entities (memorizes the point that holds the minimum distance point on the point-cloud).

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshPtcloudDistanceIfSmaller](#geom_getMeshPtcloudDistanceIfSmaller), [geom_getOctreePtcloudDistanceIfSmaller](#geom_getOctreePtcloudDistanceIfSmaller), [geom_getPtcloudPtcloudDistanceIfSmaller](#geom_getPtcloudPtcloudDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller), [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller), [geom_getTriangleTriangleDistanceIfSmaller](#geom_getTriangleTriangleDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller)

### <a name="geom_getSegmentPointDistanceIfSmaller" id="geom_getSegmentPointDistanceIfSmaller"></a> geom_getSegmentPointDistanceIfSmaller

Computes the minimum distance between a segment and a point.

#### Synopsis:

```cpp
bool geom_getSegmentPointDistanceIfSmaller(const C3Vector& segmentEndPoint,const C3Vector& segmentVector,const C3Vector& point,double& dist,C3Vector* segmentDistSegPt=nullptr)
```

#### Parameters:

- **segmentEndPoint** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **point** (const C3Vector&): position of the point
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **segmentDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the segment.

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller), [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller), [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller), [geom_getSegmentSegmentDistanceIfSmaller](#geom_getSegmentSegmentDistanceIfSmaller)

### <a name="geom_getSegmentSegmentDistanceIfSmaller" id="geom_getSegmentSegmentDistanceIfSmaller"></a> geom_getSegmentSegmentDistanceIfSmaller

Computes the minimum distance between two segments.

#### Synopsis:

```cpp
bool geom_getSegmentSegmentDistanceIfSmaller(const C3Vector& segment1EndPoint,const C3Vector& segment1Vector,const C3Vector& segment2EndPoint,const C3Vector& segment2Vector,double& dist,C3Vector* segment1DistSegPt=nullptr,C3Vector* segment2DistSegPt=nullptr)
```

#### Parameters:

- **segment1EndPoint** (const C3Vector&): position of point A on the first segment
- **segment1Vector** (const C3Vector&): vector stretching from point A to point B on the first segment.
- **segment2EndPoint** (const C3Vector&): position of point A on the second segment
- **segment2Vector** (const C3Vector&): vector stretching from point A to point B on the second segment.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **segment1DistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the first segment.
- **segment2DistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the second segment.

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getTrianglePointDistanceIfSmaller" id="geom_getTrianglePointDistanceIfSmaller"></a> geom_getTrianglePointDistanceIfSmaller

Computes the minimum distance between a triangle and a point.

#### Synopsis:

```cpp
bool geom_getTrianglePointDistanceIfSmaller(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& point,double& dist,C3Vector* triangleDistSegPt=nullptr)
```

#### Parameters:

- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **point** (const C3Vector&): position of the point
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **triangleDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the triangle.

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller), [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller), [geom_getBoxPointDistanceIfSmaller](#geom_getBoxPointDistanceIfSmaller), [geom_getTriangleTriangleDistanceIfSmaller](#geom_getTriangleTriangleDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getTriangleSegmentCollision" id="geom_getTriangleSegmentCollision"></a> geom_getTriangleSegmentCollision

Checks for collision between a triangle and a segment.

#### Synopsis:

```cpp
bool geom_getTriangleSegmentCollision(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,std::vector* intersections=nullptr)
```

#### Parameters:

- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **segmentEndPoint** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **intersections** (std::vector*, default: nullptr): if not nullptr, then a possible collision will be extracted as interference segment (where both segment points are always coincident) (set to nullptr for fast operation).

#### Return values:

the collision status

#### See also:

[geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision), [geom_getMeshSegmentCollision](#geom_getMeshSegmentCollision), [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision), [geom_getOctreeSegmentCollision](#geom_getOctreeSegmentCollision), [geom_getBoxTriangleCollision](#geom_getBoxTriangleCollision), [geom_getBoxSegmentCollision](#geom_getBoxSegmentCollision), [geom_getTriangleTriangleCollision](#geom_getTriangleTriangleCollision)

### <a name="geom_getTriangleSegmentDistanceIfSmaller" id="geom_getTriangleSegmentDistanceIfSmaller"></a> geom_getTriangleSegmentDistanceIfSmaller

Computes the minimum distance between a triangle and a segment.

#### Synopsis:

```cpp
bool geom_getTriangleSegmentDistanceIfSmaller(const C3Vector& p,const C3Vector& v,const C3Vector& w,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,C3Vector* triangleDistSegPt=nullptr,C3Vector* segmentDistSegPt=nullptr)
```

#### Parameters:

- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **segmentEndPoint** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **triangleDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the triangle.
- **segmentDistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the segment.

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getPtcloudSegmentDistanceIfSmaller](#geom_getPtcloudSegmentDistanceIfSmaller), [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller), [geom_getBoxSegmentDistanceIfSmaller](#geom_getBoxSegmentDistanceIfSmaller), [geom_getTriangleTriangleDistanceIfSmaller](#geom_getTriangleTriangleDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller), [geom_getSegmentSegmentDistanceIfSmaller](#geom_getSegmentSegmentDistanceIfSmaller), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_getTriangleTriangleCollision" id="geom_getTriangleTriangleCollision"></a> geom_getTriangleTriangleCollision

Checks for collision between two triangles.

#### Synopsis:

```cpp
bool geom_getTriangleTriangleCollision(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,std::vector* intersections=nullptr)
```

#### Parameters:

- **p1** (const C3Vector&): position of point A on triangle 1.
- **v1** (const C3Vector&): vector stretching from point A to point B on triangle 1.
- **w1** (const C3Vector&): vector stretching from point A to point C on triangle 1.
- **p2** (const C3Vector&): position of point A on triangle 2.
- **v2** (const C3Vector&): vector stretching from point A to point B on triangle 2.
- **w2** (const C3Vector&): vector stretching from point A to point C on triangle 2.
- **intersections** (std::vector*, default: nullptr): if not nullptr, then all collisions will be extracted as interference segments (set to nullptr for fast operation).

#### Return values:

the collision status

#### See also:

[geom_getMeshTriangleCollision](#geom_getMeshTriangleCollision), [geom_getOctreeTriangleCollision](#geom_getOctreeTriangleCollision), [geom_getBoxTriangleCollision](#geom_getBoxTriangleCollision), [geom_getTriangleSegmentCollision](#geom_getTriangleSegmentCollision)

### <a name="geom_getTriangleTriangleDistanceIfSmaller" id="geom_getTriangleTriangleDistanceIfSmaller"></a> geom_getTriangleTriangleDistanceIfSmaller

Computes the minimum distance between two triangles.

#### Synopsis:

```cpp
bool geom_getTriangleTriangleDistanceIfSmaller(const C3Vector& p1,const C3Vector& v1,const C3Vector& w1,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,double& dist,C3Vector* triangle1DistSegPt=nullptr,C3Vector* triangle2DistSegPt=nullptr)
```

#### Parameters:

- **p1** (const C3Vector&): position of point A on triangle 1.
- **v1** (const C3Vector&): vector stretching from point A to point B on triangle 1.
- **w1** (const C3Vector&): vector stretching from point A to point C on triangle 1.
- **p2** (const C3Vector&): position of point A on triangle 2.
- **v2** (const C3Vector&): vector stretching from point A to point B on triangle 2.
- **w2** (const C3Vector&): vector stretching from point A to point C on triangle 2.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **triangle1DistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the first triangle.
- **triangle2DistSegPt** (C3Vector*, default: nullptr): the minimum distance point on the second triangle.

#### Return values:

whether the minimum distance is smaller than the specified threshold

#### See also:

[geom_getMeshTriangleDistanceIfSmaller](#geom_getMeshTriangleDistanceIfSmaller), [geom_getOctreeTriangleDistanceIfSmaller](#geom_getOctreeTriangleDistanceIfSmaller), [geom_getPtcloudTriangleDistanceIfSmaller](#geom_getPtcloudTriangleDistanceIfSmaller), [geom_getBoxTriangleDistanceIfSmaller](#geom_getBoxTriangleDistanceIfSmaller), [geom_getTriangleSegmentDistanceIfSmaller](#geom_getTriangleSegmentDistanceIfSmaller), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller)

### <a name="geom_insertColorPointsIntoOctree" id="geom_insertColorPointsIntoOctree"></a> geom_insertColorPointsIntoOctree

Inserts points (with individual colors) into an OC-tree, effectively creating OC-tree voxels.

#### Synopsis:

```cpp
void geom_insertColorPointsIntoOctree(COcStruct* ocStruct,const CPose& octreeTransformation,const double* points,int pointCnt,const unsigned char* rgbaData=nullptr,const unsigned int* usrData=nullptr)
```

#### Parameters:

- **ocStruct** (COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **points** (const double*): an array of point values (XYZ-triplets). Points are expressed relative to the world reference frame.
- **pointCnt** (int): the number of points.
- **rgbaData** (const unsigned char*, default: nullptr): the RGBA colors (0-255) of the various points (i.e. 4 values per point: red, green, blue and alpha).
- **usrData** (const unsigned int*, default: nullptr): the user data of the various points (i.e. one user data per point).

#### Return values:

No return values

#### See also:

[geom_insertPointsIntoOctree](#geom_insertPointsIntoOctree), [geom_insertMeshIntoOctree](#geom_insertMeshIntoOctree), [geom_insertOctreeIntoOctree](#geom_insertOctreeIntoOctree), [geom_removePointsFromOctree](#geom_removePointsFromOctree)

### <a name="geom_insertColorPointsIntoPtcloud" id="geom_insertColorPointsIntoPtcloud"></a> geom_insertColorPointsIntoPtcloud

Inserts points (with individual colors) into point-cloud object.

#### Synopsis:

```cpp
void geom_insertColorPointsIntoPtcloud(CPcStruct* pcStruct,const CPose& ptcloudTransformation,const double* points,int pointCnt,const unsigned char* rgbaData=nullptr,double proximityTol=0.001)
```

#### Parameters:

- **pcStruct** (CPcStruct*): a pointer to the point-cloud object.
- **ptcloudTransformation** (const CPose&): the transformation frame of the point-cloud.
- **points** (const double*): an array of point values (XYZ-triplets). Points are expressed relative to the world reference frame.
- **pointCnt** (int): the number of points.
- **rgbaData** (const unsigned char*, default: nullptr): the RGBA colors (0-255) of the various points (i.e. 4 values per point: red, green, blue and alpha).
- **proximityTol** (double, default: 0.001): the distance tolerance to other points (i.e. point-point distances below the specified value are not permitted and points will be omitted).

#### Return values:

No return values

#### See also:

[geom_insertPointsIntoPtcloud](#geom_insertPointsIntoPtcloud), [geom_removePointsFromPtcloud](#geom_removePointsFromPtcloud), [geom_removeOctreeFromPtcloud](#geom_removeOctreeFromPtcloud), [geom_intersectPointsWithPtcloud](#geom_intersectPointsWithPtcloud)

### <a name="geom_insertMeshIntoOctree" id="geom_insertMeshIntoOctree"></a> geom_insertMeshIntoOctree

Inserts a mesh object into an OC-tree, effectively creating OC-tree voxels.

#### Synopsis:

```cpp
void geom_insertMeshIntoOctree(COcStruct* ocStruct,const CPose& octreeTransformation,const CObbStruct* obbStruct,const CPose& meshTransformation,const unsigned char rgbaData[4]=nullptr,unsigned int usrData=0)
```

#### Parameters:

- **ocStruct** (COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **obbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **rgbaData** (const unsigned char, default: nullptr): the RGBA color (0-255) of the generated voxels (4 values: red, green, blue and alpha).
- **usrData** (unsigned int, default: 0): the user data of the generated voxels.

#### Return values:

No return values

#### See also:

[geom_insertPointsIntoOctree](#geom_insertPointsIntoOctree), [geom_insertColorPointsIntoOctree](#geom_insertColorPointsIntoOctree), [geom_insertOctreeIntoOctree](#geom_insertOctreeIntoOctree), [geom_removeMeshFromOctree](#geom_removeMeshFromOctree)

### <a name="geom_insertOctreeIntoOctree" id="geom_insertOctreeIntoOctree"></a> geom_insertOctreeIntoOctree

Inserts an OC-tree object into another OC-tree object, effectively creating OC-tree voxels in the second OC-tree object.

#### Synopsis:

```cpp
void geom_insertOctreeIntoOctree(COcStruct* oc1Struct,const CPose& octree1Transformation,const COcStruct* oc2Struct,const CPose& octree2Transformation,const unsigned char rgbaData[4]=nullptr,unsigned int usrData=0)
```

#### Parameters:

- **oc1Struct** (COcStruct*): a pointer to the OC-tree object.
- **octree1Transformation** (const CPose&): the transformation frame of the OC-tree.
- **oc2Struct** (const COcStruct*): a pointer to the other OC-tree object.
- **octree2Transformation** (const CPose&): the transformation frame of the other OC-tree object.
- **rgbaData** (const unsigned char, default: nullptr): the RGBA color (0-255) of the generated voxels (4 values: red, green, blue and alpha).
- **usrData** (unsigned int, default: 0): the user data of the generated voxels.

#### Return values:

No return values

#### See also:

[geom_insertPointsIntoOctree](#geom_insertPointsIntoOctree), [geom_insertColorPointsIntoOctree](#geom_insertColorPointsIntoOctree), [geom_insertMeshIntoOctree](#geom_insertMeshIntoOctree), [geom_removeOctreeFromOctree](#geom_removeOctreeFromOctree)

### <a name="geom_insertPointsIntoOctree" id="geom_insertPointsIntoOctree"></a> geom_insertPointsIntoOctree

Inserts points into an OC-tree, effectively creating OC-tree voxels.

#### Synopsis:

```cpp
void geom_insertPointsIntoOctree(COcStruct* ocStruct,const CPose& octreeTransformation,const double* points,int pointCnt,const unsigned char rgbaData[4]=nullptr,unsigned int usrData=0)
```

#### Parameters:

- **ocStruct** (COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **points** (const double*): an array of point values (XYZ-triplets). Points are expressed relative to the world reference frame.
- **pointCnt** (int): the number of points.
- **rgbaData** (const unsigned char, default: nullptr): the RGBA color (0-255) of the generated voxels (4 values: red, green, blue and alpha).
- **usrData** (unsigned int, default: 0): the user data of the generated voxels.

#### Return values:

No return values

#### See also:

[geom_insertColorPointsIntoOctree](#geom_insertColorPointsIntoOctree), [geom_insertMeshIntoOctree](#geom_insertMeshIntoOctree), [geom_insertOctreeIntoOctree](#geom_insertOctreeIntoOctree), [geom_removePointsFromOctree](#geom_removePointsFromOctree)

### <a name="geom_insertPointsIntoPtcloud" id="geom_insertPointsIntoPtcloud"></a> geom_insertPointsIntoPtcloud

Inserts points into a point-cloud object.

#### Synopsis:

```cpp
void geom_insertPointsIntoPtcloud(CPcStruct* pcStruct,const CPose& ptcloudTransformation,const double* points,int pointCnt,const unsigned char rgbaData[4]=nullptr,double proximityTol=0.001)
```

#### Parameters:

- **pcStruct** (CPcStruct*): a pointer to the point-cloud object.
- **ptcloudTransformation** (const CPose&): the transformation frame of the point-cloud.
- **points** (const double*): an array of point values (XYZ-triplets). Points are expressed relative to the world reference frame.
- **pointCnt** (int): the number of points.
- **rgbaData** (const unsigned char, default: nullptr): the RGBA color (0-255) of the generated points (4 values: red, green, blue and alpha).
- **proximityTol** (double, default: 0.001): the distance tolerance to other points (i.e. point-point distances below the specified value are not permitted and points will be omitted).

#### Return values:

No return values

#### See also:

[geom_insertColorPointsIntoPtcloud](#geom_insertColorPointsIntoPtcloud), [geom_removePointsFromPtcloud](#geom_removePointsFromPtcloud), [geom_removeOctreeFromPtcloud](#geom_removeOctreeFromPtcloud), [geom_intersectPointsWithPtcloud](#geom_intersectPointsWithPtcloud)

### <a name="geom_intersectPointsWithPtcloud" id="geom_intersectPointsWithPtcloud"></a> geom_intersectPointsWithPtcloud

Performs an intersection operation between a point-cloud and specific points: point-cloud points within a certain tolerance to the specified points will be kept, others will be removed.

#### Synopsis:

```cpp
bool geom_intersectPointsWithPtcloud(CPcStruct* pcStruct,const CPose& ptcloudTransformation,const double* points,int pointCnt,double proximityTol=0.001)
```

#### Parameters:

- **pcStruct** (CPcStruct*): a pointer to the point-cloud object.
- **ptcloudTransformation** (const CPose&): the transformation frame of the point-cloud.
- **points** (const double*): an array of point values (XYZ-triplets). Points are expressed relative to the world reference frame.
- **pointCnt** (int): the number of points.
- **proximityTol** (double, default: 0.001): the distance tolerance to other points (i.e. point-point distances above the specified value will result in removal of those point-cloud points.

#### Return values:

No return values

### <a name="geom_isPointInVolume" id="geom_isPointInVolume"></a> geom_isPointInVolume

Checks whether a point lies within a proximity sensor volume.

#### Synopsis:

```cpp
bool geom_isPointInVolume(const double* planesIn,int planesInSize,const CPose& sensorTransformation,const C3Vector& point)
```

#### Parameters:

- **planesIn** (const double*): an array describing a convex volume, via planes. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the inside if N*P+D>=0. N is the normal vector of the plane, pointing towards the volume inside.
- **planesInSize** (int): the number of values in the planesIn array.
- **sensorTransformation** (const CPose&): the transformation frame of the volume.
- **point** (const C3Vector&): the point to test.

#### Return values:

whether the point lies within the sensor's volume.

### <a name="geom_raySensorDetectMeshIfSmaller" id="geom_raySensorDetectMeshIfSmaller"></a> geom_raySensorDetectMeshIfSmaller

Computes the distance from a ray-type proximity sensor to a mesh object.

#### Synopsis:

```cpp
bool geom_raySensorDetectMeshIfSmaller(const CPose& sensorTransformation,double rayOffset,double rayLength,const CObbStruct* meshObbStruct,const CPose& meshTransformation,double& dist,double forbiddenDist=0.0,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr,bool* forbiddenDistTouched=nullptr)
```

#### Parameters:

- **sensorTransformation** (const CPose&): the transformation frame of the sensor.
- **rayOffset** (double): the offset of the ray (i.e. where the ray starts), along its frame's Z-axis.
- **rayLength** (double): the length of the ray.
- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **forbiddenDist** (double, default: 0.0): the distance threshold along the sensor's Z-axis, where detection is forbidden. If something is detected in that section of the ray, then the return will be false.
- **fast** (bool, default: false): if true, calculation will be faster, however only the first encountered feature that satisfies the detection requirements will be returned (and not the closest of those features).
- **frontDetection** (bool, default: true): if true, front faces of the mesh will be detected.
- **backDetection** (bool, default: true): if true, back faces of the mesh will be detected.
- **maxAngle** (double, default: 0.0): the maximum allowed angle between detection ray and normal vector of the detected face. Zero to ignore the maximum angle.
- **detectPt** (C3Vector*, default: nullptr): the position of the detected point, if return value is true.
- **triN** (C3Vector*, default: nullptr): the normal vector of the detected face, if return value is true.
- **forbiddenDistTouched** (bool*, default: nullptr): if something was detected in the forbidden section of the sensor, then the value pointed by this pointer will be true.

#### Return values:

whether the sensor detected something and the distance is smaller than the specified threshold

#### See also:

[geom_raySensorDetectOctreeIfSmaller](#geom_raySensorDetectOctreeIfSmaller), [geom_getMeshSegmentDistanceIfSmaller](#geom_getMeshSegmentDistanceIfSmaller), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller)

### <a name="geom_raySensorDetectOctreeIfSmaller" id="geom_raySensorDetectOctreeIfSmaller"></a> geom_raySensorDetectOctreeIfSmaller

Computes the distance from a ray-type proximity sensor to an OC-tree object.

#### Synopsis:

```cpp
bool geom_raySensorDetectOctreeIfSmaller(const CPose& sensorTransformation,double rayOffset,double rayLength,const COcStruct* ocStruct,const CPose& octreeTransformation,double& dist,double forbiddenDist=0.0,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr,bool* forbiddenDistTouched=nullptr)
```

#### Parameters:

- **sensorTransformation** (const CPose&): the transformation frame of the sensor.
- **rayOffset** (double): the offset of the ray (i.e. where the ray starts), along its frame's Z-axis.
- **rayLength** (double): the length of the ray.
- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **forbiddenDist** (double, default: 0.0): the distance threshold along the sensor's Z-axis, where detection is forbidden. If something is detected in that section of the ray, then the return will be false.
- **fast** (bool, default: false): if true, calculation will be faster, however only the first encountered feature that satisfies the detection requirements will be returned (and not the closest of those features).
- **frontDetection** (bool, default: true): if true, front faces of the OC-tree will be detected.
- **backDetection** (bool, default: true): if true, back faces of the OC-tree will be detected.
- **maxAngle** (double, default: 0.0): the maximum allowed angle between detection ray and normal vector of the detected face. Zero to ignore the maximum angle.
- **detectPt** (C3Vector*, default: nullptr): the position of the detected point, if return value is true.
- **triN** (C3Vector*, default: nullptr): the normal vector of the detected face, if return value is true.
- **forbiddenDistTouched** (bool*, default: nullptr): if something was detected in the forbidden section of the sensor, then the value pointed by this pointer will be true.

#### Return values:

whether the sensor detected something and the distance is smaller than the specified threshold

#### See also:

[geom_raySensorDetectMeshIfSmaller](#geom_raySensorDetectMeshIfSmaller), [geom_getOctreeSegmentDistanceIfSmaller](#geom_getOctreeSegmentDistanceIfSmaller), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller)

### <a name="geom_removeMeshFromOctree" id="geom_removeMeshFromOctree"></a> geom_removeMeshFromOctree

Deletes OC-tree voxels that occupy the space of the specified mesh.

#### Synopsis:

```cpp
bool geom_removeMeshFromOctree(COcStruct* ocStruct,const CPose& octreeTransformation,const CObbStruct* obbStruct,const CPose& meshTransformation)
```

#### Parameters:

- **ocStruct** (COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **obbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.

#### Return values:

true if the OC-tree does not contain any voxels anymore, after this operation. false otherwise.

#### See also:

[geom_removePointsFromOctree](#geom_removePointsFromOctree), [geom_removeOctreeFromOctree](#geom_removeOctreeFromOctree), [geom_insertMeshIntoOctree](#geom_insertMeshIntoOctree)

### <a name="geom_removeOctreeFromOctree" id="geom_removeOctreeFromOctree"></a> geom_removeOctreeFromOctree

Deletes OC-tree voxels that occupy the space of the other specified OC-tree.

#### Synopsis:

```cpp
bool geom_removeOctreeFromOctree(COcStruct* oc1Struct,const CPose& octree1Transformation,const COcStruct* oc2Struct,const CPose& octree2Transformation)
```

#### Parameters:

- **oc1Struct** (COcStruct*): a pointer to the OC-tree object.
- **octree1Transformation** (const CPose&): the transformation frame of the OC-tree.
- **oc2Struct** (const COcStruct*): a pointer to the other OC-tree object.
- **octree2Transformation** (const CPose&): the transformation frame of the other OC-tree.

#### Return values:

true if the OC-tree does not contain any voxels anymore, after this operation. false otherwise.

#### See also:

[geom_removePointsFromOctree](#geom_removePointsFromOctree), [geom_removeMeshFromOctree](#geom_removeMeshFromOctree), [geom_insertOctreeIntoOctree](#geom_insertOctreeIntoOctree)

### <a name="geom_removeOctreeFromPtcloud" id="geom_removeOctreeFromPtcloud"></a> geom_removeOctreeFromPtcloud

Deletes point-cloud points that occupy the space of the specified OC-tree voxels.

#### Synopsis:

```cpp
bool geom_removeOctreeFromPtcloud(CPcStruct* pcStruct,const CPose& ptcloudTransformation,const COcStruct* ocStruct,const CPose& octreeTransformation,int* countRemoved=nullptr)
```

#### Parameters:

- **pcStruct** (CPcStruct*): a pointer to the point-cloud object.
- **ptcloudTransformation** (const CPose&): the transformation frame of the point-cloud.
- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **countRemoved** (int*, default: nullptr): the number of removed points.

#### Return values:

true if the point-cloud does not contain any points anymore, after this operation. false otherwise.

#### See also:

[geom_removePointsFromPtcloud](#geom_removePointsFromPtcloud), [geom_insertPointsIntoPtcloud](#geom_insertPointsIntoPtcloud), [geom_intersectPointsWithPtcloud](#geom_intersectPointsWithPtcloud)

### <a name="geom_removePointsFromOctree" id="geom_removePointsFromOctree"></a> geom_removePointsFromOctree

Deletes OC-tree voxels that occupy the space of the specified points.

#### Synopsis:

```cpp
bool geom_removePointsFromOctree(COcStruct* ocStruct,const CPose& octreeTransformation,const double* points,int pointCnt)
```

#### Parameters:

- **ocStruct** (COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **points** (const double*): an array of point values (XYZ-triplets). Points are expressed relative to the world reference frame.
- **pointCnt** (int): the number of points.

#### Return values:

true if the OC-tree does not contain any voxels anymore, after this operation. false otherwise.

#### See also:

[geom_removeMeshFromOctree](#geom_removeMeshFromOctree), [geom_removeOctreeFromOctree](#geom_removeOctreeFromOctree), [geom_insertPointsIntoOctree](#geom_insertPointsIntoOctree)

### <a name="geom_removePointsFromPtcloud" id="geom_removePointsFromPtcloud"></a> geom_removePointsFromPtcloud

Deletes point-cloud points that occupy the proximity space of the specified points.

#### Synopsis:

```cpp
bool geom_removePointsFromPtcloud(CPcStruct* pcStruct,const CPose& ptcloudTransformation,const double* points,int pointCnt,double proximityTol,int* countRemoved=nullptr)
```

#### Parameters:

- **pcStruct** (CPcStruct*): a pointer to the point-cloud object.
- **ptcloudTransformation** (const CPose&): the transformation frame of the point-cloud.
- **points** (const double*): an array of point values (XYZ-triplets). Points are expressed relative to the world reference frame.
- **pointCnt** (int): the number of points.
- **proximityTol** (double): the proximity tolerance.
- **countRemoved** (int*, default: nullptr): the number of removed points.

#### Return values:

true if the point-cloud does not contain any points anymore, after this operation. false otherwise.

#### See also:

[geom_removeOctreeFromPtcloud](#geom_removeOctreeFromPtcloud), [geom_insertPointsIntoPtcloud](#geom_insertPointsIntoPtcloud), [geom_intersectPointsWithPtcloud](#geom_intersectPointsWithPtcloud)

### <a name="geom_scaleMesh" id="geom_scaleMesh"></a> geom_scaleMesh

Scales a mesh object.

#### Synopsis:

```cpp
void geom_scaleMesh(CObbStruct* meshObbStruct,double scalingFactor)
```

#### Parameters:

- **meshObbStruct** (CObbStruct*): a pointer to the mesh object.
- **scalingFactor** (double): the scaling factor.

#### Return values:

No return values

### <a name="geom_scaleOctree" id="geom_scaleOctree"></a> geom_scaleOctree

Scales an OC-tree object.

#### Synopsis:

```cpp
void geom_scaleOctree(COcStruct* ocStruct,double scalingFactor)
```

#### Parameters:

- **ocStruct** (COcStruct*): a pointer to the OC-tree object.
- **scalingFactor** (double): the scaling factor.

#### Return values:

No return values

### <a name="geom_scalePtcloud" id="geom_scalePtcloud"></a> geom_scalePtcloud

Scales a point-cloud object.

#### Synopsis:

```cpp
void geom_scalePtcloud(CPcStruct* pcStruct,double scalingFactor)
```

#### Parameters:

- **pcStruct** (CPcStruct*): a pointer to the point-cloud object.
- **scalingFactor** (double): the scaling factor.

#### Return values:

No return values

### <a name="geom_volumeSensorDetectMeshIfSmaller" id="geom_volumeSensorDetectMeshIfSmaller"></a> geom_volumeSensorDetectMeshIfSmaller

Computes the distance from a proximity sensor to parts of a mesh object that lie within the sensor's detection volume.

#### Synopsis:

```cpp
bool geom_volumeSensorDetectMeshIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const CPose& sensorTransformation,const CObbStruct* obbStruct,const CPose& meshTransformation,double& dist,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr)
```

#### Parameters:

- **planesIn** (const double*): an array describing a convex volume, via planes. The mesh object (or parts of it) inside the volume, and outside of volume planesOut, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the inside if N*P+D>=0. N is the normal vector of the plane, pointing towards the volume inside.
- **planesInSize** (int): the number of values in the planesIn array.
- **planesOut** (const double*): an array describing a convex volume, via planes. The mesh object (or parts of it) outside the volume and inside volume planesIn, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the outside if N*P+D<0. N is the normal vector of the plane, pointing towards the volume inside. planesOut can be nullptr if planesOutSize is 0
- **planesOutSize** (int): the number of values in the planesOut array.
- **sensorTransformation** (const CPose&): the transformation frame of the sensor (also transforms the sensor's volumes).
- **meshObbStruct** (const CObbStruct*): a pointer to the mesh object.
- **meshTransformation** (const CPose&): the transformation frame of the mesh.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **fast** (bool, default: false): if true, calculation will be faster, however only the first encountered feature that satisfies the detection requirements will be returned (and not the closest of those features).
- **frontDetection** (bool, default: true): if true, front faces of the mesh will be detected.
- **backDetection** (bool, default: true): if true, back faces of the mesh will be detected.
- **maxAngle** (double, default: 0.0): the maximum allowed angle between detection ray and normal vector of the detected face. Zero to ignore the maximum angle.
- **detectPt** (C3Vector*, default: nullptr): the position of the detected point, if return value is true.
- **triN** (C3Vector*, default: nullptr): the normal vector of the detected face, if return value is true.

#### Return values:

whether the sensor detected something and the distance is smaller than the specified threshold

#### See also:

[geom_volumeSensorDetectOctreeIfSmaller](#geom_volumeSensorDetectOctreeIfSmaller), [geom_volumeSensorDetectPtcloudIfSmaller](#geom_volumeSensorDetectPtcloudIfSmaller), [geom_volumeSensorDetectTriangleIfSmaller](#geom_volumeSensorDetectTriangleIfSmaller), [geom_volumeSensorDetectSegmentIfSmaller](#geom_volumeSensorDetectSegmentIfSmaller), [geom_raySensorDetectMeshIfSmaller](#geom_raySensorDetectMeshIfSmaller), [geom_isPointInVolume](#geom_isPointInVolume), [geom_getMeshPointDistanceIfSmaller](#geom_getMeshPointDistanceIfSmaller)

### <a name="geom_volumeSensorDetectOctreeIfSmaller" id="geom_volumeSensorDetectOctreeIfSmaller"></a> geom_volumeSensorDetectOctreeIfSmaller

Computes the distance from a proximity sensor to parts of an OC-tree object that lie within the sensor's detection volume.

#### Synopsis:

```cpp
bool geom_volumeSensorDetectOctreeIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const CPose& sensorTransformation,const COcStruct* ocStruct,const CPose& octreeTransformation,double& dist,bool fast=false,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr)
```

#### Parameters:

- **planesIn** (const double*): an array describing a convex volume, via planes. The OC-tree object (or parts of it) inside the volume, and outside of volume planesOut, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the inside if N*P+D>=0. N is the normal vector of the plane, pointing towards the volume inside.
- **planesInSize** (int): the number of values in the planesIn array.
- **planesOut** (const double*): an array describing a convex volume, via planes. The OC-tree object (or parts of it) outside the volume and inside volume planesIn, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the outside if N*P+D<0. N is the normal vector of the plane, pointing towards the volume inside. planesOut can be nullptr if planesOutSize is 0
- **planesOutSize** (int): the number of values in the planesOut array.
- **sensorTransformation** (const CPose&): the transformation frame of the sensor (also transforms the sensor's volumes).
- **ocStruct** (const COcStruct*): a pointer to the OC-tree object.
- **octreeTransformation** (const CPose&): the transformation frame of the OC-tree.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **fast** (bool, default: false): if true, calculation will be faster, however only the first encountered feature that satisfies the detection requirements will be returned (and not the closest of those features).
- **frontDetection** (bool, default: true): if true, front faces of the OC-tree will be detected.
- **backDetection** (bool, default: true): if true, back faces of the OC-tree will be detected.
- **maxAngle** (double, default: 0.0): the maximum allowed angle between detection ray and normal vector of the detected face. Zero to ignore the maximum angle.
- **detectPt** (C3Vector*, default: nullptr): the position of the detected point, if return value is true.
- **triN** (C3Vector*, default: nullptr): the normal vector of the detected face, if return value is true.

#### Return values:

whether the sensor detected something and the distance is smaller than the specified threshold

#### See also:

[geom_volumeSensorDetectMeshIfSmaller](#geom_volumeSensorDetectMeshIfSmaller), [geom_volumeSensorDetectPtcloudIfSmaller](#geom_volumeSensorDetectPtcloudIfSmaller), [geom_volumeSensorDetectTriangleIfSmaller](#geom_volumeSensorDetectTriangleIfSmaller), [geom_volumeSensorDetectSegmentIfSmaller](#geom_volumeSensorDetectSegmentIfSmaller), [geom_raySensorDetectOctreeIfSmaller](#geom_raySensorDetectOctreeIfSmaller), [geom_isPointInVolume](#geom_isPointInVolume), [geom_getOctreePointDistanceIfSmaller](#geom_getOctreePointDistanceIfSmaller)

### <a name="geom_volumeSensorDetectPtcloudIfSmaller" id="geom_volumeSensorDetectPtcloudIfSmaller"></a> geom_volumeSensorDetectPtcloudIfSmaller

Computes the distance from a proximity sensor to parts of a point-cloud object that lie within the sensor's detection volume.

#### Synopsis:

```cpp
bool geom_volumeSensorDetectPtcloudIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const CPose& sensorTransformation,const CPcStruct* pcStruct,const CPose& ptcloudTransformation,double& dist,bool fast=false,C3Vector* detectPt=nullptr)
```

#### Parameters:

- **planesIn** (const double*): an array describing a convex volume, via planes. The point-cloud object (or parts of it) inside the volume, and outside of volume planesOut, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the inside if N*P+D>=0. N is the normal vector of the plane, pointing towards the volume inside.
- **planesInSize** (int): the number of values in the planesIn array.
- **planesOut** (const double*): an array describing a convex volume, via planes. The point-cloud object (or parts of it) outside the volume and inside volume planesIn, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the outside if N*P+D<0. N is the normal vector of the plane, pointing towards the volume inside. planesOut can be nullptr if planesOutSize is 0
- **planesOutSize** (int): the number of values in the planesOut array.
- **sensorTransformation** (const CPose&): the transformation frame of the sensor (also transforms the sensor's volumes).
- **pcStruct** (const CPcStruct*): a pointer to the point-cloud object.
- **ptcloudTransformation** (const CPose&): the transformation frame of the point-cloud.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **fast** (bool, default: false): if true, calculation will be faster, however only the first encountered feature that satisfies the detection requirements will be returned (and not the closest of those features).
- **detectPt** (C3Vector*, default: nullptr): the position of the detected point, if return value is true.

#### Return values:

whether the sensor detected something and the distance is smaller than the specified threshold

#### See also:

[geom_volumeSensorDetectMeshIfSmaller](#geom_volumeSensorDetectMeshIfSmaller), [geom_volumeSensorDetectOctreeIfSmaller](#geom_volumeSensorDetectOctreeIfSmaller), [geom_volumeSensorDetectTriangleIfSmaller](#geom_volumeSensorDetectTriangleIfSmaller), [geom_volumeSensorDetectSegmentIfSmaller](#geom_volumeSensorDetectSegmentIfSmaller), [geom_isPointInVolume](#geom_isPointInVolume), [geom_getPtcloudPointDistanceIfSmaller](#geom_getPtcloudPointDistanceIfSmaller)

### <a name="geom_volumeSensorDetectSegmentIfSmaller" id="geom_volumeSensorDetectSegmentIfSmaller"></a> geom_volumeSensorDetectSegmentIfSmaller

Computes the distance from a proximity sensor to parts of a segment that lie within the sensor's detection volume.

#### Synopsis:

```cpp
bool geom_volumeSensorDetectSegmentIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const CPose& sensorTransformation,const C3Vector& segmentEndPoint,const C3Vector& segmentVector,double& dist,double maxAngle=0.0,C3Vector* detectPt=nullptr)
```

#### Parameters:

- **planesIn** (const double*): an array describing a convex volume, via planes. The segment (or parts of it) inside the volume, and outside of volume planesOut, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the inside if N*P+D>=0. N is the normal vector of the plane, pointing towards the volume inside.
- **planesInSize** (int): the number of values in the planesIn array.
- **planesOut** (const double*): an array describing a convex volume, via planes. The segment (or parts of it) outside the volume and inside volume planesIn, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the outside if N*P+D<0. N is the normal vector of the plane, pointing towards the volume inside. planesOut can be nullptr if planesOutSize is 0
- **planesOutSize** (int): the number of values in the planesOut array.
- **sensorTransformation** (const CPose&): the transformation frame of the sensor (also transforms the sensor's volumes).
- **segmentEndPoint** (const C3Vector&): position of vertex A on the segment.
- **segmentVector** (const C3Vector&): vector stretching from vertex A to vertex B on the segment.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **maxAngle** (double, default: 0.0): the maximum allowed angle between detection ray and a perpendicular vector of the segment. Zero to ignore the maximum angle.
- **detectPt** (C3Vector*, default: nullptr): the position of the detected point, if return value is true.

#### Return values:

whether the sensor detected something and the distance is smaller than the specified threshold

#### See also:

[geom_volumeSensorDetectMeshIfSmaller](#geom_volumeSensorDetectMeshIfSmaller), [geom_volumeSensorDetectOctreeIfSmaller](#geom_volumeSensorDetectOctreeIfSmaller), [geom_volumeSensorDetectPtcloudIfSmaller](#geom_volumeSensorDetectPtcloudIfSmaller), [geom_volumeSensorDetectTriangleIfSmaller](#geom_volumeSensorDetectTriangleIfSmaller), [geom_isPointInVolume](#geom_isPointInVolume), [geom_getSegmentPointDistanceIfSmaller](#geom_getSegmentPointDistanceIfSmaller)

### <a name="geom_volumeSensorDetectTriangleIfSmaller" id="geom_volumeSensorDetectTriangleIfSmaller"></a> geom_volumeSensorDetectTriangleIfSmaller

Computes the distance from a proximity sensor to parts of a triangle that lie within the sensor's detection volume.

#### Synopsis:

```cpp
bool geom_volumeSensorDetectTriangleIfSmaller(const double* planesIn,int planesInSize,const double* planesOut,int planesOutSize,const CPose& sensorTransformation,const C3Vector& p,const C3Vector& v,const C3Vector& w,double& dist,bool frontDetection=true,bool backDetection=true,double maxAngle=0.0,C3Vector* detectPt=nullptr,C3Vector* triN=nullptr)
```

#### Parameters:

- **planesIn** (const double*): an array describing a convex volume, via planes. The triangle (or parts of it) inside the volume, and outside of volume planesOut, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the inside if N*P+D>=0. N is the normal vector of the plane, pointing towards the volume inside.
- **planesInSize** (int): the number of values in the planesIn array.
- **planesOut** (const double*): an array describing a convex volume, via planes. The triangle (or parts of it) outside the volume and inside volume planesIn, is detected. Each plane is defined by 4 values: (Nx,Ny,Nz,D), where a point (Px,Py,Pz) lies on the outside if N*P+D<0. N is the normal vector of the plane, pointing towards the volume inside. planesOut can be nullptr if planesOutSize is 0
- **planesOutSize** (int): the number of values in the planesOut array.
- **sensorTransformation** (const CPose&): the transformation frame of the sensor (also transforms the sensor's volumes).
- **p** (const C3Vector&): position of vertex A on the triangle.
- **v** (const C3Vector&): vector stretching from vertex A to vertex B on the triangle.
- **w** (const C3Vector&): vector stretching from vertex A to vertex C on the triangle.
- **dist** (double&): the distance threshold as input, and the minimum distance as output if it is below the threshold.
- **frontDetection** (bool, default: true): if true, the front of the triangle will be detected.
- **backDetection** (bool, default: true): if true, the back of the triangle will be detected.
- **maxAngle** (double, default: 0.0): the maximum allowed angle between detection ray and normal vector of the triangle. Zero to ignore the maximum angle.
- **detectPt** (C3Vector*, default: nullptr): the position of the detected point, if return value is true.
- **triN** (C3Vector*, default: nullptr): the normal vector of the triangle, if return value is true.

#### Return values:

whether the sensor detected something and the distance is smaller than the specified threshold

#### See also:

[geom_volumeSensorDetectMeshIfSmaller](#geom_volumeSensorDetectMeshIfSmaller), [geom_volumeSensorDetectOctreeIfSmaller](#geom_volumeSensorDetectOctreeIfSmaller), [geom_volumeSensorDetectPtcloudIfSmaller](#geom_volumeSensorDetectPtcloudIfSmaller), [geom_volumeSensorDetectSegmentIfSmaller](#geom_volumeSensorDetectSegmentIfSmaller), [geom_isPointInVolume](#geom_isPointInVolume), [geom_getTrianglePointDistanceIfSmaller](#geom_getTrianglePointDistanceIfSmaller)
