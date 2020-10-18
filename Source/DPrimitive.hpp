#ifndef _DPRIMITIVE_HPP_
#define _DPRIMITIVE_HPP_

#include "DDataTypes.hpp"

typedef CDPoint (CurveFunc)(void *pvData, double dt);
typedef double (PtProjFunc)(void *pvData, double dOffset, CDPoint cPt, CDPoint cStart, CDPoint cEnd);

int CmpAngle(CDPoint cPt1, CDPoint cPt2);
int CropPrimitive(CDPrimitive cPrim, PDRect pRect, PDPrimObject pPrimList);
double ApproxLineSeg(int iPoints, PDPoint pPoints, PDPoint pStartDir,
	PDPoint pEndDir, PDPrimitive pPrim);
bool PointInArc(CDPoint cPt, CDLine cStart, CDLine cEnd);
CDPoint GetQuadPoint(PDPrimitive pQuad, double dt);
CDPoint GetQuadDir(PDPrimitive pQuad, double dt);
CDPoint GetQuadNormal(PDPrimitive pQuad, double dt);
double GetQuadLength(PDPrimitive pQuad, double t1, double t2);
double GetQuadPointAtDist(PDPrimitive pQuad, double t1, double dDist);
// returns 3 if it is inside, 2 if it coincidents with da2, 1 if it coincidents with da1, 0 otherwise
int RefInBounds(double da1, double da2, double dRef);
int RefInOpenBounds(PDRefPoint pBounds, double dRef);
int MergeBounds(double da1, double da2, double db1, double db2, bool bFullCycle, double *pdBnds);

bool PtInDblList(double du, int iSize, double *pdList);
// cStart, cEnd - x = 0 .. not set, x = 1 .. set, y is the reference
bool GetRefInUboundSeg(double dRef, CDPoint cStart, CDPoint cEnd);

// iSampleStrategy - 0 = constant, 1 = progressive
CDPoint GetCurveRefAtDist(void *pvData, double dr, CDPoint cBreak, double dDist,
  CurveFunc pFunc, CurveFunc pFuncDer, double dInterval, int iSampleStrategy, CDPoint cMaxRef);
double GetCurveDistAtRef(void *pvData, double dr, CDPoint cBreak, double dRef,
  CurveFunc pFunc, CurveFunc pFuncDer, double dInterval, int iSampleStrategy, CDPoint cMaxRef);
int AddCurveSegment(void *pvData, double dr, CDPoint cBreak, CurveFunc pFunc, CurveFunc pFuncDer,
  double dt1, double dt2, double dInterval, int iSampleStrategy, PDPrimObject pPrimList);
int AddCurveInterLine(void *pvData, double dr, CurveFunc pFunc, CurveFunc pFuncDer,
  PtProjFunc pFuncProj, CDPoint cTangent, CDPoint cStart, CDPoint cEnd,
  CDPoint cLn1, CDPoint cLn2, PDRefList pIntersects);

void RotatePrimitives(PDPrimObject pSrcList, PDPrimObject pDestList, CDPoint cOrig, CDPoint cMainDir);

CDPoint GetPrimRegion(CDPrimitive cPrim, double dLineWidth, double dScale,
  PDPoint pPoints1, PDPoint pPoints2);

#endif
