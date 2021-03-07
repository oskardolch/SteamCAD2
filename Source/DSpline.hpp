#ifndef _DSPLINE_HPP_
#define _DSPLINE_HPP_

#include "DDataTypes.hpp"

bool AddSplinePoint(double x, double y, char iCtrl, PDPointList pPoints);
bool BuildSplineCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache);
int GetSplineNumSegments(PDPointList pCache);
double GetSplineDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX);
int GetSplineAttractors(CDPoint cPt, PDPointList pCache, double dScale, PDPointList pPoints);
bool HasSplineEnoughPoints(PDPointList pPoints);
double GetSplineRadiusAtPt(CDLine cPtX, PDPointList pCache, PDLine pPtR, bool bNewPt);
bool GetSplinePointRefDist(double dRef, double dOffset, PDPointList pCache, double *pdDist);
void AddSplineSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList);
bool GetSplineRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt);
bool GetSplineRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache);
bool GetSplineRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetSplineReference(double dDist, double dOffset, PDPointList pCache, double *pdRef);
int AddSplineInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
int GetSplineSnapPoints(CDPoint cPt, double dDist, PDPointList pCache, PDRefList pRefs);

#endif
