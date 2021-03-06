#ifndef _DPARABOLA_HPP_
#define _DPARABOLA_HPP_

#include "DDataTypes.hpp"

bool AddParabPoint(double x, double y, char iCtrl, PDPointList pPoints, int iInputLines);
bool BuildParabCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache, PDLine pLines);
void UpdateParabCache(PDPointList pCache);
int AddParabInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
double GetParabDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX);
int GetParabAttractors(CDPoint cPt, PDPointList pCache, PDPoint pPoints);
bool HasParabEnoughPoints(PDPointList pPoints, int iInputLines);
double GetParabRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt);
bool GetParabPointRefDist(double dRef, double dOffset, PDPointList pCache, double *pdDist);
void AddParabSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList);
bool GetParabRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt);
bool GetParabRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache);
bool GetParabRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetParabReference(double dDist, double dOffset, PDPointList pCache, double *pdRef);
int GetParabSnapPoints(PDPointList pCache, double *pdRefs);

#endif
