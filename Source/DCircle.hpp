#ifndef _DCIRCLE_HPP_
#define _DCIRCLE_HPP_

#include "DDataTypes.hpp"

bool AddCirclePoint(double x, double y, char iCtrl, PDPointList pPoints, PDLine pLines);
bool BuildCircCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache, PDLine pLines, double *pdMovedDist);
void UpdateCircleCache(PDPointList pPoints, PDPointList pCache);
int AddCircleInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
double GetCircDistFromPt(CDPoint cPt, CDPoint cRefPt, bool bSnapCenters, PDPointList pCache, PDLine pPtX);
bool HasCircEnoughPoints(PDPointList pPoints, int iInputLines);
bool GetCircleRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache, PDPointList pPoints, PDLine pLines);
double GetCircRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt);
bool GetCirceRad(PDPointList pCache, double *pdVal);
bool GetCircPointRefDist(double dRef, double dOffset, PDPointList pCache, double *pdDist);
void AddCircSegment(double dt1, double dt2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList);
void AddCircleExtPrim(PDRect pRect, PDPointList pCache, PDPrimObject pPrimList);
bool GetCircRefPoint(double dRef, double dOffset, PDPointList pCache, PDPoint pPt);
bool GetCircRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetCircReference(double dDist, double dOffset, PDPointList pCache, double *pdRef);

#endif
