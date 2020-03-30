#ifndef _DLINE_HPP_
#define _DLINE_HPP_

#include "DDataTypes.hpp"

bool AddLinePoint(double x, double y, char iCtrl, PDPointList pPoints);
bool BuildLineCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache,
  double *pdMovedDist);
int GetLineBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDPointList pPoints,
  PDPointList pCache, PDLineStyle pStyle, PDRefList pBounds, double *pdMovedDist);
int BuildLinePrimitives(CDLine cTmpPt, int iMode, PDRect pRect, PDPointList pPoints,
  PDPointList pCache, PDPrimObject pPrimList, PDRefPoint pBounds, double dOffset,
  double *pdMovedDist, PDPoint pDrawBnds);
double GetLineDistFromPt(CDPoint cPt, PDPointList pCache, PDLine pPtX);
bool HasLineEnoughPoints(PDPointList pPoints);
bool GetLineRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pPoints);
double GetLineRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt);
bool GetLineAngle(PDPointList pCache, double *pdVal);
bool GetLinePointRefDist(double dRef, PDPointList pCache, double *pdDist);
//void AddLineSegment(double d1, double d2, PDPointList pCache, PDPrimObject pPrimList, PDRect pRect);
void AddLineSegment(double d1, double d2, PDPointList pCache, PDPrimObject pPrimList);
bool GetLineRefPoint(double dRef, PDPointList pCache, PDPoint pPt);
bool GetLineRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetLineReference(double dDist, PDPointList pCache, double *pdRef);

#endif
