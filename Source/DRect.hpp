#ifndef _DRECT_HPP_
#define _DRECT_HPP_

#include "DDataTypes.hpp"

bool AddRectPoint(double x, double y, char iCtrl, PDPointList pPoints);
bool BuildRectCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache);
//void UpdateRectCache(CDLine cTmpPt, PDPointList pPoints, PDPointList pCache);
//int AddRectInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
//double GetRectDistFromPt(CDPoint cPt, PDPointList pCache, PDLine pPtX);
bool HasRectEnoughPoints(PDPointList pPoints);
bool GetRectRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pPoints);
//double GetLineRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt);
//bool GetLineAngle(PDPointList pCache, double *pdVal);
//bool GetLinePointRefDist(double dRef, PDPointList pCache, double *pdDist);
//void AddLineSegment(double dt1, double dt2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList);
//bool GetLineRefPoint(double dRef, double dOffset, PDPointList pCache, PDPoint pPt);
//bool GetLineRefDir(double dRef, PDPointList pCache, PDPoint pPt);
//bool GetLineReference(double dDist, PDPointList pCache, double *pdRef);

#endif
