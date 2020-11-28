#ifndef _DEVOLV_HPP_
#define _DEVOLV_HPP_

#include "DDataTypes.hpp"

bool AddEvolvPoint(double x, double y, char iCtrl, PDPointList pPoints, int iInputLines);
bool BuildEvolvCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache,
  PDLine pCircle, double *pdDist);
double GetEvolvOffset(PDPointList pCache);
int AddEvolvInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
/*int BuildEvolvPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, PDPointList pPoints,
    PDPointList pCache, PDPrimObject pPrimList, PDLine pCircle, PDRefPoint pBounds, double dOffset,
    double *pdDist, PDPoint pDrawBnds, bool bQuadsOnly = false);*/
double GetEvolvDistFromPt(CDPoint cPt, CDPoint cRefPt, PDPointList pCache, PDLine pPtX);
bool HasEvolvEnoughPoints(PDPointList pPoints, int iInputLines);
double GetEvolvRadiusAtPt(CDLine cPtX, PDPointList pCache, PDLine pPtR, bool bNewPt);
bool GetEvolvPointRefDist(double dRef, PDPointList pCache, double *pdDist);
//void AddEvolvSegment(double d1, double d2, PDPointList pCache, PDPrimObject pPrimList, PDRect pRect);
void AddEvolvSegment(double d1, double d2, double dExt, PDPointList pCache, PDPrimObject pPrimList);
bool GetEvolvRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt);
bool GetEvolvRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache);
bool GetEvolvRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetEvolvReference(double dDist, PDPointList pCache, double *pdRef);

#endif
