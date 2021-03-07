#ifndef _DEVOLV_HPP_
#define _DEVOLV_HPP_

#include "DDataTypes.hpp"

bool AddEvolvPoint(double x, double y, char iCtrl, PDPointList pPoints, int iInputLines);
bool BuildEvolvCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache, PDLine pCircle);
int AddEvolvInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
double GetEvolvDistFromPt(CDPoint cPt, CDPoint cRefPt, PDPointList pCache, PDLine pPtX);
bool HasEvolvEnoughPoints(PDPointList pPoints, int iInputLines);
double GetEvolvRadiusAtPt(CDLine cPtX, PDPointList pCache, PDLine pPtR, bool bNewPt);
bool GetEvolvPointRefDist(double dRef, double dOffset, PDPointList pCache, double *pdDist);
void AddEvolvSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList);
bool GetEvolvRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt);
bool GetEvolvRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache);
bool GetEvolvRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetEvolvReference(double dDist, double dOffset, PDPointList pCache, double *pdRef);
int GetEvolvSnapPoints(PDPointList pCache, double *pdRefs);

#endif
