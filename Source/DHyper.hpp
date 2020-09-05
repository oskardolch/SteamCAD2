#ifndef _DHYPER_HPP_
#define _DHYPER_HPP_

#include "DDataTypes.hpp"

bool AddHyperPoint(double x, double y, char iCtrl, double dRestrictVal, PDPointList pPoints, int iInputLines);
bool BuildHyperCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache,
  PDLine pLines, double *pdDist);
int AddHyperInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
double GetHyperOffset(PDPointList pCache);
double GetHyperDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX);
int GetHyperAttractors(CDPoint cPt, PDPointList pCache, PDPoint pPoints);
bool HasHyperEnoughPoints(PDPointList pPoints, int iInputLines);
double GetHyperRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt);
bool GetHyperPointRefDist(double dRef, PDPointList pCache, double *pdDist);
void AddHyperSegment(double d1, double d2, double dExt, PDPointList pCache, PDPrimObject pPrimList);
bool GetHyperRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt);
bool GetHyperRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache);
bool GetHyperRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetHyperReference(double dDist, PDPointList pCache, double *pdRef);
int GetHyperNumParts(PDPointList pCache, PDRefPoint pBounds);
bool HyperRemovePart(bool bDown, PDPointList pCache, PDRefPoint pBounds);

#endif
