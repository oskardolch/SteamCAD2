#ifndef _DHYPER_HPP_
#define _DHYPER_HPP_

#include "DDataTypes.hpp"

bool AddHyperPoint(double x, double y, char iCtrl, PDPointList pPoints, int iInputLines);
bool BuildHyperCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache, PDLine pLines);
void UpdateHyperCache(PDPointList pCache);
int AddHyperInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
double GetHyperDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX);
int GetHyperAttractors(CDPoint cPt, PDPointList pCache, PDPoint pPoints);
bool HasHyperEnoughPoints(PDPointList pPoints, int iInputLines);
double GetHyperRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt);
bool GetHyperPointRefDist(double dRef, PDPointList pCache, double *pdDist);
void AddHyperSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList);
bool GetHyperRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt);
bool GetHyperRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache);
bool GetHyperRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetHyperReference(double dDist, PDPointList pCache, double *pdRef);
int GetHyperSnapPoints(PDPointList pCache, double *pdRefs);

#endif
