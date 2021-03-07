#ifndef _DARCELPS_HPP_
#define _DARCELPS_HPP_

#include "DDataTypes.hpp"

bool AddArcElpsPoint(double x, double y, char iCtrl, PDPointList pPoints, int iInputLines);
bool BuildArcElpsCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache, PDLine pLines);
void UpdateArcElpsCache(PDPointList pCache);
int AddArcElpsInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
double GetArcElpsDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX);
bool HasArcElpsEnoughPoints(PDPointList pPoints, int iInputLines);
bool GetArcElpsRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache);
double GetArcElpsRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt,
  PDPointList pPoints, PDLine pLines);
bool GetArcElpsPointRefDist(double dRef, double dOffset, PDPointList pCache, double *pdDist);
void AddArcElpsSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList);
bool GetArcElpsRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt);
bool GetArcElpsRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetArcElpsReference(double dDist, double dOffset, PDPointList pCache, double *pdRef);
int GetArcElpsSnapPoints(PDPointList pCache, double *pdRefs);

#endif
