#ifndef _DELLIPSE_HPP_
#define _DELLIPSE_HPP_

#include "DDataTypes.hpp"

bool AddEllipsePoint(double x, double y, char iCtrl, double dRestrictVal, PDPointList pPoints, int iInputLines);
bool BuildEllipseCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache,
  PDLine pLines, double *pdDist);
int AddEllipseInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds);
double GetElpsOffset(PDPointList pCache);
double GetElpsDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX);
int GetEllipseAttractors(CDPoint cPt, PDPointList pCache, PDPoint pPoints);
bool GetElpsRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt, PDPointList pCache);
bool HasElpsEnoughPoints(PDPointList pPoints, int iInputLines);
double GetElpsRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt,
  PDPointList pPoints, PDLine pLines);
bool GetElpsPointRefDist(double dRef, PDPointList pCache, double *pdDist);
void AddElpsSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList);
void AddElpsExtPrim(PDRect pRect, PDPointList pCache, PDPrimObject pPrimList);
bool GetElpsRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt);
bool GetElpsRefDir(double dRef, PDPointList pCache, PDPoint pPt);
bool GetElpsReference(double dDist, PDPointList pCache, double *pdRef);
int GetElpsSnapPoints(PDPointList pCache, double *pdRefs);

#endif
