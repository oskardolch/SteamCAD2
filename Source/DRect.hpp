#ifndef _DRECT_HPP_
#define _DRECT_HPP_

#include "DDataTypes.hpp"

bool AddRectPoint(double x, double y, char iCtrl, PDPointList pPoints);
bool GetRectDimen(CDPoint cTmpPt, PDPointList pPoints, double *pdVals);
bool HasRectEnoughPoints(PDPointList pPoints);
int GetRectRestrictPoint(CDPoint cPt, int iMode, int iRestrictMask, double *pdRestrictValue,
  PDPoint pSnapPt, PDPointList pPoints);

#endif
