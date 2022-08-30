#include "DRect.hpp"
#include "DMath.hpp"
#include "DPrimitive.hpp"
#include <math.h>
#include <stdio.h>
#include <algorithm>

bool AddRectPoint(double x, double y, char iCtrl, PDPointList pPoints)
{
  pPoints->AddPoint(x, y, 0);
  return (pPoints->GetCount(-1) == 2);
}

bool GetRectDimen(CDPoint cTmpPt, PDPointList pPoints, double *pdVals)
{
  if(pPoints->GetCount(-1) < 1) return false;
  CDPoint cPt1 = pPoints->GetPoint(0, -1).cPoint;
  pdVals[0] = fabs(cPt1.x - cTmpPt.x);
  pdVals[1] = fabs(cPt1.y - cTmpPt.y);
  return true;
}

bool HasRectEnoughPoints(PDPointList pPoints)
{
  return pPoints->GetCount(-1) > 1;
}

int GetRectRestrictPoint(CDPoint cPt, int iMode, int iRestrictMask, double *pdRestrictValue,
  PDPoint pSnapPt, PDPointList pPoints)
{
  int iCnt = pPoints->GetCount(-1);
  if(iCnt < 1) return 0;

  CDPoint cOrig = pPoints->GetPoint(0, -1).cPoint;
  if(iRestrictMask & 1) pSnapPt->x = cOrig.x + pdRestrictValue[0];
  else pSnapPt->x = cPt.x;
  if(iRestrictMask & 2) pSnapPt->y = cOrig.y - pdRestrictValue[1];
  else pSnapPt->y = cPt.y;

  return iRestrictMask;
}

