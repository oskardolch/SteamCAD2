#include "DLine.hpp"
#include "DMath.hpp"
#include "DPrimitive.hpp"
#include <math.h>
#include <stdio.h>
#include <algorithm>

bool AddLinePoint(double x, double y, char iCtrl, PDPointList pPoints)
{
  if(iCtrl > 1)
  {
    if(pPoints->GetCount(-1) != 2) return false;

    CDInputPoint cInPt1 = pPoints->GetPoint(0, -1);
    CDPoint cPt1 = cInPt1.cPoint;
    CDPoint cPt2 = pPoints->GetPoint(1, -1).cPoint;
    CDPoint cOrig = cPt1;
    CDPoint cDir = cPt2 - cPt1;
    double dNorm = GetNorm(cDir);
    if(dNorm < g_dPrec) return false;

    cDir /= dNorm;
    if(cInPt1.iCtrl == 1)
    {
      cOrig = (cPt1 + cPt2)/2;
      cDir = GetNormal(cDir);
    }

    CDPoint cPt3 = {x - cOrig.x, y - cOrig.y};
    CDPoint cPt4 = Rotate(cPt3, cDir, false);
    cPt4.x = 0.0;
    cPt3 = Rotate(cPt4, cDir, true);
    cPt1 += cPt3;
    cPt2 += cPt3;
    pPoints->SetPoint(0, -1, cPt1.x, cPt1.y, cInPt1.iCtrl);
    pPoints->SetPoint(1, -1, cPt2.x, cPt2.y, 0);
    return true;
  }

  pPoints->AddPoint(x, y, iCtrl);
  return (pPoints->GetCount(-1) == 2);
}

bool BuildLineCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache, double *pdMovedDist)
{
    pCache->ClearAll();

    int iPts = pPoints->GetCount(-1);
    if(iPts < 1) return false;
    if((iPts < 2) && (iMode != 1)) return false;

    CDInputPoint cInPt1 = pPoints->GetPoint(0, -1);
    CDPoint cPt1, cPt2;

    if((iMode == 1) && cTmpPt.bIsSet) cPt1 = cTmpPt.cDirection;
    else cPt1 = cInPt1.cPoint;
    if(iMode == 1) cPt2 = cTmpPt.cOrigin;
    else cPt2 = pPoints->GetPoint(1, -1).cPoint;

    CDPoint cOrig = cPt1;
    CDPoint cDir = cPt2 - cPt1;
    double dNorm = GetNorm(cDir);
    if(dNorm < g_dPrec) return false;

    cDir /= dNorm;
    if(cInPt1.iCtrl == 1)
    {
        cOrig = (cPt1 + cPt2)/2.0;
        cDir = GetNormal(cDir);
    }

    pCache->AddPoint(cOrig.x, cOrig.y, 0);
    pCache->AddPoint(cDir.x, cDir.y, 0);

    if(iMode == 2)
    {
        cPt2 = Rotate(cTmpPt.cOrigin - cOrig, cDir, false);
        if(pdMovedDist) *pdMovedDist = fabs(cPt2.y);
        pCache->AddPoint(cPt2.y, 0.0, 2);
    }
    return true;
}

int AddLineInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return 0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cNorm = pCache->GetPoint(1, 0).cPoint;

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cOrigOff;
  cOrigOff.x = cOrig.x - dr*cNorm.y;
  cOrigOff.y = cOrig.y + dr*cNorm.x;

  CDPoint cPt3 = Rotate(cPt1 - cOrigOff, cNorm, false);
  CDPoint cPt4 = Rotate(cPt2 - cOrigOff, cNorm, false);
  if(cPt3.y*cPt4.y > -g_dPrec) return 0;

  double dt = cPt3.y/(cPt3.y - cPt4.y);
  double dx = (1.0 - dt)*cPt3.x + dt*cPt4.x;
  pBounds->AddPoint(dx);
  return 1;
}

void AddLineSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cNorm = pCache->GetPoint(1, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0)
  {
    dr = pCache->GetPoint(0, 2).cPoint.x;
    CDPoint cPtOff;
    cPtOff.x = -dr*cNorm.y;
    cPtOff.y = dr*cNorm.x;
    cOrig += cPtOff;
  }
  cOrig.x -= dExt*cNorm.y;
  cOrig.y += dExt*cNorm.x;

  CDPoint cPt1 = {d1, 0.0};
  CDPoint cPt2 = {d2, 0.0};

  CDPrimitive cPrim;
  cPrim.iType = 1;
  if(bReverse)
  {
    cPrim.cPt1 = cOrig + Rotate(cPt2, cNorm, true);
    cPrim.cPt2 = cOrig + Rotate(cPt1, cNorm, true);
  }
  else
  {
    cPrim.cPt1 = cOrig + Rotate(cPt1, cNorm, true);
    cPrim.cPt2 = cOrig + Rotate(cPt2, cNorm, true);
  }
  pPrimList->AddPrimitive(cPrim);
}

double GetLineDistFromPt(CDPoint cPt, PDPointList pCache, PDLine pPtX)
{
  pPtX->bIsSet = false;

  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return 0.0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cNorm = pCache->GetPoint(1, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0)
  {
    dr = pCache->GetPoint(0, 2).cPoint.x;
    CDPoint cPtOff;
    cPtOff.x = -dr*cNorm.y;
    cPtOff.y = dr*cNorm.x;
    cOrig += cPtOff;
  }

  CDPoint cPt1, cPt2;
  cPt1 = Rotate(cPt - cOrig, cNorm, false);

  cPt2.x = cPt1.x;
  cPt2.y = 0.0;
  pPtX->bIsSet = true;
  pPtX->cOrigin = cOrig + Rotate(cPt2, cNorm, true);
  pPtX->cDirection = -1.0*GetNormal(cNorm);
  pPtX->dRef = cPt1.x;

  return cPt1.y;
}

bool HasLineEnoughPoints(PDPointList pPoints)
{
  return pPoints->GetCount(-1) > 1;
}

bool GetLineRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
    PDPointList pCache)
{
  int iCnt = pCache->GetCount(0);

  if(iMode == 2)
  {
    if(iCnt != 2) return false;

    CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
    CDPoint cMainDir = pCache->GetPoint(1, 0).cPoint;

    CDPoint cPt1 = Rotate(cPt - cOrig, cMainDir, false);

    CDPoint cPt2;
    cPt2.x = cPt1.x;
    cPt2.y = dRestrictValue;
    if(cPt1.y < 0) cPt2.y = -dRestrictValue;

    *pSnapPt = cOrig + Rotate(cPt2, cMainDir, true);
    return true;
  }

  if(iCnt < 1) return false;

  CDPoint cN1 = {cos(dRestrictValue), -sin(dRestrictValue)};
  CDInputPoint cInPt = pCache->GetPoint(0, -1);
  GetLineProj(cPt, cInPt.cPoint, cN1, pSnapPt);
  return true;
}

double GetLineRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt)
{
  pPtR->bIsSet = false;
  pPtR->cOrigin = 0;
  pPtR->cDirection = 0;

  int iCnt = pCache->GetCount(-1);

  if(iCnt < 1) return -1.0;
  if(!bNewPt && (iCnt < 2)) return -1.0;

  CDPoint cNorm = {0, 0};
  if(bNewPt)
  {
    CDPoint cDir = cPt - pCache->GetPoint(0, -1).cPoint;
    double d1 = GetNorm(cDir);
    if(d1 > g_dPrec) cNorm = cDir/d1;
  }
  else cNorm = pCache->GetPoint(1, -1).cPoint;

  pPtR->cDirection = GetNormal(cNorm);

  return -1.0;
}

bool GetLineAngle(PDPointList pCache, double *pdVal)
{
  int iCnt = pCache->GetCount(-1);
  if(iCnt < 2) return false;

  CDPoint cNorm = pCache->GetPoint(1, -1).cPoint;
  double dAng1 = atan2(cNorm.y, cNorm.x);
  if((fabs(dAng1) > g_dPrec) && (fabs(dAng1) < M_PI - g_dPrec)) dAng1 *= -1.0;
  *pdVal = dAng1;
  return true;
}

bool GetLinePointRefDist(double dRef, PDPointList pCache, double *pdDist)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  *pdDist = dRef;
  return true;
}

bool GetLineReference(double dDist, PDPointList pCache, double *pdRef)
{
  *pdRef = dDist;
  return true;
}

bool GetLineRefPoint(double dRef, double dOffset, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cNorm = pCache->GetPoint(1, 0).cPoint;

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cPt1 = {dRef, dr};
  *pPt = cOrig + Rotate(cPt1, cNorm, true);
  return true;
}

bool GetLineRefDir(double dRef, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  *pPt = pCache->GetPoint(1, 0).cPoint;
  return true;
}

