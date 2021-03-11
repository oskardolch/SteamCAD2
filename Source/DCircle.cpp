#include "DCircle.hpp"
#include "DMath.hpp"
#include "DPrimitive.hpp"
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <memory.h>

// for debugging purpose only
/*#include <windows.h>
#include <commctrl.h>
#include <wchar.h>
HWND g_hStatus;*/
// -----

bool GetCircOrigAndRad(PDPoint pTmpPt, PDPointList pPoints, PDLine pLines,
  PDPoint pOrig, double *pdRad)
{
  int nNorm = pPoints->GetCount(0);
  int nCtrl = pPoints->GetCount(1);

  CDPoint cOrig, cPt1, cPt2, cPt3, cDir1;
  double dr = -1.0, d1;

  int iPts = nNorm;
  if(pTmpPt) iPts++;
  if(nNorm > 0) cPt1 = pPoints->GetPoint(0, 0).cPoint;
  else if(pTmpPt) cPt1 = *pTmpPt;
  if(nNorm > 1) cPt2 = pPoints->GetPoint(1, 0).cPoint;
  else if(pTmpPt) cPt2 = *pTmpPt;
  if(nNorm > 2) cPt3 = pPoints->GetPoint(2, 0).cPoint;
  else if(pTmpPt) cPt3 = *pTmpPt;

  if(pLines[0].bIsSet)
  {
    if(iPts == 1)
    {
      dr = GetLineProj(cPt1, pLines[0].cOrigin, pLines[0].cDirection, &cOrig);
    }
    else if(iPts > 1)
    {
      cDir1 = cPt2 - cPt1;
      d1 = GetNorm(cDir1);
      if(d1 < g_dPrec) return false;

      cDir1 = GetNormal(cDir1)/d1;
      cPt3 = (cPt2 + cPt1)/2.0;
      int iX = LineXLine(cPt3, cDir1, pLines[0].cOrigin, pLines[0].cDirection, &cOrig);
      if(iX < 1) return false;

      dr = GetDist(cPt1, cOrig);
    }
    else return false;
  }
  else if(nCtrl > 0)
  {
    cOrig = pPoints->GetPoint(0, 1).cPoint;
    if(iPts > 0) dr = GetDist(cOrig, cPt1);
    else return false;
  }
  else if(iPts > 2)
  {
    dr = GetCircOrigin(cPt1, cPt2, cPt3, &cOrig);
  }
  else if(iPts > 1)
  {
    cOrig = (cPt1 + cPt2)/2.0;
    dr = GetDist(cOrig, cPt1);
  }
  else return false;

  *pOrig = cOrig;
  *pdRad = dr;
  return true;
}

bool AddCirclePoint(double x, double y, char iCtrl, PDPointList pPoints, PDLine pLines)
{
  int nNorm = pPoints->GetCount(0);

  if(iCtrl > 1)
  {
    CDPoint cOrig;
    double dRad;
    if(!GetCircOrigAndRad(NULL, pPoints, pLines, &cOrig, &dRad)) return false;
    if(dRad < g_dPrec) return false;

    CDPoint cPt1 = {x, y};
    double dNewRad = GetDist(cOrig, cPt1);
    if(dNewRad < g_dPrec) return false;

    CDInputPoint cInPt1;
    CDPoint cDir;
    for(int i = 0; i < nNorm; i++)
    {
      cInPt1 = pPoints->GetPoint(i, 0);
      cDir = cInPt1.cPoint - cOrig;
      cPt1 = cOrig + dNewRad*cDir/dRad;
      pPoints->SetPoint(i, 0, cPt1.x, cPt1.y, 0);
    }
    return true;
  }

  int nCtrl = pPoints->GetCount(1);
  if((nCtrl == 1) && (nNorm == 1)) return true;

  bool bRes = false;

  if(pLines[0].bIsSet)
  {
    if(iCtrl == 0)
    {
      if(nNorm < 2)
      {
        pPoints->AddPoint(x, y, 0);
        nNorm++;
      }
      bRes = (nNorm > 1);
    }
  }
  else
  {
    if(iCtrl == 1)
    {
      if(nCtrl < 1)
      {
        pPoints->AddPoint(x, y, 1);
        nCtrl++;
      }
    }
    else
    {
      if(nNorm < 3)
      {
        pPoints->AddPoint(x, y, 0);
        nNorm++;
      }
    }
    bRes = ((nCtrl > 0) && (nNorm > 0)) || (nNorm > 2);
  }
  return bRes;
}

bool BuildCircCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache,
  PDLine pLines, double *pdMovedDist)
{
  pCache->ClearAll();

  bool bFound = false;
  CDPoint cOrig;
  double dRad;
  double dOffs = 0.0;

  if(iMode == 1)
    bFound = GetCircOrigAndRad(&cTmpPt.cOrigin, pPoints, pLines, &cOrig, &dRad);
  else bFound = GetCircOrigAndRad(NULL, pPoints, pLines, &cOrig, &dRad);

  if(!bFound) return false;

  if(iMode == 2)
  {
    if(cTmpPt.cDirection.x > 0.5) dOffs = cTmpPt.cDirection.y;
    else
    {
      dOffs = GetDist(cTmpPt.cOrigin, cOrig);
      if(cTmpPt.cDirection.x < -0.5)
      {
        dOffs += dRad;
        dOffs *= -1.0;
      }
      else dOffs -= dRad;
    }
    *pdMovedDist = dOffs;
  }

  pCache->AddPoint(cOrig.x, cOrig.y, 0);
  pCache->AddPoint(dRad, dRad, 0);

  if(fabs(dOffs) > g_dPrec) pCache->AddPoint(dOffs, 0, 2);
  return true;
}

int AddCircleInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return 0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;

  double dr = fabs(cRad.x) + dOffset;
  CDPoint cRes;
  int iRes = CircXSegParams(cOrig, dr, cPt1, cPt2, &cRes);
  if(iRes > 0) pBounds->AddPoint(cRes.x);
  if(iRes > 1) pBounds->AddPoint(cRes.y);
  return iRes;
}

void AddCircSegment(double dt1, double dt2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;
  cRad.x += dExt;
  double dr = fabs(cRad.x);
  if(dr < g_dPrec) return;

  CDPrimitive cPrim;
  cPrim.iType = 2;
  cPrim.cPt1 = cOrig;
  cPrim.cPt2.x = dr;
  cPrim.cPt2.y = 0.0;
  if(cRad.x < 0.0)
  {
    cPrim.cPt3.x = OpositeAngle(dt1);
    cPrim.cPt3.y = OpositeAngle(dt2);
  }
  else
  {
    cPrim.cPt3.x = dt1;
    cPrim.cPt3.y = dt2;
  }
  cPrim.cPt4 = 0;
  if(bReverse) cPrim.cPt4.x = 1.0;
  pPrimList->AddPrimitive(cPrim);
}

void AddCircleExtPrim(PDRect pRect, PDPointList pCache, PDPrimObject pPrimList)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  //CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPrimitive cPrimPt;

  cPrimPt.iType = 7;
  cPrimPt.cPt1.x = 1;
  cPrimPt.cPt1.y = 0;
  cPrimPt.cPt2 = cOrig;
  cPrimPt.cPt3 = 0;
  cPrimPt.cPt4 = 0;
  CropPoints(cPrimPt, pRect, pPrimList);
}

double GetCircDistFromPt(CDPoint cPt, CDPoint cRefPt, bool bSnapCenters, PDPointList pCache, PDLine pPtX)
{
  pPtX->bIsSet = false;

  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return 0.0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cN1 = {1.0, 0.0};
  CDPoint cDir = cPt - cOrig;
  double dr2 = GetNorm(cDir);
  if(dr2 < g_dPrec)
  {
    pPtX->bIsSet = true;
    if(bSnapCenters)
    {
      pPtX->cOrigin = cOrig;
      pPtX->cDirection = 0;
      pPtX->dRef = 0.0;
      return 0.0;
    }
    pPtX->cOrigin = cRad.x*cN1;
    pPtX->cDirection = 0;
    pPtX->dRef = 0.0;
    return -cRad.x;
  }

  CDPoint cPt1, cPt2;

  double dr = fabs(cRad.x);
  double dRes = dr2 - dr;

  if(dr < g_dPrec)
  {
    pPtX->bIsSet = true;
    pPtX->cOrigin = cOrig;
    pPtX->cDirection = 0;
    return dRes;
  }

  double dDir = 1.0;
  if(dRes < 0) dDir = -1.0;

  double d1, d2, dMin;

  cN1 = cDir/dr2;
  cPt1 = cOrig + cRad.x*cN1;
  cPt2 = cOrig - cRad.x*cN1;
  d1 = GetDist(cPt1, cRefPt);
  d2 = GetDist(cPt2, cRefPt);

  pPtX->bIsSet = true;
  if(d1 < d2)
  {
    dMin = d1;
    dRes = dDir*d1;
    pPtX->cOrigin = cPt1;
    pPtX->cDirection = cN1;
  }
  else
  {
    dMin = d2;
    dRes = dDir*d2;
    pPtX->cOrigin = cPt2;
    pPtX->cDirection = -1.0*cN1;
  }
  pPtX->dRef = atan2(pPtX->cDirection.y, pPtX->cDirection.x);

  if(bSnapCenters && (dr2 < dMin))
  {
    dRes = dr2;
    pPtX->bIsSet = true;
    pPtX->cOrigin = cOrig;
    pPtX->cDirection = 0;
  }

  return dRes;
}

bool HasCircEnoughPoints(PDPointList pPoints, int iInputLines)
{
  int nNorm = pPoints->GetCount(0);
  int nCtrl = pPoints->GetCount(1);
  bool bRes = false;

  if(iInputLines > 0) bRes = (nNorm > 0);
  else if(nCtrl > 0) bRes = (nNorm > 0);
  else bRes = (nNorm > 1);

  return bRes;
}

bool GetCircleRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache, PDPointList pPoints, PDLine pLines)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 1) return false;

  double dRad = dRestrictValue;
  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;

  if(iMode == 2)
  {
    if(iCnt < 2) return false;
    CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
    dRad += cRad.x;
  }

  if(pLines[0].bIsSet && (iMode != 2))
  {
    int iCnt1 = pPoints->GetCount(0);
    if(iCnt1 < 1) return false;

    CDPoint cPt1 = pPoints->GetPoint(0, 0).cPoint;
    CDPoint cPt2 = pLines[0].cOrigin - cPt1;
    CDPoint cDir = pLines[0].cDirection;

    double dPoly[3];
    dPoly[0] = cPt2*cPt2 - Power2(dRad);
    dPoly[1] = 2.0*cDir*cPt2;
    dPoly[2] = cDir*cDir;

    double dRoots[2];
    int iRoots = SolvePolynom(2, dPoly, dRoots);

    if(iRoots < 1) return false;

    CDPoint cOrig1 = pLines[0].cOrigin + dRoots[0]*cDir;
    CDPoint cDir1 = cPt - cOrig1;
    double dNorm1 = GetNorm(cDir1);

    if(iRoots < 2)
    {
      if(dNorm1 > g_dPrec)
      {
        *pSnapPt = cOrig1 + dRad*cDir1/dNorm1;
        return true;
      }
      return false;
    }

    CDPoint cOrig2 = pLines[0].cOrigin + dRoots[1]*cDir;
    CDPoint cDir2 = cPt - cOrig2;
    double dNorm2 = GetNorm(cDir2);

    if(dNorm1 > g_dPrec)
    {
      cPt1 = cOrig1 + dRad*cDir1/dNorm1;
      if(dNorm2 > g_dPrec)
      {
        cPt2 = cOrig2 + dRad*cDir2/dNorm2;
        double d1 = GetDist(cPt, cPt1);
        double d2 = GetDist(cPt, cPt2);
        if(d1 < d2)
        {
          *pSnapPt = cPt1;
          return true;
        }

        *pSnapPt = cPt2;
        return true;
      }
      else
      {
        *pSnapPt = cPt1;
        return true;
      }
    }
    else if(dNorm2 > g_dPrec)
    {
      cPt2 = cOrig2 + dRad*cDir2/dNorm2;
      *pSnapPt = cPt2;
      return true;
    }

    return false;
  }

  CDPoint cDir = cPt - cOrig;
  double dNorm = GetNorm(cDir);

  if(dNorm < g_dPrec)
  {
    pSnapPt->x = dRad;
    pSnapPt->y = 0.0;
    return true;
  }

  cDir /= dNorm;
  *pSnapPt = cOrig + dRad*cDir;
  return true;
}

double GetCircRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt)
{
  pPtR->bIsSet = false;
  pPtR->cOrigin = 0;
  pPtR->cDirection = 0;

  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return -1.0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cDir = cPt - cOrig;
  double dNorm = GetNorm(cDir);

  pPtR->bIsSet = true;
  pPtR->cOrigin = cOrig;
  if(dNorm > g_dPrec) pPtR->cDirection = cDir/dNorm;

  double dr = fabs(cRad.x);
  if(bNewPt) dr = dNorm;

  return dr;
}

bool GetCirceRad(PDPointList pCache, double *pdVal)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;

  *pdVal = fabs(cRad.x);
  return true;
}

bool GetCircPointRefDist(double dRef, double dOffset, PDPointList pCache, double *pdDist)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;
  cRad.x += dOffset;
  double dr = fabs(cRad.x);
  if(dr < g_dPrec) return false;

  *pdDist = dr*dRef;
  return true;
}

bool GetCircPointAtDist(double dDist, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;

  double dr = fabs(cRad.x);
  if(dr < g_dPrec) return false;

  double dAng = dDist/dr;
  pPt->x = cOrig.x + cRad.x*cos(dAng);
  pPt->y = cOrig.y + cRad.x*sin(dAng);
  return true;
}

bool GetCircRefPoint(double dRef, double dOffset, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;
  cRad.x += dOffset;
  double dr = fabs(cRad.x);
  if(dr < g_dPrec) return false;
  if(cRad.x < 0.0) dRef = OpositeAngle(dRef);
  pPt->x = cOrig.x + dr*cos(dRef);
  pPt->y = cOrig.y + dr*sin(dRef);
  return true;
}

bool GetCircRefDir(double dRef, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  pPt->x = -sin(dRef);
  pPt->y = cos(dRef);
  return true;
}

bool GetCircReference(double dDist, double dOffset, PDPointList pCache, double *pdRef)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;
  cRad.x += dOffset;

  double dr = fabs(cRad.x);
  if(dr < g_dPrec) return false;

  *pdRef = dDist/dr;
  return true;
}

