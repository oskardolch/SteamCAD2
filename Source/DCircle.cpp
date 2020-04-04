#include "DCircle.hpp"
#include "DMath.hpp"
#include "DPrimitive.hpp"
#include <math.h>
#include <stdio.h>
#include <algorithm>

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

  if((iCtrl == 2) || (iCtrl == 3))
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
  if(dRad < g_dPrec) return false;

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

  if(dRad > g_dPrec)
  {
    pCache->AddPoint(cOrig.x, cOrig.y, 0);
    pCache->AddPoint(dRad, dRad, 0);

    if(fabs(dOffs) > g_dPrec) pCache->AddPoint(dOffs, 0, 2);
  }
  return true;
}

int GetCircleRectInterBnds(CDPoint cOrig, double dRad, PDRect pRect, PDPoint pRes)
{
  if(dRad < g_dPrec) return 0;

  int iCandidates = 0;
  double dCandidates[8];

  double dt, dt2, dy, dc = (pRect->cPt1.x - cOrig.x)/dRad;
  if(fabs(dc) < 1.0 - g_dPrec)
  {
    dt = acos(dc);
    dy = cOrig.y + dRad*sin(-dt);
    if((dy > pRect->cPt1.y + g_dPrec) && (dy < pRect->cPt2.y - g_dPrec)) dCandidates[iCandidates++] = -dt;
    dy = cOrig.y + dRad*sin(dt);
    if((dy > pRect->cPt1.y + g_dPrec) && (dy < pRect->cPt2.y - g_dPrec)) dCandidates[iCandidates++] = dt;
  }
  dc = (pRect->cPt2.x - cOrig.x)/dRad;
  if(fabs(dc) < 1.0 - g_dPrec)
  {
    dt = acos(dc);
    dy = cOrig.y + dRad*sin(-dt);
    if((dy > pRect->cPt1.y + g_dPrec) && (dy < pRect->cPt2.y - g_dPrec)) dCandidates[iCandidates++] = -dt;
    dy = cOrig.y + dRad*sin(dt);
    if((dy > pRect->cPt1.y + g_dPrec) && (dy < pRect->cPt2.y - g_dPrec)) dCandidates[iCandidates++] = dt;
  }
  dc = (pRect->cPt1.y - cOrig.y)/dRad;
  if(fabs(dc) < 1.0 - g_dPrec)
  {
    dt = asin(dc);
    if(dt < -g_dPrec) dt2 = -dt - M_PI;
    else dt2 = -dt + M_PI;
    dy = cOrig.x + dRad*cos(dt2);
    if((dy > pRect->cPt1.x + g_dPrec) && (dy < pRect->cPt2.x - g_dPrec)) dCandidates[iCandidates++] = dt2;
    dy = cOrig.x + dRad*cos(dt);
    if((dy > pRect->cPt1.x + g_dPrec) && (dy < pRect->cPt2.x - g_dPrec)) dCandidates[iCandidates++] = dt;
  }
  dc = (pRect->cPt2.y - cOrig.y)/dRad;
  if(fabs(dc) < 1.0 - g_dPrec)
  {
    dt = asin(dc);
    if(dt < -g_dPrec) dt2 = -dt - M_PI;
    else dt2 = -dt + M_PI;
    dy = cOrig.x + dRad*cos(dt2);
    if((dy > pRect->cPt1.x + g_dPrec) && (dy < pRect->cPt2.x - g_dPrec)) dCandidates[iCandidates++] = dt2;
    dy = cOrig.x + dRad*cos(dt);
    if((dy > pRect->cPt1.x + g_dPrec) && (dy < pRect->cPt2.x - g_dPrec)) dCandidates[iCandidates++] = dt;
  }

  CDPoint cTestPt;
  if(iCandidates < 1)
  {
    cTestPt.x = cOrig.x + dRad*sqrt(3.0)/2.0;
    cTestPt.y = cOrig.y + dRad/2.0;
    if(DPtInDRect(cTestPt, pRect))
    {
      pRes[0].x = -M_PI;
      pRes[0].y = M_PI;
      return -1;
    }
    return 0;
  }

  int iRes = iCandidates/2;

  std::sort(dCandidates, &dCandidates[iCandidates]);

  dt = (dCandidates[0] + dCandidates[1])/2.0;
  cTestPt.x = cOrig.x + dRad*cos(dt);
  cTestPt.y = cOrig.y + dRad*sin(dt);
  if(DPtInDRect(cTestPt, pRect))
  {
    for(int i = 0; i < iRes; i++)
    {
      pRes[i].x = dCandidates[2*i];
      pRes[i].y = dCandidates[2*i + 1];
    }
  }
  else
  {
    pRes[0].x = dCandidates[iCandidates - 1];
    pRes[0].y = dCandidates[0];
    for(int i = 1; i < iRes; i++)
    {
      pRes[i].x = dCandidates[2*i - 1];
      pRes[i].y = dCandidates[2*i];
    }
  }
  return iRes;
}

int GetCircleBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDPointList pPoints,
  PDPointList pCache, PDLine pLines, PDLineStyle pStyle, PDRefList pBounds, PDPoint pDrawBnds, double *pdMovedDist)
{
  if(iMode > 0) BuildCircCache(cTmpPt, iMode, pPoints, pCache, pLines, pdMovedDist);

  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return 0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;

  double dr = fabs(cRad.x);
  double d1 = dr*M_PI;
  pDrawBnds->x = -d1;
  pDrawBnds->y = d1;

  double dr1 = dr + pStyle->dPercent*pStyle->dWidth/200.0;
  double dExt = pStyle->dWidth/2.0;

  CDPoint pInt1[4], pInt2[4];
  int iInt1, iInt2;
  iInt1 = GetCircleRectInterBnds(cOrig, dr1 - dExt, pRect, pInt1);
  iInt2 = GetCircleRectInterBnds(cOrig, dr1 + dExt, pRect, pInt2);

  if((iInt1 < 0) || (iInt2 < 0))
  {
    pBounds->AddPoint(-d1);
    pBounds->AddPoint(d1);
    return 1;
  }

  CDPoint cCenter;
  if(iInt1 + iInt2 < 1)
  {
    cCenter = (pRect->cPt1 + pRect->cPt2)/2.0;
    d1 = GetDist(cCenter, cOrig) - dr1;
    if(fabs(d1) < dExt + g_dPrec)
    {
    }
  }

  //int iInt = 0;

  //pBounds->AddPoint(-d1);
  //pBounds->AddPoint(d1);
  return 1;
}

int BuildCircPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, PDPointList pPoints,
  PDPointList pCache, PDPrimObject pPrimList, PDLine pLines, PDRefPoint pBounds, double dOffset,
  double *pdMovedDist, PDPoint pDrawBnds)
{
  if(iMode > 0) BuildCircCache(cTmpPt, iMode, pPoints, pCache, pLines, pdMovedDist);

  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return 0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  cRad.x += dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;

  CDPrimitive cPrim, cPrimPt;

  cPrimPt.iType = 7;
  cPrimPt.cPt1.x = 1;
  cPrimPt.cPt1.y = 0;
  cPrimPt.cPt2 = cOrig;
  cPrimPt.cPt3 = 0;
  cPrimPt.cPt4 = 0;
  CropPrimitive(cPrimPt, pRect, pPrimList);

  double dr = fabs(cRad.x);

  pDrawBnds->y = M_PI*dr;
  pDrawBnds->x = -pDrawBnds->y;

  if(pBounds[0].bIsSet)
  {
    if(pBounds[1].bIsSet)
    {
      cPrim.iType = 2;
      cPrim.cPt1 = cOrig;
      cPrim.cPt2.x = cOrig.x + dr;
      cPrim.cPt2.y = cOrig.y + dr;
      GetCircRefPoint(pBounds[1].dRef, pCache, &cPrim.cPt3);
      GetCircRefPoint(pBounds[0].dRef, pCache, &cPrim.cPt4);
      return CropPrimitive(cPrim, pRect, pPrimList);
    }
  }

  cPrim.iType = 3;
  cPrim.cPt1 = cOrig;
  cPrim.cPt2.x = cOrig.x + dr;
  cPrim.cPt2.y = cOrig.y + dr;
  cPrim.cPt3 = 0;
  cPrim.cPt4 = 0;

  return CropPrimitive(cPrim, pRect, pPrimList);
}

void AddCircSegment(double d1, double d2, double dExt, PDPointList pCache, PDPrimObject pPrimList) //, PDRect pRect)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;
  if(cRad.x < g_dPrec) return;

  double dAng1, dAng2;
  dAng1 = d2/cRad.x;
  dAng2 = d1/cRad.x;

  CDPrimitive cPrim;
  cPrim.iType = 2;
  //if(fabs(fabs(dAng2 - dAng1) - 2*M_PI) < g_dPrec) cPrim.iType = 3;
  cPrim.cPt1 = cOrig;
  cPrim.cPt2.x = cRad.x + dExt;
  cPrim.cPt2.y = 0.0; //cOrig.y + cRad.x;
  if(cPrim.iType == 2)
  {
    cPrim.cPt3.x = dAng2;
    cPrim.cPt3.y = dAng1;
    cPrim.cPt4 = 0;
    /*cPrim.cPt3.x = cOrig.x + cRad.x*cos(dAng1);
    cPrim.cPt3.y = cOrig.y + cRad.x*sin(dAng1);
    cPrim.cPt4.x = cOrig.x + cRad.x*cos(dAng2);
    cPrim.cPt4.y = cOrig.y + cRad.x*sin(dAng2);*/
  }
  else
  {
    cPrim.cPt3 = 0;
    cPrim.cPt4 = 0;
  }
  pPrimList->AddPrimitive(cPrim);
  //CropPrimitive(cPrim, pRect, pPrimList);
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

  CDPoint cDir = cPt - cOrig;
  CDPoint cPt1, cPt2;

  double dr2 = GetNorm(cDir);
  double dRes = dr2 - cRad.x;

  double dDir = 1.0;
  if(dRes < 0) dDir = -1.0;

  double d1, d2, dMin;

  CDPoint cN1 = {1.0, 0.0};
  if(dr2 > g_dPrec) cN1 = cDir/dr2;

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

  double dr = cRad.x;
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

  *pdVal = cRad.x;
  return true;
}

bool GetCircPointRefDist(double dRef, PDPointList pCache, double *pdDist)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;

  *pdDist = cRad.x*dRef;
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

  double dAng = dDist/cRad.x;
  pPt->x = cOrig.x + cRad.x*cos(dAng);
  pPt->y = cOrig.y + cRad.x*sin(dAng);
  return true;
}

bool GetCircRefPoint(double dRef, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;
  pPt->x = cOrig.x + cRad.x*cos(dRef);
  pPt->y = cOrig.y + cRad.x*sin(dRef);
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

bool GetCircReference(double dDist, PDPointList pCache, double *pdRef)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) cRad.x += pCache->GetPoint(0, 2).cPoint.x;

  if(cRad.x < g_dPrec) return false;

  *pdRef = dDist/cRad.x;
  return true;
}

