#include "DParabola.hpp"
#include "DMath.hpp"
#include <math.h>
#include <stdio.h>
#include "DPrimitive.hpp"

// for debugging purpose only
/*#include <windows.h>
#include <commctrl.h>
#include <wchar.h>
extern HWND g_hStatus;
extern HANDLE g_hConsole;*/
// -----

bool AddParabPoint(double x, double y, char iCtrl, double dRestrictVal, PDPointList pPoints, int iInputLines)
{
  if((iCtrl == 2) || (iCtrl == 3) || (iCtrl == 4))
  {
    CDPoint cNewPt = {x, y};
    if(iCtrl == 4)
    {
      cNewPt.x = dRestrictVal;
      cNewPt.y = 0.0;
    }

    int nOffs2 = pPoints->GetCount(2);
    int nOffs3 = pPoints->GetCount(3);
    int nOffs4 = pPoints->GetCount(4);
    if(nOffs2 > 0) pPoints->SetPoint(0, 2, cNewPt.x, cNewPt.y, iCtrl);
    else if(nOffs3 > 0) pPoints->SetPoint(0, 3, cNewPt.x, cNewPt.y, iCtrl);
    else if(nOffs4 > 0) pPoints->SetPoint(0, 4, cNewPt.x, cNewPt.y, iCtrl);
    else pPoints->AddPoint(cNewPt.x, cNewPt.y, iCtrl);
    return true;
  }

  bool bRes = false;
  int nNorm = pPoints->GetCount(0);

  if(iInputLines == 1)
  {
    if((iCtrl < 1) && (nNorm < 1))
    {
      pPoints->AddPoint(x, y, iCtrl);
      nNorm++;
    }
    bRes = (nNorm > 0);
  }
  return bRes;
}

double GetParabBreakAngle(double dr, double da, double dr1)
{
  double dRes = -1.0;
  if(dr > dr1 - g_dPrec)
  {
    if(dr < dr1 + g_dPrec) dRes = 0.0;
    else
    {
      double d1 = 2.0*da;
      double d2 = cbrt(Power2(d1*dr));
      dRes = sqrt(d2 - 1.0)/d1;
    }
  }
  return dRes;
}

bool BuildParabCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache,
  PDLine pLines, double *pdDist)
{
  pCache->ClearAll();

  int nNorm = pPoints->GetCount(0);

  CDInputPoint cInPt1;
  CDPoint cOrig, cMainDir, cPt1, cPt2, cPt3;
  double da;
  double dr1 = -1.0;

  if(!pLines[0].bIsSet) return false;

  if(!(((nNorm < 1) && (iMode == 1)) || ((nNorm == 1) && (iMode != 1)))) return false;

  if(iMode == 1) cPt1 = cTmpPt.cOrigin;
  else if(nNorm > 0)
  {
    cInPt1 = pPoints->GetPoint(0, 0);
    cPt1 = cInPt1.cPoint;
  }
  else return false;

  cMainDir = pLines[0].cDirection;
  dr1 = GetLineProj(cPt1, pLines[0].cOrigin, cMainDir, &cPt2)/2.0;
  cPt3 = Rotate(cPt1 - cPt2, cMainDir, false);
  if(cPt3.y < 0)
  {
    cMainDir = -1.0*pLines[0].cDirection;
    cPt3 = Rotate(cPt1 - cPt2, cMainDir, false);
  }

  cOrig = (cPt1 + cPt2)/2.0;
  da = 1.0/dr1/4.0;

  pCache->AddPoint(cOrig.x, cOrig.y, 0);
  pCache->AddPoint(da, 0.0, 0);
  pCache->AddPoint(cMainDir.x, cMainDir.y, 0);

  dr1 = 0.5/da;
  pCache->AddPoint(0.0, dr1, 3);
  double dr;

  //if((iMode == 2) && (cTmpPt.cDirection.x > 0.5))
  //{
  //  dr = GetParabBreakAngle(-cTmpPt.cDirection.y, da, dr1);
  //  if(dr > -0.5) pCache->AddPoint(dr, 0.0, 4);
  //
  //  if(pdDist) *pdDist = cTmpPt.cDirection.y;
  //  pCache->AddPoint(cTmpPt.cDirection.y, 0.0, 2);
  //  return true;
  //}

  int nOffs2 = pPoints->GetCount(2);
  int nOffs3 = pPoints->GetCount(3);
  int nOffs4 = pPoints->GetCount(4);
  if((iMode == 2) || (nOffs2 > 0) || (nOffs3 > 0) || (nOffs4 > 0))
  {
    CDLine cPtX;
    int iSrchMask = 0;
    double dDist = 0.0;
    double dDistOld = 0.0;

    if(iMode == 2)
    {
      cPt1 = cTmpPt.cOrigin;
      if(cTmpPt.cDirection.x < -0.5) iSrchMask = 2;
    }
    else if(nOffs2 > 0) cPt1 = pPoints->GetPoint(0, 2).cPoint;
    else
    {
      cPt1 = pPoints->GetPoint(0, 3).cPoint;
      iSrchMask = 2;
    }

    if((iMode == 2) || (nOffs4 == 0))
      dDist = GetParabDistFromPt(cPt1, cPt1, iSrchMask, pCache, &cPtX);

    if(iMode == 2)
    {
      if(nOffs4 > 0)
        dDistOld = pPoints->GetPoint(0, 4).cPoint.x;
      else if(nOffs2 > 0)
      {
        cPt1 = pPoints->GetPoint(0, 2).cPoint;
        dDistOld = GetParabDistFromPt(cPt1, cPt1, 0, pCache, &cPtX);
      }
      else if(nOffs3 > 0)
      {
        cPt1 = pPoints->GetPoint(0, 3).cPoint;
        dDistOld = GetParabDistFromPt(cPt1, cPt1, 2, pCache, &cPtX);
      }
      if(cTmpPt.cDirection.x > 0.5) dDist = dDistOld + cTmpPt.cDirection.y;
    }
    else if(nOffs4 > 0) dDist = pPoints->GetPoint(0, 4).cPoint.x;

    if(pdDist) *pdDist = dDist - dDistOld;
    if((fabs(dDist) > g_dPrec) || (fabs(dDistOld) > g_dPrec)) pCache->AddPoint(dDist, dDistOld, 2);

    dr = GetParabBreakAngle(-dDist, da, dr1);
    if(dr > -0.5) pCache->AddPoint(dr, 0.0, 4);
  }
  return true;
}

CDPoint GetParabPointDir(double da, double dOffset, double dx, PDPoint pDir)
{
  CDPoint cRes, cDir;
  cRes.x = dx;
  cRes.y = da*Power2(dx);
  cDir.x = 2.0*da*dx;
  cDir.y = -1.0;
  double dNorm = GetNorm(cDir);
  cDir /= dNorm;
  pDir->x = -cDir.y;
  pDir->y = cDir.x;
  return cRes + dOffset*cDir;
}

CDPoint ParabFunc(void *pvData, double dt)
{
  double da = *(double*)pvData;
  return {dt, da*Power2(dt)};
}

CDPoint ParabFuncDer(void *pvData, double dt)
{
  double da = *(double*)pvData;
  return {1.0, 2.0*da*dt};
}

double ParProjFn(double da, double dx, double dy, double du)
{
  return 2.0*Power2(da)*Power3(du) + (1.0 - 2.0*da*dy)*du - dx;
}

double ParProjFnDer(double da, double dx, double dy, double du)
{
  return 6.0*Power2(da*du) + (1.0 - 2.0*da*dy);
}

bool GetParabPtProjFromU(double da, double dStart, CDPoint cPt, double *pdRes)
{
  int j = 0;
  double du1 = dStart;
  double df = ParProjFn(da, cPt.x, cPt.y, du1);
  double df2 = ParProjFnDer(da, cPt.x, cPt.y, du1);
  if(fabs(df2) < g_dPrec) return 0.0;

  double du2 = du1 - df/df2;

  while((j < 8) && (fabs(df) > g_dRootPrec))
  {
    j++;
    du1 = du2;
    df = ParProjFn(da, cPt.x, cPt.y, du1);
    df2 = ParProjFnDer(da, cPt.x, cPt.y, du1);
    if(fabs(df2) > g_dPrec) du2 -= df/df2;
    else j = 8;
  }
  *pdRes = du2;
  return fabs(df) < g_dRootPrec;
}

int GetParabPtProj(double da, CDPoint cPt, double *pdRoots)
{
  double dPoly[4];
  dPoly[0] = -cPt.x;
  dPoly[1] = 1.0 - 2.0*da*cPt.y;
  dPoly[2] = 0.0;
  dPoly[3] = 2.0*Power2(da);

  double dRoots[3];
  int iRoots = SolvePolynom(3, dPoly, dRoots);

  if(iRoots < 1)
  {
    if(GetParabPtProjFromU(da, 0.0, cPt, pdRoots)) return 1;
    return 0;
  }

  int iRes = 0;
  double dx;
  for(int j = 0; j < iRoots; j++)
  {
    if(GetParabPtProjFromU(da, dRoots[j], cPt, &dx))
    {
      if(!PtInDblList(dx, iRes, pdRoots)) pdRoots[iRes++] = dx;
    }
  }
  return iRes;
}

int GetParabAttractors(CDPoint cPt, PDPointList pCache, PDPoint pPoints)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return 0.0;

  CDPoint cOrig, cRad, cNorm;

  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cPt1 = Rotate(cPt - cOrig, cNorm, false);

  double dProjs[4];
  int iRes = GetParabPtProj(cRad.x, cPt1, dProjs);

  CDPoint cPtNorm;
  double du, dNorm;

  for(int i = 0; i < iRes; i++)
  {
    du = dProjs[i];

    cPtNorm.x = 2.0*cRad.x*du;
    cPtNorm.y = -1.0;
    dNorm = GetNorm(cPtNorm);
    cPtNorm /= dNorm;

    cPt1.x = du + dr*cPtNorm.x;
    cPt1.y = cRad.x*Power2(du) + dr*cPtNorm.y;

    pPoints[i] = cOrig + Rotate(cPt1, cNorm, true);
  }
  return iRes;
}

double GetParabBoundProj(double da, double dOffset, CDPoint cPt, CDPoint cRefPt, bool bSecond)
{
  double pProjs[4];
  double pDists[2];
  int iRoots = GetParabPtProj(da, cPt, pProjs);
  if(iRoots < 2) return pProjs[0];

  int i = 0;

  CDPoint cNorm, cProjPt;
  double du = pProjs[i];
  cProjPt.x = du;
  cProjPt.y = da*Power2(du);
  cNorm.x = 2.0*da*du;
  cNorm.y = -1.0;
  double dNorm = GetNorm(cNorm);
  pDists[0] = GetDist(cRefPt, cProjPt + dOffset*cNorm/dNorm);
  i++;

  int i0 = 0, i1 = 0;
  du = pProjs[i];
  cProjPt.x = du;
  cProjPt.y = da*Power2(du);
  cNorm.x = 2.0*da*du;
  cNorm.y = -1.0;
  dNorm = GetNorm(cNorm);
  double dMin = GetDist(cRefPt, cProjPt + dOffset*cNorm/dNorm);
  if(dMin < pDists[0])
  {
    pDists[1] = pDists[0];
    pDists[0] = dMin;
    i0 = i;
  }
  else
  {
    pDists[1] = dMin;
    i1 = i;
  }
  i++;
  while(i < iRoots)
  {
    du = pProjs[i];
    cProjPt.x = du;
    cProjPt.y = da*Power2(du);
    cNorm.x = 2.0*da*du;
    cNorm.y = -1.0;
    dNorm = GetNorm(cNorm);
    dMin = GetDist(cRefPt, cProjPt + dOffset*cNorm/dNorm);
    if(dMin < pDists[0])
    {
      pDists[1] = pDists[0];
      pDists[0] = dMin;
      i1 = i0;
      i0 = i;
    }
    else if(dMin < pDists[1])
    {
      pDists[1] = dMin;
      i1 = i;
    }
    i++;
  }
  if(bSecond) return pProjs[i1];
  return pProjs[i0];
}

double GetParabDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX)
{
  pPtX->bIsSet = false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return 0.0;

  CDPoint cOrig, cRad, cNorm, cPt1, cPt2, cPt3, cPt4;

  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  cPt1 = Rotate(cPt - cOrig, cNorm, false);
  cPt2 = Rotate(cRefPt - cOrig, cNorm, false);

  double dMinX = 0.0;
  if((iSrchMask & 1) && (pCache->GetCount(3) > 0))
  {
    cPt3 = pCache->GetPoint(0, 3).cPoint;
    dMinX = GetDist(cPt2, cPt3);

    pPtX->bIsSet = true;
    pPtX->cOrigin = cOrig + Rotate(cPt3, cNorm, true);
    pPtX->cDirection = 0;
  }

  double dx = GetParabBoundProj(cRad.x, dr, cPt1, cPt2, iSrchMask & 2);

  CDPoint cDir;
  cDir.x = 2.0*cRad.x*dx;
  cDir.y = -1.0;
  double d1 = GetNorm(cDir);
  if(d1 < g_dPrec) return dMinX;
  cDir /= d1;

  cPt3.x = dx + dr*cDir.x;
  cPt3.y = cRad.x*Power2(dx) + dr*cDir.y;

  cPt4 = Rotate(cPt2 - cPt3, cDir, false);

  if(!pPtX->bIsSet || (fabs(cPt4.x) < dMinX))
  {
    dMinX = cPt4.x;
    pPtX->bIsSet = true;
    pPtX->cOrigin = cOrig + Rotate(cPt3, cNorm, true);
    pPtX->cDirection = Rotate(cDir, cNorm, true);
    pPtX->dRef = dx;
  }
  return dMinX;
}

bool HasParabEnoughPoints(PDPointList pPoints, int iInputLines)
{
  int nNorm = pPoints->GetCount(0);
  bool bRes = false;

  if(iInputLines == 1) bRes = (nNorm > 0);

  return bRes;
}

double GetParabRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt)
{
  pPtR->bIsSet = false;
  pPtR->cOrigin = 0;
  pPtR->cDirection = 0;

  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return -1.0;

  CDPoint cOrig, cRad, cNorm, cPt1, cPt2, cDir;

  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  cPt1 = Rotate(cPt - cOrig, cNorm, false);

  cPt2.x = -4.0*cRad.x*cPt1.x*cPt1.y;
  cPt2.y = 3.0*cPt1.y + 1/cRad.x/2.0;

  cDir = cPt1 - cPt2;
  double dNorm = GetNorm(cDir);

  pPtR->bIsSet = true;
  pPtR->cOrigin = cOrig + Rotate(cPt2, cNorm, true);
  if(dNorm > g_dPrec) pPtR->cDirection = Rotate(cDir/dNorm, cNorm, true);

  return dNorm + dr;
}

bool GetParabPointRefDist(double dRef, PDPointList pCache, double *pdDist)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  //CDPoint cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  double dBreak = -1.0;
  if(pCache->GetCount(4) > 0) dBreak = pCache->GetPoint(0, 4).cPoint.x;

  *pdDist = GetCurveDistAtRef(&cRad.x, dr, {dBreak, -1.0}, fabs(dRef),
    ParabFunc, ParabFuncDer, 0.5, 1, {0.0, 0.0});
  if(dRef < 0.0) *pdDist *= -1.0;
  return true;
}

double GetParabPointAtDist(double da, double dr, double dBreak, double dDist)
{
  CDPoint cPt1 = GetCurveRefAtDist(&da, dr, {dBreak, -1.0}, fabs(dDist),
    ParabFunc, ParabFuncDer, 0.5, 1, {0.0, 0.0});

  if(dBreak > -0.5)
  {
    CDPoint cPt2 = {dBreak, da*Power2(dBreak)};
    CDPoint cNorm = {2.0*da*dBreak, -1.0};
    double dNorm = GetNorm(cNorm);
    cNorm /= dNorm;

    double dDist2 = GetDist(cPt1, cPt2 + dr*cNorm);
    double dblDist = 2.0*g_dPrec;
    if(dDist2 < dblDist)
    {
      if(dDist < 0.0) return -dBreak;
      return dBreak;
    }

    double dDisc = Power2(dr) - 0.25/Power2(da);
    if(dDisc > g_dPrec)
    {
      double dt = sqrt(dDisc);

      cPt2.x = dt;
      cPt2.y = da*Power2(dt);
      cNorm.x = 2.0*da*dt;
      cNorm.y = -1.0;
      dNorm = GetNorm(cNorm);
      cNorm /= dNorm;

      dDist2 = GetDist(cPt1, cPt2 + dr*cNorm);
      if(dDist2 < 0.1)
      {
        if(dDist < 0.0) return -dt;
        return dt;
      }
    }
  }

  double dRes = GetParabBoundProj(da, dr, cPt1, cPt1, false);
  if(dDist < 0.0) dRes *= -1.0;
  return dRes;
}

void AddParabSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return;

  CDPoint cOrig, cRad, cNorm;

  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  double dBreak = -1.0;
  if(pCache->GetCount(4) > 0) dBreak = pCache->GetPoint(0, 4).cPoint.x;

  double dx1 = GetParabPointAtDist(cRad.x, dr, dBreak, d1);
  double dx2 = GetParabPointAtDist(cRad.x, dr, dBreak, d2);

  dr += dExt;

  PDPrimObject pTmpPrim = new CDPrimObject();
  AddCurveSegment(&cRad.x, dr, {dBreak, -1.0}, ParabFunc, ParabFuncDer, dx1, dx2, 0.5, 1, pTmpPrim);
  RotatePrimitives(pTmpPrim, pPrimList, cOrig, cNorm);
  delete pTmpPrim;
}

bool GetParabRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  CDPoint cOrig, cRad, cNorm;

  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cDir;
  cDir.x = 2.0*cRad.x*dRef;
  cDir.y = -1.0;
  double d1 = GetNorm(cDir);
  if(d1 < g_dPrec) return false;
  cDir /= d1;

  CDPoint cPt1;
  cPt1.x = dRef + dr*cDir.x;
  cPt1.y = cRad.x*Power2(dRef) + dr*cDir.y;
  *pPt = cOrig + Rotate(cPt1, cNorm, true);
  return true;
}

bool GetParabRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache)
{
  if(iMode != 2) return false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  double dDist = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dDist = pCache->GetPoint(0, 2).cPoint.y;

  double dRad = dDist + dRestrictValue;

  CDPoint cPt1 = Rotate(cPt - cOrig, cMainDir, false);
  double dx = GetParabBoundProj(cRad.x, dRad, cPt1, cPt1, false);

  CDPoint cDir;
  cDir.x = 1.0;
  cDir.y = 2.0*cRad.x*dx;
  double dNorm = GetNorm(cDir);
  if(dNorm < g_dPrec) return false;

  CDPoint cPt2;
  cPt2.x = dx + dRad*cDir.y/dNorm;
  cPt2.y = cRad.x*Power2(dx) - dRad*cDir.x/dNorm;
  *pSnapPt = cOrig + Rotate(cPt2, cMainDir, true);
  return true;
}

double GetParabOffset(PDPointList pCache)
{
  int nOffs = pCache->GetCount(2);
  if(nOffs < 1) return 0.0;
  return pCache->GetPoint(0, 2).cPoint.x;
}

bool GetParabRefDir(double dRef, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cNorm = pCache->GetPoint(2, 0).cPoint;

  CDPoint cDir;
  cDir.x = 1.0;
  cDir.y = 2.0*cRad.x*dRef;
  double d1 = GetNorm(cDir);
  if(d1 < g_dPrec) return false;
  cDir /= d1;

  *pPt = Rotate(cDir, cNorm, true);
  return true;
}

bool GetParabReference(double dDist, PDPointList pCache, double *pdRef)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  //CDPoint cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  double dBreak = -1.0;
  if(pCache->GetCount(4) > 0) dBreak = pCache->GetPoint(0, 4).cPoint.x;

  *pdRef = GetParabPointAtDist(cRad.x, dr, dBreak, dDist);
  return true;
}

double ParabPtProjFunc(void *pvData, double dOffset, CDPoint cPt, CDPoint cStart, CDPoint cEnd)
{
  double da = *(double*)pvData;
  double pProjs[4];
  int iRoots = GetParabPtProj(da, cPt, pProjs);

  double pDists[4];
  bool pValid[4];

  CDPoint cNorm, cProjPt;
  double du, dNorm;

  for(int i = 0; i < iRoots; i++)
  {
    du = pProjs[i];
    pValid[i] = GetRefInUboundSeg(du, cStart, cEnd);
    cProjPt.x = du;
    cProjPt.y = da*Power2(du);
    cNorm.x = 2.0*da*du;
    cNorm.y = -1.0;
    dNorm = GetNorm(cNorm);
    pDists[i] = GetDist(cPt, cProjPt + dOffset*cNorm/dNorm);
  }

  bool bFound = false;
  int iMin;
  double dMin;
  int i = 0;
  while(!bFound && (i < iRoots))
  {
    bFound = pValid[i++];
  }
  if(bFound)
  {
    iMin = i - 1;
    dMin = pDists[iMin];
    while(i < iRoots)
    {
      if(pValid[i] && (pDists[i] < dMin))
      {
        iMin = i;
        dMin = pDists[i];
      }
      i++;
    }
    return pProjs[iMin];
  }

  // if we could not find the point inside the interval, return the nearest one
  iMin = 0;
  dMin = pDists[0];
  i = 1;
  while(i < iRoots)
  {
    if(pDists[i] < dMin)
    {
      iMin = i;
      dMin = pDists[i];
    }
    i++;
  }
  return pProjs[iMin];
}

int AddParabInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  double dBreak = -1.0;
  if(pCache->GetCount(4) > 0) dBreak = pCache->GetPoint(0, 4).cPoint.x;

  CDPoint cLn1, cLn2;
  cLn1 = Rotate(cPt1 - cOrig, cNorm, false);
  cLn2 = Rotate(cPt2 - cOrig, cNorm, false);
  CDPoint cDir = cLn2 - cLn1;

  double dTangent = 0.0;
  if(fabs(cDir.x) > g_dPrec)
    dTangent = cDir.y/cDir.x/cRad.x/2.0;

  int iRes = 0;
  double dRefs[6];
  if(dBreak < -g_dPrec)
  {
    iRes = AddCurveInterLine(&cRad.x, dr, ParabFunc, ParabFuncDer, ParabPtProjFunc,
      {1.0, dTangent}, {0.0, 0.0}, {0.0, 0.0}, cLn1, cLn2, &dRefs[iRes]);
  }
  else if(dBreak < g_dPrec)
  {
    iRes = AddCurveInterLine(&cRad.x, dr, ParabFunc, ParabFuncDer, ParabPtProjFunc,
      {1.0, dTangent}, {0.0, 0.0}, {1.0, 0.0}, cLn1, cLn2, &dRefs[iRes]);
    iRes += AddCurveInterLine(&cRad.x, dr, ParabFunc, ParabFuncDer, ParabPtProjFunc,
      {1.0, dTangent}, {1.0, 0.0}, {0.0, 0.0}, cLn1, cLn2, &dRefs[iRes]);
  }
  else
  {
    iRes = AddCurveInterLine(&cRad.x, dr, ParabFunc, ParabFuncDer, ParabPtProjFunc,
      {1.0, dTangent}, {0.0, 0.0}, {1.0, -dBreak}, cLn1, cLn2, &dRefs[iRes]);
    iRes += AddCurveInterLine(&cRad.x, dr, ParabFunc, ParabFuncDer, ParabPtProjFunc,
      {1.0, dTangent}, {1.0, -dBreak}, {1.0, dBreak}, cLn1, cLn2, &dRefs[iRes]);
    iRes += AddCurveInterLine(&cRad.x, dr, ParabFunc, ParabFuncDer, ParabPtProjFunc,
      {1.0, dTangent}, {1.0, dBreak}, {0.0, 0.0}, cLn1, cLn2, &dRefs[iRes]);
  }
  for(int i = 0; i < iRes; i++) pBounds->AddPoint(dRefs[i]);
  return iRes;
}

int GetParabNumParts(PDPointList pCache, PDRefPoint pBounds)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return 0;

  int iBreaks = pCache->GetCount(4);
  if(iBreaks < 1) return 0;

  CDPoint cBreak = pCache->GetPoint(0, 4).cPoint;

  double dAng = cBreak.x;

  if(dAng < g_dPrec)
  {
    if(pBounds[0].bIsSet && (pBounds[0].dRef > -g_dPrec)) return 0;
    if(pBounds[1].bIsSet && (pBounds[1].dRef < g_dPrec)) return 0;
    return 1;
  }

  int iRes = 0;
  if(RefInOpenBounds(pBounds, -dAng) > 2) iRes++;
  if(RefInOpenBounds(pBounds, dAng) > 2) iRes++;

  return iRes;
}

bool ParabRemovePart(bool bDown, PDPointList pCache, PDRefPoint pBounds)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  int iBreaks = pCache->GetCount(4);
  if(iBreaks < 1) return false;

  CDPoint cBreak = pCache->GetPoint(0, 4).cPoint;

  double dAng = cBreak.x;

  if(dAng < g_dPrec)
  {
    if(bDown)
    {
      pBounds[0].bIsSet = true;
      pBounds[0].dRef = 0.0;
    }
    else
    {
      pBounds[1].bIsSet = true;
      pBounds[1].dRef = 0.0;
    }
    return true;
  }

  if(bDown)
  {
    if(RefInOpenBounds(pBounds, dAng) > 2)
    {
      pBounds[0].bIsSet = true;
      pBounds[0].dRef = dAng;
    }
    else if(RefInOpenBounds(pBounds, -dAng) > 2)
    {
      pBounds[0].bIsSet = true;
      pBounds[0].dRef = -dAng;
    }
  }
  else
  {
    if(RefInOpenBounds(pBounds, dAng) > 2)
    {
      pBounds[1].bIsSet = true;
      pBounds[1].dRef = dAng;
    }
    else if(RefInOpenBounds(pBounds, -dAng) > 2)
    {
      pBounds[1].bIsSet = true;
      pBounds[1].dRef = -dAng;
    }
  }
  return true;
}

int GetParabSnapPoints(PDPointList pCache, double *pdRefs)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  //CDPoint cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  double dBreak = -1.0;
  if(pCache->GetCount(4) > 0) dBreak = pCache->GetPoint(0, 4).cPoint.x;

  if(dBreak < -g_dPrec) return 0;

  int iRes = 0;
  if(dBreak < g_dPrec)
  {
    pdRefs[iRes++] = 0.0;
    return iRes;
  }

  pdRefs[iRes++] = -dBreak;
  pdRefs[iRes++] = dBreak;

  double dDisc = Power2(dr) - 0.25/Power2(cRad.x);

  if(dDisc < -g_dPrec) return iRes;
  if(dDisc < g_dPrec) return iRes;
  double dt = sqrt(dDisc);
  pdRefs[iRes++] = -dt;
  pdRefs[iRes++] = dt;
  return iRes;
}

