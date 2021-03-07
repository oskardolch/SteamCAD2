#include "DHyper.hpp"
#include "DMath.hpp"
#include <math.h>
#include "DPrimitive.hpp"
#include <stdio.h>

// for debugging purpose only
/*#include <windows.h>
#include <commctrl.h>
#include <wchar.h>
extern HWND g_hStatus;*/
// -----

double GetHyperBreakAngle(double da, double db, double dr)
{
  double da2 = Power2(da);
  double db2 = Power2(db);
  double d1 = da2 + db2;
  if(d1 < g_dPrec) return -1.0;

  double d2 = cbrt(da2*db2*Power2(dr));
  double d3 = d2 - db2;

  if(d3 < -g_dPrec) return -1.0;
  if(d3 < g_dPrec) return 0.0;

  return sqrt(d3/d1);
}

CDPoint HyperFunc(void *pvData, double dt)
{
  PDPoint pPt = (PDPoint)pvData;
  double dv = sqrt(1.0 + Power2(dt));
  CDPoint cRes;
  cRes.x = pPt->x*dv;
  cRes.y = pPt->y*dt;
  return cRes;
}

CDPoint HyperFuncDer(void *pvData, double dt)
{
  PDPoint pPt = (PDPoint)pvData;
  double dv = sqrt(1.0 + Power2(dt));
  CDPoint cRes;
  cRes.x = -pPt->x*dt/dv;
  cRes.y = -pPt->y;
  return cRes;
}

double HypProjFn(double da, double db, double dx, double dy, double du)
{
  double dv = sqrt(1.0 + Power2(du));
  return du*(Power2(da) + Power2(db)) - da*dx*du/dv - db*dy;
}

double HypProjFnDer(double da, double db, double dx, double dy, double du)
{
  double da1 = 1.0 + Power2(du);
  double dv = sqrt(da1);
  return Power2(da) + Power2(db) - da*dx/da1/dv;
}

bool GetHyperPtProjFromU(double da, double db, double dStart, CDPoint cPt, double *pdRes)
{
  int j = 0;
  double du1 = dStart;
  double df = HypProjFn(da, db, cPt.x, cPt.y, du1);
  double df2 = HypProjFnDer(da, db, cPt.x, cPt.y, du1);
  if(fabs(df2) < g_dPrec) return 0.0;

  du1 -= df/df2;

  while((j < 16) && (fabs(df) > g_dRootPrec))
  {
    j++;
    df = HypProjFn(da, db, cPt.x, cPt.y, du1);
    df2 = HypProjFnDer(da, db, cPt.x, cPt.y, du1);
    if(fabs(df2) > g_dPrec) du1 -= df/df2;
    else j = 16;
  }
  *pdRes = du1;
  return fabs(df) < g_dRootPrec;
}

int GetHyperPtProj(double da, double db, CDPoint cPt, double *pdRoots)
{
  double dPoly[5];
  double d1 = Power2(da) + Power2(db);
  double dU = db*cPt.y/d1;
  double dV = da*cPt.x/d1;
  dPoly[0] = Power2(dU);
  dPoly[1] = -2.0*dU;
  dPoly[2] = 1.0 + dPoly[0] - Power2(dV);
  dPoly[3] = dPoly[1];
  dPoly[4] = 1.0;

  double dRoots[4];
  int iRoots = SolvePolynom(4, dPoly, dRoots);
  if(iRoots < 1)
  {
    if(GetHyperPtProjFromU(da, db, 0.0, cPt, pdRoots)) return 1;
    return 0;
  }

  int iRes = 0;
  double du;
  for(int j = 0; j < iRoots; j++)
  {
    if(GetHyperPtProjFromU(da, db, dRoots[j], cPt, &du))
    {
      if(!PtInDblList(du, iRes, pdRoots)) pdRoots[iRes++] = du;
    }
  }
  return iRes;
}

int GetHyperAttractors(CDPoint cPt, PDPointList pCache, PDPoint pPoints)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return 0.0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cPt1 = Rotate(cPt - cOrig, cMainDir, false);

  double dProjs[4];
  int iRes = GetHyperPtProj(cRad.x, cRad.y, cPt1, dProjs);

  CDPoint cPtNorm;
  double d1, du, dNorm;

  for(int i = 0; i < iRes; i++)
  {
    du = dProjs[i];
    d1 = sqrt(1.0 + Power2(du));

    cPtNorm.x = -cRad.y;
    cPtNorm.y = cRad.x*du/d1;
    dNorm = GetNorm(cPtNorm);
    cPtNorm /= dNorm;

    cPt1.x = cRad.x*d1 + dr*cPtNorm.x;
    cPt1.y = cRad.y*du + dr*cPtNorm.y;
    pPoints[i] = cOrig + Rotate(cPt1, cMainDir, true);
  }
  return iRes;
}

double GetHyperBoundProj(double da, double db, double dOffset, CDPoint cPt, CDPoint cRefPt, bool bSecond)
{
  double pProjs[4];
  double pDists[2];
  int iRoots = GetHyperPtProj(da, db, cPt, pProjs);
  if(iRoots < 2) return pProjs[0];

  int i = 0;

  CDPoint cNorm, cProjPt;
  double du = pProjs[i];
  double dc = sqrt(1.0 + Power2(du));
  cProjPt.x = da*dc;
  cProjPt.y = db*du;
  cNorm.x = -db;
  cNorm.y = da*du/dc;
  double dNorm = GetNorm(cNorm);
  pDists[0] = GetDist(cRefPt, cProjPt + dOffset*cNorm/dNorm);
  i++;

  int i0 = 0, i1 = 0;
  du = pProjs[i];
  dc = sqrt(1.0 + Power2(du));
  cProjPt.x = da*dc;
  cProjPt.y = db*du;
  cNorm.x = -db;
  cNorm.y = da*du/dc;
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
    dc = sqrt(1.0 + Power2(du));
    cProjPt.x = da*dc;
    cProjPt.y = db*du;
    cNorm.x = -db;
    cNorm.y = da*du/dc;
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

bool AddHyperPoint(double x, double y, char iCtrl, PDPointList pPoints, int iInputLines)
{
  bool bRes = false;
  int nNorm = pPoints->GetCount(0);

  if(iInputLines == 2)
  {
    if((iCtrl < 1) && (nNorm < 1))
    {
      pPoints->AddPoint(x, y, 0);
      nNorm++;
    }
    bRes = (nNorm > 0);
  }
  return bRes;
}

bool BuildHyperCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache, PDLine pLines)
{
  pCache->ClearAll();

  int nNorm = pPoints->GetCount(0);

  CDInputPoint cInPt1;
  CDPoint cOrig, cMainDir, cPt1, cPt2, cPt3;
  double d1, d2, d3;
  double dr1 = -1.0, dr2 = -1.0;

  if(!(pLines[0].bIsSet && pLines[1].bIsSet)) return false;

  int iX = LineXLine(pLines[0].cOrigin, pLines[0].cDirection,
    pLines[1].cOrigin, pLines[1].cDirection, &cOrig);
  if(iX < 1) return false;

  if(!(((nNorm < 1) && (iMode == 1)) || ((nNorm == 1) && (iMode != 1)))) return false;

  if(iMode == 1) cPt1 = cTmpPt.cOrigin - cOrig;
  else if(nNorm > 0)
  {
    cInPt1 = pPoints->GetPoint(0, 0);
    cPt1 = cInPt1.cPoint - cOrig;
  }
  else return false;

  cPt2 = pLines[0].cDirection + pLines[1].cDirection;
  d2 = GetNorm(cPt2);
  cMainDir = cPt2/d2;

  cPt2 = Rotate(cPt1, cMainDir, false);
  cPt3 = Abs(Rotate(pLines[0].cDirection, cMainDir, false));

  d1 = Deter2(Abs(cPt2), cPt3);
  if(d1 < 0)
  {
    d2 = cMainDir.x;
    cMainDir.x = -cMainDir.y;
    cMainDir.y = d2;

    cPt2 = Rotate(cPt1, cMainDir, false);
    cPt3 = Abs(Rotate(pLines[0].cDirection, cMainDir, false));
  }

  if(cPt2.x < 0)
  {
    cMainDir.x *= -1.0;
    cMainDir.y *= -1.0;

    cPt2 = Rotate(cPt1, cMainDir, false);
    cPt3 = Abs(Rotate(pLines[0].cDirection, cMainDir, false));
  }

  d3 = sqrt(Power2(cPt2.x/cPt3.x) - Power2(cPt2.y/cPt3.y));

  dr1 = d3*cPt3.x;
  dr2 = d3*cPt3.y;

  if((dr1 < g_dPrec) || (dr2 < g_dPrec)) return false;

  pCache->AddPoint(cOrig.x, cOrig.y, 0);
  pCache->AddPoint(dr1, dr2, 0);
  pCache->AddPoint(cMainDir.x, cMainDir.y, 0);

  double dr = Power2(dr2)/dr1;
  pCache->AddPoint(dr1 + dr, 0.0, 3);
  return true;
}

void UpdateHyperCache(PDPointList pCache)
{
  if(pCache->GetCount(4) > 0) pCache->Remove(0, 4);
  int nNorm = pCache->GetCount(0);
  int nOffs = pCache->GetCount(2);
  if((nNorm > 1) && (nOffs > 0))
  {
    CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
    double dDist = pCache->GetPoint(0, 2).cPoint.x;
    double dr = GetHyperBreakAngle(cRad.x, cRad.y, -dDist);
    if(dr > -0.5) pCache->AddPoint(dr, 0.0, 4);
  }
}

CDPoint GetHyperPointDir(double da, double db, double dOffset, double dx, PDPoint pDir)
{
  CDPoint cRes, cDir;
  double dv = sqrt(1.0 + Power2(dx));
  cDir.x = -db;
  cDir.y = da*dx/dv;
  double dNorm = GetNorm(cDir);
  cDir /= dNorm;
  cRes.x = da*dv;
  cRes.y = db*dx;
  pDir->x = -cDir.y;
  pDir->y = cDir.x;
  return cRes + dOffset*cDir;
}

double GetHyperDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX)
{
  pPtX->bIsSet = false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return 0.0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cNorm = pCache->GetPoint(2, 0).cPoint;

  CDPoint cPt1 = Rotate(cPt - cOrig, cNorm, false);
  CDPoint cRefPt1 = Rotate(cRefPt - cOrig, cNorm, false);
  CDPoint cPt2;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  double dMinX = 0.0;
  if((iSrchMask & 1) && (pCache->GetCount(3) > 0))
  {
    cPt2 = pCache->GetPoint(0, 3).cPoint;
    dMinX = GetDist(cRefPt1, cPt2);

    pPtX->bIsSet = true;
    pPtX->cOrigin = cOrig + Rotate(cPt2, cNorm, true);
    pPtX->cDirection = 0;
  }

  double du = GetHyperBoundProj(cRad.x, cRad.y, dr, cPt1, cRefPt1, iSrchMask & 2);

  double d1 = sqrt(1.0 + Power2(du));

  CDPoint cDir;
  cDir.x = -cRad.y;
  cDir.y = cRad.x*du/d1;
  double dNorm = GetNorm(cDir);
  cDir /= dNorm;

  cPt2.x = cRad.x*d1 + dr*cDir.x;
  cPt2.y = cRad.y*du + dr*cDir.y;
  double dDir = 1.0;

  double dMin = GetDist(cRefPt1, cPt2);
  if(!pPtX->bIsSet || (dMin < dMinX))
  {
    if(cPt1.x > cPt2.x) dDir = -1.0;

    CDPoint cPtMin = cOrig + Rotate(cPt2, cNorm, true);
    CDPoint cNormMin = Rotate(cDir, cNorm, true);

    pPtX->bIsSet = true;
    pPtX->cOrigin = cPtMin;
    pPtX->cDirection = cNormMin;
    pPtX->dRef = du;
    dMinX = dMin;
  }
  return dDir*dMinX;
}

bool HasHyperEnoughPoints(PDPointList pPoints, int iInputLines)
{
  int nNorm = pPoints->GetCount(0);
  bool bRes = false;

  if(iInputLines == 2) bRes = (nNorm > 0);

  return bRes;
}

double GetHyperRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt)
{
  pPtR->bIsSet = false;
  pPtR->cOrigin = 0;
  pPtR->cDirection = 0;

  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return -1.0;

  CDPoint cOrig, cRad, cNorm;

  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cDir, cPt1, cPt2;

  cPt1 = Rotate(cPt - cOrig, cNorm, false);
  double du = GetHyperBoundProj(cRad.x, cRad.y, dr, cPt1, cPt1, false);

  double dx2 = 1.0 + Power2(du);
  double dy2 = Power2(du);
  double dab = Power2(cRad.y/cRad.x);

  cPt2.x = cRad.x*sqrt(dx2)*(1.0 + dy2 + dab*dx2);
  cPt2.y = cRad.y*du*(1.0 - dx2 - dy2/dab);

  pPtR->bIsSet = true;
  pPtR->cOrigin = cOrig + Rotate(cPt2, cNorm, true);

  cDir = cPt2 - cPt1;
  double dNorm = GetNorm(cDir);
  if(dNorm > g_dPrec) pPtR->cDirection = Rotate(cDir/dNorm, cNorm, true);

  return dNorm + dr;
}

bool GetHyperPointRefDist(double dRef, double dOffset, PDPointList pCache, double *pdDist)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  //CDPoint cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  double dBreak = -1.0;
  if(pCache->GetCount(4) > 0) dBreak = pCache->GetPoint(0, 4).cPoint.x;

  *pdDist = GetCurveDistAtRef(&cRad, dr, {dBreak, -1.0}, fabs(dRef),
    HyperFunc, HyperFuncDer, 0.5, 1, {0.0, 0.0});
  if(dRef < 0.0) *pdDist *= -1.0;
  return true;
}

double GetHyperPointAtDist(double da, double db, double dr, double dBreak, double dDist)
{
  CDPoint cRad = {da, db};
  CDPoint cPt1 = GetCurveRefAtDist(&cRad, dr, {dBreak, -1.0}, fabs(dDist),
    HyperFunc, HyperFuncDer, 0.5, 1, {0.0, 0.0});

  if(dBreak > -0.5)
  {
    double dv = sqrt(1.0 + Power2(dBreak));
    CDPoint cPt2 = {cRad.x*dv, cRad.y*dBreak};
    CDPoint cNorm = {-cRad.y, cRad.x*dBreak/dv};
    double dNorm = GetNorm(cNorm);
    cNorm /= dNorm;

    double dDist2 = GetDist(cPt1, cPt2 + dr*cNorm);
    double dblDist = 2.0*g_dPrec;
    if(dDist2 < dblDist)
    {
      if(dDist < 0.0) return -dBreak;
      return dBreak;
    }

    double a2 = Power2(cRad.x);
    double b2 = Power2(cRad.y);
    double c2 = Power2(dr/cRad.y);
    double dDisc = (a2*c2 - b2)/(a2 + b2);
    if(dDisc > g_dPrec)
    {
      double du = sqrt(dDisc);
      dv = sqrt(1.0 + dDisc);

      cPt2.x = cRad.x*dv;
      cPt2.y = cRad.y*du;
      cNorm.x = -cRad.y;
      cNorm.y = cRad.x*du/dv;
      dNorm = GetNorm(cNorm);
      cNorm /= dNorm;

      dDist2 = GetDist(cPt1, cPt2 + dr*cNorm);
      if(dDist2 < 0.1)
      {
        if(dDist < 0.0) return -du;
        return du;
      }
    }
  }

  double dRes = GetHyperBoundProj(da, db, dr, cPt1, cPt1, false);
  if(dDist < 0.0) dRes *= -1.0;
  return dRes;
}

void AddHyperSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return;

  CDPoint cOrig, cRad, cNorm;

  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  double dBreak = GetHyperBreakAngle(cRad.x, cRad.y, -dr);
  double dy1 = GetHyperPointAtDist(cRad.x, cRad.y, dr, dBreak, d1);
  double dy2 = GetHyperPointAtDist(cRad.x, cRad.y, dr, dBreak, d2);

  PDPrimObject pTmpPrim = new CDPrimObject();
  PDPrimObject pRotPrim = new CDPrimObject();
  AddCurveSegment(&cRad, dr, {dBreak, -1.0}, HyperFunc, HyperFuncDer, dy1, dy2, 0.5, 1, pTmpPrim);
  RotatePrimitives(pTmpPrim, pRotPrim, cOrig, cNorm);
  if(bReverse)
  {
    pTmpPrim->Clear();
    ReversePrimitives(pRotPrim, pTmpPrim);
    pPrimList->CopyFrom(pTmpPrim);
  }
  else pPrimList->CopyFrom(pRotPrim);
  delete pRotPrim;
  delete pTmpPrim;
}

bool GetHyperRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt)
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

  double dv = sqrt(1.0 + Power2(dRef));
  CDPoint cDir;
  cDir.x = -cRad.y;
  cDir.y = cRad.x*dRef/dv;
  double dN1 = GetNorm(cDir);
  if(dN1 < g_dPrec) return false;

  CDPoint cPt1;
  cPt1.x = cRad.x*dv + dr*cDir.x/dN1;
  cPt1.y = cRad.y*dRef + dr*cDir.y/dN1;
  *pPt = cOrig + Rotate(cPt1, cNorm, true);
  return true;
}

bool GetHyperRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
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
  double dy = GetHyperBoundProj(cRad.x, cRad.y, dRad, cPt1, cPt1, false);

  double dv = sqrt(1.0 + Power2(dy));

  CDPoint cDir;
  cDir.x = cRad.x*dy/dv;
  cDir.y = cRad.y;
  double dNorm = GetNorm(cDir);
  if(dNorm < g_dPrec) return false;

  CDPoint cPt2;
  cPt2.x = cRad.x*dv - dRad*cDir.y/dNorm;
  cPt2.y = cRad.y*dy + dRad*cDir.x/dNorm;
  *pSnapPt = cOrig + Rotate(cPt2, cMainDir, true);
  return true;
}

bool GetHyperRefDir(double dRef, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return false;

  CDPoint cRad, cNorm;

  //cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cNorm = pCache->GetPoint(2, 0).cPoint;

  double dv = sqrt(1.0 + Power2(dRef));
  CDPoint cDir;
  cDir.x = cRad.x*dRef/dv;
  cDir.y = cRad.y;
  double dN1 = GetNorm(cDir);
  if(dN1 < g_dPrec) return false;

  CDPoint cPt1 = cDir/dN1;
  *pPt = Rotate(cPt1, cNorm, true);

  if(pCache->GetCount(4) > 0)
  {
    double dBreak = pCache->GetPoint(0, 4).cPoint.x;
    if((dBreak > -0.5) && (dRef > -dBreak) && (dRef < dBreak)) *pPt *= -1.0;
  }
  return true;
}

bool GetHyperReference(double dDist, double dOffset, PDPointList pCache, double *pdRef)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  //CDPoint cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  double dBreak = GetHyperBreakAngle(cRad.x, cRad.y, -dr);
  *pdRef = GetHyperPointAtDist(cRad.x, cRad.y, dr, dBreak, dDist);
  return true;
}

double HyperPtProjFunc(void *pvData, double dOffset, CDPoint cPt, CDPoint cStart, CDPoint cEnd)
{
  PDPoint pPt = (PDPoint)pvData;
  double pProjs[4];
  int iRoots = GetHyperPtProj(pPt->x, pPt->y, cPt, pProjs);

  double pDists[4];
  bool pValid[4];

  CDPoint cNorm, cProjPt;
  double du, dc, dNorm;

  for(int i = 0; i < iRoots; i++)
  {
    du = pProjs[i];
    pValid[i] = GetRefInUboundSeg(du, cStart, cEnd);
    dc = sqrt(1.0 + Power2(du));
    cProjPt.x = pPt->x*dc;
    cProjPt.y = pPt->y*du;
    cNorm.x = -pPt->y;
    cNorm.y = pPt->x*du/dc;
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

int AddHyperInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  double dBreak = GetHyperBreakAngle(cRad.x, cRad.y, -dr);

  CDPoint cLn1, cLn2;
  cLn1 = Rotate(cPt1 - cOrig, cNorm, false);
  cLn2 = Rotate(cPt2 - cOrig, cNorm, false);
  CDPoint cDir = cLn2 - cLn1;

  bool bTangent = true;
  double dTangent = 0.0;
  if(fabs(cDir.x) > g_dPrec)
  {
    double dc = Power2(cRad.x*cDir.y) - Power2(cRad.y*cDir.x);
    if(dc < g_dPrec) bTangent = false;
    else
    {
      dTangent = cRad.y*fabs(cDir.x)/sqrt(dc);
      if(cDir.x*cDir.y < -g_dPrec) dTangent *= -1.0;
    }
  }

  int iRes = 0;
  double dRefs[6];
  if(dBreak < -g_dPrec)
  {
    iRes = AddCurveInterLine(&cRad, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {0.0, 0.0}, {0.0, 0.0}, cLn1, cLn2, &dRefs[iRes]);
  }
  else if(dBreak < g_dPrec)
  {
    iRes = AddCurveInterLine(&cRad, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {0.0, 0.0}, {1.0, 0.0}, cLn1, cLn2, &dRefs[iRes]);
    iRes += AddCurveInterLine(&cRad, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {1.0, 0.0}, {0.0, 0.0}, cLn1, cLn2, &dRefs[iRes]);
  }
  else
  {
    iRes = AddCurveInterLine(&cRad, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {0.0, 0.0}, {1.0, -dBreak}, cLn1, cLn2, &dRefs[iRes]);
    iRes += AddCurveInterLine(&cRad, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {1.0, -dBreak}, {1.0, dBreak}, cLn1, cLn2, &dRefs[iRes]);
    iRes += AddCurveInterLine(&cRad, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {1.0, dBreak}, {0.0, 0.0}, cLn1, cLn2, &dRefs[iRes]);
  }

  for(int i = 0; i < iRes; i++) pBounds->AddPoint(dRefs[i]);
  return iRes;
}

int GetHyperSnapPoints(PDPointList pCache, double *pdRefs)
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

  double a2 = Power2(cRad.x);
  double b2 = Power2(cRad.y);
  double c2 = Power2(dr/cRad.y);
  double dDisc = (a2*c2 - b2)/(a2 + b2);

  if(dDisc < -g_dPrec) return iRes;
  if(dDisc < g_dPrec) return iRes;
  double dt = sqrt(dDisc);
  pdRefs[iRes++] = -dt;
  pdRefs[iRes++] = dt;
  return iRes;
}

