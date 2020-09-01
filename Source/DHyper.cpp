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

CDPoint HyperFunc(double da, double db, double dt)
{
  double dv = sqrt(1.0 + Power2(dt));
  CDPoint cRes;
  cRes.x = da*dv;
  cRes.y = db*dt;
  return cRes;
}

CDPoint HyperFuncDer(double da, double db, double dt)
{
  double dv = sqrt(1.0 + Power2(dt));
  CDPoint cRes;
  cRes.x = -da*dt/dv;
  cRes.y = -db;
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

bool PtInList(double du, int iSize, double *pdList)
{
  int i = 0;
  bool bFound = false;
  while(!bFound && (i < iSize))
  {
    bFound = fabs(pdList[i++] - du) < g_dPrec;
  }
  return bFound;
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
      if(!PtInList(du, iRes, pdRoots)) pdRoots[iRes++] = du;
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

bool AddHyperPoint(double x, double y, char iCtrl, double dRestrictVal, PDPointList pPoints, int iInputLines)
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

bool BuildHyperCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache,
  PDLine pLines, double *pdDist)
{
/*double dref1, dref2;
double ddist1;
CDPoint cPt8;
for(int i = 0; i < 16; i++)
{
  dref1 = 64.0*i/16.0;
  ddist1 = GetCurveDistAtRef(1.0, 2.0, 10.0, -1.0, dref1, HyperFunc, HyperFuncDer, 0.5, 1);
  cPt8 = GetCurveRefAtDist(1.0, 2.0, 10.0, -1.0, ddist1, HyperFunc, HyperFuncDer, 0.5, 1);
  dref2 = GetHyperBoundProj(1.0, 2.0, 10.0, cPt8, cPt8, false);
  printf("%d - %f, %f, %f\n", i, dref1, ddist1, dref2);
}*/

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

  //if((iMode == 2) && (cTmpPt.cDirection.x > 0.5))
  //{
  //  dr = GetHyperBreakAngle(dr1, dr2, -cTmpPt.cDirection.y);
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
    double dDist = 0.0;
    double dDistOld = 0.0;
    int iSrchMask = 0;

    if(iMode == 2)
    {
      cPt1 = cTmpPt.cOrigin;
      if(cTmpPt.cDirection.x < -0.5) iSrchMask = 2;
    }
    else if(nOffs2) cPt1 = pPoints->GetPoint(0, 2).cPoint;
    else if(nOffs3 > 0)
    {
      cPt1 = pPoints->GetPoint(0, 3).cPoint;
      iSrchMask = 2;
    }

    if((iMode == 2) || (nOffs4 == 0))
      dDist = GetHyperDistFromPt(cPt1, cPt1, iSrchMask, pCache, &cPtX);

    if(iMode == 2)
    {
      if(nOffs4 > 0)
        dDistOld = pPoints->GetPoint(0, 4).cPoint.x;
      else if(nOffs2 > 0)
      {
        cPt1 = pPoints->GetPoint(0, 2).cPoint;
        dDistOld = GetHyperDistFromPt(cPt1, cPt1, 0, pCache, &cPtX);
      }
      else if(nOffs3 > 0)
      {
        cPt1 = pPoints->GetPoint(0, 3).cPoint;
        dDistOld = GetHyperDistFromPt(cPt1, cPt1, 2, pCache, &cPtX);
      }
      if(cTmpPt.cDirection.x > 0.5) dDist = dDistOld + cTmpPt.cDirection.y;
    }
    else if(nOffs4 > 0) dDist = pPoints->GetPoint(0, 4).cPoint.x;

    if(pdDist) *pdDist = dDist - dDistOld;
    if((fabs(dDist) > g_dPrec) || (fabs(dDistOld) > g_dPrec)) pCache->AddPoint(dDist, dDistOld, 2);

    dr = GetHyperBreakAngle(dr1, dr2, -dDist);
    if(dr > -0.5) pCache->AddPoint(dr, 0.0, 4);
  }
  return true;
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

double GetHyperDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX) //, PDRefPoint pBounds)
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

bool GetHyperPointRefDist(double dRef, PDPointList pCache, double *pdDist)
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

  *pdDist = GetCurveDistAtRef(cRad.x, cRad.y, dr, dBreak, fabs(dRef), HyperFunc, HyperFuncDer, 0.5, 1);
  if(dRef < 0.0) *pdDist *= -1.0;
  return true;
}

double GetHyperPointAtDist(double da, double db, double dr, double dBreak, double dDist)
{
  CDPoint cPt1 = GetCurveRefAtDist(da, db, dr, dBreak, fabs(dDist), HyperFunc, HyperFuncDer, 0.5, 1);
  double dRes = GetHyperBoundProj(da, db, dr, cPt1, cPt1, false);
  if(dDist < 0.0) dRes *= -1.0;
  return dRes;
}

void AddHyperSegment(double d1, double d2, double dExt, PDPointList pCache, PDPrimObject pPrimList)
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

  double dy1 = GetHyperPointAtDist(cRad.x, cRad.y, dr, dBreak, d1);
  double dy2 = GetHyperPointAtDist(cRad.x, cRad.y, dr, dBreak, d2);

  PDPrimObject pTmpPrim = new CDPrimObject();
  AddCurveSegment(cRad.x, cRad.y, dr, dBreak, HyperFunc, HyperFuncDer, dy1, dy2, 0.5, 1, pTmpPrim);
  RotatePrimitives(pTmpPrim, pPrimList, cOrig, cNorm);
  delete pTmpPrim;
}

bool GetHyperRefPoint(double dRef, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return false;

  CDPoint cOrig, cRad, cNorm;

  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cNorm = pCache->GetPoint(2, 0).cPoint;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

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

double GetHyperOffset(PDPointList pCache)
{
  int nOffs = pCache->GetCount(2);
  if(nOffs < 1) return 0.0;
  return pCache->GetPoint(0, 2).cPoint.x;
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
  return true;
}

bool GetHyperReference(double dDist, PDPointList pCache, double *pdRef)
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

  *pdRef = GetHyperPointAtDist(cRad.x, cRad.y, dr, dBreak, dDist);
  return true;
}

double HyperPtProjFunc(double da, double db, double dOffset, CDPoint cPt, CDPoint cStart, CDPoint cEnd)
{
  double pProjs[4];
  int iRoots = GetHyperPtProj(da, db, cPt, pProjs);

  double pDists[4];
  bool pValid[4];

  CDPoint cNorm, cProjPt;
  double du, dc, dNorm;

  for(int i = 0; i < iRoots; i++)
  {
    du = pProjs[i];
    pValid[i] = GetRefInUboundSeg(du, cStart, cEnd);
    dc = sqrt(1.0 + Power2(du));
    cProjPt.x = da*dc;
    cProjPt.y = db*du;
    cNorm.x = -db;
    cNorm.y = da*du/dc;
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

  double dBreak = -1.0;
  if(pCache->GetCount(4) > 0) dBreak = pCache->GetPoint(0, 4).cPoint.x;

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
  if(dBreak < -g_dPrec)
  {
    iRes = AddCurveInterLine(cRad.x, cRad.y, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {0.0, 0.0}, {0.0, 0.0}, cLn1, cLn2, pBounds);
  }
  else if(dBreak < g_dPrec)
  {
    iRes = AddCurveInterLine(cRad.x, cRad.y, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {0.0, 0.0}, {1.0, 0.0}, cLn1, cLn2, pBounds);
    iRes += AddCurveInterLine(cRad.x, cRad.y, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {1.0, 0.0}, {0.0, 0.0}, cLn1, cLn2, pBounds);
  }
  else
  {
    iRes = AddCurveInterLine(cRad.x, cRad.y, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {0.0, 0.0}, {1.0, -dBreak}, cLn1, cLn2, pBounds);
    iRes += AddCurveInterLine(cRad.x, cRad.y, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {1.0, -dBreak}, {1.0, dBreak}, cLn1, cLn2, pBounds);
    iRes += AddCurveInterLine(cRad.x, cRad.y, dr, HyperFunc, HyperFuncDer, HyperPtProjFunc,
      {bTangent ? 1.0 : 0.0, dTangent}, {1.0, dBreak}, {0.0, 0.0}, cLn1, cLn2, pBounds);
  }
  return iRes;
}

int GetHyperNumParts(PDPointList pCache, PDRefPoint pBounds)
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

bool HyperRemovePart(bool bDown, PDPointList pCache, PDRefPoint pBounds)
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
    if(RefInOpenBounds(pBounds, -dAng) > 2)
    {
      pBounds[1].bIsSet = true;
      pBounds[1].dRef = -dAng;
    }
    else if(RefInOpenBounds(pBounds, dAng) > 2)
    {
      pBounds[1].bIsSet = true;
      pBounds[1].dRef = dAng;
    }
    else return false;
  }
  else
  {
    if(RefInOpenBounds(pBounds, -dAng) > 2)
    {
      pBounds[0].bIsSet = true;
      pBounds[0].dRef = -dAng;
    }
    else if(RefInOpenBounds(pBounds, dAng) > 2)
    {
      pBounds[0].bIsSet = true;
      pBounds[0].dRef = dAng;
    }
    else return false;
  }
  return true;
}

