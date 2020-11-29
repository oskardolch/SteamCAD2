#include "DEvolv.hpp"
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

bool AddEvolvPoint(double x, double y, char iCtrl, PDPointList pPoints, int iInputLines)
{
  if(iCtrl > 1)
  {
    int nOffs = pPoints->GetCount(2);
    if(nOffs > 0) pPoints->SetPoint(0, 2, x, y, 2);
    else pPoints->AddPoint(x, y, 2);
    return true;
  }

  bool bRes = false;

  int nNorm = pPoints->GetCount(0);
  if(iInputLines == 1)
  {
    if((iCtrl < 1) && (nNorm < 2))
    {
      pPoints->AddPoint(x, y, 0);
      nNorm++;
    }
    bRes = (nNorm > 1);
  }
  return bRes;
}

bool BuildEvolvCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache,
  PDLine pCircle, double *pdDist)
{
  pCache->ClearAll();

  if(!pCircle[0].bIsSet) return false;

  double dr1, dr2;
  dr1 = pCircle[0].cDirection.x;
  if(dr1 < g_dPrec) return false;

  CDPoint cOrig = pCircle[0].cOrigin;
  CDPoint cPt1;

  int nNorm = pPoints->GetCount(0);

  if(nNorm > 0) cPt1 = pPoints->GetPoint(0, 0).cPoint - cOrig;
  else if(iMode == 1) cPt1 = cTmpPt.cOrigin - cOrig;
  else return false;

  dr2 = GetNorm(cPt1);
  if(dr2 < dr1 - g_dPrec) return false;
  if(dr2 < dr1 + g_dPrec) dr2 = dr1;

  CDPoint cN2 = cPt1/dr2;
  CDPoint cPt2;
  double dDir = 1.0;

  if((nNorm > 0) && (iMode == 1))
  {
    cPt2 = Rotate(cTmpPt.cOrigin - cOrig, cN2, false);
    if(cPt2.y < 0) dDir = -1.0;
  }
  else if(nNorm > 1)
  {
    cPt2 = Rotate(pPoints->GetPoint(1, 0).cPoint - cOrig, cN2, false);
    if(cPt2.y < 0) dDir = -1.0;
  }

  double dt = sqrt(Power2(dr2/dr1) - 1);

  CDPoint cb = {dr1*(cos(dt) + dt*sin(dt)), dDir*dr1*(sin(dt) - dt*cos(dt))};
  cPt2.x = cPt1.y;
  cPt2.y = -cPt1.x;

  CDPoint cN1;
  if(!Solve2x2Matrix(cPt1, cPt2, cb, &cN1)) return false;

  double dN1 = GetNorm(cN1);
  if(fabs(dN1 - 1) > g_dPrec) return false;

  pCache->AddPoint(cOrig.x, cOrig.y, 0);
  pCache->AddPoint(cN1.x, cN1.y, 0);
  pCache->AddPoint(dr1, dDir, 0);

  int nOffs = pPoints->GetCount(2);
  if((iMode == 2) || (nOffs > 0))
  {
    if(iMode == 2) cPt1 = cTmpPt.cOrigin;
    else cPt1 = pPoints->GetPoint(0, 2).cPoint;

    CDLine cPtX;
    double dDist = GetEvolvDistFromPt(cPt1, cPt1, pCache, &cPtX);
    double dDistOld = 0.0;

    if((nOffs > 0) && (iMode == 2))
    {
      cPt1 = pPoints->GetPoint(0, 2).cPoint;
      dDistOld = GetEvolvDistFromPt(cPt1, cPt1, pCache, &cPtX);
    }

    *pdDist = dDist - dDistOld;
    if(fabs(dDist) > g_dPrec) pCache->AddPoint(dDist, dDistOld, 2);
  }
  return true;
}

double GetEvolvPtProj(CDPoint cPt, CDPoint cRefPt, CDPoint cRad)
{
  double dr1 = cRad.x;
  double dDir = cRad.y;

  double dN = GetNorm(cPt);

  if(dN < dr1 + g_dPrec) return 0.0;

  CDPoint cN2 = cPt/dN;
  double dMainAngle = dDir*atan2(cN2.y, cN2.x);

  double dAng2 = acos(dr1/dN);

  double da1 = dMainAngle + dAng2;
  double da2 = dMainAngle - dAng2;

  CDPoint cPt1, cPt2;
  cPt1.x = cos(da1);
  cPt1.y = dDir*sin(da1);
  cPt2.x = cos(da2);
  cPt2.y = dDir*sin(da2);

  CDPoint cPt3, cPt4;
  cPt3 = cPt - dr1*cPt1;
  cPt4 = dr1*cPt2 - cPt;

  dN = GetNorm(cPt3);
  if(dN < g_dPrec) return 0.0;
  cPt3 /= dN;

  dN = GetNorm(cPt4);
  if(dN < g_dPrec) return 0.0;
  cPt4 /= dN;

  CDPoint cPt5 = Rotate(cRefPt - dr1*cPt1, cPt3, false);
  CDPoint cPt6 = Rotate(cRefPt - dr1*cPt2, cPt4, false);

  double dt, dt2;
  int k;

  if(cPt5.x > g_dPrec)
  {
    dt = da1;
    k = Round((cPt5.x/dr1 - dt)/M_PI/2.0);
    dt2 = dt + k*2.0*M_PI;
  }
  else
  {
    dt = da2;
    k = Round((cPt6.x/dr1 - dt)/M_PI/2.0);
    dt2 = dt + k*2.0*M_PI;
  }

  return dt2;
}

double GetEvolvDistFromPt(CDPoint cPt, CDPoint cRefPt, PDPointList pCache, PDLine pPtX)
{
  pPtX->bIsSet = false;
  pPtX->dRef = 0.0;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return 0.0;

  CDPoint cOrig, cN1, cRad;
  cOrig = pCache->GetPoint(0, 0).cPoint;
  cN1 = pCache->GetPoint(1, 0).cPoint;
  cRad = pCache->GetPoint(2, 0).cPoint;
  double dr1 = cRad.x;
  double dDir = cRad.y;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cPt1 = Rotate(cPt - cOrig, cN1, false);
  CDPoint cPt2 = Rotate(cRefPt - cOrig, cN1, false);
  double dt = GetEvolvPtProj(cPt1, cPt2, cRad);

  if(dt < g_dPrec)
  {
    double da = dr/dr1;
    cPt2.x = dr1*cos(da);
    cPt2.y = dDir*dr1*sin(da);
    pPtX->bIsSet = true;
    pPtX->cOrigin = cOrig + Rotate(cPt2, cN1, true);
    pPtX->cDirection = 0;
    pPtX->dRef = 0.0;
    return GetDist(cPt1, cPt2);
  }

  double dco = cos(dt);
  double dsi = sin(dt);

  cPt2.x = dr1*(dco + dt*dsi);
  cPt2.y = dDir*dr1*(dsi - dt*dco);

  pPtX->bIsSet = true;
  pPtX->cOrigin = cOrig + Rotate(cPt2, cN1, true);

  cPt2.x = dr1*dt*dsi;
  cPt2.y = -dDir*dr1*dt*dco;
  double dN2 = GetNorm(cPt2);
  CDPoint cDir = cPt2/dN2;

  CDPoint cPt3 = Rotate(cPt1 - cPt2, cDir, false);

  pPtX->cDirection = Rotate(cPt2, cN1, true);
  pPtX->dRef = dt;
  return cPt3.x - dr;
}

bool HasEvolvEnoughPoints(PDPointList pPoints, int iInputCircles)
{
  int nNorm = pPoints->GetCount(0);
  bool bRes = false;

  if(iInputCircles == 2) bRes = (nNorm > 1);

  return bRes;
}

double GetEvolvRadiusAtPt(CDLine cPtX, PDPointList pCache, PDLine pPtR, bool bNewPt)
{
  pPtR->bIsSet = false;
  pPtR->cOrigin = 0;
  pPtR->cDirection = 0;

  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return -1.0;

  CDPoint cOrig, cN1, cPt1;
  cOrig = pCache->GetPoint(0, 0).cPoint;
  cN1 = pCache->GetPoint(1, 0).cPoint;
  cPt1 = pCache->GetPoint(2, 0).cPoint;
  double dr1 = cPt1.x;
  double dDir = cPt1.y;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cPt2 = {dr1, 0.0};
  double dt = cPtX.dRef;
  if(dt < g_dPrec)
  {
    pPtR->bIsSet = true;
    pPtR->cOrigin = cOrig + Rotate(cPt2, cN1, true);
    pPtR->cDirection = 0;
    return 0.0;
  }

  double dco = cos(dt);
  double dsi = sin(dt);

  cPt1.x = dsi;
  cPt1.y = -dDir*dco;
  cPt2.x = dr1*dco;
  cPt2.y = dDir*dr1*dsi;

  pPtR->bIsSet = true;
  pPtR->cOrigin = cOrig + Rotate(cPt2, cN1, true);
  pPtR->cDirection = Rotate(cPt1, cN1, true);
  return dr1*dt + dr;
}

bool GetEvolvPointRefDist(double dRef, PDPointList pCache, double *pdDist)
{
  if(dRef < 0) return false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  //CDPoint cOrig, cN1;
  //cOrig = pCache->GetPoint(0, 0).cPoint;
  //cN1 = pCache->GetPoint(1, 0).cPoint;
  double dr1 = pCache->GetPoint(2, 0).cPoint.x;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  double dRef1 = dr/dr1;
  if(dr > g_dPrec) *pdDist = dr1*(Power2(dRef + dRef1) - Power2(dRef1))/2.0;
  else if(dRef > -dRef1) *pdDist = dr1*(Power2(dRef + dRef1) + Power2(dRef1))/2.0;
  else *pdDist = dr1*(Power2(dRef1) - Power2(dRef + dRef1))/2.0;

  return true;
}

double GetEvolvOffset(PDPointList pCache)
{
  int nOffs = pCache->GetCount(2);
  if(nOffs < 1) return 0.0;
  return pCache->GetPoint(0, 2).cPoint.x;
}

int GetEvolvInterLineForK(int k, double dOffset, double dr, double ds1, double ds2, PDPoint pRes)
{
  double dpi = (double)2.0*k*M_PI;
  double dh = (double)(2.0*k + 1)*M_PI + dOffset;
  if(dh < dr) return 0;

  int iRes = 0;

  double dt, ds, dsi, dco, fx, dfx, dNorm;
  CDPoint cPt1, cPt2, cPt3;
  int i;
  bool bFound;

  double dx = sqrt(Power2(dh) - Power2(dr));
  cPt1.x = -1.0 + dx;
  cPt1.y = dh;
  double dx2 = cPt1*cPt1;
  if(dx2 > 1.0 + g_dPrec)
  {
    dNorm = sqrt(dx2);
    cPt1 /= dNorm;

    dco = 1.0/dNorm;
    dsi = dco*sqrt(dx2 - 1.0);

    cPt2.x = dco;
    cPt2.y = dsi;

    cPt3 = Rotate(cPt2, cPt1, true);
    dt = atan2(cPt3.y, cPt3.x);
    if(dt < -g_dPrec) dt += 2.0*M_PI;
    dsi = sin(dt);
    dco = cos(dt);
    fx = dsi - (dpi + dt + dOffset)*dco - dr;
    dfx = (dpi + dt + dOffset)*dsi;
    i = 0;
    bFound = fabs(fx) < g_dPrec;
    while((i < 8) && !bFound && (fabs(dfx) > g_dPrec))
    {
      dt -= fx/dfx;
      dsi = sin(dt);
      dco = cos(dt);
      fx = dsi - (dpi + dt + dOffset)*dco - dr;
      dfx = (dpi + dt + dOffset)*dsi;
      bFound = fabs(fx) < g_dPrec;
      i++;
    }
    if(bFound)
    {
      ds = -(dco + (dpi + dt + dOffset)*dsi);
      if((ds > ds1 - g_dPrec) && (ds < ds2 - g_dPrec))
      {
        iRes = 1;
        pRes->x = dt + dpi + dOffset;
      }
    }
  }

  cPt1.x = -1.0 - dx;
  cPt1.y = dh;
  dx2 = cPt1*cPt1;
  if(dx2 > 1.0 + g_dPrec)
  {
    dNorm = sqrt(dx2);
    cPt1 /= dNorm;

    dco = 1.0/dNorm;
    dsi = dco*sqrt(dx2 - 1.0);

    cPt2.x = dco;
    cPt2.y = dsi;

    cPt3 = Rotate(cPt2, cPt1, true);
    dt = atan2(cPt3.y, cPt3.x);
    if(dt < -g_dPrec) dt += 2.0*M_PI;
    dsi = sin(dt);
    dco = cos(dt);
    fx = dsi - (dpi + dt + dOffset)*dco - dr;
    dfx = (dpi + dt + dOffset)*dsi;
    i = 0;
    bFound = fabs(fx) < g_dPrec;
    while((i < 8) && !bFound && (fabs(dfx) > g_dPrec))
    {
      dt -= fx/dfx;
      dsi = sin(dt);
      dco = cos(dt);
      fx = dsi - (dpi + dt + dOffset)*dco - dr;
      dfx = (dpi + dt + dOffset)*dsi;
      bFound = fabs(fx) < g_dPrec;
      i++;
    }
    if(bFound)
    {
      ds = -(dco + (dpi + dt + dOffset)*dsi);
      if((ds > ds1 - g_dPrec) && (ds < ds2 - g_dPrec))
      {
        iRes++;
        if(iRes > 1) pRes->y = dt + dpi + dOffset;
        else pRes->x = dt + dpi + dOffset;
      }
    }
  }
  return iRes;
}

int AddEvolvInterLineDir(CDPoint cPt1, CDPoint cPt2, CDPoint cOrig, CDPoint cMainDir,
  double dRad, double dDir, double dOffset1, double dOffset2, bool bReverse, PDRefList pBounds)
{
  CDPoint cLn1 = Rotate(cPt1 - cOrig, cMainDir, false);
  CDPoint cLn2 = Rotate(cPt2 - cOrig, cMainDir, false);
  cLn1.y *= dDir;
  cLn2.y *= dDir;
  CDPoint cLnDir = cLn2 - cLn1;
  double dLnNorm = GetNorm(cLnDir);
  if(dLnNorm < g_dPrec) return 0;
  cLnDir /= dLnNorm;

  double dr = dOffset1 + dOffset2;
  if(bReverse) dr *= -1.0;

  double dl1 = GetNorm(cPt1 - cOrig);
  double dl2 = GetNorm(cPt2 - cOrig);

  int k1 = 0;
  int k2 = 0;
  if(dl1 > dRad + g_dPrec)
  {
    dl1 = sqrt(Power2(dl1) - Power2(dRad));
    k1 = (int)(dl1/2.0/M_PI/dRad);
  }
  if(dl2 > dRad)
  {
    dl2 = sqrt(Power2(dl2) - Power2(dRad));
    k2 = (int)(dl2/2.0/M_PI/dRad);
  }
  if(k1 > k2)
  {
    int i = k2;
    k2 = k1 + 1;
    k1 = i;
  }
  else k2++;

  CDPoint cLnOrg = Rotate(-1.0*cLn1, cLnDir, false);
  CDPoint cEvOrgProj = {cLnOrg.x, 0.0};
  CDPoint cOrgProj = cLn1 + Rotate(cEvOrgProj, cLnDir, true);

  double dProjNorm = GetNorm(cOrgProj);
  double dv = 0.0;
  CDPoint cLnNorm;
  if(dProjNorm < g_dPrec)
  {
    cLnNorm = GetNormal(cLnDir);
    dv = atan2(cLnNorm.y, cLnNorm.x);
    if(dv < 0.0) dv += M_PI;
  }
  else
  {
    cLnNorm = cOrgProj/dProjNorm;
    dv = atan2(cLnNorm.y, cLnNorm.x);
    //if(dv < 0.0) dv += 2.0*M_PI;
  }

  double dr2 = dProjNorm/dRad;
  int k = (int)dr2/2.0/M_PI;
  if(k < k1) k1 = k;

  double dtOff = dr/dRad;
  double du = dv + dtOff;
  du -= M_PI/2.0;

  CDPoint cLnProj1 = Rotate(cLn1 - cOrgProj, cLnNorm, false);
  CDPoint cLnProj2 = Rotate(cLn2 - cOrgProj, cLnNorm, false);
  double ds1 = cLnProj1.y;
  double ds2 = cLnProj2.y;
  if(ds1 > ds2)
  {
    ds1 = cLnProj2.y;
    ds2 = cLnProj1.y;
  }
  ds1 /= dRad;
  ds2 /= dRad;

  int iRes = 0;
  int iLoc;
  CDPoint cRoots;

  if(bReverse) k1 = 0;

  double dt;
  for(int i = k1; i <= k2; i++)
  {
    iLoc = GetEvolvInterLineForK(i, du, dr2, ds1, ds2, &cRoots);
    if(bReverse)
    {
      if(iLoc > 0)
      {
        dt = dtOff - cRoots.x;
        if((dt > -g_dPrec) && (dt < -dOffset1/dRad + g_dPrec))
        {
          pBounds->AddPoint(dt);
          iRes++;
        }
      }
      if(iLoc > 1)
      {
        dt = dtOff - cRoots.y;
        if((dt > -g_dPrec) && (dt < -dOffset1/dRad + g_dPrec))
        {
          pBounds->AddPoint(dt);
          iRes++;
        }
      }
//printf("%i, %i, %i, %f, %f - %f, %f\n", i, iRes, iLoc, dtOff - cRoots.x, dtOff - cRoots.y, dtOff, dOffset1/dRad);
    }
    else
    {
      if(iLoc > 0) pBounds->AddPoint(cRoots.x - dtOff);
      if(iLoc > 1) pBounds->AddPoint(cRoots.y - dtOff);
      iRes += iLoc;
    }
  }
  return iRes;
}

int AddEvolvInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return 0;

  CDPoint cOrig, cN1, cRad;
  cOrig = pCache->GetPoint(0, 0).cPoint;
  cN1 = pCache->GetPoint(1, 0).cPoint;
  cRad = pCache->GetPoint(2, 0).cPoint;
  double dr1 = cRad.x;
  if(dr1 < g_dPrec) return 0;

  int iRes = 0;
  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0)
  {
    dr = pCache->GetPoint(0, 2).cPoint.x;
    if(dr < -g_dPrec)
      iRes += AddEvolvInterLineDir(cPt1, cPt2, cOrig, cN1, cRad.x, -cRad.y, dr, dOffset, true, pBounds);
  }
  iRes += AddEvolvInterLineDir(cPt1, cPt2, cOrig, cN1, cRad.x, cRad.y, dr, dOffset, false, pBounds);
  return iRes;
}

CDPoint EvolvFunc(void *pvData, double dt)
{
  PDPoint pEvPts = (PDPoint)pvData;
  CDPoint cRad = pEvPts[0];
  double dt1 = dt + pEvPts[1].x;
  double dco = cos(dt1);
  double dsi = sin(dt1);
  CDPoint cRes = {dco + dt1*dsi, cRad.y*(dsi - dt1*dco)};
  return cRad.x*cRes;
}

CDPoint EvolvFuncDer(void *pvData, double dt)
{
  PDPoint pEvPts = (PDPoint)pvData;
  CDPoint cRad = pEvPts[0];
  double dt1 = dt + pEvPts[1].x;
  double dco = cos(dt1);
  double dsi = sin(dt1);
  CDPoint cRes = {dco, cRad.y*dsi};
  return cRad.x*cRes;
}

void AddEvolvSegment(double d1, double d2, double dExt, PDPointList pCache, PDPrimObject pPrimList)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return;

  CDPoint cOrig, cN1, cRad;
  cOrig = pCache->GetPoint(0, 0).cPoint;
  cN1 = pCache->GetPoint(1, 0).cPoint;
  cRad = pCache->GetPoint(2, 0).cPoint;
  double dr1 = cRad.x;
  CDPoint cBreak = {-1.0, -1.0};

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0)
  {
    dr = pCache->GetPoint(0, 2).cPoint.x;
    if(dr < -g_dPrec) cBreak.x = -dr/dr1;
  }
  dr += dExt;

  double dRef1 = 0.0;

  if(fabs(dr) > g_dPrec)
  {
    dRef1 = dr/dr1;
    CDPoint cPt1;
    cPt1.x = cos(dRef1);
    cPt1.y = -cRad.y*sin(dRef1);
    cN1 = Rotate(cN1, cPt1, true);
    //dRef += dRef1;
  }

  double dt1, dt2;
  
  if(dr > g_dPrec)
  {
    dt1 = sqrt(Power2(dRef1) + 2.0*d1/dr1) - dRef1;
    dt2 = sqrt(Power2(dRef1) + 2.0*d2/dr1) - dRef1;
  }
  else
  {
    if(d1 > dr1*Power2(dRef1)/2.0) 
      dt1 = sqrt(2.0*d1/dr1 - Power2(dRef1)) - dRef1;
    else dt1 = -dRef1 - sqrt(Power2(dRef1) - 2.0*d1/dr1);
    if(d2 > dr1*Power2(dRef1)/2.0)
      dt2 = sqrt(2.0*d2/dr1 - Power2(dRef1)) - dRef1;
    else dt2 = -dRef1 - sqrt(Power2(dRef1) - 2.0*d2/dr1);
  }

  CDPoint cEvPts[2];
  cEvPts[0] = cRad;
  cEvPts[1].x = dRef1;
  cEvPts[1].y = dExt/dr1;
  PDPrimObject pTmpPrim = new CDPrimObject();
  AddCurveSegment(cEvPts, 0.0, cBreak, EvolvFunc, EvolvFuncDer, dt1, dt2, M_PI/8.0, 0, pTmpPrim);
  RotatePrimitives(pTmpPrim, pPrimList, cOrig, cN1);
  delete pTmpPrim;
}

bool GetEvolvRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt)
{
  if(dRef < 0) return false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cN1 = pCache->GetPoint(1, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(2, 0).cPoint;
  double dr1 = cRad.x;
  double dDir = cRad.y;

  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cPt1;

  if(fabs(dr) > g_dPrec)
  {
    double da = dr/dr1;
    cPt1.x = cos(da);
    cPt1.y = -dDir*sin(da);
    cN1 = Rotate(cN1, cPt1, true);
    dRef += da;
  }

  double dco = cos(dRef);
  double dsi = sin(dRef);

  cPt1.x = dr1*(dco + dRef*dsi);
  cPt1.y = dDir*dr1*(dsi - dRef*dco);

  *pPt = cOrig + Rotate(cPt1, cN1, true);
  return true;
}

bool GetEvolvRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache)
{
  if(iMode != 2) return false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cN1 = pCache->GetPoint(1, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(2, 0).cPoint;
  double dr1 = cRad.x;
  double dDir = cRad.y;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.y;

  double dRad = dr + dRestrictValue;
  double da = dRad/dr1;

  CDPoint cPt1 = Rotate(cPt - cOrig, cN1, false);
  double dt = GetEvolvPtProj(cPt1, cPt1, cRad) + da;

  double dco = cos(dt);
  double dsi = sin(dt);

  cPt1.x = dr1*(dco + dt*dsi);
  cPt1.y = dDir*dr1*(dsi - dt*dco);

  CDPoint cPt2 = {cos(da), sin(da)};
  cN1 = Rotate(cN1, cPt2, false);

  *pSnapPt = cOrig + Rotate(cPt1, cN1, true);
  return true;
}

bool GetEvolvRefDir(double dRef, PDPointList pCache, PDPoint pPt)
{
  if(dRef < 0) return false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cN1 = pCache->GetPoint(1, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(2, 0).cPoint;
  double dr1 = cRad.x;
  double dDir = cRad.y;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cPt1;

  if(fabs(dr) > g_dPrec)
  {
    double da = dr/dr1;
    cPt1.x = cos(da);
    cPt1.y = -dDir*sin(da);
    cN1 = Rotate(cN1, cPt1, true);
    dRef += da;
  }

  double dco = cos(dRef);
  double dsi = sin(dRef);

  cPt1.x = dRef*dco;
  cPt1.y = dRef*dsi;
  double dNorm = GetNorm(cPt1);
  if(dNorm < g_dPrec) return false;

  cPt1 /= dNorm;

  *pPt = Rotate(cPt1, cN1, true);
  return true;
}

bool GetEvolvReference(double dDist, PDPointList pCache, double *pdRef)
{
  if(dDist < -g_dPrec) return false;
  if(dDist < g_dPrec) dDist = 0.0;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  //CDPoint cN1 = pCache->GetPoint(1, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(2, 0).cPoint;
  double dr1 = cRad.x;
  //double dDir = cRad.y;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  double dRef1 = dr/dr1;
  /*if(dr > g_dPrec) *pdDist = dr1*(Power2(dRef + dRef1) - Power2(dRef1))/2.0;
  else if(dRef > -dRef1) *pdDist = dr1*(Power2(dRef + dRef1) + Power2(dRef1))/2.0;
  else *pdDist = dr1*(Power2(dRef1) - Power2(dRef + dRef1))/2.0;*/
  if(dr > g_dPrec) *pdRef = sqrt(Power2(dRef1) + 2.0*dDist/dr1) - dRef1;
  else if(dDist > dr1*Power2(dRef1)/2.0) *pdRef = sqrt(2.0*dDist/dr1 - Power2(dRef1)) - dRef1;
  else *pdRef = -dRef1 - sqrt(Power2(dRef1) - 2.0*dDist/dr1);

  return true;
}

