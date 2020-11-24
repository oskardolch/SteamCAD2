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

/*void GetEvolvBounds(CDPoint cOrig, PDRect pRect, PDPoint pBounds)
{
    CDPoint cPt1 = {pRect->cPt1.x, pRect->cPt1.y};
    CDPoint cPt2 = {pRect->cPt1.x, pRect->cPt2.y};
    CDPoint cPt3 = {pRect->cPt2.x, pRect->cPt2.y};
    CDPoint cPt4 = {pRect->cPt2.x, pRect->cPt1.y};
    bool bIsInside = (cOrig.x > cPt1.x - g_dPrec) && (cOrig.x < cPt3.x + g_dPrec) &&
        (cOrig.y > cPt1.y - g_dPrec) && (cOrig.y < cPt3.y + g_dPrec);

    CDLine cPtX;
    double d1 = GetPtDistFromLineSeg(cOrig, cPt1, cPt2, &cPtX);
    double d2 = GetPtDistFromLineSeg(cOrig, cPt2, cPt3, &cPtX);
    double d3 = GetPtDistFromLineSeg(cOrig, cPt3, cPt4, &cPtX);
    double d4 = GetPtDistFromLineSeg(cOrig, cPt4, cPt1, &cPtX);

    if(bIsInside) pBounds->x = 0.0;
    else
    {
        pBounds->x = d1;
        if(pBounds->x > d2) pBounds->x = d2;
        if(pBounds->x > d3) pBounds->x = d3;
        if(pBounds->x > d4) pBounds->x = d4;
    }

    d1 = GetDist(cOrig, cPt1);
    d2 = GetDist(cOrig, cPt2);
    d3 = GetDist(cOrig, cPt3);
    d4 = GetDist(cOrig, cPt4);

    pBounds->y = d1;
    if(pBounds->y < d2) pBounds->y = d2;
    if(pBounds->y < d3) pBounds->y = d3;
    if(pBounds->y < d4) pBounds->y = d4;
}

int AddEvolvSegWithBounds(double dt1, double dt2, CDPoint cOrig, CDPoint cMainDir, CDPoint cRad,
    PDPrimObject pPrimList, PDRect pRect)
{
    double dr1 = cRad.x;
    double dDir = cRad.y;

    double dtInc = M_PI/8.0;
    double dtTot = dt2 - dt1;
    int iSegs = (int)dtTot/dtInc;
    if(iSegs < 1) iSegs = 1;
    dtInc = dtTot/(double)iSegs;

    double dco = cos(dt1);
    double dsi = sin(dt1);

    CDPoint cHypPts[5];
    cHypPts[4].x = dr1*(dco + dt1*dsi);
    cHypPts[4].y = dDir*dr1*(dsi - dt1*dco);

    CDPoint cPt1, cPt2;
    double dt;

    cPt2.x = dco;
    cPt2.y = dDir*dsi;

    CDPrimitive cPrim, cTmpPrim;
    int iRes = -1;
    int iCurRes;

    for(int i = 0; i < iSegs; i++)
    {
        cHypPts[0] = cHypPts[4];
        cPt1 = cPt2;

        for(int j = 1; j < 5; j++)
        {
            dt = dt1 + dtInc*j/4.0;
            dco = cos(dt);
            dsi = sin(dt);
            cHypPts[j].x = dr1*(dco + dt*dsi);
            cHypPts[j].y = dDir*dr1*(dsi - dt*dco);
        }

        cPt2.x = dco;
        cPt2.y = dDir*dsi;

        if(ApproxLineSeg(5, cHypPts, &cPt1, &cPt2, &cTmpPrim) > -0.5)
        {
            cPrim.iType = 5;
            cPrim.cPt1 = cOrig + Rotate(cTmpPrim.cPt1, cMainDir, true);
            cPrim.cPt2 = cOrig + Rotate(cTmpPrim.cPt2, cMainDir, true);
            cPrim.cPt3 = cOrig + Rotate(cTmpPrim.cPt3, cMainDir, true);
            cPrim.cPt4 = cOrig + Rotate(cTmpPrim.cPt4, cMainDir, true);
        }
        else
        {
            cPrim.iType = 1;
            cPrim.cPt1 = cOrig + Rotate(cHypPts[0], cMainDir, true);
            cPrim.cPt2 = cOrig + Rotate(cHypPts[4], cMainDir, true);
        }

        iCurRes = CropPrimitive(cPrim, pRect, pPrimList);
        dt1 += dtInc;
        if(iRes < 0) iRes = iCurRes;
        else if(iRes != iCurRes) iRes = 1;
    }
    return iRes;
}

int AddEvolvSegQuadsWithBounds(double dt1, double dt2, CDPoint cOrig, CDPoint cMainDir, CDPoint cRad,
    PDPrimObject pPrimList, PDRect pRect)
{
    double dr1 = cRad.x;
    double dDir = cRad.y;

    double dtInc = M_PI/8.0;
    double dtTot = dt2 - dt1;
    int iSegs = (int)dtTot/dtInc;
    if(iSegs < 1) iSegs = 1;
    dtInc = dtTot/(double)iSegs;

    double dco = cos(dt1);
    double dsi = sin(dt1);

    CDPrimitive cPrim, cTmpPrim;
    cTmpPrim.cPt3.x = dr1*(dco + dt1*dsi);
    cTmpPrim.cPt3.y = dDir*dr1*(dsi - dt1*dco);

    CDPoint cDir1, cDir2;
    double dt;

    cDir2.x = dco;
    cDir2.y = dDir*dsi;

    int iRes = -1;
    int iCurRes;

    cPrim.iType = 4;

    for(int i = 0; i < iSegs; i++)
    {
        cTmpPrim.cPt1 = cTmpPrim.cPt3;
        cDir1 = cDir2;

        dt = dt1 + dtInc;
        dco = cos(dt);
        dsi = sin(dt);
        cTmpPrim.cPt3.x = dr1*(dco + dt*dsi);
        cTmpPrim.cPt3.y = dDir*dr1*(dsi - dt*dco);

        cDir2.x = dco;
        cDir2.y = dDir*dsi;

        LineXLine(cTmpPrim.cPt1, cDir1, cTmpPrim.cPt3, cDir2, &cTmpPrim.cPt2);

        cPrim.cPt1 = cOrig + Rotate(cTmpPrim.cPt1, cMainDir, true);
        cPrim.cPt2 = cOrig + Rotate(cTmpPrim.cPt2, cMainDir, true);
        cPrim.cPt3 = cOrig + Rotate(cTmpPrim.cPt3, cMainDir, true);

        iCurRes = CropPrimitive(cPrim, pRect, pPrimList);
        dt1 += dtInc;
        if(iRes < 0) iRes = iCurRes;
        else if(iRes != iCurRes) iRes = 1;
    }
    return iRes;
}

int BuildEvolvPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, PDPointList pPoints,
    PDPointList pCache, PDPrimObject pPrimList, PDLine pCircle, PDRefPoint pBounds, double dOffset,
    double *pdDist, PDPoint pDrawBnds, bool bQuadsOnly)
{
    if(iMode > 0) BuildEvolvCache(cTmpPt, iMode, pPoints, pCache, pCircle, pdDist);

    int iCnt = pCache->GetCount(0);
    if(iCnt < 3) return 0;

    CDPoint cOrig, cN1, cRad;
    cOrig = pCache->GetPoint(0, 0).cPoint;
    cN1 = pCache->GetPoint(1, 0).cPoint;
    cRad = pCache->GetPoint(2, 0).cPoint;
    double dr1 = cRad.x;

    double dr = dOffset;
    int nOffs = pCache->GetCount(2);
    if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

    CDPoint cBounds;
    GetEvolvBounds(cOrig, pRect, &cBounds);

    double dtMin, dtMax;
    if(cBounds.x < dr1 + g_dPrec) dtMin = 0.0;
    else dtMin = sqrt(Power2(cBounds.x/dr1) - 1);
    if(cBounds.y < dr1 + g_dPrec) dtMax = -1.0;
    else dtMax = sqrt(Power2(cBounds.y/dr1) - 1);

    if(dtMax < -g_dPrec) return 0;

    double dtInc = M_PI/4.0;
    double dt1 = dtMin - dtInc;
    if(dt1 < 0) dt1 = 0;
    dtMax += dtInc;

    pDrawBnds->x = 0.0;
    pDrawBnds->y = dr1*Power2(dtMax)/2.0;

    if(pBounds[0].bIsSet)
    {
        if(dt1 < pBounds[0].dRef) dt1 = pBounds[0].dRef;
    }
    if(pBounds[1].bIsSet)
    {
        if(dtMax > pBounds[1].dRef) dtMax = pBounds[1].dRef;
    }

    if(fabs(dr) > g_dPrec)
    {
        double dDir = -1.0*cRad.y;
        double da = dr/dr1;
        CDPoint cPt1;
        cPt1.x = cos(da);
        cPt1.y = dDir*sin(da);
        cN1 = Rotate(cN1, cPt1, true);

        if(pBounds[0].bIsSet) dt1 += da;
        dtMax += da;
    }

    int iRes = 2;
    if(bQuadsOnly)
        iRes = AddEvolvSegQuadsWithBounds(dt1, dtMax, cOrig, cN1, cRad, pPrimList, pRect);
    else
        iRes = AddEvolvSegWithBounds(dt1, dtMax, cOrig, cN1, cRad, pPrimList, pRect);
    if(iRes < 0) iRes = 0;
    return iRes;
}*/

/*double GetEvolvPtProjDown(CDPoint cPt, CDPoint cRad, int *piPeriod)
{
  *piPeriod = 0;
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
}*/

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

/*void AddEvolvSegment(double d1, double d2, PDPointList pCache, PDPrimObject pPrimList, PDRect pRect)
{
    int iCnt = pCache->GetCount(0);
    if(iCnt < 3) return;

    CDPoint cOrig, cN1, cRad;
    cOrig = pCache->GetPoint(0, 0).cPoint;
    cN1 = pCache->GetPoint(1, 0).cPoint;
    cRad = pCache->GetPoint(2, 0).cPoint;
    double dr1 = cRad.x;

    double dr = 0.0;
    int nOffs = pCache->GetCount(2);
    if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

    double dt1 = sqrt(2.0*d1/dr1);
    double dt2 = sqrt(2.0*d2/dr1);

    if(fabs(dr) > g_dPrec)
    {
        double dDir = -1.0*cRad.y;
        double da = dr/dr1;
        CDPoint cPt1;
        cPt1.x = cos(da);
        cPt1.y = dDir*sin(da);
        cN1 = Rotate(cN1, cPt1, true);
    }

    AddEvolvSegWithBounds(dt1, dt2, cOrig, cN1, cRad, pPrimList, pRect);
}*/

double GetEvolvOffset(PDPointList pCache)
{
  int nOffs = pCache->GetCount(2);
  if(nOffs < 1) return 0.0;
  return pCache->GetPoint(0, 2).cPoint.x;
}

int GetEvolvInterLineForK(int k, double dOffset, double dr, double ds1, double ds2, PDPoint pRes)
{
  double dh = (double)(2.0*k + 1)*M_PI + dOffset;
  if(dh < dr) return 0;

  int iRes = 0;
  double dpi = (double)2.0*k*M_PI;

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
printf("GetEvolvInterLineForK 0.5: %f, %f\n", dt, dh);
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
printf("GetEvolvInterLineForK 1: %d, %d, %f, %f - %f, %f - %f, %f\n", i, k, fx, dt, ds1, ds2, dco + (dpi + dt + dOffset)*dsi, dx);
    if(bFound)
    {
      ds = dco + (dpi + dt + dOffset)*dsi;
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
printf("GetEvolvInterLineForK 1.5: %f, %f\n", dt, dOffset);
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
printf("GetEvolvInterLineForK 2: %d, %d, %f, %f - %f, %f - %f\n", i, k, fx, dt, ds1, ds2, dco + (dpi + dt + dOffset)*dsi);
    if(bFound)
    {
      ds = dco + (dpi + dt + dOffset)*dsi;
      if((ds > ds1 - g_dPrec) && (ds < ds2 - g_dPrec))
      {
        iRes++;
        if(iRes > 1) pRes->y = dt + dpi + dOffset;
        else pRes->x = dt + dpi + dOffset;
      }
    }
  }
//if(iRes > 0)
//printf("GetEvolvInterLineForK: %d, %d - %f, %f\n", iRes, k, pRes->x, pRes->y);
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

  CDPoint cLn1 = Rotate(cPt1 - cOrig, cN1, false);
  CDPoint cLn2 = Rotate(cPt2 - cOrig, cN1, false);
  CDPoint cLnDir = cLn2 - cLn1;
  double dLnNorm = GetNorm(cLnDir);
  if(dLnNorm < g_dPrec) return 0;
  cLnDir /= dLnNorm;

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  //double du = cRad.y*(atan2(cLnDir.y, cLnDir.x) + dr/dr1);
  double du = cRad.y*atan2(cLnDir.y, cLnDir.x);

  double dl1 = GetNorm(cPt1 - cOrig);
  double dl2 = GetNorm(cPt2 - cOrig);

  int k1 = 0;
  int k2 = 0;
  if(dl1 > dr1 + g_dPrec)
  {
    dl1 = sqrt(Power2(dl1) - Power2(dr1));
    k1 = (int)(dl1/2.0/M_PI/dr1);
  }
  if(dl2 > dr1)
  {
    dl2 = sqrt(Power2(dl2) - Power2(dr1));
    k2 = (int)(dl2/2.0/M_PI/dr1);
  }
  if(k1 > k2)
  {
    int i = k2;
    k2 = k1 + 1;
    k1 = i;
  }
  else k2++;
printf("AddEvolvInterLine 1: %d, %d - %f, %f\n", k1, k2, cRad.x, cRad.y);

  CDPoint cLnOrg = Rotate(-1.0*cLn1, cLnDir, false);
  double dr2 = cRad.y*cLnOrg.y/dr1;
  if(dr2 < g_dPrec) dr2 *= -1.0;
  else du -= M_PI;
  int k = (int)dr2/2.0/M_PI;
  if(k < k1) k1 = k;
  int iRes = 0;
  int iLoc;
  CDPoint cRoots;
  double ds1 = -cLnOrg.x;
  double ds2 = ds1 + dLnNorm;
  ds1 /= dr1;
  ds2 /= dr1;

  for(int i = k1; i <= k2; i++)
  {
    iLoc = GetEvolvInterLineForK(i, du, dr2, ds1, ds2, &cRoots);
    if(iLoc > 0) pBounds->AddPoint(cRoots.x);
    if(iLoc > 1) pBounds->AddPoint(cRoots.y);
    iRes += iLoc;
  }
//printf("AddEvolvInterLine 2: %d\n", iRes);
  return iRes;
}

CDPoint EvolvFunc(void *pvData, double dt)
{
  CDPoint cRad = *(PDPoint)pvData;
  double dco = cos(dt);
  double dsi = sin(dt);
  CDPoint cRes = {dco + dt*dsi, cRad.y*(dsi - dt*dco)};
  return cRad.x*cRes;
}

CDPoint EvolvFuncDer(void *pvData, double dt)
{
  CDPoint cRad = *(PDPoint)pvData;
  double dco = cos(dt);
  double dsi = sin(dt);
  CDPoint cRes = {dt*dco, cRad.y*dt*dsi};
  return cRad.x*cRes;
}

void AddEvolvSegment(double d1, double d2, double dExt, PDPointList pCache, PDPrimObject pPrimList)
{
//printf("AddEvolvSegment: %f, %f\n", d1, d2);
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return;

/*for(int i = 0 ; i < 10; i++)
{
double dDist = (double)10.0*i;
double dRef, dNewDist, dNewRef;
GetEvolvReference(dDist, pCache, &dRef);
GetEvolvPointRefDist(dRef, pCache, &dNewDist);
GetEvolvReference(dNewDist, pCache, &dNewRef);
printf("Check: %d - %f, %f, %f, %f\n", i, dDist, dRef, dNewDist, dNewRef);
}*/

  CDPoint cOrig, cN1, cRad;
  cOrig = pCache->GetPoint(0, 0).cPoint;
  cN1 = pCache->GetPoint(1, 0).cPoint;
  cRad = pCache->GetPoint(2, 0).cPoint;
  double dr1 = cRad.x;

  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  double dt1 = sqrt(2.0*d1/dr1);
  double dt2 = sqrt(2.0*d2/dr1);

  PDPrimObject pTmpPrim = new CDPrimObject();
  AddCurveSegment(&cRad, dr, {-1.0, -1.0}, EvolvFunc, EvolvFuncDer, dt1, dt2, M_PI/8.0, 0, pTmpPrim);
  RotatePrimitives(pTmpPrim, pPrimList, cOrig, cN1);
  delete pTmpPrim;


  /*if(fabs(dr) > g_dPrec)
  {
    double dDir = -1.0*cRad.y;
    double da = dr/dr1;
    CDPoint cPt1;
    cPt1.x = cos(da);
    cPt1.y = dDir*sin(da);
    cN1 = Rotate(cN1, cPt1, true);
  }

  AddEvolvSegWithBounds(dt1, dt2, cOrig, cN1, cRad, pPrimList, pRect);*/
}

bool GetEvolvRefPoint(double dRef, PDPointList pCache, PDPoint pPt)
{
  if(dRef < 0) return false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
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

  /**pdRef = sqrt(2.0*dDist/dr1);

  if(fabs(dr) > g_dPrec)
  {
    double da = dr/dr1;
    *pdRef -= da;
  }*/

  double dRef1 = dr/dr1;
  /*if(dr > g_dPrec) *pdDist = dr1*(Power2(dRef + dRef1) - Power2(dRef1))/2.0;
  else if(dRef > -dRef1) *pdDist = dr1*(Power2(dRef + dRef1) + Power2(dRef1))/2.0;
  else *pdDist = dr1*(Power2(dRef1) - Power2(dRef + dRef1))/2.0;*/
  if(dr > g_dPrec) *pdRef = sqrt(Power2(dRef1) + 2.0*dDist/dr1) + dRef1;
  else if(dDist > dr1*Power2(dRef1)/2.0) *pdRef = sqrt(2.0*dDist/dr1 - Power2(dRef1)) - dRef1;
  else *pdRef = sqrt(Power2(dRef1) - 2.0*dDist/dr1) - dRef1;

  return true;
}

