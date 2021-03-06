#include "DArcElps.hpp"
#include "DMath.hpp"
#include <math.h>
#include <stddef.h>
#include "DPrimitive.hpp"

#include <stdio.h>

// for debugging purpose only
/*#include <windows.h>
#include <commctrl.h>
#include <wchar.h>
extern HWND g_hStatus;*/
// -----

bool AddArcElpsPoint(double x, double y, char iCtrl, PDPointList pPoints, int iInputLines)
{
  bool bRes = false;
  int nNorm = pPoints->GetCount(0);

  if(iInputLines == 2)
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

CDPoint DAEFunc(CDPoint cPt1, CDPoint cPt2, CDPoint cPtSol)
{
  CDPoint cN1 = {cPt1.x - cPtSol.x, cPt1.y};
  CDPoint cN2 = {cPt2.x, cPt2.y + cPtSol.y};
  double dN1 = GetNorm(cN1);
  double dN2 = GetNorm(cN2);
  double dSol2 = cPtSol*cPtSol;
  CDPoint cRes;
  cRes.x = sqrt(dSol2) + dN1 - dN2;
  cRes.y = dSol2 + cPtSol.x*dN1 - cPtSol.y*dN2;
  return cRes;
}

CDPoint DAEFuncX(CDPoint cPt1, CDPoint cPt2, CDPoint cPtSol)
{
  CDPoint cN1 = {cPt1.x - cPtSol.x, cPt1.y};
  //CDPoint cN2 = {cPt2.x, cPt2.y + cPtSol.y};
  double dN1 = GetNorm(cN1);
  //double dN2 = GetNorm(cN2);
  double dSol2 = cPtSol*cPtSol;
  CDPoint cRes;
  cRes.x = cPtSol.x/sqrt(dSol2) - (cPt1.x - cPtSol.x)/dN1;
  cRes.y = 2.0*cPtSol.x + dN1 - cPtSol.x*(cPt1.x - cPtSol.x)/dN1;
  return cRes;
}

CDPoint DAEFuncY(CDPoint cPt1, CDPoint cPt2, CDPoint cPtSol)
{
  //CDPoint cN1 = {cPt1.x - cPtSol.x, cPt1.y};
  CDPoint cN2 = {cPt2.x, cPt2.y + cPtSol.y};
  //double dN1 = GetNorm(cN1);
  double dN2 = GetNorm(cN2);
  double dSol2 = cPtSol*cPtSol;
  CDPoint cRes;
  cRes.x = cPtSol.y/sqrt(dSol2) - (cPt2.y + cPtSol.y)/dN2;
  cRes.y = 2.0*cPtSol.y - dN2 - cPtSol.y*(cPt2.y + cPtSol.y)/dN2;
  return cRes;
}

bool BuildArcElpsCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache, PDLine pLines)
{
/*wchar_t buf[128];
swprintf(buf, L"%d %d", iNorm, iCtrl);
SendMessage(g_hStatus, SB_SETTEXT, 2, (LPARAM)buf);*/
  pCache->ClearAll();

  int nNorm = pPoints->GetCount(0);

  CDInputPoint cInPt1, cInPt2;
  CDPoint cOrig, cPt1, cPt2, cMainDir;
  CDPoint cInter, cSol;
  double d1, d2, da, db;
  double dr1 = -1.0, dr2 = -1.0;

  if(!(pLines[0].bIsSet && pLines[1].bIsSet)) return false;

  CDPoint cDir1 = pLines[0].cDirection;
  CDPoint cDir2 = pLines[1].cDirection;

  int iX = LineXLine(pLines[0].cOrigin, cDir1, pLines[1].cOrigin, cDir2, &cOrig);
  if(iX < 1) return false;

  bool bTwoPoints = false;

  if(((nNorm < 1) && (iMode == 1)) || ((nNorm == 1) && (iMode != 1)))
  {
    if(iMode == 1) cInPt1.cPoint = cTmpPt.cOrigin - cOrig;
    else if(nNorm > 0) cInPt1.cPoint = pPoints->GetPoint(0, 0).cPoint - cOrig;
    else return false;

    d1 = fabs(Deter2(cDir1, cDir2));
    if(d1 < g_dPrec) return false;

    cPt1 = cInPt1.cPoint;
    dr1 = sqrt(Power2(Deter2(cPt1, cDir1)) + Power2(Deter2(cPt1, cDir2)))/d1;
    dr2 = dr1;
  }
  else if(nNorm > 0)
  {
    cInPt1.cPoint = pPoints->GetPoint(0, 0).cPoint - cOrig;
    if(iMode == 1) cInPt2.cPoint = cTmpPt.cOrigin - cOrig;
    else if(nNorm > 1) cInPt2.cPoint = pPoints->GetPoint(1, 0).cPoint - cOrig;
    else return false;

    bTwoPoints = true;
    cPt1 = cInPt1.cPoint;
    cPt2 = cInPt2.cPoint;

    CDPoint cMat1, cMat2;
    cMat1.x = Power2(Deter2(cPt1, cDir1));
    cMat1.y = Power2(Deter2(cPt2, cDir1));
    cMat2.x = Power2(Deter2(cPt1, cDir2));
    cMat2.y = Power2(Deter2(cPt2, cDir2));

    db = Power2(Deter2(cDir1, cDir2));
    da = Deter2(cMat1, cMat2);

    d1 = db*(cMat1.x - cMat1.y);
    if(fabs(d1) < g_dPrec) return false;

    d2 = da/d1;
    if(d2 < -g_dPrec) return false;
    if(d2 < g_dPrec) d2 = 0;

    dr1 = sqrt(d2);

    d1 = db*(cMat2.y - cMat2.x);
    if(fabs(d1) < g_dPrec) return false;

    d2 = da/d1;
    if(d2 < -g_dPrec) return false;
    if(d2 < g_dPrec) d2 = 0;

    dr2 = sqrt(d2);
  }
  else return false;

  double dco, dsi;

  if(fabs(dr1 - dr2) < g_dPrec)
  {
    dco = sqrt(2.0)/2.0;
    dsi = dco;
  }
  else
  {
    double dTanT = 2.0*dr1*dr2*(cDir1*cDir2)/(Power2(dr1) - Power2(dr2));
    double dt = atan(dTanT)/2.0;
    dco = cos(dt);
    dsi = sin(dt);
  }
  cMainDir = dco*dr1*cDir1 + dsi*dr2*cDir2;
  da = GetNorm(cMainDir);
  cMainDir /= da;

  d1 = dsi;
  dsi = dco;
  dco = -d1;

  cPt1 = dco*dr1*cDir1 + dsi*dr2*cDir2;
  db = GetNorm(cPt1);

  if(db > da)
  {
    dco = cMainDir.x;
    dsi = cMainDir.y;
    cMainDir.x = -dsi;
    cMainDir.y = dco;
    dsi = da;
    da = db;
    db = dsi;
  }

  if(db < g_dPrec) return false;

  if(fabs(da - db) < g_dPrec)
  {
    pCache->AddPoint(cOrig.x, cOrig.y, 0);
    pCache->AddPoint(da, da, 0);
    return true;
  }

  if(!bTwoPoints)
  {
    cPt1 = Abs(Rotate(cInPt1.cPoint, cMainDir, false));

    double deps = da/db;
    double du = 1 + Power2(deps);
    double dfact = du + (deps - 1)*sqrt(du);

    double dPoly[3];
    dPoly[0] = Power2(cPt1.x) + Power2(cPt1.y);
    dPoly[1] = cPt1.y*(dfact - 2);
    dPoly[2] = 1 - dfact;

    double dRoots[2];
    int iRoots = SolvePolynom(2, dPoly, dRoots);

    db = -1.0;
    if(iRoots > 0) db = dRoots[0];
    if((db < g_dPrec) && (iRoots > 1)) db = dRoots[1];

    CDLine cLn1, cLn2;
    cLn1.bIsSet = true;
    cLn2.bIsSet = true;

    if(db > g_dPrec)
    {
      da = deps*db;
      du = Power2(da) + Power2(db);
      dr1 = (du - (da - db)*sqrt(du))/da/2.0;
      dr2 = (du + (da - db)*sqrt(du))/db/2.0;

      cDir1.x = da - dr1;
      cDir1.y = dr2 - db;
      d1 = GetNorm(cDir1);
      cDir2 = cDir1/d1;
      cInter.x = cDir1.x + dr1*cDir2.x;
      cInter.y = dr1*cDir2.y;

      cLn1.cOrigin = cInter;
      cLn1.cDirection = cDir2;
      cLn2.cOrigin.x = 0;
      cLn2.cOrigin.y = db;
      cLn2.cDirection.x = 0;
      cLn2.cDirection.y = 1;
      if(PointInArc(cPt1, cLn1, cLn2))
      {
        pCache->AddPoint(cOrig.x, cOrig.y, 0);
        pCache->AddPoint(dr1, dr2, 0);
        pCache->AddPoint(cMainDir.x, cMainDir.y, 0);
        pCache->AddPoint(cDir1.x, cDir1.y, 0);
        pCache->AddPoint(cInter.x, cInter.y, 0);
        return true;
      }
    }

    deps = db/da;
    du = 1 + Power2(deps);
    dfact = du - (1 - deps)*sqrt(du);

    dPoly[0] = Power2(cPt1.x) + Power2(cPt1.y);
    dPoly[1] = cPt1.x*(dfact - 2);
    dPoly[2] = 1 - dfact;

    iRoots = SolvePolynom(2, dPoly, dRoots);

    da = -1.0;
    if(iRoots > 0) da = dRoots[0];
    if((da < g_dPrec) && (iRoots > 1)) da = dRoots[1];

    if(da < g_dPrec) return false;

    db = deps*da;
    du = Power2(da) + Power2(db);
    dr1 = (du - (da - db)*sqrt(du))/da/2.0;
    dr2 = (du + (da - db)*sqrt(du))/db/2.0;

    cDir1.x = da - dr1;
    cDir1.y = dr2 - db;
    d1 = GetNorm(cDir1);
    cDir2 = cDir1/d1;
    cInter.x = cDir1.x + dr1*cDir2.x;
    cInter.y = dr1*cDir2.y;

    cLn1.cOrigin.x = da;
    cLn1.cOrigin.y = 0;
    cLn1.cDirection.x = 1;
    cLn1.cDirection.y = 0;
    cLn2.cOrigin = cInter;
    cLn2.cDirection = cDir2;
    if(PointInArc(cPt1, cLn1, cLn2))
    {
      pCache->AddPoint(cOrig.x, cOrig.y, 0);
      pCache->AddPoint(dr1, dr2, 0);
      pCache->AddPoint(cMainDir.x, cMainDir.y, 0);
      pCache->AddPoint(cDir1.x, cDir1.y, 0);
      pCache->AddPoint(cInter.x, cInter.y, 0);
    }
    return true;
  }

  cPt1 = Rotate(cInPt1.cPoint, cMainDir, false);
  cPt2 = Rotate(cInPt2.cPoint, cMainDir, false);

  if(fabs(cPt2.y) < fabs(cPt1.y)) SwapPoints(&cPt1, &cPt2);

  d1 = GetNorm(cPt1);
  d2 = GetNorm(cPt2);

  if(d2 > d1)
  {
    dco = cMainDir.x;
    cMainDir.x = -cMainDir.y;
    cMainDir.y = dco;
    cPt1 = Rotate(cInPt1.cPoint, cMainDir, false);
    cPt2 = Rotate(cInPt2.cPoint, cMainDir, false);
    if(fabs(cPt2.y) < fabs(cPt1.y)) SwapPoints(&cPt1, &cPt2);
  }

  CDPoint cAxes;
  cAxes.x = fabs(cPt1.x);
  cAxes.y = fabs(cPt2.y);

  if((cAxes.x < g_dPrec) || (cAxes.y < g_dPrec)) return false;

  cPt1.x = cAxes.x;
  cPt2.y = cAxes.y;

  dr1 = Power2(db)/da;
  dr2 = Power2(da)/db;

  cSol.x = cAxes.x - dr1;
  cSol.y = dr2 - cAxes.y;

  CDPoint cFinalSol, cLowBound = {g_dPrec, g_dPrec};
  if(!Solve2ParamSystem(DAEFunc, DAEFuncX, DAEFuncY, cPt1, cPt2, cSol, &cFinalSol, &cLowBound)) return false;

  cSol = cFinalSol;

  CDPoint cMat1, cMat2;
  cMat1.x = cPt1.x - cSol.x;
  cMat1.y = cPt1.y;
  dr1 = GetNorm(cMat1);

  cMat2.x = cPt2.x;
  cMat2.y = cPt2.y + cSol.y;
  dr2 = GetNorm(cMat2);

  double dNormX = GetNorm(cSol);
  cDir2 = cSol/dNormX;
  cInter.x = cSol.x + dr1*cDir2.x;
  cInter.y = dr1*cDir2.y;

  pCache->AddPoint(cOrig.x, cOrig.y, 0);
  pCache->AddPoint(dr1, dr2, 0);
  pCache->AddPoint(cMainDir.x, cMainDir.y, 0);
  pCache->AddPoint(cSol.x, cSol.y, 0);
  pCache->AddPoint(cInter.x, cInter.y, 0);
  return true;
}

void UpdateArcElpsCache(PDPointList pCache)
{
  int nNorm = pCache->GetCount(0);
  if(nNorm < 3) return;

  double dr = -1.0;
  int nOffs = pCache->GetCount(2);
  if((nOffs > 0) && (nNorm > 3))
  {
    //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
    CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
    CDPoint cCentr = pCache->GetPoint(3, 0).cPoint;

    double dDist = pCache->GetPoint(0, 2).cPoint.x;

    if((cRad.x + dDist < g_dPrec) && (cRad.y + dDist > -g_dPrec)) dr = atan2(cCentr.y, cCentr.x);
  }
  pCache->AddPoint(dr, 0.0, 4);
}

int AddArcElpsInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds)
{
  int nCtrls = pCache->GetCount(0);
  if(nCtrls < 2) return 0;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dOffset += pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cRot1;
  CDPoint cRot2;
  int iRoots;
  double dr, dt;
  CDPoint cCros, cRes;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  if(nCtrls < 5)
  {
    dr = fabs(cRad.x + dOffset);
    if(dr < g_dPrec) return 0;

    int iRes = CircXSegParams(cOrig, dr, cPt1, cPt2, &cRes);
    if(iRes > 0) pBounds->AddPoint(cRes.x);
    if(iRes > 1) pBounds->AddPoint(cRes.y);

    return iRes;
  }

  int iRes = 0;
  bool bValid;

  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  CDPoint cCenters = pCache->GetPoint(3, 0).cPoint;
  double dSweep = atan2(cCenters.y, cCenters.x);

  cRot1 = Rotate(cPt1 - cOrig, cMainDir, false);
  cRot2 = Rotate(cPt2 - cOrig, cMainDir, false);
  dr = cRad.x + dOffset;

  if(fabs(dr) > g_dPrec)
  {
    cCros.x = cCenters.x;
    cCros.y = 0.0;

    iRoots = CircXSegParams(cCros, fabs(dr), cRot1, cRot2, &cRes);
    if(iRoots > 0)
    {
      dt = cRes.x;
      if(dr > 0.0) bValid = (dt < dSweep + g_dPrec) && (dt > -dSweep - g_dPrec);
      else bValid = (dt > M_PI - dSweep - g_dPrec) || (dt < dSweep - M_PI + g_dPrec);
      if(bValid && !pBounds->HasPoint(dt))
      {
        pBounds->AddPoint(dt);
        iRes++;
      }
    }
    if(iRoots > 1)
    {
      dt = cRes.y;
      if(dr > 0.0) bValid = (dt < dSweep + g_dPrec) && (dt > -dSweep - g_dPrec);
      else bValid = (dt > M_PI - dSweep - g_dPrec) || (dt < dSweep - M_PI + g_dPrec);
      if(bValid && !pBounds->HasPoint(dt))
      {
        pBounds->AddPoint(dt);
        iRes++;
      }
    }

    cCros.x = -cCenters.x;

    iRoots = CircXSegParams(cCros, fabs(dr), cRot1, cRot2, &cRes);
    if(iRoots > 0)
    {
      dt = cRes.x;
      if(dr < 0.0) bValid = (dt < dSweep + g_dPrec) && (dt > -dSweep - g_dPrec);
      else bValid = (dt > M_PI - dSweep - g_dPrec) || (dt < dSweep - M_PI + g_dPrec);
      if(bValid && !pBounds->HasPoint(dt))
      {
        pBounds->AddPoint(dt);
        iRes++;
      }
    }
    if(iRoots > 1)
    {
      dt = cRes.y;
      if(dr < 0.0) bValid = (dt < dSweep + g_dPrec) && (dt > -dSweep - g_dPrec);
      else bValid = (dt > M_PI - dSweep - g_dPrec) || (dt < dSweep - M_PI + g_dPrec);
      if(bValid && !pBounds->HasPoint(dt))
      {
        pBounds->AddPoint(dt);
        iRes++;
      }
    }
  }

  dr = cRad.y + dOffset;

  if(fabs(dr) > g_dPrec)
  {
    cCros.x = 0.0;
    cCros.y = -cCenters.y;

    iRoots = CircXSegParams(cCros, fabs(dr), cRot1, cRot2, &cRes);
    if(iRoots > 0)
    {
      dt = cRes.x;
      if(dr > 0.0) bValid = (dt > dSweep - g_dPrec) && (dt < M_PI - dSweep + g_dPrec);
      else bValid = (dt < -dSweep + g_dPrec) && (dt > dSweep - M_PI - g_dPrec);
      if(bValid && !pBounds->HasPoint(dt))
      {
        pBounds->AddPoint(dt);
        iRes++;
      }
    }
    if(iRoots > 1)
    {
      dt = cRes.y;
      if(dr > 0.0) bValid = (dt > dSweep - g_dPrec) && (dt < M_PI - dSweep + g_dPrec);
      else bValid = (dt < -dSweep + g_dPrec) && (dt > dSweep - M_PI - g_dPrec);
      if(bValid && !pBounds->HasPoint(dt))
      {
        pBounds->AddPoint(dt);
        iRes++;
      }
    }

    cCros.y = cCenters.y;

    iRoots = CircXSegParams(cCros, fabs(dr), cRot1, cRot2, &cRes);
    if(iRoots > 0)
    {
      dt = cRes.x;
      if(dr < 0.0) bValid = (dt > dSweep - g_dPrec) && (dt < M_PI - dSweep + g_dPrec);
      else bValid = (dt < -dSweep + g_dPrec) && (dt > dSweep - M_PI - g_dPrec);
      if(bValid && !pBounds->HasPoint(dt))
      {
        pBounds->AddPoint(dt);
        iRes++;
      }
    }
    if(iRoots > 1)
    {
      dt = cRes.y;
      if(dr < 0.0) bValid = (dt > dSweep - g_dPrec) && (dt < M_PI - dSweep + g_dPrec);
      else bValid = (dt < -dSweep + g_dPrec) && (dt > dSweep - M_PI - g_dPrec);
      if(bValid && !pBounds->HasPoint(dt))
      {
        pBounds->AddPoint(dt);
        iRes++;
      }
    }
  }
  return iRes;
}

CDPrimitive GetArcPrimitive(double dRad, CDPoint cCenter, bool bReverse, double da1, double da2)
{
  CDPrimitive cRes;
  cRes.iType = 0;
  double dr = fabs(dRad);
  if(dr < g_dPrec) return cRes;

  if(dRad < 0.0)
  {
    da1 = OpositeAngle(da1);
    da2 = OpositeAngle(da2);
  }

  cRes.iType = 2;
  cRes.cPt1 = cCenter;
  cRes.cPt2.x = dr;
  cRes.cPt2.y = 0.0;
  cRes.cPt3.x = da1;
  cRes.cPt3.y = da2;
  cRes.cPt4 = 0;
  if(bReverse) cRes.cPt4.x = 1.0;
  return cRes;
}

CDPrimitive TransArcPrimitive(CDPoint cOrig, CDPoint cMainDir, CDPrimitive cPrim)
{
  CDPrimitive cRes;
  cRes.iType = 0;
  if(cPrim.iType != 2) return cRes;

  double dRot = atan2(cMainDir.y, cMainDir.x);
  cRes.iType = 2;
  cRes.cPt1 = cOrig + Rotate(cPrim.cPt1, cMainDir, true);
  cRes.cPt2 = cPrim.cPt2;
  cRes.cPt3.x = cPrim.cPt3.x + dRot;
  cRes.cPt3.y = cPrim.cPt3.y + dRot;
  cRes.cPt4 = cPrim.cPt4;
  return cRes;
}

void AddArcPrimitive(CDPoint cRad, CDPoint cOrig, CDPoint cMainDir, CDPoint cSol,
  bool bReverse, PDPrimObject pPrimList, double d1, double d2, bool bFullCycle)
{
  double dAng = atan2(cSol.y, cSol.x);
  double db1 = M_PI - dAng;
  double dIntervals[4] = {-db1, -dAng, dAng, db1};
  CDPoint cCenters[4] = {{-cSol.x, 0.0}, {0.0, cSol.y}, {cSol.x, 0.0}, {0.0, -cSol.y}};
  double dRads[4] = {cRad.x, cRad.y, cRad.x, cRad.y};
  int iStart = 0;
  while((d1 > dIntervals[iStart] - g_dPrec) && (iStart < 4)) iStart++;

  CDPrimitive cPrim;
  double dt1, dt2 = d1;

  if((d1 > d2) || (d2 > M_PI + g_dPrec))
  {
    for(int i = iStart; i < 4; i++)
    {
      dt1 = dt2;
      dt2 = dIntervals[i];
      cPrim = TransArcPrimitive(cOrig, cMainDir,
        GetArcPrimitive(dRads[i], cCenters[i], bReverse, dt1, dt2));
      if(cPrim.iType == 2) pPrimList->AddPrimitive(cPrim);
    }
    iStart = 0;
    if(d2 > M_PI + g_dPrec) d2 -= 2*M_PI;
  }
  dt1 = dt2;
  int i = iStart;
  if(i < 4)
  {
    dt2 = dIntervals[i++];
    while((dt2 < d2 - g_dPrec) && (i < 4))
    {
      cPrim = TransArcPrimitive(cOrig, cMainDir,
        GetArcPrimitive(dRads[i - 1], cCenters[i - 1], bReverse, dt1, dt2));
      if(cPrim.iType == 2) pPrimList->AddPrimitive(cPrim);
      dt1 = dt2;
      dt2 = dIntervals[i++];
    }
    if(dt2 < d2 - g_dPrec)
    {
      cPrim = TransArcPrimitive(cOrig, cMainDir,
        GetArcPrimitive(dRads[i - 1], cCenters[i - 1], bReverse, dt1, dt2));
      if(cPrim.iType == 2) pPrimList->AddPrimitive(cPrim);
      dt1 = dt2;
      i++;
    }
    else i--;
  }
  dt2 = d2;
  if(i > 3) i = 0;
  cPrim = TransArcPrimitive(cOrig, cMainDir,
    GetArcPrimitive(dRads[i], cCenters[i], bReverse, dt1, dt2));
  if(cPrim.iType == 2) pPrimList->AddPrimitive(cPrim);
}

int GetArcElpsProjCenter(CDPoint cSol, CDPoint cPt, char *piCenters)
{
  double dNorm = GetNorm(cSol);
  if(dNorm < g_dPrec) return 0;

  CDPoint cDir = cSol/dNorm;
  CDPoint cPos[4], cOrig;
  cOrig.x = cSol.x;
  cOrig.y = 0.0;
  cPos[0] = Rotate(cPt - cOrig, cDir, false);
  cDir.y *= -1.0;
  cPos[1] = Rotate(cPt - cOrig, cDir, false);
  cDir.x *= -1.0;
  cOrig.x *= -1.0;
  cPos[2] = Rotate(cPt - cOrig, cDir, false);
  cDir.y *= -1.0;
  cPos[3] = Rotate(cPt - cOrig, cDir, false);

  // 1
  if((cPos[0].y < -g_dPrec) && (cPos[1].y > g_dPrec))
  {
    piCenters[0] = 0;
    piCenters[1] = 2;
    return 2;
  }

  // 2
  if((cPos[2].y < -g_dPrec) && (cPos[3].y > g_dPrec))
  {
    piCenters[0] = 2;
    piCenters[1] = 0;
    return 2;
  }

  // 3
  if((cPos[1].y > g_dPrec) && (cPos[2].y < -g_dPrec))
  {
    piCenters[0] = 1;
    piCenters[1] = 3;
    return 2;
  }

  // 4
  if((cPos[0].y < -g_dPrec) && (cPos[3].y > g_dPrec))
  {
    piCenters[0] = 3;
    piCenters[1] = 1;
    return 2;
  }

  // 5
  if((cPos[0].y > g_dPrec) && (cPos[1].y > g_dPrec))
  {
    piCenters[0] = 1;
    piCenters[1] = 2;
    return 2;
  }

  // 6
  if((cPos[0].y < -g_dPrec) && (cPos[1].y < -g_dPrec))
  {
    piCenters[0] = 3;
    piCenters[1] = 2;
    return 2;
  }

  // 7
  if((cPos[2].y < -g_dPrec) && (cPos[3].y < -g_dPrec))
  {
    piCenters[0] = 1;
    piCenters[1] = 0;
    return 2;
  }

  // 8
  if((cPos[2].y > g_dPrec) && (cPos[3].y > g_dPrec))
  {
    piCenters[0] = 3;
    piCenters[1] = 0;
    return 2;
  }

  // 9
  if(cPt.y > g_dPrec)
  {
    piCenters[0] = 1;
    piCenters[1] = 3;
    piCenters[2] = 0;
    piCenters[3] = 2;
    return 4;
  }

  // 10
  piCenters[0] = 3;
  piCenters[1] = 1;
  piCenters[2] = 0;
  piCenters[3] = 2;
  return 4;
}

double GetCenterAndRad(int iCenter, CDPoint cRad, CDPoint cSol, PDPoint pCenter)
{
  if(iCenter == 0)
  {
    pCenter->x = cSol.x;
    pCenter->y = 0.0;
    return cRad.x;
  }

  if(iCenter == 1)
  {
    pCenter->x = 0.0;
    pCenter->y = -cSol.y;
    return cRad.y;
  }

  if(iCenter == 2)
  {
    pCenter->x = -cSol.x;
    pCenter->y = 0.0;
    return cRad.x;
  }

  pCenter->x = 0.0;
  pCenter->y = cSol.y;
  return cRad.y;
}

double GetArcElpsNorm(int iCenter, CDPoint cDir, PDPoint pNorm)
{
  double dNorm = GetNorm(cDir);
  if(dNorm < g_dPrec)
  {
    switch(iCenter)
    {
    case 0:
      pNorm->x = -1.0;
      pNorm->y = 0.0;
      break;
    case 1:
      pNorm->x = 0.0;
      pNorm->y = 1.0;
      break;
    case 2:
      pNorm->x = 1.0;
      pNorm->y = 0.0;
      break;
    case 3:
      pNorm->x = 0.0;
      pNorm->y = -1.0;
      break;
    }
  }
  else *pNorm = cDir/dNorm;
  return dNorm;
}

double GetArcElpsDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX)
{
  pPtX->bIsSet = false;
  pPtX->dRef = 0.0;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return 0.0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  CDPoint cDir, cClosePt, cCloseNorm;
  CDPoint cMainDir = {1.0, 0.0};
  CDPoint cPt1, cPt2, cPt3;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0)
  {
    double dOff = pCache->GetPoint(0, 2).cPoint.x;
    cRad.x += dOff;
    cRad.y += dOff;
  }

  double dRes, dMin, dNorm, d1;
  bool bIsNearCenter = true;
  bool bCenters = (iSrchMask & 1);
  bool bReverse = false;

  if(iCnt < 3)
  {
    bReverse = cRad.x < -g_dPrec;
    if(bReverse) cDir = cOrig - cPt;
    else cDir = cPt - cOrig;
    dNorm = GetNorm(cDir);
    if(dNorm > g_dPrec) cCloseNorm = cDir/dNorm;
    else
    {
      cCloseNorm.x = 1.0;
      cCloseNorm.y = 0.0;
    }
    cClosePt = cRad.x*cCloseNorm;
    dMin = GetDist(cClosePt, cRefPt - cOrig);
    dRes = dNorm - fabs(cRad.x);
    bIsNearCenter = false;

    if(bCenters)
    {
      d1 = GetDist(cOrig, cRefPt);
      if(d1 < dMin)
      {
        bIsNearCenter = true;
        dMin = d1;
        dRes = dNorm;
        cCloseNorm = 0;
        cClosePt = 0;
      }
    }
    if(bReverse) dRes *= -1.0;
  }
  else
  {
    if(iCnt < 5) return 0.0;

    cMainDir = pCache->GetPoint(2, 0).cPoint;
    CDPoint cSol = pCache->GetPoint(3, 0).cPoint;
    //CDPoint cInter = pCache->GetPoint(4, 0).cPoint;

    cPt1 = Rotate(cPt - cOrig, cMainDir, false);
    cPt2 = Rotate(cRefPt - cOrig, cMainDir, false);

    CDPoint cCenter, cNorm;
    double dRad, dNorm;

    char iCenters[4];
    int iCents = GetArcElpsProjCenter(cSol, cPt1, iCenters);
    if(iCents < 2) return 0.0;

    if(iSrchMask & 2)
    {
      dRad = GetCenterAndRad(iCenters[1], cRad, cSol, &cCenter);
      if(iCents > 2) cDir = cPt1 - cCenter;
      else cDir = cCenter - cPt1;
      dNorm = GetArcElpsNorm(iCenters[1], cDir, &cNorm);
      cPt3 = cCenter + dRad*cNorm;
      cCloseNorm = cNorm;
      cClosePt = cPt3;
      if(iCents > 2) dRes = dNorm - dRad;
      else dRes = -(dNorm + dRad);

      pPtX->dRef = atan2(cCloseNorm.y, cCloseNorm.x);

      cPt1 = cClosePt;
      cPt2 = cCloseNorm;
      cClosePt = cOrig + Rotate(cPt1, cMainDir, true);
      cCloseNorm = Rotate(cPt2, cMainDir, true);

      pPtX->bIsSet = true;
      pPtX->cOrigin = cClosePt;
      pPtX->cDirection = cCloseNorm;

      return dRes;
    }

    int iCentStart = 0;
    bool bFirstSet = false;

    if(bCenters)
    {
      bIsNearCenter = true;

      cCloseNorm = 0;
      GetCenterAndRad(0, cRad, cSol, &cCenter);
      cClosePt = cCenter;
      dMin = GetDist(cPt2, cClosePt);
      dRes = GetDist(cPt1, cClosePt);

      for(int i = 1; i < 4; i++)
      {
        GetCenterAndRad(i, cRad, cSol, &cCenter);
        d1 = GetDist(cPt2, cCenter);
        if(d1 < dMin)
        {
          cClosePt = cCenter;
          dMin = d1;
          dRes = GetDist(cPt1, cClosePt);
        }
      }
    }
    else
    {
      bIsNearCenter = false;
      iCentStart = 1;

      dRad = GetCenterAndRad(iCenters[0], cRad, cSol, &cCenter);
      cDir = cPt1 - cCenter;
      dNorm = GetArcElpsNorm(iCenters[0], cDir, &cNorm);
      cPt3 = cCenter + dRad*cNorm;
      dMin = GetDist(cPt2, cPt3);
      cCloseNorm = cNorm;
      cClosePt = cPt3;
      dRes = dNorm - dRad;
      bFirstSet = true;
    }

    bool bIsBetter;

    for(int i = iCentStart; i < iCents; i++)
    {
      dRad = GetCenterAndRad(iCenters[i], cRad, cSol, &cCenter);
      bReverse = ((iCents < 3) && (i > 0)) || ((iCents > 2) && (i > 1));
      if(bReverse) cDir = cCenter - cPt1;
      else cDir = cPt1 - cCenter;
      dNorm = GetArcElpsNorm(iCenters[i], cDir, &cNorm);
      cPt3 = cCenter + dRad*cNorm;
      d1 = GetDist(cPt2, cPt3);
      bIsBetter = (d1 < dMin - g_dPrec) || !bFirstSet;

      if(bIsBetter)
      {
        cCloseNorm = cNorm;
        cClosePt = cPt3;
        dMin = d1;
        bFirstSet = true;

        if(bReverse) dRes = -(dNorm + dRad);
        else dRes = dNorm - dRad;
        bIsNearCenter = false;
      }
    }
  }

  if(!bIsNearCenter)
  pPtX->dRef = atan2(cCloseNorm.y, cCloseNorm.x);

  cPt1 = cClosePt;
  cPt2 = cCloseNorm;
  cClosePt = cOrig + Rotate(cPt1, cMainDir, true);
  cCloseNorm = Rotate(cPt2, cMainDir, true);

  pPtX->bIsSet = true;
  pPtX->cOrigin = cClosePt;
  pPtX->cDirection = cCloseNorm;

  return dRes;
}

bool HasArcElpsEnoughPoints(PDPointList pPoints, int iInputLines)
{
  int nNorm = pPoints->GetCount(0);
  bool bRes = false;

  if(iInputLines == 2) bRes = (nNorm > 0);

  return bRes;
}

bool GetArcElpsRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache)
{
  if(iMode != 2) return false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  double dRad = dRestrictValue;

  int iOffs = pCache->GetCount(2);

  if(iOffs > 0)
  {
    CDPoint cOffs = pCache->GetPoint(0, 2).cPoint;
    cRad.x += cOffs.y;
    cRad.y += cOffs.y;
  }

  CDPoint cDir;
  double dNorm;

  if(iCnt < 3)
  {
    cDir = cPt - cOrig;
    dNorm = GetNorm(cDir);
    if(dNorm > g_dPrec)
    {
      cDir /= dNorm;
      *pSnapPt = cOrig + (cRad.x + dRad)*cDir;
    }
    else
    {
      pSnapPt->x = cOrig.x + cRad.x + dRad;
      pSnapPt->y = 0.0;
    }
    return true;
  }

  if(iCnt < 5) return false;

  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  CDPoint cSol = pCache->GetPoint(3, 0).cPoint;
  //CDPoint cInter = pCache->GetPoint(4, 0).cPoint;

  CDPoint cPt1 = Rotate(cPt - cOrig, cMainDir, false);
  char iCenters[4];
  int iCnts = GetArcElpsProjCenter(cSol, cPt1, iCenters);
  if(iCnts < 2) return false;

  CDPoint cCenter;
  double dr1 = GetCenterAndRad(iCenters[0], cRad, cSol, &cCenter);
  cDir = cPt1 - cCenter;
  dNorm = GetNorm(cDir);
  if(dNorm < g_dPrec) return false;

  cDir /= dNorm;
  CDPoint cPt2 = cCenter + (dr1 + dRad)*cDir;
  *pSnapPt = cOrig + Rotate(cPt2, cMainDir, true);
  return true;
}

double GetArcElpsRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt,
  PDPointList pPoints, PDLine pLines)
{
  pPtR->bIsSet = false;
  pPtR->cOrigin = 0;
  pPtR->cDirection = 0;

  PDPointList pLocCache = pCache;
  CDLine cLn;
  cLn.bIsSet = false;
  cLn.cOrigin = cPt;
  cLn.cDirection.x = 0.0;
  if(bNewPt)
  {
    pLocCache = new CDPointList();
    BuildArcElpsCache(cLn, 1, pPoints, pLocCache, pLines);
  }

  int iCnt = pLocCache->GetCount(0);

  if(iCnt < 2) return -1.0;

  CDPoint cOrig = pLocCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pLocCache->GetPoint(1, 0).cPoint;

  int iOffs = pLocCache->GetCount(2);

  if(iOffs > 0)
  {
    CDPoint cOffs = pLocCache->GetPoint(0, 2).cPoint;
    cRad.x += cOffs.x;
    cRad.y += cOffs.x;
  }

  CDPoint cDir;
  CDPoint cMainDir = {1.0, 0.0};
  double dNorm, dRes = -1.0;

  if(iCnt < 3)
  {
    pPtR->bIsSet = true;
    pPtR->cOrigin = cOrig;
    cDir = cPt - cOrig;
    dNorm = GetNorm(cDir);
    if(dNorm > g_dPrec) pPtR->cDirection = cDir/dNorm;
    dRes = fabs(cRad.x);
  }
  else
  {
    if(iCnt < 5) return 0.0;

    cMainDir = pLocCache->GetPoint(2, 0).cPoint;
    CDPoint cSol = pLocCache->GetPoint(3, 0).cPoint;
    //CDPoint cInter = pLocCache->GetPoint(4, 0).cPoint;

    CDPoint cPt1 = Rotate(cPt - cOrig, cMainDir, false);
    char iCenters[4];
    int iCnts = GetArcElpsProjCenter(cSol, cPt1, iCenters);
    if(iCnts < 2) return dRes;

    CDPoint cCenter;
    dRes = fabs(GetCenterAndRad(iCenters[0], cRad, cSol, &cCenter));
    cDir = cPt1 - cCenter;
    dNorm = GetNorm(cDir);

    pPtR->bIsSet = true;
    pPtR->cOrigin = cOrig + Rotate(cCenter, cMainDir, true);
    if(dNorm > g_dPrec) pPtR->cDirection = Rotate(cDir/dNorm, cMainDir, true);
  }

  if(bNewPt) delete pLocCache;

  return dRes;
}

bool GetArcElpsPointRefDist(double dRef, PDPointList pCache, double *pdDist)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0)
  {
    double dOff = pCache->GetPoint(0, 2).cPoint.x;
    cRad.x += dOff;
    cRad.y += dOff;
  }

  double rx = fabs(cRad.x);

  if(iCnt < 3)
  {
    *pdDist = dRef*rx;
    return true;
  }

  if(iCnt < 5) return false;

  //CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  CDPoint cSol = pCache->GetPoint(3, 0).cPoint;
  //CDPoint cInter = pCache->GetPoint(4, 0).cPoint;

  double ry = fabs(cRad.y);

  double dAng = atan2(cSol.y, cSol.x);
  double dDir = 1.0;
  if(dRef < 0.0)
  {
    dDir = -1.0;
    dRef *= dDir;
  }

  double dFullLength = 4.0*(rx*dAng + ry*(M_PI/2.0 - dAng));
  double dExtra = 0.0;
  while(dRef > M_PI + g_dPrec)
  {
    dRef -= 2.0*M_PI;
    dExtra += dFullLength;
  }

  if(dRef < dAng)
  {
    *pdDist = dDir*(dRef*rx + dExtra);
    return true;
  }

  double dRes = dAng*rx;

  if(dRef < M_PI - dAng)
  {
    *pdDist = dDir*(dRes + (dRef - dAng)*ry + dExtra);
    return true;
  }

  dRes += (M_PI - 2*dAng)*ry;
  *pdDist = dDir*(dRes + (dRef + dAng - M_PI)*rx + dExtra);
  return true;
}

double GetArcElpsRefAtDist(double dDist, CDPoint cRad, double dAng)
{
  if(fabs(dDist) < g_dPrec) return 0.0;

  double rx = fabs(cRad.x);
  double ry = fabs(cRad.y);

  double dHalfLen = 2.0*dAng*rx + (M_PI - 2.0*dAng)*ry;
  double dOffs = 0.0;

  double dDir = 1.0;
  if(dDist < 0)
  {
    dDir = -1.0;
    dDist *= dDir;
  }

  while(dDist > dHalfLen + g_dPrec)
  {
    dDist -= 2.0*dHalfLen;
    dOffs += 2.0*M_PI;
  }

  double d1 = rx*dAng;
  if(d1 > dDist - g_dPrec)
  {
    if(d1 > dDist + g_dPrec) return dDir*(dDist/rx + dOffs);
    return dDir*(dAng + dOffs);
  }

  dDist -= d1;

  d1 = (M_PI - 2.0*dAng)*ry;

  if(d1 > dDist - g_dPrec)
  {
    if(d1 > dDist + g_dPrec) return dDir*(dAng + dDist/ry + dOffs);
    return dDir*(M_PI - dAng + dOffs);
  }

  dDist -= d1;
  //d1 = rx*dAng;
  return dDir*(M_PI - dAng + dDist/rx + dOffs);
}

void AddArcElpsSegment(double d1, double d2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList) //, PDRect pRect)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  double dOff = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dOff += pCache->GetPoint(0, 2).cPoint.x;
  cRad.x += dOff;
  cRad.y += dOff;

  CDPrimitive cPrim;
  double dAng1, dAng2;

  if(iCnt < 3)
  {
    double dr = fabs(cRad.x);
    if(dr < g_dPrec) return;

    dAng1 = d1/dr;
    dAng2 = d2/dr;
    if(cRad.x < 0.0)
    {
      dAng1 = OpositeAngle(dAng1);
      dAng2 = OpositeAngle(dAng2);
    }

    cPrim.iType = 2;
    cPrim.cPt1 = cOrig;
    cPrim.cPt2.x = dr;
    cPrim.cPt2.y = 0.0;
    cPrim.cPt3.x = dAng1;
    cPrim.cPt3.y = dAng2;
    cPrim.cPt4 = 0;
    if(bReverse) cPrim.cPt4.x = 1.0;

    pPrimList->AddPrimitive(cPrim);
    return;
  }

  if(iCnt < 5) return;

  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  CDPoint cSol = pCache->GetPoint(3, 0).cPoint;

  double dAng = atan2(cSol.y, cSol.x);
  double dt1 = GetArcElpsRefAtDist(d1, cRad, dAng);
  double dt2 = GetArcElpsRefAtDist(d2, cRad, dAng);
  bool bFullCycle = fabs(dt2 - dt1 - 2.0*M_PI) < g_dPrec;
  AddArcPrimitive(cRad, cOrig, cMainDir, cSol, bReverse, pPrimList, dt1, dt2, bFullCycle);
}

bool GetArcElpsRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cPt1;

  double dOff = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dOff += pCache->GetPoint(0, 2).cPoint.x;
  cRad.x += dOff;
  cRad.y += dOff;

  if(iCnt < 3)
  {
    cPt1.x = cos(dRef);
    cPt1.y = sin(dRef);
    *pPt = cOrig + cRad.x*cPt1;
    return true;
  }

  if(iCnt < 5) return false;

  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  CDPoint cSol = pCache->GetPoint(3, 0).cPoint;
  //CDPoint cInter = pCache->GetPoint(4, 0).cPoint;

  double dAng1 = atan2(cSol.y, cSol.x);
  double dAng2 = fabs(dRef);
  if(dAng2 < dAng1)
  {
    cPt1.x = cSol.x + cRad.x*cos(dRef);
    cPt1.y = cRad.x*sin(dRef);
  }
  else if(dAng2 < M_PI - dAng1)
  {
    cPt1.x = cRad.y*cos(dRef);
    if(dRef > 0) cPt1.y = -cSol.y + cRad.y*sin(dRef);
    else cPt1.y = cSol.y + cRad.y*sin(dRef);
  }
  else
  {
    cPt1.x = -cSol.x + cRad.x*cos(dRef);
    cPt1.y = cRad.x*sin(dRef);
  }

  *pPt = cOrig + Rotate(cPt1, cMainDir, true);
  return true;
}

bool GetArcElpsRefDir(double dRef, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cPt1;

  cPt1.x = -sin(dRef);
  cPt1.y = cos(dRef);

  if(iCnt < 3)
  {
    if(cRad.x < -g_dPrec) *pPt = -1.0*cPt1;
    else *pPt = cPt1;
    return true;
  }

  if(iCnt < 5) return false;

  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  //CDPoint cSol = pCache->GetPoint(3, 0).cPoint;
  //CDPoint cInter = pCache->GetPoint(4, 0).cPoint;

  *pPt = Rotate(cPt1, cMainDir, true);
  return true;
}

bool GetArcElpsReference(double dDist, PDPointList pCache, double *pdRef)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0)
  {
    double dOff = pCache->GetPoint(0, 2).cPoint.x;
    cRad.x += dOff;
    cRad.y += dOff;
  }

  if(iCnt < 3)
  {
    double dr = fabs(cRad.x);
    if(dr < g_dPrec) return false;
    *pdRef = dDist/dr;
    return true;
  }

  if(iCnt < 5) return false;

  //CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  CDPoint cSol = pCache->GetPoint(3, 0).cPoint;
  //CDPoint cInter = pCache->GetPoint(4, 0).cPoint;

  double dAng = atan2(cSol.y, cSol.x);
  *pdRef = GetArcElpsRefAtDist(dDist, cRad, dAng);
  return true;
}

int GetArcElpsSnapPoints(PDPointList pCache, double *pdRefs)
{
  int iRes = 0;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 5) return iRes;

  //CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  int nOffs = pCache->GetCount(2);
  if(nOffs > 0)
  {
    double dOff = pCache->GetPoint(0, 2).cPoint.x;
    cRad.x += dOff;
    cRad.y += dOff;
  }

  //CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  CDPoint cSol = pCache->GetPoint(3, 0).cPoint;
  //CDPoint cInter = pCache->GetPoint(4, 0).cPoint;

  if(cRad.x > g_dPrec) return iRes;
  if(cRad.y < -g_dPrec) return iRes;

  double dAng = atan2(cSol.y, cSol.x);
  pdRefs[iRes++] = dAng - M_PI;
  pdRefs[iRes++] = -dAng;
  pdRefs[iRes++] = dAng;
  pdRefs[iRes++] = M_PI - dAng;
  if(cRad.x > -g_dPrec) return iRes;
  if(cRad.y < g_dPrec) return iRes;
  if(cRad.y < cSol.y - g_dPrec) return iRes;
  if(cRad.y < cSol.y + g_dPrec)
  {
    pdRefs[iRes++] = -M_PI/2.0;
    pdRefs[iRes++] = M_PI/2.0;
    return iRes;
  }

  dAng = acos(cSol.y/cRad.y);
  pdRefs[iRes++] = -M_PI/2.0 - dAng;
  pdRefs[iRes++] = -M_PI/2.0 + dAng;
  pdRefs[iRes++] = M_PI/2.0 - dAng;
  pdRefs[iRes++] = M_PI/2.0 + dAng;
  return iRes;
}

