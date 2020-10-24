#include "DEllipse.hpp"
#include "DPrimitive.hpp"
#include "DMath.hpp"
#include <math.h>

#include <stdio.h>

// for debugging purpose only
/*#include <windows.h>
#include <commctrl.h>
#include <wchar.h>
extern HWND g_hStatus;*/
// -----

// da ... long semiaxis, db ... short semiaxis
// dOffset ... offset from ellipse
// dr1 ... short radius, dr2 ... long radius
// returns break angle in <0, pi/2> if exists, -1 otherwise
double GetElspBreakAngle(double da, double db, double dOffset, double dr1, double dr2)
{
  double dRes = -1.0;
  if(fabs(db - da) < g_dPrec) return dRes;

  if((dOffset > dr1 - g_dPrec) && (dOffset < dr2 + g_dPrec))
  {
    if(dOffset < dr1 + g_dPrec) dRes = 0.0;
    else if(dOffset > dr2 - g_dPrec) dRes = M_PI/2.0;
    else
    {
      double da2 = Power2(da);
      double db2 = Power2(db);
      double d1 = cbrt(Power2(da*db*dOffset));
      double d2 = da2 - db2;
      double dsi = sqrt((d1 - db2)/d2);
      double dco = sqrt((da2 - d1)/d2);
      dRes = atan2(dsi, dco);
    }
  }
  return dRes;
}

CDPoint ElpsFunc(void *pvData, double dt)
{
  PDPoint pPt = (PDPoint)pvData;
  double dco = cos(dt);
  double dsi = sin(dt);
  return {pPt->x*dco, pPt->y*dsi};
}

CDPoint ElpsFuncDer(void *pvData, double dt)
{
  PDPoint pPt = (PDPoint)pvData;
  double dco = cos(dt);
  double dsi = sin(dt);
  return {-pPt->x*dsi, pPt->y*dco};
}

bool GetElpsPtProjFromStartPt(double da, double db, CDPoint cPt, PDPoint pProj)
{
  double dt1, dt2 = atan2(pProj->y, pProj->x);
  double dA = Power2(da) - Power2(db);
  double dB = db*cPt.y;
  double dC = da*cPt.x;
  double f1 = dA*pProj->x*pProj->y + dB*pProj->x - dC*pProj->y;
  double df1 = dA*cos(2.0*dt2) - dB*pProj->y - dC*pProj->x;
  bool bFound = fabs(f1) < g_dPrec;
  int i = 0;
  double dco, dsi;
  while(!bFound && (i < 16) && (fabs(df1) > g_dPrec))
  {
    dt1 = dt2;
    dt2 = dt1 - f1/df1;
    dco = cos(dt2);
    dsi = sin(dt2);
    f1 = dA*dco*dsi + dB*dco - dC*dsi;
    df1 = dA*cos(2.0*dt2) - dB*dsi - dC*dco;
    bFound = fabs(f1) < g_dPrec;
    i++;
  }

  pProj->x = dco;
  pProj->y = dsi;

  return bFound;
}

bool PtInList(CDPoint cPt, int iSize, PDPoint pList)
{
  int i = 0;
  bool bFound = false;
  while(!bFound && (i < iSize))
  {
    bFound = GetDist(cPt, pList[i++]) < g_dPrec;
  }
  return bFound;
}

// return cos and sin of dt
int GetElpsPtProj(double da, double db, CDPoint cPt, PDPoint pProjs)
{
  if(fabs(db - da) < g_dPrec)
  {
    double dNorm = GetNorm(cPt);
    if(dNorm < g_dPrec) return 0;
    pProjs[0] = cPt/dNorm;
    pProjs[1] = -1.0*pProjs[0];
    return 2;
  }

  double da2 = Power2(da);
  double db2 = Power2(db);
  if(da2 - db2 < g_dPrec) return 0;

  double dx2, dy2;
  CDPoint cProjPt;

  if(fabs(cPt.x) < g_dPrec)
  {
    pProjs[0].x = 0.0;
    pProjs[0].y = 1.0;
    pProjs[1].x = 0.0;
    pProjs[1].y = -1.0;
    if(fabs(cPt.y) < g_dPrec)
    {
      pProjs[2].x = 1.0;
      pProjs[2].y = 0.0;
      pProjs[3].x = -1.0;
      pProjs[3].y = 0.0;
      return 4;
    }
    pProjs[2].y = -cPt.y*db/(da2 - db2);
    dy2 = Power2(pProjs[2].y);
    if(dy2 > 1.0 - g_dPrec) return 2;
    pProjs[2].x = sqrt(1.0 - dy2);
    pProjs[3].y = pProjs[2].y;

    pProjs[3].x = -pProjs[2].x;
    cProjPt.x = da*pProjs[2].x;
    cProjPt.y = db*pProjs[2].y;
    return 4;
  }

  if(fabs(cPt.y) < g_dPrec)
  {
    pProjs[0].x = 1.0;

    pProjs[0].y = 0.0;
    pProjs[1].x = -1.0;
    pProjs[1].y = 0.0;
    pProjs[2].x = cPt.x*da/(da2 - db2);
    dx2 = Power2(pProjs[2].x);
    if(dx2 > 1.0 - g_dPrec) return 2;
    pProjs[2].y = sqrt(1.0 - dx2);
    pProjs[3].x = pProjs[2].x;
    pProjs[3].y = -pProjs[2].y;
    cProjPt.x = da*pProjs[2].x;
    cProjPt.y = db*pProjs[2].y;
    return 4;
  }

  double dco, dsi, dt;

  double dFlipX = 1.0;
  double dFlipY = 1.0;
  if(cPt.x < 0.0) dFlipX = -1.0;
  if(cPt.y < 0.0) dFlipY = -1.0;

  int iRes = 2;
  int iResActual = 0;

  cProjPt.x = dFlipX*0.8;
  cProjPt.y = dFlipY*0.6;
  if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt))
    pProjs[iResActual++] = cProjPt;

  cProjPt.x = -dFlipX*0.8;
  cProjPt.y = -dFlipY*0.6;
  if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt) && !PtInList(cProjPt, iResActual, pProjs))
    pProjs[iResActual++] = cProjPt;

  double da1, db1, dc1;
  da1 = cbrt(Power2(da*cPt.x));
  db1 = cbrt(Power2(db*cPt.y));
  dc1 = cbrt(Power2(da2 - db2));
  double dab1 = (da1 + db1)/dc1;
  if(dab1 < 1.0 + g_dPrec)
  {
    iRes = 3;
    dco = cbrt(cPt.x*da/(da2 - db2));
    dsi = cbrt(cPt.y*db/(db2 - da2));
    if(dab1 < 1.0 - g_dPrec)
    {
      iRes = 4;
      cProjPt.x = dco;
      cProjPt.y = dsi;
      dt = GetNorm(cProjPt);
      cProjPt /= dt;
      dt = atan2(cProjPt.y, cProjPt.x);
      cProjPt.x = dFlipX*cos(dt/2.0);
      cProjPt.y = dFlipY*sin(dt/2.0);
      if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt) && !PtInList(cProjPt, iResActual, pProjs))
        pProjs[iResActual++] = cProjPt;
      cProjPt.x = dFlipX*cos(-(M_PI + dt)/2.0);
      cProjPt.y = dFlipY*sin(-(M_PI + dt)/2.0);
      if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt) && !PtInList(cProjPt, iResActual, pProjs))
        pProjs[iResActual++] = cProjPt;
    }
    else
    {
      cProjPt.x = dco;
      cProjPt.y = dsi;
      dt = GetNorm(cProjPt);
      cProjPt /= dt;
      if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt) && !PtInList(cProjPt, iResActual, pProjs))
        pProjs[iResActual++] = cProjPt;
    }
  }

  if(iResActual < iRes)
  {
    cProjPt.x = 1.0;
    cProjPt.y = 0.0;
    if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt) && !PtInList(cProjPt, iResActual, pProjs))
      pProjs[iResActual++] = cProjPt;
  }
  if(iResActual < iRes)
  {
    cProjPt.x = -1.0;
    cProjPt.y = 0.0;
    if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt) && !PtInList(cProjPt, iResActual, pProjs))
      pProjs[iResActual++] = cProjPt;
  }
  if(iResActual < iRes)
  {
    cProjPt.x = 0.0;
    cProjPt.y = 1.0;
    if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt) && !PtInList(cProjPt, iResActual, pProjs))
      pProjs[iResActual++] = cProjPt;
  }
  if(iResActual < iRes)
  {
    cProjPt.x = 0.0;
    cProjPt.y = -1.0;
    if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt) && !PtInList(cProjPt, iResActual, pProjs))
      pProjs[iResActual++] = cProjPt;
  }

  return iRes;
}

int GetEllipseAttractors(CDPoint cPt, PDPointList pCache, PDPoint pPoints)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return 0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;

  double dDist = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dDist = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cPt1 = Rotate(cPt - cOrig, cMainDir, false);

  CDPoint cProjs[4];
  int iRes = GetElpsPtProj(cRad.x, cRad.y, cPt1, cProjs);
  CDPoint cPtNorm;
  double dNorm;

  for(int i = 0; i < iRes; i++)
  {
    cPtNorm.x = cRad.y*cProjs[i].x;
    cPtNorm.y = cRad.x*cProjs[i].y;
    dNorm = GetNorm(cPtNorm);
    cPt1.x = cRad.x*cProjs[i].x + dDist*cPtNorm.x/dNorm;
    cPt1.y = cRad.y*cProjs[i].y + dDist*cPtNorm.y/dNorm;
    pPoints[i] = cOrig + Rotate(cPt1, cMainDir, true);
  }
  return iRes;
}

CDPoint GetElpsBoundProj(double da, double db, double dOffset, CDPoint cPt, CDPoint cPtRef, bool bSecond)
{
  CDPoint pProjs[4];
  double pDists[2];
  int iRoots = GetElpsPtProj(da, db, cPt, pProjs);
  if(iRoots < 2) return pProjs[0];

  int i = 0;
  CDPoint cNorm = {db*pProjs[i].x, da*pProjs[i].y};
  double dNorm = GetNorm(cNorm);
  CDPoint cProjPt = {da*pProjs[i].x + dOffset*cNorm.x/dNorm, db*pProjs[i].y + dOffset*cNorm.y/dNorm};
  pDists[0] = GetDist(cPtRef, cProjPt);
  i++;
  CDPoint cRes;
  double dMin;

  int i0 = 0, i1 = 0;
  cNorm.x = db*pProjs[i].x;
  cNorm.y = da*pProjs[i].y;
  dNorm = GetNorm(cNorm);
  cProjPt.x = da*pProjs[i].x + dOffset*cNorm.x/dNorm;
  cProjPt.y = db*pProjs[i].y + dOffset*cNorm.y/dNorm;
  dMin = GetDist(cPtRef, cProjPt);
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
    cNorm.x = db*pProjs[i].x;
    cNorm.y = da*pProjs[i].y;
    dNorm = GetNorm(cNorm);
    cProjPt.x = da*pProjs[i].x + dOffset*cNorm.x/dNorm;
    cProjPt.y = db*pProjs[i].y + dOffset*cNorm.y/dNorm;
    dMin = GetDist(cPtRef, cProjPt);
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
  if(bSecond) cRes = pProjs[i1];
  else cRes = pProjs[i0];
  return cRes;
}

bool GetRefInSeg(double dtStart, double dtEnd, double dt)
{
  if(dtStart > dtEnd)
  {
    return (dt > dtStart - g_dPrec) || (dt < dtEnd - g_dPrec);
  }
  return (dt > dtStart - g_dPrec) && (dt < dtEnd - g_dPrec);
}

CDPoint GetElpsProjInSeg(double da, double db, double dr, CDPoint cPt, double dtStart, double dtEnd)
{
  CDPoint pProjs[4];
  int iRoots = GetElpsPtProj(da, db, cPt, pProjs);

  bool bValid[4];
  double dDist[4];
  CDPoint cPtProj;
  double dt;

  for(int i = 0; i < iRoots; i++)
  {
    dt = atan2(pProjs[i].y, pProjs[i].x);
    bValid[i] = GetRefInSeg(dtStart, dtEnd, dt);
    cPtProj.x = da*pProjs[i].x + dr*pProjs[i].y;
    cPtProj.y = db*pProjs[i].y - dr*pProjs[i].x;
    dDist[i] = GetDist(cPtProj, cPt);
  }

  int iMin = 0;
  double dMin = 0.0;
  bool bFound = false;
  int i = 0;
  while(!bFound && (i < iRoots))
  {
    bFound = bValid[i++];
  }
  if(bFound)
  {
    iMin = i - 1;
    dMin = dDist[iMin];
    while(i < iRoots)
    {
      if(bValid[i])
      {
        if(dDist[i] < dMin)
        {
          iMin = i;
          dMin = dDist[i];
        }
      }
      i++;
    }
  }
  if(!bFound)
  {
    dMin = dDist[0];
    i = 1;
    while(i < iRoots)
    {
      if(dDist[i] < dMin)
      {
        iMin = i;
        dMin = dDist[i];
      }
      i++;
    }
  }
  return pProjs[iMin];
}

bool AddEllipsePoint(double x, double y, char iCtrl, double dRestrictVal, PDPointList pPoints, int iInputLines)
{
  int nOffs2 = pPoints->GetCount(2);
  int nOffs3 = pPoints->GetCount(3);
  int nOffs4 = pPoints->GetCount(4);

  if((iCtrl == 2) || (iCtrl == 3) || (iCtrl == 4))
  {
    CDPoint cNewPt = {x, y};
    if(iCtrl == 4)
    {
      cNewPt.x = dRestrictVal;
      cNewPt.y = 0.0;
    }

    if(nOffs2 > 0) pPoints->SetPoint(0, 2, cNewPt.x, cNewPt.y, iCtrl);
    else if(nOffs3 > 0) pPoints->SetPoint(0, 3, cNewPt.x, cNewPt.y, iCtrl);
    else if(nOffs4 > 0) pPoints->SetPoint(0, 4, cNewPt.x, cNewPt.y, iCtrl);
    else pPoints->AddPoint(cNewPt.x, cNewPt.y, iCtrl);
    return true;
  }

  bool bRes = false;
  int nNorm = pPoints->GetCount(0);
  int nCtrl = pPoints->GetCount(1);

  if(iInputLines == 2)
  {
    if(iCtrl < 1)
    {
      if(nNorm < 2)
      {
        pPoints->AddPoint(x, y, iCtrl);
        nNorm++;
      }
    }
    bRes = (nNorm > 1);
  }
  else
  {
    if(iCtrl == 1)
    {
      if(nCtrl < 2)
      {
        pPoints->AddPoint(x, y, iCtrl);
        nCtrl++;
      }
    }
    else if(nCtrl > 1)
    {
      if(nNorm < 2)
      {
        pPoints->AddPoint(x, y, iCtrl);
        nNorm++;
      }
    }
    bRes = (nCtrl > 1) && (nNorm > 0);
  }
  return bRes;
}

bool BuildEllipseCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache,
  PDLine pLines, double *pdDist)
{
  pCache->ClearAll();

  int nNorm = pPoints->GetCount(0);
  int nCtrl = pPoints->GetCount(1);

  CDPoint cOrig, cMainDir, cPt1, cPt2, cPt3;
  double d1, d2, d3, da = -1.0, db = -1.0;
  double dr1 = -1.0, dr2 = -1.0;

  if(pLines[0].bIsSet && pLines[1].bIsSet)
  {
    CDPoint cDir1 = pLines[0].cDirection;
    CDPoint cDir2 = pLines[1].cDirection;

    int iX = LineXLine(pLines[0].cOrigin, cDir1, pLines[1].cOrigin, cDir2, &cOrig);
    if(iX < 1) return false;

    if(((nNorm < 1) && (iMode == 1)) || ((nNorm == 1) && (iMode != 1)))
    {
      if(iMode == 1) cPt1 = cTmpPt.cOrigin - cOrig;
      else if(nNorm > 0) cPt1 = pPoints->GetPoint(0, 0).cPoint - cOrig;
      else return false;

      d1 = fabs(Deter2(cDir1, cDir2));
      if(d1 < g_dPrec) return false;

      dr1 = sqrt(Power2(Deter2(cPt1, cDir1)) + Power2(Deter2(cPt1, cDir2)))/d1;
      dr2 = dr1;
    }
    else
    {
      if(nNorm > 0) cPt1 = pPoints->GetPoint(0, 0).cPoint - cOrig;
      else return false;

      if(iMode == 1) cPt2 = cTmpPt.cOrigin - cOrig;
      else if(nNorm > 1) cPt2 = pPoints->GetPoint(1, 0).cPoint - cOrig;
      else return false;

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
  }
  else if(nCtrl > 0)
  {
    if(nCtrl < 2)
    {
      cPt1 = pPoints->GetPoint(0, 1).cPoint;
      if(nNorm > 0) cPt2 = pPoints->GetPoint(0, 0).cPoint;
      else if(iMode == 1) cPt2 = cTmpPt.cOrigin;
      else return false;

      pCache->AddPoint(cPt1.x, cPt1.y, 0);
      pCache->AddPoint(cPt2.x, cPt2.y, 0);
      return true;
    }

    cPt1 = pPoints->GetPoint(0, 1).cPoint;
    cPt2 = pPoints->GetPoint(1, 1).cPoint;

    if(iMode == 1) cPt3 = cTmpPt.cOrigin;
    else if(nNorm > 0) cPt3 = pPoints->GetPoint(0, 0).cPoint;
    else return false;

    CDPoint cDir = cPt2 - cPt1;
    d1 = GetNorm(cDir);

    if(d1 < g_dPrec) return false;

    d2 = GetDist(cPt1, cPt3);
    d3 = GetDist(cPt2, cPt3);

    da = (d2 + d3)/2.0;
    db = sqrt(Power2(d2 + d3) - Power2(d1))/2.0;

    cMainDir = cDir/d1;
    cOrig = (cPt2 + cPt1)/2.0;
  }

  if((da > g_dPrec) && (db > g_dPrec))
  {
    pCache->AddPoint(cOrig.x, cOrig.y, 0);
    pCache->AddPoint(da, db, 0);
    pCache->AddPoint(cMainDir.x, cMainDir.y, 0);

    dr1 = Power2(db)/da;
    dr2 = Power2(da)/db;
    pCache->AddPoint(dr1, dr2, 3);

    double dr = -1.0;

    int nOffs2 = pPoints->GetCount(2);
    int nOffs3 = pPoints->GetCount(3);
    int nOffs4 = pPoints->GetCount(4);
    int iSrchMask = 0;
    if((iMode == 2) || (nOffs2 > 0) || (nOffs3 > 0) || (nOffs4 > 0))
    {
      CDLine cPtX;
      double dDist = 0.0;
      double dDistOld = 0.0;

      if(iMode == 2)
      {
        cPt1 = cTmpPt.cOrigin;
        if(cTmpPt.cDirection.x < -0.5) iSrchMask = 2;
      }
      else if(nOffs2 > 0) cPt1 = pPoints->GetPoint(0, 2).cPoint;
      else if(nOffs3 > 0)
      {
        cPt1 = pPoints->GetPoint(0, 3).cPoint;
        iSrchMask = 2;
      }

      if((iMode == 2) || (nOffs4 == 0))
        dDist = GetElpsDistFromPt(cPt1, cPt1, iSrchMask, pCache, &cPtX);

      if(iMode == 2)
      {
        if(nOffs4 > 0)
          dDistOld = pPoints->GetPoint(0, 4).cPoint.x;
        else if(nOffs2 > 0)
        {
          cPt1 = pPoints->GetPoint(0, 2).cPoint;
          dDistOld = GetElpsDistFromPt(cPt1, cPt1, 0, pCache, &cPtX);
        }
        else if(nOffs3 > 0)
        {
          cPt1 = pPoints->GetPoint(0, 3).cPoint;
          dDistOld = GetElpsDistFromPt(cPt1, cPt1, 2, pCache, &cPtX);
        }
        if(cTmpPt.cDirection.x > 0.5) dDist = dDistOld + cTmpPt.cDirection.y;
      }
      else if(nOffs4 > 0) dDist = pPoints->GetPoint(0, 4).cPoint.x;

      if(pdDist) *pdDist = dDist - dDistOld;
      if((fabs(dDist) > g_dPrec) || (fabs(dDistOld) > g_dPrec)) pCache->AddPoint(dDist, dDistOld, 2);

      dr = GetElspBreakAngle(da, db, -dDist, dr1, dr2);
      pCache->AddPoint(dr, 0.0, 4);
    }
    else pCache->AddPoint(-1.0, 0.0, 4);
  }

  return true;
}

double ElpsProjFunc(void *pvData, double dOffset, CDPoint cPt, CDPoint cStart, CDPoint cEnd)
{
  PDPoint pPt = (PDPoint)pvData;
  CDPoint cPt1 = GetElpsProjInSeg(pPt->x, pPt->y, dOffset, cPt, cStart.y, cEnd.y);
  return atan2(cPt1.y, cPt1.x);
}

int AddEllipseInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return 0;

  CDPoint cOrig, cRad, cMainDir;
  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;

  int iRes = 0;

  if(iCnt < 3)
  {
    double dLen = GetDist(cOrig, cRad);
    if(dLen < g_dPrec) return 0;
    cMainDir = (cRad - cOrig)/dLen;
    cOrig.x -= dOffset*cMainDir.y;
    cOrig.y += dOffset*cMainDir.x;
    cRad.x -= dOffset*cMainDir.y;
    cRad.y += dOffset*cMainDir.x;
    iRes = SegXSeg(cOrig, cRad, cPt1, cPt2, &cMainDir);
    if(iRes > 0) pBounds->AddPoint(cMainDir.x);
    return iRes;
  }

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  if(fabs(cRad.x - cRad.y) < g_dPrec)
  {
    CDPoint cRes;
    int iRes = CircXSegParams(cOrig, cRad.x + dr, cPt1, cPt2, &cRes);
    if(iRes > 0) pBounds->AddPoint(cRes.x);
    if(iRes > 1) pBounds->AddPoint(cRes.y);
    return iRes;
  }

  cMainDir = pCache->GetPoint(2, 0).cPoint;

  CDPoint cLn1 = Rotate(cPt1 - cOrig, cMainDir, false);
  CDPoint cLn2 = Rotate(cPt2 - cOrig, cMainDir, false);
  CDPoint cLnDir = cLn2 - cLn1;
  double dLnLen = GetNorm(cLnDir);
  if(dLnLen < g_dPrec) return 0;
  cLnDir /= dLnLen;

  int iBreaks = pCache->GetCount(4);
  double dBreak = M_PI/4.0;
  if(iBreaks > 0)
  {
    CDPoint cBreak = pCache->GetPoint(0, 4).cPoint;
    if(cBreak.x > -0.5) dBreak = cBreak.x;
  }

  double pi2 = M_PI/2.0;
  double dt1 = -pi2;
  if(fabs(cLnDir.y) > g_dPrec)
  {
    dt1 = atan(-cRad.y*cLnDir.x/cRad.x/cLnDir.y);
    if(dt1 > g_dPrec) dt1 -= M_PI;
  }

  if(dBreak > g_dPrec)
    iRes += AddCurveInterLine(&cRad, dr, ElpsFunc, ElpsFuncDer,
      ElpsProjFunc, {1.0, dt1}, {1.0, -M_PI}, {1.0, dBreak - M_PI}, cLn1, cLn2, pBounds);
  if(dBreak < pi2 - g_dPrec)
    iRes += AddCurveInterLine(&cRad, dr, ElpsFunc, ElpsFuncDer,
      ElpsProjFunc, {1.0, dt1}, {1.0, dBreak - M_PI}, {1.0, -dBreak}, cLn1, cLn2, pBounds);
  if(dBreak > g_dPrec)
  {
    iRes += AddCurveInterLine(&cRad, dr, ElpsFunc, ElpsFuncDer,
      ElpsProjFunc, {1.0, dt1}, {1.0, -dBreak}, {1.0, 0.0}, cLn1, cLn2, pBounds);
    dt1 += M_PI;
    iRes += AddCurveInterLine(&cRad, dr, ElpsFunc, ElpsFuncDer,
      ElpsProjFunc, {1.0, dt1}, {1.0, 0.0}, {1.0, dBreak}, cLn1, cLn2, pBounds);
  }
  else dt1 += M_PI;
  if(dBreak < pi2 - g_dPrec)
    iRes += AddCurveInterLine(&cRad, dr, ElpsFunc, ElpsFuncDer,
      ElpsProjFunc, {1.0, dt1}, {1.0, dBreak}, {1.0, M_PI - dBreak}, cLn1, cLn2, pBounds);
  if(dBreak > g_dPrec)
    iRes += AddCurveInterLine(&cRad, dr, ElpsFunc, ElpsFuncDer,
      ElpsProjFunc, {1.0, dt1}, {1.0, M_PI - dBreak}, {1.0, M_PI}, cLn1, cLn2, pBounds);

  return iRes;
}

double GetElpsOffset(PDPointList pCache)
{
  int nOffs = pCache->GetCount(2);
  if(nOffs < 1) return 0.0;
  return pCache->GetPoint(0, 2).cPoint.x;
}

double GetElpsDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX)
{
  pPtX->bIsSet = false;
  pPtX->dRef = 0.0;
  pPtX->cOrigin = 0;
  pPtX->cDirection = 0;

  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return -1.0;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  double dDist = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dDist = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cPt1 = Rotate(cPt - cOrig, cMainDir, false);
  CDPoint cRefPt1 = Rotate(cRefPt - cOrig, cMainDir, false);

  double dMin = -1.0;
  double dNorm;

  if((pCache->GetCount(3) > 0) && (iSrchMask & 1))
  {
    CDPoint cPt2 = pCache->GetPoint(0, 3).cPoint;
    CDPoint cPt3 = {cRad.x - cPt2.x, 0.0};
    pPtX->bIsSet = true;
    pPtX->cOrigin = cOrig + Rotate(cPt3, cMainDir, true);
    dMin = GetDist(cRefPt1, cPt3);

    cPt3.x *= -1.0;
    dNorm = GetDist(cRefPt1, cPt3);
    if(dNorm < dMin)
    {
      dMin = dNorm;
      pPtX->cOrigin = cOrig + Rotate(cPt3, cMainDir, true);
    }

    cPt3.x = 0.0;
    cPt3.y = cPt2.y - cRad.y;
    dNorm = GetDist(cRefPt1, cPt3);
    if(dNorm < dMin)
    {
      dMin = dNorm;
      pPtX->cOrigin = cOrig + Rotate(cPt3, cMainDir, true);
    }

    cPt3.y *= -1.0;
    dNorm = GetDist(cRefPt1, cPt3);
    if(dNorm < dMin)
    {
      dMin = dNorm;
      pPtX->cOrigin = cOrig + Rotate(cPt3, cMainDir, true);
    }
  }

  CDPoint cProj = GetElpsBoundProj(cRad.x, cRad.y, dDist, cPt1, cRefPt1, iSrchMask & 2);
  if(cProj.x > 4.0) return dMin;

  CDPoint cProjDir, cProjPt, cProjOrg;

  cProjDir.x = cRad.y*cProj.x;
  cProjDir.y = cRad.x*cProj.y;
  dNorm = GetNorm(cProjDir);
  if(dNorm < g_dPrec) return dMin;

  cProjDir /= dNorm;
  cProjPt.x = cRad.x*cProj.x;
  cProjPt.y = cRad.y*cProj.y;
  double dNorm2 = GetDist(cProjPt + dDist*cProjDir, cRefPt1);

  double da2 = Power2(cRad.x);
  double db2 = Power2(cRad.y);
  cProjOrg.x = (da2 - db2)*Power3(cProj.x)/cRad.x;
  cProjOrg.y = (db2 - da2)*Power3(cProj.y)/cRad.y;

  double d1 = GetDist(cRefPt1, cProjOrg);
  double d2 = GetDist(cProjPt, cProjOrg);

  CDPoint cPt2 = Rotate(cRefPt1 - cProjOrg, cProjDir, false);
  double dDir = 1.0;
  if(cPt2.x < -g_dPrec) dDir = -1.0;

  dNorm = dDir*d1 - d2 - dDist;
  if(!pPtX->bIsSet || (dNorm2 < dMin - g_dPrec))
  {
    dMin = dNorm;
    pPtX->bIsSet = true;
    pPtX->cOrigin = cOrig + Rotate(cProjPt + dDist*cProjDir, cMainDir, true);
    pPtX->cDirection = Rotate(cProjDir, cMainDir, true);
    pPtX->dRef = atan2(cProj.y, cProj.x);
  }

  return dMin;
}

bool GetElpsRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
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
  CDPoint cProj = GetElpsBoundProj(cRad.x, cRad.y, dRad, cPt1, cPt1, false);

  CDPoint cDir;
  cDir.x = cRad.y*cProj.x;
  cDir.y = cRad.x*cProj.y;
  double dNorm = GetNorm(cDir);
  if(dNorm < g_dPrec) return false;

  CDPoint cPt2;
  cPt2.x = cRad.x*cProj.x + dRad*cDir.x/dNorm;
  cPt2.y = cRad.y*cProj.y + dRad*cDir.y/dNorm;
  *pSnapPt = cOrig + Rotate(cPt2, cMainDir, true);
  return true;
}

bool HasElpsEnoughPoints(PDPointList pPoints, int iInputLines)
{
  int nNorm = pPoints->GetCount(0);
  int nCtrl = pPoints->GetCount(1);
  bool bRes = false;

  if(iInputLines == 2) bRes = (nNorm > 0);
  else bRes = (nCtrl > 1) && (nNorm > 0);

  return bRes;
}

double GetElpsRadiusAtPt(CDPoint cPt, PDPointList pCache, PDLine pPtR, bool bNewPt,
  PDPointList pPoints, PDLine pLines)
{
  pPtR->bIsSet = false;
  pPtR->cOrigin = 0;
  pPtR->cDirection = 0;

  PDPointList pLocCache = pCache;
  if(bNewPt)
  {
    pLocCache = new CDPointList();
    CDLine cPtX;
    cPtX.bIsSet = false;
    cPtX.cOrigin = cPt;
    double dOff;
    BuildEllipseCache(cPtX, 1, pPoints, pLocCache, pLines, &dOff);
  }

  int iCnt = pLocCache->GetCount(0);

  if(iCnt < 3)
  {
    if(bNewPt) delete pLocCache;
    return -1.0;
  }

  CDPoint cOrig = pLocCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pLocCache->GetPoint(1, 0).cPoint;
  CDPoint cMainDir = pLocCache->GetPoint(2, 0).cPoint;
  double dDist = 0.0;
  int nOffs = pLocCache->GetCount(2);
  if(nOffs > 0) dDist = pLocCache->GetPoint(0, 2).cPoint.x;

  if(bNewPt) delete pLocCache;

  CDPoint cDir, cPt1, cPt2, cPt3;

  cPt1 = Rotate(cPt - cOrig, cMainDir, false);

  CDPoint cProj = GetElpsBoundProj(cRad.x, cRad.y, dDist, cPt1, cPt1, false);
  if(cProj.x > 4.0) return -1.0;

  cDir.x = cRad.y*cProj.x;
  cDir.y = cRad.x*cProj.y;

  cPt3.x = cRad.x*cProj.x;
  cPt3.y = cRad.y*cProj.y;

  double da2 = Power2(cRad.x);
  double db2 = Power2(cRad.y);
  cPt2.x = (da2 - db2)*Power3(cProj.x)/cRad.x;
  cPt2.y = (db2 - da2)*Power3(cProj.y)/cRad.y;

  double dNorm = GetNorm(cDir);
  double dRad = GetDist(cPt3, cPt2);

  pPtR->bIsSet = true;
  pPtR->cOrigin = cOrig + Rotate(cPt2, cMainDir, true);
  if(dNorm > g_dPrec)
    pPtR->cDirection = Rotate(cDir/dNorm, cMainDir, true);

  return fabs(dRad + dDist);
}

bool GetElpsPointRefDist(double dRef, PDPointList pCache, double *pdDist)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  double dDist = 0.0;

  if(iCnt < 3)
  {
    dDist = GetDist(cRad, cOrig);
    *pdDist = dRef*dDist;
    return true;
  }

  //CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dDist = pCache->GetPoint(0, 2).cPoint.x;

  int iBreaks = pCache->GetCount(4);
  CDPoint cBreak = {-1.0, 0.0};
  if(iBreaks > 0)
  {
    cBreak = pCache->GetPoint(0, 4).cPoint;
  }
  if(cBreak.x > -0.5) cBreak.y = M_PI - cBreak.x;
  else cBreak.y = -1.0;

  *pdDist = GetCurveDistAtRef(&cRad, dDist, cBreak, fabs(dRef),
    ElpsFunc, ElpsFuncDer, M_PI/4.0, 0, {1.0, M_PI});
  if(dRef < 0.0) *pdDist *= -1.0;
  return true;
}

void AddElpsSegment(double d1, double d2, double dExt, PDPointList pCache, PDPrimObject pPrimList)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;

  if(iCnt < 3)
  {
    CDPrimitive cPrim;
    CDPoint cPt1;
    CDPoint cDir = cRad - cOrig;
    double dNorm = GetNorm(cDir);
    if(dNorm < g_dPrec) return;

    cDir /= dNorm;
    cPt1.x = d1;
    cPt1.y = 0.0;

    cPrim.iType = 1;
    cPrim.cPt1 = cOrig + Rotate(cPt1, cDir, true);
    cPt1.x = d2;
    cPrim.cPt2 = cOrig + Rotate(cPt1, cDir, true);
    cPrim.cPt3 = 0;
    cPrim.cPt4 = 0;
    pPrimList->AddPrimitive(cPrim);
    return;
  }

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  if(fabs(cRad.y - cRad.x) < g_dPrec)
  {
    double dRad = cRad.x + dr;
    if(fabs(dRad) < g_dPrec) return;

    double dAng1, dAng2;
    dAng1 = d1/dRad;
    dAng2 = d2/dRad;

    CDPrimitive cPrim;
    cPrim.iType = 2;
    cPrim.cPt1 = cOrig;
    cPrim.cPt2.x = dRad;
    cPrim.cPt2.y = 0.0;
    cPrim.cPt3.x = dAng1;
    cPrim.cPt3.y = dAng2;
    cPrim.cPt4 = 0;
    pPrimList->AddPrimitive(cPrim);
    return;
  }

  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;

  int iBreaks = pCache->GetCount(4);
  CDPoint cBreak = {-1.0, 0.0};
  if(iBreaks > 0) cBreak = pCache->GetPoint(0, 4).cPoint;
  if(cBreak.x > -0.5) cBreak.y = M_PI - cBreak.x;
  else cBreak.y = -1.0;

  CDPoint cPt = GetCurveRefAtDist(&cRad, dr, cBreak, fabs(d1),
    ElpsFunc, ElpsFuncDer, M_PI/4.0, 0, {1.0, M_PI});
  CDPoint cPt2 = GetElpsBoundProj(cRad.x, cRad.y, dr, cPt, cPt, false);
  double dt1 = atan2(cPt2.y, cPt2.x);
  if(d1 < 0.0) dt1 *= -1.0;
  cPt = GetCurveRefAtDist(&cRad, dr, cBreak, fabs(d2),
    ElpsFunc, ElpsFuncDer, M_PI/4.0, 0, {1.0, M_PI});
  cPt2 = GetElpsBoundProj(cRad.x, cRad.y, dr, cPt, cPt, false);
  double dt2 = atan2(cPt2.y, cPt2.x);
  if(d2 < 0.0) dt2 *= -1.0;

  dr += dExt;

  PDPrimObject pTmpPrim = new CDPrimObject();
  if(dt1 > dt2)
  {
    AddCurveSegment(&cRad, dr, cBreak, ElpsFunc, ElpsFuncDer,
      dt1, M_PI, M_PI/4.0, 0, pTmpPrim);
    RotatePrimitives(pTmpPrim, pPrimList, cOrig, cMainDir);
    pTmpPrim->Clear();
    AddCurveSegment(&cRad, dr, cBreak, ElpsFunc, ElpsFuncDer,
      -M_PI, dt2, M_PI/4.0, 0, pTmpPrim);
    RotatePrimitives(pTmpPrim, pPrimList, cOrig, cMainDir);
  }
  else
  {
    AddCurveSegment(&cRad, dr, cBreak, ElpsFunc, ElpsFuncDer,
      dt1, dt2, M_PI/4.0, 0, pTmpPrim);
    RotatePrimitives(pTmpPrim, pPrimList, cOrig, cMainDir);
  }
  delete pTmpPrim;
}

void AddElpsExtPrim(PDRect pRect, PDPointList pCache, PDPrimObject pPrimList)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return;

  CDPoint cOrig, cRad, cMainDir;

  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;
  cMainDir = pCache->GetPoint(2, 0).cPoint;

  if(pCache->GetCount(3) > 0)
  {
    CDPrimitive cPrimPt;
    CDPoint cPt2 = pCache->GetPoint(0, 3).cPoint;
    CDPoint cCenter;
    cCenter.x = cRad.x - cPt2.x;
    cCenter.y = 0.0;

    cPrimPt.iType = 7;
    cPrimPt.cPt1.x = 1;
    cPrimPt.cPt1.y = 1;
    cPrimPt.cPt2 = cOrig + Rotate(cCenter, cMainDir, true);
    cCenter.x *= -1.0;
    cPrimPt.cPt3 = cOrig + Rotate(cCenter, cMainDir, true);
    cPrimPt.cPt4 = 0;
    CropPrimitive(cPrimPt, pRect, pPrimList);

    cCenter.x = 0.0;
    cCenter.y = cPt2.y - cRad.y;

    cPrimPt.iType = 7;
    cPrimPt.cPt1.x = 1;
    cPrimPt.cPt1.y = 1;
    cPrimPt.cPt2 = cOrig + Rotate(cCenter, cMainDir, true);
    cCenter.y *= -1.0;
    cPrimPt.cPt3 = cOrig + Rotate(cCenter, cMainDir, true);
    cPrimPt.cPt4 = 0;
    CropPrimitive(cPrimPt, pRect, pPrimList);
  }
}

bool GetElpsRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;

  if(iCnt < 3)
  {
    *pPt = (1.0 - dRef)*cOrig + dRef*cRad;
    return true;
  }

  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  double dco = cos(dRef);
  double dsi = sin(dRef);

  CDPoint cNorm;
  cNorm.x = cRad.y*dco;
  cNorm.y = cRad.x*dsi;
  double dN1 = GetNorm(cNorm);
  if(dN1 < g_dPrec) return false;

  CDPoint cPt1;
  cPt1.x = cRad.x*dco + dr*cNorm.x/dN1;
  cPt1.y = cRad.y*dsi + dr*cNorm.y/dN1;
  *pPt = cOrig + Rotate(cPt1, cMainDir, true);
  return true;
}

bool GetElpsRefDir(double dRef, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;

  if(iCnt < 3)
  {
    double dLen = GetDist(cOrig, cRad);
    if(dLen < g_dPrec) return false;
    *pPt = (cRad - cOrig)/dLen;
    return true;
  }

  double dco = cos(dRef);
  double dsi = sin(dRef);

  CDPoint cNorm;
  cNorm.x = -cRad.x*dsi;
  cNorm.y = cRad.y*dco;
  double dN1 = GetNorm(cNorm);
  if(dN1 < g_dPrec) return false;

  CDPoint cPt1;
  cPt1.x = cNorm.x/dN1;
  cPt1.y = cNorm.y/dN1;
  *pPt = Rotate(cPt1, cMainDir, true);
  return true;
}

bool GetElpsReference(double dDist, PDPointList pCache, double *pdRef)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return false;

  CDPoint cOrig = pCache->GetPoint(0, 0).cPoint;
  CDPoint cRad = pCache->GetPoint(1, 0).cPoint;
  //CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;

  if(iCnt < 3)
  {
    double dLen = GetDist(cOrig, cRad);
    if(dLen < g_dPrec) return false;
    *pdRef = dDist/dLen;
    return true;
  }

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  int iBreaks = pCache->GetCount(4);
  CDPoint cBreak = {-1.0, 0.0};
  if(iBreaks > 0) cBreak = pCache->GetPoint(0, 4).cPoint;
  if(cBreak.x > -0.5) cBreak.y = M_PI - cBreak.x;
  else cBreak.y = -1.0;

  CDPoint cPt = GetCurveRefAtDist(&cRad, dr, cBreak, fabs(dDist),
    ElpsFunc, ElpsFuncDer, M_PI/4.0, 0, {1.0, M_PI});
  CDPoint cPt2 = GetElpsBoundProj(cRad.x, cRad.y, dr, cPt, cPt, false);
  *pdRef = atan2(cPt2.y, cPt2.x);
  if(dDist < 0.0) *pdRef *= -1.0;

  return true;
}

int GetElpsNumParts(PDPointList pCache, PDRefPoint pBounds)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return 0;

  CDPoint cBreak = {-0.5, 0.0};
  int iBreaks = pCache->GetCount(4);
  if(iBreaks > 0) cBreak = pCache->GetPoint(0, 4).cPoint;
  if(cBreak.x < -0.5) return 0;

  int iRes = 0;
  int iRef;
  double dAng = cBreak.x;

  if(dAng < g_dPrec)
  {
    if(!pBounds[0].bIsSet) return 1;
    if(!pBounds[1].bIsSet)
    {
      if(fabs(pBounds[0].dRef) < g_dPrec) return 1;
      if((fabs(pBounds[0].dRef - M_PI) < g_dPrec) || (fabs(pBounds[0].dRef + M_PI) < g_dPrec)) return 1;
      return 2;
    }

    if(RefInBounds(pBounds[0].dRef, pBounds[1].dRef, 0.0) > 2) iRes++;
    if((RefInBounds(pBounds[0].dRef, pBounds[1].dRef, M_PI) > 2) || (RefInBounds(pBounds[0].dRef, pBounds[1].dRef, -M_PI) > 2)) iRes++;
    return iRes;
  }

  if(dAng < M_PI/2.0 - g_dPrec)
  {
    if(!pBounds[0].bIsSet) return 3;
    if(!pBounds[1].bIsSet)
    {
      iRef = RefInBounds(-dAng, dAng, pBounds[0].dRef);
      if(iRef > 2) return 4;
      if(iRef > 0) return 3;
      iRef = RefInBounds(dAng, M_PI - dAng, pBounds[0].dRef);
      if(iRef > 2) return 4;
      if(iRef > 0) return 3;
      iRef = RefInBounds(M_PI - dAng, dAng - M_PI, pBounds[0].dRef);
      if(iRef > 2) return 4;
      if(iRef > 0) return 3;
      iRef = RefInBounds(dAng - M_PI, -dAng, pBounds[0].dRef);
      if(iRef > 2) return 4;
      if(iRef > 0) return 3;
      return 0;
    }

    if(RefInBounds(pBounds[0].dRef, pBounds[1].dRef, dAng) > 2) iRes++;
    if(RefInBounds(pBounds[0].dRef, pBounds[1].dRef, -dAng) > 2) iRes++;
    if(RefInBounds(pBounds[0].dRef, pBounds[1].dRef, M_PI - dAng) > 2) iRes++;
    if(RefInBounds(pBounds[0].dRef, pBounds[1].dRef, dAng - M_PI) > 2) iRes++;

    return iRes;
  }

  if(!pBounds[0].bIsSet) return 1;
  if(!pBounds[1].bIsSet)
  {
    if(fabs(pBounds[0].dRef - M_PI/2.0) < g_dPrec) return 1;
    if(fabs(pBounds[0].dRef + M_PI/2.0) < g_dPrec) return 1;
    return 2;
  }

  if(RefInBounds(pBounds[0].dRef, pBounds[1].dRef, -M_PI/2.0) > 2) iRes++;
  if(RefInBounds(pBounds[0].dRef, pBounds[1].dRef, M_PI/2.0) > 2) iRes++;
  return iRes;
}

bool ElpsRemovePart(bool bDown, PDPointList pCache, PDRefPoint pBounds)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 3) return 0;

  CDPoint cBreak = {-0.5, 0.0};
  int iBreaks = pCache->GetCount(4);
  if(iBreaks > 0) cBreak = pCache->GetPoint(0, 4).cPoint;
  if(cBreak.x < -0.5) return 0;

  double dAng = cBreak.x;
  bool b1;

  if(dAng < g_dPrec)
  {
    b1 = !pBounds[0].bIsSet || (!pBounds[1].bIsSet && ((fabs(pBounds[0].dRef) < g_dPrec) ||
        (fabs(pBounds[0].dRef - M_PI) < g_dPrec) || (fabs(pBounds[0].dRef + M_PI) < g_dPrec)));
    if(b1)
    {
      if(bDown)
      {
        pBounds[0].bIsSet = true;
        pBounds[0].dRef = -M_PI;
        pBounds[1].bIsSet = true;
        pBounds[1].dRef = 0.0;
      }
      else
      {
        pBounds[0].bIsSet = true;
        pBounds[0].dRef = 0.0;
        pBounds[1].bIsSet = true;
        pBounds[1].dRef = M_PI;
      }
      return true;
    }

    double dTestVal = pBounds[0].dRef;
    if(pBounds[1].bIsSet) dTestVal = pBounds[1].dRef;

    int iEnd = -1;
    int iRef = RefInBounds(-M_PI, 0.0, dTestVal);
    if(iRef > 0) iEnd = 0;

    if(iEnd < 0)
    {
      iRef = RefInBounds(0.0, M_PI, dTestVal);
      if(iRef > 0) iEnd = 1;
    }
    if(iEnd < 0) return false;

    if(!pBounds[1].bIsSet)
    {
      pBounds[1].bIsSet = true;
      if(bDown) pBounds[1].dRef = pBounds[0].dRef;
    }

    double *pdVal = &pBounds[1].dRef;
    if(bDown) pdVal = &pBounds[0].dRef;

    switch(iEnd)
    {
    case 0:
      if(bDown) *pdVal = -M_PI;
      else *pdVal = M_PI;
      break;
    case 1:
      *pdVal = 0.0;
      break;
    }

    return true;
  }

  if(dAng < M_PI/2.0 - g_dPrec)
  {
    if(!pBounds[0].bIsSet)
    {
      if(bDown)
      {
        pBounds[0].bIsSet = true;
        pBounds[0].dRef = -dAng;
        pBounds[1].bIsSet = true;
        pBounds[1].dRef = dAng;
      }
      else
      {
        pBounds[0].bIsSet = true;
        pBounds[0].dRef = dAng;
        pBounds[1].bIsSet = true;
        pBounds[1].dRef = -dAng;
      }
      return true;
    }

    double dTestVal = pBounds[0].dRef;
    if(pBounds[1].bIsSet) dTestVal = pBounds[1].dRef;

    int iEnd = -1;
    int iRef = RefInBounds(-dAng, dAng, dTestVal);
    if(iRef > 1) iEnd = 3;
    else if(iRef > 0) iEnd = 2;

    if(iEnd < 0)
    {
      iRef = RefInBounds(dAng, M_PI - dAng, dTestVal);
      if(iRef > 1) iEnd = 0;
      else if(iRef > 0) iEnd = 3;
    }
    if(iEnd < 0)
    {
      iRef = RefInBounds(M_PI - dAng, dAng - M_PI, dTestVal);
      if(iRef > 1) iEnd = 1;
      else if(iRef > 0) iEnd = 0;
    }
    if(iEnd < 0)
    {
      iRef = RefInBounds(dAng - M_PI, -dAng, dTestVal);
      if(iRef > 1) iEnd = 2;
      else if(iRef > 0) iEnd = 1;
    }
    if(iEnd < 0) return false;

    if(!pBounds[1].bIsSet)
    {
      pBounds[1].bIsSet = true;
      if(bDown) pBounds[1].dRef = pBounds[0].dRef;
    }

    double *pdVal = &pBounds[1].dRef;
    if(bDown) pdVal = &pBounds[0].dRef;

    switch(iEnd)
    {
    case 0:
      *pdVal = dAng;
      break;
    case 1:
      *pdVal = M_PI - dAng;
      break;
    case 2:
      *pdVal = dAng - M_PI;
      break;
    case 3:
      *pdVal = -dAng;
      break;
    }

    return true;
  }

  b1 = !pBounds[0].bIsSet || (!pBounds[1].bIsSet &&
    ((fabs(pBounds[0].dRef - M_PI/2.0) < g_dPrec) || (fabs(pBounds[0].dRef + M_PI/2.0) < g_dPrec)));

  if(b1)
  {
    if(bDown)
    {
      pBounds[0].bIsSet = true;
      pBounds[0].dRef = M_PI/2.0;
      pBounds[1].bIsSet = true;
      pBounds[1].dRef = -M_PI/2.0;
    }
    else
    {
      pBounds[0].bIsSet = true;
      pBounds[0].dRef = -M_PI/2.0;
      pBounds[1].bIsSet = true;
      pBounds[1].dRef = M_PI/2.0;
    }
    return true;
  }

  double dTestVal = pBounds[0].dRef;
  if(pBounds[1].bIsSet) dTestVal = pBounds[1].dRef;

  int iEnd = -1;
  int iRef = RefInBounds(-M_PI/2.0, M_PI/2.0, dTestVal);
  if(iRef > 2) iEnd = 0;
  else if(iRef > 0) iEnd = 1;

  if(iEnd < 0)
  {
    iRef = RefInBounds(M_PI/2.0, -M_PI/2.0, dTestVal);
    if(iRef > 2) iEnd = 1;
    else if(iRef > 0) iEnd = 0;
  }
  if(iEnd < 0) return false;

  if(!pBounds[1].bIsSet)
  {
    pBounds[1].bIsSet = true;
    if(bDown) pBounds[1].dRef = pBounds[0].dRef;
  }

  double *pdVal = &pBounds[1].dRef;
  if(bDown) pdVal = &pBounds[0].dRef;

  switch(iEnd)
  {
  case 0:
    *pdVal = -M_PI/2.0;
    break;
  case 1:
    *pdVal = M_PI/2.0;
    break;
  }
  return true;
}

