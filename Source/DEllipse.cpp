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

bool GetElpsPtProjFromStartPt(double da, double db, CDPoint cPt, PDPoint pProj)
{
    int j = 0;
    double dA = Power2(da) - Power2(db);
    double dB = db*cPt.y;
    double dC = da*cPt.x;

    double du1 = pProj->x;
    double dv1 = pProj->y;
    double du2, dv2;

    double df1 = dA*du1*dv1 + dB*du1 - dC*dv1;
    double df2 = Power2(du1) + Power2(dv1) - 1.0;
    double df11 = dA*dv1 + dB;
    double df12 = dA*du1 - dC;
    double df21 = 2.0*du1;
    double df22 = 2.0*dv1;

    double dDet = df11*df22 - df12*df21;
    bool bFound = (fabs(df1) < g_dPrec) && (fabs(df2) < g_dPrec);
    while(!bFound && (j < 8) && (fabs(dDet) > g_dPrec))
    {
        du2 = du1 - (df1*df22 - df2*df12)/dDet;
        dv2 = dv1 - (-df1*df21 + df2*df11)/dDet;
        du1 = du2;
        dv1 = dv2;

        df1 = dA*du1*dv1 + dB*du1 - dC*dv1;
        df2 = Power2(du1) + Power2(dv1) - 1.0;
        df11 = dA*dv1 + dB;
        df12 = dA*du1 - dC;
        df21 = 2.0*du1;
        df22 = 2.0*dv1;

        dDet = df11*df22 - df12*df21;
        bFound = (fabs(df1) < g_dPrec) && (fabs(df2) < g_dPrec);
        j++;
    }
    pProj->x = du1;
    pProj->y = dv1;

    return bFound;
}

/*bool PtInList(CDPoint cPt, int iSize, PDPoint pList)
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
CDPoint GetElpsPtProj(double da, double db, CDPoint cPt, CDPoint cRefPt, double dOffset, int iSrchMask, PDRefPoint pBounds)
{
    double d1 = Power2(da) - Power2(db);
    double dDist2;
    CDPoint cPt1, cPt2;
    double dNorm, dDist, dDistMin;
    CDPoint cProj, cProjMin;

    if(fabs(d1) < g_dPrec)
    {
        dDist2 = GetNorm(cPt);
        if(dDist2 > g_dPrec)
        {
            cProjMin = cPt/dDist2;
            cPt1 = da*cProjMin;
            dDistMin = GetDist(cRefPt, cPt1);
            cProj = -1.0*cProjMin;
            cPt1 = da*cProj;
            dDist = GetDist(cRefPt, cPt1);
            if(dDist < dDistMin) cProjMin = cProj;
            return cProjMin;
        }

        dDist2 = GetNorm(cRefPt);
        if(dDist2 > g_dPrec) return cRefPt/dDist2;
        cProjMin.x = 6.0;
        return cProjMin;
    }

    dNorm = sqrt(Power2(cRefPt.x/da) + Power2(cRefPt.y/db));
    if(dNorm < g_dPrec)
    {
        cProjMin.x = 6.0;
        return cProjMin;
    }

    int iSols = 0;
    CDPoint cSols[4];
    bool bInBounds[4];

    cProj.x = 1.0;
    cProj.y = 0.0;
    if(GetElpsPtProjFromStartPt(da, db, cPt, &cProj)) cSols[iSols++] = cProj;
    cProj.x = -1.0;
    cProj.y = 0.0;
    if(GetElpsPtProjFromStartPt(da, db, cPt, &cProj) && !PtInList(cProj, iSols, cSols)) cSols[iSols++] = cProj;
    cProj.x = 0.0;
    cProj.y = 1.0;
    if(GetElpsPtProjFromStartPt(da, db, cPt, &cProj) && !PtInList(cProj, iSols, cSols)) cSols[iSols++] = cProj;
    cProj.x = 0.0;
    cProj.y = -1.0;
    if(GetElpsPtProjFromStartPt(da, db, cPt, &cProj) && !PtInList(cProj, iSols, cSols)) cSols[iSols++] = cProj;

    double dt;
    if(pBounds && pBounds[0].bIsSet && pBounds[1].bIsSet)
    {
        for(int j = 0; j < iSols; j++)
        {
            dt = atan2(cSols[j].y, cSols[j].x);
            bInBounds[j] = (RefInBounds(pBounds[0].dRef, pBounds[1].dRef, dt) > 0);
        }
    }
    else
    {
        for(int j = 0; j < iSols; j++) bInBounds[j] = true;
    }

    cProjMin = cSols[0];

    cPt1.x = da*cProjMin.x;
    cPt1.y = db*cProjMin.y;
    cPt2.x = db*cProjMin.x;
    cPt2.y = da*cProjMin.y;
    dNorm = GetNorm(cPt2);
    dDistMin = GetDist(cRefPt - dOffset*cPt2/dNorm, cPt1);
    bool bFirstSet = bInBounds[0];
    bool bIsBetter;

    bool bSecondSet = false;
    CDPoint cProjSecond = cProjMin;
    double dDistSecond = dDistMin;

    int i = 1;
    while(i < iSols)
    {
        cProj = cSols[i];
        cPt1.x = da*cProj.x;
        cPt1.y = db*cProj.y;
        cPt2.x = db*cProj.x;
        cPt2.y = da*cProj.y;
        dNorm = GetNorm(cPt2);
        dDist = GetDist(cRefPt - dOffset*cPt2/dNorm, cPt1);

        bIsBetter = ((dDist < dDistMin - g_dPrec) && !(bFirstSet && !bInBounds[i])) || (!bFirstSet && bInBounds[i]);

        if(bIsBetter)
        {
            if(bSecondSet)
            {
                cProjSecond = cProjMin;
                dDistSecond = dDistMin;
            }
            cProjMin = cProj;
            dDistMin = dDist;
            bFirstSet |= bInBounds[i];
        }
        else if(bSecondSet)
        {
            if(dDist < dDistSecond)
            {
                cProjSecond = cProj;
                dDistSecond = dDist;
            }
        }
        else
        {
            cProjSecond = cProj;
            dDistSecond = dDist;
            bSecondSet = true;
        }
        i++;
    }

    if(iSrchMask & 2) return cProjSecond;

    return cProjMin;
}*/

// return cos and sin of dt
int GetElpsPtProj(double da, double db, CDPoint cPt, PDPoint pProjs, double *pdDists)
{
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
    pdDists[0] = fabs(db - cPt.y);
    pdDists[1] = fabs(db + cPt.y);
    if(fabs(cPt.y) < g_dPrec)
    {
      pProjs[2].x = 1.0;
      pProjs[2].y = 0.0;
      pProjs[3].x = -1.0;
      pProjs[3].y = 0.0;
      pdDists[2] = da;
      pdDists[3] = da;
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
    pdDists[2] = GetDist(cPt, cProjPt);
    pdDists[3] = pdDists[3];
    return 4;
  }

  if(fabs(cPt.y) < g_dPrec)
  {
    pProjs[0].x = 1.0;
    pProjs[0].y = 0.0;
    pProjs[1].x = -1.0;
    pProjs[1].y = 0.0;
    pdDists[0] = fabs(da - cPt.x);
    pdDists[1] = fabs(da + cPt.x);
    pProjs[2].x = cPt.x*da/(da2 - db2);
    dx2 = Power2(pProjs[2].x);
    if(dx2 > 1.0 - g_dPrec) return 2;
    pProjs[2].y = sqrt(1.0 - dx2);
    pProjs[3].x = pProjs[2].x;
    pProjs[3].y = -pProjs[2].y;
    cProjPt.x = da*pProjs[2].x;
    cProjPt.y = db*pProjs[2].y;
    pdDists[2] = GetDist(cPt, cProjPt);
    pdDists[3] = pdDists[3];
    return 4;
  }

  dx2 = Power2(cPt.x);
  dy2 = Power2(cPt.y);
  double pdCoefs[5];
  pdCoefs[0] = 1.0 - dx2/da2 - dy2/db2;
  pdCoefs[1] = 2.0*(da2 + db2 - dx2 - dy2)/da/db;
  pdCoefs[2] = da2/db2 + 4.0 + db2/da2 - dx2/db2 - dy2/da2;
  pdCoefs[3] = 2.0*(da2 + db2)/da/db;
  pdCoefs[4] = 1.0;
  double pdRoots[4];
  int iRoots = SolvePolynom(4, pdCoefs, pdRoots);
  int iRes = 0;
  for(int i = 0; i < iRoots; i++)
  {
    da2 = da + db*pdRoots[i];
    db2 = db + da*pdRoots[i];
    if((fabs(da2) > g_dPrec) && (fabs(db2) > g_dPrec))
    {
      cProjPt.x = cPt.x/da2;
      cProjPt.y = cPt.y/db2;
      if(GetElpsPtProjFromStartPt(da, db, cPt, &cProjPt))
      {
        pProjs[iRes] = cProjPt;
        cProjPt.x = da*pProjs[iRes].x;
        cProjPt.y = db*pProjs[iRes].y;
        pdDists[iRes++] = GetDist(cProjPt, cPt);
      }
    }
  }
  return iRes;
}

CDPoint GetElpsNearProj(double da, double db, CDPoint cPt)
{
  CDPoint pProjs[4];
  double pDists[4];
  int iRoots = GetElpsPtProj(da, db, cPt, pProjs, pDists);
  int i = 0;
  double dMin = pDists[i];
  CDPoint cRes = pProjs[i++];
  while(i < iRoots)
  {
    if(pDists[i] < dMin)
    {
      dMin = pDists[i];
      cRes = pProjs[i];
    }
    i++;
  }
  return cRes;
}

/*CDPoint GetElpsFarProj(double da, double db, CDPoint cPt)
{
  CDPoint pProjs[4];
  double pDists[4];
  int iRoots = GetElpsPtProj(da, db, cPt, pProjs, pDists);
  int i = 0;
  double dMax = pDists[i];
  CDPoint cRes = pProjs[i++];
  while(i < iRoots)
  {
    if(pDists[i] > dMax)
    {
      dMax = pDists[i];
      cRes = pProjs[i];
    }
    i++;
  }
  return cRes;
}*/

CDPoint GetElpsBoundProj(double da, double db, double dOffset, CDPoint cPt, CDPoint cPtRef, bool bSecond)
{
  CDPoint pProjs[4];
  double pDists[4];
  int iRoots = GetElpsPtProj(da, db, cPt, pProjs, pDists);

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
      dMin = pDists[0];
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

int GetQuadrant(double dx, double dQuarterLen)
{
  int iDir = 1;
  if(dx < 0.0)
  {
    iDir = -1;
    dx *= (double)iDir;
  }
  int iQuad = (int)(dx/dQuarterLen);
  dx -= (double)iQuad*dQuarterLen;
  if(dx < g_dPrec) return iQuad*iDir;
  return (iQuad + 1)*iDir;
}

// calculate ellipse length from dStart to dRef approximating (dStart, dEnd) by iSeg segments
double GetElpsPureLen(double da, double db, double dOffset, double dStart, double dEnd, int iSeg, double dRef)
{
  if(dRef - dStart < g_dPrec) return 0.0;

  double dRes = 0.0;
  double dStep = (dEnd - dStart)/(double)iSeg;

  CDPrimitive cQuad;

  double dt = dStart;
  double dco = cos(dt);
  double dsi = sin(dt);
  CDPoint cDirEnd = {-da*dsi, db*dco};
  CDPoint cDirStart;

  double dNorm = GetNorm(cDirEnd);
  cQuad.cPt3.x = da*dco + dOffset*cDirEnd.y/dNorm;
  cQuad.cPt3.y = db*dsi - dOffset*cDirEnd.x/dNorm;

  dt += dStep;
  while(dt < dRef - g_dPrec)
  {
    dco = cos(dt);
    dsi = sin(dt);

    cDirStart = cDirEnd;
    cDirEnd.x = -da*dsi;
    cDirEnd.y = db*dco;
    dNorm = GetNorm(cDirEnd);

    cQuad.cPt1 = cQuad.cPt3;
    cQuad.cPt3.x = da*dco + dOffset*cDirEnd.y/dNorm;
    cQuad.cPt3.y = db*dsi - dOffset*cDirEnd.x/dNorm;

    LineXLine(cQuad.cPt1, cDirStart, cQuad.cPt3, cDirEnd, &cQuad.cPt2);

    dRes += GetQuadLength(&cQuad, 0.0, 1.0);
    dt += dStep;
  }

  dco = cos(dt);
  dsi = sin(dt);

  cDirStart = cDirEnd;
  cDirEnd.x = -da*dsi;
  cDirEnd.y = db*dco;
  dNorm = GetNorm(cDirEnd);

  cQuad.cPt1 = cQuad.cPt3;
  cQuad.cPt3.x = da*dco + dOffset*cDirEnd.y/dNorm;
  cQuad.cPt3.y = db*dsi - dOffset*cDirEnd.x/dNorm;

  LineXLine(cQuad.cPt1, cDirStart, cQuad.cPt3, cDirEnd, &cQuad.cPt2);

  dco = cos(dRef);
  dsi = sin(dRef);

  cDirStart.x = da*dco;
  cDirStart.y = db*dsi;
  cDirEnd.x = db*dco;
  cDirEnd.y = da*dsi;
  dNorm = GetNorm(cDirEnd);

  CDPoint cpQuad[3] = {cQuad.cPt1, cQuad.cPt2, cQuad.cPt3};
  double dts[2];
  int iX = QuadXLine(cpQuad, cDirStart, cDirEnd/dNorm, &cQuad.cPt4, dts);
  dRes += GetQuadLength(&cQuad, 0.0, dts[0]);

  return dRes;
}

// da ... long semiaxis, db ... short semiaxis, dOffset ... offset from the ellipse
// dBreak ... either break angle or pi/4 if it does not exist
// dl1 ... length from zero to dBreak, dl2 ... length from dBreak to pi/2
double GetElpsLen(double da, double db, double dOffset, double dBreak, double dl1, double dl2, double dRef)
{
  double pi2 = M_PI/2.0;

  int iSeg1 = (int)Round(8.0*dBreak/M_PI);
  if((iSeg1 < 1) && (dBreak > g_dPrec)) iSeg1 = 1;
  if((iSeg1 > 3) && (dBreak < pi2 - g_dPrec)) iSeg1 = 3;

  if(dl1 < -0.5)
  {
    return GetElpsPureLen(da, db, dOffset, 0.0, dBreak, iSeg1, dRef);
  }

  int iSeg2 = 4 - iSeg1;
  if(dl2 < -0.5)
  {
    return GetElpsPureLen(da, db, dOffset, dBreak, pi2, iSeg2, dRef);
  }

  int iQuad = GetQuadrant(dRef, pi2);
  if(iQuad == 0) return 0.0;

  int iDir = 1;
  if(iQuad < 0)
  {
    iDir = -1;
    iQuad *= iDir;
  }

  double dx = (double)iDir*dRef - (double)(iQuad - 1)*pi2;

  double dRes = (double)(iQuad - 1)*(dl1 + dl2);
  if(iQuad % 2 > 0)
  {
    if(dx < dBreak - g_dPrec) dRes += GetElpsPureLen(da, db, dOffset, 0.0, dBreak, iSeg1, dx);
    else dRes += dl1 + GetElpsPureLen(da, db, dOffset, dBreak, pi2, iSeg2, dx);
  }
  else
  {
    if(pi2 - dx < dBreak - g_dPrec)
      dRes += dl1 + dl2 - GetElpsPureLen(da, db, dOffset, 0.0, dBreak, iSeg1, pi2 - dx);
    else dRes += dl2 - GetElpsPureLen(da, db, dOffset, dBreak, pi2, iSeg2, pi2 - dx);
  }
  return (double)iDir*dRes;
}

// calculate ellipse reference from dStart to dLen approximating (dStart, dEnd) by iSeg segments
double GetElpsPureRef(double da, double db, double dOffset, double dStart, double dEnd, int iSeg, double dLen)
{
  double dStep = (dEnd - dStart)/(double)iSeg;

  CDPrimitive cQuad;

  double dt = dStart;
  double dco = cos(dt);
  double dsi = sin(dt);
  CDPoint cDirStart = {-da*dsi, db*dco};

  double dNorm = GetNorm(cDirStart);
  cQuad.cPt1.x = da*dco + dOffset*cDirStart.y/dNorm;
  cQuad.cPt1.y = db*dsi - dOffset*cDirStart.x/dNorm;

  dt += dStep;
  dco = cos(dt);
  dsi = sin(dt);
  CDPoint cDirEnd = {-da*dsi, db*dco};
  cDirEnd.x = -da*dsi;
  cDirEnd.y = db*dco;
  dNorm = GetNorm(cDirEnd);
  cQuad.cPt3.x = da*dco + dOffset*cDirEnd.y/dNorm;
  cQuad.cPt3.y = db*dsi - dOffset*cDirEnd.x/dNorm;
  LineXLine(cQuad.cPt1, cDirStart, cQuad.cPt3, cDirEnd, &cQuad.cPt2);
  double dQuadLen = GetQuadLength(&cQuad, 0.0, 1.0);

  while(dQuadLen < dLen - g_dPrec)
  {
    dLen -= dQuadLen;

    dt += dStep;
    dco = cos(dt);
    dsi = sin(dt);

    cDirStart = cDirEnd;
    cDirEnd.x = -da*dsi;
    cDirEnd.y = db*dco;
    dNorm = GetNorm(cDirEnd);

    cQuad.cPt1 = cQuad.cPt3;
    cQuad.cPt3.x = da*dco + dOffset*cDirEnd.y/dNorm;
    cQuad.cPt3.y = db*dsi - dOffset*cDirEnd.x/dNorm;

    LineXLine(cQuad.cPt1, cDirStart, cQuad.cPt3, cDirEnd, &cQuad.cPt2);

    dQuadLen = GetQuadLength(&cQuad, 0.0, 1.0);
  }

  dt = GetQuadPointAtDist(&cQuad, 0.0, dLen);
  cQuad.cPt4 = GetQuadPoint(&cQuad, dt);

  CDPoint cProj = GetElpsBoundProj(da, db, dOffset, cQuad.cPt4, cQuad.cPt4, false);
  return atan2(cProj.y, cProj.x);
}

// dl1 is length from 0 to dBreak, dl2 is length from dBreak to pi/2
double GetElpsRef(double da, double db, double dOffset, double dBreak, double dl1, double dl2, double dLen)
{
  double pi2 = M_PI/2.0;
  int iSeg1 = (int)Round(8.0*dBreak/M_PI);
  if((iSeg1 < 1) && (dBreak > g_dPrec)) iSeg1 = 1;
  if((iSeg1 > 3) && (dBreak < pi2 - g_dPrec)) iSeg1 = 3;

  double dQuartLen = dl1 + dl2;
  int iQuad = GetQuadrant(dLen, dQuartLen);
  if(iQuad == 0) return 0.0;

  int iDir = 1;
  if(iQuad < 0)
  {
    iDir = -1;
    iQuad *= iDir;
  }

  int iSeg2 = 4 - iSeg1;

  double dx = (double)iDir*dLen - (double)(iQuad - 1)*dQuartLen;

  double dRes = (double)(iQuad - 1)*pi2;
  if(iQuad % 2 > 0)
  {
    if(dx < dl1 - g_dPrec) dRes += GetElpsPureRef(da, db, dOffset, 0.0, dBreak, iSeg1, dx);
    else dRes += GetElpsPureRef(da, db, dOffset, dBreak, pi2, iSeg2, dx - dl1);
  }
  else
  {
    if(dQuartLen - dx < dl1 - g_dPrec)
      dRes += pi2 - GetElpsPureRef(da, db, dOffset, 0.0, dBreak, iSeg1, dQuartLen - dx);
    else dRes += pi2 - GetElpsPureRef(da, db, dOffset, dBreak, pi2, iSeg2, dQuartLen - dx - dl1);
  }
  return (double)iDir*dRes;
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
    double dl1, dl2;
    double pi2 = M_PI/2.0;

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
        dDist = GetElpsDistFromPt(cPt1, cPt1, iSrchMask, pCache, &cPtX, NULL);

      if(iMode == 2)
      {
        if(nOffs4 > 0)
        {
          dDistOld = pPoints->GetPoint(0, 4).cPoint.x;
        }
        else if(nOffs2 > 0)
        {
          cPt1 = pPoints->GetPoint(0, 2).cPoint;
          dDistOld = GetElpsDistFromPt(cPt1, cPt1, 0, pCache, &cPtX, NULL);
        }
        else if(nOffs3 > 0)
        {
          cPt1 = pPoints->GetPoint(0, 3).cPoint;
          dDistOld = GetElpsDistFromPt(cPt1, cPt1, 2, pCache, &cPtX, NULL);
        }
        if(cTmpPt.cDirection.x > 0.5) dDist = dDistOld + cTmpPt.cDirection.y;
      }
      else if(nOffs4 > 0) dDist = pPoints->GetPoint(0, 4).cPoint.x;


      if(pdDist) *pdDist = dDist - dDistOld;
      if((fabs(dDist) > g_dPrec) || (fabs(dDistOld) > g_dPrec)) pCache->AddPoint(dDist, dDistOld, 2);

      dr = GetElspBreakAngle(da, db, -dDist, dr1, dr2);
      pCache->AddPoint(dr, 0.0, 4);
      if(dr > -0.5)
      {
        dl1 = GetElpsLen(da, db, dDist, dr, -1.0, -1.0, dr);
        dl2 = GetElpsLen(da, db, dDist, dr, dl1, -1.0, pi2);
      }
      else
      {
        dl1 = GetElpsLen(da, db, dDist, pi2/2.0, -1.0, -1.0, pi2/2.0);
        dl2 = GetElpsLen(da, db, dDist, pi2/2.0, dl1, -1.0, pi2);
      }
      pCache->AddPoint(dl1, dl2, 4);
    }
    else
    {
      pCache->AddPoint(-1.0, 0.0, 4);
      dl1 = GetElpsLen(da, db, 0.0, pi2/2.0, -1.0, -1.0, pi2/2.0);
      dl2 = GetElpsLen(da, db, 0.0, pi2/2.0, dl1, -1.0, pi2);
      pCache->AddPoint(dl1, dl2, 4);
    }
  }

  return true;
}

bool GetElpsInterLineIter(double da, double db, double dr, CDPoint cStartPt,
  CDPoint cLnOrg, CDPoint cLnDir, int iSrchMask, double *pdRes, double *pdlt)
{
  CDPoint cProjDir, cProjPt, cProjOrg;
  double dNorm;
  CDPoint cProj = {0.0, 0.0};
  int i = 0;
  while((i < 4) && (cProj.x < 4.0))
  {
    cProj = GetElpsNearProj(da, db, cStartPt);
    if(cProj.x < 4.0)
    {
      cProjDir.x = -da*cProj.y;
      cProjDir.y = db*cProj.x;
      dNorm = GetNorm(cProjDir);
      if(dNorm < g_dPrec) cProj.x = 5.0;
      else
      {
        cProjDir /= dNorm;
        cProjPt.x = da*cProj.x + dr*cProjDir.y;
        cProjPt.y = db*cProj.y - dr*cProjDir.x;
        GetLineProj(cProjPt, cLnOrg, cLnDir, &cStartPt);
      }
    }
    i++;
  }
  if(cProj.x > 4.0) return false;

  i = 0;
  int iInter = 1;
  while((i < 16) && (iInter > 0) && (cProj.x < 4.0))
  {
    cProj = GetElpsNearProj(da, db, cStartPt);
    if(cProj.x < 4.0)
    {
      cProjDir.x = -da*cProj.y;
      cProjDir.y = db*cProj.x;
      dNorm = GetNorm(cProjDir);
      if(dNorm < g_dPrec) cProj.x = 5.0;
      else
      {
        cProjDir /= dNorm;
        cProjPt.x = da*cProj.x + dr*cProjDir.y;
        cProjPt.y = db*cProj.y - dr*cProjDir.x;
        iInter = LineXLine(cProjPt, cProjDir, cLnOrg, cLnDir, &cStartPt);
        if(GetDist(cProjPt, cStartPt) < g_dPrec) iInter = 0;
      }
    }
    i++;
  }
  if((iInter > 0) || (cProj.x > 4.0)) return false;

  *pdRes = atan2(cProj.y, cProj.x);
  if(fabs(cLnDir.x) > fabs(cLnDir.y)) *pdlt = (cStartPt.x - cLnOrg.x)/cLnDir.x;
  else *pdlt = (cStartPt.y - cLnOrg.y)/cLnDir.y;
  return true;
}

int AddEllipseInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return 0;

  CDPoint cOrig, cRad, cMainDir;
  cOrig = pCache->GetPoint(0, 0).cPoint;
  cRad = pCache->GetPoint(1, 0).cPoint;

  CDPrimitive cPrim;
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

  cMainDir = pCache->GetPoint(2, 0).cPoint;

/*  if(pCache->GetCount(3) > 0)
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
  }*/

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  CDPoint cLn1 = Rotate(cPt1 - cOrig, cMainDir, false);
  CDPoint cLn2 = Rotate(cPt2 - cOrig, cMainDir, false);
  double dLnLen = GetDist(cLn1, cLn2);
  if(dLnLen < g_dPrec) return 0;

  CDPoint cNorm = (cLn2 - cLn1)/dLnLen;
  double dt1 = atan(-cRad.y*cNorm.x/cRad.x/cNorm.y);
  double dt2 = dt1 + M_PI;
  if(dt2 > M_PI + g_dPrec) dt2 = dt1 - M_PI;

  double dc1 = cos(dt1);
  double ds1 = sin(dt1);
  CDPoint cPt3, cDir3, cN3;
  cPt3.x = cRad.x*dc1;
  cPt3.y = cRad.y*ds1;
  cDir3.x = -cRad.x*ds1;
  cDir3.y = cRad.y*dc1;
  double dn3 = GetNorm(cDir3);
  cN3.x = cDir3.y/dn3;
  cN3.y = -cDir3.x/dn3;

  double du1 = cN3*(cLn1 - cPt3) - dr;
  if(du1 > -g_dPrec) return 0;

  double du2 = -(cN3*(cLn1 + cPt3)) - dr;
  if(du2 > -g_dPrec) return 0;

  double dv, du = fabs(du2);
  if(fabs(du1) < du)
  {
    du = fabs(du1);
    dv = -(cNorm*(cLn1 - cPt3));
  }
  else dv = -(cNorm*(cLn1 + cPt3));

  CDPoint cPt4 = cLn1 + (dv + du)*cNorm;
  double dt, ds;
  if(GetElpsInterLineIter(cRad.x, cRad.y, dr, cPt4, cLn1, cNorm, 0, &dt, &ds))
  {
    if((ds > g_dPrec) && (ds < dLnLen - g_dPrec) && !pBounds->HasPoint(dt))
    {
      pBounds->AddPoint(dt);
      iRes++;
    }
  }
  if(GetElpsInterLineIter(cRad.x, cRad.y, dr, cPt4, cLn1, cNorm, 2, &dt, &ds))
  {
    if((ds > g_dPrec) && (ds < dLnLen - g_dPrec) && !pBounds->HasPoint(dt))
    {
      pBounds->AddPoint(dt);
      iRes++;
    }
  }
  cPt4 = cLn1 + (dv - du)*cNorm;
  if(GetElpsInterLineIter(cRad.x, cRad.y, dr, cPt4, cLn1, cNorm, 0, &dt, &ds))
  {
    if((ds > g_dPrec) && (ds < dLnLen - g_dPrec) && !pBounds->HasPoint(dt))
    {
      pBounds->AddPoint(dt);
      iRes++;
    }
  }
  if(GetElpsInterLineIter(cRad.x, cRad.y, dr, cPt4, cLn1, cNorm, 2, &dt, &ds))
  {
    if((ds > g_dPrec) && (ds < dLnLen - g_dPrec) && !pBounds->HasPoint(dt))
    {
      pBounds->AddPoint(dt);
      iRes++;
    }
  }

  return iRes;
}

void BuildEllipseWithBounds(double da, double db, double dr, double dtStart, double dtEnd,
    CDPoint cOrig, CDPoint cMainDir, PDPrimObject pPrimList)
{
  CDPrimitive cPrim, cTmpPrim;
  cPrim.iType = 5;

  double dAngle = dtEnd - dtStart;
  int iParts = 2 + (int)4.0*dAngle/M_PI;

  CDPoint cPts[5];
  double dt, dt1, dt2, dtDist;
  dtDist = dAngle/(double)iParts;
  dt2 = dtStart;

  CDPoint cDirStart, cDirEnd;
  double dco, dsi;

  dco = cos(dt2);
  dsi = sin(dt2);
  cDirEnd.x = -da*dsi;
  cDirEnd.y = db*dco;
  double dNorm = GetNorm(cDirEnd);

  cPts[4].x = da*dco + dr*cDirEnd.y/dNorm;
  cPts[4].y = db*dsi - dr*cDirEnd.x/dNorm;

  for(int i = 0; i < iParts; i++)
  {
    dt1 = dt2;
    dt2 += dtDist;
    cPts[0] = cPts[4];
    cDirStart = cDirEnd;

    for(int j = 1; j < 5; j++)
    {
      dt = dt1 + (dt2 - dt1)*(double)j/4.0;
      dco = cos(dt);
      dsi = sin(dt);

      cDirEnd.x = -da*dsi;
      cDirEnd.y = db*dco;
      dNorm = GetNorm(cDirEnd);

      cPts[j].x = da*dco + dr*cDirEnd.y/dNorm;
      cPts[j].y = db*dsi - dr*cDirEnd.x/dNorm;
    }

    if(ApproxLineSeg(5, cPts, &cDirStart, &cDirEnd, &cTmpPrim) > -0.5)
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
      cPrim.cPt1 = cOrig + Rotate(cPts[0], cMainDir, true);
      cPrim.cPt2 = cOrig + Rotate(cPts[4], cMainDir, true);
    }

    pPrimList->AddPrimitive(cPrim);
  }
}

void BuildEllipseWithBoundsBreaks(double da, double db, double dr, double dtStart, double dtEnd,
  CDPoint cOrig, CDPoint cMainDir, double dAngle, PDPrimObject pPrimList)
{
  double dBnds[8] = {dAngle - M_PI, -dAngle, dAngle, M_PI - dAngle, M_PI + dAngle, 2*M_PI - dAngle, 2*M_PI + dAngle, 3*M_PI - dAngle};
  int iStart = 10, iEnd = 10;
  for(int i = 0; i < 8; i++)
  {
    if((iStart > 8) && (dtStart < dBnds[i] + g_dPrec)) iStart = i;
    if((iEnd > 8) && (dtEnd < dBnds[i] + g_dPrec)) iEnd = i;
  }
  if((iStart > 8) || (iEnd > 8)) return;
  if(iStart == iEnd)
  {
    if(dtStart < dtEnd)
    {
      BuildEllipseWithBounds(da, db, dr, dtStart, dtEnd, cOrig, cMainDir, pPrimList);
      return;
    }
  }
  BuildEllipseWithBounds(da, db, dr, dtStart, dBnds[iStart], cOrig, cMainDir, pPrimList);
  if(dtEnd < dtStart)
  {
    for(int i = iStart + 1; i < 5; i++)
    {
      BuildEllipseWithBounds(da, db, dr, dBnds[i - 1], dBnds[i], cOrig, cMainDir, pPrimList);
    }
    for(int i = 1; i < iEnd; i++)
    {
      BuildEllipseWithBounds(da, db, dr, dBnds[i - 1], dBnds[i], cOrig, cMainDir, pPrimList);
    }
    BuildEllipseWithBounds(da, db, dr, dBnds[iEnd - 1], dtEnd, cOrig, cMainDir, pPrimList);
    return;
  }
  for(int i = iStart + 1; i < iEnd; i++)
  {
    BuildEllipseWithBounds(da, db, dr, dBnds[i - 1], dBnds[i], cOrig, cMainDir, pPrimList);
  }
  BuildEllipseWithBounds(da, db, dr, dBnds[iEnd - 1], dtEnd, cOrig, cMainDir, pPrimList);
}

void BuildEllipseQuadsWithBounds(double da, double db, double dr, double dtStart, double dtEnd,
  CDPoint cOrig, CDPoint cMainDir, PDPrimObject pPrimList)
{
  CDPrimitive cPrim, cTmpPrim;
  cPrim.iType = 4;

  if(dtStart > dtEnd) dtEnd += 2.0*M_PI;

  double dAngle = dtEnd - dtStart;
  int iParts = 2 + (int)4.0*dAngle/M_PI;

  double dt2, dtDist;
  dtDist = dAngle/(double)iParts;
  dt2 = dtStart;

  CDPoint cDirStart, cDirEnd;
  double dco, dsi;

  dco = cos(dt2);
  dsi = sin(dt2);
  cDirEnd.x = -da*dsi;
  cDirEnd.y = db*dco;
  double dNorm = GetNorm(cDirEnd);

  cTmpPrim.cPt3.x = da*dco + dr*cDirEnd.y/dNorm;
  cTmpPrim.cPt3.y = db*dsi - dr*cDirEnd.x/dNorm;

  int iRes1 = 2;
  int iRes2 = 0;
  int k;

  for(int i = 0; i < iParts; i++)
  {
    dt2 += dtDist;
    cTmpPrim.cPt1 = cTmpPrim.cPt3;
    cDirStart = cDirEnd;

    dco = cos(dt2);
    dsi = sin(dt2);

    cDirEnd.x = -da*dsi;
    cDirEnd.y = db*dco;
    dNorm = GetNorm(cDirEnd);
    cTmpPrim.cPt3.x = da*dco + dr*cDirEnd.y/dNorm;
    cTmpPrim.cPt3.y = db*dsi - dr*cDirEnd.x/dNorm;
    LineXLine(cTmpPrim.cPt1, cDirStart, cTmpPrim.cPt3, cDirEnd, &cTmpPrim.cPt2);

    cPrim.cPt1 = cOrig + Rotate(cTmpPrim.cPt1, cMainDir, true);
    cPrim.cPt2 = cOrig + Rotate(cTmpPrim.cPt2, cMainDir, true);
    cPrim.cPt3 = cOrig + Rotate(cTmpPrim.cPt3, cMainDir, true);

    pPrimList->AddPrimitive(cPrim);
  }
}

void BuildEllipseQuadsWithBoundsBreaks(double da, double db, double dr, double dtStart, double dtEnd,
  CDPoint cOrig, CDPoint cMainDir, double dAngle, PDPrimObject pPrimList)
{
  double dBnds[8] = {dAngle - M_PI, -dAngle, dAngle, M_PI - dAngle, M_PI + dAngle, 2*M_PI - dAngle, 2*M_PI + dAngle, 3*M_PI - dAngle};
  int iStart = 10, iEnd = 10;
  for(int i = 0; i < 8; i++)
  {
    if((iStart > 8) && (dtStart < dBnds[i] + g_dPrec)) iStart = i;
    if((iEnd > 8) && (dtEnd < dBnds[i] + g_dPrec)) iEnd = i;
  }
  if((iStart > 8) || (iEnd > 8)) return;
  if(iStart == iEnd)
  {
    if(dtStart < dtEnd)
    {
      BuildEllipseQuadsWithBounds(da, db, dr, dtStart, dtEnd, cOrig, cMainDir, pPrimList);
      return;
    }
  }
  BuildEllipseQuadsWithBounds(da, db, dr, dtStart, dBnds[iStart], cOrig, cMainDir, pPrimList);
  if(dtEnd < dtStart)
  {
    for(int i = iStart + 1; i < 5; i++)
    {
      BuildEllipseQuadsWithBounds(da, db, dr, dBnds[i - 1], dBnds[i], cOrig, cMainDir, pPrimList);
    }
    for(int i = 1; i < iEnd; i++)
    {
      BuildEllipseQuadsWithBounds(da, db, dr, dBnds[i - 1], dBnds[i], cOrig, cMainDir, pPrimList);
    }
    BuildEllipseQuadsWithBounds(da, db, dr, dBnds[iEnd - 1], dtEnd, cOrig, cMainDir, pPrimList);
    return;
  }
  for(int i = iStart + 1; i < iEnd; i++)
  {
    BuildEllipseQuadsWithBounds(da, db, dr, dBnds[i - 1], dBnds[i], cOrig, cMainDir, pPrimList);
  }
  BuildEllipseQuadsWithBounds(da, db, dr, dBnds[iEnd - 1], dtEnd, cOrig, cMainDir, pPrimList);
}

double GetElpsOffset(PDPointList pCache)
{
  int nOffs = pCache->GetCount(2);
  if(nOffs < 1) return 0.0;
  return pCache->GetPoint(0, 2).cPoint.x;
}

double GetElpsDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX, PDRefPoint pBounds)
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
  //CDPoint cPt1 = Rotate(cPt - cOrig, cMainDir, false);
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dDist = pCache->GetPoint(0, 2).cPoint.x;

  int iBreaks = pCache->GetCount(4);
  CDPoint cBreak = {-1.0, 0.0};
  CDPoint cLength = {0.0, 0.0};
  if(iBreaks > 0)
  {
    cBreak = pCache->GetPoint(0, 4).cPoint;
    cLength = pCache->GetPoint(1, 4).cPoint;
  }
  if(cBreak.x < -0.5) cBreak.x = M_PI/4.0;

  *pdDist = GetElpsLen(cRad.x, cRad.y, dDist, cBreak.x, cLength.x, cLength.y, dRef);
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

  CDPoint cMainDir = pCache->GetPoint(2, 0).cPoint;
  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  int iBreaks = pCache->GetCount(4);
  CDPoint cBreak = {-1.0, 0.0};
  CDPoint cLengths = {0.0, 0.0};
  if(iBreaks > 0)
  {
    cBreak = pCache->GetPoint(0, 4).cPoint;
    cLengths = pCache->GetPoint(1, 4).cPoint;
  }
  if(cBreak.x < -0.5) cBreak.x = M_PI/4.0;
//for(int i = -20; i < 20; i++)
//{
//double dt = (double)i*M_PI/8.0;
//double dl = GetElpsLen(cRad.x, cRad.y, dr, cBreak.x, cLengths.x, cLengths.y, dt);
//double ds = GetElpsRef(cRad.x, cRad.y, dr, cBreak.x, cLengths.x, cLengths.y, dl);
//printf("%d, %f, %f, %f\n", i, dt, dl, ds);
//}

  double dt1 = GetElpsRef(cRad.x, cRad.y, dr, cBreak.x, cLengths.x, cLengths.y, d1);
  double dt2 = GetElpsRef(cRad.x, cRad.y, dr, cBreak.x, cLengths.x, cLengths.y, d2);

  BuildEllipseWithBoundsBreaks(cRad.x, cRad.y, dr, dt1, dt2,
    cOrig, cMainDir, cBreak.x, pPrimList);
}

void AddElpsExtPrim(PDRect pRect, PDPointList pCache, PDPrimObject pPrimList)
{
  int iCnt = pCache->GetCount(0);

  if(iCnt < 3) return;

  CDPoint cOrig, cRad, cMainDir;

  CDPrimitive cPrim;
  int iRes = 0;

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
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

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
  CDPoint cLengths = {0.0, 0.0};
  if(iBreaks > 0)
  {
    cBreak = pCache->GetPoint(0, 4).cPoint;
    cLengths = pCache->GetPoint(1, 4).cPoint;
  }
  if(cBreak.x < -0.5) cBreak.x = M_PI/4.0;

  *pdRef = GetElpsRef(cRad.x, cRad.y, dr, cBreak.x, cLengths.x, cLengths.y, dDist);
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

