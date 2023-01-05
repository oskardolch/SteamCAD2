#include "DSpline.hpp"
#include "DMath.hpp"
#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include "DPrimitive.hpp"

// for debugging purpose only
/*#include <windows.h>
#include <commctrl.h>
#include <wchar.h>
extern HWND g_hStatus;*/
// -----

bool AddSplinePoint(double x, double y, char iCtrl, PDPointList pPoints)
{
  if(iCtrl == 1)
  {
    int nCtrl = pPoints->GetCount(1);
    if(nCtrl < 1) pPoints->AddPoint(x, y, 1);
    else pPoints->Remove(0, 1);
    return false;
  }

  pPoints->AddPoint(x, y, 0);
  return false;
}

int GetSplineNumSegments(PDPointList pCache)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return 0;

  int nCtrls = pCache->GetCount(1);
  bool bClosed = (nCtrls > 0);
  if(bClosed && (iCnt < 3)) return 0;
  if(bClosed) return iCnt;

  if(iCnt < 4) return 1;
  return iCnt - 2;
}

CDPrimitive GetSplineNthSegment(int iPos, PDPointList pCache)
{
  int iCnt = pCache->GetCount(0);
  int nCtrls = pCache->GetCount(1);
  bool bClosed = (nCtrls > 0);

  CDPrimitive cQuad;
  CDPoint bPt1, bPt2, bPt3;

  if(iCnt < 3)
  {
    cQuad.iType = 1;
    cQuad.cPt1 = pCache->GetPoint(0, 0).cPoint;
    cQuad.cPt2 = pCache->GetPoint(1, 0).cPoint;
    return cQuad;
  }

  cQuad.iType = 4;

  int i = iPos;
  bPt1 = pCache->GetPoint(i++, 0).cPoint;
  if(i == iCnt) i = 0;
  bPt2 = pCache->GetPoint(i++, 0).cPoint;
  if(i == iCnt) i = 0;
  bPt3 = pCache->GetPoint(i++, 0).cPoint;

  if(bClosed || (iPos > 0)) cQuad.cPt1 = (bPt1 + bPt2)/2.0;
  else cQuad.cPt1 = bPt1;
  cQuad.cPt2 = bPt2;
  if(bClosed || (iPos < iCnt - 3)) cQuad.cPt3 = (bPt2 + bPt3)/2.0;
  else cQuad.cPt3 = bPt3;

  return cQuad;
}

CDPoint GetThreePointsControl(CDPoint cp1, CDPoint cp2, CDPoint cp3)
{
  double dl1 = GetDist(cp1, cp2);
  double dl2 = GetDist(cp2, cp3);
  double dt = dl1/(dl1 + dl2);

  if(dt < g_dPrec) return cp1;
  if(dt > 1.0 - g_dPrec) return cp3;

  CDPoint cRes = (cp2 - Power2(1.0 - dt)*cp1 - Power2(dt)*cp3)/dt/(1 - dt)/2.0;
  return cRes;
}

void GetMatrix(int iCount, bool bClosed, double *pdt, double *pdRes)
{
  double x1, x2, x3;

  if(bClosed)
  {
    double *pdDiag = pdRes;
    double *pdDiag1 = &pdRes[iCount];
    double *pdDiag2 = &pdRes[2*iCount - 1];
    double *pdLastCol1 = &pdRes[3*iCount - 2];
    double *pdLastCol2 = &pdRes[4*iCount - 4];

    x1 = (1.0 - pdt[0])*(1.0 - pdt[0])/2.0;
    x3 = pdt[0]*pdt[0]/2.0;
    x2 = x1 + 2.0*pdt[0]*(1.0 - pdt[0]) + x3;

    pdDiag[0] = sqrt(x2);
    pdDiag1[0] = x3/pdDiag[0];
    pdLastCol1[0] = x1/pdDiag[0];

    x1 = (1.0 - pdt[1])*(1.0 - pdt[1])/2.0;
    x3 = pdt[1]*pdt[1]/2.0;
    x2 = x1 + 2.0*pdt[1]*(1.0 - pdt[1]) + x3;

    pdDiag2[0] = x1/pdDiag[0];

    pdDiag[1] = sqrt(x2 - pdDiag1[0]*pdDiag2[0]);
    pdDiag1[1] = x3/pdDiag[1];
    pdLastCol1[1] = -pdDiag2[0]*pdLastCol1[0]/pdDiag[1];

    for(int i = 2; i < iCount - 2; i++)
    {
      x1 = (1.0 - pdt[i])*(1.0 - pdt[i])/2.0;
      x3 = pdt[i]*pdt[i]/2.0;
      x2 = x1 + 2.0*pdt[i]*(1.0 - pdt[i]) + x3;

      pdDiag2[i - 1] = x1/pdDiag[i - 1];

      pdDiag[i] = sqrt(x2 - pdDiag1[i - 1]*pdDiag2[i - 1]);
      pdDiag1[i] = x3/pdDiag[i];
      pdLastCol1[i] = -pdDiag2[i - 1]*pdLastCol1[i - 1]/pdDiag[i];
    }
    x1 = (1.0 - pdt[iCount - 2])*(1.0 - pdt[iCount - 2])/2.0;
    x3 = pdt[iCount - 2]*pdt[iCount - 2]/2.0;
    x2 = x1 + 2.0*pdt[iCount - 2]*(1.0 - pdt[iCount - 2]) + x3;

    pdDiag2[iCount - 3] = x1/pdDiag[iCount - 3];

    pdDiag[iCount - 2] = sqrt(x2 - pdDiag1[iCount - 3]*pdDiag2[iCount - 3]);
    pdDiag1[iCount - 2] = (x3 - pdDiag2[iCount - 3]*pdLastCol1[iCount - 3])/pdDiag[iCount - 2];

    x1 = (1.0 - pdt[iCount - 1])*(1.0 - pdt[iCount - 1])/2.0;
    x3 = pdt[iCount - 1]*pdt[iCount - 1]/2.0;
    x2 = x1 + 2.0*pdt[iCount - 1]*(1.0 - pdt[iCount - 1]) + x3;

    pdLastCol2[0] = x3/pdDiag[0];
    double dLastColSum = pdLastCol1[0]*pdLastCol2[0];
    for(int i = 1; i < iCount - 2; i++)
    {
      pdLastCol2[i] = -pdLastCol2[i - 1]*pdDiag1[i - 1]/pdDiag[i];
      dLastColSum += pdLastCol1[i]*pdLastCol2[i];
    }

    pdDiag2[iCount - 2] = (x1 - pdDiag1[iCount - 3]*pdLastCol2[iCount - 3])/pdDiag[iCount - 2];

    dLastColSum += pdDiag1[iCount - 2]*pdDiag2[iCount - 2];
    pdDiag[iCount - 1] = sqrt(x2 - dLastColSum);
  }
  else
  {
    double *pdDiag = pdRes;
    double *pdDiag1 = &pdRes[iCount - 2];
    double *pdDiag2 = &pdRes[2*iCount - 5];

    x3 = pdt[0]*pdt[0]/2.0;
    x2 = 2.0*pdt[0]*(1.0 - pdt[0]) + x3;
    pdDiag[0] = sqrt(x2);
    pdDiag1[0] = x3/pdDiag[0];

    for(int i = 1; i < iCount - 3; i++)
    {
      x1 = (1.0 - pdt[i])*(1.0 - pdt[i])/2.0;
      x3 = pdt[i]*pdt[i]/2.0;
      x2 = x1 + 2.0*pdt[i]*(1.0 - pdt[i]) + x3;

      pdDiag2[i - 1] = x1/pdDiag[i - 1];
      pdDiag[i] = sqrt(x2 - pdDiag1[i - 1]*pdDiag2[i - 1]);
      pdDiag1[i] = x3/pdDiag[i];
    }

    x1 = (1.0 - pdt[iCount - 3])*(1.0 - pdt[iCount - 3])/2.0;
    x2 = x1 + 2.0*pdt[iCount - 3]*(1.0 - pdt[iCount - 3]);
    pdDiag2[iCount - 4] = x1/pdDiag[iCount - 4];
    pdDiag[iCount - 3] = sqrt(x2 - pdDiag1[iCount - 4]*pdDiag2[iCount - 4]);
  }

  return;
}

bool BuildSplineCache(CDLine cTmpPt, int iMode, PDPointList pPoints, PDPointList pCache)
{
  pCache->ClearAll();

  int nCtrl = pPoints->GetCount(1);
  bool bClosed = nCtrl > 0;

  int nNorm = pPoints->GetCount(0);
  int n = nNorm;

  if(iMode == 1) n++;

  CDInputPoint cInPt;
  CDPoint cPt1, cPt2, cPt3;

  if(bClosed)
  {
    cPt1 = pPoints->GetPoint(0, 1).cPoint;
    pCache->AddPoint(cPt1.x, cPt1.y, 1);
  }

  if(bClosed && (n < 3))
  {
    if(nNorm > 0)
    {
      cInPt = pPoints->GetPoint(0, 0);
      pCache->AddPoint(cInPt.cPoint.x, cInPt.cPoint.y, 0);
    }
    if(nNorm > 1)
    {
      cInPt = pPoints->GetPoint(1, 0);
      pCache->AddPoint(cInPt.cPoint.x, cInPt.cPoint.y, 0);
    }
    if(iMode == 1) pCache->AddPoint(cTmpPt.cOrigin.x, cTmpPt.cOrigin.y, 0);
    return true;
  }

  if(!bClosed && (n < 4))
  {
    if(nNorm > 0)
    {
      cInPt = pPoints->GetPoint(0, 0);
      pCache->AddPoint(cInPt.cPoint.x, cInPt.cPoint.y, 0);
    }
    if(nNorm > 2)
    {
      cInPt = pPoints->GetPoint(0, 0);
      cPt1 = cInPt.cPoint;
      cInPt = pPoints->GetPoint(1, 0);
      cPt2 = cInPt.cPoint;
      cInPt = pPoints->GetPoint(2, 0);
      cPt3 = cInPt.cPoint;
      CDPoint cControl = GetThreePointsControl(cPt1, cPt2, cPt3);
      pCache->AddPoint(cControl.x, cControl.y, 0);
    }
    else if((nNorm > 1) && (iMode == 1))
    {
      cInPt = pPoints->GetPoint(0, 0);
      cPt1 = cInPt.cPoint;
      cInPt = pPoints->GetPoint(1, 0);
      cPt2 = cInPt.cPoint;
      cPt3 = cTmpPt.cOrigin;
      CDPoint cControl = GetThreePointsControl(cPt1, cPt2, cPt3);
      pCache->AddPoint(cControl.x, cControl.y, 0);
    }

    if(iMode == 1)
    {
      pCache->AddPoint(cTmpPt.cOrigin.x, cTmpPt.cOrigin.y, 0);
    }
    else if(nNorm > 1)
    {
      cInPt = pPoints->GetPoint(nNorm - 1, 0);
      pCache->AddPoint(cInPt.cPoint.x, cInPt.cPoint.y, 0);
    }
    return true;
  }

  int iDim = 0;
  if(bClosed) iDim = n;
  else iDim = n - 2;

  double *dt = (double*)malloc(iDim*sizeof(double));
  double dl1, dl2;

  if(bClosed)
  {
    cPt1 = pPoints->GetPoint(0, 0).cPoint;
    if(iMode == 1) cPt2 = cTmpPt.cOrigin;
    else cPt2 = pPoints->GetPoint(nNorm - 1, 0).cPoint;
    dl1 = GetDist(cPt1, cPt2);
    cPt2 = pPoints->GetPoint(1, 0).cPoint;
    dl2 = GetDist(cPt1, cPt2);
    dt[0] = dl1/(dl1 + dl2);
    for(int i = 1; i < iDim - 2; i++)
    {
      dl1 = dl2;
      cPt1 = cPt2;
      cPt2 = pPoints->GetPoint(i + 1, 0).cPoint;
      dl2 = GetDist(cPt1, cPt2);
      dt[i] = dl1/(dl1 + dl2);
    }
    cPt1 = cPt2;
    if(iMode == 1) cPt2 = cTmpPt.cOrigin;
    else cPt2 = pPoints->GetPoint(nNorm - 1, 0).cPoint;
    dl1 = dl2;
    dl2 = GetDist(cPt1, cPt2);
    dt[iDim - 2] = dl1/(dl1 + dl2);
    cPt1 = cPt2;
    cPt2 = pPoints->GetPoint(0, 0).cPoint;
    dl1 = dl2;
    dl2 = GetDist(cPt1, cPt2);
    dt[iDim - 1] = dl1/(dl1 + dl2);
  }
  else
  {
    cPt1 = pPoints->GetPoint(0, 0).cPoint;
    cPt2 = pPoints->GetPoint(1, 0).cPoint;
    dl1 = GetDist(cPt1, cPt2);
    cPt1 = cPt2;
    cPt2 = pPoints->GetPoint(2, 0).cPoint;
    dl2 = GetDist(cPt1, cPt2);
    dt[0] = dl1/(dl1 + dl2/2.0);
    for(int i = 1; i < iDim - 1; i++)
    {
      dl1 = dl2;
      cPt1 = cPt2;
      cPt2 = pPoints->GetPoint(i + 2, 0).cPoint;
      dl2 = GetDist(cPt1, cPt2);
      dt[i] = dl1/(dl1 + dl2);
    }
    dl1 = dl2;
    cPt1 = cPt2;
    if(iMode == 1) cPt2 = cTmpPt.cOrigin;
    else cPt2 = pPoints->GetPoint(iDim + 1, 0).cPoint;
    dl2 = GetDist(cPt1, cPt2);
    dt[iDim - 1] = dl1/(dl1 + 2.0*dl2);
  }

  int iMatSize = 0;
  if(bClosed) iMatSize = 5*n - 6; // n + 2*(n - 1) + 2*(n - 2)
  else iMatSize = 3*n - 8; // (n - 2) + 2*(n - 3)

  double *pdMatrix = (double*)malloc(iMatSize*sizeof(double));

  GetMatrix(n, bClosed, dt, pdMatrix);

  double *dx = (double*)malloc(iDim*sizeof(double));
  double *dy = (double*)malloc(iDim*sizeof(double));
  double *dx2 = (double*)malloc(iDim*sizeof(double));
  double *dy2 = (double*)malloc(iDim*sizeof(double));

  if(bClosed)
  {
    double *pdDiag = pdMatrix;
    double *pdDiag1 = &pdMatrix[n];
    double *pdDiag2 = &pdMatrix[2*n - 1];
    double *pdLastCol1 = &pdMatrix[3*n - 2];
    double *pdLastCol2 = &pdMatrix[4*n - 4];

    cPt1 = pPoints->GetPoint(0, 0).cPoint;
    dx[0] = cPt1.x/pdDiag[0];
    dy[0] = cPt1.y/pdDiag[0];
    for(int i = 1; i < iDim - 1; i++)
    {
      cPt1 = pPoints->GetPoint(i, 0).cPoint;
      dx[i] = (cPt1.x - pdDiag2[i - 1]*dx[i - 1])/pdDiag[i];
      dy[i] = (cPt1.y - pdDiag2[i - 1]*dy[i - 1])/pdDiag[i];
    }

    if(iMode == 1) cPt1 = cTmpPt.cOrigin;
    else cPt1 = pPoints->GetPoint(nNorm - 1, 0).cPoint;
    dx[iDim - 1] = cPt1.x - pdDiag2[iDim - 2]*dx[iDim - 2];
    dy[iDim - 1] = cPt1.y - pdDiag2[iDim - 2]*dy[iDim - 2];
    for(int i = 0; i < iDim - 2; i++)
    {
      dx[iDim - 1] -= (dx[i]*pdLastCol2[i]);
      dy[iDim - 1] -= (dy[i]*pdLastCol2[i]);
    }
    dx[iDim - 1] /= pdDiag[iDim - 1];
    dy[iDim - 1] /= pdDiag[iDim - 1];

    dx2[iDim - 1] = dx[iDim - 1]/pdDiag[iDim - 1];
    dy2[iDim - 1] = dy[iDim - 1]/pdDiag[iDim - 1];
    dx2[iDim - 2] = (dx[iDim - 2] - pdDiag1[iDim - 2]*dx2[iDim - 1])/pdDiag[iDim - 2];
    dy2[iDim - 2] = (dy[iDim - 2] - pdDiag1[iDim - 2]*dy2[iDim - 1])/pdDiag[iDim - 2];

    for(int i = iDim - 3; i >= 0; i--)
    {
      dx2[i] = (dx[i] - pdDiag1[i]*dx2[i + 1] - pdLastCol1[i]*dx2[iDim - 1])/pdDiag[i];
      dy2[i] = (dy[i] - pdDiag1[i]*dy2[i + 1] - pdLastCol1[i]*dy2[iDim - 1])/pdDiag[i];
    }

    for(int i = 0; i < iDim; i++)
    {
      pCache->AddPoint(dx2[i], dy2[i], 0);
    }
  }
  else
  {
    double *pdDiag = pdMatrix;
    double *pdDiag1 = &pdMatrix[n - 2];
    double *pdDiag2 = &pdMatrix[2*n - 5];

    cPt1 = pPoints->GetPoint(0, 0).cPoint;
    cPt2 = pPoints->GetPoint(1, 0).cPoint;
    dx[0] = (cPt2.x - cPt1.x*Power2(1.0 - dt[0]))/pdDiag[0];
    dy[0] = (cPt2.y - cPt1.y*Power2(1.0 - dt[0]))/pdDiag[0];
    for(int i = 1; i < iDim - 1; i++)
    {
      cPt1 = pPoints->GetPoint(i + 1, 0).cPoint;
      dx[i] = (cPt1.x - pdDiag2[i - 1]*dx[i - 1])/pdDiag[i];
      dy[i] = (cPt1.y - pdDiag2[i - 1]*dy[i - 1])/pdDiag[i];
    }
    cPt1 = pPoints->GetPoint(iDim, 0).cPoint;
    if(iMode == 1) cPt2 = cTmpPt.cOrigin;
    else cPt2 = pPoints->GetPoint(iDim + 1, 0).cPoint;
    dx[iDim - 1] = ((cPt1.x - cPt2.x*dt[n - 3]*dt[n - 3]) -
      pdDiag2[iDim - 2]*dx[iDim - 2])/pdDiag[iDim - 1];
    dy[iDim - 1] = ((cPt1.y - cPt2.y*dt[n - 3]*dt[n - 3]) -
      pdDiag2[iDim - 2]*dy[iDim - 2])/pdDiag[iDim - 1];

    dx2[iDim - 1] = dx[iDim - 1]/pdDiag[iDim - 1];
    dy2[iDim - 1] = dy[iDim - 1]/pdDiag[iDim - 1];

    for(int i = iDim - 2; i >= 0; i--)
    {
      dx2[i] = (dx[i] - pdDiag1[i]*dx2[i + 1])/pdDiag[i];
      dy2[i] = (dy[i] - pdDiag1[i]*dy2[i + 1])/pdDiag[i];
    }

    cPt1 = pPoints->GetPoint(0, 0).cPoint;
    pCache->AddPoint(cPt1.x, cPt1.y, 0);
    for(int i = 0; i < iDim; i++)
    {
      pCache->AddPoint(dx2[i], dy2[i], 0);
    }
    if(iMode == 1) cPt1 = cTmpPt.cOrigin;
    else cPt1 = pPoints->GetPoint(nNorm - 1, 0).cPoint;
    pCache->AddPoint(cPt1.x, cPt1.y, 0);
  }

  free(pdMatrix);

  free(dt);

  free(dy2);
  free(dx2);
  free(dy);
  free(dx);
  return true;
}

CDPrimitive SubQuad(double dt1, double dt2, CDPrimitive cQuad)
{
  double dt = dt2 - dt1;
  CDPoint cPt1 = cQuad.cPt3 - 2.0*cQuad.cPt2 + cQuad.cPt1;
  CDPoint cPt2 = cQuad.cPt2 - cQuad.cPt1;

  CDPrimitive cRes;
  cRes.iType = 4;
  cRes.cPt1 = Power2(dt1)*cPt1 + 2.0*dt1*(cQuad.cPt2 - cQuad.cPt1) + cQuad.cPt1;
  cRes.cPt2 = dt*(dt1*cPt1 + cPt2) + cRes.cPt1;
  cRes.cPt3 = Power2(dt)*cPt1 + 2.0*cRes.cPt2 - cRes.cPt1;
  return cRes;
}

// pdBreaks should be array of 4
int GetQuadAllBreaks(CDPrimitive cQuad, double dr, double *pdBreaks)
{
  CDPoint cU = cQuad.cPt2 - cQuad.cPt1;
  double dN1 = GetNorm(cU);
  if(dN1 < g_dPrec) return 0;

  CDPoint cPt2 = cQuad.cPt3 - cQuad.cPt2;
  double dN2 = GetNorm(cPt2);
  if(dN2 < g_dPrec) return 0;

  CDPoint cDir1 = cU/dN1;
  CDPoint cDir2 = Rotate(cPt2, cDir1, false);
  if(cDir2.y*dr > -g_dPrec) return 0;

  CDPoint cV = cQuad.cPt3 - 2.0*cQuad.cPt2 + cQuad.cPt1;
  double dDet = cU.x*cV.y - cU.y*cV.x;
  if(fabs(dDet) < g_dPrec) return 0;

  double dCoefs[3];
  dCoefs[0] = cU*cU - cbrt(Power2(dDet*dr/2.0));
  dCoefs[1] = 2.0*cU*cV;
  dCoefs[2] = cV*cV;

  int iDeg = GetPolyDegree(2, dCoefs);
  int iRoots = SolvePolynom(iDeg, dCoefs, pdBreaks);

  pdBreaks[2] = 0.0;
  pdBreaks[3] = 0.0;

  if(iRoots > 1)
  {
    double dt0 = -(cU*cV)/(cV*cV);
    CDPoint cPt1 = GetQuadPoint(&cQuad, dt0);
    cDir1 = GetQuadDir(&cQuad, dt0);

    // the following is a test whether the quad is actually linear in x direction after transformation - it looks it is
    /*CDPrimitive cQuad2;
    cQuad2.iType = 4;
    cQuad2.cPt1 = Rotate(cQuad.cPt1 - cPt1, cDir1, false);
    cQuad2.cPt2 = Rotate(cQuad.cPt2 - cPt1, cDir1, false);
    cQuad2.cPt3 = Rotate(cQuad.cPt3 - cPt1, cDir1, false);
    cDir2 = cQuad2.cPt3 - 2.0*cQuad2.cPt2 + cQuad2.cPt1;
    printf("%f, %f\n", cDir2.x, cDir2.y);*/

    cPt2 = Rotate(cQuad.cPt1 - cPt1, cDir1, false);
    cDir2 = GetQuadPoint(&cQuad, 1.0);
    CDPoint cPt3 = Rotate(cDir2 - cPt1, cDir1, false);
    double dt1 = cPt2.x;
    double dt2 = cPt3.x;
    if(fabs(dt2 - dt1) < g_dPrec) return iRoots;

    double da;
    if(fabs(dt1) > g_dPrec) da = cPt2.y/Power2(dt1);
    else if(fabs(dt2) > g_dPrec) da = cPt3.y/Power2(dt2);
    else return iRoots;

    double dDisc = Power2(dr) - 0.25/Power2(da);

    if(dDisc < -g_dPrec) return iRoots;
    if(dDisc < g_dPrec) return iRoots;
    double dt = sqrt(dDisc);
    pdBreaks[iRoots++] = (-dt - dt1)/(dt2 - dt1);
    pdBreaks[iRoots++] = (dt - dt1)/(dt2 - dt1);
  }
  return iRoots;
}

int GetQuadBreaks(CDPrimitive cQuad, double dr, double dt1, double dt2, double *pdBreaks)
{
  double dRoots[4];
  int iRoots = GetQuadAllBreaks(cQuad, dr, dRoots);

  int iRes = 0;
  int iLen = iRoots;
  if(iLen > 2) iLen = 2;
  for(int i = 0; i < iLen; i++)
  {
    if((dRoots[i] > dt1 + g_dPrec) && (dRoots[i] < dt2 - g_dPrec)) pdBreaks[iRes++] = dRoots[i];
  }
  if(iRoots > 3)
  {
    if((dRoots[2] > dt1 + g_dPrec) && (dRoots[2] < dt2 - g_dPrec) && (dRoots[3] > dt1 + g_dPrec) && (dRoots[3] < dt2 - g_dPrec))
    {
      pdBreaks[iRes++] = dRoots[2];
      pdBreaks[iRes++] = dRoots[3];
    }
  }
  return iRes;
}

CDPoint GetQuadBufPoint(CDPrimitive cQuad, double dr, double dt)
{
  CDPoint cDir;
  if(cQuad.iType == 1)
  {
    cDir = cQuad.cPt2 - cQuad.cPt1;
    CDPoint cRes = cQuad.cPt1 + dt*cDir;

    double dNorm = GetNorm(cDir);
    if(dNorm < g_dPrec) return cRes;

    cDir /= dNorm;
    return cRes + dr*GetNormal(cDir);
  }

  cDir = GetQuadNormal(&cQuad, dt);
  return GetQuadPoint(&cQuad, dt) + dr*cDir;
}

CDPoint GetQuadBufDir(CDPrimitive cQuad, double dr, double dt)
{
  CDPoint cDir;
  if(cQuad.iType == 1)
  {
    cDir = cQuad.cPt2 - cQuad.cPt1;
    double dNorm = GetNorm(cDir);
    if(dNorm < g_dPrec) return cDir;
    return cDir/dNorm;
  }

  cDir = GetQuadDir(&cQuad, dt);
  double dDir = 1.0;
  double dRoots[4];
  int iRoots = GetQuadAllBreaks(cQuad, dr, dRoots);
  if(iRoots > 1)
  {
    if((dt > dRoots[0] - g_dPrec) && (dt < dRoots[1] + g_dPrec)) dDir = -1.0;
  }
  return dDir*cDir;
}

CDPoint QuadFunc(void *pvData, double dt)
{
  return GetQuadPoint((PDPrimitive)pvData, dt);
}

CDPoint QuadFuncDer(void *pvData, double dt)
{
  return GetQuadDir((PDPrimitive)pvData, dt);
}

double GetQuadBufLength(CDPrimitive cQuad, double dr, double dt1, double dt2)
{
  if(cQuad.iType == 1)
  {
    double dLen = GetDist(cQuad.cPt1, cQuad.cPt2);
    return (dt2 - dt1)*dLen;
  }

  if(fabs(dr) < g_dPrec) return GetQuadLength(&cQuad, dt1, dt2);

  double dBreaks[4];
  int iBreaks = GetQuadBreaks(cQuad, dr, 0.0, 1.0, dBreaks);
  CDPoint cBreak = {-1.0, -1.0};
  if(iBreaks > 0) cBreak.x = dBreaks[0];
  if(iBreaks > 1) cBreak.y = dBreaks[1];

  double d2 = GetCurveDistAtRef(&cQuad, dr, cBreak, dt2, QuadFunc, QuadFuncDer, 0.125, 0, {0.0, 1.0});
  if(fabs(dt1) < g_dPrec) return d2;

  double d1 = GetCurveDistAtRef(&cQuad, dr, cBreak, dt1, QuadFunc, QuadFuncDer, 0.125, 0, {0.0, 1.0});
  return d2 - d1;
}

void AddQuadBufPrimitive(CDPrimitive cQuad, double dr, double dt1, double dt2, PDPrimObject pPrimList)
{
  CDPrimitive cPrim;

  if(cQuad.iType == 1)
  {
    CDPoint cDir = cQuad.cPt2 - cQuad.cPt1;
    double dLen = GetNorm(cDir);
    if(dLen < g_dPrec) return;

    cDir = GetNormal(cDir/dLen);

    cPrim.iType = 1;
    cPrim.cPt1 = (1.0 - dt1)*cQuad.cPt1 + dt1*cQuad.cPt2 + dr*cDir;
    cPrim.cPt2 = (1.0 - dt2)*cQuad.cPt1 + dt2*cQuad.cPt2 + dr*cDir;
    pPrimList->AddPrimitive(cPrim);
    return;
  }

  if(fabs(dr) < g_dPrec)
  {
    cPrim = SubQuad(dt1, dt2, cQuad);
    pPrimList->AddPrimitive(cPrim);
    return;
  }

  double dBreaks[4];
  int iBreaks = GetQuadBreaks(cQuad, dr, 0.0, 1.0, dBreaks);
  CDPoint cBreak = {-1.0, -1.0};
  if(iBreaks > 0) cBreak.x = dBreaks[0];
  if(iBreaks > 1) cBreak.y = dBreaks[1];

  AddCurveSegment(&cQuad, dr, cBreak, QuadFunc, QuadFuncDer, dt1, dt2, 0.125, 0, pPrimList);
  return;
}

CDPrimitive CompactQuad(CDPrimitive cQuad)
{
  CDPrimitive cRes;
  cRes.cPt1 = cQuad.cPt1;
  cRes.cPt2 = 2.0*(cQuad.cPt2 - cQuad.cPt1);
  cRes.cPt3 = cQuad.cPt3 - 2.0*cQuad.cPt2 + cQuad.cPt1;
  return cRes;
}

CDPrimitive GetQuadDeriv(CDPrimitive cCompQuad)
{
  CDPrimitive cRes;
  cRes.cPt1 = cCompQuad.cPt2;
  cRes.cPt2 = 2.0*cCompQuad.cPt3;
  return cRes;
}

CDPoint EvalCompQuad(CDPrimitive cCompQuad, double dt)
{
  CDPoint cRes = cCompQuad.cPt1 + dt*cCompQuad.cPt2 + Power2(dt)*cCompQuad.cPt3;
  return cRes;
}

double Bound01(double dx, bool *pbIn01)
{
  *pbIn01 = false;
  if(dx < -g_dPrec) return 0.0;
  if(dx > 1.0 + g_dPrec) return 1.0;

  *pbIn01 = true;
  if(dx < g_dPrec) return 0.0;
  if(dx > 1.0 - g_dPrec) return 1.0;
  return dx;
}

double IterQuadPtProj(CDPoint cPt, CDPrimitive cQuad, double dStart)
{
  double dt = dStart;
  CDPoint cq = GetQuadPoint(&cQuad, dt);
  CDPoint cq1 = 2.0*((1.0 - dt)*(cQuad.cPt2 - cQuad.cPt1) + dt*(cQuad.cPt3 - cQuad.cPt2));
  CDPoint cq2 = 2.0*(cQuad.cPt3 - 2.0*cQuad.cPt2 + cQuad.cPt1);
  double df = (cPt - cq)*cq1;
  double df1 = (cPt - cq)*cq2 - cq1*cq1;
  int i = 0;
  while((i < 8) && (fabs(df) > g_dPrec) && (fabs(df1) > g_dPrec))
  {
    dt -= df/df1;
    cq = GetQuadPoint(&cQuad, dt);
    cq1 = 2.0*((1.0 - dt)*(cQuad.cPt2 - cQuad.cPt1) + dt*(cQuad.cPt3 - cQuad.cPt2));
    df = (cPt - cq)*cq1;
    df1 = (cPt - cq)*cq2 - cq1*cq1;
    i++;
  }
  if(fabs(df) > g_dPrec) return dStart;
  return dt;
}

int GetQuadPtProj(CDPoint cPt, CDPrimitive cQuad, double *pdProj)
{
  bool bIn01;
  CDLine cPtX;

  if(cQuad.iType == 1)
  {
    GetPtDistFromLineSeg(cPt, cQuad.cPt1, cQuad.cPt2, &cPtX);
    *pdProj = Bound01(cPtX.dRef, &bIn01);
    if(bIn01) return 1;
    return 0;
  }

  CDPoint cDir1 = cQuad.cPt2 - cQuad.cPt1;
  CDPoint cDir2 = cQuad.cPt3 - cQuad.cPt2;
  double d1 = GetNorm(cDir1);
  double d2 = GetNorm(cDir2);

  if(d1 < g_dPrec)
  {
    if(d2 < g_dPrec)
    {
      *pdProj = 0.0;
      return 1;
    }

    GetPtDistFromLineSeg(cPt, cQuad.cPt1, cQuad.cPt3, &cPtX);
    *pdProj = Bound01(cPtX.dRef, &bIn01);
    if(bIn01) return 1;
    return 0;
  }

  if(d2 < g_dPrec)
  {
    GetPtDistFromLineSeg(cPt, cQuad.cPt1, cQuad.cPt3, &cPtX);
    *pdProj = Bound01(cPtX.dRef, &bIn01);
    if(bIn01) return 1;
    return 0;
  }

  cDir1 /= d1;
  cDir2 /= d2;

  CDPrimitive cCompQuad = CompactQuad(cQuad);
  CDPrimitive cCompDeriv = GetQuadDeriv(cCompQuad);

  double dPoly11[3], dPoly12[3];
  double dPoly21[2], dPoly22[2];

  dPoly11[0] = cCompQuad.cPt1.x;
  dPoly11[1] = cCompQuad.cPt2.x;
  dPoly11[2] = cCompQuad.cPt3.x;

  dPoly12[0] = cCompQuad.cPt1.y;
  dPoly12[1] = cCompQuad.cPt2.y;
  dPoly12[2] = cCompQuad.cPt3.y;

  dPoly21[0] = cCompDeriv.cPt1.x;
  dPoly21[1] = cCompDeriv.cPt2.x;

  dPoly22[0] = cCompDeriv.cPt1.y;
  dPoly22[1] = cCompDeriv.cPt2.y;

  double dPoly31[4], dPoly32[4];
  int iDeg31 = MultiplyPolynoms(2, 1, dPoly11, dPoly21, dPoly31);
  int iDeg32 = MultiplyPolynoms(2, 1, dPoly12, dPoly22, dPoly32);

  double dPoly41[4];
  int iDeg41 = AddPolynoms(iDeg31, iDeg32, dPoly31, dPoly32, dPoly41);

  dPoly41[0] -= (cPt.x*dPoly21[0] + cPt.y*dPoly22[0]);
  dPoly41[1] -= (cPt.x*dPoly21[1] + cPt.y*dPoly22[1]);

  int iRoots = SolvePolynom01(iDeg41, dPoly41, pdProj);
  for(int i = 0; i < iRoots; i++)
  {
    pdProj[i] = IterQuadPtProj(cPt, cQuad, pdProj[i]);
  }
  return iRoots;
}

bool GetQuadBoundProj(CDPoint cPt, CDPoint cPtRef, double dOffset, CDPrimitive cQuad, double *pdProj)
{
  double dProjs[3];
  int iProjs = GetQuadPtProj(cPt, cQuad, dProjs);
  if(iProjs < 0) return false;

  CDPoint cPt1 = GetQuadBufPoint(cQuad, dOffset, dProjs[0]);
  double dMinDist = GetDist(cPt1, cPtRef);
  double dDist;
  *pdProj = dProjs[0];

  for(int i = 1; i < iProjs; i++)
  {
    cPt1 = GetQuadBufPoint(cQuad, dOffset, dProjs[i]);
    dDist = GetDist(cPt1, cPtRef);
    if(dDist < dMinDist)
    {
      *pdProj = dProjs[i];
      dMinDist = dDist;
    }
  }
  return true;
}

double GetQuadDistFromPoint(CDPoint cPt, double dOffset, CDPrimitive cQuad, double *pdProj)
{
  *pdProj = 0.0;
  CDPoint cPt1 = GetQuadBufPoint(cQuad, dOffset, 0.0);
  double dMinDist = GetDist(cPt1, cPt);
  cPt1 = GetQuadBufPoint(cQuad, dOffset, 1.0);
  double dDist = GetDist(cPt1, cPt);
  if(dDist < dMinDist)
  {
    *pdProj = 1.0;
    dMinDist = dDist;
  }

  double dProjs[3];
  int iProjs = GetQuadPtProj(cPt, cQuad, dProjs);

  for(int i = 0; i < iProjs; i++)
  {
    cPt1 = GetQuadBufPoint(cQuad, dOffset, dProjs[i]);
    dDist = GetDist(cPt1, cPt);
    if(dDist < dMinDist)
    {
      *pdProj = dProjs[i];
      dMinDist = dDist;
    }
  }
  return dMinDist;
}

int GetSplineAttractors(CDPoint cPt, PDPointList pCache, double dScale, PDPointList pPoints)
{
  int iRes = 0;

  int iSegs = GetSplineNumSegments(pCache);
  if(iSegs < 1) return 0;

  int nCtrls = pCache->GetCount(1);
  bool bClosed = (nCtrls > 0);

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  CDPoint bPt;
  CDPrimitive cQuad = GetSplineNthSegment(0, pCache);
  double dProjs[3];
  int iProjs = GetQuadPtProj(cPt, cQuad, dProjs);
  for(int j = 0; j < iProjs; j++)
  {
    bPt = GetQuadBufPoint(cQuad, dr, dProjs[j]);
    pPoints->AddPoint(dScale*bPt.x, dScale*bPt.y, 0);
    iRes++;
  }

  for(int i = 1; i < iSegs - 1; i++)
  {
    cQuad = GetSplineNthSegment(i, pCache);
    iProjs = GetQuadPtProj(cPt, cQuad, dProjs);
    if(iProjs > 0)
    {
      if(dProjs[0] > g_dPrec)
      {
        bPt = GetQuadBufPoint(cQuad, dr, dProjs[0]);
        pPoints->AddPoint(dScale*bPt.x, dScale*bPt.y, 0);
        iRes++;
      }
    }
    for(int j = 1; j < iProjs; j++)
    {
      bPt = GetQuadBufPoint(cQuad, dr, dProjs[j]);
      pPoints->AddPoint(dScale*bPt.x, dScale*bPt.y, 0);
      iRes++;
    }
  }

  if(iSegs > 1)
  {
    cQuad = GetSplineNthSegment(iSegs - 1, pCache);
    iProjs = GetQuadPtProj(cPt, cQuad, dProjs);
    if(iProjs > 0)
    {
      if((dProjs[0] > g_dPrec) && ((iProjs > 1) || !bClosed || (dProjs[0] < 1.0 - g_dPrec)))
      {
        bPt = GetQuadBufPoint(cQuad, dr, dProjs[0]);
        pPoints->AddPoint(dScale*bPt.x, dScale*bPt.y, 0);
        iRes++;
      }
    }
    for(int j = 1; j < iProjs - 1; j++)
    {
      bPt = GetQuadBufPoint(cQuad, dr, dProjs[j]);
      pPoints->AddPoint(dScale*bPt.x, dScale*bPt.y, 0);
      iRes++;
    }
    if(iProjs > 1)
    {
      if(!bClosed || (dProjs[iProjs - 1] < 1.0 - g_dPrec))
      {
        bPt = GetQuadBufPoint(cQuad, dr, dProjs[iProjs - 1]);
        pPoints->AddPoint(dScale*bPt.x, dScale*bPt.y, 0);
        iRes++;
      }
    }
  }

  return iRes;
}

int AddMin(int iMins, CDPoint cPt1, CDPoint cPt2, double dRef, double *pdRefs, double *pdMins)
{
  double dMin = GetDist(cPt1, cPt2);

  if(iMins < 1)
  {
    pdRefs[0] = dRef;
    pdMins[0] = dMin;
    return 1;
  }

  if(iMins < 2)
  {
    if(dMin < pdMins[0])
    {
      pdRefs[1] = pdRefs[0];
      pdMins[1] = pdMins[0];
      pdRefs[0] = dRef;
      pdMins[0] = dMin;
    }
    else
    {
      pdRefs[1] = dRef;
      pdMins[1] = dMin;
    }
    return 2;
  }

  if(dMin < pdMins[0])
  {
    pdRefs[1] = pdRefs[0];
    pdMins[1] = pdMins[0];
    pdRefs[0] = dRef;
    pdMins[0] = dMin;
  }
  else if(dMin < pdMins[1])
  {
    pdRefs[1] = dRef;
    pdMins[1] = dMin;
  }
  return 2;
}

double GetSplineBoundProj(PDPointList pCache, CDPoint cPt, CDPoint cRefPt, bool bSecond, double *pdDist)
{
  int iSegs = GetSplineNumSegments(pCache);
  if(iSegs < 1) return -1.0;

  int nCtrls = pCache->GetCount(1);
  bool bClosed = (nCtrls > 0);

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  double dRefs[2], dMins[2];
  int iMins = 0;

  CDPoint bPt;
  CDPrimitive cQuad = GetSplineNthSegment(0, pCache);
  double dProjs[3];
  int iProjs = GetQuadPtProj(cPt, cQuad, dProjs);
  for(int j = 0; j < iProjs; j++)
  {
    bPt = GetQuadBufPoint(cQuad, dr, dProjs[j]);
    iMins = AddMin(iMins, cRefPt, bPt, dProjs[j], dRefs, dMins);
  }

  for(int i = 1; i < iSegs - 1; i++)
  {
    cQuad = GetSplineNthSegment(i, pCache);
    iProjs = GetQuadPtProj(cPt, cQuad, dProjs);
    if(iProjs > 0)
    {
      if(dProjs[0] > g_dPrec)
      {
        bPt = GetQuadBufPoint(cQuad, dr, dProjs[0]);
        iMins = AddMin(iMins, cRefPt, bPt, (double)i + dProjs[0], dRefs, dMins);
      }
    }
    for(int j = 1; j < iProjs; j++)
    {
      bPt = GetQuadBufPoint(cQuad, dr, dProjs[j]);
      iMins = AddMin(iMins, cRefPt, bPt, (double)i + dProjs[j], dRefs, dMins);
    }
  }

  if(iSegs > 1)
  {
    int i = iSegs - 1;
    cQuad = GetSplineNthSegment(i, pCache);
    iProjs = GetQuadPtProj(cPt, cQuad, dProjs);
    if(iProjs > 0)
    {
      if((dProjs[0] > g_dPrec) && ((iProjs > 1) || !bClosed || (dProjs[0] < 1.0 - g_dPrec)))
      {
        bPt = GetQuadBufPoint(cQuad, dr, dProjs[0]);
        iMins = AddMin(iMins, cRefPt, bPt, (double)i + dProjs[0], dRefs, dMins);
      }
    }
    for(int j = 1; j < iProjs - 1; j++)
    {
      bPt = GetQuadBufPoint(cQuad, dr, dProjs[j]);
      iMins = AddMin(iMins, cRefPt, bPt, (double)i + dProjs[j], dRefs, dMins);
    }
    if(iProjs > 1)
    {
      if(!bClosed || (dProjs[iProjs - 1] < 1.0 - g_dPrec))
      {
        bPt = GetQuadBufPoint(cQuad, dr, dProjs[iProjs - 1]);
        iMins = AddMin(iMins, cRefPt, bPt, (double)i + dProjs[iProjs - 1], dRefs, dMins);
      }
    }
  }
  if(iMins < 1) return -1.0;

  if(bSecond)
  {
    if(iMins < 2)
    {
      if(pdDist) *pdDist = dMins[0];
      return dRefs[0];
    }
    if(pdDist) *pdDist = dMins[1];
    return dRefs[1];
  }
  if(pdDist) *pdDist = dMins[0];
  return dRefs[0];
}

double GetQuadBufProjAtDist(CDPrimitive cQuad, double dr, double t1, double dDist)
{
  if(cQuad.iType == 1)
  {
    CDPoint cDir = cQuad.cPt2 - cQuad.cPt1;
    double dNorm = GetNorm(cDir);
    if(dNorm < g_dPrec) return t1;
    return t1 + dDist/dNorm;
  }

  if(fabs(dr) < g_dPrec) return GetQuadPointAtDist(&cQuad, t1, dDist);

  double dBreaks[4];
  int iBreaks = GetQuadBreaks(cQuad, dr, 0.0, 1.0, dBreaks);
  CDPoint cBreak = {-1.0, -1.0};
  if(iBreaks > 0) cBreak.x = dBreaks[0];
  if(iBreaks > 1) cBreak.y = dBreaks[1];

  if(fabs(t1) > g_dPrec) dDist += GetQuadBufLength(cQuad, dr, 0.1, t1);

  CDPoint cPt1 = GetCurveRefAtDist(&cQuad, dr, cBreak, dDist, QuadFunc, QuadFuncDer, 0.125, 0, {0.0, 1.0});

  double dt = t1;
  GetQuadBoundProj(cPt1, cPt1, dr, cQuad, &dt);
  return dt;
}

int GetRefIndex(double dRef, double *pdt)
{
  int k = (int)(dRef + g_dPrec);
  *pdt = dRef - (double)k;
  if((k > 0) && (*pdt < g_dPrec))
  {
    k--;
    *pdt = 1.0;
  }
  return k; 
}

double GetQuadAtRef(double dRef, PDPrimitive pQuad, PDPointList pCache)
{
  double dt;
  int k = GetRefIndex(dRef, &dt);
  *pQuad = GetSplineNthSegment(k, pCache);
  return dt;
}

double GetSplineDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSrchMask, PDPointList pCache, PDLine pPtX)
{
  pPtX->bIsSet = false;

  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return 0.0;

  int nCtrl = pCache->GetCount(1);
  bool bClosed = (nCtrl > 0);

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  CDPrimitive cQuad;
  double d1 = 0.0, d2;
  CDPoint bPt1, bPt2, cDir;

  if(iCnt < 3)
  {
    bPt1 = pCache->GetPoint(0, 0).cPoint;
    bPt2 = pCache->GetPoint(1, 0).cPoint;
    d2 = GetPtDistFromLineSeg(cPt, bPt1, bPt2, pPtX);
    pPtX->cOrigin += dr*pPtX->cDirection;
    return d2 - dr;
  }

  double dDist = 0.0;
  double dProj = GetSplineBoundProj(pCache, cPt, cRefPt, iSrchMask & 2, &dDist);
  if(!bClosed)
  {
    pPtX->cDirection = 0;

    int iNumSegs = GetSplineNumSegments(pCache);
    cQuad = GetSplineNthSegment(0, pCache);
    cDir = GetQuadNormal(&cQuad, 0.0);
    bPt1 = cQuad.cPt1 + dr*cDir;
    bPt2 = Rotate(cPt - bPt1, cDir, false);
    d1 = GetDist(cPt, bPt1);
    if(bPt2.x < 0.0) d1 *= -1.0;

    if((dProj < -0.5) || (fabs(d1) < dDist))
    {
      pPtX->bIsSet = true;
      pPtX->cOrigin = bPt1;
      pPtX->dRef = 0.0;
    }

    cQuad = GetSplineNthSegment(iNumSegs - 1, pCache);
    cDir = GetQuadNormal(&cQuad, 1.0);
    bPt1 = cQuad.cPt3 + dr*cDir;
    bPt2 = Rotate(cPt - bPt1, cDir, false);
    d2 = GetDist(cPt, bPt1);
    if(bPt2.x < 0.0) d2 *= -1.0;

    if((pPtX->bIsSet && (fabs(d2) < fabs(d1))) || (!pPtX->bIsSet && ((dProj < -0.5) || (fabs(d2) < dDist))))
    {
      pPtX->bIsSet = true;
      pPtX->cOrigin = bPt1;
      pPtX->dRef = (double)iNumSegs;
      d1 = d2;
    }
    if(pPtX->bIsSet) return d1;
  }

  double dt = GetQuadAtRef(dProj, &cQuad, pCache);

  pPtX->bIsSet = true;
  cDir = GetQuadNormal(&cQuad, dt);
  pPtX->cOrigin = GetQuadPoint(&cQuad, dt) + dr*cDir;
  bPt1 = cPt - pPtX->cOrigin;
  bPt2 = Rotate(bPt1, cDir, false);
  pPtX->cDirection = cDir;
  pPtX->dRef = dProj;

  if((dt < g_dPrec) || (dt > 1.0 - g_dPrec))
  {
    d2 = GetNorm(bPt1);
    if(bPt2.x < 0.0) d2 *= -1.0;
  }
  else d2 = bPt2.x;

  return d2;
}

bool HasSplineEnoughPoints(PDPointList pPoints)
{
  int nNorm = pPoints->GetCount(0);
  int nCtrl = pPoints->GetCount(1);

  bool bRes = (nNorm > 1);
  if(nCtrl > 0) bRes = (nNorm > 2); // spline is closed

  return bRes;
}

double GetSplineRefAtDist(double dDist, double dExt, PDPointList pCache)
{
  if(dDist < g_dPrec) return 0.0;

  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  int iSegs = GetSplineNumSegments(pCache);
  int i = 0;
  CDPrimitive cQuad;
  double dLen;
  bool bFound = false;
  while(!bFound && (i < iSegs))
  {
    cQuad = GetSplineNthSegment(i++, pCache);
    dLen = GetQuadBufLength(cQuad, dr, 0.0, 1.0);
    bFound = dDist < dLen - g_dPrec;
    if(!bFound) dDist -= dLen;
  }

  if(!bFound)
  {
    if(dDist < g_dPrec) return (double)i;
    return -1.0;
  }

  return (double)(i - 1) + GetQuadBufProjAtDist(cQuad, dr, 0.0, dDist);
}

double GetSplineDistAtRef(double dRef, double dExt, PDPointList pCache)
{
  if(dRef < g_dPrec) return 0.0;

  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  int iSegs = GetSplineNumSegments(pCache);
  int i = 0;
  CDPrimitive cQuad;
  double dLen = 0.0;
  bool bFound = dRef < 1.0 - g_dPrec;
  while(!bFound && (i < iSegs))
  {
    cQuad = GetSplineNthSegment(i++, pCache);
    dLen += GetQuadBufLength(cQuad, dr, 0.0, 1.0);
    dRef -= 1.0;
    bFound = dRef < 1.0 - g_dPrec;
  }

  if(!bFound && (dRef > 1.0 + g_dPrec)) return -1.0;
  else if(i < iSegs) cQuad = GetSplineNthSegment(i, pCache);
  dLen += GetQuadBufLength(cQuad, dr, 0.0, dRef);
  return dLen;
}

double GetSplineRadiusAtPt(CDLine cPtX, PDPointList pCache, PDLine pPtR, bool bNewPt)
{
  pPtR->bIsSet = false;
  pPtR->cOrigin = 0;
  pPtR->cDirection = 0;

  int iCnt = pCache->GetCount(0);

  if(iCnt < 2) return -1.0;

  CDPoint cDir, cQuad2;
  double dNorm;
  CDPrimitive cQuad, cQuad1;

  if(iCnt < 3)
  {
    cQuad.cPt1 = pCache->GetPoint(0, 0).cPoint;
    cQuad.cPt2 = pCache->GetPoint(1, 0).cPoint;
    cDir = cQuad.cPt2 - cQuad.cPt1;
    dNorm = GetNorm(cDir);
    if(dNorm > g_dPrec) pPtR->cDirection = GetNormal(cDir/dNorm);
    return -1.0;
  }

  int nCtrl = pCache->GetCount(1);
  bool bClosed = (nCtrl > 0);
  CDPoint bPt1, bPt2, bPt3;
  double dt = -1.0;

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr = pCache->GetPoint(0, 2).cPoint.x;

  if(bNewPt)
  {
    if(bClosed)
    {
      bPt1 = pCache->GetPoint(iCnt - 2, 0).cPoint;
      bPt2 = pCache->GetPoint(iCnt - 1, 0).cPoint;
      bPt3 = pCache->GetPoint(0, 0).cPoint;
      cQuad.cPt1 = (bPt1 + bPt2)/2.0;
      cQuad.cPt2 = bPt2;
      cQuad.cPt3 = (bPt2 + bPt3)/2.0;
      if(GetQuadBoundProj(cPtX.cOrigin, cPtX.cOrigin, dr, cQuad, &dt))
      dt += (double)(iCnt - 2);
    }
    else
    {
      bPt1 = pCache->GetPoint(iCnt - 3, 0).cPoint;
      bPt2 = pCache->GetPoint(iCnt - 2, 0).cPoint;
      bPt3 = pCache->GetPoint(iCnt - 1, 0).cPoint;
      if(iCnt > 3) cQuad.cPt1 = (bPt1 + bPt2)/2.0;
      else cQuad.cPt1 = bPt1;
      cQuad.cPt2 = bPt2;
      cQuad.cPt3 = bPt3;
      dt = 1.0;
    }
  }
  else
  {
    int k = (int)cPtX.dRef;
    dt = cPtX.dRef - (double)k;
    if(k < 0) k = 0;
    if(k > iCnt - 1) k = 0;
    bPt1 = pCache->GetPoint(k++, 0).cPoint;
    if(k > iCnt - 1) k = 0;
    bPt2 = pCache->GetPoint(k++, 0).cPoint;
    if(k > iCnt - 1) k = 0;
    bPt3 = pCache->GetPoint(k++, 0).cPoint;
    if(bClosed || (k - 3 > 0)) cQuad.cPt1 = (bPt1 + bPt2)/2.0;
    else cQuad.cPt1 = bPt1;
    cQuad.cPt2 = bPt2;
    if(bClosed || (k < iCnt)) cQuad.cPt3 = (bPt2 + bPt3)/2.0;
    else cQuad.cPt3 = bPt3;
  }

  if(dt < 0) return -1.0;
  cQuad1.cPt1 = 2.0*(cQuad.cPt2 - cQuad.cPt1);
  cQuad1.cPt2 = 2.0*(cQuad.cPt3 - cQuad.cPt2);
  cQuad2 = cQuad1.cPt2 - cQuad1.cPt1;

  bPt1 = GetQuadPoint(&cQuad, dt);
  bPt2 = (1.0 - dt)*cQuad1.cPt1 + dt*cQuad1.cPt2;
  bPt3 = cQuad2;

  double dDet = bPt2.x*bPt3.y - bPt2.y*bPt3.x;
  if(fabs(dDet) < g_dPrec)
  {
    cDir = cQuad.cPt3 - cQuad.cPt2;
    dNorm = GetNorm(cDir);
    if(dNorm > g_dPrec) pPtR->cDirection = GetNormal(cDir/dNorm);
    return -1.0;
  }

  double d1 = bPt2*bPt2;
  pPtR->bIsSet = true;
  pPtR->cOrigin.x = bPt1.x - d1*bPt2.y/dDet;
  pPtR->cOrigin.y = bPt1.y + d1*bPt2.x/dDet;
  cDir = cPtX.cOrigin - pPtR->cOrigin;
  dNorm = GetNorm(cDir);
  if(dNorm > g_dPrec) pPtR->cDirection = cDir/dNorm;

  return dNorm;
}

bool GetSplinePointRefDist(double dRef, double dOffset, PDPointList pCache, double *pdDist)
{
  double dDist = GetSplineDistAtRef(dRef, dOffset, pCache);
  if(dDist < -0.5) return false;
  *pdDist = dDist;
  return true;
}

void AddSplineSegment(double dt1, double dt2, double dExt, bool bReverse, PDPointList pCache, PDPrimObject pPrimList)
{
  int iSegs = GetSplineNumSegments(pCache);
  if(iSegs < 1) return;

  int nCtrls = pCache->GetCount(1);
  bool bClosed = (nCtrls > 0);
  double dOff1 = 0.0;
  double dOff2 = 0.0;

  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  if(bClosed)
  {
    double dWhole = (double)iSegs;
    if(dt1 > dWhole + g_dPrec)
    {
      dt1 -= dWhole;
      dOff1 = dWhole;
    }
    else if(dt1 > dWhole - g_dPrec) dt1 = dWhole;
    if(dt2 > dWhole + g_dPrec)
    {
      dt2 -= dWhole;
      dOff2 = dWhole;
    }
    else if(dt2 > dWhole - g_dPrec) dt2 = dWhole;
  }

  if((dt1 < -0.5) || (dt2 < -0.5)) return;

  PDPrimObject pTmpList = new CDPrimObject();

  double dStart, dEnd;
  int i1 = GetRefIndex(dt1, &dStart);
  int i2 = GetRefIndex(dt2, &dEnd);

  dt1 += dOff1;
  dt2 += dOff2;

  CDPrimitive cQuad = GetSplineNthSegment(i1, pCache);
  if((i2 < i1) || ((i2 == i1) && ((dt2 < dt1) || (dt2 > (double)iSegs + g_dPrec))))
  {
    AddQuadBufPrimitive(cQuad, dr, dStart, 1.0, pTmpList);
    for(int i = i1 + 1; i < iSegs; i++)
    {
      cQuad = GetSplineNthSegment(i, pCache);
      AddQuadBufPrimitive(cQuad, dr, 0.0, 1.0, pTmpList);
    }
    for(int i = 0; i < i2; i++)
    {
      cQuad = GetSplineNthSegment(i, pCache);
      AddQuadBufPrimitive(cQuad, dr, 0.0, 1.0, pTmpList);
    }
    cQuad = GetSplineNthSegment(i2, pCache);
    AddQuadBufPrimitive(cQuad, dr, 0.0, dEnd, pTmpList);
    if(bReverse)
    {
      PDPrimObject pList1 = new CDPrimObject();
      ReversePrimitives(pTmpList, pList1);
      pPrimList->CopyFrom(pList1);
      delete pList1;
    }
    else pPrimList->CopyFrom(pTmpList);
    delete pTmpList;
    return;
  }

  if(i1 == i2)
  {
    AddQuadBufPrimitive(cQuad, dr, dStart, dEnd, pTmpList);
    if(bReverse)
    {
      PDPrimObject pList1 = new CDPrimObject();
      ReversePrimitives(pTmpList, pList1);
      pPrimList->CopyFrom(pList1);
      delete pList1;
    }
    else pPrimList->CopyFrom(pTmpList);
    delete pTmpList;
    return;
  }

  AddQuadBufPrimitive(cQuad, dr, dStart, 1.0, pTmpList);
  for(int i = i1 + 1; i < i2; i++)
  {
    cQuad = GetSplineNthSegment(i, pCache);
    AddQuadBufPrimitive(cQuad, dr, 0.0, 1.0, pTmpList);
  }
  cQuad = GetSplineNthSegment(i2, pCache);
  AddQuadBufPrimitive(cQuad, dr, 0.0, dEnd, pTmpList);
  if(bReverse)
  {
    PDPrimObject pList1 = new CDPrimObject();
    ReversePrimitives(pTmpList, pList1);
    pPrimList->CopyFrom(pList1);
    delete pList1;
  }
  else pPrimList->CopyFrom(pTmpList);
  delete pTmpList;
  return;
}

bool GetSplineRefPoint(double dRef, double dExt, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  double dr = dExt;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  CDPrimitive cQuad;
  double dt = GetQuadAtRef(dRef, &cQuad, pCache);
  *pPt = GetQuadBufPoint(cQuad, dr, dt);
  return true;
}

bool GetSplineRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt,
  PDPointList pCache)
{
  if(iMode != 2) return false;

  double dDist = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dDist = pCache->GetPoint(0, 2).cPoint.y;

  double dRad = dDist + dRestrictValue;

  double dRef = GetSplineBoundProj(pCache, cPt, cPt, false, NULL);
  CDPrimitive cQuad;
  double dt = GetQuadAtRef(dRef, &cQuad, pCache);

  *pSnapPt = GetQuadBufPoint(cQuad, dRad, dt);
  return true;
}

bool GetSplineRefDir(double dRef, PDPointList pCache, PDPoint pPt)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return false;

  CDPoint bPt1, bPt2;

  if(iCnt < 3)
  {
    bPt1 = pCache->GetPoint(0, 0).cPoint;
    bPt2 = pCache->GetPoint(1, 0).cPoint;

    CDPoint cDir = bPt2 - bPt1;
    double dNorm = GetNorm(cDir);
    if(dNorm < g_dPrec) return false;

    *pPt = cDir/dNorm;
    return true;
  }

  CDPrimitive cQuad;
  double dt = GetQuadAtRef(dRef, &cQuad, pCache);

  *pPt = GetQuadDir(&cQuad, dt);
  return true;
}

bool GetSplineReference(double dDist, double dOffset, PDPointList pCache, double *pdRef)
{
  double dRef = GetSplineRefAtDist(dDist, dOffset, pCache);
  if(dRef < -0.5) return false;
  *pdRef = dRef;
  return true;
}

double QuadProjFunc(void *pvData, double dOffset, CDPoint cPt, CDPoint cStart, CDPoint cEnd)
{
  CDPrimitive cQuad = *(PDPrimitive)pvData;
  double pProjs[4];
  int iRoots = GetQuadPtProj(cPt, cQuad, pProjs);
  double pDists[4];
  bool pValid[4];

  CDPoint cNorm, cProjPt;
  double du;

  for(int i = 0; i < iRoots; i++)
  {
    du = pProjs[i];
    pValid[i] = GetRefInUboundSeg(du, cStart, cEnd);
    cProjPt = GetQuadPoint(&cQuad, du);
    cNorm = GetQuadNormal(&cQuad, du);
    pDists[i] = GetDist(cPt, cProjPt + dOffset*cNorm);
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

int AddQuadBufInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, CDPrimitive cQuad, bool bIncludeLast, double *pdBounds)
{
  int iRes = 0;
  if(cQuad.iType == 1)
  {
    CDPoint cPt21, cPt22, cDir, cRes;
    cDir = cQuad.cPt2 - cQuad.cPt1;
    double dNorm = GetNorm(cDir);
    if(dNorm < g_dPrec) return 0;
    cDir = GetNormal(cDir/dNorm);
    cPt21 = cQuad.cPt1 + dOffset*cDir;
    cPt22 = cQuad.cPt2 + dOffset*cDir;
    iRes = SegXSegParams(cPt1, cPt2, cPt21, cPt22, &cRes);
    if(iRes > 0)
    {
      if((cRes.y > 1.0 - g_dPrec) && !bIncludeLast) return 0;
      pdBounds[0] = cRes.y;
    }
    return iRes;
  }

  if(fabs(dOffset) < g_dPrec)
  {
    CDPoint cPts[2];
    CDPoint cPtQuad[3];
    cPtQuad[0] = cQuad.cPt1;
    cPtQuad[1] = cQuad.cPt2;
    cPtQuad[2] = cQuad.cPt3;
    iRes = QuadXSeg(cPtQuad, cPt1, cPt2, cPts, pdBounds);
    if(iRes > 0)
    {
      if((pdBounds[iRes - 1] > 1.0 - g_dPrec) && !bIncludeLast) return iRes - 1;
    }
    return iRes;
  }

  CDPoint cDir = cPt2 - cPt1;

  double dTangent = -1.0;
  CDPoint cTan;
  if(Solve2x2Matrix(cQuad.cPt3 - 2.0*cQuad.cPt2 + cQuad.cPt1, -1.0*cDir, cQuad.cPt1 - cQuad.cPt2, &cTan)) dTangent = cTan.x;

  double dt = 0.0;
  double dBreaks[4];
  int iBreaks = GetQuadBreaks(cQuad, dOffset, 0.0, 1.0, dBreaks);
  if(iBreaks > 0)
  {
    iRes += AddCurveInterLine(&cQuad, dOffset, QuadFunc, QuadFuncDer, QuadProjFunc,
      {1.0, dTangent}, {1.0, dt}, {1.0, dBreaks[0]}, cPt1, cPt2, &pdBounds[iRes]);
    dt = dBreaks[0];
  }
  if(iBreaks > 1)
  {
    iRes += AddCurveInterLine(&cQuad, dOffset, QuadFunc, QuadFuncDer, QuadProjFunc,
      {1.0, dTangent}, {1.0, dt}, {1.0, dBreaks[1]}, cPt1, cPt2, &pdBounds[iRes]);
    dt = dBreaks[1];
  }
  iRes += AddCurveInterLine(&cQuad, dOffset, QuadFunc, QuadFuncDer, QuadProjFunc,
    {1.0, dTangent}, {1.0, dt}, {1.0, 1.0}, cPt1, cPt2, &pdBounds[iRes]);

  return iRes;
}

int AddSplineInterLine(CDPoint cPt1, CDPoint cPt2, double dOffset, PDPointList pCache, PDRefList pBounds)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return 0;

  int nCtrl = pCache->GetCount(1);
  bool bClosed = (nCtrl > 0);

  int iRes = 0;

  double dr = dOffset;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  int iSegs = GetSplineNumSegments(pCache);
  CDPrimitive cQuad;
  double dRoots[4];
  int iRoots;
  double dx;
  for(int i = 0; i < iSegs; i++)
  {
    cQuad = GetSplineNthSegment(i, pCache);
    iRoots = AddQuadBufInterLine(cPt1, cPt2, dr, cQuad, !bClosed && (i == iSegs - 1), dRoots);
    for(int j = 0; j < iRoots; j++)
    {
      dx = (double)i + dRoots[j];
      if(!pBounds->HasPoint(dx))
      {
        pBounds->AddPoint(dx);
        iRes++;
      }
    }
  }

  return iRes;
}

bool GetValidInterRefs(double dt1, double dt2, bool bNeigbours)
{
  if(bNeigbours)
  {
    if((dt1 > 1.0 - g_dPrec) && (dt2 < g_dPrec)) return false;
  }
  return (dt1 > -g_dPrec) && (dt1 < 1.0 + g_dPrec) && (dt2 > -g_dPrec) && (dt2 < 1.0 + g_dPrec);
}

int GetSplineSnapPoints(CDPoint cPt, double dDist, PDPointList pCache, PDRefList pRefs)
{
  int iRes = 0;
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return 0;

  int nCtrl = pCache->GetCount(1);
  bool bClosed = (nCtrl > 0);

  double dr = 0.0;
  int nOffs = pCache->GetCount(2);
  if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  int iSegs = GetSplineNumSegments(pCache);
  double *pdDists = (double*)malloc(iSegs*sizeof(double));
  int iCand = 0;
  CDPrimitive cQuad;
  double dBreaks[4];
  int iBreaks;
  double dt;
  CDPoint cPt1, cDir1, cDir2;
  for(int i = 0; i < iSegs; i++)
  {
    cQuad = GetSplineNthSegment(i, pCache);
    pdDists[i] = GetQuadDistFromPoint(cPt, dr, cQuad, &dt);
    if(pdDists[i] < dDist) iCand++;
    iBreaks = GetQuadBreaks(cQuad, dr, 0.0, 1.0, dBreaks);
    for(int j = 0; j < iBreaks; j++)
    {
      cPt1 = GetQuadBufPoint(cQuad, dr, dBreaks[j]);
      if(GetDist(cPt, cPt1) < dDist)
      {
        pRefs->AddPoint((double)i + dBreaks[j]);
        iRes++;
      }
    }
    cDir1 = GetQuadBufDir(cQuad, dr, 0.0);
    if((i > 0) && (GetDist(cDir1 , cDir2) > g_dPrec))
    {
      cPt1 = GetQuadBufPoint(cQuad, dr, 0.0);
      if(GetDist(cPt, cPt1) < dDist)
      {
        pRefs->AddPoint((double)i);
        iRes++;
      }
    }
    cDir2 = GetQuadBufDir(cQuad, dr, 1.0);
  }
  if(bClosed)
  {
    cQuad = GetSplineNthSegment(0, pCache);
    cDir1 = GetQuadBufDir(cQuad, dr, 0.0);
    if(GetDist(cDir1 , cDir2) > g_dPrec)
    {
      cPt1 = GetQuadBufPoint(cQuad, dr, 0.0);
      if(GetDist(cPt, cPt1) < dDist)
      {
        pRefs->AddPoint(0.0);
        iRes++;
      }
    }
  }
  if(iCand > 1)
  {
    int iFirst = 0;
    while(pdDists[iFirst] > dDist) iFirst++;
    int iSecond = iFirst + 1;
    while(pdDists[iSecond] > dDist) iSecond++;
    cQuad = GetSplineNthSegment(iFirst, pCache);
    CDPrimitive cQuad2 = GetSplineNthSegment(iSecond, pCache);

    double dDist1 = GetQuadDistFromPoint(cPt, dr, cQuad, &dt);
    double dt2;
    double dDist2 = GetQuadDistFromPoint(cPt, dr, cQuad2, &dt2);

    cPt1 = GetQuadBufPoint(cQuad, dr, dt);
    CDPoint cPt2 = GetQuadBufPoint(cQuad2, dr, dt2);
    cDir1 = GetQuadBufDir(cQuad, dr, dt);
    cDir2 = GetQuadBufDir(cQuad2, dr, dt2);

    CDPoint cX;
    int iX = LineXLine(cPt1, cDir1, cPt2, cDir2, &cX);
    int iIter = 0;
    int iIterMax = 16;
    double dDist3 = GetDist(cPt, cX);
    double dIncl;
    bool bDoIter = (iX > 0) && (dDist3 < dDist) && ((dDist1 > g_dRootPrec) || (dDist2 > g_dRootPrec));
    while(bDoIter)
    {
      iIter++;
      dDist1 = GetQuadDistFromPoint(cX, dr, cQuad, &dt);
      dDist2 = GetQuadDistFromPoint(cX, dr, cQuad2, &dt2);
      cPt1 = GetQuadBufPoint(cQuad, dr, dt);
      cPt2 = GetQuadBufPoint(cQuad2, dr, dt2);
      cDir1 = GetQuadBufDir(cQuad, dr, dt);
      cDir2 = GetQuadBufDir(cQuad2, dr, dt2);
      iX = LineXLine(cPt1, cDir1, cPt2, cDir2, &cX);
      dDist3 = GetDist(cPt, cX);
      if(iIter < 4)
        bDoIter = (iX > 0) && (dDist3 < dDist) && ((dDist1 > g_dRootPrec) || (dDist2 > g_dRootPrec));
      else
      {
        dIncl = fabs(cDir1*cDir2);
        bDoIter = (iX > 0) && (iIter < iIterMax) && (dDist3 < dDist) &&
          ((dDist1 > g_dRootPrec) || (dDist2 > g_dRootPrec) ||
          ((dIncl > 1.0 - g_dPrec) && (dIncl < 1.0 - g_dRootPrec)));
      }
    }
    bool bFound;
    if(bClosed && (iFirst == 0) && (iSecond == iSegs - 1))
      bFound = (dDist1 < g_dRootPrec) && (dDist2 < g_dRootPrec) && GetValidInterRefs(dt2, dt, true);
    else
      bFound = (dDist1 < g_dRootPrec) && (dDist2 < g_dRootPrec) && GetValidInterRefs(dt, dt2, iSecond == iFirst + 1);
    if(bFound)
    {
      pRefs->AddPoint((double)iFirst + dt);
      iRes++;
      pRefs->AddPoint((double)iSecond + dt2);
      iRes++;
      for(int i = iSecond + 1; i < iSegs; i++)
      {
        if(pdDists[i] < dDist)
        {
          cQuad = GetSplineNthSegment(i, pCache);
          dDist1 = GetQuadDistFromPoint(cX, dr, cQuad, &dt);
          if(dDist < g_dRootPrec)
          {
            dt += (double)i;
            if(!pRefs->HasPoint(dt))
            {
              pRefs->AddPoint(dt);
              iRes++;
            }
          }
        }
      }
    }
  }
  free(pdDists);
  return iRes;
}

bool GetSplineBoundRefPoints(PDPointList pCache, PDRefList pBnds)
{
  int iCnt = pCache->GetCount(0);
  if(iCnt < 2) return 0;

  //int nCtrl = pCache->GetCount(1);
  //bool bClosed = (nCtrl > 0);

  //double dr = dOffset;
  //int nOffs = pCache->GetCount(2);
  //if(nOffs > 0) dr += pCache->GetPoint(0, 2).cPoint.x;

  int iSegs = GetSplineNumSegments(pCache);
  CDPrimitive cQuad;
  double dx;
  for(int i = 0; i < iSegs; i++)
  {
    cQuad = GetSplineNthSegment(i, pCache);
    dx = cQuad.cPt3.x - 2.0*cQuad.cPt2.x + cQuad.cPt1.x;
    if(fabs(dx) > g_dPrec)
    {
      dx = (cQuad.cPt1.x - cQuad.cPt2.x)/dx;
      if((dx > -g_dPrec) && (dx < 1.0 + g_dPrec))
      {
        dx += (double)i;
        if(!pBnds->HasPoint(dx)) pBnds->AddPoint(dx);
      }
    }
    dx = cQuad.cPt3.y - 2.0*cQuad.cPt2.y + cQuad.cPt1.y;
    if(fabs(dx) > g_dPrec)
    {
      dx = (cQuad.cPt1.y - cQuad.cPt2.y)/dx;
      if((dx > -g_dPrec) && (dx < 1.0 + g_dPrec))
      {
        dx += (double)i;
        if(!pBnds->HasPoint(dx)) pBnds->AddPoint(dx);
      }
    }
  }

  return pBnds->GetCount() > 0;
}

