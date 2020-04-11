#include "DDataTypes.hpp"
#include <malloc.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include "DMath.hpp"

// for debugging purpose only
/*#include <windows.h>
#include <commctrl.h>
#include <wchar.h>
extern HWND g_hStatus;*/
// -----

double Round(double x)
{
  return floor(x + 0.5);
}

CDPoint Round(CDPoint cp)
{
  CDPoint cRes;
  cRes.x = Round(cp.x);
  cRes.y = Round(cp.y);
  return cRes;
}

CDPoint Abs(CDPoint cp)
{
  CDPoint cRes;
  cRes.x = fabs(cp.x);
  cRes.y = fabs(cp.y);
  return cRes;
}

// cNorm.x = cos(phi), cNorm.y = sin(phi), where phi is the rotation angle
CDPoint Rotate(CDPoint cp, CDPoint cNorm, bool bInverse)
{
  CDPoint cRes;
  if(bInverse)
  {
    cRes.x = cp.x*cNorm.x - cp.y*cNorm.y;
    cRes.y = cp.y*cNorm.x + cp.x*cNorm.y;
  }
  else
  {
    cRes.x = cp.x*cNorm.x + cp.y*cNorm.y;
    cRes.y = cp.y*cNorm.x - cp.x*cNorm.y;
  }
  return cRes;
}

CDPoint Mirror(CDPoint cp, CDLine cLine)
{
  CDPoint cp1 = Rotate(cp - cLine.cOrigin, cLine.cDirection, false);
  cp1.y *= -1.0;
  return cLine.cOrigin + Rotate(cp1, cLine.cDirection, true);
}

double Deter2(CDPoint cp1, CDPoint cp2)
{
  return cp1.x*cp2.y - cp1.y*cp2.x;
}

CDPoint MatProd(CDPoint cMat1, CDPoint cMat2, CDPoint cb)
{
  return cb.x*cMat1 + cb.y*cMat2;
}

bool Solve2x2Matrix(CDPoint cCol1, CDPoint cCol2, CDPoint cB, PDPoint pSol)
{
  double dDet = cCol1.x*cCol2.y - cCol1.y*cCol2.x;

  if(fabs(dDet) < g_dPrec) return false;

  CDPoint cMatInv1, cMatInv2;

  cMatInv1.x = cCol2.y;
  cMatInv1.y = -cCol2.x;

  cMatInv2.x = -cCol1.y;
  cMatInv2.y = cCol1.x;

  pSol->x = cMatInv1*cB/dDet;
  pSol->y = cMatInv2*cB/dDet;

  return true;
}

bool Solve2x2MatrixT(CDPoint cRow1, CDPoint cRow2, CDPoint cB, PDPoint pSol)
{
  double dDet = cRow1.x*cRow2.y - cRow1.y*cRow2.x;

  if(fabs(dDet) < g_dPrec) return false;

  CDPoint cMatInv1, cMatInv2;

  cMatInv1.x = cRow2.y;
  cMatInv1.y = -cRow1.y;

  cMatInv2.x = -cRow2.x;
  cMatInv2.y = cRow1.x;

  pSol->x = cMatInv1*cB/dDet;
  pSol->y = cMatInv2*cB/dDet;

  return true;
}

void SwapPoints(PDPoint pPt1, PDPoint pPt2)
{
  CDPoint cPt3 = *pPt2;
  *pPt2 = *pPt1;
  *pPt1 = cPt3;
}

double GetLineProj(CDPoint cPt, CDPoint cOrig, CDPoint cDir, PDPoint pPtX)
{
  CDPoint cp1 = Rotate(cPt - cOrig, cDir, false);

  if(pPtX)
  {
    CDPoint cp2 = {cp1.x, 0};
    *pPtX = cOrig + Rotate(cp2, cDir, true);
  }

  return fabs(cp1.y);
}

bool Solve2ParamSystem(DFunc2D pFunc, DFunc2D pFuncDX, DFunc2D pFuncDY,
  CDPoint cPar1, CDPoint cPar2, CDPoint cInitSol, PDPoint pResult, PDPoint pLowBound)
{
  CDPoint cSol = cInitSol;

  CDPoint cMat1 = pFuncDX(cPar1, cPar2, cSol);
  CDPoint cMat2 = pFuncDY(cPar1, cPar2, cSol);
  CDPoint cB = pFunc(cPar1, cPar2, cSol);

  double dNSol = cB*cB;
  int iIter = 0;
  bool bHasSol = true;
  if(pLowBound) bHasSol = (cSol.x > pLowBound->x) && (cSol.y > pLowBound->x);

  CDPoint cSol2;

  while(bHasSol && (dNSol > g_dPrec) && (iIter < 8))
  {
    bHasSol = Solve2x2Matrix(cMat1, cMat2, cB, &cSol2);
    if(bHasSol)
    {
      cSol -= cSol2;
      if(pLowBound) bHasSol = (cSol.x > pLowBound->x) && (cSol.y > pLowBound->x);
      cMat1 = pFuncDX(cPar1, cPar2, cSol);
      cMat2 = pFuncDY(cPar1, cPar2, cSol);
      cB = pFunc(cPar1, cPar2, cSol);
      dNSol = cB*cB;
    }
    iIter++;
  }

  if(bHasSol) *pResult = cSol;

  return bHasSol;
}

double GetPtDistFromLineSeg(CDPoint cPt, CDPoint cLp1, CDPoint cLp2, PDLine pPtX)
{
  pPtX->bIsSet = true;

  CDPoint cDir = cLp2 - cLp1;
  double dNorm = GetNorm(cDir);
  if(dNorm < g_dPrec)
  {
    pPtX->cOrigin = cLp1;
    pPtX->cDirection = 0;
    pPtX->dRef = 0.0;
    return GetDist(cPt, cLp1);
  }

  cDir /= dNorm;
  CDPoint cPt1 = Rotate(cPt - cLp1, cDir, false);
  CDPoint cPt2 = {dNorm, 0.0};
  double dDir = 1.0;
  if(cPt1.y < 0) dDir = -1.0;

  if(cPt1.x < g_dPrec)
  {
    pPtX->cOrigin = cLp1;
    pPtX->cDirection = 0;
    pPtX->dRef = 0.0;
    return dDir*GetNorm(cPt1);
  }
  if(cPt1.x > dNorm - g_dPrec)
  {
    pPtX->cOrigin = cLp2;
    pPtX->cDirection = 0;
    pPtX->dRef = 1.0;
    return dDir*GetDist(cPt1, cPt2);
  }

  cPt2.x = cPt1.x;
  pPtX->cOrigin = cLp1 + Rotate(cPt2, cDir, true);
  pPtX->cDirection = GetNormal(cDir);
  pPtX->dRef = cPt1.x/dNorm;

  return cPt1.y;
}

bool DPtInDRect(CDPoint cPt, PDRect pRect)
{
  return (cPt.x > pRect->cPt1.x - g_dPrec) &&
    (cPt.x < pRect->cPt2.x + g_dPrec) &&
    (cPt.y > pRect->cPt1.y - g_dPrec) &&
    (cPt.y < pRect->cPt2.y + g_dPrec);
}

// CDIntList

CDIntList::CDIntList()
{
  m_iDataLen = 0;
  m_iDataSize = 16;
  m_pData = (int*)malloc(m_iDataSize*sizeof(int));
}

CDIntList::~CDIntList()
{
  free(m_pData);
}

void CDIntList::Clear()
{
  m_iDataLen = 0;
}

void CDIntList::AddItem(int iVal)
{
  if(m_iDataLen >= m_iDataSize)
  {
    m_iDataSize += 16;
    m_pData = (int*)realloc(m_pData, m_iDataSize*sizeof(int));
  }
  m_pData[m_iDataLen++] = iVal;
  return;
}

void CDIntList::InsertItem(int iPos, int iVal)
{
  if(m_iDataLen >= m_iDataSize)
  {
    m_iDataSize += 16;
    m_pData = (int*)realloc(m_pData, m_iDataSize*sizeof(int));
  }

  memmove(&m_pData[iPos + 1], &m_pData[iPos], (m_iDataLen - iPos)*sizeof(int));

  m_pData[iPos] = iVal;
  m_iDataLen++;
  return;
}

void CDIntList::SetItem(int iPos, int iVal)
{
  m_pData[iPos] = iVal;
}

int CDIntList::GetCount()
{
  return m_iDataLen;
}

int CDIntList::GetIndex(int iVal)
{
  bool bFound = false;
  int i = 0;
  while(!bFound && (i < m_iDataLen))
  {
    bFound = (iVal == m_pData[i++]);
  }
  return bFound ? i - 1 : -1;
}

int CDIntList::GetItem(int iIndex)
{
  return m_pData[iIndex];
}

void CDIntList::Remove(int iIndex)
{
  m_iDataLen--;
  if(iIndex < m_iDataLen)
  {
    memmove(&m_pData[iIndex], &m_pData[iIndex + 1],
      (m_iDataLen - iIndex)*sizeof(int));
  }
}

void CDIntList::RemoveItem(int iVal)
{
  int iPos = GetIndex(iVal);
  if(iPos > -1) Remove(iPos);
}


// CDRefList

CDRefList::CDRefList()
{
  m_iDataLen = 0;
  m_iDataSize = 16;
  m_pPoints = (double*)malloc(m_iDataSize*sizeof(double));
}

CDRefList::~CDRefList()
{
  free(m_pPoints);
}

void CDRefList::Clear()
{
  m_iDataLen = 0;
}

void CDRefList::AddPoint(double dVal)
{
  if(m_iDataLen >= m_iDataSize)
  {
    m_iDataSize += 16;
    m_pPoints = (double*)realloc(m_pPoints, m_iDataSize*sizeof(double));
  }
  m_pPoints[m_iDataLen++] = dVal;
  return;
}

void CDRefList::InsertPoint(int iPos, double dVal)
{
  if(m_iDataLen >= m_iDataSize)
  {
    m_iDataSize += 16;
    m_pPoints = (double*)realloc(m_pPoints, m_iDataSize*sizeof(double));
  }

  memmove(&m_pPoints[iPos + 1], &m_pPoints[iPos], (m_iDataLen - iPos)*sizeof(double));

  m_pPoints[iPos] = dVal;
  m_iDataLen++;
  return;
}

int CDRefList::GetCount()
{
  return m_iDataLen;
}

int CDRefList::GetIndex(double dVal)
{
  bool bFound = false;
  int i = 0;
  while(!bFound && (i < m_iDataLen))
  {
    bFound = (dVal < m_pPoints[i++] + 0.001);
  }
  return bFound ? i - 1 : -1;
}

double CDRefList::GetPoint(int iIndex)
{
  return m_pPoints[iIndex];
}

void CDRefList::SetPoint(int iIndex, double dVal)
{
  m_pPoints[iIndex] = dVal;
}

void CDRefList::Remove(int iIndex)
{
  m_iDataLen--;
  if(iIndex < m_iDataLen)
  {
    memmove(&m_pPoints[iIndex], &m_pPoints[iIndex + 1],
      (m_iDataLen - iIndex)*sizeof(double));
  }
}

void CDRefList::Truncate(int iNewLen)
{
  m_iDataLen = iNewLen;
}

bool CDRefList::HasPoint(double dVal)
{
  bool bRes = false;
  int i = 0;
  while(!bRes && (i < m_iDataLen))
  {
    bRes = fabs(dVal - m_pPoints[i++]) < g_dPrec;
  }
  return bRes;
}

void CDRefList::Sort(int iStartIndex)
{
  std::sort(&m_pPoints[iStartIndex], &m_pPoints[m_iDataLen]);
}


// CDPointList

CDPointList::CDPointList()
{
  m_iDataLen = 0;
  m_iDataSize = 16;
  m_pPoints = (PDInputPoint)malloc(m_iDataSize*sizeof(CDInputPoint));
}

CDPointList::~CDPointList()
{
  free(m_pPoints);
}

void CDPointList::ClearAll()
{
  m_iDataLen = 0;
}

void CDPointList::AddPoint(double x, double y, char iCtrl)
{
  if(m_iDataLen >= m_iDataSize)
  {
    m_iDataSize += 16;
    m_pPoints = (PDInputPoint)realloc(m_pPoints, m_iDataSize*sizeof(CDInputPoint));
  }
  m_pPoints[m_iDataLen].iCtrl = iCtrl;
  m_pPoints[m_iDataLen].cPoint.x = x;
  m_pPoints[m_iDataLen++].cPoint.y = y;
  return;
}

int CDPointList::GetCount(char iCtrl)
{
  if(iCtrl < 0) return m_iDataLen;
  int iRes = 0;
  for(int i = 0; i < m_iDataLen; i++)
  {
    if(m_pPoints[i].iCtrl == iCtrl) iRes++;
  }
  return iRes;
}

CDInputPoint CDPointList::GetPoint(int iIndex, char iCtrl)
{
  if(iCtrl< 0) return m_pPoints[iIndex];

  int i = 0;
  int iCtrls = -1;
  while((i < m_iDataLen) && (iCtrls < iIndex))
  {
    if(m_pPoints[i++].iCtrl == iCtrl) iCtrls++;
  }
  return m_pPoints[i - 1];
}

void CDPointList::RawRemove(int iIndex)
{
  m_iDataLen--;
  if(iIndex < m_iDataLen)
  {
    memmove(&m_pPoints[iIndex], &m_pPoints[iIndex + 1],
      (m_iDataLen - iIndex)*sizeof(CDInputPoint));
  }
}

void CDPointList::Remove(int iIndex, char iCtrl)
{
  if(iCtrl < 0)
  {
    RawRemove(iIndex);
    return;
  }

  int i = 0;
  int iCtrls = -1;
  while((i < m_iDataLen) && (iCtrls < iIndex))
  {
    if(m_pPoints[i++].iCtrl == iCtrl) iCtrls++;
  }
  if(iCtrls == iIndex) RawRemove(i - 1);
}

void CDPointList::SetPoint(int iIndex, char iCtrl, double x, double y, char iNewCtrl)
{
  int iNewIdex = iIndex;
  if(iCtrl > -1)
  {
    int i = 0;
    int iCtrls = -1;
    while((i < m_iDataLen) && (iCtrls < iIndex))
    {
      if(m_pPoints[i++].iCtrl == iCtrl) iCtrls++;
    }
    if(iCtrls == iIndex) iNewIdex = i - 1;
  }

  m_pPoints[iNewIdex].cPoint.x = x;
  m_pPoints[iNewIdex].cPoint.y = y;
  m_pPoints[iNewIdex].iCtrl = iNewCtrl;
}


// CDPtrList

CDPtrList::CDPtrList()
{
  m_iDataLen = 0;
  m_iDataSize = 16;
  m_pData = (void**)malloc(m_iDataSize*sizeof(void*));
  m_dValue = 0.0;
}

CDPtrList::~CDPtrList()
{
  free(m_pData);
}

int CDPtrList::GetCount()
{
  return m_iDataLen;
}

void CDPtrList::Clear()
{
  m_iDataLen = 0;
}

void CDPtrList::Add(void *pPtr)
{
  if(m_iDataLen >= m_iDataSize)
  {
    m_iDataSize += 16;
    m_pData = (void**)realloc(m_pData, m_iDataSize*sizeof(void*));
  }
  m_pData[m_iDataLen++] = pPtr;
  return;
}

void* CDPtrList::GetItem(int iIndex)
{
  return m_pData[iIndex];
}

void CDPtrList::Remove(int iIndex)
{
  m_iDataLen--;
  if(iIndex < m_iDataLen)
  {
    memmove(&m_pData[iIndex], &m_pData[iIndex + 1],
      (m_iDataLen - iIndex)*sizeof(void*));
  }
}

double CDPtrList::GetDblVal()
{
  return m_dValue;
}

void CDPtrList::SetDblVal(double dVal)
{
  m_dValue = dVal;
}


// CDPrimObject

CDPrimObject::CDPrimObject()
{
  m_iDataLen = 0;
  m_iDataSize = 16;
  m_pData = (PDPrimitive)malloc(m_iDataSize*sizeof(CDPrimitive));
}

CDPrimObject::~CDPrimObject()
{
  free(m_pData);
}

int CDPrimObject::GetCount()
{
  return m_iDataLen;
}

void CDPrimObject::AddPrimitive(CDPrimitive cPrim)
{
  if(m_iDataLen >= m_iDataSize)
  {
    m_iDataSize += 16;
    m_pData = (PDPrimitive)realloc(m_pData, m_iDataSize*sizeof(CDPrimitive));
  }
  m_pData[m_iDataLen++] = cPrim;
}

void CDPrimObject::InsertPrimitive(int iIndex, CDPrimitive cPrim)
{
  if(m_iDataLen >= m_iDataSize)
  {
    m_iDataSize += 16;
    m_pData = (PDPrimitive)realloc(m_pData, m_iDataSize*sizeof(CDPrimitive));
  }
  int iCnt = m_iDataLen++ - iIndex;
  if(iCnt > 0) memmove(&m_pData[iIndex + 1], &m_pData[iIndex], iCnt*sizeof(CDPrimitive));
  m_pData[iIndex] = cPrim;
}

CDPrimitive CDPrimObject::GetPrimitive(int iIndex)
{
  return m_pData[iIndex];
}

void CDPrimObject::Clear()
{
  m_iDataLen = 0;
}

void CDPrimObject::Delete(int iIndex)
{
  m_iDataLen--;
  if(iIndex < m_iDataLen)
  {
    memmove(&m_pData[iIndex], &m_pData[iIndex + 1],
      (m_iDataLen - iIndex)*sizeof(CDPrimitive));
  }
}

void CDPrimObject::ClearLines()
{
  int i = m_iDataLen;
  while(i > 0)
  {
    if(m_pData[--i].iType < 6) Delete(i);
  }
}

void CDPrimObject::CopyFrom(CDPrimObject *pPrimObj)
{
  for(int i = 0; i < pPrimObj->GetCount(); i++)
  {
    AddPrimitive(pPrimObj->GetPrimitive(i));
  }
}


void CopyDimenAttrs(PDDimension pDimDst, PDDimension pDimSrc)
{
  pDimDst->iArrowType1 = pDimSrc->iArrowType1;
  pDimDst->iArrowType2 = pDimSrc->iArrowType2;
  pDimDst->cArrowDim1 = pDimSrc->cArrowDim1;
  pDimDst->cArrowDim2 = pDimSrc->cArrowDim2;
  pDimDst->dFontSize = pDimSrc->dFontSize;
  pDimDst->bFontAttrs = pDimSrc->bFontAttrs;
  strcpy(pDimDst->psFontFace, pDimSrc->psFontFace);
  if(pDimDst->psLab) free(pDimDst->psLab);
  pDimDst->psLab = NULL;
  if(pDimSrc->psLab)
  {
    int iLen = strlen(pDimSrc->psLab);
    pDimDst->psLab = (char*)malloc((iLen + 1)*sizeof(char));
    strcpy(pDimDst->psLab, pDimSrc->psLab);
  }
}

void ClearPolygonList(PDPtrList pPolygons)
{
  PDPolygon pPoly;
  for(int i = 0; i < pPolygons->GetCount(); i++)
  {
    pPoly = (PDPolygon)pPolygons->GetItem(i);
    free(pPoly->pPoints);
    free(pPoly);
  }
}

int IntersectBounds(CDPoint cBnds1, CDPoint cBnds2, int iBndMode, double dLength, PDPoint pRes)
{
  bool b1Closed = false;
  bool b2Closed = false;

  if(iBndMode & 8)
  {
    b1Closed = true;
    if((iBndMode & 2) && (iBndMode & 4)) b1Closed = fabs(cBnds1.y - cBnds1.x - dLength) < g_dPrec;
    b2Closed = fabs(cBnds2.y - cBnds2.x - dLength) < g_dPrec;
  }

  if(b1Closed)
  {
    if(iBndMode & 2)
    {
      if(b2Closed)
      {
        pRes[0].x = cBnds1.x;
        pRes[0].y = cBnds1.x + dLength;
      }
      else pRes[0] = cBnds2;
      return 1;
    }
    pRes[0] = cBnds2;
    return 1;
  }

  if(b2Closed)
  {
    if((iBndMode & 2) && (iBndMode & 4))
    {
      pRes[0] = cBnds1;
      return 1;
    }
    if(iBndMode & 2)
    {
      pRes[0].x = cBnds1.x;
      pRes[0].y = cBnds1.x + cBnds1.y;
      return 1;
    }
    pRes[0] = cBnds2;
    return 1;
  }

  if((iBndMode & 2) && (iBndMode & 4))
  {
    if((cBnds1.x < cBnds1.y) && (cBnds2.x < cBnds2.y))
    {
      if(cBnds1.x > cBnds2.y - g_dPrec) return 0;
      if(cBnds2.x > cBnds1.y - g_dPrec) return 0;
      pRes[0] = cBnds1;
      if(cBnds1.x < cBnds2.x - g_dPrec) pRes[0].x = cBnds2.x;
      if(cBnds1.y > cBnds2.y + g_dPrec) pRes[0].y = cBnds2.y;
      return 1;
    }
    if(cBnds1.x < cBnds1.y)
    {
      if(cBnds1.x < cBnds2.x)
      {
        if(cBnds1.x < cBnds2.y)
        {
          if(cBnds1.y > cBnds2.x)
          {
            pRes[0].x = cBnds1.x;
            pRes[0].y = cBnds2.y;
            pRes[1].x = cBnds2.x;
            pRes[1].y = cBnds1.y;
            return 2;
          }
          if(cBnds1.y < cBnds2.y)
          {
            pRes[0] = cBnds1;
            return 1;
          }
          pRes[0].x = cBnds1.x;
          pRes[0].y = cBnds2.y;
          return 1;
        }
        if(cBnds1.y > cBnds2.x)
        {
          pRes[0].x = cBnds2.x;
          pRes[0].y = cBnds1.y;
          return 1;
        }
        return 0;
      }
      pRes[0] = cBnds1;
      return 1;
    }
    if(cBnds2.x < cBnds2.y)
    {
      if(cBnds2.x < cBnds1.x)
      {
        if(cBnds2.x < cBnds1.y)
        {
          if(cBnds2.y > cBnds1.x)
          {
            pRes[0].x = cBnds2.x;
            pRes[0].y = cBnds1.y;
            pRes[1].x = cBnds1.x;
            pRes[1].y = cBnds2.y;
            return 2;
          }
          if(cBnds2.y < cBnds1.y)
          {
            pRes[0] = cBnds2;
            return 1;
          }
          pRes[0].x = cBnds2.x;
          pRes[0].y = cBnds1.y;
          return 1;
        }
        if(cBnds2.y > cBnds1.x)
        {
          pRes[0].x = cBnds1.x;
          pRes[0].y = cBnds2.y;
          return 1;
        }
        return 0;
      }
      pRes[0] = cBnds2;
      return 1;
    }
    if(cBnds1.x > cBnds2.x)
    {
      if(cBnds1.y < cBnds2.y)
      {
        pRes[0] = cBnds1;
        return 1;
      }
      pRes[0].x = cBnds1.x;
      pRes[0].y = cBnds2.y;
      return 1;
    }
    if(cBnds1.y > cBnds2.y)
    {
      pRes[0] = cBnds2;
      return 1;
    }
    pRes[0].x = cBnds2.x;
    pRes[0].y = cBnds1.y;
    return 1;
  }

  if(iBndMode & 2)
  {
    pRes[0] = cBnds2;
    if(cBnds1.x > cBnds2.x + g_dPrec) pRes[0].x = cBnds1.x;
    return 1;
  }

  if(iBndMode & 4)
  {
    pRes[0] = cBnds2;
    if(cBnds1.y < cBnds2.y - g_dPrec) pRes[0].y = cBnds1.y;
    return 1;
  }

  pRes[0] = cBnds2;
  return 1;
}

int UnionBoundPts(CDPoint cBnds1, CDPoint cBnds2, double dLength, PDPoint pRes)
{
  bool b1Closed = false;
  bool b2Closed = false;

  if(dLength > g_dPrec)
  {
    b1Closed = fabs(cBnds1.y - cBnds1.x - dLength) < g_dPrec;
    b2Closed = fabs(cBnds2.y - cBnds2.x - dLength) < g_dPrec;
  }

  if(b1Closed)
  {
    pRes[0] = cBnds1;
    return 1;
  }

  if(b2Closed)
  {
    pRes[0] = cBnds2;
    return 1;
  }

  if((cBnds1.x > cBnds1.y) && (cBnds2.x > cBnds2.y))
  {
    pRes[0] = cBnds1;
    if(cBnds2.x < cBnds1.x) pRes[0].x = cBnds2.x;
    if(cBnds2.y > cBnds1.y) pRes[0].y = cBnds2.y;
    return 1;
  }

  if(cBnds1.x > cBnds1.y)
  {
    if((cBnds2.x > cBnds1.x) || (cBnds2.y < cBnds1.y))
    {
      pRes[0] = cBnds1;
      return 1;
    }
    if(cBnds2.x > cBnds1.y)
    {
      if(cBnds2.y < cBnds1.x)
      {
        pRes[0] = cBnds1;
        pRes[1] = cBnds2;
        return 2;
      }
      pRes[0].x = cBnds2.x;
      pRes[0].y = cBnds1.y;
      return 1;
    }
    if(cBnds2.y < cBnds1.x)
    {
      pRes[0].x = cBnds1.x;
      pRes[0].y = cBnds2.y;
      return 1;
    }
    // the next should not happen, but just for sure:
    pRes[0].x = cBnds1.x;
    pRes[0].y = cBnds1.x + dLength;
    return 1;
  }

  if(cBnds2.x > cBnds2.y)
  {
    if((cBnds1.x > cBnds2.x) || (cBnds1.y < cBnds2.y))
    {
      pRes[0] = cBnds2;
      return 1;
    }
    if(cBnds1.x > cBnds2.y)
    {
      if(cBnds1.y < cBnds2.x)
      {
        pRes[0] = cBnds1;
        pRes[1] = cBnds2;
        return 2;
      }
      pRes[0].x = cBnds1.x;
      pRes[0].y = cBnds2.y;
      return 1;
    }
    if(cBnds1.y < cBnds2.x)
    {
      pRes[0].x = cBnds2.x;
      pRes[0].y = cBnds1.y;
      return 1;
    }
    // the next should not happen, but just for sure:
    pRes[0].x = cBnds1.x;
    pRes[0].y = cBnds1.x + dLength;
    return 1;
  }

  if((cBnds2.x > cBnds1.y) || (cBnds1.x > cBnds2.y))
  {
    pRes[0] = cBnds1;
    pRes[1] = cBnds2;
    return 2;
  }

  pRes[0] = cBnds1;
  if(cBnds2.x < cBnds1.x) pRes[0].x = cBnds2.x;
  if(cBnds2.y > cBnds1.y) pRes[0].y = cBnds2.y;
  return 1;
}

int UnionBounds(PDRefList pBnds1, PDRefList pBnds2, double dLen, PDRefList pBndsRes)
{
  int iInt1 = pBnds1->GetCount();
  int iInt2 = pBnds2->GetCount();
  if((iInt1 < 1) && (iInt2 < 1)) return 0;

  if(iInt1 < 1)
  {
    for(int i = 0; i < iInt2; i++) pBndsRes->AddPoint((*pBnds2)[i]);
  }
  else if(iInt2 < 1)
  {
    for(int i = 0; i < iInt1; i++) pBndsRes->AddPoint((*pBnds1)[i]);
  }
  else
  {
    iInt1 /= 2;
    iInt2 /= 2;

    CDPoint cPt1, cPt2;
    CDPoint cUni1[2];
    bool bFound;
    for(int j = iInt2 - 1; j >= 0; j--)
    {
      cPt2.x = (*pBnds2)[2*j];
      cPt2.y = (*pBnds2)[2*j + 1];

      bFound = false;
      int i = 0;
      while(!bFound && (i < iInt1))
      {
        cPt1.x = (*pBnds1)[2*i];
        cPt1.y = (*pBnds1)[2*i + 1];
        bFound = (UnionBoundPts(cPt1, cPt2, dLen, cUni1) == 1);
        i++;
      }
      if(bFound)
      {
        iInt2--;
        if(j < iInt2)
        {
          pBnds2->Remove(2*j);
          pBnds2->Remove(2*j);
        }

        cPt1 = cUni1[0];
        pBnds1->SetPoint(2*(i - 1), cPt1.x);
        pBnds1->SetPoint(2*(i - 1) + 1, cPt1.y);
        for(int k = iInt1 - 1; k >= i; k--)
        {
          cPt2.x = (*pBnds1)[2*k];
          cPt2.y = (*pBnds1)[2*k + 1];
          if(UnionBoundPts(cPt1, cPt2, dLen, cUni1) == 1)
          {
            pBnds1->SetPoint(2*(i - 1), (*pBnds1)[2*k]);
            pBnds1->SetPoint(2*(i - 1) + 1, (*pBnds1)[2*k + 1]);
            iInt1--;
            if(k < iInt1)
            {
              pBnds1->Remove(2*k);
              pBnds1->Remove(2*k);
            }
          }
        }
      }
    }
    if((iInt1 > 0) && ((*pBnds1)[0] > (*pBnds1)[1]))
    {
      for(int i = 0; i < iInt1; i++)
      {
        pBndsRes->AddPoint((*pBnds1)[2*i]);
        pBndsRes->AddPoint((*pBnds1)[2*i + 1]);
      }
      for(int i = 0; i < iInt2; i++)
      {
        pBndsRes->AddPoint((*pBnds2)[2*i]);
        pBndsRes->AddPoint((*pBnds2)[2*i + 1]);
      }
      pBndsRes->Sort(1);
    }
    else if((iInt2 > 0) && ((*pBnds2)[0] > (*pBnds2)[1]))
    {
      for(int i = 0; i < iInt2; i++)
      {
        pBndsRes->AddPoint((*pBnds2)[2*i]);
        pBndsRes->AddPoint((*pBnds2)[2*i + 1]);
      }
      for(int i = 0; i < iInt1; i++)
      {
        pBndsRes->AddPoint((*pBnds1)[2*i]);
        pBndsRes->AddPoint((*pBnds1)[2*i + 1]);
      }
      pBndsRes->Sort(1);
    }
    else
    {
      for(int i = 0; i < iInt1; i++)
      {
        pBndsRes->AddPoint((*pBnds1)[2*i]);
        pBndsRes->AddPoint((*pBnds1)[2*i + 1]);
      }
      for(int i = 0; i < iInt2; i++)
      {
        pBndsRes->AddPoint((*pBnds2)[2*i]);
        pBndsRes->AddPoint((*pBnds2)[2*i + 1]);
      }
      pBndsRes->Sort(0);
    }
  }
  return 1;
}

void MergeCornerRef(double dRef, PDRefList pBnds, PDPoint pRefBnds)
{
  int n = pBnds->GetCount();
  if(n < 2) return;

  double d1, d2;

  if(pRefBnds)
  {
    if((*pBnds)[0] > (*pBnds)[1])
    {
      if(dRef < (*pBnds)[1] + g_dPrec) return;
      if(dRef > (*pBnds)[0] - g_dPrec) return;
    }
    else
    {
      if(dRef < (*pBnds)[0] + g_dPrec)
      {
        d1 = (*pBnds)[0] - dRef;
        d2 = (pRefBnds->y - (*pBnds)[n - 1]) + (dRef - pRefBnds->x);
        if(d1 < d2) pBnds->SetPoint(0, dRef);
        else pBnds->SetPoint(n - 1, dRef);
        return;
      }
      if(dRef > (*pBnds)[n - 1] - g_dPrec)
      {
        d1 = dRef - (*pBnds)[n - 1];
        d2 = (pRefBnds->y - dRef) + ((*pBnds)[0] - pRefBnds->x);
        if(d1 < d2) pBnds->SetPoint(n - 1, dRef);
        else pBnds->SetPoint(0, dRef);
        return;
      }
    }
  }

  int i = 0;
  if((*pBnds)[0] > (*pBnds)[1]) i = 1;
  bool bFound = dRef < (*pBnds)[i++] + g_dPrec;
  while(!bFound && (i < n))
  {
    bFound = dRef < (*pBnds)[i++] + g_dPrec;
  }

  if(!bFound && ((*pBnds)[0] > (*pBnds)[1]))
  {
    i = 0;
    bFound = dRef < (*pBnds)[i++] + g_dPrec;
    if(bFound)
    {
      d1 = dRef - (*pBnds)[n - 1];
      d2 = (*pBnds)[0] - dRef;
      if(d1 < d2) pBnds->SetPoint(n - 1, dRef);
      else pBnds->SetPoint(0, dRef);
      return;
    }
  }

  if(!bFound && pRefBnds) return; // something strange, it should not happen

  if(!pRefBnds)
  {
    if(!bFound)
    {
      pBnds->SetPoint(n - 1, dRef);
      return;
    }
    if(i < 2)
    {
      pBnds->SetPoint(0, dRef);
      return;
    }
  }

  if(i < 2) return;
  if(i % 2 == 0) return;

  d1 = dRef - pBnds->GetPoint(i - 2);
  d2 = pBnds->GetPoint(i - 1) - dRef;
  if(d1 < d2) pBnds->SetPoint(i - 2, dRef);
  else pBnds->SetPoint(i - 1, dRef);
}

void MergeCornerRefs(PDRefList pBnds, PDPoint pRefBnds, int iRectFlag, double *pdRefs)
{
  if(iRectFlag & 1) MergeCornerRef(pdRefs[0], pBnds, pRefBnds);
  if(iRectFlag & 2) MergeCornerRef(pdRefs[1], pBnds, pRefBnds);
  if(iRectFlag & 4) MergeCornerRef(pdRefs[2], pBnds, pRefBnds);
  if(iRectFlag & 8) MergeCornerRef(pdRefs[3], pBnds, pRefBnds);
}

