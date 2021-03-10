#include "DDrawTypes.hpp"
#include "DMath.hpp"
#include <math.h>
#include "DLine.hpp"
#include "DCircle.hpp"
#include "DEllipse.hpp"
#include "DPrimitive.hpp"
#include "DArcElps.hpp"
#include "DHyper.hpp"
#include "DParabola.hpp"
#include "DSpline.hpp"
#include "DEvolv.hpp"
#include <malloc.h>
#include <string.h>
#include <algorithm>

// for debugging purpose only
/*#include <windows.h>
#include <commctrl.h>
#include <wchar.h>
extern HWND g_hStatus;*/
// -----


void SwapBytes(unsigned char *pDest, unsigned char *pSrc, int iLen, bool bSwap)
{
  if(bSwap)
  {
    for(int i = 0; i < iLen; i++) pDest[i] = pSrc[iLen - i - 1];
  }
  else for(int i = 0; i < iLen; i++) pDest[i] = pSrc[i];
}

int CmpDbls(double d1, double d2)
{
  if(d1 < d2 - g_dPrec) return 1;
  if(d2 < d1 - g_dPrec) return -1;
  return 0;
}

int GetTangSnap(CDPoint cPt, double dDist, bool bNewPt, PDLine pSnapPt, PDObject pObj, PDObject pDynObj)
{
  CDLine cPtX1;
  cPtX1.dRef = 0.0;
  pObj->GetDistFromPt(cPt, cPt, 0, &cPtX1, NULL);
  if(!cPtX1.bIsSet) return 0;

  CDLine cRad1, cRad2;
  double d1 = pObj->GetRadiusAtPt(cPtX1, &cRad1, false);
  double d2 = pDynObj->GetRadiusAtPt(cPtX1, &cRad2, bNewPt);
  //int iType1 = pObj->GetType();
  int iType2 = pDynObj->GetType();
  CDInputPoint cInPt1;
  double d3;

  double dcos = cRad1.cDirection*cRad2.cDirection;
  if(fabs(dcos) < 0.1)
  {
    if(iType2 == 1)
    {
      if(!pDynObj->GetPoint(0, 0, &cInPt1)) return 0;
      pObj->GetDistFromPt(cInPt1.cPoint, cPt, 0, &cPtX1, NULL);
      if(cPtX1.bIsSet)
      {
        d3 = GetDist(cPtX1.cOrigin, cPt);
        if(fabs(d3) < dDist)
        {
          *pSnapPt = cPtX1;
          return 1;
        }
      }
    }
    return 0;
  }

  double dsin = Deter2(cRad1.cDirection, cRad2.cDirection);
  if(fabs(dsin) < 0.1)
  {
    bool bFound = fabs(dsin) < g_dPrec;
    int i = 0;
    int iIterMax = 16;
    CDPoint cPt1, cPt2;
    double dDir;

    if(cRad1.bIsSet)
    {
      cPt2 = cPt;
      while(cRad1.bIsSet && !bFound && (i < iIterMax))
      {
        if(cRad2.bIsSet) pObj->GetDistFromPt(cRad2.cOrigin, cPt2, 0, &cPtX1, NULL);
        else
        {
          dDir = 1.0;
          cPt1 = cPt2 - cRad1.cOrigin;
          dsin = GetNorm(cPt1);
          if(dsin > g_dPrec)
          {
            if(cPt1*cRad2.cDirection/dsin < -0.5) dDir = -1.0;
          }
          cPt1 = cRad1.cOrigin + dDir*d1*cRad2.cDirection;
          pObj->GetDistFromPt(cPt1, cPt1, 0, &cPtX1, NULL);
        }

        i++;
        d1 = pObj->GetRadiusAtPt(cPtX1, &cRad1, false);
        d2 = pDynObj->GetRadiusAtPt(cPtX1, &cRad2, bNewPt);
        dsin = Deter2(cRad1.cDirection, cRad2.cDirection);
        bFound = (fabs(dsin) < g_dPrec);
        cPt2 = cPtX1.cOrigin;
      }

      if(bFound && (GetDist(cPt, cPtX1.cOrigin) < dDist))
      {
        *pSnapPt = cPtX1;
        return 1;
      }
    }
    else if(cRad2.bIsSet)
    {
      cPt2 = cPt;
      while(cRad2.bIsSet && !bFound && (i < iIterMax))
      {
        dDir = 1.0;
        if((cPt2 - cRad2.cOrigin)*cRad1.cDirection < -0.5) dDir = -1.0;
        cPt1 = cRad2.cOrigin + dDir*d2*cRad1.cDirection;
        pObj->GetDistFromPt(cPt1, cPt1, 0, &cPtX1, NULL);

        i++;
        d1 = pObj->GetRadiusAtPt(cPtX1, &cRad1, false);
        d2 = pDynObj->GetRadiusAtPt(cPtX1, &cRad2, bNewPt);
        dsin = Deter2(cRad1.cDirection, cRad2.cDirection);
        bFound = (fabs(dsin) < g_dPrec);
        cPt2 = cPtX1.cOrigin;
      }

      if(bFound && (GetDist(cPt, cPtX1.cOrigin) < dDist))
      {
        *pSnapPt = cPtX1;
        return 1;
      }
    }
  }

  return 0;
}

int GetSnapPointFromList(int iSnapMask, CDPoint cPt, double dDist, PDLine pSnapPt, PDObject pDynObj, 
  PDObject *pObjList, int iCount, bool bHonorSnapTo)
{
  pSnapPt->bIsSet = false;

  int i = 0;
  CDLine cPtSnap1;
  double dDist1, dblDist;
  PDObject pObj1 = NULL;
  dblDist = 2.0*dDist;
  int iRes1 = 0;

  while((iRes1 < 1) && (i < iCount))
  {
    pObj1 = pObjList[i++];
    iRes1 = pObj1->GetSnapPoint(iSnapMask, cPt, dDist, &cPtSnap1, pDynObj, bHonorSnapTo);
  }
  if(iRes1 > 1)
  {
    *pSnapPt = cPtSnap1;
    return iRes1;
  }

  double dDist2;
  CDLine cPtSnap2;
  PDObject pObj2 = NULL;
  int iRes2 = 0;

  while((iRes2 < 1) && (i < iCount))
  {
    pObj2 = pObjList[i++];
    iRes2 = pObj2->GetSnapPoint(iSnapMask, cPt, dDist, &cPtSnap2, pDynObj, bHonorSnapTo);
  }

  if((iRes1 < 1) && (iRes2 < 1)) return 0;

  if(iRes2 > 1)
  {
    *pSnapPt = cPtSnap2;
    return iRes2;
  }

  *pSnapPt = cPtSnap1;

  if(iRes2 < 1)
  {
    if(pDynObj)
    {
      int iRes = GetTangSnap(cPt, dblDist, iSnapMask != 2, pSnapPt, pObj1, pDynObj);
      if(iRes > 0) return iRes;
    }
    if(iRes1 > 0) return iRes1;
  }
  else if(iRes1 < 1)
  {
    *pSnapPt = cPtSnap2;
    return iRes2;
  }

  double df1, df2;
  df1 = GetNorm(cPtSnap1.cDirection);
  df2 = GetNorm(cPtSnap2.cDirection);

  if(df1 < g_dPrec) return 5;
  if(df2 < g_dPrec)
  {
    *pSnapPt = cPtSnap2;
    return 5;
  }

  CDPoint cX, cDir = {0, 0};
  int iX = LineXLine(cPtSnap1.cOrigin, GetNormal(cPtSnap1.cDirection),
    cPtSnap2.cOrigin, GetNormal(cPtSnap2.cDirection), &cX);
  if(iX < 1) return 1;

  int iIterMax = 16;
  int iIter = 0;
  double dDist3 = GetDist(cPt, cX);
  dDist1 = fabs(pObj1->GetDistFromPt(cX, cX, 1, &cPtSnap1, NULL));
  dDist2 = fabs(pObj2->GetDistFromPt(cX, cX, 1, &cPtSnap2, NULL));
  bool bFound2 = dDist3 < dblDist;

  df1 = GetNorm(cPtSnap1.cDirection);
  df2 = GetNorm(cPtSnap2.cDirection);
  double dIncl;

  if(df1 < g_dPrec) return 5;
  if(df2 < g_dPrec)
  {
    *pSnapPt = cPtSnap2;
    return 5;
  }

  bool bDoIter = bFound2;
  while(bDoIter)
  {
    iIter++;
    dIncl = 0.0;
    iX = LineXLine(cPtSnap1.cOrigin, GetNormal(cPtSnap1.cDirection),
      cPtSnap2.cOrigin, GetNormal(cPtSnap2.cDirection), &cX);
    if(iX > 0)
    {
      dDist3 = GetDist(cPt, cX);
      dDist1 = fabs(pObj1->GetDistFromPt(cX, cX, 1, &cPtSnap1, NULL));
      dDist2 = fabs(pObj2->GetDistFromPt(cX, cX, 1, &cPtSnap2, NULL));
      bFound2 = dDist3 < dblDist;

      df1 = GetNorm(cPtSnap1.cDirection);
      df2 = GetNorm(cPtSnap2.cDirection);
      if(df1 < g_dPrec)
      {
        cX = cPtSnap1.cOrigin;
        cDir = cPtSnap1.cDirection;
        iIter = iIterMax;
      }
      else if(df2 < g_dPrec)
      {
        cX = cPtSnap2.cOrigin;
        cDir = cPtSnap2.cDirection;
        iIter = iIterMax;
      }
      else dIncl = fabs(cPtSnap1.cDirection*cPtSnap2.cDirection);
    }
    else bFound2 = false;

    bDoIter = bFound2 && (iIter < iIterMax) &&
      ((dDist1 > g_dRootPrec) || (dDist2 > g_dRootPrec) || (iIter < 4) ||
      ((dIncl > 1.0 - g_dPrec) && (dIncl < 1.0 - g_dRootPrec)));
    /*if(!bDoIter && bFound2 && (iIter < iIterMax))
    {
      // do one more iteration
      iIter = iIterMax - 1;
      bDoIter = true;
    }*/
  }

  if(bFound2)
  {
    pSnapPt->bIsSet = true;
    pSnapPt->cOrigin = cX;
    pSnapPt->cDirection = 0;
    return 4;
  }

  return 0;
}


// CDObject

CDObject::CDObject(CDDrawType iType, double dWidth)
{
  m_iType = iType;
  m_pInputPoints = new CDPointList();
  m_pUndoPoints = new CDPointList();
  m_pCachePoints = new CDPointList();
  m_pCrossPoints = new CDRefList();
  m_pPrimitive = new CDPrimObject();
  m_pDimens = new CDPtrList();
  m_iPrimitiveRequested = 0;
  m_bSelected = false;
  m_cLines[0].bIsSet = false;
  m_cLines[1].bIsSet = false;
  m_cBounds[0].bIsSet = false;
  m_cBounds[1].bIsSet = false;
  m_cLineStyle.dWidth = dWidth;
  m_cLineStyle.dPercent = 0.0;
  m_cLineStyle.cCapType = 1;
  m_cLineStyle.cJoinType = 1;
  m_cLineStyle.iSegments = 0;
  for(int i = 0; i < 6; i++) m_cLineStyle.dPattern[i] = 0.0;
  m_cLineStyle.cColor[0] = 0;
  m_cLineStyle.cColor[1] = 0;
  m_cLineStyle.cColor[2] = 0;
  m_cLineStyle.cColor[3] = 255;
  m_cLineStyle.dBlend = 0.0;
  m_dMovedDist = 0.0;
  m_bFirstDimSet = false;
  m_iDimenDir = 0;
  m_cTmpDim.psLab = m_sTmpDimBuf;
  m_sTmpDimBuf[0] = 0;
  m_bSnapTo = true;
  m_pSubObjects = new CDPtrList();
}

CDObject::~CDObject()
{
  if(m_iType < dtBorder)
  {
    PDPathSeg pSubObj;
    for(int i = 0; i < m_pSubObjects->GetCount(); i++)
    {
      pSubObj = (PDPathSeg)m_pSubObjects->GetItem(i);
      if(m_iType != dtBorderPath)
      {
        delete pSubObj->pSegment;
      }
      free(pSubObj);
    }
  }
  else
  {
    PDObject pSubObj;
    for(int i = 0; i < m_pSubObjects->GetCount(); i++)
    {
      pSubObj = (PDObject)m_pSubObjects->GetItem(i);
      delete pSubObj;
    }
  }
  delete m_pSubObjects;

  PDDimension pDim;
  for(int i = 0; i < m_pDimens->GetCount(); i++)
  {
    pDim = (PDDimension)m_pDimens->GetItem(i);
    if(pDim->psLab) free(pDim->psLab);
    free(pDim);
  }
  delete m_pDimens;

  delete m_pPrimitive;
  delete m_pCrossPoints;
  delete m_pCachePoints;
  delete m_pUndoPoints;
  delete m_pInputPoints;
}

bool CDObject::AddPoint(double x, double y, char iCtrl, double dRestrictVal)
{
  // we have to maintain this ugly code for backward compatibility
  if(m_iType > dtCircle)
  {
    int nOffs2 = m_pInputPoints->GetCount(2);
    int nOffs3 = m_pInputPoints->GetCount(3);
    int nOffs4 = m_pInputPoints->GetCount(4);

    if((iCtrl == 2) || (iCtrl == 3) || (iCtrl == 4))
    {
      CDPoint cNewPt = {x, y};
      if(iCtrl == 4)
      {
        cNewPt.x = dRestrictVal;
        cNewPt.y = 0.0;
      }

      if(nOffs2 > 0) m_pInputPoints->SetPoint(0, 2, cNewPt.x, cNewPt.y, iCtrl);
      else if(nOffs3 > 0) m_pInputPoints->SetPoint(0, 3, cNewPt.x, cNewPt.y, iCtrl);
      else if(nOffs4 > 0) m_pInputPoints->SetPoint(0, 4, cNewPt.x, cNewPt.y, iCtrl);
      else m_pInputPoints->AddPoint(cNewPt.x, cNewPt.y, iCtrl);
      return true;
    }
  }

  bool bRes = false;
  int iInputLines = 0;
  if(m_cLines[0].bIsSet) iInputLines++;
  if(m_cLines[1].bIsSet) iInputLines++;

  switch(m_iType)
  {
  case dtLine:
    bRes = AddLinePoint(x, y, iCtrl, m_pInputPoints);
    break;
  case dtCircle:
    bRes = AddCirclePoint(x, y, iCtrl, m_pInputPoints, m_cLines);
    break;
  case dtEllipse:
    bRes = AddEllipsePoint(x, y, iCtrl, m_pInputPoints, iInputLines);
    break;
  case dtArcEllipse:
    bRes = AddArcElpsPoint(x, y, iCtrl, m_pInputPoints, iInputLines);
    break;
  case dtHyperbola:
    bRes = AddHyperPoint(x, y, iCtrl, m_pInputPoints, iInputLines);
    break;
  case dtParabola:
    bRes = AddParabPoint(x, y, iCtrl, m_pInputPoints, iInputLines);
    break;
  case dtSpline:
    bRes = AddSplinePoint(x, y, iCtrl, m_pInputPoints);
    break;
  case dtEvolvent:
    bRes = AddEvolvPoint(x, y, iCtrl, m_pInputPoints, iInputLines);
    break;
  case dtLogSpiral:
  case dtArchSpiral:
  case dtPath:
  case dtBorderPath:
  case dtBorder:
  case dtArea:
  case dtGroup:
    break;
  }

  return bRes;
}

void CDObject::RemoveLastPoint()
{
  int iCnt = m_pInputPoints->GetCount(-1);
  if(iCnt < 1) return;

  CDInputPoint cInPt = m_pInputPoints->GetPoint(iCnt - 1, -1);
  m_pUndoPoints->AddPoint(cInPt.cPoint.x, cInPt.cPoint.y, cInPt.iCtrl);
  m_pInputPoints->Remove(iCnt - 1, -1);
}

void CDObject::Undo()
{
  RemoveLastPoint();
}

void CDObject::Redo()
{
  int iCnt = m_pUndoPoints->GetCount(-1);
  if(iCnt < 1) return;

  CDInputPoint cInPt = m_pUndoPoints->GetPoint(iCnt - 1, -1);
  m_pInputPoints->AddPoint(cInPt.cPoint.x, cInPt.cPoint.y, cInPt.iCtrl);
  m_pUndoPoints->Remove(iCnt - 1, -1);
}

bool CDObject::BuildSubCache(CDLine cTmpPt, int iMode)
{
  int n = m_pSubObjects->GetCount();
  bool bRes = n > 0;
  int i = 0;
  if(m_iType == dtPath)
  {
    if(iMode == 0)
    {
      CDLine cOffL;
      cOffL.bIsSet = false;
      PDPathSeg pObj;
      while(bRes && (i < n))
      {
        pObj = (PDPathSeg)m_pSubObjects->GetItem(i++);
        bRes = pObj->pSegment->BuildCache(cOffL, 0);
      }
    }

    if(iMode == 2)
    {
      CDLine cSnap;
      m_dMovedDist = GetPathDistFromPt(cTmpPt.cOrigin, cTmpPt.cOrigin, false, &cSnap);
    }
  }
  else if(m_iType == dtGroup)
  {
    /*cOffL.bIsSet = false;
    PDObject pObj;
    while(bRes && (i < n))
    {
      pObj = (PDObject)m_pSubObjects->GetItem(i++);
      bRes = pObj->BuildCache(cOffL, iMode);
    }*/
  }
  return bRes;
}

double CDObject::GetMovedDist(CDLine cTmpPt, int iMode)
{
  double dRes = 0.0;
  int nOffs2 = m_pInputPoints->GetCount(2);
  int nOffs3 = m_pInputPoints->GetCount(3);
  int nOffs4 = m_pInputPoints->GetCount(4);
  int iSrchMask = 0;
  if((iMode == 2) || (nOffs2 > 0) || (nOffs3 > 0) || (nOffs4 > 0))
  {
    int iOffs = m_pCachePoints->GetCount(2);
    if(iOffs > 0) m_pCachePoints->Remove(0, 2);

    CDPoint cPt1;
    CDLine cPtX;
    double dDist = 0.0;
    double dDistOld = 0.0;

    if(iMode == 2)
    {
      cPt1 = cTmpPt.cOrigin;
      if(cTmpPt.cDirection.x < -0.5) iSrchMask = 2;
    }
    else if(nOffs2 > 0) cPt1 = m_pInputPoints->GetPoint(0, 2).cPoint;
    else if(nOffs3 > 0)
    {
      cPt1 = m_pInputPoints->GetPoint(0, 3).cPoint;
      iSrchMask = 2;
    }

    if((iMode == 2) || (nOffs4 == 0))
      dDist = GetDistFromPt(cPt1, cPt1, iSrchMask, &cPtX, NULL);

    if(iMode == 2)
    {
      if(nOffs4 > 0)
        dDistOld = m_pInputPoints->GetPoint(0, 4).cPoint.x;
      else if(nOffs2 > 0)
      {
        cPt1 = m_pInputPoints->GetPoint(0, 2).cPoint;
        dDistOld = GetDistFromPt(cPt1, cPt1, 0, &cPtX, NULL);
      }
      else if(nOffs3 > 0)
      {
        cPt1 = m_pInputPoints->GetPoint(0, 3).cPoint;
        dDistOld = GetDistFromPt(cPt1, cPt1, 2, &cPtX, NULL);
      }
      if(cTmpPt.cDirection.x > 0.5) dDist = dDistOld + cTmpPt.cDirection.y;
    }
    else if(nOffs4 > 0) dDist = m_pInputPoints->GetPoint(0, 4).cPoint.x;

    dRes = dDist - dDistOld;
    if((fabs(dDist) > g_dPrec) || (fabs(dDistOld) > g_dPrec))
      m_pCachePoints->AddPoint(dDist, dDistOld, 2);
  }
  switch(m_iType)
  {
  case dtEllipse:
    UpdateEllipseCache(m_pCachePoints);
    break;
  case dtArcEllipse:
    UpdateArcElpsCache(m_pCachePoints);
    break;
  case dtHyperbola:
    UpdateHyperCache(m_pCachePoints);
    break;
  case dtParabola:
    UpdateParabCache(m_pCachePoints);
    break;
  default:
    break;
  }
  return dRes;
}

bool CDObject::BuildCache(CDLine cTmpPt, int iMode)
{
  bool bRes = false;
  switch(m_iType)
  {
  case dtLine:
    return BuildLineCache(cTmpPt, iMode, m_pInputPoints, m_pCachePoints, &m_dMovedDist);
  case dtCircle:
    return BuildCircCache(cTmpPt, iMode, m_pInputPoints, m_pCachePoints, m_cLines, &m_dMovedDist);
  case dtEllipse:
    bRes = BuildEllipseCache(cTmpPt, iMode, m_pInputPoints, m_pCachePoints, m_cLines);
    break;
  case dtArcEllipse:
    bRes = BuildArcElpsCache(cTmpPt, iMode, m_pInputPoints, m_pCachePoints, m_cLines);
    break;
  case dtHyperbola:
    bRes = BuildHyperCache(cTmpPt, iMode, m_pInputPoints, m_pCachePoints, m_cLines);
    break;
  case dtParabola:
    bRes = BuildParabCache(cTmpPt, iMode, m_pInputPoints, m_pCachePoints, m_cLines);
    break;
  case dtSpline:
    bRes = BuildSplineCache(cTmpPt, iMode, m_pInputPoints, m_pCachePoints);
    break;
  case dtEvolvent:
    bRes = BuildEvolvCache(cTmpPt, iMode, m_pInputPoints, m_pCachePoints, m_cLines);
    break;
  case dtPath:
  case dtGroup:
    bRes = BuildSubCache(cTmpPt, iMode);
    break;
  default:
    return true;
  }

  if(bRes) m_dMovedDist = GetMovedDist(cTmpPt, iMode);
  return bRes;
}

void CDObject::AddSimpleSegment(double dt1, double dt2, double dExt, bool bReverse, PDPrimObject pPrimList)
{
  switch(m_iType)
  {
  case dtLine:
    AddLineSegment(dt1, dt2, dExt, bReverse, m_pCachePoints, pPrimList);
    break;
  case dtCircle:
    AddCircSegment(dt1, dt2, dExt, bReverse, m_pCachePoints, pPrimList);
    break;
  case dtEllipse:
    AddElpsSegment(dt1, dt2, dExt, bReverse, m_pCachePoints, pPrimList);
    break;
  case dtArcEllipse:
    AddArcElpsSegment(dt1, dt2, dExt, bReverse, m_pCachePoints, pPrimList);
    break;
  case dtHyperbola:
    AddHyperSegment(dt1, dt2, dExt, bReverse, m_pCachePoints, pPrimList);
    break;
  case dtParabola:
    AddParabSegment(dt1, dt2, dExt, bReverse, m_pCachePoints, pPrimList);
    break;
  case dtSpline:
    AddSplineSegment(dt1, dt2, dExt, bReverse, m_pCachePoints, pPrimList);
    break;
  case dtEvolvent:
    AddEvolvSegment(dt1, dt2, dExt, bReverse, m_pCachePoints, pPrimList);
    break;
  case dtPath:
    AddPathSegment(dt1, dt2, dExt, pPrimList);
    break;
  default:
    break;
  }
}

void CDObject::AddPathSegment(double d1, double d2, double dExt, PDPrimObject pPrimList)
{
  dExt += m_dMovedDist;

  double dRef1, dRef2;
  int i1, i2;
  PDPathSeg pSeg = GetPathRefSegment(d1, dExt, &dRef1, &i1);
  GetPathRefSegment(d2, dExt, &dRef2, &i2);

  PDObject pObj = pSeg->pSegment;
  double dCurExt;

  if((i1 == i2) && (d1 < d2 - g_dPrec))
  {
    if(pSeg->bReverse) dCurExt = -dExt;
    else dCurExt = dExt;
    if(d2 - d1 < pObj->GetLength(dCurExt) + g_dPrec)
    {
      if(pSeg->bReverse)
        pObj->AddSimpleSegment(dRef2, dRef1, dCurExt, true, pPrimList);
      else
        pObj->AddSimpleSegment(dRef1, dRef2, dCurExt, false, pPrimList);
      return;
    }
  }

  CDPoint cBounds;
  if(pSeg->bReverse) dCurExt = -dExt;
  else dCurExt = dExt;
  if(pObj->GetBoundsRef(&cBounds, dCurExt, true) > 2)
  {
    if(pSeg->bReverse)
      pObj->AddSimpleSegment(cBounds.x, dRef1, dCurExt, true, pPrimList);
    else
      pObj->AddSimpleSegment(dRef1, cBounds.y, dCurExt, false, pPrimList);
  }

  if(i1 < i2)
  {
    for(int i = i1 + 1; i < i2; i++)
    {
      pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
      if(pSeg->bReverse) dCurExt = -dExt;
      else dCurExt = dExt;
      pObj = pSeg->pSegment;
      if(pObj->GetBoundsRef(&cBounds, dCurExt, true) > 2)
        pObj->AddSimpleSegment(cBounds.x, cBounds.y, dCurExt, pSeg->bReverse, pPrimList);
    }
  }
  else
  {
    int iCnt = m_pSubObjects->GetCount();
    for(int i = i1 + 1; i < iCnt; i++)
    {
      pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
      if(pSeg->bReverse) dCurExt = -dExt;
      else dCurExt = dExt;
      pObj = pSeg->pSegment;
      if(pObj->GetBoundsRef(&cBounds, dCurExt, true) > 2)
        pObj->AddSimpleSegment(cBounds.x, cBounds.y, dCurExt, pSeg->bReverse, pPrimList);
    }
    for(int i = 0; i < i2; i++)
    {
      pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
      if(pSeg->bReverse) dCurExt = -dExt;
      else dCurExt = dExt;
      pObj = pSeg->pSegment;
      if(pObj->GetBoundsRef(&cBounds, dCurExt, true) > 2)
        pObj->AddSimpleSegment(cBounds.x, cBounds.y, dCurExt, pSeg->bReverse, pPrimList);
    }
  }

  pSeg = (PDPathSeg)m_pSubObjects->GetItem(i2);
  if(pSeg->bReverse) dCurExt = -dExt;
  else dCurExt = dExt;
  pObj = pSeg->pSegment;
  if(pObj->GetBoundsRef(&cBounds, dCurExt, true) > 2)
  {
    if(pSeg->bReverse)
      pObj->AddSimpleSegment(dRef2, cBounds.y, dCurExt, true, pPrimList);
    else
      pObj->AddSimpleSegment(cBounds.x, dRef2, dCurExt, false, pPrimList);
  }
}

void CDObject::AddCurveSegment(CDPrimitive cAddMode, PDPrimObject pPrimitive, PDRefList pBounds)
{
  double dExt = m_cLineStyle.dPercent*m_cLineStyle.dWidth/200.0;

  int iSegs = pBounds->GetCount()/2;
  CDPrimitive cPath;
  cPath.iType = 11;
  cPath.cPt1 = 0;
  cPath.cPt2 = 0;
  cPath.cPt3 = 0;
  double d1, dDist = 0.0;
  CDPoint cBnds2, pBnds[2];
  int iInterMode = cAddMode.iType & ~8;
  if(cAddMode.cPt4.x > 0.5) iInterMode |= 8;
  int iInter;

  for(int i = 0; i < iSegs; i++)
  {
    cBnds2.x = pBounds->GetPoint(2*i);
    cBnds2.y = pBounds->GetPoint(2*i + 1);
    iInter = IntersectBounds(cAddMode.cPt1, cBnds2, iInterMode, cAddMode.cPt4.y, pBnds);

    for(int j = 0; j < iInter; j++)
    {
      if(cAddMode.iType & 1)
      {
        cPath.cPt1.x = 1;
        cPath.cPt1.y = 0;
        cPath.cPt2.x = 0;
        cPath.cPt3 = 0;
        if(cAddMode.iType & 8)
        {
          cPath.cPt2.x = 1;
          cPath.cPt2.y = pBnds[j].x;
          GetNativeRefPoint(cPath.cPt2.y, dExt, &cPath.cPt3);
        }
        pPrimitive->AddPrimitive(cPath);
      }

      AddSimpleSegment(pBnds[j].x, pBnds[j].y, dExt, false, pPrimitive);

      if(cAddMode.iType & 1)
      {
        cPath.cPt1.x = 2;
        cPath.cPt1.y = 0;
        cPath.cPt2.x = cAddMode.cPt2.x;
        if(cAddMode.cPt2.x > g_dPrec)
        {
          GetPointRefDist(pBnds[j].x, dExt, &d1);
          dDist = cAddMode.cPt3.y - d1 - cAddMode.cPt3.x*cAddMode.cPt2.x;
          if(cAddMode.cPt1.x > pBnds[j].x) dDist -= cAddMode.cPt4.y;
          int n = (int)dDist/cAddMode.cPt2.y/cAddMode.cPt2.x;
          cPath.cPt2.y = n*cAddMode.cPt2.y*cAddMode.cPt2.x - dDist;
        }
        if(cAddMode.cPt4.x > 1.5) cPath.cPt1.y = 2;
        pPrimitive->AddPrimitive(cPath);
      }
    }
  }
}

/*void CDObject::AddPatSegment(double dStart, int iStart, double dEnd, int iEnd,
  PDPoint pBnds, PDRect pRect)
{
  double dPatScale = 1.0;
  double dPatStart2 = m_cLineStyle.dPattern[0]/2.0;
  int iRep;

  double dDist = dEnd - dStart;
  double dPatLen = 0;
  double dPerLen = 0;
  for(int i = 0; i < m_cLineStyle.iSegments; i++)
    dPatLen += m_cLineStyle.dPattern[i];

  if(dStart > dEnd)
  {
    dPerLen = pBnds->y - pBnds->x;
    dDist = dPerLen - dStart + dEnd;
  }

  if((iStart > 0) && (iEnd > 0))
  {
    if(iStart > 1) dDist -= dPatStart2;
    if(iEnd > 1) dDist -= dPatStart2;

    double dRep = dDist/dPatLen;
    if(dRep < 0.8)
    {
      AddCurveSegment(dStart, dEnd, pRect);
      return;
    }

    iRep = (int)Round(dRep);
    if(dStart > dEnd) dDist = dPerLen - dStart + dEnd;
    else dDist = dEnd - dStart;
    double dStretchDist = iRep*dPatLen;
    if(iStart > 1) dStretchDist += dPatStart2;
    if(iEnd > 1) dStretchDist += dPatStart2;

    dPatScale = dDist/dStretchDist;
  }
  else iRep = dDist/dPatLen + 1;

  double d1, d2;
  int i;
  if(iStart > 0)
  {
    i = 0;
    d1 = dStart;
    d2 = d1 + dPatScale*m_cLineStyle.dPattern[i++];
    if(iStart < 2) d2 = d1 + dPatScale*dPatStart2;
    AddCurveSegment(d1, d2, pRect);

    d1 = d2 + dPatScale*m_cLineStyle.dPattern[i++];
    for(int j = 0; j < iRep; j++)
    {
      while(i < m_cLineStyle.iSegments)
      {
        if(d1 > pBnds->y) d1 -= dPerLen;
        d2 = d1 + dPatScale*m_cLineStyle.dPattern[i++];
        AddCurveSegment(d1, d2, pRect);
        d1 = d2 + dPatScale*m_cLineStyle.dPattern[i++];
      }
      i = 0;
    }
    if(d1 > pBnds->y) d1 -= dPerLen;
    d2 = d1 + dPatScale*m_cLineStyle.dPattern[0];
    if((iEnd > 0) && (iEnd < 2)) d2 = d1 + dPatScale*dPatStart2;
    AddCurveSegment(d1, d2, pRect);
  }
  else // iEnd should be > 0
  {
    d2 = dEnd;
    d1 = d2 - dPatScale*m_cLineStyle.dPattern[0];
    if(iEnd < 2) d1 = d2 - dPatScale*dPatStart2;
    AddCurveSegment(d1, d2, pRect);

    for(int j = 0; j < iRep; j++)
    {
      i = m_cLineStyle.iSegments;
      while(i > 0)
      {
        d2 = d1 - dPatScale*m_cLineStyle.dPattern[--i];
        d1 = d2 - dPatScale*m_cLineStyle.dPattern[--i];
        AddCurveSegment(d1, d2, pRect);
      }
    }
  }
}*/

void CDObject::AddPatSegment(double dStart, int iStart, double dEnd, int iEnd,
  int iBoundMode, PDRefList pViewBnds, PDPoint pBnds)
{
  double dExt = m_cLineStyle.dPercent*m_cLineStyle.dWidth/200.0;

  CDPrimitive cAdd;
  cAdd.iType = 1;
  cAdd.cPt4.x = 0.0;
  cAdd.cPt4.y = 0.0;
  if(iBoundMode & 1)
  {
    cAdd.cPt4.x = 1.0;
    cAdd.cPt4.y = pBnds->y - pBnds->x;
  }
  if(iBoundMode & 2) cAdd.iType |= 8;
  if(iBoundMode & 4) cAdd.cPt4.x = 2.0;

  double dPatScale = 1.0;
  double dSegLen = m_cLineStyle.dPattern[0];
  if(dSegLen < g_dDashMin) dSegLen = g_dDashMin;
  double dPatStart2 = dSegLen/2.0;
//  int iRep;

  double d1, d2;
  GetPointRefDist(dStart, dExt, &d1);
  GetPointRefDist(dEnd, dExt, &d2);
  double dDist = d2 - d1;
  double dPatLen = 0.0;
  double dPerLen = 0.0;
  for(int i = 0; i < m_cLineStyle.iSegments; i++)
  {
    dSegLen = m_cLineStyle.dPattern[i];
    if((i % 2 == 0) && (dSegLen < g_dDashMin)) dSegLen = g_dDashMin;
    dPatLen += dSegLen;
  }

  if(dStart > dEnd)
  {
    double dd1, dd2;
    GetPointRefDist(pBnds->x, dExt, &dd1);
    GetPointRefDist(pBnds->y, dExt, &dd2);
    dPerLen = dd2 - dd1;
    dDist = dPerLen - d1 + d2;
  }

  if((iStart > 0) && (iEnd > 0))
  {
    if(iStart > 1) dDist -= dPatStart2;
    if(iEnd > 1) dDist -= dPatStart2;

    double dRep = dDist/dPatLen;
    if(dRep < 0.8)
    {
      cAdd.iType |= 7;
      cAdd.cPt1.x = dStart;
      cAdd.cPt1.y = dEnd;
      cAdd.cPt2.x = 0.0;
      AddCurveSegment(cAdd, m_pPrimitive, pViewBnds);
      return;
    }

    int iRep = (int)Round(dRep);
    if(dStart > dEnd) dDist = dPerLen - d1 + d2;
    else dDist = d2 - d1;
    double dStretchDist = iRep*dPatLen;
    if(iStart > 1) dStretchDist += dPatStart2;
    if(iEnd > 1) dStretchDist += dPatStart2;

    dPatScale = dDist/dStretchDist;
  }
//  else iRep = dDist/dPatLen + 1;

  cAdd.iType |= 1;
  if(iStart > 0) cAdd.iType |= 2;
  if(iEnd > 0) cAdd.iType |= 4;
  cAdd.cPt1.x = dStart;
  cAdd.cPt1.y = dEnd;
  cAdd.cPt2.x = dPatScale;
  cAdd.cPt2.y = dPatLen;
  if(iStart > 1) cAdd.cPt3.x = 0.0;
  else if(iStart > 0) cAdd.cPt3.x = dPatStart2;
  else if(iEnd > 1) cAdd.cPt3.x = 2.0*dPatStart2;
  else cAdd.cPt3.x = dPatStart2;
  if(iStart > 0) cAdd.cPt3.y = d1;
  else cAdd.cPt3.y = d2;
  AddCurveSegment(cAdd, m_pPrimitive, pViewBnds);

/*  double d1, d2;
  int i;
  if(iStart > 0)
  {
    i = 0;
    d1 = dStart;
    d2 = d1 + dPatScale*m_cLineStyle.dPattern[i++];
    if(iStart < 2) d2 = d1 + dPatScale*dPatStart2;
    cAdd.cPt1.x = d1;
    cAdd.cPt1.y = d2;
    AddCurveSegment(cAdd, m_pPrimitive, 0, pViewBnds);

    d1 = d2 + dPatScale*m_cLineStyle.dPattern[i++];
    for(int j = 0; j < iRep; j++)
    {
      while(i < m_cLineStyle.iSegments)
      {
        if(d1 > pBnds->y) d1 -= dPerLen;
        d2 = d1 + dPatScale*m_cLineStyle.dPattern[i++];
        cAdd.cPt1.x = d1;
        cAdd.cPt1.y = d2;
        AddCurveSegment(cAdd, m_pPrimitive, 0, pViewBnds);
        d1 = d2 + dPatScale*m_cLineStyle.dPattern[i++];
      }
      i = 0;
    }
    if(d1 > pBnds->y) d1 -= dPerLen;
    d2 = d1 + dPatScale*m_cLineStyle.dPattern[0];
    if((iEnd > 0) && (iEnd < 2)) d2 = d1 + dPatScale*dPatStart2;
    cAdd.cPt1.x = d1;
    cAdd.cPt1.y = d2;
    AddCurveSegment(cAdd, m_pPrimitive, 0, pViewBnds);
  }
  else // iEnd should be > 0
  {
    d2 = dEnd;
    d1 = d2 - dPatScale*m_cLineStyle.dPattern[0];
    if(iEnd < 2) d1 = d2 - dPatScale*dPatStart2;
    cAdd.cPt1.x = d1;
    cAdd.cPt1.y = d2;
    AddCurveSegment(cAdd, m_pPrimitive, 0, pViewBnds);

    for(int j = 0; j < iRep; j++)
    {
      i = m_cLineStyle.iSegments;
      while(i > 0)
      {
        d2 = d1 - dPatScale*m_cLineStyle.dPattern[--i];
        d1 = d2 - dPatScale*m_cLineStyle.dPattern[--i];
        cAdd.cPt1.x = d1;
        cAdd.cPt1.y = d2;
        AddCurveSegment(cAdd, m_pPrimitive, 0, pViewBnds);
      }
    }
  }*/
}

int CDObject::AddDimenPrimitive(int iPos, PDDimension pDim, PDPrimObject pPrimitive, PDRect pRect)
{
  CDPrimitive cPrim;
  cPrim.iType = 9;
  CDPoint cDir, bPt1;

  if(pDim->iArrowType1 > 0)
  {
    cPrim.cPt1.x = pDim->iArrowType1;
    cPrim.cPt1.y = iPos;
    if(!GetNativeRefPoint(pDim->dRef1, 0.0, &cPrim.cPt2)) return 0;
    if(!GetNativeRefDir(pDim->dRef1, &cDir)) return 0;

    if(pDim->iRefDir < 0) cDir *= -1.0;

    bPt1 = pDim->cArrowDim1;
    if(pDim->iArrowType1 == 3) cPrim.cPt3 = cPrim.cPt2 + bPt1;
    else
    {
      if(pDim->iArrowType1 == 5) bPt1.x *= -1.0;
      cPrim.cPt3 = cPrim.cPt2 + Rotate(bPt1, cDir, true);
      bPt1.y *= -1.0;
      if(pDim->iArrowType1 > 3) bPt1.x *= -1.0;
      cPrim.cPt4 = cPrim.cPt2 + Rotate(bPt1, cDir, true);
    }
    pPrimitive->AddPrimitive(cPrim);
  }

  if(pDim->iArrowType2 > 0)
  {
    cPrim.cPt1.x = pDim->iArrowType2;
    cPrim.cPt1.y = iPos;
    if(!GetNativeRefPoint(pDim->dRef2, 0.0, &cPrim.cPt2)) return 0;
    if(!GetNativeRefDir(pDim->dRef2, &cDir)) return 0;

    if(pDim->iRefDir > 0) cDir *= -1.0;

    bPt1 = pDim->cArrowDim2;
    if(pDim->iArrowType2 == 3) cPrim.cPt3 = cPrim.cPt2 + bPt1;
    else
    {
      if(pDim->iArrowType2 == 5) bPt1.x *= -1.0;
      cPrim.cPt3 = cPrim.cPt2 + Rotate(bPt1, cDir, true);
      bPt1.y *= -1.0;
      if(pDim->iArrowType2 > 3) bPt1.x *= -1.0;
      cPrim.cPt4 = cPrim.cPt2 + Rotate(bPt1, cDir, true);
    }
    pPrimitive->AddPrimitive(cPrim);
  }

  cPrim.iType = 10;
  cPrim.cPt1 = pDim->cLabelPos.cPoint;
  cPrim.cPt2.x = pDim->cLabelPos.dOrientation;
  cPrim.cPt2.y = iPos;
  pPrimitive->AddPrimitive(cPrim);
  return 0;
}

int CDObject::AddSubIntersects(CDPoint cPt1, CDPoint cPt2, double dOffset, PDRefList pBounds)
{
  int n = m_pSubObjects->GetCount();
  bool bRes = n > 0;
  int i = 0;
  if(m_iType == dtPath)
  {
    //PDPathSeg pObj;
    while(bRes && (i < n))
    {
      //pObj = (PDPathSeg)m_pSubObjects->GetItem(i++);
      //bRes = pObj->pSegment->BuildCache(cTmpPt, iMode);
    }
  }
  else if(m_iType == dtGroup)
  {
    //PDObject pObj;
    //while(bRes && (i < n))
    //{
    //  pObj = (PDObject)m_pSubObjects->GetItem(i++);
    //  bRes = pObj->BuildCache(cTmpPt, iMode);
    //}
  }
  return 0;
}

int CDObject::AddLineIntersects(CDPoint cPt1, CDPoint cPt2, double dOffset, PDRefList pBounds)
{
  int iRes = 0;
  switch(m_iType)
  {
  case dtLine:
    iRes = AddLineInterLine(cPt1, cPt2, dOffset, m_pCachePoints, pBounds);
    break;
  case dtCircle:
    iRes = AddCircleInterLine(cPt1, cPt2, dOffset, m_pCachePoints, pBounds);
    break;
  case dtEllipse:
    iRes = AddEllipseInterLine(cPt1, cPt2, dOffset, m_pCachePoints, pBounds);
    break;
  case dtArcEllipse:
    iRes = AddArcElpsInterLine(cPt1, cPt2, dOffset, m_pCachePoints, pBounds);
    break;
  case dtHyperbola:
    iRes = AddHyperInterLine(cPt1, cPt2, dOffset, m_pCachePoints, pBounds);
    break;
  case dtParabola:
    iRes = AddParabInterLine(cPt1, cPt2, dOffset, m_pCachePoints, pBounds);
    break;
  case dtSpline:
    iRes = AddSplineInterLine(cPt1, cPt2, dOffset, m_pCachePoints, pBounds);
    break;
  case dtEvolvent:
    iRes = AddEvolvInterLine(cPt1, cPt2, dOffset, m_pCachePoints, pBounds);
    break;
  case dtLogSpiral:
  case dtArchSpiral:
    break;
  case dtPath:
    iRes = AddSubIntersects(cPt1, cPt2, dOffset, pBounds);
    break;
  case dtBorderPath:
  case dtBorder:
  case dtArea:
  case dtGroup:
    break;
  }
  return iRes;
}

int CDObject::GetRectangleIntersects(PDRect pRect, double dOffset, int iBndMode, PDPoint pRefBnds, PDRefList pBounds)
{
  CDPoint cPt1, cPt2;
  cPt1 = pRect->cPt1;
  cPt2.x = pRect->cPt2.x;
  cPt2.y = cPt1.y;
  int iTot = 0;
  iTot += AddLineIntersects(cPt1, cPt2, dOffset, pBounds);
  cPt1 = cPt2;
  cPt2.y = pRect->cPt2.y;
  iTot += AddLineIntersects(cPt1, cPt2, dOffset, pBounds);
  cPt1 = cPt2;
  cPt2.x = pRect->cPt1.x;
  iTot += AddLineIntersects(cPt1, cPt2, dOffset, pBounds);
  cPt1 = cPt2;
  cPt2.y = pRect->cPt1.y;
  iTot += AddLineIntersects(cPt1, cPt2, dOffset, pBounds);

  if(iTot > 0)
  {
    CDPoint cProbe;
    pBounds->Sort(0);
    if(iBndMode > 0)
    {
      GetNativeRefPoint(pRefBnds->x, dOffset, &cProbe);
      if(DPtInDRect(cProbe, pRect))
      {
        pBounds->InsertPoint(0, pRefBnds->x);
        iTot++;
      }
      if(iBndMode > 1)
      {
        GetNativeRefPoint(pRefBnds->y, dOffset, &cProbe);
        if(DPtInDRect(cProbe, pRect))
        {
          pBounds->AddPoint(pRefBnds->y);
          iTot++;
        }
      }
    }

    int i = iTot - 1;
    bool bLastIn = false;
    bool bCurIn = false;
    double dRef1, dRef2, dRef;
    dRef1 = pBounds->GetPoint(i--);
    while(i >= 0)
    {
      bLastIn = bCurIn;
      dRef2 = dRef1;
      dRef1 = pBounds->GetPoint(i--);
      dRef = (dRef1 + dRef2)/2.0;
      GetNativeRefPoint(dRef, dOffset, &cProbe);
      bCurIn = DPtInDRect(cProbe, pRect);
      if(bCurIn && bLastIn)
      {
        pBounds->Remove(i + 2);
        iTot--;
      }
    }
  }

  return iTot;
}

int CDObject::GetSimpleViewBounds(CDLine cTmpPt, int iMode, double dOffset, double dLineHalfWidth,
  PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds, bool bMergeWithBounds)
{
  if(iMode == 1) BuildCache(cTmpPt, iMode);
  else if(iMode == 2)
  {
    if(m_iType > dtCircle)
      m_dMovedDist = GetMovedDist(cTmpPt, iMode);
    else BuildCache(cTmpPt, iMode);
  }

  int iRectFlag = 0;
  double dCornerRefs[4] = {0.0, 0.0, 0.0, 0.0};
  CDLine cPtX;
  double dMid = dOffset;
  double dExt = dLineHalfWidth;

  CDPoint cCorner = pRect->cPt1;
  double d1 = GetDistFromPt(cCorner, cCorner, 0, &cPtX, NULL);
  double d2;
  if(cPtX.bIsSet && fabs(d1 - dMid) < dExt + g_dPrec)
  {
    iRectFlag |= 1;
    dCornerRefs[0] = cPtX.dRef;
  }

  cCorner = pRect->cPt2;
  d1 = GetDistFromPt(cCorner, cCorner, 0, &cPtX, NULL);
  if(cPtX.bIsSet && fabs(d1 - dMid) < dExt + g_dPrec)
  {
    iRectFlag |= 2;
    dCornerRefs[1] = cPtX.dRef;
  }

  cCorner.x = pRect->cPt1.x;
  cCorner.y = pRect->cPt2.y;
  d1 = GetDistFromPt(cCorner, cCorner, 0, &cPtX, NULL);
  if(cPtX.bIsSet && fabs(d1 - dMid) < dExt + g_dPrec)
  {
    iRectFlag |= 4;
    dCornerRefs[2] = cPtX.dRef;
  }

  cCorner.x = pRect->cPt2.x;
  cCorner.y = pRect->cPt1.y;
  d1 = GetDistFromPt(cCorner, cCorner, 0, &cPtX, NULL);
  if(cPtX.bIsSet && fabs(d1 - dMid) < dExt + g_dPrec)
  {
    iRectFlag |= 8;
    dCornerRefs[3] = cPtX.dRef;
  }

  int iBndType = GetRefBounds(pDrawBnds);

  if(iRectFlag == 15)
  {
    std::sort(dCornerRefs, &dCornerRefs[4]);
    d1 = dCornerRefs[0];
    d2 = dCornerRefs[3];
    if(IsClosedShape())
    {
      double dSpan = (pDrawBnds->y - pDrawBnds->x)/2.0;
      if(dCornerRefs[3] - dCornerRefs[0] > dSpan)
      {
        if(dCornerRefs[2] - dCornerRefs[0] > dSpan)
        {
          if(dCornerRefs[1] - dCornerRefs[0] > dSpan)
          {
            d1 = dCornerRefs[1];
            d2 = dCornerRefs[0];
          }
          else
          {
            d1 = dCornerRefs[2];
            d2 = dCornerRefs[1];
          }
        }
        else
        {
          d1 = dCornerRefs[3];
          d2 = dCornerRefs[2];
        }
      }
    }
    pBounds->AddPoint(d1);
    pBounds->AddPoint(d2);
    return 1;
  }

  CDPoint cProbe;
  PDRefList pInt1 = new CDRefList();
  PDRefList pInt2 = new CDRefList();
  int iCnt1 = GetRectangleIntersects(pRect, dMid - dExt, iBndType, pDrawBnds, pInt1);
  int iCnt2 = GetRectangleIntersects(pRect, dMid + dExt, iBndType, pDrawBnds, pInt2);

  if((iCnt1 < 1) || (iCnt2 < 1))
  {
    if(iBndType > 1)
    {
      if(iCnt1 < 1) GetNativeRefPoint(0.0, dMid - dExt, &cProbe);
      else GetNativeRefPoint(0.0, dMid + dExt, &cProbe);
      if(DPtInDRect(cProbe, pRect))
      {
        delete pInt2;
        delete pInt1;
        pBounds->AddPoint(pDrawBnds->x);
        pBounds->AddPoint(pDrawBnds->y);
        if(IsClosedShape() || (iBndType > 0)) return 2;
        return 1;
      }
    }
  }

  if((iCnt1 < 1) && (iCnt2 < 1))
  {
    delete pInt2;
    delete pInt1;
    return 0;
  }

  int iRes = UnionBounds(pInt1, pInt2, iBndType, pDrawBnds);
  delete pInt2;

  if(IsClosedShape())
    MergeCornerRefs(pInt1, pDrawBnds, iRectFlag, dCornerRefs);
  else
    MergeCornerRefs(pInt1, NULL, iRectFlag, dCornerRefs);

  int iLen = pInt1->GetCount();
  for(int i = 0; i < iLen; i++)
  {
    pBounds->AddPoint((*pInt1)[i]);
  }

  if(bMergeWithBounds && (iRes == 1))
  {
    if(IsClosedShape())
    {
      if(m_cBounds[0].bIsSet && m_cBounds[1].bIsSet)
      {
        if(m_cBounds[1].dRef < m_cBounds[0].dRef)
        {
          if((*pInt1)[0] > (*pInt1)[1])
          {
            if((m_cBounds[0].dRef > (*pInt1)[0] - g_dPrec) && (m_cBounds[1].dRef < (*pInt1)[1] + g_dPrec))
              iRes = 2;
          }
          else if((m_cBounds[1].dRef < (*pInt1)[0] + g_dPrec) && (m_cBounds[0].dRef > (*pInt1)[iLen - 1] - g_dPrec))
            iRes = 0;
        }
        else
        {
          bool bFound = false;
          int i = 0;
          if((*pInt1)[0] > (*pInt1)[1])
          {
            if((m_cBounds[1].dRef < (*pInt1)[1] + g_dPrec) || (m_cBounds[0].dRef > (*pInt1)[0] - g_dPrec))
            {
              iRes = 2;
              i = iLen;
            }
            else if((m_cBounds[0].dRef > (*pInt1)[1] - g_dPrec) || (m_cBounds[1].dRef < (*pInt1)[0] + g_dPrec))
            {
              iRes = 0;
              i = iLen;
            }
            else i = 1;
          }
          else if((m_cBounds[0].dRef > (*pInt1)[iLen - 1] - g_dPrec) || (m_cBounds[1].dRef < (*pInt1)[0] + g_dPrec))
          {
            iRes = 0;
            i = iLen;
          }
          while(!bFound && (i < iLen - 1))
          {
            bFound = (m_cBounds[0].dRef > (*pInt1)[i] - g_dPrec) &&
              (m_cBounds[0].dRef < (*pInt1)[i + 1] + g_dPrec) &&
              (m_cBounds[1].dRef > (*pInt1)[i] - g_dPrec) &&
              (m_cBounds[1].dRef < (*pInt1)[i + 1] + g_dPrec);
            i++;
          }
          if(bFound)
          {
            if(i % 2 > 0) iRes = 2;
            else iRes = 0;
          }
        }
      }
    }
    else
    {
      if(m_cBounds[0].bIsSet)
      {
        if(m_cBounds[1].bIsSet)
        {
          if(m_cBounds[1].dRef < (*pInt1)[0] + g_dPrec) iRes = 0;
          else if(m_cBounds[0].dRef > (*pInt1)[iLen - 1] - g_dPrec) iRes = 0;
          else
          {
            bool bFound = false;
            int i = 0;
            while(!bFound && (i < iLen - 1))
            {
              bFound = (m_cBounds[0].dRef > (*pInt1)[i] - g_dPrec) &&
                (m_cBounds[0].dRef < (*pInt1)[i + 1] + g_dPrec) &&
                (m_cBounds[1].dRef > (*pInt1)[i] - g_dPrec) &&
                (m_cBounds[1].dRef < (*pInt1)[i + 1] + g_dPrec);
              i++;
            }
            if(bFound)
            {
              if(i % 2 > 0) iRes = 2;
              else iRes = 0;
            }
          }
        }
        else if(m_cBounds[0].dRef > (*pInt1)[iLen - 1] - g_dPrec) iRes = 0;
      }
      else if(m_cBounds[1].bIsSet && (m_cBounds[1].dRef < (*pInt1)[0] + g_dPrec)) iRes = 0;
    }
  }

  delete pInt1;

  return iRes;
}

void CDObject::AddExtraPrimitives(PDRect pRect, PDPrimObject pPrimList)
{
  switch(m_iType)
  {
  case dtLine:
    break;
  case dtCircle:
    AddCircleExtPrim(pRect, m_pCachePoints, pPrimList);
    break;
  case dtEllipse:
    AddElpsExtPrim(pRect, m_pCachePoints, pPrimList);
    break;
  case dtArcEllipse:
  case dtHyperbola:
  case dtParabola:
  case dtSpline:
  case dtEvolvent:
  case dtLogSpiral:
  case dtArchSpiral:
  case dtPath:
  case dtBorderPath:
  case dtBorder:
  case dtArea:
  case dtGroup:
    break;
  }
}

double NormalizeAngle(PDPoint pNorm)
{
  double dAng = atan2(pNorm->y, pNorm->x);
  double dPi2 = M_PI/2.0;
  if(dAng < g_dPrec - M_PI)
  {
    dAng = -M_PI;
    pNorm->x = -1.0;
    pNorm->y = 0;
  }
  else if(dAng > M_PI - g_dPrec)
  {
    dAng = M_PI;
    pNorm->x = -1.0;
    pNorm->y = 0;
  }
  else if(fabs(dAng) < g_dPrec)
  {
    dAng = 0.0;
    pNorm->x = 1.0;
    pNorm->y = 0.0;
  }
  else if(fabs(dAng - dPi2) < g_dPrec)
  {
    dAng = dPi2;
    pNorm->x = 0.0;
    pNorm->y = 1.0;
  }
  else if(fabs(dAng + dPi2) < g_dPrec)
  {
    dAng = -dPi2;
    pNorm->x = 0.0;
    pNorm->y = -1.0;
  }

  return dAng;
}

int CDObject::GetPathViewBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds)
{
  int iCnt = m_pSubObjects->GetCount();
  int iRes = -1;
  int iLocRes;
  PDPathSeg pSeg;
  PDRefList pLocBounds = new CDRefList();
  CDPoint cLocBnds = {0.0, 0.0};
  CDLine cOffL;
  cOffL.bIsSet = false;

  if(iMode == 2)
  {
    CDLine cSnap;
    m_dMovedDist = GetPathDistFromPt(cTmpPt.cOrigin, cTmpPt.cOrigin, false, &cSnap);
  }

  double dExt = m_cLineStyle.dWidth/2.0;
  double dMid = m_dMovedDist + m_cLineStyle.dPercent*dExt/100.0;

  for(int i = 0; i < iCnt; i++)
  {
    pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
    if(pSeg->bReverse)
      iLocRes = pSeg->pSegment->GetSimpleViewBounds(cOffL, 0, -dMid, dExt, pRect, pLocBounds, &cLocBnds, true);
    else
      iLocRes = pSeg->pSegment->GetSimpleViewBounds(cOffL, 0, dMid, dExt, pRect, pLocBounds, &cLocBnds, true);
    if(iRes < 0) iRes = iLocRes;
    else if(iLocRes != iRes) iRes = 1;
  }

  if(iRes > 0)
  {
    pDrawBnds->x = 0.0;
    pDrawBnds->y = GetLength(dMid);
    pBounds->AddPoint(pDrawBnds->x);
    pBounds->AddPoint(pDrawBnds->y);
  }
  return iRes;
}

int CDObject::GetViewBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds, bool bMergeWithBounds)
{
  int iRes = 0;
  switch(m_iType)
  {
  case dtPath:
    iRes = GetPathViewBounds(cTmpPt, iMode, pRect, pBounds, pDrawBnds);
  case dtBorderPath:
  case dtBorder:
  case dtArea:
  case dtGroup:
    break;
  default:
    double dExt = m_cLineStyle.dWidth/2.0;
    double dMid = m_cLineStyle.dPercent*dExt/100.0;
    iRes = GetSimpleViewBounds(cTmpPt, iMode, dMid, dExt, pRect, pBounds, pDrawBnds, bMergeWithBounds);
  }
  return iRes;
}

int CDObject::BuildPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, int iTemp, PDFileAttrs pAttrs)
{
  PDPrimObject plPrimitive = m_pPrimitive;
  if(iTemp > 1) plPrimitive = new CDPrimObject();
  plPrimitive->Clear();

  double dMid = m_cLineStyle.dPercent*m_cLineStyle.dWidth/200.0;

  CDPoint cBnds = {0.0, 0.0};
  PDRefList pBounds = new CDRefList();
  int iRes = GetViewBounds(cTmpPt, iMode, pRect, pBounds, &cBnds, false);

  int nCrs = m_pCrossPoints->GetCount();

  int iClosed = IsClosed();
  bool bClosed = IsClosedShape();

  CDPrimitive cAdd;
  cAdd.iType = 9;
  cAdd.cPt2.x = 0.0;
  cAdd.cPt4.x = 0.0;
  cAdd.cPt4.y = 0.0;
  if(iClosed > 0)
  {
    cAdd.cPt4.x = 1.0;
    cAdd.cPt4.y = cBnds.y - cBnds.x;
  }

  if((iTemp < 1) && (iRes > 0) && (m_cLineStyle.iSegments > 0))
  {
    //plPrimitive->ClearLines();

    double dStart, dEnd;
    int iStart, iEnd;
    double d1, d2;
    double dPatLen = 0.0;
    int iBoundMode = 2;
    if(bClosed) iBoundMode |= 1;
    if((iClosed > 1) && (iRes > 1) && (nCrs < 2)) iBoundMode |= 4;
    double dSegLen;
    for(int i = 0; i < m_cLineStyle.iSegments; i++)
    {
      dSegLen = m_cLineStyle.dPattern[i];
      if((i % 2 == 0) && (dSegLen < g_dDashMin)) dSegLen = g_dDashMin;
      dPatLen += dSegLen;
    }

    if(m_cBounds[0].bIsSet)
    {
      d1 = m_cBounds[0].dRef;

      dStart = d1;
      dEnd = d1;
      iStart = 2;
      iEnd = 0;
      if(m_iType == dtSpline) iEnd = 2;
      else if(iClosed > 0) iEnd = 1;

      int iFirst = 0;
      if(m_cBounds[1].bIsSet)
      {
        d2 = m_cBounds[1].dRef;
        if(d1 > d2) iFirst = m_pCrossPoints->GetIndex(m_cBounds[0].dRef);
        if(iFirst < 0) iFirst = 0;
      }
      else if(iClosed > 0)
      {
        iFirst = m_pCrossPoints->GetIndex(m_cBounds[0].dRef);
        if(iFirst < 0) iFirst = 0;
      }

      if(nCrs > 0)
      {
        iStart = 1;
        dEnd = m_pCrossPoints->GetPoint(iFirst);
        if(fabs(dEnd - d1) > g_dPrec)
        {
          AddPatSegment(d1, 2, dEnd, 1, iBoundMode, pBounds, &cBnds);
          dStart = dEnd;
        }
        else if((iClosed > 0) && (nCrs < 2))
        {
          dEnd = d1 + cAdd.cPt4.y;
          AddPatSegment(d1, 1, dEnd, 1, iBoundMode, pBounds, &cBnds);
          dStart = dEnd;
        }

        for(int i = iFirst + 1; i < nCrs; i++)
        {
          dStart = dEnd;
          dEnd = m_pCrossPoints->GetPoint(i);
          AddPatSegment(dStart, 1, dEnd, 1, iBoundMode, pBounds, &cBnds);
        }

        if(iFirst > 0)
        {
          for(int i = 0; i < iFirst; i++)
          {
            dStart = dEnd;
            dEnd = m_pCrossPoints->GetPoint(i);
            AddPatSegment(dStart, 1, dEnd, 1, iBoundMode, pBounds, &cBnds);
          }
        }
      }

      if(m_cBounds[1].bIsSet)
      {
        if(nCrs > 0)
        {
          if(fabs(dEnd - d2) > g_dPrec)
          AddPatSegment(dEnd, 1, d2, 2, iBoundMode, pBounds, &cBnds);
        }
        else
          AddPatSegment(d1, 2, d2, 2, iBoundMode, pBounds, &cBnds);
      }
      else
      {
        if(iClosed > 0)
        {
          iEnd = 2;
          dStart = dEnd;
          dEnd = d1;
          d2 = d1 - 1.0;
          if(nCrs < 1) dEnd += cAdd.cPt4.y;
          else d2 = m_pCrossPoints->GetPoint(iFirst);
          if(fabs(dEnd - d2) < g_dPrec) iEnd = 1;
          AddPatSegment(dStart, iStart, dEnd, iEnd, iBoundMode, pBounds, &cBnds);
        }
        else
          AddPatSegment(dEnd, iStart, cBnds.y, iEnd, iBoundMode, pBounds, &cBnds);
      }
    }
    else if(m_cBounds[1].bIsSet)
    {
      d2 = m_cBounds[1].dRef;

      iStart = 0;
      if(m_iType > dtParabola) iStart = 2;

      if(nCrs > 0)
      {
        dEnd = m_pCrossPoints->GetPoint(0);
        AddPatSegment(cBnds.x, iStart, dEnd, 1, iBoundMode, pBounds, &cBnds);

        for(int i = 1; i < nCrs; i++)
        {
          dStart = dEnd;
          dEnd = m_pCrossPoints->GetPoint(i);
          AddPatSegment(dStart, 1, dEnd, 1, iBoundMode, pBounds, &cBnds);
        }
        iStart = 1;
        if(fabs(dEnd - d2) > g_dPrec)
          AddPatSegment(dEnd, 1, d2, 2, iBoundMode, pBounds, &cBnds);
      }
      else
      {
        dEnd = d2;
        AddPatSegment(cBnds.x, iStart, dEnd, 2, iBoundMode, pBounds, &cBnds);
      }
    }
    else if(iClosed > 0)
    {
      if(nCrs > 0)
      {
        dStart = m_pCrossPoints->GetPoint(0);
        for(int i = 1; i < nCrs; i++)
        {
          dEnd = m_pCrossPoints->GetPoint(i);
          AddPatSegment(dStart, 1, dEnd, 1, iBoundMode, pBounds, &cBnds);
          dStart = dEnd;
        }
        dEnd = m_pCrossPoints->GetPoint(0);
        if(nCrs < 2) dEnd += (cBnds.y - cBnds.x);
      }
      else
      {
        dStart = cBnds.x;
        dEnd = cBnds.y;
      }
      AddPatSegment(dStart, 1, dEnd, 1, iBoundMode, pBounds, &cBnds);
    }
    else
    {
      iStart = 0;
      iEnd = 0;
      if(m_iType > dtParabola) iStart = 2;
      if(m_iType == dtSpline || m_iType == dtPath) iEnd = 2;

      if(nCrs < 1)
      {
        dStart = cBnds.x;
        dEnd = cBnds.y;
        AddPatSegment(dStart, iStart, dEnd, iEnd, iBoundMode, pBounds, &cBnds);
      }
      else
      {
        int i = 0;
        int iTmpEnd = 1;
        dStart = 0.0;
        dEnd = m_pCrossPoints->GetPoint(i++);
        if(iStart > 0)
        {
          dStart = cBnds.x;
          if(dEnd < dStart + g_dPrec)
          {
            iStart = 1;
            if(nCrs > 1) dEnd = m_pCrossPoints->GetPoint(i++);
            else
            {
              dEnd = cBnds.y;
              iTmpEnd = 2;
            }
          }
        }
        AddPatSegment(dStart, iStart, dEnd, iTmpEnd, iBoundMode, pBounds, &cBnds);
        iStart = 1;
        while(i < nCrs)
        {
          dStart = dEnd;
          dEnd = m_pCrossPoints->GetPoint(i++);
          AddPatSegment(dStart, 1, dEnd, 1, iBoundMode, pBounds, &cBnds);
        }
        dStart = dEnd;
        dEnd = 0.0;
        if(iEnd > 0) dEnd = cBnds.y;
        if((iEnd < 1) || (dStart < dEnd - g_dPrec))
          AddPatSegment(dStart, iStart, dEnd, iEnd, iBoundMode, pBounds, &cBnds);
      }
    }
  }
  else if(iRes > 0)
  {
    if(bClosed)
    {
      cAdd.cPt4.x = 1.0;
      cAdd.cPt4.y = cBnds.y - cBnds.x;
    }
    if((iClosed > 1) && (iRes > 1)) cAdd.cPt4.x = 2.0;

    int iBndType = GetBoundsRef(&cAdd.cPt1, dMid, iClosed > 0);
    if(iBndType & 1) cAdd.iType |= 2;
    if(iBndType & 2) cAdd.iType |= 4;
    if((nCrs > 0) && (iClosed > 1))
    {
      double dLen = cAdd.cPt1.y - cAdd.cPt1.x;
      double dt = m_pCrossPoints->GetPoint(0);
      GetPointRefDist(dt, dMid, &cAdd.cPt1.x);
      cAdd.cPt1.y = cAdd.cPt1.x + dLen;
    }
    AddCurveSegment(cAdd, plPrimitive, pBounds);
  }

  AddExtraPrimitives(pRect, plPrimitive);

  CDPrimitive cPrim;

  if((iTemp < 1) && (iRes > 0))
  {
    cPrim.iType = 6;
    cPrim.cPt1 = 0;
    if(m_cBounds[0].bIsSet)
    {
      if(GetNativeRefPoint(m_cBounds[0].dRef, 0.0, &cPrim.cPt2)) cPrim.cPt1.x = 1.0;
    }
    if(m_cBounds[1].bIsSet)
    {
      if(GetNativeRefPoint(m_cBounds[1].dRef, 0.0, &cPrim.cPt3)) cPrim.cPt1.y = 1.0;
    }
    CropPoints(cPrim, pRect, plPrimitive);
  }

  if((iTemp < 1) && (iRes > 0) && (nCrs > 0))
  {
    cPrim.iType = 8;
    for(int i = 0; i < nCrs; i++)
    {
      if(GetNativeRefPoint(m_pCrossPoints->GetPoint(i), 0.0, &cPrim.cPt1))
        plPrimitive->AddPrimitive(cPrim);
    }
  }

  if((iTemp < 2) && (iRes > 0))
  {
    int nDim = m_pDimens->GetCount();
    PDDimension pDim;
    for(int i = 0; i < nDim; i++)
    {
      pDim = (PDDimension)m_pDimens->GetItem(i);
      AddDimenPrimitive(i, pDim, plPrimitive, pRect);
    }

    if((iMode == 4) && m_bFirstDimSet && pAttrs)
    {
      CDLine cPtX;
      m_cTmpDim.dRef1 = m_dFirstDimen;
      GetDistFromPt(cTmpPt.cOrigin, cTmpPt.cOrigin, 1, &cPtX, NULL);

      m_cTmpDim.dRef2 = cPtX.dRef;
      m_cTmpDim.iArrowType1 = pAttrs->iArrowType;
      m_cTmpDim.iArrowType2 = pAttrs->iArrowType;
      m_cTmpDim.cArrowDim1 = pAttrs->cArrowDim;
      m_cTmpDim.cArrowDim2 = pAttrs->cArrowDim;
      m_cTmpDim.dFontSize = pAttrs->dFontSize;
      m_cTmpDim.bFontAttrs = pAttrs->bFontAttrs;
      strcpy(m_cTmpDim.psFontFace, pAttrs->sFontFace);
      m_cTmpDim.bSelected = false;

      CDPoint cFirstPt;
      GetNativeRefPoint(m_dFirstDimen, 0.0, &cFirstPt);
      double dDimPixelLen = GetDist(cFirstPt, cPtX.cOrigin)*pAttrs->dScaleDenom;
      if(dDimPixelLen < 3.0) m_iDimenDir = 0;
      else if(m_iDimenDir == 0) m_iDimenDir = GetDimenDir(m_dFirstDimen, cPtX.dRef);

      if(m_iDimenDir != 0)
      {
        m_cTmpDim.iRefDir = m_iDimenDir;
        double dRef3 = GetDimenMidPointRef(m_cTmpDim.dRef1, m_cTmpDim.dRef2, m_iDimenDir);

        CDPoint cPt1, cDir, cNorm;
        GetNativeRefPoint(dRef3, 0.0, &cPt1);
        GetNativeRefDir(dRef3, &cDir);
        cNorm = GetNormal(cDir);
        if(m_iDimenDir < 0) cNorm *= -1.0;
        double dAng = NormalizeAngle(&cNorm);

        m_cTmpDim.cLabelPos.cPoint = cPt1 - pAttrs->dBaseLine*cNorm;
        m_cTmpDim.cLabelPos.dOrientation = dAng;

        switch(m_iType)
        {
        case dtLine:
          strcpy(m_cTmpDim.psLab, pAttrs->sLengthMask);
          break;
        case dtCircle:
          strcpy(m_cTmpDim.psLab, pAttrs->sAngleMask);
          break;
        default:
          strcpy(m_cTmpDim.psLab, "???");
          break;
        }
        AddDimenPrimitive(-1, &m_cTmpDim, plPrimitive, pRect);
      }
    }
  }

  if(iTemp > 1) delete plPrimitive;
  delete pBounds;
  return iRes;
}

/*int CDObject::BuildPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, int iTemp, PDFileAttrs pAttrs)
{
  PDPrimObject plPrimitive = m_pPrimitive;
  if(iTemp > 1) plPrimitive = new CDPrimObject();
  plPrimitive->Clear();
  CDRect cRect = *pRect;
  cRect.cPt1.x -= m_cLineStyle.dWidth;
  cRect.cPt1.y -= m_cLineStyle.dWidth;
  cRect.cPt2.x += m_cLineStyle.dWidth;
  cRect.cPt2.y += m_cLineStyle.dWidth;

  CDPoint cBnds;
  double dExt = m_cLineStyle.dPercent*m_cLineStyle.dWidth/200.0;

  int iRes = BuildPurePrimitives(cTmpPt, iMode, &cRect, iTemp, plPrimitive, dExt, &m_dMovedDist, &cBnds);
  int nCrs = m_pCrossPoints->GetCount();

  if((iTemp < 1) && (iRes > 0) && (m_cLineStyle.iSegments > 0))
  {
    plPrimitive->ClearLines();

    double dStart, dEnd;
    int iStart, iEnd;
    double d1, d2;

    int iClosed = IsClosed();

    if(m_cBounds[0].bIsSet)
    {
      if(!GetPointRefDist(m_cBounds[0].dRef, &d1)) return iRes;

      dStart = d1;
      dEnd = d1;
      iStart = 2;
      iEnd = 0;
      if(m_iType == dtSpline) iEnd = 2;
      else if(iClosed > 0) iEnd = 1;

      int iFirst = 0;
      if(m_cBounds[1].bIsSet)
      {
        if(!GetPointRefDist(m_cBounds[1].dRef, &d2)) return iRes;
        if(d1 > d2) iFirst = m_pCrossPoints->GetIndex(m_cBounds[0].dRef);
        if(iFirst < 0) iFirst = 0;
      }

      if(nCrs > 0)
      {
        iStart = 1;
        GetPointRefDist(m_pCrossPoints->GetPoint(iFirst), &dEnd);
        if(fabs(dEnd - d1) > g_dPrec)
        {
          AddPatSegment(d1, 2, dEnd, 1, &cBnds, pRect);
          dStart = dEnd;
        }

        for(int i = iFirst + 1; i < nCrs; i++)
        {
          dStart = dEnd;
          GetPointRefDist(m_pCrossPoints->GetPoint(i), &dEnd);
          AddPatSegment(dStart, 1, dEnd, 1, &cBnds, pRect);
        }

        if(iFirst > 0)
        {
          for(int i = 0; i < iFirst; i++)
          {
            dStart = dEnd;
            GetPointRefDist(m_pCrossPoints->GetPoint(i), &dEnd);
            AddPatSegment(dStart, 1, dEnd, 1, &cBnds, pRect);
          }
        }
      }

      if(m_cBounds[1].bIsSet)
      {
        if(nCrs > 0)
        {
          if(fabs(dEnd - d2) > g_dPrec)
          AddPatSegment(dEnd, 1, d2, 2, &cBnds, pRect);
        }
        else AddPatSegment(d1, 2, d2, 2, &cBnds, pRect);
      }
      else
      {
        if(iClosed > 0)
        {
          dStart = dEnd;
          dEnd = d1 + cBnds.y - cBnds.x;
          AddPatSegment(dStart, iStart, dEnd, 2, &cBnds, pRect);
        }
        else AddPatSegment(dEnd, iStart, cBnds.y, iEnd, &cBnds, pRect);
      }
    }
    else if(m_cBounds[1].bIsSet)
    {
      if(!GetPointRefDist(m_cBounds[1].dRef, &d2)) return iRes;

      iStart = 0;
      if(m_iType > dtParabola) iStart = 2;

      if(nCrs > 0)
      {
        GetPointRefDist(m_pCrossPoints->GetPoint(0), &dEnd);
        AddPatSegment(cBnds.x, iStart, dEnd, 1, &cBnds, pRect);

        for(int i = 1; i < nCrs; i++)
        {
          dStart = dEnd;
          GetPointRefDist(m_pCrossPoints->GetPoint(i), &dEnd);
          AddPatSegment(dStart, 1, dEnd, 1, &cBnds, pRect);
        }
        iStart = 1;
        if(fabs(dEnd - d2) > g_dPrec)
          AddPatSegment(dEnd, 1, d2, 2, &cBnds, pRect);
      }
      else
      {
        dEnd = d2;
        AddPatSegment(cBnds.x, iStart, dEnd, 2, &cBnds, pRect);
      }
    }
    else if(iClosed > 0)
    {
      if(nCrs > 0)
      {
        GetPointRefDist(m_pCrossPoints->GetPoint(0), &dStart);
        for(int i = 1; i < nCrs; i++)
        {
          GetPointRefDist(m_pCrossPoints->GetPoint(i), &dEnd);
          AddPatSegment(dStart, 1, dEnd, 1, &cBnds, pRect);
          dStart = dEnd;
        }
        GetPointRefDist(m_pCrossPoints->GetPoint(0), &dEnd);
        if(nCrs < 2) dEnd += (cBnds.y - cBnds.x);
      }
      else
      {
        dStart = cBnds.x;
        dEnd = cBnds.y;
      }
      AddPatSegment(dStart, 1, dEnd, 1, &cBnds, pRect);
    }
    else
    {
      iStart = 0;
      iEnd = 0;
      if(m_iType > dtParabola) iStart = 2;
      if(m_iType == dtSpline) iEnd = 2;

      if(nCrs > 0)
      {
        GetPointRefDist(m_pCrossPoints->GetPoint(0), &dEnd);
      }
      else dEnd = 0.0;
      if(dEnd > cBnds.x + g_dPrec)
      {
        AddPatSegment(cBnds.x, iStart, dEnd, 1, &cBnds, pRect);
        iStart = 1;
      }
      for(int i = 1; i < nCrs; i++)
      {
        dStart = dEnd;
        GetPointRefDist(m_pCrossPoints->GetPoint(i), &dEnd);
        AddPatSegment(dStart, 1, dEnd, 1, &cBnds, pRect);
      }
      dStart = dEnd;
      if(nCrs > 0) iStart = 1;
      if(cBnds.y > dStart + g_dPrec)
        AddPatSegment(dStart, iStart, cBnds.y, iEnd, &cBnds, pRect);
    }
  }

  CDPrimitive cPrim;

  if((iTemp < 1) && (iRes > 0))
  {
    cPrim.iType = 6;
    cPrim.cPt1 = 0;
    if(m_cBounds[0].bIsSet)
    {
      if(GetNativeRefPoint(m_cBounds[0].dRef, &cPrim.cPt2)) cPrim.cPt1.x = 1.0;
    }
    if(m_cBounds[1].bIsSet)
    {
      if(GetNativeRefPoint(m_cBounds[1].dRef, &cPrim.cPt3)) cPrim.cPt1.y = 1.0;
    }
    CropPrimitive(cPrim, &cRect, plPrimitive);
  }

  if((iTemp < 1) && (iRes > 0) && (nCrs > 0))
  {
    cPrim.iType = 8;
    for(int i = 0; i < nCrs; i++)
    {
      if(GetNativeRefPoint(m_pCrossPoints->GetPoint(i), &cPrim.cPt1))
        plPrimitive->AddPrimitive(cPrim);
    }
  }

  if((iTemp < 2) && (iRes > 0))
  {
    int nDim = m_pDimens->GetCount();
    PDDimension pDim;
    for(int i = 0; i < nDim; i++)
    {
      pDim = (PDDimension)m_pDimens->GetItem(i);
      AddDimenPrimitive(i, pDim, plPrimitive, &cRect);
    }

    if((iMode == 4) && m_bFirstDimSet && pAttrs)
    {
      CDLine cPtX;
      m_cTmpDim.dRef1 = m_dFirstDimen;
      GetDistFromPt(cTmpPt.cOrigin, cTmpPt.cOrigin, true, &cPtX, NULL);

      m_cTmpDim.dRef2 = cPtX.dRef;
      m_cTmpDim.iArrowType1 = pAttrs->iArrowType;
      m_cTmpDim.iArrowType2 = pAttrs->iArrowType;
      m_cTmpDim.cArrowDim1 = pAttrs->cArrowDim;
      m_cTmpDim.cArrowDim2 = pAttrs->cArrowDim;
      m_cTmpDim.dFontSize = pAttrs->dFontSize;
      m_cTmpDim.bFontAttrs = pAttrs->bFontAttrs;
      strcpy(m_cTmpDim.psFontFace, pAttrs->sFontFace);
      m_cTmpDim.bSelected = false;

      CDPoint cFirstPt;
      GetNativeRefPoint(m_dFirstDimen, &cFirstPt);
      double dDimPixelLen = GetDist(cFirstPt, cPtX.cOrigin)*pAttrs->dScaleDenom;
      if(dDimPixelLen < 3.0) m_iDimenDir = 0;
      else if(m_iDimenDir == 0) m_iDimenDir = GetDimenDir(m_dFirstDimen, cPtX.dRef);

      if(m_iDimenDir != 0)
      {
        m_cTmpDim.iRefDir = m_iDimenDir;
        double dRef3 = GetDimenMidPointRef(m_cTmpDim.dRef1, m_cTmpDim.dRef2, m_iDimenDir);

        CDPoint cPt1, cDir, cNorm;
        GetNativeRefPoint(dRef3, &cPt1);
        GetNativeRefDir(dRef3, &cDir);
        cNorm = GetNormal(cDir);
        if(m_iDimenDir < 0) cNorm *= -1.0;
        double dAng = NormalizeAngle(&cNorm);

        m_cTmpDim.cLabelPos.cPoint = cPt1 - pAttrs->dBaseLine*cNorm;
        m_cTmpDim.cLabelPos.dOrientation = dAng;

        switch(m_iType)
        {
        case dtLine:
          strcpy(m_cTmpDim.psLab, pAttrs->sLengthMask);
          break;
        case dtCircle:
          strcpy(m_cTmpDim.psLab, pAttrs->sAngleMask);
          break;
        default:
          strcpy(m_cTmpDim.psLab, "???");
          break;
        }
        AddDimenPrimitive(-1, &m_cTmpDim, plPrimitive, &cRect);
      }
    }
  }

  if(iTemp > 1) delete plPrimitive;
  return iRes;
}*/

void CDObject::GetFirstPrimitive(PDPrimitive pPrim, double dScale, int iDimen)
{
  m_iPrimitiveRequested = 0;
  GetNextPrimitive(pPrim, dScale, iDimen);
  return;
}

void CDObject::GetNextPrimitive(PDPrimitive pPrim, double dScale, int iDimen)
{
  pPrim->iType = 0;
  int iDimCount = m_pPrimitive->GetCount();
  int i = m_iPrimitiveRequested;
  bool bFound = false;
  CDPrimitive cStPrim;
  int iPos;

  while(!bFound && (i < iDimCount))
  {
    cStPrim = m_pPrimitive->GetPrimitive(i++);
    if(iDimen < -1) bFound = (cStPrim.iType < 9) || (cStPrim.iType == 11);
    else
    {
      if(cStPrim.iType == 9)
      {
        iPos = Round(cStPrim.cPt1.y);
        bFound = (iPos == iDimen);
      }
      else if(cStPrim.iType == 10)
      {
        iPos = Round(cStPrim.cPt2.y);
        bFound = (iPos == iDimen);
      }
    }
  }

  if(bFound)
  {
    pPrim->iType = cStPrim.iType;
    pPrim->cPt3 = cStPrim.cPt3;
    pPrim->cPt4 = cStPrim.cPt4;
    if(cStPrim.iType == 11)
    {
      pPrim->cPt1 = cStPrim.cPt1;
      pPrim->cPt2.x = cStPrim.cPt2.x;
      pPrim->cPt2.y = dScale*cStPrim.cPt2.y;
      pPrim->cPt3 = dScale*cStPrim.cPt3;
    }
    else
    {
      if(cStPrim.iType != 9) pPrim->cPt1 = dScale*cStPrim.cPt1;
      else pPrim->cPt1 = cStPrim.cPt1;
      if(cStPrim.iType != 10) pPrim->cPt2 = dScale*cStPrim.cPt2;
      else pPrim->cPt2 = cStPrim.cPt2;
      if(cStPrim.iType != 2)
      {
        pPrim->cPt3 = dScale*cStPrim.cPt3;
        pPrim->cPt4 = dScale*cStPrim.cPt4;
      }
    }
  }
  m_iPrimitiveRequested = i;
  return;
}

int CDObject::GetAttractors(CDPoint cPt, double dScale, PDPointList pPoints)
{
  if(m_iType == dtSpline)
    return GetSplineAttractors(cPt, m_pCachePoints, dScale, pPoints);

  CDPoint cPts[4];
  int iRes = 0;
  switch(m_iType)
  {
  case dtEllipse:
    iRes = GetEllipseAttractors(cPt, m_pCachePoints, cPts);
    break;
  case dtHyperbola:
    iRes = GetHyperAttractors(cPt, m_pCachePoints, cPts);
    break;
  case dtParabola:
    iRes = GetParabAttractors(cPt, m_pCachePoints, cPts);
    break;
  default:
    break;
  }
  for(int i = 0; i < iRes; i++)
  {
    pPoints->AddPoint(dScale*cPts[i].x, dScale*cPts[i].y, 0);
  }
  return iRes;
}

int CDObject::GetBSplineParts()
{
    int iRes = 0;
    int iCount = m_pPrimitive->GetCount();
    if(iCount < 1) return iRes;

    CDPrimitive cPrim1, cPrim2 = m_pPrimitive->GetPrimitive(0);
    bool b2set = (cPrim2.iType == 4);
    if(b2set) iRes++;

    double dDist;

    for(int i = 1; i < iCount; i++)
    {
        cPrim1 = m_pPrimitive->GetPrimitive(i);
        if(cPrim1.iType != 4)
        {
            b2set = false;
        }
        else if(b2set)
        {
            dDist = GetDist(cPrim2.cPt3, cPrim1.cPt1);
            if(dDist > g_dPrec) iRes++;
            cPrim2 = cPrim1;
        }
        else
        {
            iRes++;
            b2set = true;
            cPrim2 = cPrim1;
        }
    }

    return iRes;
}

bool CDObject::GetBSplines(int iParts, double dScale, int *piCtrls, double **ppdKnots, PDPoint *ppPoints)
{
    int iCount = m_pPrimitive->GetCount();
    int *piPairs = (int*)malloc(2*iParts*sizeof(int));

    int iCurPair = -1;
    CDPrimitive cPrim1, cPrim2 = m_pPrimitive->GetPrimitive(0);
    bool b2set = (cPrim2.iType == 4);
    if(b2set)
    {
        iCurPair = 0;
        piPairs[2*iCurPair] = 0;
        piPairs[2*iCurPair + 1] = -1;
    }

    double dDist;

    for(int i = 1; i < iCount; i++)
    {
        cPrim1 = m_pPrimitive->GetPrimitive(i);
        if(cPrim1.iType != 4)
        {
            if(b2set) piPairs[2*iCurPair + 1] = i;
            b2set = false;
        }
        else if(b2set)
        {
            dDist = GetDist(cPrim2.cPt3, cPrim1.cPt1);
            if(dDist > g_dPrec)
            {
                piPairs[2*iCurPair + 1] = i;
                iCurPair++;
                piPairs[2*iCurPair] = i;
                piPairs[2*iCurPair + 1] = -1;
            }
            cPrim2 = cPrim1;
        }
        else
        {
            iCurPair++;
            piPairs[2*iCurPair] = i;
            piPairs[2*iCurPair + 1] = -1;
            b2set = true;
            cPrim2 = cPrim1;
        }
    }

    CDPoint cPt1, cPt2;
    double dDom, du;
    double *pdt;

    if((IsClosed() > 0) && (iParts == 1))
    {
        if(piPairs[1] < 0) piPairs[1] = iCount;
        piCtrls[0] = piPairs[1] - piPairs[0];
        ppPoints[0] = (PDPoint)malloc(piCtrls[0]*sizeof(CDPoint));
        ppdKnots[0] = (double*)malloc((piCtrls[0] + 4)*sizeof(double));

        pdt = (double*)malloc((piCtrls[0])*sizeof(double));

        cPrim1 = m_pPrimitive->GetPrimitive(piPairs[0]);
        cPt2 = cPrim1.cPt2;

        ppdKnots[0][0] = 0.0;
        ppdKnots[0][1] = 0.0;
        ppdKnots[0][2] = 0.0;

        cPrim1 = m_pPrimitive->GetPrimitive(piPairs[1] - 1);
        cPt2 = cPrim1.cPt2;

        for(int i = piPairs[0]; i < piPairs[1]; i++)
        {
            cPt1 = cPt2;
            cPrim1 = m_pPrimitive->GetPrimitive(i);
            cPt2 = cPrim1.cPt2;
            ppPoints[0][i - piPairs[0]] = dScale*cPt2;

            dDom = GetDist(cPt1, cPt2);
            du = GetDist(cPt1, cPrim1.cPt1);
            pdt[i - piPairs[0]] = du/dDom;
        }
        if(piCtrls[0] > 4)
        {
            for(int j = 1; j < piCtrls[0] - 3; j++)
            {
                du = pdt[j]/(1.0 - pdt[j - 1]*(1 - pdt[j]));
                pdt[j] = du;
            }
        }
        ppdKnots[0][piCtrls[0] - 1] = pdt[piCtrls[0] - 4];
        if(piCtrls[0] > 4)
        {
            for(int j = piCtrls[0] - 5; j >= 0; j--)
            {
                ppdKnots[0][j + 3] = ppdKnots[0][j + 4]*pdt[j];
            }
        }
        ppdKnots[0][piCtrls[0]] = 1.0;
        ppdKnots[0][piCtrls[0] + 1] = 1.0 + ppdKnots[0][3];
        ppdKnots[0][piCtrls[0] + 2] = 1.0 + ppdKnots[0][4];

        ppdKnots[0][0] = ppdKnots[0][piCtrls[0] - 2] - 1.0;
        ppdKnots[0][1] = ppdKnots[0][piCtrls[0] - 1] - 1.0;
//for(int j = 0; j < piCtrls[0] + 3; j++)
//printf("%f\n", ppdKnots[0][j]);

        free(pdt);
        free(piPairs);
        return true;
    }

    if(piPairs[2*iCurPair + 1] < 0) piPairs[2*iCurPair + 1] = iCount;

    for(int i = 0; i < iParts; i++)
    {
        piCtrls[i] = piPairs[2*i + 1] - piPairs[2*i] + 2;
        ppPoints[i] = (PDPoint)malloc(piCtrls[i]*sizeof(CDPoint));
        cPrim1 = m_pPrimitive->GetPrimitive(piPairs[2*i]);
        ppPoints[i][0] = dScale*cPrim1.cPt1;
        cPrim1 = m_pPrimitive->GetPrimitive(piPairs[2*i + 1] - 1);
        ppPoints[i][piCtrls[i] - 1] = dScale*cPrim1.cPt3;

        pdt = (double*)malloc((piCtrls[i] - 3)*sizeof(double));

        ppdKnots[i] = (double*)malloc((piCtrls[i] + 3)*sizeof(double));
        ppdKnots[i][0] = 0.0;
        ppdKnots[i][1] = 0.0;
        ppdKnots[i][2] = 0.0;

        cPrim1 = m_pPrimitive->GetPrimitive(piPairs[2*i]);
        cPt2 = cPrim1.cPt2;

        for(int j = piPairs[2*i]; j < piPairs[2*i + 1]; j++)
        {
            cPt1 = cPt2;
            cPrim1 = m_pPrimitive->GetPrimitive(j);
            cPt2 = cPrim1.cPt2;
            ppPoints[i][j - piPairs[2*i] + 1] = dScale*cPt2;

            if(j > piPairs[2*i])
            {
                dDom = GetDist(cPt1, cPt2);
                du = GetDist(cPt1, cPrim1.cPt1);
                pdt[j - piPairs[2*i] - 1] = du/dDom;
            }
        }
        if(piCtrls[i] > 4)
        {
            for(int j = 1; j < piCtrls[i] - 3; j++)
            {
                du = pdt[j]/(1.0 - pdt[j - 1]*(1 - pdt[j]));
                pdt[j] = du;
            }
        }
        ppdKnots[i][piCtrls[i] - 1] = pdt[piCtrls[i] - 4];
        if(piCtrls[i] > 4)
        {
            for(int j = piCtrls[i] - 5; j >= 0; j--)
            {
                ppdKnots[i][j + 3] = ppdKnots[i][j + 4]*pdt[j];
            }
        }
        ppdKnots[i][piCtrls[i]] = 1.0;
        ppdKnots[i][piCtrls[i] + 1] = ppdKnots[i][piCtrls[i]];
        ppdKnots[i][piCtrls[i] + 2] = ppdKnots[i][piCtrls[i]];
        free(pdt);
    }

    free(piPairs);
    return false;
}

bool CDObject::IsNearPoint(CDPoint cPt, double dTolerance, int *piDimen)
{
  CDLine cPtX;
  cPtX.bIsSet = false;
  double dDist = GetDistFromPt(cPt, cPt, 1, &cPtX, piDimen);

  if(!cPtX.bIsSet) return false;

  return fabs(dDist) < dTolerance;
}

int CDObject::GetRefBounds(PDPoint pPoint)
{
  switch(m_iType)
  {
  case dtLine:
    return 0;
  case dtCircle:
    pPoint->x = -M_PI;
    pPoint->y = M_PI;
    return 2;
  case dtEllipse:
    if(m_pCachePoints->GetCount(0) > 2)
    {
      pPoint->x = -M_PI;
      pPoint->y = M_PI;
    }
    else
    {
      pPoint->x = 0.0;
      pPoint->y = 1.0;
    }
    return 2;
  case dtArcEllipse:
    pPoint->x = -M_PI;
    pPoint->y = M_PI;
    return 2;
  case dtHyperbola:
  case dtParabola:
    return 0;
  case dtSpline:
    pPoint->x = 0.0;
    pPoint->y = (double)GetSplineNumSegments(m_pCachePoints);
    return 2;
  case dtEvolvent:
  case dtLogSpiral:
  case dtArchSpiral:
    pPoint->x = 0.0;
    pPoint->y = -1.0;
    return 1;
  case dtPath:
    pPoint->x = 0.0;
    pPoint->y = GetLength(0.0);
    return 2;
  default:
    return 0;
  }
}

int CDObject::GetDimenDir(double dRef1, double dRef2)
{
  CDPoint refBnds;
  if(GetRefBounds(&refBnds) > 1)
  {
    if((fabs(dRef1 - refBnds.x) < g_dPrec) || (fabs(dRef1 - refBnds.y) < g_dPrec))
    {
      return CmpDbls(dRef2 - refBnds.x, refBnds.y - dRef2);
    }
  }
  return CmpDbls(dRef1, dRef2);
}

double CDObject::GetDimenMidPointRef(double dRef1, double dRef2, int iDir)
{
  double d1, d2, d3;
  GetPointRefDist(dRef1, 0.0, &d1);
  GetPointRefDist(dRef2, 0.0, &d2);
  int i1 = iDir;

  CDPoint refBnds;
  if(GetRefBounds(&refBnds) > 1) i1 = CmpDbls(dRef1, dRef2);

  if(i1 == iDir) d3 = (d1 + d2)/2.0;
  else
  {
    double d4, d5;
    GetPointRefDist(refBnds.x, 0.0, &d4);
    GetPointRefDist(refBnds.y, 0.0, &d5);
    double dTotLen = d5 - d4;
    double dLen = (dTotLen - fabs(d2 - d1))/2.0;
    if(i1 > 0)
    {
      d3 = d1 - dLen;
      if(d3 < refBnds.x) d3 = d2 + dLen;
    }
    else
    {
      d3 = d2 - dLen;
      if(d3 < refBnds.x) d3 = d1 + dLen;
    }
  }

  double dRef;
  GetNativeReference(d3, 0.0, &dRef);
  return dRef;
}

bool CDObject::GetNativeReference(double dDist, double dOffset, double *pdRef)
{
  switch(m_iType)
  {
  case dtLine:
    return GetLineReference(dDist, m_pCachePoints, pdRef);
  case dtCircle:
    return GetCircReference(dDist, dOffset, m_pCachePoints, pdRef);
  case dtEllipse:
    return GetElpsReference(dDist, dOffset, m_pCachePoints, pdRef);
  case dtArcEllipse:
    return GetArcElpsReference(dDist, dOffset, m_pCachePoints, pdRef);
  case dtHyperbola:
    return GetHyperReference(dDist, dOffset, m_pCachePoints, pdRef);
  case dtParabola:
    return GetParabReference(dDist, dOffset, m_pCachePoints, pdRef);
  case dtSpline:
    return GetSplineReference(dDist, dOffset, m_pCachePoints, pdRef);
  case dtEvolvent:
    return GetEvolvReference(dDist, dOffset, m_pCachePoints, pdRef);
  case dtPath:
    *pdRef = dDist;
    return true;
  default:
    return false;
  }
}

int CDObject::GetBounds(PDPoint pBounds, double dOffset, bool bAdvancePeriod)
{
  if(m_iType < dtPath)
  {
    CDPoint cLocBnds = {0.0, 0.0};
    CDPoint cLocRefs = {0.0, 0.0};
    int iRefBounds = GetRefBounds(&cLocBnds);

    if(m_cBounds[0].bIsSet && m_cBounds[1].bIsSet)
    {
      cLocRefs.x = m_cBounds[0].dRef;
      cLocRefs.y = m_cBounds[1].dRef;
      if(!GetPointRefDist(cLocRefs.x, dOffset, &pBounds->x)) return 0;
      if(m_cBounds[1].dRef < m_cBounds[0].dRef - g_dPrec)
      {
        if(iRefBounds < 1) return 1;
        if(bAdvancePeriod) cLocRefs.y += (cLocBnds.y - cLocBnds.x);
      }
      if(!GetPointRefDist(cLocRefs.y, dOffset, &pBounds->y)) return 0;
      return 3;
    }

    if(m_cBounds[0].bIsSet)
    {
      cLocRefs.x = m_cBounds[0].dRef;
      if(!GetPointRefDist(cLocRefs.x, dOffset, &pBounds->x)) return 0;
      if(iRefBounds < 2) return 1;
      if(bAdvancePeriod) cLocRefs.y = m_cBounds[0].dRef + cLocBnds.y - cLocBnds.x;
      if(!GetPointRefDist(cLocRefs.y, dOffset, &pBounds->y)) return 0;
      return 3;
    }

    if(m_cBounds[1].bIsSet)
    {
      cLocRefs.y = m_cBounds[1].dRef;
      if(!GetPointRefDist(cLocRefs.y, dOffset, &pBounds->y)) return 0;
      if(iRefBounds < 1) return 2;
      cLocRefs.x = cLocBnds.x;
      if(!GetPointRefDist(cLocRefs.x, dOffset, &pBounds->x)) return 0;
      return 3;
    }

    if(iRefBounds > 0)
    {
      if(!GetPointRefDist(cLocBnds.x, dOffset, &pBounds->x)) return 0;
      if(iRefBounds < 2) return 1;
      if(!GetPointRefDist(cLocBnds.y, dOffset, &pBounds->y)) return 0;
      return 3;
    }
    return 0;
  }

  if(m_iType == dtPath)
  {
    pBounds->x = 0.0;
    pBounds->y = GetLength(dOffset + m_dMovedDist);
    return 3;
  }
  return 0;
}

int CDObject::GetBoundsRef(PDPoint pBounds, double dOffset, bool bAdvancePeriod)
{
  if(m_iType < dtPath)
  {
    CDPoint cLocBnds = {0.0, 0.0};
    int iRefBounds = GetRefBounds(&cLocBnds);

    if(m_cBounds[0].bIsSet && m_cBounds[1].bIsSet)
    {
      pBounds->x = m_cBounds[0].dRef;
      pBounds->y = m_cBounds[1].dRef;
      if(m_cBounds[1].dRef < m_cBounds[0].dRef - g_dPrec)
      {
        if(iRefBounds < 1) return 1;
        if(bAdvancePeriod) pBounds->y += (cLocBnds.y - cLocBnds.x);
      }
      return 3;
    }

    if(m_cBounds[0].bIsSet)
    {
      pBounds->x = m_cBounds[0].dRef;
      if(iRefBounds < 2) return 1;
      pBounds->y = cLocBnds.y;
      if(bAdvancePeriod) pBounds->y = m_cBounds[0].dRef + cLocBnds.y - cLocBnds.x;
      return 3;
    }

    if(m_cBounds[1].bIsSet)
    {
      pBounds->y = m_cBounds[1].dRef;
      if(iRefBounds < 1) return 2;
      pBounds->x = cLocBnds.x;
      return 3;
    }

    if(iRefBounds > 0)
    {
      pBounds->x = cLocBnds.x;
      if(iRefBounds < 2) return 1;
      pBounds->y = cLocBnds.y;
      return 3;
    }
    return 0;
  }

  if(m_iType == dtPath)
  {
    pBounds->x = 0.0;
    pBounds->y = GetLength(dOffset + m_dMovedDist);
    return 3;
  }
  return 0;
}

double CDObject::GetLength(double dOffset)
{
  if(m_iType < dtPath)
  {
    CDPoint cBounds = {0.0, 0.0};
    if(GetBounds(&cBounds, dOffset, true) > 2) return cBounds.y - cBounds.x;
    return 0.0;
  }

  double dTotLen = 0.0;

  if(m_iType == dtPath)
  {
    int n = m_pSubObjects->GetCount();
    if(n < 1) return 0.0;

    PDPathSeg pSeg;
    for(int i = 0; i < n; i++)
    {
      pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
      if(pSeg->bReverse)
        dTotLen += pSeg->pSegment->GetLength(-dOffset);
      else
        dTotLen += pSeg->pSegment->GetLength(dOffset);
    }
  }
  return dTotLen;
}

PDPathSeg CDObject::GetPathRefSegment(double dRef, double dOffset, double *pdSegRef, int *piPos)
{
  if(piPos) *piPos = -1;
  int n = m_pSubObjects->GetCount();
  if(n < 1) return NULL;

  double dTotLen = GetLength(dOffset);
  bool bClosedPath = IsClosedPath();

  if(dRef < -g_dPrec)
  {
    if(!bClosedPath) return NULL;

    dRef += dTotLen;
    while(dRef < -g_dPrec) dRef += dTotLen;
  }

  if(dRef > dTotLen + g_dPrec)
  {
    if(!bClosedPath) return NULL;

    dRef -= dTotLen;
    while(dRef > dTotLen + g_dPrec) dRef -= dTotLen;
  }

  PDPathSeg pSeg = (PDPathSeg)m_pSubObjects->GetItem(0);
  double dLen = 0.0;
  if(pSeg->bReverse)
    dLen = pSeg->pSegment->GetLength(-dOffset);
  else
    dLen = pSeg->pSegment->GetLength(dOffset);
  int i = 1;

  while((dLen < dRef - g_dPrec) && (i < n))
  {
    dRef -= dLen;
    pSeg = (PDPathSeg)m_pSubObjects->GetItem(i++);
    if(pSeg->bReverse)
      dLen = pSeg->pSegment->GetLength(-dOffset);
    else
      dLen = pSeg->pSegment->GetLength(dOffset);
  }

  if(dRef > dLen + g_dPrec) return NULL;

  while((dLen < g_dPrec) && (i < n))
  {
    pSeg = (PDPathSeg)m_pSubObjects->GetItem(i++);
    if(pSeg->bReverse)
      dLen = pSeg->pSegment->GetLength(-dOffset);
    else
      dLen = pSeg->pSegment->GetLength(dOffset);
  }
  if(dLen < g_dPrec)
  {
    if(!bClosedPath) return NULL;
    i = 0;
    while((dLen < g_dPrec) && (i < n))
    {
      pSeg = (PDPathSeg)m_pSubObjects->GetItem(i++);
      if(pSeg->bReverse)
        dLen = pSeg->pSegment->GetLength(-dOffset);
      else
        dLen = pSeg->pSegment->GetLength(dOffset);
    }
  }
  if(dLen < g_dPrec) return NULL;

  PDObject pObj = pSeg->pSegment;

  CDPoint cBounds;

  if(pSeg->bReverse)
  {
    if(pObj->GetBounds(&cBounds, -dOffset, true) < 3) return NULL;
    if(pObj->GetNativeReference(cBounds.y - dRef, -dOffset, pdSegRef))
    {
      if(piPos) *piPos = i - 1;
      return pSeg;
    }

    return NULL;
  }

  if(pObj->GetBounds(&cBounds, dOffset, true) < 3) return NULL;
  if(pObj->GetNativeReference(cBounds.x + dRef, dOffset, pdSegRef))
  {
    if(piPos) *piPos = i - 1;
    return pSeg;
  }

  return NULL;
}

bool CDObject::GetPathRefPoint(double dRef, double dOffset, PDPoint pPt)
{
  dOffset += m_dMovedDist;
  double d1;
  PDPathSeg pSeg = GetPathRefSegment(dRef, dOffset, &d1, NULL);
  if(!pSeg) return false;

  double dRes = 0.0;
  if(pSeg->bReverse)
    dRes = pSeg->pSegment->GetNativeRefPoint(d1, -dOffset, pPt);
  else
    dRes = pSeg->pSegment->GetNativeRefPoint(d1, dOffset, pPt);
  return dRes;
}

bool CDObject::GetNativeRefPoint(double dRef, double dOffset, PDPoint pPt)
{
  switch(m_iType)
  {
  case dtLine:
    return GetLineRefPoint(dRef, dOffset, m_pCachePoints, pPt);
  case dtCircle:
    return GetCircRefPoint(dRef, dOffset, m_pCachePoints, pPt);
  case dtEllipse:
    return GetElpsRefPoint(dRef, dOffset, m_pCachePoints, pPt);
  case dtArcEllipse:
    return GetArcElpsRefPoint(dRef, dOffset, m_pCachePoints, pPt);
  case dtHyperbola:
    return GetHyperRefPoint(dRef, dOffset, m_pCachePoints, pPt);
  case dtParabola:
    return GetParabRefPoint(dRef, dOffset, m_pCachePoints, pPt);
  case dtSpline:
    return GetSplineRefPoint(dRef, dOffset, m_pCachePoints, pPt);
  case dtEvolvent:
    return GetEvolvRefPoint(dRef, dOffset, m_pCachePoints, pPt);
  case dtPath:
    return GetPathRefPoint(dRef, dOffset, pPt);
  default:
    return false;
  }
}

bool CDObject::GetPathRefDir(double dRef, PDPoint pPt)
{
  double d1;
  PDPathSeg pSeg = GetPathRefSegment(dRef, 0.0, &d1, NULL);
  if(!pSeg) return false;

  bool bRes = pSeg->pSegment->GetNativeRefDir(d1, pPt);
  if(pSeg->bReverse) *pPt *= -1.0;
  return bRes;
}

bool CDObject::GetNativeRefDir(double dRef, PDPoint pPt)
{
  switch(m_iType)
  {
  case dtLine:
    return GetLineRefDir(dRef, m_pCachePoints, pPt);
  case dtCircle:
    return GetCircRefDir(dRef, m_pCachePoints, pPt);
  case dtEllipse:
    return GetElpsRefDir(dRef, m_pCachePoints, pPt);
  case dtArcEllipse:
    return GetArcElpsRefDir(dRef, m_pCachePoints, pPt);
  case dtHyperbola:
    return GetHyperRefDir(dRef, m_pCachePoints, pPt);
  case dtParabola:
    return GetParabRefDir(dRef, m_pCachePoints, pPt);
  case dtSpline:
    return GetSplineRefDir(dRef, m_pCachePoints, pPt);
  case dtEvolvent:
    return GetEvolvRefDir(dRef, m_pCachePoints, pPt);
  case dtPath:
    return GetPathRefDir(dRef, pPt);
  default:
    return false;
  }
}

bool CDObject::IsValidRef(double dRef)
{
  if(m_iType == dtPath)
  {
    if(IsClosedPath()) return true;
    if(dRef < -g_dPrec) return false;
    double dLen = GetLength(0.0);
    if(dRef > dLen + g_dPrec) return false;
    return true;
  }

  if(IsClosedShape())
  {
    if(!m_cBounds[1].bIsSet) return true;
    if(!m_cBounds[0].bIsSet) return true;

    if(m_cBounds[0].dRef > m_cBounds[1].dRef)
    {
      if((m_cBounds[1].dRef < dRef - g_dPrec) && (dRef + g_dPrec < m_cBounds[0].dRef))
        return false;
      return true;
    }
  }

  if(m_cBounds[0].bIsSet)
  {
    if(dRef < m_cBounds[0].dRef - g_dPrec) return false;
  }

  if(m_cBounds[1].bIsSet)
  {
    if(dRef > m_cBounds[1].dRef + g_dPrec) return false;
  }

  return true;
}

bool CDObject::GetSelected()
{
  return m_bSelected;
}

void CDObject::SetSelected(bool bSelect, bool bInvert, int iDimen, PDPtrList pRegions)
{
  PDDimension pDim;
  bool bChanged = false;
  if(bInvert)
  {
    if(iDimen > -1)
    {
      pDim = (PDDimension)m_pDimens->GetItem(iDimen);
      if(pDim->bSelected == bSelect) pDim->bSelected = !bSelect;
      else pDim->bSelected = bSelect;
    }
    else if(m_bSelected == bSelect) m_bSelected = !bSelect;
    else m_bSelected = bSelect;
    bChanged = true;
  }
  else
  {
    if(iDimen > -1)
    {
      pDim = (PDDimension)m_pDimens->GetItem(iDimen);
      bChanged = (pDim->bSelected != bSelect);
      pDim->bSelected = bSelect;
    }
    else
    {
      bChanged = (m_bSelected != bSelect);
      m_bSelected = bSelect;
      for(int i = 0; i < m_pDimens->GetCount(); i++)
      {
        pDim = (PDDimension)m_pDimens->GetItem(i);
        bChanged |= pDim->bSelected;
        pDim->bSelected = false;
      }
    }
  }
  if(bChanged) AddRegions(pRegions, -1);
}

int CDObject::GetType()
{
  return m_iType;
}

CDLine CDObject::GetLine()
{
  CDLine cRes = {false, {0, 0}, {0, 0}};

  if(m_iType != 1) return cRes;

  int iCnt = m_pCachePoints->GetCount(0);
  if(iCnt < 2) return cRes;

  CDInputPoint cInPt1 = m_pCachePoints->GetPoint(0, 0);
  CDInputPoint cInPt2 = m_pCachePoints->GetPoint(1, 0);

  cRes.bIsSet = true;
  cRes.cOrigin = cInPt1.cPoint;
  cRes.cDirection = cInPt2.cPoint;
  return cRes;
}

CDLine CDObject::GetCircle()
{
  CDLine cRes = {false, {0, 0}, {0, 0}};

  if(m_iType != 2) return cRes;

  int iCnt = m_pCachePoints->GetCount(0);
  if(iCnt < 2) return cRes;

  CDInputPoint cInPt1 = m_pCachePoints->GetPoint(0, 0);
  CDInputPoint cInPt2 = m_pCachePoints->GetPoint(1, 0);

  cRes.bIsSet = true;
  cRes.cOrigin = cInPt1.cPoint;
  cRes.cDirection = cInPt2.cPoint;
  return cRes;
}

void CDObject::SetInputLine(int iIndex, CDLine cLine)
{
  m_cLines[iIndex] = cLine;
}

bool CDObject::HasEnoughPoints()
{
  int iInputLines = 0;
  if(m_cLines[0].bIsSet) iInputLines++;
  if(m_cLines[1].bIsSet) iInputLines++;

  switch(m_iType)
  {
  case dtLine:
    return(HasLineEnoughPoints(m_pInputPoints));
  case dtCircle:
    return(HasCircEnoughPoints(m_pInputPoints, iInputLines));
  case dtEllipse:
    return(HasElpsEnoughPoints(m_pInputPoints, iInputLines));
  case dtArcEllipse:
    return(HasArcElpsEnoughPoints(m_pInputPoints, iInputLines));
  case dtHyperbola:
    return(HasHyperEnoughPoints(m_pInputPoints, iInputLines));
  case dtParabola:
    return(HasParabEnoughPoints(m_pInputPoints, iInputLines));
  case dtSpline:
    return(HasSplineEnoughPoints(m_pInputPoints));
  case dtEvolvent:
    return(HasEvolvEnoughPoints(m_pInputPoints, iInputLines));
  default:
    return false;
  }
}

bool CDObject::GetPoint(int iIndex, char iCtrl, PDInputPoint pPoint)
{
  int iCnt = m_pInputPoints->GetCount(iCtrl);
  if(iIndex >= iCnt) return false;
  *pPoint = m_pInputPoints->GetPoint(iIndex, iCtrl);
  return true;
}

void CDObject::SetPoint(int iIndex, char iCtrl, CDInputPoint cPoint)
{
  m_pInputPoints->SetPoint(iIndex, iCtrl, cPoint.cPoint.x, cPoint.cPoint.y, cPoint.iCtrl);
}

bool CDObject::BoundPoint(CDPoint cRefPt, PDLine pPtX, double *pdDist)
{
  CDPoint cDir;

  if(IsClosedShape())
  {
    bool bRes = false;
    if(m_cBounds[0].bIsSet && m_cBounds[1].bIsSet)
    {
      if(m_cBounds[0].dRef > m_cBounds[1].dRef)
      {
        bRes = ((pPtX->dRef > m_cBounds[1].dRef) && (m_cBounds[0].dRef > pPtX->dRef));
      }
      else bRes = ((pPtX->dRef < m_cBounds[0].dRef) || (m_cBounds[1].dRef < pPtX->dRef));
    }
    if(bRes)
    {
      pPtX->dRef = m_cBounds[0].dRef;
      GetNativeRefPoint(m_cBounds[0].dRef, 0.0, &pPtX->cOrigin);
      GetNativeRefDir(m_cBounds[0].dRef, &cDir);
      pPtX->cDirection = GetNormal(cDir);
      *pdDist = GetDist(cRefPt, pPtX->cOrigin);

      CDPoint cPt2;
      GetNativeRefPoint(m_cBounds[1].dRef, 0.0, &cPt2);
      double d1 = GetDist(cRefPt, cPt2);
      if(d1 < *pdDist)
      {
        pPtX->dRef = m_cBounds[1].dRef;
        pPtX->cOrigin = cPt2;
        GetNativeRefDir(m_cBounds[1].dRef, &cDir);
        pPtX->cDirection = GetNormal(cDir);
        *pdDist = d1;
      }
      return true;
    }
    return false;
  }

  if(m_cBounds[0].bIsSet)
  {
    if(pPtX->dRef < m_cBounds[0].dRef)
    {
      pPtX->dRef = m_cBounds[0].dRef;
      GetNativeRefPoint(m_cBounds[0].dRef, 0.0, &pPtX->cOrigin);
      GetNativeRefDir(m_cBounds[0].dRef, &cDir);
      pPtX->cDirection = GetNormal(cDir);
      *pdDist = GetDist(cRefPt, pPtX->cOrigin);
      return true;
    }
  }
  if(m_cBounds[1].bIsSet)
  {
    if(pPtX->dRef > m_cBounds[1].dRef)
    {
      pPtX->dRef = m_cBounds[1].dRef;
      GetNativeRefPoint(m_cBounds[1].dRef, 0.0, &pPtX->cOrigin);
      GetNativeRefDir(m_cBounds[1].dRef, &cDir);
      pPtX->cDirection = GetNormal(cDir);
      *pdDist = GetDist(cRefPt, pPtX->cOrigin);
      return true;
    }
  }
  return false;
}

double CDObject::GetPathDistFromPt(CDPoint cPt, CDPoint cRefPt, bool bSnapCenters, PDLine pPtX)
{
  int n = m_pSubObjects->GetCount();
  if(n < 1) return -1.0;

  double dMinAbs = -1.0;
  double dMin;
  double dCur;
  PDPathSeg pSeg;
  CDLine cMin, cCur;
  int iMin = -1;
  int iSrchMask = 0;
  if(bSnapCenters) iSrchMask = 1;

  for(int i = 0; i < n; i++)
  {
    pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
    dCur = pSeg->pSegment->GetDistFromPt(cPt, cRefPt, iSrchMask, &cCur, NULL);
    if((iMin < 0) || (fabs(dCur) < dMinAbs))
    {
      iMin = i;
      dMinAbs = fabs(dCur);
      cMin = cCur;
      dMin = dCur;
      if(pSeg->bReverse) dMin *= -1.0;
    }
  }

  double dNorm = GetNorm(cMin.cDirection);
  if(dNorm < g_dPrec)
  {
    *pPtX = cMin;
    return dMin;
  }

  *pPtX = cMin;
  pPtX->dRef = 0.0;
  for(int i = 0; i < iMin; i++)
  {
    pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
    pPtX->dRef += pSeg->pSegment->GetLength(0.0);
  }

  pSeg = (PDPathSeg)m_pSubObjects->GetItem(iMin);
  PDObject pObj = pSeg->pSegment;
  double d1;

  pObj->GetPointRefDist(cMin.dRef, 0.0, &d1);
  CDPoint cBounds;
  if(pObj->GetBounds(&cBounds, 0.0, false) > 2)
  {
    if((cBounds.x > cBounds.y) && !pSeg->bReverse)
    {
      CDPoint cBnds2;
      pObj->GetRefBounds(&cBnds2);
      double d2, d3;
      if(!pObj->GetPointRefDist(cBnds2.x, 0.0, &d2)) d2 = 0.0;
      if(!pObj->GetPointRefDist(cBnds2.y, 0.0, &d3)) d3 = 0.0;
      d1 += (d3 - d2);
    }
    if(pSeg->bReverse)
      pPtX->dRef += (cBounds.y - d1);
    else
      pPtX->dRef += (d1 - cBounds.x);
  }

  return dMin;
}

double CDObject::GetOffset()
{
  if(m_iType < dtEllipse) return 0.0;

  int nOffs = m_pCachePoints->GetCount(2);
  if(nOffs < 1) return 0.0;
  return m_pCachePoints->GetPoint(0, 2).cPoint.x;
}

double CDObject::GetDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSearchMask, PDLine pPtX, int *piDimen)
{
  double dRes = -1.0;
  CDLine cPtX;

  switch(m_iType)
  {
  case dtLine:
    dRes = GetLineDistFromPt(cPt, m_pCachePoints, &cPtX);
    break;
  case dtCircle:
    dRes = GetCircDistFromPt(cPt, cRefPt, iSearchMask & 1, m_pCachePoints, &cPtX);
    break;
  case dtEllipse:
    dRes = GetElpsDistFromPt(cPt, cRefPt, iSearchMask, m_pCachePoints, &cPtX);
    break;
  case dtArcEllipse:
    dRes = GetArcElpsDistFromPt(cPt, cRefPt, iSearchMask, m_pCachePoints, &cPtX);
    break;
  case dtHyperbola:
    dRes = GetHyperDistFromPt(cPt, cRefPt, iSearchMask, m_pCachePoints, &cPtX);
    break;
  case dtParabola:
    dRes = GetParabDistFromPt(cPt, cRefPt, iSearchMask, m_pCachePoints, &cPtX);
    break;
  case dtSpline:
    dRes = GetSplineDistFromPt(cPt, cRefPt, iSearchMask, m_pCachePoints, &cPtX);
    break;
  case dtEvolvent:
    dRes = GetEvolvDistFromPt(cPt, cRefPt, m_pCachePoints, &cPtX);
    break;
  case dtLogSpiral:
  case dtArchSpiral:
  break;
  case dtPath:
    dRes = GetPathDistFromPt(cPt, cRefPt, iSearchMask & 1, &cPtX);
    break;
  case dtBorderPath:
  case dtBorder:
  case dtArea:
  case dtGroup:
    break;
  }

  if(piDimen)
  {
    *piDimen = -2;
    PDDimension pDim;
    double dDist1;
    CDPoint cPt1, cPt2;
    for(int i = 0; i < m_pDimens->GetCount(); i++)
    {
      pDim = (PDDimension)m_pDimens->GetItem(i);
      GetNativeRefPoint(pDim->dRef1, 0.0, &cPt1);
      dDist1 = GetDist(cPt, cPt1);
      if(dDist1 < fabs(dRes))
      {
        dRes = dDist1;
        *piDimen = i;
        cPtX.bIsSet = true;
        cPtX.cOrigin = cPt1;
        cPtX.dRef = pDim->dRef1;
        cPtX.cDirection = 0;
      }
      GetNativeRefPoint(pDim->dRef2, 0.0, &cPt1);
      dDist1 = GetDist(cPt, cPt1);
      if(dDist1 < fabs(dRes))
      {
        dRes = dDist1;
        *piDimen = i;
        cPtX.bIsSet = true;
        cPtX.cOrigin = cPt1;
        cPtX.dRef = pDim->dRef2;
        cPtX.cDirection = 0;
      }
      cPt1.x = -sin(pDim->cLabelPos.dOrientation);
      cPt1.y = cos(pDim->cLabelPos.dOrientation);
      cPt2 = Rotate(cPt - pDim->cLabelPos.cPoint, cPt1, false);
      if(DPtInDRect(cPt2, &pDim->cExt))
      {
        dRes = 0.0;
        *piDimen = i;
        cPtX.bIsSet = true;
        cPtX.cOrigin = pDim->cLabelPos.cPoint;
        cPtX.dRef = (pDim->dRef1 + pDim->dRef2)/2.0;
        cPtX.cDirection = 0;
      }
    }
  }

  double dNorm = GetNorm(cPtX.cDirection);
  if((dNorm < g_dPrec) || (m_iType == dtPath))
  {
    if(pPtX) *pPtX = cPtX;
    return dRes;
  }

  if(BoundPoint(cRefPt, &cPtX, &dRes))
  {
    if(pPtX) *pPtX = cPtX;
    return dRes;
  }

  dNorm = GetNorm(cPtX.cDirection);
  if(dNorm < g_dPrec)
  {
    if(pPtX) *pPtX = cPtX;
    return dRes;
  }

  CDPoint cPtTan = GetNormal(cPtX.cDirection);

  CDPoint cPtX2;
  double dRes2 = GetLineProj(cPt, cPtX.cOrigin, cPtTan, &cPtX2);
  if(dRes2 < dRes)
  {
    if(pPtX)
    {
      pPtX->bIsSet = true;
      pPtX->cOrigin = cPtX2;
      pPtX->cDirection = cPtX.cDirection;
      pPtX->dRef = cPtX.dRef;
    }
    return dRes2;
  }

  if(pPtX) *pPtX = cPtX;
  return dRes;
}

CDLineStyle CDObject::GetLineStyle()
{
  return m_cLineStyle;
}

void CDObject::SetLineStyle(int iMask, CDLineStyle cStyle)
{
  //m_cLineStyle = cStyle;
  if(iMask & 1) m_cLineStyle.dWidth = cStyle.dWidth;
  if(iMask & 2) m_cLineStyle.dPercent = cStyle.dPercent;
  if(iMask & 4)
  {
    int n = cStyle.iSegments;
    if(n > 6) n = 6;
    m_cLineStyle.iSegments = n;
    for(int i = 0; i < n; i++) m_cLineStyle.dPattern[i] = cStyle.dPattern[i];
  }
  if(iMask & 8) m_cLineStyle.cCapType = cStyle.cCapType;
  if(iMask & 16) m_cLineStyle.cJoinType = cStyle.cJoinType;
  if(iMask & 32)
  {
    for(int i = 0; i < 4; i++) m_cLineStyle.cColor[i] = cStyle.cColor[i];
  }
}

bool CDObject::GetRestrictPoint(CDPoint cPt, int iMode, bool bRestrictSet, double dRestrictValue,
  PDPoint pSnapPt)
{
  *pSnapPt = cPt;

  if(!bRestrictSet) return false;

  switch(m_iType)
  {
  case dtLine:
    return GetLineRestrictPoint(cPt, iMode, dRestrictValue, pSnapPt, m_pCachePoints);
  case dtCircle:
    return GetCircleRestrictPoint(cPt, iMode, dRestrictValue, pSnapPt, m_pCachePoints,
      m_pInputPoints, m_cLines);
  case dtEllipse:
    return GetElpsRestrictPoint(cPt, iMode, dRestrictValue, pSnapPt, m_pCachePoints);
  case dtArcEllipse:
    return GetArcElpsRestrictPoint(cPt, iMode, dRestrictValue, pSnapPt, m_pCachePoints);
  case dtHyperbola:
    return GetHyperRestrictPoint(cPt, iMode, dRestrictValue, pSnapPt, m_pCachePoints);
  case dtParabola:
    return GetParabRestrictPoint(cPt, iMode, dRestrictValue, pSnapPt, m_pCachePoints);
  case dtSpline:
    return GetSplineRestrictPoint(cPt, iMode, dRestrictValue, pSnapPt, m_pCachePoints);
  case dtEvolvent:
    return GetEvolvRestrictPoint(cPt, iMode, dRestrictValue, pSnapPt, m_pCachePoints);
  default:
    return false;
  }
}

CDObject* CDObject::Copy()
{
  PDObject pRes = new CDObject(m_iType, m_cLineStyle.dWidth);
  pRes->SetInputLine(0, m_cLines[0]);
  pRes->SetInputLine(1, m_cLines[1]);
  CDInputPoint cInPt;

  for(int i = 0; i < m_pInputPoints->GetCount(-1); i++)
  {
    cInPt = m_pInputPoints->GetPoint(i, -1);
    pRes->AddPoint(cInPt.cPoint.x, cInPt.cPoint.y, cInPt.iCtrl, cInPt.cPoint.x);
  }
  pRes->SetBound(0, m_cBounds[0]);
  pRes->SetBound(1, m_cBounds[1]);
  pRes->SetLineStyle(63, m_cLineStyle);

  double dRef;
  for(int i = 0; i < m_pCrossPoints->GetCount(); i++)
  {
    dRef = m_pCrossPoints->GetPoint(i);
    pRes->AddCrossPoint(dRef);
  }

  if(m_iType == dtPath)
  {
    PDPathSeg pSegIn, pSegOut;
    for(int i = 0; i < m_pSubObjects->GetCount(); i++)
    {
      pSegIn = (PDPathSeg)m_pSubObjects->GetItem(i);
      pSegOut = (PDPathSeg)malloc(sizeof(CDPathSeg));
      pSegOut->bReverse = pSegIn->bReverse;
      pSegOut->pSegment = pSegIn->pSegment->Copy();
      pRes->m_pSubObjects->Add(pSegOut);
    }
  }

  CDLine cPtX;
  cPtX.bIsSet = false;
  pRes->BuildCache(cPtX, 0);
  return pRes;
}

bool CDObject::IsClosedPath()
{
  int n = m_pSubObjects->GetCount();
  if(n < 1) return false;
  PDPathSeg pSeg = (PDPathSeg)m_pSubObjects->GetItem(0);
  if(n < 2) return pSeg->pSegment->IsClosedShape();

  CDPoint cPt1, cPt2;
  if(pSeg->bReverse) pSeg->pSegment->GetEndPoint(&cPt1, 0.0);
  else pSeg->pSegment->GetStartPoint(&cPt1, 0.0);

  pSeg = (PDPathSeg)m_pSubObjects->GetItem(n - 1);
  if(pSeg->bReverse) pSeg->pSegment->GetStartPoint(&cPt2, 0.0);
  else pSeg->pSegment->GetEndPoint(&cPt2, 0.0);

  double dJoinTol = 100.0*g_dPrec;
  return GetDist(cPt1, cPt2) < dJoinTol;
}

bool CDObject::IsClosedShape()
{
  switch(m_iType)
  {
  case dtCircle:
    return true;
  case dtEllipse:
    return m_pCachePoints->GetCount(0) > 2;
  case dtArcEllipse:
    return true;
  case dtSpline:
    if(m_pInputPoints->GetCount(1) < 1) return false;
    return true;
  case dtPath:
    return IsClosedPath();
  default:
    return m_iType > dtPath;
  }
}

int CDObject::GetBoundType()
{
  switch(m_iType)
  {
  case dtCircle:
  case dtEllipse:
  case dtArcEllipse:
    return 2;
  case dtSpline:
    if(m_pInputPoints->GetCount(1) < 1) return 3;
    return 2;
  case dtEvolvent:
  case dtLogSpiral:
  case dtArchSpiral:
    return 1;
  case dtPath:
    if(IsClosedPath()) return 2;
    return 3;
  default:
    if(m_iType >= dtPath) return 2;
    return 0;
  }
}

int CDObject::IsClosed()
{
  switch(m_iType)
  {
  case dtCircle:
  case dtEllipse:
  case dtArcEllipse:
    if(m_cBounds[0].bIsSet)
    {
      if(m_cBounds[1].bIsSet) return 0;
      return 1;
    }
    return 2;
  case dtSpline:
    if(m_pInputPoints->GetCount(1) < 1) return 0;
    if(m_cBounds[0].bIsSet)
    {
      if(m_cBounds[1].bIsSet) return 0;
      return 1;
    }
    return 2;
  case dtPath:
    if(IsClosedPath())
    {
      if(m_cBounds[0].bIsSet)
      {
        if(m_cBounds[1].bIsSet) return 0;
        return 1;
      }
      return 2;
    }
    return 0;
  default:
    if(m_iType < dtPath) return 0;
    return 2;
  }
}

void CDObject::SetBound(int iIndex, CDLine cBound)
{
  m_cBounds[iIndex].bIsSet = cBound.bIsSet;
  m_cBounds[iIndex].dRef = cBound.dRef;
  if(IsClosedShape() && !m_cBounds[0].bIsSet && m_cBounds[1].bIsSet)
  {
    m_cBounds[0] = m_cBounds[1];
    m_cBounds[1].bIsSet = false;
  }

  int n = m_pCrossPoints->GetCount();
  double dRef;
  while(n > 0)
  {
    dRef = m_pCrossPoints->GetPoint(--n);
    if(!IsValidRef(dRef)) m_pCrossPoints->Remove(n);
  }
}

void CDObject::SetBound(int iIndex, CDRefPoint cBound)
{
  m_cBounds[iIndex] = cBound;
  if(IsClosedShape() && !m_cBounds[0].bIsSet && m_cBounds[1].bIsSet)
  {
    m_cBounds[0] = m_cBounds[1];
    m_cBounds[1].bIsSet = false;
  }

  int n = m_pCrossPoints->GetCount();
  double dRef;
  while(n > 0)
  {
    dRef = m_pCrossPoints->GetPoint(--n);
    if(!IsValidRef(dRef)) m_pCrossPoints->Remove(n);
  }
}

PDObject CDObject::SplitByRef(double dRef, PDPtrList pRegions)
{
  CDRefPoint cBnd;
  cBnd.bIsSet = true;
  cBnd.dRef = dRef;

  int iClosed = IsClosed();
  if(iClosed == 2)
  {
    SetBound(0, cBnd);
    AddRegions(pRegions, -1);
    return NULL;
  }

  PDObject pNewObj = NULL;

  if(m_cBounds[0].bIsSet)
  {
    if(m_cBounds[1].bIsSet)
    {
      pNewObj = Copy();
      SetBound(1, cBnd);
      pNewObj->SetBound(0, cBnd);
    }
    else
    {
      pNewObj = Copy();
      SetBound(1, cBnd);
      pNewObj->SetBound(0, cBnd);
      if(iClosed > 0) pNewObj->SetBound(1, m_cBounds[0]);
    }
  }
  else
  {
    if(m_cBounds[1].bIsSet)
    {
      pNewObj = Copy();
      SetBound(0, cBnd);
      pNewObj->SetBound(1, cBnd);
    }
    else if(iClosed < 1)
    {
      pNewObj = Copy();
      SetBound(0, cBnd);
      pNewObj->SetBound(1, cBnd);
    }
    else SetBound(0, cBnd);
  }

  PDDimension pDim;
  int n = m_pDimens->GetCount();
  for(int i = n - 1; i >= 0; i--)
  {
    pDim = (PDDimension)m_pDimens->GetItem(i);
    if(m_cBounds[0].bIsSet && m_cBounds[1].bIsSet)
    {
      if(m_cBounds[0].dRef < m_cBounds[1].dRef)
      {
        if(pDim->dRef1 < m_cBounds[0].dRef - g_dPrec)
        {
          m_pDimens->Remove(i);
          if(pDim->dRef2 < m_cBounds[0].dRef + g_dPrec) pNewObj->AddDimenPtr(pDim);
          else free(pDim);
        }
        else if(pDim->dRef2 > m_cBounds[1].dRef + g_dPrec)
        {
          m_pDimens->Remove(i);
          if(pDim->dRef1 > m_cBounds[1].dRef - g_dPrec) pNewObj->AddDimenPtr(pDim);
          else free(pDim);
        }
      }
    }
    else if(m_cBounds[0].bIsSet)
    {
      if(pDim->dRef1 < m_cBounds[0].dRef - g_dPrec)
      {
        m_pDimens->Remove(i);
        if(pDim->dRef2 < m_cBounds[0].dRef + g_dPrec) pNewObj->AddDimenPtr(pDim);
        else free(pDim);
      }
    }
    else if(m_cBounds[1].bIsSet)
    {
      if(pDim->dRef2 > m_cBounds[1].dRef + g_dPrec)
      {
        m_pDimens->Remove(i);
        if(pDim->dRef1 > m_cBounds[1].dRef - g_dPrec) pNewObj->AddDimenPtr(pDim);
        else free(pDim);
      }
    }
  }

  //CDLine cPtX;
  //cPtX.bIsSet = false;
  AddRegions(pRegions, -1);
  //pNewObj->BuildCache(cPtX, 0);
  pNewObj->SetSelected(true, false, -1, pRegions);
  return pNewObj;
}

bool CDObject::Split(CDPoint cPt, PDPtrList pNewObjects, PDRect pRect, PDPtrList pRegions)
{
  // new version should be based on reference list:
  PDRefList pRefs = new CDRefList();
  int iRefs = GetPointReferences(cPt, pRefs);
  pRefs->Sort(0);
  double dRef;
  PDObject pNewObj = NULL;
  for(int i = 0; i < iRefs; i++)
  {
    dRef = pRefs->GetPoint(i);
    if(pNewObj)
    {
      pNewObjects->Add(pNewObj);
      pNewObj = pNewObj->SplitByRef(dRef, pRegions);
    }
    else pNewObj = SplitByRef(dRef, pRegions);
  }
  if(pNewObj) pNewObjects->Add(pNewObj);
  delete pRefs;
  return pNewObjects->GetCount() > 0;
  // ----

  /*
  *ppNewObj = NULL;

  CDLine cPtX, cLn;
  cLn.bIsSet = false;
  double d1 = fabs(GetDistFromPt(cPt, cPt, false, &cPtX, NULL));
  if(d1 > dDist) return false;
  if(!cPtX.bIsSet) return false;

  int iClosed = IsClosed();
  if(iClosed == 2)
  {
    SetBound(0, cPtX);
    BuildPrimitives(cLn, 0, pRect, 0, NULL);
    AddRegions(pRegions, -1);
    return true;
  }

  PDObject pNewObj = NULL;

  if(m_cBounds[0].bIsSet)
  {
    if(m_cBounds[1].bIsSet)
    {
      pNewObj = Copy();
      SetBound(1, cPtX);
      pNewObj->SetBound(0, cPtX);
    }
    else
    {
      pNewObj = Copy();
      SetBound(1, cPtX);
      pNewObj->SetBound(0, cPtX);
      if(iClosed > 0) pNewObj->SetBound(1, m_cBounds[0]);
    }
  }
  else
  {
    if(m_cBounds[1].bIsSet)
    {
      pNewObj = Copy();
      SetBound(0, cPtX);
      pNewObj->SetBound(1, cPtX);
    }
    else if(iClosed < 1)
    {
      pNewObj = Copy();
      SetBound(0, cPtX);
      pNewObj->SetBound(1, cPtX);
    }
    else SetBound(0, cPtX);
  }

  PDDimension pDim;
  int n = m_pDimens->GetCount();
  for(int i = n - 1; i >= 0; i--)
  {
    pDim = (PDDimension)m_pDimens->GetItem(i);
    if(m_cBounds[0].bIsSet && m_cBounds[1].bIsSet)
    {
      if(m_cBounds[0].dRef < m_cBounds[1].dRef)
      {
        if(pDim->dRef1 < m_cBounds[0].dRef - g_dPrec)
        {
          m_pDimens->Remove(i);
          if(pDim->dRef2 < m_cBounds[0].dRef + g_dPrec) pNewObj->AddDimenPtr(pDim);
          else free(pDim);
        }
        else if(pDim->dRef2 > m_cBounds[1].dRef + g_dPrec)
        {
          m_pDimens->Remove(i);
          if(pDim->dRef1 > m_cBounds[1].dRef - g_dPrec) pNewObj->AddDimenPtr(pDim);
          else free(pDim);
        }
      }
    }
    else if(m_cBounds[0].bIsSet)
    {
      if(pDim->dRef1 < m_cBounds[0].dRef - g_dPrec)
      {
        m_pDimens->Remove(i);
        if(pDim->dRef2 < m_cBounds[0].dRef + g_dPrec) pNewObj->AddDimenPtr(pDim);
        else free(pDim);
      }
    }
    else if(m_cBounds[1].bIsSet)
    {
      if(pDim->dRef2 > m_cBounds[1].dRef + g_dPrec)
      {
        m_pDimens->Remove(i);
        if(pDim->dRef1 > m_cBounds[1].dRef - g_dPrec) pNewObj->AddDimenPtr(pDim);
        else free(pDim);
      }
    }
  }

  cPtX.bIsSet = false;
  BuildPrimitives(cLn, 0, pRect, 0, NULL);
  AddRegions(pRegions, -1);
  pNewObj->BuildCache(cPtX, 0);
  pNewObj->BuildPrimitives(cLn, 0, pRect, 0, NULL);
  pNewObj->SetSelected(true, false, -1, pRegions);

  *ppNewObj = pNewObj;
  return true;*/
}

bool CDObject::Extend(CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions)
{
  double d1;
  bool bRes = false;
  CDPoint cPt1;
  if(m_cBounds[0].bIsSet)
  {
    GetNativeRefPoint(m_cBounds[0].dRef, 0.0, &cPt1);
    d1 = GetDist(cPt1, cPt);
    if(d1 < dDist)
    {
      m_cBounds[0].bIsSet = false;
      bRes = true;
    }
  }
  if(m_cBounds[1].bIsSet)
  {
    GetNativeRefPoint(m_cBounds[1].dRef, 0.0, &cPt1);
    d1 = GetDist(cPt1, cPt);
    if(d1 < dDist)
    {
      m_cBounds[1].bIsSet = false;
      bRes = true;
    }
  }
  if(bRes)
  {
    if(IsClosedShape() && !m_cBounds[0].bIsSet && m_cBounds[1].bIsSet)
    {
      m_cBounds[0] = m_cBounds[1];
      m_cBounds[1].bIsSet = false;
    }
    CDLine cLn;
    cLn.bIsSet = false;
    BuildPrimitives(cLn, 0, pRect, 0, NULL);
    AddRegions(pRegions, -1);
  }
  return bRes;
}

void CDObject::SavePoint(FILE *pf, bool bSwapBytes, CDPoint cPoint)
{
  unsigned char buf[16], *pbuf;

  pbuf = (unsigned char*)&cPoint.x;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  pbuf = (unsigned char*)&cPoint.y;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);
}

void CDObject::SaveInputPoint(FILE *pf, bool bSwapBytes, CDInputPoint cInPoint)
{
  unsigned char buf = cInPoint.iCtrl;
  fwrite(&buf, 1, 1, pf);
  SavePoint(pf, bSwapBytes, cInPoint.cPoint);
}

void CDObject::SaveReference(FILE *pf, bool bSwapBytes, double dRef)
{
  unsigned char buf[16], *pbuf;
  pbuf = (unsigned char*)&dRef;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);
}

void CDObject::SaveRefPoint(FILE *pf, bool bSwapBytes, CDRefPoint cRefPoint)
{
  unsigned char buf = cRefPoint.bIsSet;
  fwrite(&buf, 1, 1, pf);
  SaveReference(pf, bSwapBytes, cRefPoint.dRef);
}

void CDObject::SaveLine(FILE *pf, bool bSwapBytes, CDLine cLine)
{
  unsigned char buf[16], *pbuf;

  buf[0] = cLine.bIsSet;
  fwrite(buf, 1, 1, pf);
  SavePoint(pf, bSwapBytes, cLine.cOrigin);
  SavePoint(pf, bSwapBytes, cLine.cDirection);

  pbuf = (unsigned char*)&cLine.dRef;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);
}

void CDObject::SaveLineStyle(FILE *pf, bool bSwapBytes, CDLineStyle cLineStyle, unsigned char cVersion)
{
  unsigned char buf[16], *pbuf;

  pbuf = (unsigned char*)&cLineStyle.dWidth;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  pbuf = (unsigned char*)&cLineStyle.dPercent;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  unsigned long ulVal = cLineStyle.iSegments;
  pbuf = (unsigned char*)&ulVal;
  SwapBytes(buf, pbuf, 4, bSwapBytes);
  fwrite(buf, 1, 4, pf);

  for(int i = 0; i < 6; i++)
  {
    pbuf = (unsigned char*)&cLineStyle.dPattern[i];
    SwapBytes(buf, pbuf, 8, bSwapBytes);
    fwrite(buf, 1, 8, pf);
  }

  if(cVersion > 1)
  {
    buf[0] = cLineStyle.cCapType;
    buf[1] = cLineStyle.cJoinType;
    fwrite(buf, 1, 2, pf);
    fwrite(cLineStyle.cColor, 1, 4, pf);
  }
}

void CDObject::SaveDimension(FILE *pf, bool bSwapBytes, PDDimension pDim, unsigned char cVersion)
{
  unsigned char buf[16], *pbuf;

  pbuf = (unsigned char*)&pDim->dRef1;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  pbuf = (unsigned char*)&pDim->dRef2;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  if(cVersion > 1)
  {
    buf[0] = (unsigned char)pDim->iRefDir;
    fwrite(buf, 1, 1, pf);
  }

  buf[0] = (unsigned char)pDim->iArrowType1;
  fwrite(buf, 1, 1, pf);

  buf[0] = (unsigned char)pDim->iArrowType2;
  fwrite(buf, 1, 1, pf);

  SavePoint(pf, bSwapBytes, pDim->cArrowDim1);
  SavePoint(pf, bSwapBytes, pDim->cArrowDim2);

  pbuf = (unsigned char*)&pDim->dFontSize;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  buf[0] = pDim->bFontAttrs;
  fwrite(buf, 1, 1, pf);

  buf[0] = (unsigned char)strlen(pDim->psFontFace);
  fwrite(buf, 1, 1, pf);
  fwrite(pDim->psFontFace, 1, buf[0], pf);

  SavePoint(pf, bSwapBytes, pDim->cLabelPos.cPoint);

  pbuf = (unsigned char*)&pDim->cLabelPos.dOrientation;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  unsigned long ulVal = 0;
  if(pDim->psLab) ulVal = strlen(pDim->psLab);
  pbuf = (unsigned char*)&ulVal;
  SwapBytes(buf, pbuf, 4, bSwapBytes);
  fwrite(buf, 1, 4, pf);
  if(pDim->psLab) fwrite(pDim->psLab, 1, ulVal, pf);
}

void CDObject::SaveToFile(FILE *pf, bool bSwapBytes, unsigned char cVersion)
{
  unsigned char buf[16], *pbuf;
  unsigned long lCnt;

  if((cVersion > 1) || (m_iType < dtPath))
  {
    buf[0] = m_iType;
    fwrite(buf, 1, 1, pf);

    SaveLine(pf, bSwapBytes, m_cLines[0]);
    SaveLine(pf, bSwapBytes, m_cLines[1]);
    if((cVersion == 1) && ((m_iType == dtHyperbola)))
    {
      // for backward compatibility, we have changed hyperbola direction to be compatible with paths
      CDRefPoint cLocBnds[2];
      cLocBnds[0] = m_cBounds[1];
      cLocBnds[1] = m_cBounds[0];
      if(cLocBnds[0].bIsSet) cLocBnds[0].dRef *= -1.0;
      if(cLocBnds[1].bIsSet) cLocBnds[1].dRef *= -1.0;
      SaveRefPoint(pf, bSwapBytes, cLocBnds[0]);
      SaveRefPoint(pf, bSwapBytes, cLocBnds[1]);
    }
    else
    {
      SaveRefPoint(pf, bSwapBytes, m_cBounds[0]);
      SaveRefPoint(pf, bSwapBytes, m_cBounds[1]);
    }

    SaveLineStyle(pf, bSwapBytes, m_cLineStyle, cVersion);

    lCnt = m_pInputPoints->GetCount(-1);
    pbuf = (unsigned char*)&lCnt;
    SwapBytes(buf, pbuf, 4, bSwapBytes);
    fwrite(buf, 1, 4, pf);

    CDInputPoint cInPt;
    for(unsigned int i = 0; i < lCnt; i++)
    {
      cInPt = m_pInputPoints->GetPoint(i, -1);
      SaveInputPoint(pf, bSwapBytes, cInPt);
    }

    lCnt = m_pCrossPoints->GetCount();
    pbuf = (unsigned char*)&lCnt;
    SwapBytes(buf, pbuf, 4, bSwapBytes);
    fwrite(buf, 1, 4, pf);

    double dRef;
    for(unsigned int i = 0; i < lCnt; i++)
    {
      dRef = m_pCrossPoints->GetPoint(i);
      SaveReference(pf, bSwapBytes, dRef);
    }

    lCnt = m_pDimens->GetCount();
    pbuf = (unsigned char*)&lCnt;
    SwapBytes(buf, pbuf, 4, bSwapBytes);
    fwrite(buf, 1, 4, pf);

    PDDimension pDim;
    for(unsigned int i = 0; i < lCnt; i++)
    {
      pDim = (PDDimension)m_pDimens->GetItem(i);
      SaveDimension(pf, bSwapBytes, pDim, cVersion);
    }
  }

  lCnt = m_pSubObjects->GetCount();
  PDObject pObj;
  PDPathSeg pSeg;

  if(cVersion > 1)
  {
    pbuf = (unsigned char*)&lCnt;
    SwapBytes(buf, pbuf, 4, bSwapBytes);
    fwrite(buf, 1, 4, pf);

    if(m_iType < dtBorder)
    {
      for(unsigned int i = 0; i < lCnt; i++)
      {
        pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
        buf[0] = (unsigned char)pSeg->bReverse;
        fwrite(buf, 1, 1, pf);
        pSeg->pSegment->SaveToFile(pf, bSwapBytes, cVersion);
      }
    }
    else
    {
      for(unsigned int i = 0; i < lCnt; i++)
      {
        pObj = (PDObject)m_pSubObjects->GetItem(i);
        pObj->SaveToFile(pf, bSwapBytes, cVersion);
      }
    }
  }
  else if(m_iType < dtBorder)
  {
    for(unsigned int i = 0; i < lCnt; i++)
    {
      pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
      buf[0] = (unsigned char)pSeg->bReverse;
      fwrite(buf, 1, 1, pf);
      pSeg->pSegment->SaveToFile(pf, bSwapBytes, cVersion);
    }
  }
}

void CDObject::LoadPoint(FILE *pf, bool bSwapBytes, PDPoint pPoint)
{
  unsigned char buf[16], *pbuf;

  pPoint->x = 0.0;
  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)&pPoint->x;
  SwapBytes(pbuf, buf, 8, bSwapBytes);

  pPoint->y = 0.0;
  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)&pPoint->y;
  SwapBytes(pbuf, buf, 8, bSwapBytes);
}

void CDObject::LoadInputPoint(FILE *pf, bool bSwapBytes, PDInputPoint pInPoint)
{
  unsigned char buf;
  fread(&buf, 1, 1, pf);
  pInPoint->iCtrl = buf;
  LoadPoint(pf, bSwapBytes, &pInPoint->cPoint);
}

void CDObject::LoadReference(FILE *pf, bool bSwapBytes, double *pdRef)
{
  unsigned char buf[16], *pbuf;

  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)pdRef;
  SwapBytes(pbuf, buf, 8, bSwapBytes);
}

void CDObject::LoadRefPoint(FILE *pf, bool bSwapBytes, PDRefPoint pRefPoint)
{
  unsigned char buf;
  fread(&buf, 1, 1, pf);
  pRefPoint->bIsSet = buf;
  pRefPoint->dRef = 0.0;
  LoadReference(pf, bSwapBytes, &pRefPoint->dRef);
}

void CDObject::LoadLine(FILE *pf, bool bSwapBytes, PDLine pLine)
{
  unsigned char buf[16], *pbuf;

  fread(buf, 1, 1, pf);
  pLine->bIsSet = buf[0];
  pLine->cOrigin.x = 0.0;
  pLine->cOrigin.y = 0.0;
  LoadPoint(pf, bSwapBytes, &pLine->cOrigin);
  pLine->cDirection.x = 0.0;
  pLine->cDirection.y = 0.0;
  LoadPoint(pf, bSwapBytes, &pLine->cDirection);

  pLine->dRef = 0.0;
  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)&pLine->dRef;
  SwapBytes(pbuf, buf, 8, bSwapBytes);
}

void CDObject::LoadLineStyle(FILE *pf, bool bSwapBytes, PDLineStyle pLineStyle, unsigned char cVersion)
{
  unsigned char buf[16], *pbuf;

  pLineStyle->dWidth = 0.0;
  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)&pLineStyle->dWidth;
  SwapBytes(pbuf, buf, 8, bSwapBytes);

  pLineStyle->dPercent = 0.0;
  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)&pLineStyle->dPercent;
  SwapBytes(pbuf, buf, 8, bSwapBytes);

  fread(buf, 1, 4, pf);
  unsigned long ulVal = 0;
  pbuf = (unsigned char*)&ulVal;
  SwapBytes(pbuf, buf, 4, bSwapBytes);
  pLineStyle->iSegments = ulVal;

  for(int i = 0; i < 6; i++)
  {
    pLineStyle->dPattern[i] = 0.0;
    fread(buf, 1, 8, pf);
    pbuf = (unsigned char*)&pLineStyle->dPattern[i];
    SwapBytes(pbuf, buf, 8, bSwapBytes);
  }

  if(cVersion > 1)
  {
    fread(buf, 1, 2, pf);
    pLineStyle->cCapType = buf[0];
    pLineStyle->cJoinType = buf[1];
    fread(pLineStyle->cColor, 1, 4, pf);
  }
  else
  {
    pLineStyle->cCapType = 1;
    pLineStyle->cJoinType = 1;
    pLineStyle->cColor[0] = 0;
    pLineStyle->cColor[1] = 0;
    pLineStyle->cColor[2] = 0;
    pLineStyle->cColor[3] = 255;
  }
}

void CDObject::LoadDimension(FILE *pf, bool bSwapBytes, PDDimension pDim, unsigned char cVersion)
{
  unsigned char buf[16], *pbuf;

  pDim->dRef1 = 0.0;
  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)&pDim->dRef1;
  SwapBytes(pbuf, buf, 8, bSwapBytes);

  pDim->dRef2 = 0.0;
  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)&pDim->dRef2;
  SwapBytes(pbuf, buf, 8, bSwapBytes);

  if(cVersion > 1)
  {
    fread(buf, 1, 1, pf);
    pDim->iRefDir = buf[0];
  }
  else
  {
    if(pDim->dRef1 > pDim->dRef2 + g_dPrec)
    {
      double d1 = pDim->dRef1;
      pDim->dRef1 = pDim->dRef2;
      pDim->dRef2 = d1;
    }
    pDim->iRefDir = 1;
  }

  fread(buf, 1, 1, pf);
  pDim->iArrowType1 = buf[0];

  fread(buf, 1, 1, pf);
  pDim->iArrowType2 = buf[0];

  pDim->cArrowDim1.x = 0.0;
  pDim->cArrowDim1.y = 0.0;
  LoadPoint(pf, bSwapBytes, &pDim->cArrowDim1);
  pDim->cArrowDim2.x = 0.0;
  pDim->cArrowDim2.y = 0.0;
  LoadPoint(pf, bSwapBytes, &pDim->cArrowDim2);

  pDim->dFontSize = 0.0;
  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)&pDim->dFontSize;
  SwapBytes(pbuf, buf, 8, bSwapBytes);

  fread(buf, 1, 1, pf);
  pDim->bFontAttrs = buf[0];

  fread(buf, 1, 1, pf);
  fread(pDim->psFontFace, 1, buf[0], pf);
  pDim->psFontFace[buf[0]] = 0;

  pDim->cLabelPos.cPoint.x = 0.0;
  pDim->cLabelPos.cPoint.y = 0.0;
  LoadPoint(pf, bSwapBytes, &pDim->cLabelPos.cPoint);

  pDim->cLabelPos.dOrientation = 0.0;
  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)&pDim->cLabelPos.dOrientation;
  SwapBytes(pbuf, buf, 8, bSwapBytes);

  unsigned long ulVal = 0;
  fread(buf, 1, 4, pf);
  pbuf = (unsigned char*)&ulVal;
  SwapBytes(pbuf, buf, 4, bSwapBytes);

  pDim->psLab = NULL;
  if(ulVal > 0)
  {
    pDim->psLab = (char*)malloc((ulVal + 1)*sizeof(char));
    fread(pDim->psLab, 1, ulVal, pf);
    pDim->psLab[ulVal] = 0;
  }

  pDim->bSelected = false;
  pDim->cExt.cPt1 = 0;
  pDim->cExt.cPt2 = 0;
}

bool CDObject::ReadFromFile(FILE *pf, bool bSwapBytes, unsigned char cVersion)
{
  unsigned char buf[16], *pbuf;

  LoadLine(pf, bSwapBytes, &m_cLines[0]);
  LoadLine(pf, bSwapBytes, &m_cLines[1]);
  if((cVersion == 1) && ((m_iType == dtHyperbola)))
  {
    CDRefPoint cLocBnds[2];
    LoadRefPoint(pf, bSwapBytes, &cLocBnds[0]);
    LoadRefPoint(pf, bSwapBytes, &cLocBnds[1]);
    m_cBounds[0] = cLocBnds[1];
    m_cBounds[1] = cLocBnds[0];
    if(m_cBounds[0].bIsSet) m_cBounds[0].dRef *= -1.0;
    if(m_cBounds[1].bIsSet) m_cBounds[1].dRef *= -1.0;
  }
  else
  {
    LoadRefPoint(pf, bSwapBytes, &m_cBounds[0]);
    LoadRefPoint(pf, bSwapBytes, &m_cBounds[1]);
  }

  LoadLineStyle(pf, bSwapBytes, &m_cLineStyle, cVersion);

  unsigned long lCnt = 0;
  fread(buf, 1, 4, pf);
  pbuf = (unsigned char*)&lCnt;
  SwapBytes(pbuf, buf, 4, bSwapBytes);

  CDInputPoint cInPt = {0, {0.0, 0.0}};
  for(unsigned int i = 0; i < lCnt; i++)
  {
    LoadInputPoint(pf, bSwapBytes, &cInPt);
    m_pInputPoints->AddPoint(cInPt.cPoint.x, cInPt.cPoint.y, cInPt.iCtrl);
  }

  fread(buf, 1, 4, pf);
  pbuf = (unsigned char*)&lCnt;
  SwapBytes(pbuf, buf, 4, bSwapBytes);

  double dRef = 0.0;
  for(unsigned int i = 0; i < lCnt; i++)
  {
    LoadReference(pf, bSwapBytes, &dRef);
    m_pCrossPoints->AddPoint(dRef);
  }

  fread(buf, 1, 4, pf);
  pbuf = (unsigned char*)&lCnt;
  SwapBytes(pbuf, buf, 4, bSwapBytes);

  PDDimension pDim;
  for(unsigned int i = 0; i < lCnt; i++)
  {
    pDim = (PDDimension)malloc(sizeof(CDDimension));
    LoadDimension(pf, bSwapBytes, pDim, cVersion);
    m_pDimens->Add(pDim);
  }

  if(cVersion > 1)
  {
    CDLine cPtX;
    cPtX.bIsSet = false;
    PDObject pObj;
    PDPathSeg pSeg;
    fread(buf, 1, 4, pf);
    pbuf = (unsigned char*)&lCnt;
    SwapBytes(pbuf, buf, 4, bSwapBytes);
    if(m_iType < dtBorder)
    {
      for(unsigned int i = 0; i < lCnt; i++)
      {
        pSeg = (PDPathSeg)malloc(sizeof(CDPathSeg));
        fread(buf, 1, 1, pf);
        pSeg->bReverse = (bool)buf[0];
        fread(buf, 1, 1, pf);
        pObj = new CDObject((CDDrawType)buf[0], 0.2);
        pObj->ReadFromFile(pf, bSwapBytes, cVersion);
        pSeg->pSegment = pObj;
        if(pObj->BuildCache(cPtX, 0)) m_pSubObjects->Add(pSeg);
        else
        {
          delete pObj;
          free(pSeg);
        }
      }
    }
    else
    {
      for(unsigned int i = 0; i < lCnt; i++)
      {
        fread(buf, 1, 1, pf);
        pObj = new CDObject((CDDrawType)buf[0], 0.2);
        pObj->ReadFromFile(pf, bSwapBytes, cVersion);
        if(pObj->BuildCache(cPtX, 0)) m_pSubObjects->Add(pObj);
        else delete pObj;
      }
    }
  }
  return true;
}

double CDObject::GetRadiusAtPt(CDLine cPtX, PDLine pPtR, bool bNewPt)
{
  switch(m_iType)
  {
  case dtLine:
    return GetLineRadiusAtPt(cPtX.cOrigin, m_pCachePoints, pPtR, bNewPt);
  case dtCircle:
    return GetCircRadiusAtPt(cPtX.cOrigin, m_pCachePoints, pPtR, bNewPt);
  case dtEllipse:
    return GetElpsRadiusAtPt(cPtX.cOrigin, m_pCachePoints, pPtR, bNewPt, m_pInputPoints, m_cLines);
  case dtArcEllipse:
    return GetArcElpsRadiusAtPt(cPtX.cOrigin, m_pCachePoints, pPtR, bNewPt, m_pInputPoints, m_cLines);
  case dtHyperbola:
    return GetHyperRadiusAtPt(cPtX.cOrigin, m_pCachePoints, pPtR, bNewPt);
  case dtParabola:
    return GetParabRadiusAtPt(cPtX.cOrigin, m_pCachePoints, pPtR, bNewPt);
  case dtSpline:
    return GetSplineRadiusAtPt(cPtX, m_pCachePoints, pPtR, bNewPt);
  case dtEvolvent:
    return GetEvolvRadiusAtPt(cPtX, m_pCachePoints, pPtR, bNewPt);
  default:
    return -1.0;
  }
}

bool CDObject::GetDynValue(CDPoint cPt, int iMode, double *pdVal)
{
  bool bRes = false;
  if(iMode == 2)
  {
    *pdVal = m_dMovedDist;
    bRes = true;
  }
  else
  {
    switch(m_iType)
    {
    case dtLine:
      bRes = GetLineAngle(m_pCachePoints, pdVal);
      break;
    case dtCircle:
      bRes = GetCirceRad(m_pCachePoints, pdVal);
      break;
    default:
      break;
    }
  }
  return bRes;
}

void CDObject::BuildRound(CDObject *pObj1, CDObject *pObj2, CDPoint cPt, bool bRestSet, double dRad)
{
  m_pInputPoints->ClearAll();

  CDLine cPtX1, cPtX2;
  double d1 = pObj1->GetDistFromPt(cPt, cPt, 0, &cPtX1, NULL);
  double d2 = pObj2->GetDistFromPt(cPt, cPt, 0, &cPtX2, NULL);

  double dr = fabs(d1);
  if(bRestSet) dr = dRad;
  else if(fabs(d2) > dr) dr = fabs(d2);

  int i = 0;
  int iMaxIter = 16;
  bool bFound = (fabs(fabs(d1) - dr) < g_dPrec) && (fabs(fabs(d2) - dr) < g_dPrec);
  int iX;
  CDPoint cPt1, cPt2, cX = cPt;
  CDPoint cDir1, cDir2;

  while(!bFound && (i < iMaxIter))
  {
    i++;
    if(cPtX1.bIsSet && cPtX2.bIsSet)
    {
      cDir1 = GetNormal(cPtX1.cDirection);
      cDir2 = GetNormal(cPtX2.cDirection);
      if(d1 > 0) cPt1 = cPtX1.cOrigin + dr*cPtX1.cDirection;
      else cPt1 = cPtX1.cOrigin - dr*cPtX1.cDirection;
      if(d2 > 0) cPt2 = cPtX2.cOrigin + dr*cPtX2.cDirection;
      else cPt2 = cPtX2.cOrigin - dr*cPtX2.cDirection;
      iX = LineXLine(cPt1, cDir1, cPt2, cDir2, &cX);

      if(iX > 0)
      {
        d1 = pObj1->GetDistFromPt(cX, cX, 0, &cPtX1, NULL);
        d2 = pObj2->GetDistFromPt(cX, cX, 0, &cPtX2, NULL);
        bFound = (fabs(fabs(d1) - dr) < g_dPrec) && (fabs(fabs(d2) - dr) < g_dPrec);
      }
      else i = iMaxIter;
    }
    else i = iMaxIter;
  }

  if(bFound)
  {
    m_pInputPoints->AddPoint(cX.x, cX.y, 1);
    m_pInputPoints->AddPoint(cX.x + dr, cX.y, 0);

    double dDist;
    CDLine cPtX;
    BuildCircCache(cPtX, 0, m_pInputPoints, m_pCachePoints, m_cLines, &dDist);

    cPt2 = cPtX1.cOrigin;
    GetCircDistFromPt(cPt2, cPt2, false, m_pCachePoints, &cPtX1);
    cPt2 = cPtX2.cOrigin;
    GetCircDistFromPt(cPt2, cPt2, false, m_pCachePoints, &cPtX2);

    if(d1 < 0) cDir1 *= -1.0;
    if(d2 < 0) cDir2 *= -1.0;
    cPt1 = Rotate(cDir2, cDir1, false);
    if(cPt1.y < 0)
    {
      SetBound(0, cPtX2);
      SetBound(1, cPtX1);
    }
    else
    {
      SetBound(0, cPtX1);
      SetBound(1, cPtX2);
    }
  }
}

void RotateLine(PDLine pLine, CDPoint cOrig, CDPoint cRot)
{
  CDPoint cDir;
  double dNorm;
  if(pLine->bIsSet)
  {
    cDir = pLine->cOrigin - cOrig;
    dNorm = GetNorm(cDir);
    if(dNorm > g_dPrec) pLine->cOrigin = cOrig + Rotate(dNorm*cRot, cDir/dNorm, true);
    pLine->cDirection = Rotate(cRot, pLine->cDirection, true);
  }
}

bool CDObject::RotatePoints(CDPoint cOrig, double dRot, int iDimFlag)
{
  bool bRes = false;
  CDLine cPtX;

  BuildCache(cPtX, 0);

  int iCnt;
  PDDimension pDim;
  if(iDimFlag == 1)
  {
    iCnt = m_pDimens->GetCount();
    for(int i = 0; i < iCnt; i++)
    {
      pDim = (PDDimension)m_pDimens->GetItem(i);
      if(pDim->bSelected)
      {
        pDim->cLabelPos.dOrientation += dRot;
        bRes = true;
      }
    }
    BuildCache(cPtX, 0);
    return bRes;
  }

  CDPoint bPt1, bPt2;
  bool b1 = m_cBounds[0].bIsSet;
  bool b2 = m_cBounds[1].bIsSet;

  m_cBounds[0].bIsSet = false;
  m_cBounds[1].bIsSet = false;

  if(b1) GetNativeRefPoint(m_cBounds[0].dRef, 0.0, &bPt1);
  if(b2) GetNativeRefPoint(m_cBounds[1].dRef, 0.0, &bPt2);

  iCnt = m_pInputPoints->GetCount(-1);
  CDPoint cDir, cPt1, cPt2;
  double dNorm;
  CDInputPoint cInPt;
  cPt1.x = cos(dRot);
  cPt1.y = sin(dRot);
  for(int i = 0; i < iCnt; i++)
  {
    cInPt = m_pInputPoints->GetPoint(i, -1);
    cDir = cInPt.cPoint - cOrig;
    dNorm = GetNorm(cDir);
    if(dNorm > g_dPrec)
    {
      cPt2 = cOrig + Rotate(dNorm*cPt1, cDir/dNorm, true);
      m_pInputPoints->SetPoint(i, -1, cPt2.x, cPt2.y, cInPt.iCtrl);
    }
  }
  RotateLine(&m_cLines[0], cOrig, cPt1);
  RotateLine(&m_cLines[1], cOrig, cPt1);

  BuildCache(cPtX, 0);

  if(b1)
  {
    cDir = bPt1 - cOrig;
    dNorm = GetNorm(cDir);
    if(dNorm > g_dPrec) cPt2 = cOrig + Rotate(dNorm*cPt1, cDir/dNorm, true);
    else cPt2 = bPt1;

    GetDistFromPt(cPt2, cPt2, 0, &cPtX, NULL);
    m_cBounds[0].dRef = cPtX.dRef;
  }

  if(b2)
  {
    cDir = bPt2 - cOrig;
    dNorm = GetNorm(cDir);
    if(dNorm > g_dPrec) cPt2 = cOrig + Rotate(dNorm*cPt1, cDir/dNorm, true);
    else cPt2 = bPt2;

    GetDistFromPt(cPt2, cPt2, 0, &cPtX, NULL);
    m_cBounds[1].dRef = cPtX.dRef;
  }

  m_cBounds[0].bIsSet = b1;
  m_cBounds[1].bIsSet = b2;

  if(iDimFlag > 1)
  {
    iCnt = m_pDimens->GetCount();
    for(int i = 0; i < iCnt; i++)
    {
      pDim = (PDDimension)m_pDimens->GetItem(i);
      pDim->cLabelPos.dOrientation += dRot;
    }
  }
  return true;
}

bool CDObject::MovePoints(CDPoint cDir, double dDist, int iDimFlag)
{
  bool bRes = false;
  CDPoint dx = dDist*cDir;
  int iCnt;
  PDDimension pDim;
  CDLine cPtX;
  if(iDimFlag == 1)
  {
    iCnt = m_pDimens->GetCount();
    for(int i = 0; i < iCnt; i++)
    {
      pDim = (PDDimension)m_pDimens->GetItem(i);
      if(pDim->bSelected)
      {
        pDim->cLabelPos.cPoint += dx;
        bRes = true;
      }
    }
    BuildCache(cPtX, 0);
    return bRes;
  }

  iCnt = m_pInputPoints->GetCount(-1);
  CDInputPoint cInPt;
  for(int i = 0; i < iCnt; i++)
  {
    cInPt = m_pInputPoints->GetPoint(i, -1);
    m_pInputPoints->SetPoint(i, -1, cInPt.cPoint.x + dx.x, cInPt.cPoint.y + dx.y, cInPt.iCtrl);
  }
  if(m_cLines[0].bIsSet) m_cLines[0].cOrigin += dx;
  if(m_cLines[1].bIsSet) m_cLines[1].cOrigin += dx;
  if(iDimFlag > 1)
  {
    iCnt = m_pDimens->GetCount();
    for(int i = 0; i < iCnt; i++)
    {
      pDim = (PDDimension)m_pDimens->GetItem(i);
      pDim->cLabelPos.cPoint += dx;
    }
  }
  BuildCache(cPtX, 0);
  return true;
}

void MirrorLine(PDLine pLine, CDLine cLine)
{
  CDPoint cPt1;
  if(pLine->bIsSet)
  {
    pLine->cOrigin = Mirror(pLine->cOrigin, cLine);
    cPt1 = Rotate(pLine->cDirection, cLine.cDirection, false);
    cPt1.y *= -1.0;
    pLine->cDirection = Rotate(cPt1, cLine.cDirection, true);
  }
}

void CDObject::SwapBounds()
{
  CDRefPoint cLn1 = m_cBounds[0];
  m_cBounds[0] = m_cBounds[1];
  m_cBounds[1] = cLn1;
}

void CDObject::MirrorPoints(CDLine cLine)
{
  CDLine cPtX;

  BuildCache(cPtX, 0);

  CDPoint bPt1, bPt2;
  bool b1 = m_cBounds[0].bIsSet;
  bool b2 = m_cBounds[1].bIsSet;

  m_cBounds[0].bIsSet = false;
  m_cBounds[1].bIsSet = false;

  if(b1) GetNativeRefPoint(m_cBounds[0].dRef, 0.0, &bPt1);
  if(b2) GetNativeRefPoint(m_cBounds[1].dRef, 0.0, &bPt2);

  int iCnt = m_pInputPoints->GetCount(-1);
  CDInputPoint cInPt;
  CDPoint cPt1;
  for(int i = 0; i < iCnt; i++)
  {
    cInPt = m_pInputPoints->GetPoint(i, -1);
    cPt1 = Mirror(cInPt.cPoint, cLine);
    m_pInputPoints->SetPoint(i, -1, cPt1.x, cPt1.y, cInPt.iCtrl);
  }
  MirrorLine(&m_cLines[0], cLine);
  MirrorLine(&m_cLines[1], cLine);
  m_pCrossPoints->Clear();
  BuildCache(cPtX, 0);

  if(b1)
  {
    cPt1 = Mirror(bPt1, cLine);
    GetDistFromPt(cPt1, cPt1, 0, &cPtX, NULL);
    m_cBounds[0].dRef = cPtX.dRef;
  }

  if(b2)
  {
    cPt1 = Mirror(bPt2, cLine);
    GetDistFromPt(cPt1, cPt1, 0, &cPtX, NULL);
    m_cBounds[1].dRef = cPtX.dRef;
  }

  m_cBounds[0].bIsSet = b1;
  m_cBounds[1].bIsSet = b2;

  if((m_iType == 1) && (b1 || b2))
  {
    cInPt = m_pInputPoints->GetPoint(0, -1);
    if(cInPt.iCtrl == 1) SwapBounds();
  }

  if(((m_iType != 1) && (m_iType != 7)) ||
    (b1 && b2 && (m_cBounds[1].dRef < m_cBounds[0].dRef)))
  {
    SwapBounds();
  }
}

int CDObject::GetPointReferences(CDPoint cPt, PDRefList pRefs)
{
  double dblDist = 2.0*g_dPrec;
  CDPoint cPt1;

  double dRefs[8];
  int iLen = 0;

  if(m_iType == dtEllipse) iLen = GetElpsSnapPoints(m_pCachePoints, dRefs);
  else if(m_iType == dtArcEllipse) iLen = GetArcElpsSnapPoints(m_pCachePoints, dRefs);
  else if(m_iType == dtHyperbola) iLen = GetHyperSnapPoints(m_pCachePoints, dRefs);
  else if(m_iType == dtParabola) iLen = GetParabSnapPoints(m_pCachePoints, dRefs);
  else if(m_iType == dtEvolvent) iLen = GetEvolvSnapPoints(m_pCachePoints, dRefs);

  if(iLen > 0)
  {
    for(int i = 0; i < iLen; i++)
    {
      if(IsValidRef(dRefs[i]))
      {
        GetNativeRefPoint(dRefs[i], 0.0, &cPt1);
        if(GetDist(cPt, cPt1) < dblDist) pRefs->AddPoint(dRefs[i]);
      }
    }
    int iCnt = pRefs->GetCount();
    if(iCnt > 0) return iCnt;
  }

  if(m_iType == dtSpline)
  {
    double dt;
    PDRefList pRefs1 = new CDRefList();
    iLen = GetSplineSnapPoints(cPt, dblDist, m_pCachePoints, pRefs1);
    for(int i = 0; i < iLen; i++)
    {
      dt = pRefs1->GetPoint(i);
      if(IsValidRef(dt)) pRefs->AddPoint(dt);
    }
    delete pRefs1;
    int iCnt = pRefs->GetCount();
    if(iCnt > 0) return iCnt;
  }

  /*if(m_iType == dtPath)
  {
    PDPtrList pList = new CDPtrList();
    PDPathSeg pSeg;
    for(int i = 0; i < m_pSubObjects->GetCount(); i++)
    {
      pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
      pList->Add(pSeg->pSegment);
    }
    int iRes = GetSnapPointFromList(iSnapMask, cPt, dDist, pSnapPt, pDynObj,
      (PDObject*)pList->GetList(), pList->GetCount(), false);
    delete pList;
    if(iRes > 1)
    {
      CDLine cLnRef;
      GetPathDistFromPt(pSnapPt->cOrigin, pSnapPt->cOrigin, false, &cLnRef);
      pSnapPt->dRef = cLnRef.dRef;
      return iRes;
    }
  }*/

  CDLine cPtX;
  double d1 = GetDistFromPt(cPt, cPt, 0, &cPtX, NULL);
  if(!cPtX.bIsSet || fabs(d1) > dblDist) return 0;
  if(GetNorm(cPtX.cDirection) < 0.5) return 0;
  pRefs->AddPoint(cPtX.dRef);
  return pRefs->GetCount();
}

bool CDObject::AddCrossPoint(CDPoint cPt, double dDist)
{
  PDRefList pRefs = new CDRefList();
  int iRefs = GetPointReferences(cPt, pRefs);
  double dRef, dRef2;
  int j;
  for(int i = 0; i < iRefs; i++)
  {
    dRef = pRefs->GetPoint(i);
    j = m_pCrossPoints->GetIndex(dRef);

    if(j < 0) m_pCrossPoints->AddPoint(dRef);
    else
    {
      dRef2 = m_pCrossPoints->GetPoint(j);
      if(dRef > dRef2 - 0.001) m_pCrossPoints->Remove(j);
      else m_pCrossPoints->InsertPoint(j, dRef);
    }
  }
  delete pRefs;

  return iRefs > 0;
}

bool CDObject::AddCrossPoint(double dRef)
{
  m_pCrossPoints->AddPoint(dRef);
  return true;
}

bool CDObject::GetPointRefDist(double dRef, double dOffset, double *pdDist)
{
  switch(m_iType)
  {
  case dtLine:
    return GetLinePointRefDist(dRef, m_pCachePoints, pdDist);
  case dtCircle:
    return GetCircPointRefDist(dRef, dOffset, m_pCachePoints, pdDist);
  case dtEllipse:
    return GetElpsPointRefDist(dRef, dOffset, m_pCachePoints, pdDist);
  case dtArcEllipse:
    return GetArcElpsPointRefDist(dRef, dOffset, m_pCachePoints, pdDist);
  case dtHyperbola:
    return GetHyperPointRefDist(dRef, dOffset, m_pCachePoints, pdDist);
  case dtParabola:
    return GetParabPointRefDist(dRef, dOffset, m_pCachePoints, pdDist);
  case dtSpline:
    return GetSplinePointRefDist(dRef, dOffset, m_pCachePoints, pdDist);
  case dtEvolvent:
    return GetEvolvPointRefDist(dRef, dOffset, m_pCachePoints, pdDist);
  case dtPath:
    *pdDist = dRef;
    return true;
  default:
    return false;
  }
}

double CDObject::GetNearestCrossPoint(CDPoint cPt, PDPoint pPt)
{
  double d1, d2;
  int iCnt = m_pCrossPoints->GetCount();
  if(iCnt < 1) return -1.0;

  CDPoint cPt1;
  GetNativeRefPoint(m_pCrossPoints->GetPoint(0), 0.0, &cPt1);
  d1 = GetDist(cPt1, cPt);
  *pPt = cPt1;

  for(int i = 1; i < iCnt; i++)
  {
    GetNativeRefPoint(m_pCrossPoints->GetPoint(i), 0.0, &cPt1);
    d2 = GetDist(cPt1, cPt);
    if(d2 < d1)
    {
      d1 = d2;
      *pPt = cPt1;
    }
  }
  return d1;
}

double CDObject::GetNearestBoundPoint(CDPoint cPt, PDPoint pPt)
{
  CDPoint cPt1;
  double dRes = -1.0;
  if(m_cBounds[0].bIsSet)
  {
    GetNativeRefPoint(m_cBounds[0].dRef, 0.0, &cPt1);
    *pPt = cPt1;
    dRes = GetDist(cPt, cPt1);
  }

  if(m_cBounds[1].bIsSet)
  {
    GetNativeRefPoint(m_cBounds[1].dRef, 0.0, &cPt1);
    double d1 = GetDist(cPt, cPt1);
    if((dRes < -0.5) || (d1 < dRes))
    {
      dRes = d1;
      *pPt = cPt1;
    }
  }

  if(!IsClosedShape())
  {
    CDPoint cRefBnds;
    int iBoundType = GetRefBounds(&cRefBnds);
    if(iBoundType > 0)
    {
      GetNativeRefPoint(cRefBnds.x, 0.0, &cPt1);
      double d1 = GetDist(cPt, cPt1);
      if((dRes < -0.5) || (d1 < dRes))
      {
        dRes = d1;
        *pPt = cPt1;
      }
    }
    if(iBoundType > 1)
    {
      GetNativeRefPoint(cRefBnds.y, 0.0, &cPt1);
      double d1 = GetDist(cPt, cPt1);
      if((dRes < -0.5) || (d1 < dRes))
      {
        dRes = d1;
        *pPt = cPt1;
      }
    }
  }

  return dRes;
}

bool CDObject::AddDimen(CDPoint cPt, double dDist, PDRect pRect, PDFileAttrs pAttrs)
{
  CDLine cPtX;
  double d1 = GetDistFromPt(cPt, cPt, 1, &cPtX, NULL);
  if(d1 > dDist) return false;

  if(!m_bFirstDimSet)
  {
    m_bFirstDimSet = true;
    m_dFirstDimen = cPtX.dRef;
    m_iDimenDir = 0;
    return false;
  }

  if(m_iDimenDir == 0) return false;

  PDDimension pDim = (PDDimension)malloc(sizeof(CDDimension));
  pDim->dRef1 = m_dFirstDimen;
  pDim->dRef2 = cPtX.dRef;
  pDim->iRefDir = m_iDimenDir;
  pDim->iArrowType1 = pAttrs->iArrowType;
  pDim->iArrowType2 = pAttrs->iArrowType;
  pDim->cArrowDim1 = pAttrs->cArrowDim;
  pDim->cArrowDim2 = pAttrs->cArrowDim;
  pDim->dFontSize = pAttrs->dFontSize;
  pDim->bFontAttrs = pAttrs->bFontAttrs;
  pDim->cExt.cPt1 = 0;
  pDim->cExt.cPt2 = 0;
  pDim->bSelected = false;
  strcpy(pDim->psFontFace, pAttrs->sFontFace);

  double dRef3 = GetDimenMidPointRef(m_cTmpDim.dRef1, m_cTmpDim.dRef2, m_iDimenDir);

  CDPoint cPt1, cDir, cNorm;
  GetNativeRefPoint(dRef3, 0.0, &cPt1);
  GetNativeRefDir(dRef3, &cDir);
  cNorm = GetNormal(cDir);
  if(m_iDimenDir < 0) cNorm *= -1.0;
  double dAng = NormalizeAngle(&cNorm);

  pDim->cLabelPos.cPoint = cPt1 - pAttrs->dBaseLine*cNorm;
  pDim->cLabelPos.dOrientation = dAng;

  char sBuf[64];
  switch(m_iType)
  {
  case dtLine:
    strcpy(sBuf, pAttrs->sLengthMask);
    break;
  case dtCircle:
    strcpy(sBuf, pAttrs->sAngleMask);
    break;
  default:
    strcpy(sBuf, "???");
  }
  int iLen = strlen(sBuf) + 1;
  pDim->psLab = (char*)malloc(iLen*sizeof(char));
  strcpy(pDim->psLab, sBuf);

  m_pDimens->Add(pDim);
  m_bFirstDimSet = false;
  m_iDimenDir = 0;
  return true;
}

void CDObject::DiscardDimen()
{
  if(m_bFirstDimSet)
  {
    m_bFirstDimSet = false;
    m_iDimenDir = 0;
  }
}

void CDObject::AddDimenPtr(PDDimension pDimen)
{
  m_pDimens->Add(pDimen);
}

int CDObject::GetDimenCount()
{
  return m_pDimens->GetCount();
}

PDDimension CDObject::GetDimen(int iPos)
{
  if(iPos < -1) return NULL;
  if(iPos < 0) return &m_cTmpDim;
  return (PDDimension)m_pDimens->GetItem(iPos);
}

int CDObject::PreParseDimText(int iPos, char *sBuf, int iBufLen, double dScale, PDUnitList pUnits)
{
  sBuf[0] = 0;
  PDDimension pDim;
  if(iPos < 0) pDim = &m_cTmpDim;
  else pDim = (PDDimension)m_pDimens->GetItem(iPos);

  if(!pDim->psLab)
  {
    if(iBufLen < 1) return 1;
    *sBuf = 0;
    return 0;
  }

  const char *s1 = GetEscapeOpening(pDim->psLab);
  if(!s1)
  {
    int iLen = GetPlainMaskLen(pDim->psLab) + 1;
    if(iBufLen < iLen) return iLen;

    CopyPlainMask(sBuf, pDim->psLab);
    return 0;
  }

  PDUnit pUnit = GetUnitAtBuf(s1 + 1, pUnits);
  if(!pUnit) return -1;

  double dVal = -1.0;
  if(pUnit->iUnitType == 1)
  {
    double d1, d2;
    GetPointRefDist(pDim->dRef1, 0.0, &d1);
    GetPointRefDist(pDim->dRef2, 0.0, &d2);
    dVal = fabs(d2 - d1);
    if(*s1 == '[') dVal /= dScale;
  }
  else if(pUnit->iUnitType == 2)
  {
    CDPoint cDir1, cDir2, cPt1;
    GetNativeRefDir(pDim->dRef1, &cDir1);
    GetNativeRefDir(pDim->dRef2, &cDir2);
    cPt1 = Rotate(cDir1, cDir2, false);
    dVal = 180.0*atan2(cPt1.y, cPt1.x)/M_PI;
    if(pDim->iRefDir > 0) dVal *= -1.0;
    if(dVal < -g_dPrec) dVal += 360.0;
    else if(dVal < g_dPrec) dVal = 0.0;
  }
  else return -1;

  return PreParseValue(pDim->psLab, pUnits, dVal, dScale, sBuf, iBufLen);
}

int CDObject::GetUnitMask(int iUnitType, char *psBuf, PDUnitList pUnits)
{
  PDDimension pDim = NULL;
  int iRes = 0;
  int iLen;

  bool bFound = false;
  int i = 0;
  while(!bFound && (i < m_pDimens->GetCount()))
  {
    pDim = (PDDimension)m_pDimens->GetItem(i++);
    if(pDim->psLab)
    {
      iLen = strlen(pDim->psLab);
      if(iLen > 0)
      {
        if(*pDim->psLab == '*') iLen--;
      }
      if((iLen > 0) && (iLen < 64))
      {
        iRes = GuessMaskUnitType(pDim->psLab, pUnits);
        bFound = (iRes == iUnitType);
      }
    }
  }

  if(bFound && pDim)
  {
    if(*pDim->psLab == '*') strcpy(psBuf, &pDim->psLab[1]);
    else strcpy(psBuf, pDim->psLab);
    return 0;
  }

  return -1;
}

bool CDObject::ChangeUnitMask(int iUnitType, char *psMask, PDUnitList pUnits)
{
  PDDimension pDim = NULL;
  int iMaskLen = strlen(psMask);
  bool bStar;
  int iLen, iNewLen, iRes;
  bool bRes = false;

  for(int i = 0; i < m_pDimens->GetCount(); i++)
  {
    pDim = (PDDimension)m_pDimens->GetItem(i);
    if(pDim->psLab)
    {
      iLen = strlen(pDim->psLab);
      if(iLen > 0)
      {
        bStar = (*pDim->psLab == '*');
        if(bStar) iLen--;
      }
      if((iLen > 0) && (iLen < 64))
      {
        iRes = GuessMaskUnitType(pDim->psLab, pUnits);
        if(iRes == iUnitType)
        {
          free(pDim->psLab);
          pDim->psLab = NULL;
          iNewLen = iMaskLen;
          if(bStar) iNewLen++;
          if(iNewLen > 0)
          {
            pDim->psLab = (char*)malloc((iNewLen + 1)*sizeof(char));
            if(bStar)
            {
              *pDim->psLab = '*';
              if(iMaskLen > 0) strcpy(&pDim->psLab[1], psMask);
              else pDim->psLab[1] = 0;
            }
            else strcpy(pDim->psLab, psMask);
          }
          bRes = true;
        }
      }
    }
  }
  return bRes;
}

void CDObject::Rescale(double dRatio, bool bWidths, bool bPatterns, bool bArrows, bool bLabels)
{
  if(m_cLines[0].bIsSet) m_cLines[0].cOrigin *= dRatio;
  if(m_cLines[1].bIsSet) m_cLines[1].cOrigin *= dRatio;

  bool bBnds[2];
  bBnds[0] = m_cBounds[0].bIsSet;
  bBnds[1] = m_cBounds[1].bIsSet;

  CDPoint cBnds[2];
  if(bBnds[0]) GetNativeRefPoint(m_cBounds[0].dRef, 0.0, &cBnds[0]);
  if(bBnds[1]) GetNativeRefPoint(m_cBounds[1].dRef, 0.0, &cBnds[1]);
  m_cBounds[0].bIsSet = false;
  m_cBounds[1].bIsSet = false;

  CDPoint cPt;
  PDPointList pCrossPoints = new CDPointList();
  for(int i = 0; i < m_pCrossPoints->GetCount(); i++)
  {
    GetNativeRefPoint(m_pCrossPoints->GetPoint(i), 0.0, &cPt);
    pCrossPoints->AddPoint(cPt.x, cPt.y, 0);
  }

  PDDimension pDim;
  PDPointList pDimens = new CDPointList();
  for(int i = 0; i < m_pDimens->GetCount(); i++)
  {
    pDim = (PDDimension)m_pDimens->GetItem(i);
    GetNativeRefPoint(pDim->dRef1, 0.0, &cPt);
    pDimens->AddPoint(cPt.x, cPt.y, 0);
    GetNativeRefPoint(pDim->dRef2, 0.0, &cPt);
    pDimens->AddPoint(cPt.x, cPt.y, 0);
  }

  m_pUndoPoints->ClearAll();
  m_pCachePoints->ClearAll();

  CDInputPoint cInPt;
  for(int i = 0; i < m_pInputPoints->GetCount(-1); i++)
  {
    cInPt = m_pInputPoints->GetPoint(i, -1);
    cInPt.cPoint *= dRatio;
    m_pInputPoints->SetPoint(i, -1, cInPt.cPoint.x, cInPt.cPoint.y, cInPt.iCtrl);
  }

  CDLine cLine;
  BuildCache(cLine, 0);

  for(int i = 0; i < m_pDimens->GetCount(); i++)
  {
    pDim = (PDDimension)m_pDimens->GetItem(i);

    cInPt = pDimens->GetPoint(2*i, -1);
    cInPt.cPoint *= dRatio;
    GetDistFromPt(cInPt.cPoint, cInPt.cPoint, 0, &cLine, NULL);
    pDim->dRef1 = cLine.dRef;

    cInPt = pDimens->GetPoint(2*i + 1, -1);
    cInPt.cPoint *= dRatio;
    GetDistFromPt(cInPt.cPoint, cInPt.cPoint, 0, &cLine, NULL);
    pDim->dRef2 = cLine.dRef;

    pDim->cLabelPos.cPoint *= dRatio;
    if(bArrows)
    {
      pDim->cArrowDim1 *= dRatio;
      pDim->cArrowDim2 *= dRatio;
    }
    if(bLabels) pDim->dFontSize *= dRatio;
  }
  delete pDimens;

  m_pCrossPoints->Clear();
  for(int i = 0; i < pCrossPoints->GetCount(-1); i++)
  {
    cInPt = pCrossPoints->GetPoint(i, -1);
    cInPt.cPoint *= dRatio;
    GetDistFromPt(cInPt.cPoint, cInPt.cPoint, 0, &cLine, NULL);
    m_pCrossPoints->AddPoint(cLine.dRef);
  }
  delete pCrossPoints;

  if(bBnds[0])
  {
    cBnds[0] *= dRatio;
    GetDistFromPt(cBnds[0], cBnds[0], 0, &cLine, NULL);
    m_cBounds[0].dRef = cLine.dRef;
    m_cBounds[0].bIsSet = true;
  }
  if(bBnds[1])
  {
    cBnds[1] *= dRatio;
    GetDistFromPt(cBnds[1], cBnds[1], 0, &cLine, NULL);
    m_cBounds[1].dRef = cLine.dRef;
    m_cBounds[1].bIsSet = true;
  }

  if(bWidths) m_cLineStyle.dWidth *= dRatio;

  if(bPatterns)
  {
    for(int i = 0; i < m_cLineStyle.iSegments; i++) m_cLineStyle.dPattern[i] *= dRatio;
    for(int i = m_cLineStyle.iSegments; i < 6; i++) m_cLineStyle.dPattern[i] = 0.0;
  }
}

CDPoint CDObject::GetPointToDir(CDPoint cPoint, double dAngle, CDPoint cPtTarget)
{
  CDPoint cRes = cPoint;

  int iIter = 4, i = 0;
  CDLine cPtX;
  CDPoint cMainDir, cDir1, cDir2, cPt1;

  CDLine cRad;
  double dd = GetDistFromPt(cPtTarget, cPtTarget, 0, &cPtX, NULL);
  double dr = GetRadiusAtPt(cPtX, &cRad, false);
  double dta = tan(dAngle);
  double dta2 = 1.0 + Power2(dta);
  double dv, du, dtf, df;
  double dd1;
  double dPi2Prec = M_PI/2.0 + g_dPrec;

  if(cRad.bIsSet)
  {
    dv = (sqrt(Power2(dr) + dta2*(Power2(dd) + 2.0*dr*dd)) - dr)/dta2;
    du = dv*dta;
    dtf = du/(dr + dv);

    df = atan(dtf);
    if(fabs(dAngle) < dPi2Prec) df *= -1.0;
    cDir1.x = cos(df);
    cDir1.y = sin(df);
    cDir2 = Rotate(cDir1, cRad.cDirection, true);
    cRes = cRad.cOrigin + dr*cDir2;
  }
  else
  {
    GetNativeRefDir(cPtX.dRef, &cMainDir);
    cDir1.x = cos(dAngle);
    cDir1.y = sin(dAngle);
    cRad.cDirection = GetNormal(cMainDir);
    cDir2 = Rotate(cDir1, cRad.cDirection, true);
    LineXLine(cRes, cMainDir, cPtTarget, cDir2, &cPt1);
    cRes = cPt1;
  }

  GetDistFromPt(cRes, cRes, 0, &cPtX, NULL);

  if(cPtX.bIsSet)
  {
    cDir1 = cPtTarget - cPtX.cOrigin;
    cDir2 = Rotate(cDir1, cRad.cDirection, false);
    df = atan2(cDir2.y, cDir2.x);
  }
  else i = iIter;

  while((i < iIter) && (fabs(df - dAngle) > g_dPrec))
  {
    dr = GetRadiusAtPt(cPtX, &cRad, false);

    if(cRad.bIsSet)
    {
      cMainDir = cPtTarget - cRad.cOrigin;
      dd1 = GetNorm(cMainDir);
      cRad.cDirection = cMainDir/dd1;
      dd = dd1 - dr;
      dv = (sqrt(Power2(dr) + dta2*(Power2(dd) + 2.0*dr*dd)) - dr)/dta2;
      du = dv*dta;
      dtf = du/(dr + dv);

      df = atan(dtf);
      if(fabs(dAngle) < dPi2Prec) df *= -1.0;
      cDir1.x = cos(df);
      cDir1.y = sin(df);
      cDir2 = Rotate(cDir1, cRad.cDirection, true);
      cRes = cRad.cOrigin + dr*cDir2;
    }
    else
    {
      GetNativeRefDir(cPtX.dRef, &cMainDir);
      cDir1.x = cos(dAngle);
      cDir1.y = sin(dAngle);
      cRad.cDirection = GetNormal(cMainDir);
      cDir2 = Rotate(cDir1, cRad.cDirection, true);
      LineXLine(cRes, cMainDir, cPtTarget, cDir2, &cPt1);
      cRes = cPt1;
    }

    GetDistFromPt(cRes, cRes, 0, &cPtX, NULL);

    if(cPtX.bIsSet)
    {
      cDir1 = cPtTarget - cPtX.cOrigin;
      cDir2 = Rotate(cDir1, cRad.cDirection, false);
      df = atan2(cDir2.y, cDir2.x);
      i++;
    }
    else i = iIter;
  }

  return cPtX.cOrigin;
}

void CDObject::GetDimFontAttrs(int iPos, PDFileAttrs pAttrs)
{
  if(iPos < 0) return;

  PDDimension pDim = (PDDimension)m_pDimens->GetItem(iPos);
  pAttrs->dFontSize = pDim->dFontSize;
  pAttrs->bFontAttrs = pDim->bFontAttrs;
  strcpy(pAttrs->sFontFace, pDim->psFontFace);
}

bool CDObject::DeleteSelDimens(PDRect pRect, PDPtrList pRegions)
{
  bool bRes = false;
  PDDimension pDim;
  int i = m_pDimens->GetCount();
  while(i > 0)
  {
    pDim = (PDDimension)m_pDimens->GetItem(--i);
    if(pDim->bSelected)
    {
      bRes = true;
      if(pDim->psLab) free(pDim->psLab);
      free(pDim);
      m_pDimens->Remove(i);
      AddRegions(pRegions, 9);
      AddRegions(pRegions, 10);
    }
  }
  if(bRes && pRect)
  {
    CDLine cLn;
    cLn.bIsSet = false;
    BuildPrimitives(cLn, 0, pRect, 0, NULL);
  }
  return bRes;
}

bool CDObject::GetSelectedDimen(PDDimension pDimen)
{
  bool bFound = false;
  int i = 0;
  int n = m_pDimens->GetCount();
  PDDimension pDim1;
  while(!bFound && (i < n))
  {
    pDim1 = (PDDimension)m_pDimens->GetItem(i++);
    bFound = pDim1->bSelected;
  }
  if(bFound) CopyDimenAttrs(pDimen, pDim1);
  return bFound;
}

bool CDObject::SetSelectedDimen(PDDimension pDimen, PDPtrList pRegions)
{
  bool bRes = false;
  int n = m_pDimens->GetCount();
  PDDimension pDim1;
  for(int i = 0; i < n; i++)
  {
    pDim1 = (PDDimension)m_pDimens->GetItem(i);
    if(pDim1->bSelected)
    {
      CopyDimenAttrs(pDim1, pDimen);
      AddRegions(pRegions, 9);
      AddRegions(pRegions, 10);
      bRes = true;
    }
  }
  return bRes;
}

bool CDObject::GetSnapTo()
{
  return m_bSnapTo;
}

void CDObject::SetSnapTo(bool bSnap)
{
  m_bSnapTo = bSnap;
}

void CDObject::AddRegions(PDPtrList pRegions, int iPrimType)
{
  PDPoint pPts1, pPts2;
  CDPoint cDim;
  int i1, i2;
  CDPrimitive cPrim;
  PDPolygon pPoly;
  double dScale = pRegions->GetDblVal();
  double dLWidth = fabs(m_cLineStyle.dWidth);
  if(dLWidth < 1.0) dLWidth = 1.0;

  for(int i = 0; i < m_pPrimitive->GetCount(); i++)
  {
    cPrim = m_pPrimitive->GetPrimitive(i);
    if((iPrimType < 0) || (iPrimType == cPrim.iType))
    {
      cDim = GetPrimRegion(cPrim, dLWidth, dScale, NULL, NULL);
      i1 = cDim.x;
      i2 = cDim.y;
      pPts1 = NULL;
      pPts2 = NULL;
      if(i1 > 0) pPts1 = (PDPoint)malloc(i1*sizeof(CDPoint));
      if(i2 > 0) pPts2 = (PDPoint)malloc(i2*sizeof(CDPoint));

      cDim = GetPrimRegion(cPrim, dLWidth, dScale, pPts1, pPts2);
      i1 = cDim.x;
      i2 = cDim.y;
      if(i1 > 0)
      {
        pPoly = (PDPolygon)malloc(sizeof(CDPolygon));
        pPoly->iPoints = i1;
        pPoly->pPoints = pPts1;
        pRegions->Add(pPoly);
      }
      else if(pPts1) free(pPts1);
      if(i2 > 0)
      {
        pPoly = (PDPolygon)malloc(sizeof(CDPolygon));
        pPoly->iPoints = i2;
        pPoly->pPoints = pPts2;
        pRegions->Add(pPoly);
      }
      else if(pPts2) free(pPts2);
    }
  }
}

void CDObject::SetAuxInt(int iVal)
{
  m_iAuxInt = iVal;
}

int CDObject::GetAuxInt()
{
  return m_iAuxInt;
}

/*int CDObject::GetNumParts()
{
  switch(m_iType)
  {
  case dtEllipse:
    return GetElpsNumParts(m_pCachePoints, m_cBounds);
  case dtArcEllipse:
    return GetArcElpsNumParts(m_pCachePoints, m_cBounds);
  case dtHyperbola:
    return GetHyperNumParts(m_pCachePoints, m_cBounds);
  case dtParabola:
    return GetParabNumParts(m_pCachePoints, m_cBounds);
  default:
    return 0;
  }
  return 0;
}

bool CDObject::RemovePart(bool bDown, PDRefPoint pBounds)
{
  CDRefPoint cBounds[2];
  cBounds[0] = pBounds[0];
  cBounds[1] = pBounds[1];

  bool bRes = false;
  switch(m_iType)
  {
  case dtEllipse:
    bRes = ElpsRemovePart(bDown, m_pCachePoints, cBounds);
    break;
  case dtArcEllipse:
    bRes = ArcElpsRemovePart(bDown, m_pCachePoints, cBounds);
    break;
  case dtHyperbola:
    bRes = HyperRemovePart(bDown, m_pCachePoints, cBounds);
    break;
  case dtParabola:
    bRes = ParabRemovePart(bDown, m_pCachePoints, cBounds);
    break;
  default:
    break;
  }
  if(bRes)
  {
    m_cBounds[0] = cBounds[0];
    m_cBounds[1] = cBounds[1];
  }
  return bRes;
}

CDObject* CDObject::SplitPart(PDRect pRect, PDPtrList pRegions)
{
  CDRefPoint cBounds[2];
  cBounds[0] = m_cBounds[0];
  cBounds[1] = m_cBounds[1];

  PDObject pNewObj = NULL;
  if(RemovePart(false, cBounds))
  {
    CDLine cLn;
    cLn.bIsSet = false;

    BuildPrimitives(cLn, 0, pRect, 0, NULL);
    AddRegions(pRegions, -1);

    pNewObj = Copy();
    pNewObj->BuildCache(cLn, 0);
    if(pNewObj->RemovePart(true, cBounds))
    {
      pNewObj->BuildPrimitives(cLn, 0, pRect, 0, NULL);
      pNewObj->SetSelected(true, false, -1, pRegions);
    }
    else
    {
      delete pNewObj;
      pNewObj = NULL;
    }
  }
  return pNewObj;
}*/

bool CDObject::IsBoundShape()
{
  switch(m_iType)
  {
  case dtLine:
  case dtHyperbola:
  case dtParabola:
    return m_cBounds[0].bIsSet && m_cBounds[1].bIsSet;
  case dtEvolvent:
    return m_cBounds[1].bIsSet;
  default:
    return true;
  }
}

bool CDObject::GetStartPoint(PDPoint pPt, double dOffset)
{
  CDPoint cBnds;
  if(GetBounds(&cBnds, dOffset, true) & 1)
  {
    double dRef;
    GetNativeReference(cBnds.x, dOffset, &dRef);
    return GetNativeRefPoint(dRef, dOffset, pPt);
  }
  return false;
}

bool CDObject::GetEndPoint(PDPoint pPt, double dOffset)
{
  CDPoint cBnds;
  if(GetBounds(&cBnds, dOffset, true) & 2)
  {
    double dRef;
    GetNativeReference(cBnds.y, dOffset, &dRef);
    return GetNativeRefPoint(dRef, dOffset, pPt);
  }
  return false;
}

void CDObject::AddSegmentToPath(PDPathSeg pNewSeg, bool bInsert)
{
  int iCnt = m_pSubObjects->GetCount();
  if(iCnt < 1)
  {
    if(bInsert) m_pSubObjects->Add(pNewSeg);
    return;
  }

  PDPathSeg pLastSeg = (PDPathSeg)m_pSubObjects->GetItem(iCnt - 1);
  PDObject pObj = pLastSeg->pSegment;
  CDPoint cBounds;
  if(pObj->GetBoundsRef(&cBounds, 0.0, false) < 3)
  {
    if(bInsert) m_pSubObjects->Add(pNewSeg);
    return;
  }

  CDPoint cLastDir;
  bool bHasLastDir = false;
  if(pLastSeg->bReverse)
  {
    bHasLastDir = pObj->GetNativeRefDir(cBounds.x, &cLastDir);
    if(bHasLastDir) cLastDir *= -1.0;
  }
  else bHasLastDir = pObj->GetNativeRefDir(cBounds.y, &cLastDir);
  if(!bHasLastDir)
  {
    if(bInsert) m_pSubObjects->Add(pNewSeg);
    return;
  }

  pObj = pNewSeg->pSegment;
  if(pObj->GetBoundsRef(&cBounds, 0.0, false) < 3)
  {
    if(bInsert) m_pSubObjects->Add(pNewSeg);
    return;
  }

  CDPoint cNewDir;
  bool bHasNewDir = false;
  if(pNewSeg->bReverse)
  {
    bHasNewDir = pObj->GetNativeRefDir(cBounds.y, &cNewDir);
    if(bHasNewDir) cNewDir *= -1.0;
  }
  else bHasNewDir = pObj->GetNativeRefDir(cBounds.x, &cNewDir);
  if(!bHasNewDir)
  {
    if(bInsert) m_pSubObjects->Add(pNewSeg);
    return;
  }

  double dDirProd = cLastDir*cNewDir;
  if(dDirProd > 1.0 - g_dPrec)
  {
    if(bInsert) m_pSubObjects->Add(pNewSeg);
    return;
  }

  CDPoint cOrig;
  if(pNewSeg->bReverse) pObj->GetNativeRefPoint(cBounds.y, 0.0, &cOrig);
  else pObj->GetNativeRefPoint(cBounds.x, 0.0, &cOrig);

  PDObject pMidCirc = new CDObject(dtCircle, m_cLineStyle.dWidth);
  pMidCirc->AddPoint(cOrig.x, cOrig.y, 1, 0.0);
  pMidCirc->AddPoint(cOrig.x, cOrig.y, 0, 0.0);
  cOrig = GetNormal(cLastDir);
  CDRefPoint cBnd = {true, atan2(cOrig.y, cOrig.x)};
  pMidCirc->SetBound(0, cBnd);
  cOrig = GetNormal(cNewDir);
  cBnd.dRef = atan2(cOrig.y, cOrig.x);
  pMidCirc->SetBound(1, cBnd);
  CDLine cTmpPt;
  pMidCirc->BuildCache(cTmpPt, 0);

  PDPathSeg pMidSeg = (PDPathSeg)malloc(sizeof(CDPathSeg));
  cOrig = Rotate(cNewDir, cLastDir, false);
  pMidSeg->bReverse = (cOrig.y < 0.0);
  pMidSeg->pSegment = pMidCirc;

  m_pSubObjects->Add(pMidSeg);
  if(bInsert) m_pSubObjects->Add(pNewSeg);
}

void CDObject::BuildPath(CDObject **ppObjects, PDIntList pPath)
{
  int n = pPath->GetCount();
  PDPathSeg pSeg;
  PDObject pObj;
  int iIdx, iType;
  for(int i = 0; i < n; i++)
  {
    iIdx = pPath->GetItem(i);
    pObj = ppObjects[abs(iIdx) - 1];
    iType = pObj->GetType();
    if(iType < dtPath)
    {
      pSeg = (PDPathSeg)malloc(sizeof(CDPathSeg));
      pSeg->bReverse = iIdx < 0;
      pSeg->pSegment = pObj;
      AddSegmentToPath(pSeg, true);
    }
    else if(iType == dtPath)
    {
      if(iIdx < 0)
      {
        for(int j = pObj->m_pSubObjects->GetCount() - 1; j >= 0; j--)
        {
          pSeg = (PDPathSeg)pObj->m_pSubObjects->GetItem(j);
          pSeg->bReverse = !pSeg->bReverse;
          AddSegmentToPath(pSeg, true);
        }
      }
      else
      {
        for(int j = 0; j < pObj->m_pSubObjects->GetCount(); j++)
        {
          pSeg = (PDPathSeg)pObj->m_pSubObjects->GetItem(j);
          AddSegmentToPath(pSeg, true);
        }
      }
      pObj->m_pSubObjects->Clear();
      delete pObj;
    }
  }
  if(IsClosedPath())
  {
    pSeg = (PDPathSeg)m_pSubObjects->GetItem(0);
    AddSegmentToPath(pSeg, false);
  }
}

int CDObject::GetSubObjectCount()
{
  if(m_iType < dtPath) return 1;
  int iRes = 0;
  PDObject pObj;
  for(int i = 0; i < pObj->m_pSubObjects->GetCount(); i++)
  {
    pObj = (PDObject)m_pSubObjects->GetItem(i);
    iRes += pObj->GetSubObjectCount();
  }
  return iRes;
}

int CDObject::GetSnapPoint(int iSnapMask, CDPoint cPt, double dDist, PDLine pSnapPt, PDObject pDynObj, bool bHonorSnapTo)
{
  if(bHonorSnapTo && !m_bSnapTo) return 0;

  CDPoint cPt1;
  double dblDist = 2.0*dDist;

  if(m_cBounds[0].bIsSet)
  {
    GetNativeRefPoint(m_cBounds[0].dRef, 0.0, &cPt1);
    if(GetDist(cPt, cPt1) < dblDist)
    {
      pSnapPt->bIsSet = true;
      pSnapPt->cOrigin = cPt1;
      pSnapPt->dRef = m_cBounds[0].dRef;
      GetNativeRefDir(m_cBounds[0].dRef, &cPt1);
      pSnapPt->cDirection = GetNormal(cPt1);
      return 2;
    }
  }
  if(m_cBounds[1].bIsSet)
  {
    GetNativeRefPoint(m_cBounds[1].dRef, 0.0, &cPt1);
    if(GetDist(cPt, cPt1) < dblDist)
    {
      pSnapPt->bIsSet = true;
      pSnapPt->cOrigin = cPt1;
      pSnapPt->dRef = m_cBounds[1].dRef;
      GetNativeRefDir(m_cBounds[1].dRef, &cPt1);
      pSnapPt->cDirection = GetNormal(cPt1);
      return 2;
    }
  }

  if(!IsClosedShape())
  {
    CDPoint cLocBnds = {0.0, 0.0};
    int iRefBounds = GetRefBounds(&cLocBnds);
    if(iRefBounds > 0)
    {
      GetNativeRefPoint(cLocBnds.x, 0.0, &cPt1);
      if(GetDist(cPt, cPt1) < dblDist)
      {
        pSnapPt->bIsSet = true;
        pSnapPt->cOrigin = cPt1;
        pSnapPt->dRef = cLocBnds.x;
        GetNativeRefDir(cLocBnds.x, &cPt1);
        pSnapPt->cDirection = GetNormal(cPt1);
        return 2;
      }
      if(iRefBounds > 1)
      {
        GetNativeRefPoint(cLocBnds.y, 0.0, &cPt1);
        if(GetDist(cPt, cPt1) < dblDist)
        {
          pSnapPt->bIsSet = true;
          pSnapPt->cOrigin = cPt1;
          pSnapPt->dRef = cLocBnds.y;
          GetNativeRefDir(cLocBnds.y, &cPt1);
          pSnapPt->cDirection = GetNormal(cPt1);
          return 2;
        }
      }
    }
  }

  if(bHonorSnapTo && (iSnapMask == 1) && m_bSelected)
  {
    double dDist2 = GetNearestCrossPoint(cPt, &cPt1);
    if((dDist2 > -0.5) && (dDist2 < dblDist))
    {
      pSnapPt->bIsSet = true;
      pSnapPt->cOrigin = cPt1;
      pSnapPt->cDirection = 0;
      return 3;
    }
  }

  double dRefs[8];
  int iLen = 0;

  if(m_iType == dtEllipse) iLen = GetElpsSnapPoints(m_pCachePoints, dRefs);
  else if(m_iType == dtArcEllipse) iLen = GetArcElpsSnapPoints(m_pCachePoints, dRefs);
  else if(m_iType == dtHyperbola) iLen = GetHyperSnapPoints(m_pCachePoints, dRefs);
  else if(m_iType == dtParabola) iLen = GetParabSnapPoints(m_pCachePoints, dRefs);
  else if(m_iType == dtEvolvent) iLen = GetEvolvSnapPoints(m_pCachePoints, dRefs);

  if(iLen > 0)
  {
    int i = 0;
    bool bFound = false;
    while(!bFound && (i < iLen))
    {
      if(IsValidRef(dRefs[i]))
      {
        GetNativeRefPoint(dRefs[i], 0.0, &cPt1);
        bFound = (GetDist(cPt, cPt1) < dblDist);
      }
      i++;
    }
    if(bFound)
    {
      pSnapPt->bIsSet = true;
      pSnapPt->cOrigin = cPt1;
      pSnapPt->dRef = dRefs[i - 1];
      GetNativeRefDir(pSnapPt->dRef, &cPt1);
      pSnapPt->cDirection = GetNormal(cPt1);
      return 2;
    }
  }

  if(m_iType == dtSpline)
  {
    bool bFound = false;
    PDRefList pRefs = new CDRefList();
    iLen = GetSplineSnapPoints(cPt, dblDist, m_pCachePoints, pRefs);
    if(iLen > 0)
    {
      double dt;
      int i = 0;
      while(!bFound && (i < iLen))
      {
        dt = pRefs->GetPoint(i++);
        bFound = IsValidRef(dt);
      }
      if(bFound)
      {
        GetNativeRefPoint(dt, 0.0, &cPt1);
        pSnapPt->bIsSet = true;
        pSnapPt->cOrigin = cPt1;
        pSnapPt->dRef = dt;
        GetNativeRefDir(dt, &cPt1);
        pSnapPt->cDirection = GetNormal(cPt1);
      }
    }
    delete pRefs;
    if(bFound) return 2;
  }

  if(m_iType == dtPath)
  {
    PDPtrList pList = new CDPtrList();
    PDPathSeg pSeg;
    for(int i = 0; i < m_pSubObjects->GetCount(); i++)
    {
      pSeg = (PDPathSeg)m_pSubObjects->GetItem(i);
      pList->Add(pSeg->pSegment);
    }
    int iRes = GetSnapPointFromList(iSnapMask, cPt, dDist, pSnapPt, pDynObj,
      (PDObject*)pList->GetList(), pList->GetCount(), false);
    delete pList;
    if(iRes > 1)
    {
      CDLine cLnRef;
      GetPathDistFromPt(pSnapPt->cOrigin, pSnapPt->cOrigin, false, &cLnRef);
      pSnapPt->dRef = cLnRef.dRef;
      return iRes;
    }
  }

  CDLine cPtSnap1;
  int iDim;
  double dDist1 = fabs(GetDistFromPt(cPt, cPt, 1, &cPtSnap1, &iDim));
  if(!cPtSnap1.bIsSet || (dDist1 > dblDist)) return 0;

  if(cPtSnap1.bIsSet && (GetNorm(cPtSnap1.cDirection) < 0.5))
  {
    *pSnapPt = cPtSnap1;
    return 5;
  }

  if(dDist1 < dDist)
  {
    *pSnapPt = cPtSnap1;
    return 1;
  }
  return 0;
}


// CDataList

CDataList::CDataList()
{
  m_iDataLen = 0;
  m_iDataSize = 16;
  m_ppObjects = (PDObject*)malloc(m_iDataSize*sizeof(PDObject));
  m_bHasChanged = false;
}

CDataList::~CDataList()
{
  for(int i = 0; i < m_iDataLen; i++)
  {
    delete m_ppObjects[i];
  }
  free(m_ppObjects);
}

int CDataList::GetCount()
{
  return m_iDataLen;
}

void CDataList::Add(PDObject pObject)
{
  CDLine cPtX;
  cPtX.bIsSet = false;
  if(!pObject->BuildCache(cPtX, 0)) return;
  if(m_iDataLen >= m_iDataSize)
  {
    m_iDataSize += 16;
    m_ppObjects = (PDObject*)realloc(m_ppObjects, m_iDataSize*sizeof(PDObject));
  }
  m_ppObjects[m_iDataLen++] = pObject;
  m_bHasChanged = true;
  return;
}

void CDataList::Remove(int iIndex, bool bFree)
{
  if(bFree) delete m_ppObjects[iIndex];

  m_iDataLen--;
  if(iIndex < m_iDataLen)
  {
    memmove(&m_ppObjects[iIndex], &m_ppObjects[iIndex + 1], (m_iDataLen - iIndex)*sizeof(PDObject));
  }
}

PDObject CDataList::GetItem(int iIndex)
{
  return m_ppObjects[iIndex];
}

void CDataList::BuildAllPrimitives(PDRect pRect, bool bResolvePatterns)
{
  CDLine cLn;
  cLn.bIsSet = false;
  int iTemp = 0;
  if(!bResolvePatterns) iTemp = 1;
  for(int i = 0; i < m_iDataLen; i++)
  {
    m_ppObjects[i]->BuildPrimitives(cLn, 0, pRect, iTemp, NULL);
  }
}

PDObject CDataList::SelectByPoint(CDPoint cPt, double dDist, int *piDimen)
{
  int i = 0;
  bool bFound = false;
  while(!bFound && (i < m_iDataLen))
  {
    bFound = m_ppObjects[i++]->IsNearPoint(cPt, dDist, piDimen);
  }
  return bFound ? m_ppObjects[i - 1] : NULL;
}

PDObject CDataList::SelectLineByPoint(CDPoint cPt, double dDist)
{
  int i = 0;
  bool bFound = false;
  PDObject pObj = NULL;
  int iDimen;
  while(!bFound && (i < m_iDataLen))
  {
    pObj = m_ppObjects[i++];
    bFound = (pObj->GetType() == 1) && pObj->IsNearPoint(cPt, dDist, &iDimen);
  }
  return bFound ? pObj : NULL;
}

void CDataList::ClearSelection(PDPtrList pRegions)
{
  for(int i = 0; i < m_iDataLen; i++)
  {
    m_ppObjects[i]->SetSelected(false, false, -1, pRegions);
  }
}

int CDataList::GetNumOfSelectedLines()
{
  int iRes = 0;
  for(int i = 0; i < m_iDataLen; i++)
  {
    if(m_ppObjects[i]->GetSelected() && (m_ppObjects[i]->GetType() == 1)) iRes++;
  }
  return iRes;
}

PDObject CDataList::GetSelectedLine(int iIndex)
{
  PDObject pRes = NULL;
  int i = 0;
  int iCurSelLine = -1;
  while((i < m_iDataLen) && (iCurSelLine < iIndex))
  {
    pRes = m_ppObjects[i++];
    if(pRes->GetSelected() && (pRes->GetType() == 1)) iCurSelLine++;
  }

  if(iCurSelLine >= iIndex) return pRes;

  return NULL;
}

int CDataList::GetNumOfSelectedCircles()
{
  int iRes = 0;
  for(int i = 0; i < m_iDataLen; i++)
  {
    if(m_ppObjects[i]->GetSelected() && (m_ppObjects[i]->GetType() == 2)) iRes++;
  }
  return iRes;
}

PDObject CDataList::GetSelectedCircle(int iIndex)
{
  PDObject pRes = NULL;
  int i = 0;
  int iCurSelLine = -1;
  while((i < m_iDataLen) && (iCurSelLine < iIndex))
  {
    pRes = m_ppObjects[i++];
    if(pRes->GetSelected() && (pRes->GetType() == 2)) iCurSelLine++;
  }

  if(iCurSelLine >= iIndex) return pRes;

  return NULL;
}

int CDataList::GetSnapPoint(int iSnapMask, CDPoint cPt, double dDist, PDLine pSnapPt, PDObject pDynObj)
{
  return GetSnapPointFromList(iSnapMask, cPt, dDist, pSnapPt, pDynObj, m_ppObjects, m_iDataLen, true);
}

bool CDataList::DeleteSelected(CDataList *pUndoList, PDRect pRect, PDPtrList pRegions)
{
  bool bRes = false;
  PDObject pObj;
  for(int i = m_iDataLen - 1; i >= 0; i--)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      pObj->AddRegions(pRegions, -1);
      bRes = true;
      pUndoList->Add(pObj);
      m_iDataLen--;
      if(i < m_iDataLen)
      {
        memmove(&m_ppObjects[i], &m_ppObjects[i + 1], (m_iDataLen - i)*sizeof(PDObject));
      }
    }
    else bRes |= pObj->DeleteSelDimens(pRect, pRegions);
  }
  if(bRes) m_bHasChanged = true;
  return bRes;
}

int CDataList::GetSelectCount(unsigned char cVersion)
{
  int iRes = 0;
  PDObject pObj;
  if(cVersion > 1)
  {
    for(int i = 0; i < m_iDataLen; i++)
    {
      pObj = m_ppObjects[i];
      if(pObj->GetSelected()) iRes++;
    }
  }
  else
  {
    for(int i = 0; i < m_iDataLen; i++)
    {
      pObj = m_ppObjects[i];
      if(pObj->GetSelected()) iRes += pObj->GetSubObjectCount();
    }
  }
  return iRes;
}

PDObject CDataList::GetSelected(int iIndex)
{
  int i = 0;
  PDObject pObj = NULL;
  while((i < m_iDataLen) && (iIndex > -1))
  {
    pObj = m_ppObjects[i++];
    if(pObj->GetSelected()) iIndex--;
  }
  return iIndex < 0 ? pObj : NULL;
}

bool CDataList::CutSelected(CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions)
{
  bool bRes = false;
  PDObject pObj, pObjNew = NULL;
  int n = m_iDataLen;
  PDPtrList pNewObjs = new CDPtrList();
  for(int i = 0; i < n; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      pNewObjs->Clear();
      if(pObj->Split(cPt, pNewObjs, pRect, pRegions))
      {
        bRes = true;
        for(int i = 0; i < pNewObjs->GetCount(); i++)
        {
          pObjNew = (PDObject)pNewObjs->GetItem(i);
          Add(pObjNew);
        }
      }
    }
  }
  delete pNewObjs;
  if(bRes) m_bHasChanged = true;
  return bRes;
}

bool CDataList::ExtendSelected(CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions)
{
  bool bRes = false;
  PDObject pObj;
  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      bRes |= pObj->Extend(cPt, dDist, pRect, pRegions);
    }
  }
  if(bRes) m_bHasChanged = true;
  return bRes;
}

void CDataList::ClearAll()
{
  for(int i = 0; i < m_iDataLen; i++)
  {
    delete m_ppObjects[i];
  }
  m_iDataLen = 0;
  m_bHasChanged = false;
}

void CDataList::SaveToFile(FILE *pf, bool bSwapBytes, bool bSelectOnly, unsigned char cVersion)
{
  unsigned char buf[16], *pbuf;
  buf[0] = 3;
  buf[1] = 8;
  buf[2] = 7;
  buf[3] = 0;
  buf[4] = 1;
  buf[5] = 6;

  // magic number
  fwrite(buf, 1, 6, pf);

  buf[0] = cVersion;

  // version
  fwrite(buf, 1, 1, pf);

  pbuf = (unsigned char*)&m_cFileAttrs.dWidth;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  pbuf = (unsigned char*)&m_cFileAttrs.dHeight;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  pbuf = (unsigned char*)&m_cFileAttrs.dScaleNom;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  pbuf = (unsigned char*)&m_cFileAttrs.dScaleDenom;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  unsigned long lDataLen = m_iDataLen;
  if(bSelectOnly) lDataLen = GetSelectCount(cVersion);
  else if(cVersion < 2)
  {
    lDataLen = 0;
    for(int i = 0; i < m_iDataLen; i++)
    {
      lDataLen += m_ppObjects[i]->GetSubObjectCount();
    }
  }

  pbuf = (unsigned char*)&lDataLen;
  SwapBytes(buf, pbuf, 4, bSwapBytes);
  fwrite(buf, 1, 4, pf);

  if(bSelectOnly)
  {
    for(int i = 0; i < m_iDataLen; i++)
    {
      if(m_ppObjects[i]->GetSelected())
      m_ppObjects[i]->SaveToFile(pf, bSwapBytes, cVersion);
    }
  }
  else
  {
    for(int i = 0; i < m_iDataLen; i++)
    {
      m_ppObjects[i]->SaveToFile(pf, bSwapBytes, cVersion);
    }
  }
  m_bHasChanged = bSelectOnly;
}

bool CDataList::ReadFromFile(FILE *pf, bool bSwapBytes, bool bClear)
{
  unsigned char buf[16], *pbuf;
  fread(buf, 1, 6, pf);

  bool bMagicOK = (buf[0] == 3) && (buf[1] == 8) && (buf[2] == 7) &&
    (buf[3] == 0) && (buf[4] == 1) && (buf[5] == 6);

  if(!bMagicOK) return false;

  // version
  fread(buf, 1, 1, pf);
  if(buf[0] > 2) return false; // we don't know that version yet
  unsigned char cVer = buf[0];

  if(bClear) ClearAll();

  fread(buf, 1, 8, pf);
  if(bClear)
  {
    pbuf = (unsigned char*)&m_cFileAttrs.dWidth;
    SwapBytes(pbuf, buf, 8, bSwapBytes);
  }

  fread(buf, 1, 8, pf);
  if(bClear)
  {
    pbuf = (unsigned char*)&m_cFileAttrs.dHeight;
    SwapBytes(pbuf, buf, 8, bSwapBytes);
  }

  fread(buf, 1, 8, pf);
  if(bClear)
  {
    pbuf = (unsigned char*)&m_cFileAttrs.dScaleNom;
    SwapBytes(pbuf, buf, 8, bSwapBytes);
  }

  fread(buf, 1, 8, pf);
  if(bClear)
  {
    pbuf = (unsigned char*)&m_cFileAttrs.dScaleDenom;
    SwapBytes(pbuf, buf, 8, bSwapBytes);
  }

  fread(buf, 1, 4, pf);
  unsigned long lDataLen = 0;
  pbuf = (unsigned char*)&lDataLen;
  SwapBytes(pbuf, buf, 4, bSwapBytes);

  PDObject pObj;

  for(unsigned int i = 0; i < lDataLen; i++)
  {
    fread(buf, 1, 1, pf);
    pObj = new CDObject((CDDrawType)buf[0], 0.2);
    pObj->ReadFromFile(pf, bSwapBytes, cVer);
    Add(pObj);
  }
  m_bHasChanged = !bClear;
  return true;
}

void CDataList::SelectByRectangle(PDRect pRect, int iMode, PDPtrList pRegions)
{
  double x;
  if(pRect->cPt1.x > pRect->cPt2.x)
  {
    x = pRect->cPt1.x;
    pRect->cPt1.x = pRect->cPt2.x;
    pRect->cPt2.x = x;
  }
  if(pRect->cPt1.y > pRect->cPt2.y)
  {
    x = pRect->cPt1.y;
    pRect->cPt1.y = pRect->cPt2.y;
    pRect->cPt2.y = x;
  }

  PDObject pObj;
  CDLine cLn;
  cLn.bIsSet = false;
  int k;
  CDPoint cBnds = {0.0, 0.0};
  PDRefList pBounds = new CDRefList();
  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    //k = pObj->BuildPrimitives(cLn, 0, pRect, 2, NULL);
    pBounds->Clear();
    k = pObj->GetViewBounds(cLn, 0, pRect, pBounds, &cBnds, true);
    if(k == iMode) pObj->SetSelected(true, false, -1, pRegions);
  }
  delete pBounds;

  PDPoint pPts = (PDPoint)malloc(4*sizeof(CDPoint));
  pPts[0].x = pRect->cPt1.x - 1.0;
  pPts[0].y = pRect->cPt1.y - 1.0;
  pPts[1].x = pRect->cPt2.x + 1.0;
  pPts[1].y = pRect->cPt1.y - 1.0;
  pPts[2].x = pRect->cPt2.x + 1.0;
  pPts[2].y = pRect->cPt2.y + 1.0;
  pPts[3].x = pRect->cPt1.x - 1.0;
  pPts[3].y = pRect->cPt2.y + 1.0;

  PDPolygon pPoly = (PDPolygon)malloc(sizeof(CDPolygon));
  pPoly->iPoints = 4;
  pPoly->pPoints = pPts;
  pRegions->Add(pPoly);
}

bool CDataList::RotateSelected(CDPoint cOrig, double dRot, int iCop, PDRect pRect, PDPtrList pRegions)
{
  bool bRes = false;
  double dRotStep = dRot;
  if(iCop > 1) dRotStep = dRot/iCop;
  if(fabs(fabs(dRot) - 2*M_PI) < g_dPrec)
  {
    if(iCop > 1) iCop--;
    else return false;
  }

  PDObject pObj, pObj1;
  CDLine cLn;
  cLn.bIsSet = false;
  int iCurLen = m_iDataLen;
  for(int i = 0; i < iCurLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      bRes = true;
      if(iCop < 1)
      {
        pObj->AddRegions(pRegions, -1);
        pObj->RotatePoints(cOrig, dRot, 2);
        pObj->BuildPrimitives(cLn, 0, pRect, 0, NULL);
        pObj->AddRegions(pRegions, -1);
      }
      else
      {
        for(int j = 0; j < iCop; j++)
        {
          pObj1 = pObj->Copy();
          pObj1->RotatePoints(cOrig, (j + 1)*dRotStep, 0);
          pObj1->BuildPrimitives(cLn, 0, pRect, 0, NULL);
          pObj1->AddRegions(pRegions, -1);
          Add(pObj1);
        }
        if(iCop == 1)
        {
          pObj->SetSelected(false, false, -1, pRegions);
          pObj1->SetSelected(true, false, -1, pRegions);
        }
      }
    }
    else
    {
      if(pObj->RotatePoints(cOrig, dRot, 1))
      {
        pObj->AddRegions(pRegions, -1);
        bRes = true;
        pObj->BuildPrimitives(cLn, 0, pRect, 0, NULL);
        pObj->AddRegions(pRegions, -1);
      }
    }
  }
  if(bRes) m_bHasChanged = true;
  return bRes;
}

bool CDataList::MoveSelected(CDLine cLine, double dDist, int iCop, PDRect pRect,
  bool bPreserveDir, PDPtrList pRegions)
{
  bool bRes = false;
  double dDistStep = dDist;
  if(iCop > 1) dDistStep = dDist/iCop;

  CDPoint cDir = cLine.cDirection;
  CDLine cLn;
  cLn.bIsSet = false;
  if(!bPreserveDir)
  {
    double dAng = atan2(cDir.y, cDir.x);
    if((dAng > 0.8) || (dAng < -2.4)) cDir *= -1.0;
  }

  PDObject pObj, pObj1;
  int iCurLen = m_iDataLen;
  for(int i = 0; i < iCurLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      bRes = true;
      if(iCop < 1)
      {
        pObj->AddRegions(pRegions, -1);
        pObj->MovePoints(cDir, dDist, 2);
        pObj->BuildPrimitives(cLn, 0, pRect, 0, NULL);
        pObj->AddRegions(pRegions, -1);
      }
      else
      {
        for(int j = 0; j < iCop; j++)
        {
          pObj1 = pObj->Copy();
          pObj1->MovePoints(cDir, (j + 1)*dDistStep, 0);
          pObj1->BuildPrimitives(cLn, 0, pRect, 0, NULL);
          pObj1->AddRegions(pRegions, -1);
          Add(pObj1);
        }
        if(iCop == 1)
        {
          pObj->SetSelected(false, false, -1, pRegions);
          pObj1->SetSelected(true, false, -1, pRegions);
        }
      }
    }
    else
    {
      if(pObj->MovePoints(cDir, dDist, 1))
      {
        pObj->AddRegions(pRegions, -1);
        bRes = true;
        pObj->BuildPrimitives(cLn, 0, pRect, 0, NULL);
        pObj->AddRegions(pRegions, -1);
      }
    }
  }
  if(bRes) m_bHasChanged = true;
  return bRes;
}

bool CDataList::MirrorSelected(CDLine cLine, PDRect pRect, PDPtrList pRegions)
{
  bool bRes = false;

  PDObject pObj, pObj1;
  CDLine cLn;
  cLn.bIsSet = false;
  int iCurLen = m_iDataLen;
  for(int i = 0; i < iCurLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      pObj->AddRegions(pRegions, -1);
      bRes = true;
      pObj1 = pObj->Copy();
      pObj1->MirrorPoints(cLine);
      pObj1->BuildPrimitives(cLn, 0, pRect, 0, NULL);
      pObj1->AddRegions(pRegions, -1);
      Add(pObj1);
    }
  }
  if(bRes) m_bHasChanged = true;
  return bRes;
}

bool LSPatternMatch(CDLineStyle cStyle1, CDLineStyle cStyle2)
{
  bool bRes = (cStyle1.iSegments == cStyle2.iSegments);
  int i = 0;
  int n = cStyle1.iSegments;
  if(n > 6) n = 6;
  while(bRes && (i < n))
  {
    bRes = (fabs(cStyle1.dPattern[i] - cStyle2.dPattern[i]) < g_dPrec);
    i++;
  }
  return bRes;
}

bool LSColorMatch(unsigned char *cCol1, unsigned char *cCol2)
{
  return (cCol1[0] == cCol2[0]) && (cCol1[1] == cCol2[1]) && (cCol1[2] == cCol2[2]) && (cCol1[3] == cCol2[3]);
}

int CDataList::GetSelectedLineStyle(PDLineStyle pStyle)
{
  int iRes = -1;
  CDLineStyle cSt1, cSt2;
  PDObject pObj;
  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      if(iRes < 0)
      {
        cSt1 = pObj->GetLineStyle();
        iRes = 63;
      }
      else
      {
        cSt2 = pObj->GetLineStyle();
        if(fabs(cSt1.dWidth - cSt2.dWidth) > g_dPrec) iRes &= ~1;
        if(fabs(cSt1.dPercent - cSt2.dPercent) > g_dPrec) iRes &= ~2;
        if(!LSPatternMatch(cSt1, cSt2)) iRes &= ~4;
        if(cSt1.cCapType != cSt2.cCapType) iRes &= ~8;
        if(cSt1.cJoinType != cSt2.cJoinType) iRes &= ~16;
        if(!LSColorMatch(cSt1.cColor, cSt2.cColor)) iRes &= ~32;
      }
    }
  }
  if(iRes > -1) *pStyle = cSt1;
  return iRes;
}

bool CDataList::SetSelectedLineStyle(int iMask, PDLineStyle pStyle, PDPtrList pRegions)
{
  bool bRes = false;
  PDObject pObj;
  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      bRes = true;
      pObj->SetLineStyle(iMask, *pStyle);
      pObj->AddRegions(pRegions, -1);
      m_bHasChanged = true;
    }
  }
  return bRes;
}

bool CDataList::SetCrossSelected(CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions)
{
  bool bRes = false;
  PDObject pObj;
  CDLine cLn;
  cLn.bIsSet = false;
  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      if(pObj->AddCrossPoint(cPt, dDist))
      {
        bRes = true;
        pObj->BuildPrimitives(cLn, 0, pRect, 0, NULL);
        pObj->AddRegions(pRegions, -1);
      }
    }
  }
  if(bRes) m_bHasChanged = true;
  return bRes;
}

bool CDataList::AddDimen(PDObject pSelForDiment, CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions)
{
    if(!pSelForDiment) return false;

    bool bRes = pSelForDiment->AddDimen(cPt, dDist, pRect, &m_cFileAttrs);
    if(bRes)
    {
        m_bHasChanged = true;
        pSelForDiment->AddRegions(pRegions, -1);
    }
    return bRes;
}

bool CDataList::GetChanged()
{
  return m_bHasChanged;
}

void CDataList::SetChanged()
{
  m_bHasChanged = true;
}

void CDataList::SetFileAttrs(PDFileAttrs pFileAttrs, bool bNewFile)
{
  m_cFileAttrs.dWidth = pFileAttrs->dWidth;
  m_cFileAttrs.dHeight = pFileAttrs->dHeight;
  m_cFileAttrs.dScaleNom = pFileAttrs->dScaleNom;
  m_cFileAttrs.dScaleDenom = pFileAttrs->dScaleDenom;
  m_cFileAttrs.iArrowType = pFileAttrs->iArrowType;
  m_cFileAttrs.cArrowDim = pFileAttrs->cArrowDim;
  m_cFileAttrs.dFontSize = pFileAttrs->dFontSize;
  m_cFileAttrs.dBaseLine = pFileAttrs->dBaseLine;
  m_cFileAttrs.bFontAttrs = pFileAttrs->bFontAttrs;
  strcpy(m_cFileAttrs.sFontFace, pFileAttrs->sFontFace);
  strcpy(m_cFileAttrs.sLengthMask, pFileAttrs->sLengthMask);
  strcpy(m_cFileAttrs.sAngleMask, pFileAttrs->sAngleMask);
  if(!bNewFile) m_bHasChanged = true;
}

void CDataList::GetFileAttrs(PDFileAttrs pFileAttrs)
{
  pFileAttrs->dWidth = m_cFileAttrs.dWidth;
  pFileAttrs->dHeight = m_cFileAttrs.dHeight;
  pFileAttrs->dScaleNom = m_cFileAttrs.dScaleNom;
  pFileAttrs->dScaleDenom = m_cFileAttrs.dScaleDenom;
  pFileAttrs->iArrowType = m_cFileAttrs.iArrowType;
  pFileAttrs->cArrowDim = m_cFileAttrs.cArrowDim;
  pFileAttrs->dFontSize = m_cFileAttrs.dFontSize;
  pFileAttrs->dBaseLine = m_cFileAttrs.dBaseLine;
  pFileAttrs->bFontAttrs = m_cFileAttrs.bFontAttrs;
  strcpy(pFileAttrs->sFontFace, m_cFileAttrs.sFontFace);
}

bool CDataList::GetSelectedDimen(PDDimension pDimen)
{
  bool bFound = false;
  int i = 0;
  PDObject pObj;
  while(!bFound && (i < m_iDataLen))
  {
    pObj = m_ppObjects[i++];
    bFound = pObj->GetSelectedDimen(pDimen);
  }
  return bFound;
}

bool CDataList::SetSelectedDimen(PDDimension pDimen, PDPtrList pRegions)
{
  bool bRes = false;
  PDObject pObj;
  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    bRes |= pObj->SetSelectedDimen(pDimen, pRegions);
  }
  if(bRes) m_bHasChanged = true;
  return bRes;
}

void CDataList::GetStatistics(int *piStats)
{
  PDObject pObj;
  int iType;
  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    // 1 line, 2 circle, 3 ellipse, 4 arc ellipse, 5 hyperbola, 6 parabola, 7 spline
    iType = pObj->GetType();
    piStats[iType] += 1;
    piStats[0] += pObj->GetDimenCount();
  }
}

int CDataList::GetUnitMask(int iUnitType, char *psBuf, PDUnitList pUnits)
{
  bool bFound = false;
  int i = 0;
  int iRes = -1;
  PDObject pObj;

  while(!bFound && (i < m_iDataLen))
  {
    pObj = m_ppObjects[i++];
    iRes = pObj->GetUnitMask(iUnitType, psBuf, pUnits);
    bFound = (iRes > -1);
  }
  return iRes;
}

void CDataList::ChangeUnitMask(int iUnitType, char *psMask, PDUnitList pUnits)
{
  PDObject pObj;
  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    m_bHasChanged |= pObj->ChangeUnitMask(iUnitType, psMask, pUnits);
  }
  return;
}

void CDataList::RescaleDrawing(double dNewScaleNom, double dNewScaleDenom, bool bWidths,
  bool bPatterns, bool bArrows, bool bLabels)
{
  double dScaleRatio = dNewScaleNom*m_cFileAttrs.dScaleDenom/(dNewScaleDenom*m_cFileAttrs.dScaleNom);
  if(fabs(dScaleRatio - 1.0) < g_dPrec) return;

  PDObject pObj;
  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    pObj->Rescale(dScaleRatio, bWidths, bPatterns, bArrows, bLabels);
  }

  m_cFileAttrs.dScaleNom = dNewScaleNom;
  m_cFileAttrs.dScaleDenom = dNewScaleDenom;
  m_bHasChanged = true;
  return;
}

bool CDataList::GetSelSnapEnabled()
{
  int iSelTot = 0;
  int iEnabled = 0;
  PDObject pObj;

  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      iSelTot++;
      if(pObj->GetSnapTo()) iEnabled++;
    }
  }

  return 2*iEnabled >= iSelTot;
}

void CDataList::SetSelSnapEnabled(bool bEnable)
{
  PDObject pObj;

  for(int i = 0; i < m_iDataLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected()) pObj->SetSnapTo(bEnable);
  }
}

int CDataList::GetNextSeg(PDIntList pSelObjs, PDIntList pPath, CDPoint cPt1, CDPoint cPt2, PDPoint pPt1, PDPoint pPt2,
  bool *pbFirst)
{
  *pbFirst = false;
  int n = pPath->GetCount();
  int iIdxFirst = abs(pPath->GetItem(0)) - 1;
  int iIdxLast = abs(pPath->GetItem(n - 1)) - 1;

  n = pSelObjs->GetCount();
  if(n < 2) return 0;

  int iIdx, i = 0;
  int iFound = 0;
  PDObject pObj;
  double dDist;
  double dJoinTol = 100.0*g_dPrec;

  while((iFound == 0) && (i < n))
  {
    iIdx = pSelObjs->GetItem(i++);
    pObj = m_ppObjects[iIdx];
    pObj->GetStartPoint(pPt1, 0.0);
    pObj->GetEndPoint(pPt2, 0.0);

    if(iIdx != iIdxFirst)
    {
      dDist = GetDist(cPt1, *pPt2);
      if(dDist < dJoinTol) iFound = 1;
      else
      {
        dDist = GetDist(cPt1, *pPt1);
        if(dDist < dJoinTol) iFound = 2;
      }
      if(iFound > 0) *pbFirst = true;
    }
    if((iFound == 0) && (iIdx != iIdxLast))
    {
      dDist = GetDist(cPt2, *pPt1);
      if(dDist < dJoinTol) iFound = 1;
      else
      {
        dDist = GetDist(cPt2, *pPt2);
        if(dDist < dJoinTol) iFound = 2;
      }
    }
  }

  if(iFound > 1) return -i;
  if(iFound > 0) return i;
  return 0;
}

bool CDataList::BuildPath(PDIntList pSelObjs, PDIntList pSel2, PDIntList pPath)
{
  CDPoint cPt1, cPt2, cPt3, cPt4;
  int n = pSel2->GetCount();
  if(n < 2)
  {
    if(n > 0)
    {
      pSelObjs->RemoveItem(pSel2->GetItem(0));
      pSel2->Remove(0);
    }
    return false;
  }

  int iSeg = pSel2->GetItem(0);
  pPath->AddItem(1 + iSeg);
  PDObject pObj = m_ppObjects[iSeg];
  pObj->GetStartPoint(&cPt1, 0.0);
  pObj->GetEndPoint(&cPt2, 0.0);

  bool bFirst = false;
  int iNext = GetNextSeg(pSel2, pPath, cPt1, cPt2, &cPt3, &cPt4, &bFirst);

  if(iNext == 0)
  {
    pSelObjs->RemoveItem(iSeg);
    pSel2->Remove(0);
    return false;
  }

  double dJoinTol = 100.0*g_dPrec;
  double dDist;
  int iPathIdx;
  while(iNext != 0)
  {
    if(iNext > 0) iPathIdx = 1 + pSel2->GetItem(iNext - 1);
    else iPathIdx = -(1 + pSel2->GetItem(-iNext - 1));

    if(bFirst)
    {
      if(iNext > 0) cPt1 = cPt3;
      else cPt1 = cPt4;
      pPath->InsertItem(0, iPathIdx);
    }
    else
    {
      if(iNext > 0) cPt2 = cPt4;
      else cPt2 = cPt3;
      pPath->AddItem(iPathIdx);
    }
    dDist = GetDist(cPt1, cPt2);

    if(dDist < dJoinTol) iNext = 0;
    else iNext = GetNextSeg(pSel2, pPath, cPt1, cPt2, &cPt3, &cPt4, &bFirst);
  }

  n = pPath->GetCount();
  if(n < 2)
  {
    for(int i = 0; i < n; i++)
    {
      pSelObjs->RemoveItem(abs(pPath->GetItem(i)) - 1);
    }
  }
  for(int i = 0; i < n; i++)
    pSel2->RemoveItem(abs(pPath->GetItem(i)) - 1);

  return n > 1;
}

bool CDataList::BuildPaths(PDIntList pSelObjs, PDPtrList pPaths)
{
  int n = pSelObjs->GetCount();
  if(n < 1) return false;

  PDIntList pSel2 = new CDIntList();
  for(int i = 0; i < n; i++) pSel2->AddItem(pSelObjs->GetItem(i));

  PDIntList pPath = new CDIntList();
  bool bRes = false;

  while(n > 0)
  {
    bRes = BuildPath(pSelObjs, pSel2, pPath);
    if(bRes)
    {
      pPaths->Add(pPath);
      pPath = new CDIntList();
    }
    else pPath->Clear();

    n = pSel2->GetCount();
  }

  delete pPath;
  delete pSel2;
  return pPaths->GetCount() > 0;
}

int CDataList::CreatePath(PDPtrList pRegions)
{
  PDObject pObj, pNewObj;
  int n, i = m_iDataLen;
  PDIntList pPath, pSelObjs;
  pSelObjs = new CDIntList();

  while(i > 0)
  {
    pObj = m_ppObjects[--i];
    if(pObj->GetSelected())
    {
      if(pObj->IsBoundShape() && (pObj->IsClosed() < 2) && (pObj->GetType() <= dtPath))
      {
        pSelObjs->AddItem(i);
      }
    }
  }

  n = pSelObjs->GetCount();
  if(n < 2)
  {
    delete pSelObjs;
    return 0;
  }

  int iRes = 0;

  PDPtrList pPaths = new CDPtrList();
  CDLineStyle cSt;
  if(BuildPaths(pSelObjs, pPaths))
  {
    iRes = pPaths->GetCount();
    for(i = 0; i < iRes; i++)
    {
      pPath = (PDIntList)pPaths->GetItem(i);
      pObj = m_ppObjects[abs(pPath->GetItem(0)) - 1];
      cSt = pObj->GetLineStyle();
      pNewObj = new CDObject(dtPath, cSt.dWidth);
      pNewObj->BuildPath(m_ppObjects, pPath);
      pNewObj->SetSelected(true, false, -1, pRegions);
      Add(pNewObj);
      delete pPath;
    }

    n = pSelObjs->GetCount();
    for(i = 0; i < n; i++) Remove(pSelObjs->GetItem(i), false);
  }

  delete pPaths;
  delete pSelObjs;

  return iRes;
}

bool CDataList::BreakSelObjects(PDRect pRect, PDPtrList pRegions)
{
  PDObject pObj; //, pNewObj;
  //int iParts;
  int iLen = m_iDataLen;
  bool bRes = false;

  for(int i = 0; i < iLen; i++)
  {
    pObj = m_ppObjects[i];
    if(pObj->GetSelected())
    {
      /*iParts = pObj->GetNumParts();
      for(int j = 0; j < iParts; j++)
      {
        pNewObj = pObj->SplitPart(pRect, pRegions);
        if(pNewObj)
        {
          Add(pNewObj);
          bRes = true;
        }
      }*/
    }
  }
  return bRes;
}

