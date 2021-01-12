#ifndef _DDRAWTYPES_HPP_
#define _DDRAWTYPES_HPP_

#include "DDataTypes.hpp"
#include <stdio.h>
#include "DParser.hpp"

class CDObject;

enum CDDrawType
{
  dtLine = 1,
  dtCircle = 2,
  dtEllipse = 3,
  dtArcEllipse = 4,
  dtHyperbola = 5,
  dtParabola = 6,
  dtSpline = 7,
  dtEvolvent = 8,
  dtLogSpiral = 9,
  dtArchSpiral = 10,
  dtPath = 100, // contains consecutive PDPathSeg objects
  dtBorderPath = 101, // border path is a closed path solely constructed to form areas. It does not release its children
  dtBorder = 110, // border should only contain border paths in m_pSubObjects, 1st path is the boundary, the rest are holes
  dtArea = 111, // m_pSubObjects contain one or more dtBorder objects
  dtGroup = 120 // contains dtArea, dtPath or any of the base types
};

enum CDDrawSubType
{
  dstNone = 0,
  dstRectangle = 1
};

typedef struct CDPathSeg
{
  bool bReverse;
  CDObject *pSegment;
} *PDPathSeg;

typedef class CDObject
{
private:
  CDDrawType m_iType;
  CDLine m_cLines[2];
  CDRefPoint m_cBounds[2];
  PDPointList m_pInputPoints;
  PDPointList m_pUndoPoints;
  PDPointList m_pCachePoints;
  PDRefList m_pCrossPoints;
  PDPtrList m_pDimens;
  int m_iPrimitiveRequested;

  PDPrimObject m_pPrimitive;
  bool m_bSelected;
  CDLineStyle m_cLineStyle;
  double m_dMovedDist;
  bool m_bFirstDimSet;
  double m_dFirstDimen;
  char m_iDimenDir;
  char m_sTmpDimBuf[64];
  CDDimension m_cTmpDim;
  bool m_bSnapTo;

  int m_iAuxInt;
  PDPtrList m_pSubObjects;

  bool IsClosedShape();
  int GetBoundType();
  bool BuildSubCache(CDLine cTmpPt, int iMode);
  void SavePoint(FILE *pf, bool bSwapBytes, CDPoint cPoint);
  void SaveInputPoint(FILE *pf, bool bSwapBytes, CDInputPoint cInPoint);
  void SaveReference(FILE *pf, bool bSwapBytes, double dRef);
  void SaveRefPoint(FILE *pf, bool bSwapBytes, CDRefPoint cRefPoint);
  void SaveLine(FILE *pf, bool bSwapBytes, CDLine cLine);
  void SaveLineStyle(FILE *pf, bool bSwapBytes, CDLineStyle cLineStyle, unsigned char cVersion);
  void SaveDimension(FILE *pf, bool bSwapBytes, PDDimension pDim, unsigned char cVersion);
  void LoadPoint(FILE *pf, bool bSwapBytes, PDPoint pPoint);
  void LoadInputPoint(FILE *pf, bool bSwapBytes, PDInputPoint pInPoint);
  void LoadReference(FILE *pf, bool bSwapBytes, double *pdRef);
  void LoadRefPoint(FILE *pf, bool bSwapBytes, PDRefPoint pRefPoint);
  void LoadLine(FILE *pf, bool bSwapBytes, PDLine pLine);
  void LoadLineStyle(FILE *pf, bool bSwapBytes, PDLineStyle pLineStyle, unsigned char cVersion);
  void LoadDimension(FILE *pf, bool bSwapBytes, PDDimension pDim, unsigned char cVersion);
  //void AddCurveSegment(double dStart, double dEnd, PDRect pRect);
  // cAddMode:
  //   iType: mask: 0 - do not insert path marks, 1 - insert path marks, 2 - cPt1.x contains lower bound,
  //     4 - cPt1.y contains upper bound, 8 - move current point
  //   cPt1.x - start, cPt1.y - end
  //   cPt2.x - pattern scale, 0.0 for no pattern
  //   cPt2.y - pattern length
  //   cPt3.x - pattern offset
  //   cPt3.y - pattern origin
  //   cPt4.x - 0 : 1 = open : closed
  //   cPt4.y - length of the closed curve
  void AddCurveSegment(CDPrimitive cAddMode, PDPrimObject pPrimitive, PDRefList pBounds);
  //void AddPatSegment(double dStart, int iStart, double dEnd, int iEnd,
  //  PDPoint pBnds, PDRect pRect);
  // iBoundMode - mask: 1 closed, 2 move point
  void AddPatSegment(double dStart, int iStart, double dEnd, int iEnd,
    int iBoundMode, PDRefList pViewBnds, PDPoint pBnds);
  bool GetRefBounds(PDPoint pPoint);
  int GetDimenDir(double dRef1, double dRef2);
  double GetDimenMidPointRef(double dRef1, double dRef2, int iDir);
  bool GetNativeReference(double dDist, double *pdRef);
  bool GetBounds(PDPoint pBounds);
  double GetLength();
  PDPathSeg GetPathRefSegment(double dRef, double *pdSegRef, int *piPos);
  bool GetPathRefPoint(double dRef, double dOffset, PDPoint pPt);
  bool GetNativeRefPoint(double dRef, double dOffset, PDPoint pPt);
  bool GetPathRefDir(double dRef, PDPoint pPt);
  bool GetNativeRefDir(double dRef, PDPoint pPt);
  bool IsValidRef(double dRef);
  bool BoundPoint(CDPoint cRefPt, PDLine pPtX, double *pdDist);
  int AddDimenPrimitive(int iPos, PDDimension pDim, PDPrimObject pPrimitive, PDRect pRect);
  void SwapBounds();
  bool RemovePart(bool bDown, PDRefPoint pBounds);
  bool IsClosedPath();
  int AddSubIntersects(CDPoint cPt1, CDPoint cPt2, double dOffset, PDRefList pBounds);
  int AddLineIntersects(CDPoint cPt1, CDPoint cPt2, double dOffset, PDRefList pBounds);
  int GetRectangleIntersects(PDRect pRect, double dOffset, int iBndMode, PDPoint pRefBnds, PDRefList pBounds);
  // returns: 0 - nothing is visible, 1 - part is visible, 2 - whole curve is visible and is closed
  void AddExtraPrimitives(PDRect pRect, PDPrimObject pPrimList);
  //int BuildPurePrimitives(CDLine cTmpPt, int iMode, PDRect pRect, int iTemp, PDPrimObject plPrimitive,
  //  double dExt, double *pdMovedDist, PDPoint pBnds);
  //int BuildPathPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, int iTemp, PDPrimObject plPrimitive,
  //  double dExt, double *pdMovedDist, PDPoint pBnds);
  double GetPathDistFromPt(CDPoint cPt, CDPoint cRefPt, bool bSnapCenters, PDLine pPtX);
  int GetSimpleViewBounds(CDLine cTmpPt, int iMode, CDLineStyle cStyle, PDRect pRect, PDRefList pBounds,
    PDPoint pDrawBnds, bool bMergeWithBounds);
  int GetPathViewBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds);
  void AddSimpleSegment(double d1, double d2, double dExt, bool bReverse, PDPrimObject pPrimList);
  void AddPathSegment(double d1, double d2, double dExt, PDPrimObject pPrimList);
public:
  CDObject(CDDrawType iType, double dWidth);
  ~CDObject();
  bool AddPoint(double x, double y, char iCtrl, double dRestrictVal, bool bFromGui); // returns true if the point is the last point
  void RemoveLastPoint();
  void Undo();
  void Redo();
  // iMode: 0 - normal, 1 - inserting, 2 - buffering, 3 - rounding
  bool BuildCache(CDLine cTmpPt, int iMode);
  int GetViewBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds, bool bMergeWithBounds);
  // returns 0 - not in rect, 1 - partially in rect, 2 - full in rect
  int BuildPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, int iTemp, PDFileAttrs pAttrs);
  void GetFirstPrimitive(PDPrimitive pPrim, double dScale, int iDimen);
  void GetNextPrimitive(PDPrimitive pPrim, double dScale, int iDimen);
  int GetAttractors(CDPoint cPt, double dScale, PDPointList pPoints);
  int GetBSplineParts();
  // returns true if spline parts is equal to 1, and the spline is closed
  bool GetBSplines(int iParts, double dScale, int *piCtrls, double **ppdKnots, PDPoint *ppPoints);
  bool IsNearPoint(CDPoint cPt, double dTolerance, int *piDimen);
  bool GetSelected();
  void SetSelected(bool bSelect, bool bInvert, int iDimen, PDPtrList pRegions);
  int GetType();
  CDLine GetLine();
  CDLine GetCircle();
  void SetInputLine(int iIndex, CDLine cLine);
  bool HasEnoughPoints();
  bool GetPoint(int iIndex, char iCtrl, PDInputPoint pPoint);
  void SetPoint(int iIndex, char iCtrl, CDInputPoint cPoint);
  double GetOffset();
  double GetDistFromPt(CDPoint cPt, CDPoint cRefPt, bool bSnapCenters, PDLine pPtX, int *piDimen);
  CDLineStyle GetLineStyle();
  void SetLineStyle(int iMask, CDLineStyle cStyle);
  bool GetRestrictPoint(CDPoint cPt, int iMode, bool bRestrictSet, double dRestrictValue,
      PDPoint pSnapPt);
  CDObject* Copy();
  bool Split(CDPoint cPt, double dDist, PDRect pRect, CDObject** ppNewObj, PDPtrList pRegions);
  bool Extend(CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions);
  void SetBound(int iIndex, CDLine cBound);
  void SetBound(int iIndex, CDRefPoint cBound);
  void SaveToFile(FILE *pf, bool bSwapBytes, unsigned char cVersion);
  bool ReadFromFile(FILE *pf, bool bSwapBytes, unsigned char cVersion);
  // pPtX.bIsSet true if center is set, pPtX.cOrigin = center, pPtX.cDirection = normal
  double GetRadiusAtPt(CDLine cPtX, PDLine pPtR, bool bNewPt);
  bool GetDynValue(CDPoint cPt, int iMode, double *pdVal);
  void BuildRound(CDObject *pObj1, CDObject *pObj2, CDPoint cPt, bool bRestSet, double dRad);
  // iDimFlag 0 - don't move dims, 1 - move only selected, 2 - move all dims
  bool RotatePoints(CDPoint cOrig, double dRot, int iDimFlag);
  bool MovePoints(CDPoint cDir, double dDist, int iDimFlag);
  void MirrorPoints(CDLine cLine);
  bool AddCrossPoint(CDPoint cPt, double dDist);
  bool AddCrossPoint(double dRef);
  bool GetPointRefDist(double dRef, double *pdDist);
  double GetNearestCrossPoint(CDPoint cPt, PDPoint pPt);
  double GetNearestBoundPoint(CDPoint cPt, PDPoint pPt);
  bool AddDimen(CDPoint cPt, double dDist, PDRect pRect, PDFileAttrs pAttrs);
  void DiscardDimen();
  void AddDimenPtr(PDDimension pDimen);
  int GetDimenCount();
  PDDimension GetDimen(int iPos);
  int PreParseDimText(int iPos, char *sBuf, int iBufLen, double dScale, PDUnitList pUnits);
  void GetDimFontAttrs(int iPos, PDFileAttrs pAttrs);
  bool DeleteSelDimens(PDRect pRect, PDPtrList pRegions);
  bool GetSelectedDimen(PDDimension pDimen);
  bool SetSelectedDimen(PDDimension pDimen, PDPtrList pRegions);
  bool GetSnapTo();
  void SetSnapTo(bool bSnap);
  void AddRegions(PDPtrList pRegions, int iPrimType);
  int GetUnitMask(int iUnitType, char *psBuf, PDUnitList pUnits);
  bool ChangeUnitMask(int iUnitType, char *psMask, PDUnitList pUnits);
  void Rescale(double dRatio, bool bWidths, bool bPatterns, bool bArrows, bool bLabels);
  CDPoint GetPointToDir(CDPoint cPoint, double dAngle, CDPoint cPtTarget);
  void SetAuxInt(int iVal);
  int GetAuxInt();
  int GetNumParts();
  CDObject* SplitPart(PDRect pRect, PDPtrList pRegions);
  int IsClosed();
  bool IsBoundShape();
  bool GetStartPoint(PDPoint pPt);
  bool GetEndPoint(PDPoint pPt);
  void BuildPath(CDObject **ppObjects, PDIntList pPath);
  int GetSubObjectCount();
} *PDObject;

typedef class CDataList
{
private:
  int m_iDataLen;
  int m_iDataSize;
  PDObject *m_ppObjects;
  CDFileAttrs m_cFileAttrs;
  bool m_bHasChanged;
  int GetTangSnap(CDPoint cPt, double dDist, bool bNewPt, PDLine pSnapPt, PDObject pObj, PDObject pDynObj);
  int GetNextSeg(PDIntList pSelObjs, PDIntList pPath, CDPoint cPt1, CDPoint cPt2, PDPoint pPt1, PDPoint pPt2, bool *pbFirst);
  bool BuildPath(PDIntList pSelObjs, PDIntList pSel2, PDIntList pPath);
  bool BuildPaths(PDIntList pSelObjs, PDPtrList pPaths);
public:
  CDataList();
  ~CDataList();
  int GetCount();
  void Add(PDObject pObject);
  void Remove(int iIndex, bool bFree);
  PDObject GetItem(int iIndex);
  void BuildAllPrimitives(PDRect pRect, bool bResolvePatterns = true);
  PDObject SelectByPoint(CDPoint cPt, double dDist, int *piDimen);
  PDObject SelectLineByPoint(CDPoint cPt, double dDist);
  void ClearSelection(PDPtrList pRegions);
  int GetNumOfSelectedLines();
  PDObject GetSelectedLine(int iIndex);
  int GetNumOfSelectedCircles();
  PDObject GetSelectedCircle(int iIndex);
  int GetSnapPoint(int iSnapMask, CDPoint cPt, double dDist, PDLine pSnapPt, PDObject pDynObj);
  bool DeleteSelected(CDataList *pUndoList, PDRect pRect, PDPtrList pRegions);
  int GetSelectCount(unsigned char cVersion);
  PDObject GetSelected(int iIndex);
  bool CutSelected(CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions);
  bool ExtendSelected(CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions);
  void ClearAll();
  void SaveToFile(FILE *pf, bool bSwapBytes, bool bSelectOnly, unsigned char cVersion);
  bool ReadFromFile(FILE *pf, bool bSwapBytes, bool bClear);
  void SelectByRectangle(PDRect pRect, int iMode, PDPtrList pRegions);
  bool RotateSelected(CDPoint cOrig, double dRot, int iCop, PDRect pRect, PDPtrList pRegions);
  bool MoveSelected(CDLine cLine, double dDist, int iCop, PDRect pRect,
      bool bPreserveDir, PDPtrList pRegions);
  bool MirrorSelected(CDLine cLine, PDRect pRect, PDPtrList pRegions);
  int GetSelectedLineStyle(PDLineStyle pStyle);
  bool SetSelectedLineStyle(int iMask, PDLineStyle pStyle, PDPtrList pRegions);
  bool SetCrossSelected(CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions);
  bool AddDimen(PDObject pSelForDiment, CDPoint cPt, double dDist, PDRect pRect, PDPtrList pRegions);
  bool GetChanged();
  void SetChanged();
  void SetFileAttrs(PDFileAttrs pFileAttrs, bool bNewFile);
  void GetFileAttrs(PDFileAttrs pFileAttrs);
  bool GetSelectedDimen(PDDimension pDimen);
  bool SetSelectedDimen(PDDimension pDimen, PDPtrList pRegions);
  void GetStatistics(int *piStats);
  // returns:
  // 0 - success, -1 - no mask found
  int GetUnitMask(int iUnitType, char *psBuf, PDUnitList pUnits);
  void ChangeUnitMask(int iUnitType, char *psMask, PDUnitList pUnits);
  void RescaleDrawing(double dNewScaleNom, double dNewScaleDenom, bool bWidths, bool bPatterns,
      bool bArrows, bool bLabels);
  bool GetSelSnapEnabled();
  void SetSelSnapEnabled(bool bEnable);
  int CreatePath();
  bool BreakSelObjects(PDRect pRect, PDPtrList pRegions);
} *PDataList;

#endif

