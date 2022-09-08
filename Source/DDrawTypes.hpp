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
  dtRect = 11,
  dtPath = 100, // contains consecutive PDPathSeg objects
  //dtBorderPath = 101, // border path is a closed path solely constructed to form areas. It does not release its children
  //dtBorder = 110, // border should only contain border paths in m_pSubObjects, 1st path is the boundary, the rest are holes
  dtBorder = 110, // border contains paths or closed shapes in m_pSubObjects, 1st path is the boundary, the rest are holes
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
  void SaveLineStyle(FILE *pf, bool bSwapBytes, CDLineStyle cLineStyle, unsigned char cVersion);
  int GetLineStyleStreamSize(unsigned char cVersion);
  int SaveLineStyleToStream(unsigned char *pBuf, CDLineStyle cLineStyle, unsigned char cVersion);
  void LoadLineStyle(FILE *pf, bool bSwapBytes, PDLineStyle pLineStyle, unsigned char cVersion);
  int LoadLineStyleFromStream(unsigned char *pBuf, PDLineStyle pLineStyle, unsigned char cVersion);

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
  void AddPatSegment(double dStart, int iStart, double dEnd, int iEnd,
    int iBoundMode, PDRefList pViewBnds, PDPoint pBnds, PDPrimObject pPrimitives);
  int GetRefBounds(PDPoint pPoint);
  int GetDimenDir(double dRef1, double dRef2);
  double GetDimenMidPointRef(double dRef1, double dRef2, int iDir);
  bool GetNativeReference(double dDist, double dOffset, double *pdRef);
  // GetBounds = mask: 1 .. pBounds->x is set, 2 .. pBounds->y is set
  int GetBounds(PDPoint pBounds, double dOffset, bool bAdvancePeriod);
  int GetBoundsRef(PDPoint pBounds, double dOffset, bool bAdvancePeriod);
  PDPathSeg GetPathRefSegment(double dRef, double dOffset, double *pdSegRef, int *piPos);
  bool GetPathRefPoint(double dRef, double dOffset, PDPoint pPt);
  bool GetNativeRefPoint(double dRef, double dOffset, PDPoint pPt);
  bool GetPathRefDir(double dRef, PDPoint pPt);
  bool GetNativeRefDir(double dRef, PDPoint pPt);
  bool IsValidRef(double dRef);
  bool BoundPoint(CDPoint cRefPt, PDLine pPtX, double *pdDist);
  int AddDimenPrimitive(int iPos, PDDimension pDim, PDPrimObject pPrimitive, PDRect pRect);
  void SwapBounds();
  bool IsClosedPath();
  int AddSubIntersects(CDPoint cPt1, CDPoint cPt2, double dOffset, PDRefList pBounds);
  int AddLineIntersects(CDPoint cPt1, CDPoint cPt2, double dOffset, PDRefList pBounds);
  int GetRectangleIntersects(PDRect pRect, double dOffset, int iBndMode, PDPoint pRefBnds, PDRefList pBounds);
  // returns: 0 - nothing is visible, 1 - part is visible, 2 - whole curve is visible and is closed
  void AddExtraPrimitives(PDRect pRect, PDPrimObject pPrimList);
  double GetPathDistFromPt(CDPoint cPt, CDPoint cRefPt, bool bSnapCenters, PDLine pPtX);
  double GetAreaDistFromPt(CDPoint cPt, PDLine pPtX);
  double GetGroupDistFromPt(CDPoint cPt, PDLine pPtX);
  int GetSimpleViewBounds(CDLine cTmpPt, int iMode, double dOffset, double dLineHalfWidth,
    PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds, bool bMergeWithBounds);
  int GetPathViewBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds, bool bMergeWithBounds);
  int GetAreaViewBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds, bool bMergeWithBounds);
  int GetGroupViewBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds, bool bMergeWithBounds);
  void AddSimpleSegment(double dt1, double dt2, double dExt, bool bReverse, PDPrimObject pPrimList);
  void AddPathSegment(double d1, double d2, double dExt, PDPrimObject pPrimList);
  void AddAreaSegment(double d1, double d2, double dExt, PDPrimObject pPrimList);
  void AddGroupSegment(double d1, double d2, double dExt, PDPrimObject pPrimList);
  int GetPointReferences(CDPoint cPt, PDRefList pRefs);
  CDObject* SplitByRef(double dRef, bool *pbRes);
  double GetMovedDist(CDLine cTmpPt, int iMode);
  void AddSegmentToPath(PDPathSeg pNewSeg, bool bInsert);
  void SetupRectCache();
  void UpdateRectCache(CDLine cTmpPt);
  void UpdatePathCache();
  bool GetPathRestrictPoint(CDPoint cPt, int iMode, double dRestrictValue, PDPoint pSnapPt);
  double GetRawDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSearchMask, PDLine pPtX);
  int MergeWithBounds(PDRefList pInt1);
  double GetPathMovedRef(double dDist, double dRef);
  double GetMovedRef(int iMode, double dRef);
  CDPrimitive GetPathBBOX();
  int PtInPathArea(CDPoint cPt);
  int PtInArea(CDPoint cPt);
  bool ContainsObject(CDObject *pObj);
  int BuildAreaPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, PDFileAttrs pAttrs);
  int BuildGroupPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, PDFileAttrs pAttrs);
public:
  CDObject(CDDrawType iType, double dWidth);
  ~CDObject();
  bool AddPoint(double x, double y, char iCtrl, double dRestrictVal); // returns true if the point is the last point
  void RemoveLastPoint();
  void Undo();
  void Redo();
  // iMode: 0 - normal, 1 - inserting, 2 - buffering, 3 - rounding
  bool BuildCache(CDLine cTmpPt, int iMode);
  int GetViewBounds(CDLine cTmpPt, int iMode, PDRect pRect, PDRefList pBounds, PDPoint pDrawBnds, bool bMergeWithBounds);
  // returns 0 - not in rect, 1 - partially in rect, 2 - full in rect
  int BuildPrimitives(CDLine cTmpPt, int iMode, PDRect pRect, int iTemp, PDFileAttrs pAttrs, PDPrimObject pPrimitives);
  void GetFirstPrimitive(PDPrimitive pPrim, double dScale, int iDimen);
  void GetNextPrimitive(PDPrimitive pPrim, double dScale, int iDimen);
  int GetAttractors(CDPoint cPt, double dScale, PDPointList pPoints);
  int GetBSplineParts();
  // returns true if spline parts is equal to 1, and the spline is closed
  bool GetBSplines(int iParts, double dScale, int *piCtrls, double **ppdKnots, PDPoint *ppPoints);
  bool IsNearPoint(CDPoint cPt, double dTolerance, int *piDimen);
  bool GetSelected();
  void SetSelected(bool bSelect, bool bInvert, int iDimen);
  int GetType();
  CDLine GetLine();
  CDLine GetCircle();
  void SetInputLine(int iIndex, CDLine cLine);
  bool HasEnoughPoints();
  bool GetPoint(int iIndex, char iCtrl, PDInputPoint pPoint);
  void SetPoint(int iIndex, char iCtrl, CDInputPoint cPoint);
  double GetLength(double dOffset);
  double GetOffset();
  double GetDistFromPt(CDPoint cPt, CDPoint cRefPt, int iSearchMask, PDLine pPtX, int *piDimen);
  CDLineStyle GetLineStyle();
  void SetLineStyle(int iMask, CDLineStyle cStyle);
  int GetRestrictPoint(CDPoint cPt, int iMode, int iRestrictMask, double *pdRestrictValue, PDPoint pSnapPt);
  CDObject* Copy();
  bool Split(CDPoint cPt, PDPtrList pNewObjects, PDRect pRect);
  bool Extend(CDPoint cPt, double dDist, PDRect pRect);
  void SetBound(int iIndex, CDLine cBound);
  void SetBound(int iIndex, CDRefPoint cBound);
  void SaveToFile(FILE *pf, bool bSwapBytes, unsigned char cVersion);
  bool ReadFromFile(FILE *pf, bool bSwapBytes, unsigned char cVersion);
  int GetStreamSize(unsigned char cVersion);
  int SaveToStream(unsigned char *pBuf, unsigned char cVersion);
  int ReadFromStream(unsigned char *pBuf, unsigned char cVersion);
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
  bool GetPointRefDist(double dRef, double dOffset, double *pdDist);
  double GetNearestCrossPoint(CDPoint cPt, PDPoint pPt);
  double GetNearestBoundPoint(CDPoint cPt, PDPoint pPt);
  bool AddDimen(CDPoint cPt, double dDist, PDRect pRect, PDFileAttrs pAttrs);
  void DiscardDimen();
  void AddDimenPtr(PDDimension pDimen);
  int GetDimenCount();
  PDDimension GetDimen(int iPos);
  int PreParseDimText(int iPos, char *sBuf, int iBufLen, double dScale, PDUnitList pUnits);
  void GetDimFontAttrs(int iPos, PDFileAttrs pAttrs);
  bool DeleteSelDimens(PDRect pRect);
  bool GetSelectedDimen(PDDimension pDimen);
  bool SetSelectedDimen(PDDimension pDimen);
  bool GetSnapTo();
  void SetSnapTo(bool bSnap);
  int GetUnitMask(int iUnitType, char *psBuf, PDUnitList pUnits);
  bool ChangeUnitMask(int iUnitType, char *psMask, PDUnitList pUnits);
  void Rescale(double dRatio, bool bWidths, bool bPatterns, bool bArrows, bool bLabels);
  CDPoint GetPointToDir(CDPoint cPoint, double dAngle, CDPoint cPtTarget);
  void SetAuxInt(int iVal);
  int GetAuxInt();
  int IsClosed();
  bool IsBoundShape();
  bool GetStartPoint(PDPoint pPt, double dOffset);
  bool GetEndPoint(PDPoint pPt, double dOffset);
  void BuildPath(CDObject **ppObjects, PDIntList pPath, PDLineStyle pStyle);
  int GetSubObjectCount(bool bCountSubObjects);
  CDObject* GetSubObject(int iIndex);
  void AddSubObject(CDObject* pObj);
  void ClearSubObjects(bool bDelete);
  int GetAreaObjectCount();
  CDObject* GetAreaObject(int iIndex);
  // returns: 0 - no snap point, 1 - on the curve, 2 - on the edge, 3 - on the cross point, 4 - on self intersection,
  // 5 - on a control point
  // pSnapPt must be array of two
  int GetSnapPoint(int iSnapMask, CDPoint cPt, double dDist, PDLine pSnapPt, CDObject* pDynObj, bool bHonorSnapTo);
  double GetCircleRadius();
  CDObject* GetPathObject(int iIndex);
  void ClearPath(bool bFreeSubObjects);
  void ClearArea(bool bFreeSubObjects);
  CDPrimitive GetBBOX();
  void BuildArea(PDPtrList pBoundaries, PDLineStyle pStyle);
  void ChangeToPath();
} *PDObject;

typedef class CDataList
{
private:
  int m_iDataLen;
  int m_iDataSize;
  PDObject *m_ppObjects;
  CDFileAttrs m_cFileAttrs;
  bool m_bHasChanged;
  int GetNextSeg(PDIntList pSelObjs, PDIntList pPath, CDPoint cPt1, CDPoint cPt2, PDPoint pPt1, PDPoint pPt2, bool *pbFirst);
  bool BuildPath(PDIntList pSelObjs, PDIntList pSel2, PDIntList pPath);
  bool BuildPaths(PDIntList pSelObjs, PDPtrList pPaths);
public:
  CDataList();
  ~CDataList();
  int GetCount();
  void Add(PDObject pObject);
  void Remove(int iIndex, bool bFree);
  void Insert(int iIndex, PDObject pObject);
  PDObject GetItem(int iIndex);
  void BuildAllPrimitives(PDRect pRect, bool bResolvePatterns = true);
  PDObject SelectByPoint(CDPoint cPt, double dDist, int *piDimen);
  PDObject SelectLineByPoint(CDPoint cPt, double dDist);
  void ClearSelection();
  int GetNumOfSelectedLines();
  PDObject GetSelectedLine(int iIndex);
  int GetNumOfSelectedCircles();
  PDObject GetSelectedCircle(int iIndex);
  int GetSnapPoint(int iSnapMask, CDPoint cPt, double dDist, PDLine pSnapPt, PDObject pDynObj);
  bool DeleteSelected(CDataList *pUndoList, PDRect pRect);
  int GetSelectCount(unsigned char cVersion);
  PDObject GetSelected(int iIndex);
  bool CutSelected(CDPoint cPt, double dDist, PDRect pRect);
  bool ExtendSelected(CDPoint cPt, double dDist, PDRect pRect);
  void ClearAll();
  void SaveToFile(FILE *pf, bool bSwapBytes, bool bSelectOnly, unsigned char cVersion);
  bool ReadFromFile(FILE *pf, bool bSwapBytes, bool bClear);
  int GetStreamSize(unsigned char cVersion);
  void SaveToStream(unsigned char *pBuf, unsigned char cVersion);
  bool ReadFromStream(unsigned char *pBuf, unsigned char cVersion);
  void SelectByRectangle(PDRect pRect, int iMode);
  bool RotateSelected(CDPoint cOrig, double dRot, int iCop, PDRect pRect);
  bool MoveSelected(CDLine cLine, double dDist, int iCop, PDRect pRect, bool bPreserveDir);
  bool MirrorSelected(CDLine cLine, PDRect pRect);
  int GetSelectedLineStyle(PDLineStyle pStyle);
  bool SetSelectedLineStyle(int iMask, PDLineStyle pStyle);
  bool SetCrossSelected(CDPoint cPt, double dDist, PDRect pRect);
  bool AddDimen(PDObject pSelForDiment, CDPoint cPt, double dDist, PDRect pRect);
  bool GetChanged();
  void SetChanged();
  void SetFileAttrs(PDFileAttrs pFileAttrs, bool bNewFile);
  void GetFileAttrs(PDFileAttrs pFileAttrs);
  bool GetSelectedDimen(PDDimension pDimen);
  bool SetSelectedDimen(PDDimension pDimen);
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
  bool BreakSelObjects();
  bool CreateArea();
  int GetSelectedLength(double *pdLength);
  bool Group();
  bool Ungroup();
  bool MoveUp();
  bool MoveDown();
  bool MoveTop();
  bool MoveBottom();
  // returns:
  // 0 - success, 1 - either no path selected or more than 1 path selected,
  // 2 - unbound object selected, 3 - unbound path selected to distribute about,
  // 4 - closed object to distribute in rubber mode
  int Distribute(int iCopies, bool bKeepOrient, CDPoint cPt);
} *PDataList;

#endif

