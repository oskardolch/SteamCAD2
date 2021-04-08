#ifndef _DDATATYPES_HPP_
#define _DDATATYPES_HPP_

#include "DTopo.hpp"

const double g_dMmToIn = 25.4;
const double g_dPtToIn = 72.0;
const double g_dDashMin = 0.001;

typedef struct CDRect
{
  CDPoint cPt1;
  CDPoint cPt2;
} *PDRect;

typedef class CDIntList
{
private:
  int m_iDataLen;
  int m_iDataSize;
  int *m_pData;
public:
  CDIntList();
  ~CDIntList();
  void Clear();
  void AddItem(int iVal);
  void InsertItem(int iPos, int iVal);
  void SetItem(int iPos, int iVal);
  int GetCount();
  int GetIndex(int iVal);
  int GetItem(int iIndex);
  void Remove(int iIndex);
  void RemoveItem(int iVal);
} *PDIntList;

// native reference point, the meaning differs for each curve type
typedef struct CDRefList
{
private:
  int m_iDataLen;
  int m_iDataSize;
  double *m_pPoints;
public:
  CDRefList();
  ~CDRefList();
  void Clear();
  void AddPoint(double dVal);
  void InsertPoint(int iPos, double dVal);
  int GetCount();
  int GetIndex(double dVal);
  double GetPoint(int iIndex);
  void SetPoint(int iIndex, double dVal);
  void Remove(int iIndex);
  void Truncate(int iNewLen);
  bool HasPoint(double dVal);
  void Sort(int iStartIndex);
  double& operator[](int idx){ return m_pPoints[idx]; }
} *PDRefList;

typedef struct CDRefPoint
{
  bool bIsSet;
  double dRef;
} *PDRefPoint;

typedef struct CDLine
{
  bool bIsSet;
  CDPoint cOrigin;
  CDPoint cDirection;
  double dRef;
} *PDLine;

typedef struct CDLineStyle
{
  double dWidth;
  double dPercent;
  char cCapType;
  char cJoinType;
  int iSegments;
  double dPattern[6];
  unsigned char cColor[4];
  double dBlend;
} *PDLineStyle;

enum CDGradMode
{
  gmNone = 0,
  gmLinear = 1,
  gmRadial = 2,
  gmMesh = 3
};

typedef struct CDFillStyle
{
  CDGradMode cGradMode;
} *PDFillStyle;

typedef struct CDInputPoint
{
  char iCtrl;
  CDPoint cPoint;
} *PDInputPoint;

typedef class CDPointList
{
private:
  int m_iDataLen;
  int m_iDataSize;
  PDInputPoint m_pPoints;
  void RawRemove(int iIndex);
public:
  CDPointList();
  ~CDPointList();
  void ClearAll();
  void AddPoint(double x, double y, char iCtrl);
  int GetCount(char iCtrl);
  CDInputPoint GetPoint(int iIndex, char iCtrl);
  void Remove(int iIndex, char iCtrl);
  void SetPoint(int iIndex, char iCtrl, double x, double y, char iNewCtrl);
  //PDInputPoint GetListPtr();
} *PDPointList;

typedef struct CDOrientPoint
{
  CDPoint cPoint;
  double dOrientation;
} *PDOrientPoint;

typedef struct CDDimension
{
  double dRef1;
  double dRef2;
  char iRefDir; // direction with respect to the line referencing system - 0 forward, 1 backward
  char iArrowType1; // 0 - none, 1 - standard, 2 - filled, 3 - point, 4 - slash,
      // 5 - backslash
  char iArrowType2;
  CDPoint cArrowDim1;
  CDPoint cArrowDim2;
  double dFontSize;
  char bFontAttrs;
  char psFontFace[64];
  CDOrientPoint cLabelPos; // position and orientation
  char *psLab;
  CDRect cExt; // temporary structure to hold the label extent. Not saved into file
  bool bSelected;
} *PDDimension;

typedef class CDPtrList
{
private:
  int m_iDataLen;
  int m_iDataSize;
  void **m_pData;
  double m_dValue;
public:
  CDPtrList();
  ~CDPtrList();
  int GetCount();
  void Clear();
  void Add(void *pPtr);
  void* GetItem(int iIndex);
  void* GetList();
  void Remove(int iIndex);
  double GetDblVal();
  void SetDblVal(double dVal);
} *PDPtrList;

typedef struct CDFileAttrs
{
  double dWidth;
  double dHeight;
  double dScaleNom;
  double dScaleDenom;
  int iArrowType;
  CDPoint cArrowDim;
  double dFontSize;
  double dBaseLine;
  char bFontAttrs;
  char sFontFace[64];
  char sLengthMask[64];
  char sAngleMask[64];
} *PDFileAttrs;

typedef struct CDGetBoundsRec
{
  CDLine cTmpPt;
  int iMode;
  PDRect pRect;
  PDPointList pPoints;
  PDPointList pCache;
  PDLine pLines;
  double dOffset;
  double dExt;
  int iRectFlag;
  double *pRectRefs;
  PDPoint pDrawBnds;
} *PDGetBoundsRec;

typedef struct CDPrimitive
{
  int iType; // 1 line, 2 circ arc, 3 circle, 4 quad, 5 bezier, 6 bound points, 7 center points,
    // 8 - cross point, 9 - dim arrow, 10 - dim text, 11 - path
    // 0 no other primitive available
    // in case of path (11) the points have special meaning:
    //   cPt1.x:
    //     0 - do a subpath operation
    //     1 - start new path
    //     2 - stroke (finish) path
    //   cPt1.y:
    //     0 - take no action
    //     1 - start new subpath
    //     2 - close subpath
    // they can be combined and the interpretation is then as follows:
    //   (0, 0) - INVALID
    //   (1, 0) - start a new path without subpath
    //   (2, 0) - stroke path without closing the last subpath (if exists)
    //   (0, 1) - start subpath without closing the previous one (if exists)
    //   (1, 1) - start a new path and immediatelly new subpath
    //   (2, 1) - INVALID
    //   (0, 2) - close subpath and immediately start a new one
    //   (1, 2) - INVALID
    //   (2, 2) - close last subpath and stroke path
    // if cPt1.x == 1 or cPt1.y == 1, cPt2 and cPt3 should contain information of the next start point
    //   cPt2.x > 0 = move current point
    //   cPt3 = new point coordinates
    // if cPt1.x == 2, cPt2 should carry information about dash pattern:
    //   cPt2.x = dash scale, if set to 0, there is no dash pattern applied for this segment
    //   cPt2.y = dash offest
  CDPoint cPt1;
  CDPoint cPt2;
  CDPoint cPt3;
  CDPoint cPt4;
} *PDPrimitive;

typedef class CDPrimObject
{
private:
  PDPrimitive m_pData;
  int m_iDataSize;
  int m_iDataLen;
public:
  CDPrimObject();
  ~CDPrimObject();
  int GetCount();
  void AddPrimitive(CDPrimitive cPrim);
  void InsertPrimitive(int iIndex, CDPrimitive cPrim);
  CDPrimitive GetPrimitive(int iIndex);
  void Clear();
  void Delete(int iIndex);
  void ClearLines();
  void CopyFrom(CDPrimObject *pPrimObj);
} *PDPrimObject;

double Round(double x);
CDPoint Round(CDPoint cp);
CDPoint Abs(CDPoint cp);
CDPoint Mirror(CDPoint cp, CDLine cLine);
double Deter2(CDPoint cp1, CDPoint cp2);
CDPoint MatProd(CDPoint cMat1, CDPoint cMat2, CDPoint cb);
bool Solve2x2Matrix(CDPoint cCol1, CDPoint cCol2, CDPoint cB, PDPoint pSol);
bool Solve2x2MatrixT(CDPoint cRow1, CDPoint cRow2, CDPoint cB, PDPoint pSol);
void SwapPoints(PDPoint pPt1, PDPoint pPt2);
double GetLineProj(CDPoint cPt, CDPoint cOrig, CDPoint cDir, PDPoint pPtX);
bool Solve2ParamSystem(DFunc2D pFunc, DFunc2D pFuncDX, DFunc2D pFuncDY,
    CDPoint cPar1, CDPoint cPar2, CDPoint cInitSol, PDPoint pResult, PDPoint pLowBound);
double GetPtDistFromLineSeg(CDPoint cPt, CDPoint cLp1, CDPoint cLp2, PDLine pPtX);
bool DPtInDRect(CDPoint cPt, PDRect pRect);
void CopyDimenAttrs(PDDimension pDimDst, PDDimension pDimSrc);
// iBndMode flags: 2 - cBnds1.x is the lower bound, 4 - cBnds1.y is the upper bound,
// 8 - curve is potentially closed, dLength is valid then
int IntersectBounds(CDPoint cBnds1, CDPoint cBnds2, int iBndMode, double dLength, PDPoint pRes);
int UnionBounds(PDRefList pBnds1, PDRefList pBnds2, int iBoundMode, PDPoint pDrawBnds);
void MergeCornerRefs(PDRefList pBnds, PDPoint pRefBnds, int iRectFlag, double *pdRefs);

#endif
