#include "DStreamUtils.hpp"
#include "DMath.hpp"
#include <malloc.h>
#include <string.h>

void SwapBytes(unsigned char *pDest, unsigned char *pSrc, int iLen, bool bSwap)
{
  if(bSwap)
  {
    for(int i = 0; i < iLen; i++) pDest[i] = pSrc[iLen - i - 1];
  }
  else for(int i = 0; i < iLen; i++) pDest[i] = pSrc[i];
}

void SavePoint(FILE *pf, bool bSwapBytes, CDPoint cPoint)
{
  unsigned char buf[16], *pbuf;

  pbuf = (unsigned char*)&cPoint.x;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);

  pbuf = (unsigned char*)&cPoint.y;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);
}

void SaveInputPoint(FILE *pf, bool bSwapBytes, CDInputPoint cInPoint)
{
  unsigned char buf = cInPoint.iCtrl;
  fwrite(&buf, 1, 1, pf);
  SavePoint(pf, bSwapBytes, cInPoint.cPoint);
}

void SaveReference(FILE *pf, bool bSwapBytes, double dRef)
{
  unsigned char buf[16], *pbuf;
  pbuf = (unsigned char*)&dRef;
  SwapBytes(buf, pbuf, 8, bSwapBytes);
  fwrite(buf, 1, 8, pf);
}

void SaveRefPoint(FILE *pf, bool bSwapBytes, CDRefPoint cRefPoint)
{
  unsigned char buf = cRefPoint.bIsSet;
  fwrite(&buf, 1, 1, pf);
  SaveReference(pf, bSwapBytes, cRefPoint.dRef);
}

void SaveLine(FILE *pf, bool bSwapBytes, CDLine cLine)
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

void SaveDimension(FILE *pf, bool bSwapBytes, PDDimension pDim, unsigned char cVersion)
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

int GetPointStreamSize()
{
  return 16;
}

int GetInputPointStreamSize()
{
  return 1 + GetPointStreamSize();
}

int GetRefPointStreamSize()
{
  return 9;
}

int GetLineStreamSize()
{
  return 9 + 2*GetPointStreamSize();
}

int GetDimensionStreamSize(PDDimension pDim, unsigned char cVersion)
{
  int iRes = 2*8 + 3 + 2*GetPointStreamSize() + 10;

  int iLen = strlen(pDim->psFontFace);
  iRes += iLen;

  iRes += GetPointStreamSize() + 12;

  unsigned long ulVal = 0;
  if(pDim->psLab) ulVal = strlen(pDim->psLab);
  if(pDim->psLab) iRes += ulVal;
  return iRes;
}

int SavePointToStream(unsigned char *pBuf, CDPoint cPoint)
{
  memcpy(pBuf, &cPoint.x, 8);
  memcpy(&pBuf[8], &cPoint.y, 8);
  return 16;
}

int SaveInputPointToStream(unsigned char *pBuf, CDInputPoint cInPoint)
{
  pBuf[0] = cInPoint.iCtrl;
  return 1 + SavePointToStream(&pBuf[1], cInPoint.cPoint);
}

int SaveReferenceToStream(unsigned char *pBuf, double dRef)
{
  memcpy(pBuf, &dRef, 8);
  return 8;
}

int SaveRefPointToStream(unsigned char *pBuf, CDRefPoint cRefPoint)
{
  pBuf[0] = cRefPoint.bIsSet;
  return 1 + SaveReferenceToStream(&pBuf[1], cRefPoint.dRef);
}

int SaveLineToStream(unsigned char *pBuf, CDLine cLine)
{
  int iCurPos = 0;

  pBuf[iCurPos++] = cLine.bIsSet;
  iCurPos += SavePointToStream(&pBuf[iCurPos], cLine.cOrigin);
  iCurPos += SavePointToStream(&pBuf[iCurPos], cLine.cDirection);

  memcpy(&pBuf[iCurPos], &cLine.dRef, 8);
  return 8 + iCurPos;
}

int SaveDimensionToStream(unsigned char *pBuf, PDDimension pDim, unsigned char cVersion)
{
  int iCurPos = 0;

  memcpy(&pBuf[iCurPos], &pDim->dRef1, 8);
  iCurPos += 8;

  memcpy(&pBuf[iCurPos], &pDim->dRef2, 8);
  iCurPos += 8;

  pBuf[iCurPos++] = (unsigned char)pDim->iRefDir;
  pBuf[iCurPos++] = (unsigned char)pDim->iArrowType1;
  pBuf[iCurPos++] = (unsigned char)pDim->iArrowType2;

  iCurPos += SavePointToStream(&pBuf[iCurPos], pDim->cArrowDim1);
  iCurPos += SavePointToStream(&pBuf[iCurPos], pDim->cArrowDim2);

  memcpy(&pBuf[iCurPos], &pDim->dFontSize, 8);
  iCurPos += 8;

  pBuf[iCurPos++] = (unsigned char)pDim->bFontAttrs;

  int iLen = strlen(pDim->psFontFace);
  pBuf[iCurPos++] = (unsigned char)iLen;
  memcpy(&pBuf[iCurPos], pDim->psFontFace, iLen);
  iCurPos += iLen;

  iCurPos += SavePointToStream(&pBuf[iCurPos], pDim->cLabelPos.cPoint);

  memcpy(&pBuf[iCurPos], &pDim->cLabelPos.dOrientation, 8);
  iCurPos += 8;

  unsigned long ulVal = 0;
  if(pDim->psLab) ulVal = strlen(pDim->psLab);
  memcpy(&pBuf[iCurPos], &ulVal, 4);
  iCurPos += 4;
  if(pDim->psLab)
  {
    memcpy(&pBuf[iCurPos], pDim->psLab, ulVal);
    iCurPos += ulVal;
  }
  return iCurPos;
}

void LoadPoint(FILE *pf, bool bSwapBytes, PDPoint pPoint)
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

void LoadInputPoint(FILE *pf, bool bSwapBytes, PDInputPoint pInPoint)
{
  unsigned char buf;
  fread(&buf, 1, 1, pf);
  pInPoint->iCtrl = buf;
  LoadPoint(pf, bSwapBytes, &pInPoint->cPoint);
}

void LoadReference(FILE *pf, bool bSwapBytes, double *pdRef)
{
  unsigned char buf[16], *pbuf;

  fread(buf, 1, 8, pf);
  pbuf = (unsigned char*)pdRef;
  SwapBytes(pbuf, buf, 8, bSwapBytes);
}

void LoadRefPoint(FILE *pf, bool bSwapBytes, PDRefPoint pRefPoint)
{
  unsigned char buf;
  fread(&buf, 1, 1, pf);
  pRefPoint->bIsSet = buf;
  pRefPoint->dRef = 0.0;
  LoadReference(pf, bSwapBytes, &pRefPoint->dRef);
}

void LoadLine(FILE *pf, bool bSwapBytes, PDLine pLine)
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

void LoadDimension(FILE *pf, bool bSwapBytes, PDDimension pDim, unsigned char cVersion)
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

int LoadPointFromStream(unsigned char *pBuf, PDPoint pPoint)
{
  memcpy(&pPoint->x, pBuf, 8);
  memcpy(&pPoint->y, &pBuf[8], 8);
  return 16;
}

int LoadInputPointFromStream(unsigned char *pBuf, PDInputPoint pInPoint)
{
  pInPoint->iCtrl = pBuf[0];
  return 1 + LoadPointFromStream(&pBuf[1], &pInPoint->cPoint);
}

int LoadRefPointFromStream(unsigned char *pBuf, PDRefPoint pRefPoint)
{
  pRefPoint->bIsSet = pBuf[0];
  memcpy(&pRefPoint->dRef, &pBuf[1], 8);
  return 9;
}

int LoadLineFromStream(unsigned char *pBuf, PDLine pLine)
{
  int iCurPos = 0;

  pLine->bIsSet = pBuf[iCurPos++];
  pLine->cOrigin.x = 0.0;
  pLine->cOrigin.y = 0.0;
  iCurPos += LoadPointFromStream(&pBuf[iCurPos], &pLine->cOrigin);
  pLine->cDirection.x = 0.0;
  pLine->cDirection.y = 0.0;
  iCurPos += LoadPointFromStream(&pBuf[iCurPos], &pLine->cDirection);

  pLine->dRef = 0.0;
  memcpy(&pLine->dRef, &pBuf[iCurPos], 8);
  return iCurPos + 8;
}

int LoadDimensionFromStream(unsigned char *pBuf, PDDimension pDim, unsigned char cVersion)
{
  int iCurPos = 0;

  memcpy(&pDim->dRef1, &pBuf[iCurPos], 8);
  iCurPos += 8;

  memcpy(&pDim->dRef2, &pBuf[iCurPos], 8);
  iCurPos += 8;

  pDim->iRefDir = pBuf[iCurPos++];
  pDim->iArrowType1 = pBuf[iCurPos++];
  pDim->iArrowType2 = pBuf[iCurPos++];

  pDim->cArrowDim1.x = 0.0;
  pDim->cArrowDim1.y = 0.0;
  iCurPos += LoadPointFromStream(&pBuf[iCurPos], &pDim->cArrowDim1);
  pDim->cArrowDim2.x = 0.0;
  pDim->cArrowDim2.y = 0.0;
  iCurPos += LoadPointFromStream(&pBuf[iCurPos], &pDim->cArrowDim2);

  memcpy(&pDim->dFontSize, &pBuf[iCurPos], 8);
  iCurPos += 8;

  pDim->bFontAttrs = pBuf[iCurPos++];

  unsigned char cLen = pBuf[iCurPos++];
  memcpy(pDim->psFontFace, &pBuf[iCurPos], cLen);
  pDim->psFontFace[cLen] = 0;
  iCurPos += cLen;

  pDim->cLabelPos.cPoint.x = 0.0;
  pDim->cLabelPos.cPoint.y = 0.0;
  iCurPos += LoadPointFromStream(&pBuf[iCurPos], &pDim->cLabelPos.cPoint);

  memcpy(&pDim->cLabelPos.dOrientation, &pBuf[iCurPos], 8);
  iCurPos += 8;

  unsigned long ulVal = 0;
  memcpy(&ulVal, &pBuf[iCurPos], 4);
  iCurPos += 4;

  pDim->psLab = NULL;
  if(ulVal > 0)
  {
    pDim->psLab = (char*)malloc((ulVal + 1)*sizeof(char));
    memcpy(pDim->psLab, &pBuf[iCurPos], ulVal);
    pDim->psLab[ulVal] = 0;
    iCurPos += ulVal;
  }

  pDim->bSelected = false;
  pDim->cExt.cPt1 = 0;
  pDim->cExt.cPt2 = 0;

  return iCurPos;
}

