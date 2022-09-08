#ifndef _DSTREAMUTILS_HPP_
#define _DSTREAMUTILS_HPP_

#include "DDataTypes.hpp"
#include <stdio.h>

void SwapBytes(unsigned char *pDest, unsigned char *pSrc, int iLen, bool bSwap);

void SavePoint(FILE *pf, bool bSwapBytes, CDPoint cPoint);
void SaveInputPoint(FILE *pf, bool bSwapBytes, CDInputPoint cInPoint);
void SaveReference(FILE *pf, bool bSwapBytes, double dRef);
void SaveRefPoint(FILE *pf, bool bSwapBytes, CDRefPoint cRefPoint);
void SaveLine(FILE *pf, bool bSwapBytes, CDLine cLine);
void SaveDimension(FILE *pf, bool bSwapBytes, PDDimension pDim, unsigned char cVersion);

int GetPointStreamSize();
int GetInputPointStreamSize();
int GetRefPointStreamSize();
int GetLineStreamSize();
int GetDimensionStreamSize(PDDimension pDim, unsigned char cVersion);

int SavePointToStream(unsigned char *pBuf, CDPoint cPoint);
int SaveInputPointToStream(unsigned char *pBuf, CDInputPoint cInPoint);
int SaveReferenceToStream(unsigned char *pBuf, double dRef);
int SaveRefPointToStream(unsigned char *pBuf, CDRefPoint cRefPoint);
int SaveLineToStream(unsigned char *pBuf, CDLine cLine);
int SaveDimensionToStream(unsigned char *pBuf, PDDimension pDim, unsigned char cVersion);

void LoadPoint(FILE *pf, bool bSwapBytes, PDPoint pPoint);
void LoadInputPoint(FILE *pf, bool bSwapBytes, PDInputPoint pInPoint);
void LoadReference(FILE *pf, bool bSwapBytes, double *pdRef);
void LoadRefPoint(FILE *pf, bool bSwapBytes, PDRefPoint pRefPoint);
void LoadLine(FILE *pf, bool bSwapBytes, PDLine pLine);
void LoadDimension(FILE *pf, bool bSwapBytes, PDDimension pDim, unsigned char cVersion);

int LoadPointFromStream(unsigned char *pBuf, PDPoint pPoint);
int LoadInputPointFromStream(unsigned char *pBuf, PDInputPoint pInPoint);
int LoadRefPointFromStream(unsigned char *pBuf, PDRefPoint pRefPoint);
int LoadLineFromStream(unsigned char *pBuf, PDLine pLine);
int LoadDimensionFromStream(unsigned char *pBuf, PDDimension pDim, unsigned char cVersion);

#endif

