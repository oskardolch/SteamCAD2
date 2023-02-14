#ifndef _DRASTER_HPP_
#define _DRASTER_HPP_

#include <stdio.h>
#include "DDataTypes.hpp"

typedef struct CDRasterCache
{
  CDPoint cMatrixRow1;
  CDPoint cMatrixRow2;
  CDPoint cTranslate;
  int iImageWidth;
  int iImageHeight;
  int iFileSize;
  unsigned char *pData;
} *PDRasterCache;

bool AddRasterPoint(double x, double y, char iCtrl, PDPointList pPoints);
bool BuildRasterCacheRaw(PDPointList pPoints, PDRasterCache pCache, int iWidth, int iHeight, FILE *pf);
double GetRasterDistFromPt(CDPoint cPt, PDRasterCache pCache, PDLine pPtX);
bool RegisterRasterRaw(PDPointList pPoints, PDRasterCache pCache, PDPoint pRegPoints);
int GetRasterViewBounds(PDRect pRect, PDRasterCache pCache);

#endif
