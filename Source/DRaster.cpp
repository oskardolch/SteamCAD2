#include "DRaster.hpp"
#include "DMath.hpp"
#include "DPrimitive.hpp"
#include <malloc.h>
#include <math.h>

bool AddRasterPoint(double x, double y, char iCtrl, PDPointList pPoints)
{
  pPoints->AddPoint(x, y, iCtrl);
  return (pPoints->GetCount(0) == 3) && (pPoints->GetCount(1) == 3);
}

bool BuildRasterCacheRaw(PDPointList pPoints, PDRasterCache pCache, int iWidth, int iHeight, FILE *pf)
{
  if(pf)
  {
    pCache->iImageWidth = iWidth;
    pCache->iImageHeight = iHeight;
    fseek(pf, 0L, SEEK_END);
    pCache->iFileSize = ftell(pf);
    pCache->pData = (unsigned char*)malloc(pCache->iFileSize);
    fseek(pf, 0L, SEEK_SET);
    fread(pCache->pData, 1, pCache->iFileSize, pf);
  }

  CDPoint cPt1 = pPoints->GetPoint(0, 0).cPoint;
  CDPoint cPt2 = pPoints->GetPoint(1, 0).cPoint;
  CDPoint cPt3 = pPoints->GetPoint(2, 0).cPoint;
  double dDet = cPt1.x*(cPt2.y - cPt3.y) + cPt2.x*(cPt3.y - cPt1.y) + cPt3.x*(cPt1.y - cPt2.y);
  if(fabs(dDet) < g_dPrec) return false;

  CDPoint3 cMat1, cMat2, cMat3;
  cMat1.x = cPt2.y - cPt3.y;
  cMat1.y = cPt3.y - cPt1.y;
  cMat1.z = cPt1.y - cPt2.y;
  cMat2.x = cPt3.x - cPt2.x;
  cMat2.y = cPt1.x - cPt3.x;
  cMat2.z = cPt2.x - cPt1.x;
  cMat3.x = cPt2.x*cPt3.y - cPt3.x*cPt2.y;
  cMat3.y = cPt3.x*cPt1.y - cPt1.x*cPt3.y;
  cMat3.z = cPt1.x*cPt2.y - cPt2.x*cPt1.y;

  cPt1 = pPoints->GetPoint(0, 1).cPoint;
  cPt2 = pPoints->GetPoint(1, 1).cPoint;
  cPt3 = pPoints->GetPoint(2, 1).cPoint;

  pCache->cMatrixRow1.x = (cMat1.x*cPt1.x + cMat1.y*cPt2.x + cMat1.z*cPt3.x)/dDet;
  pCache->cMatrixRow1.y = (cMat2.x*cPt1.x + cMat2.y*cPt2.x + cMat2.z*cPt3.x)/dDet;
  pCache->cTranslate.x = (cMat3.x*cPt1.x + cMat3.y*cPt2.x + cMat3.z*cPt3.x)/dDet;
  pCache->cMatrixRow2.x = (cMat1.x*cPt1.y + cMat1.y*cPt2.y + cMat1.z*cPt3.y)/dDet;
  pCache->cMatrixRow2.y = (cMat2.x*cPt1.y + cMat2.y*cPt2.y + cMat2.z*cPt3.y)/dDet;
  pCache->cTranslate.y = (cMat3.x*cPt1.y + cMat3.y*cPt2.y + cMat3.z*cPt3.y)/dDet;
  return true;
}

double GetRasterDistFromPt(CDPoint cPt, PDRasterCache pCache, PDLine pPtX)
{
  pPtX->bIsSet = false;
  double dDet = pCache->cMatrixRow1.x*pCache->cMatrixRow2.y - pCache->cMatrixRow1.y*pCache->cMatrixRow2.x;
  if(fabs(dDet) < g_dPrec) return -1.0;
  CDPoint cPt1;
  // it is much better if the raster canbe selected on the boundary only
  /*cPt1.x = (pCache->cMatrixRow2.y*(cPt.x - pCache->cTranslate.x) - pCache->cMatrixRow2.x*(cPt.y - pCache->cTranslate.y))/dDet;
  cPt1.y = (pCache->cMatrixRow1.x*(cPt.y - pCache->cTranslate.y) - pCache->cMatrixRow1.y*(cPt.x - pCache->cTranslate.x))/dDet;
  if((cPt1.x > -g_dPrec) && (cPt1.x < pCache->iImageWidth + g_dPrec) &&
    (cPt1.y > -g_dPrec) && (cPt1.y < pCache->iImageHeight + g_dPrec)) return 0.0;*/

  CDPoint cPt2, cCorner;

  cCorner.x = 0.0;
  cCorner.y = 0.0;
  cPt1.x = pCache->cMatrixRow1*cCorner + pCache->cTranslate.x;
  cPt1.y = pCache->cMatrixRow2*cCorner + pCache->cTranslate.y;
  cCorner.x = (double)pCache->iImageWidth;
  cPt2.x = pCache->cMatrixRow1*cCorner + pCache->cTranslate.x;
  cPt2.y = pCache->cMatrixRow2*cCorner + pCache->cTranslate.y;

  double dRes = fabs(GetPtDistFromLineSeg(cPt, cPt1, cPt2, pPtX));

  cPt1 = cPt2;
  cCorner.y = (double)pCache->iImageHeight;
  cPt2.x = pCache->cMatrixRow1*cCorner + pCache->cTranslate.x;
  cPt2.y = pCache->cMatrixRow2*cCorner + pCache->cTranslate.y;

  CDLine cPtX;
  double dRes1 = fabs(GetPtDistFromLineSeg(cPt, cPt1, cPt2, &cPtX));
  if(dRes1 < dRes)
  {
    dRes = dRes1;
    *pPtX = cPtX;
  }

  cPt1 = cPt2;
  cCorner.x = 0.0;
  cPt2.x = pCache->cMatrixRow1*cCorner + pCache->cTranslate.x;
  cPt2.y = pCache->cMatrixRow2*cCorner + pCache->cTranslate.y;
  dRes1 = fabs(GetPtDistFromLineSeg(cPt, cPt1, cPt2, &cPtX));
  if(dRes1 < dRes)
  {
    dRes = dRes1;
    *pPtX = cPtX;
  }

  cPt1 = cPt2;
  cCorner.y = 0.0;
  cPt2.x = pCache->cMatrixRow1*cCorner + pCache->cTranslate.x;
  cPt2.y = pCache->cMatrixRow2*cCorner + pCache->cTranslate.y;
  dRes1 = fabs(GetPtDistFromLineSeg(cPt, cPt1, cPt2, &cPtX));
  if(dRes1 < dRes)
  {
    dRes = dRes1;
    *pPtX = cPtX;
  }
  return dRes;
}

