#include "DExpCairo.hpp"
#include <malloc.h>
#include <math.h>
#include <cairo.h>
#include <cairo-pdf.h>
#include <cairo-ps.h>
#include <cairo-svg.h>
#include <string.h>

#ifdef WIN32
#include <windows.h>
#include <winbase.h>
#include <commctrl.h>
#include <shobjidl.h>
#include <gdiplus.h>
#else
#include <gdk/gdk.h>
#include <gdk/gdkpixbuf.h>
#endif

const double dMmToIn = 25.4;
const double dPtToIn = 72.0;

cairo_status_t WriteCairoStream(void *closure, const unsigned char *data,
  unsigned int length)
{
  FILE *pfile = (FILE*)closure;
  fwrite(data, 1, length, pfile);
  return CAIRO_STATUS_SUCCESS;
}

void ExpSetLColor(cairo_t *cr, unsigned char *pcColor)
{
  if(pcColor[3] == 0)
    cairo_set_source_rgb(cr, pcColor[0]/255.0, pcColor[1]/255.0, pcColor[2]/255.0);
  else
    cairo_set_source_rgba(cr, pcColor[0]/255.0, pcColor[1]/255.0, pcColor[2]/255.0, pcColor[3]/255.0);
}

void ExportDimArrow(cairo_t *pct, PDPrimitive pPrim)
{
  int iType = Round(pPrim->cPt1.x);
  double dr;

  switch(iType)
  {
  case 1:
    cairo_new_path(pct);
    cairo_move_to(pct, pPrim->cPt3.x, pPrim->cPt3.y);
    cairo_line_to(pct, pPrim->cPt2.x, pPrim->cPt2.y);
    cairo_line_to(pct, pPrim->cPt4.x, pPrim->cPt4.y);
    cairo_stroke(pct);
    break;
  case 2:
    cairo_new_path(pct);
    cairo_move_to(pct, pPrim->cPt3.x, pPrim->cPt3.y);
    cairo_line_to(pct, pPrim->cPt2.x, pPrim->cPt2.y);
    cairo_line_to(pct, pPrim->cPt4.x, pPrim->cPt4.y);
    cairo_close_path(pct);
    cairo_fill(pct);
    cairo_stroke(pct);
    break;
  case 3:
    dr = (pPrim->cPt3.x - pPrim->cPt2.x);
    cairo_new_path(pct);
    cairo_arc(pct, pPrim->cPt2.x, pPrim->cPt2.y, dr, 0.0, 2.0*M_PI);
    cairo_close_path(pct);
    cairo_fill(pct);
    cairo_stroke(pct);
    break;
  case 4:
  case 5:
    cairo_new_path(pct);
    cairo_move_to(pct, pPrim->cPt3.x, pPrim->cPt3.y);
    cairo_line_to(pct, pPrim->cPt4.x, pPrim->cPt4.y);
    cairo_stroke(pct);
    break;
  }
}

void ExportPrimitive(cairo_t *pct, PDPrimitive pPrim)
{
  double dr;
  CDPoint cPt1, cPt2;

  switch(pPrim->iType)
  {
  case 1:
    cairo_line_to(pct, pPrim->cPt2.x, pPrim->cPt2.y);
    break;
  case 2:
    dr = pPrim->cPt2.x;
    if(fabs(pPrim->cPt4.x - 1.0) < 0.2)
      cairo_arc_negative(pct, pPrim->cPt1.x, pPrim->cPt1.y, dr, pPrim->cPt3.y, pPrim->cPt3.x);
    else
      cairo_arc(pct, pPrim->cPt1.x, pPrim->cPt1.y, dr, pPrim->cPt3.x, pPrim->cPt3.y);
    break;
  case 3:
    dr = (pPrim->cPt2.x - pPrim->cPt1.x);
    cairo_arc(pct, pPrim->cPt1.x, pPrim->cPt1.y, dr, 0.0, 2.0*M_PI);
    break;
  case 4:
    cPt1 = (pPrim->cPt1 + 2.0*pPrim->cPt2)/3.0;
    cPt2 = (pPrim->cPt3 + 2.0*pPrim->cPt2)/3.0;
    cairo_curve_to(pct, cPt1.x, cPt1.y, cPt2.x, cPt2.y,
    pPrim->cPt3.x, pPrim->cPt3.y);
    break;
  case 5:
    cairo_curve_to(pct, pPrim->cPt2.x, pPrim->cPt2.y,
    pPrim->cPt3.x, pPrim->cPt3.y,
    pPrim->cPt4.x, pPrim->cPt4.y);
    break;
  case 9:
    ExportDimArrow(pct, pPrim);
    break;
  }
}

void ExportDimText(cairo_t *pct, PDPrimitive pPrim, PDObject pObj, double dScale,
  PDUnitList pUnits, double dRat)
{
  int iPos = Round(pPrim->cPt2.y);

  char sBuf[64];
  char *psBuf = sBuf;
  int iLen = pObj->PreParseDimText(iPos, psBuf, 64, dScale, pUnits);
  if(iLen > 0)
  {
    psBuf = (char*)malloc(iLen*sizeof(char));
    pObj->PreParseDimText(iPos, psBuf, iLen, dScale, pUnits);
  }

  int iLen2 = strlen(psBuf);
  if(iLen2 < 1) return;

  cairo_translate(pct, pPrim->cPt1.x, pPrim->cPt1.y);
  double dPi2 = M_PI/2.0;
  cairo_rotate(pct, pPrim->cPt2.x - dPi2);

  CDFileAttrs cFileAttrs;
  pObj->GetDimFontAttrs(iPos, &cFileAttrs);

  double da = 1.6*cFileAttrs.dFontSize*dRat;

  bool bDiam = (psBuf[0] == '*');
  int iStart = 0;
  if(bDiam) iStart = 1;
  if(iStart >= iLen2)
  {
    cairo_new_path(pct);
    cairo_arc(pct, -0.25*da, -0.325*da, 0.25*da, 0.0, 2*M_PI);
    cairo_move_to(pct, -0.25*da, -0.075*da);
    cairo_line_to(pct, dRat, -0.575*da);
    cairo_stroke(pct);
    cairo_identity_matrix(pct);
    return;
  }

  char *sBufStart = &psBuf[iStart];
  char *sFrac = strchr(sBufStart, '_');
  char sBufNom[4];
  char sBufDenom[4];
  char *sBufEnd = NULL;

  if(sFrac)
  {
    sBufEnd = sFrac + 1;
    int i = 0;
    while((*sBufEnd >= '0') && (*sBufEnd <= '9') && (i < 3))
    {
      sBufNom[i++] = *(sBufEnd++);
    }
    sBufNom[i] = 0;

    while(*sBufEnd && (*sBufEnd != '/')) sBufEnd++;
    if(*sBufEnd) sBufEnd++;
    i = 0;
    while((*sBufEnd >= '0') && (*sBufEnd <= '9') && (i < 3))
    {
      sBufDenom[i++] = *(sBufEnd++);
    }
    sBufDenom[i] = 0;
    *sFrac = 0;
  }

  cairo_font_slant_t iSlant = CAIRO_FONT_SLANT_NORMAL;
  if(cFileAttrs.bFontAttrs & 1) iSlant = CAIRO_FONT_SLANT_ITALIC;

  cairo_font_weight_t iWeight = CAIRO_FONT_WEIGHT_NORMAL;
  if(cFileAttrs.bFontAttrs & 8) iWeight = CAIRO_FONT_WEIGHT_BOLD;

  cairo_select_font_face(pct, cFileAttrs.sFontFace, iSlant, iWeight);
  cairo_set_font_size(pct, da);

  cairo_text_extents_t ctexStart, ctexNom, ctexDenom, ctexEnd;
  cairo_text_extents(pct, sBufStart, &ctexStart);

  double dTextWidth = ctexStart.width + ctexStart.x_bearing;

  if(sFrac)
  {
    cairo_text_extents(pct, sBufEnd, &ctexEnd);
    cairo_set_font_size(pct, 0.6*da);
    cairo_text_extents(pct, sBufNom, &ctexNom);
    cairo_text_extents(pct, sBufDenom, &ctexDenom);

    dTextWidth += (ctexNom.width + ctexNom.x_bearing);
    dTextWidth += (ctexDenom.width + ctexDenom.x_bearing);
    dTextWidth += (ctexEnd.width + ctexEnd.x_bearing);
    dTextWidth += 0.065*da;
  }

  cairo_set_font_size(pct, da);

  double dx = -dTextWidth/2.0;
  if(bDiam)
  {
    dTextWidth += 0.625*da;
    dx = 0.25*da - dTextWidth/2.0;
    cairo_new_path(pct);
    cairo_arc(pct, dx, -0.325*da, 0.25*da, 0.0, 2*M_PI);
    cairo_move_to(pct, dx - 0.25*da, -0.075*da);
    cairo_line_to(pct, dx + 0.25*da, -0.575*da);
    cairo_stroke(pct);
    dx += 0.375*da;
  }
  cairo_move_to(pct, dx, 0.0);
  cairo_show_text(pct, sBufStart);

  if(sFrac)
  {
    dx += (ctexStart.width + ctexStart.x_bearing + 0.02*da);
    cairo_set_font_size(pct, 0.6*da);
    cairo_move_to(pct, dx, -0.425*da);
    cairo_show_text(pct, sBufNom);

    dx += (ctexNom.width + ctexNom.x_bearing + 0.03*da);
    cairo_move_to(pct, dx, 0.1*da);
    cairo_show_text(pct, sBufDenom);

    cairo_move_to(pct, dx - 0.175*da, -0.2*da);
    cairo_line_to(pct, dx + 0.175*da, -0.55*da);
    cairo_stroke(pct);

    dx += (ctexDenom.width + ctexDenom.x_bearing + 0.015*da);
    cairo_set_font_size(pct, da);
    cairo_move_to(pct, dx, 0.0);
    cairo_show_text(pct, sBufEnd);
  }

  cairo_identity_matrix(pct);

  if(iLen > 0) free(psBuf);
}

#ifdef WIN32
cairo_format_t GetCairoFormat(Gdiplus::PixelFormat cFormat)
{
  cairo_format_t cRes = CAIRO_FORMAT_INVALID;
  switch(cFormat)
  {
  case PixelFormat1bppIndexed:
    cRes = CAIRO_FORMAT_A1;
    break;
  case PixelFormat4bppIndexed:
    break;
  case PixelFormat8bppIndexed:
    cRes = CAIRO_FORMAT_A8;
    break;
  case PixelFormat16bppARGB1555:
  case PixelFormat16bppGrayScale:
    break;
  case PixelFormat16bppRGB555:
    break;
  case PixelFormat16bppRGB565:
    cRes = CAIRO_FORMAT_RGB16_565;
    break;
  case PixelFormat24bppRGB:
    cRes = CAIRO_FORMAT_RGB24;
    break;
  case PixelFormat32bppARGB:
    cRes = CAIRO_FORMAT_ARGB32;
    break;
  case PixelFormat32bppPARGB:
    break;
  case PixelFormat32bppRGB:
    cRes = CAIRO_FORMAT_ARGB32;
    break;
  case PixelFormat48bppRGB:
  case PixelFormat64bppARGB:
  case PixelFormat64bppPARGB:
    break;
  }
  return cRes;
}
#endif

void ExportObject(PDObject pObj, cairo_t *pct, cairo_surface_t *pcs, PDFileAttrs pFileAttrs,
  double dRat, PDUnitList pUnits)
{
  if(pObj->GetType() == dtGroup)
  {
    //cairo_push_group(pct);
    int n = pObj->GetSubObjectCount(false);
    PDObject pObj1;
    for(int i = 0; i < n; i++)
    {
      pObj1 = pObj->GetSubObject(i);
      ExportObject(pObj1, pct, pcs, pFileAttrs, dRat, pUnits);
    }
    //cairo_pop_group(pct);
    return;
  }

  CDLineStyle cStyle = pObj->GetLineStyle();
  double dScale = pFileAttrs->dScaleNom/pFileAttrs->dScaleDenom;

  bool bVisible = (cStyle.dWidth > -0.00001);
  double dLineWdth = fabs(cStyle.dWidth);

  cairo_set_line_cap(pct, (cairo_line_cap_t)cStyle.cCapType);
  cairo_set_line_join(pct, (cairo_line_join_t)cStyle.cJoinType);
  ExpSetLColor(pct, cStyle.cColor);
  cairo_set_line_width(pct, dLineWdth*dRat);

  CDPrimitive cPrim;
  pObj->GetFirstPrimitive(&cPrim, dRat, -2);

  while(cPrim.iType > 0)
  {
    if(cPrim.iType == 10) ExportDimText(pct, &cPrim, pObj, dScale, pUnits, dRat);
    else if(bVisible)
    {
      if(cPrim.iType == 11)
      {
    //   (0, 0) - INVALID
    //   (1, 0) - start a new path without subpath
    //   (2, 0) - stroke path without closing the last subpath (if exists)
    //   (0, 1) - start subpath without closing the previous one (if exists)
    //   (1, 1) - start a new path and immediatelly new subpath
    //   (2, 1) - INVALID
    //   (0, 2) - close subpath and immediately start a new one
    //   (1, 2) - INVALID
    //   (2, 2) - close last subpath and stroke path
        if(fabs(cPrim.cPt1.x - 1.0) < 0.2)
        {
          cairo_new_path(pct);
          if(fabs(cPrim.cPt2.x - 1.0) < 0.2)
            cairo_move_to(pct, cPrim.cPt3.x, cPrim.cPt3.y);
        }
        if(fabs(cPrim.cPt1.y - 1.0) < 0.2)
        {
          cairo_new_sub_path(pct);
          if(fabs(cPrim.cPt2.x - 1.0) < 0.2)
            cairo_move_to(pct, cPrim.cPt3.x, cPrim.cPt3.y);
        }
        if(fabs(cPrim.cPt1.y - 2.0) < 0.2)
        {
          cairo_close_path(pct);
          if(fabs(cPrim.cPt1.x) < 0.2) cairo_new_sub_path(pct);
        }
        if(fabs(cPrim.cPt1.x - 2.0) < 0.2)
        {
          if(cPrim.cPt2.x > 0.00001)
          {
            double dDash[6];
            double dSegLen;
            for(int i = 0; i < cStyle.iSegments; i++)
            {
              dSegLen = cStyle.dPattern[i];
              if((i % 2 == 0) && (dSegLen < g_dDashMin)) dSegLen = g_dDashMin;
              dDash[i] = dRat*cPrim.cPt2.x*dSegLen;
            }
            cairo_set_dash(pct, dDash, cStyle.iSegments, cPrim.cPt2.y);
          }
          cairo_stroke(pct);
          cairo_set_dash(pct, NULL, 0, 0.0);
        }
      }
      else if(cPrim.iType == 12)
      {
    //   (0, 0) - INVALID
    //   (1, 0) - start a new path without subpath
    //   (2, 0) - close path and fill it
    //   (0, 1) - INVALID
    //   (1, 1) - start a new path and immediatelly new subpath
    //   (2, 1) - INVALID
    //   (0, 2) - close subpath and immediately start a new one
    //   (1, 2) - INVALID
    //   (2, 2) - close last subpath and fill path
        if(fabs(cPrim.cPt1.x - 1.0) < 0.2)
        {
          cairo_new_path(pct);
        }
        if(fabs(cPrim.cPt1.y - 1.0) < 0.2)
        {
          cairo_new_sub_path(pct);
        }
        if(fabs(cPrim.cPt1.y - 2.0) < 0.2)
        {
          cairo_close_path(pct);
          if(fabs(cPrim.cPt1.x) < 0.2) cairo_new_sub_path(pct);
        }
        if(fabs(cPrim.cPt1.x - 2.0) < 0.2)
        {
          ExpSetLColor(pct, cStyle.cFillColor);
          cairo_set_fill_rule(pct, CAIRO_FILL_RULE_EVEN_ODD);
          cairo_fill(pct);
          ExpSetLColor(pct, cStyle.cColor);
        }
      }
      else if(cPrim.iType == 13) // raster image
      {
        int iDataSize = 0;
        unsigned char *pData = pObj->GetRasterData(&iDataSize);
#ifdef WIN32
        HGLOBAL hGlobal = GlobalAlloc(GMEM_MOVEABLE, iDataSize);
        void *ptr = GlobalLock(hGlobal);
        memcpy(ptr, pData, iDataSize);
        GlobalUnlock(hGlobal);
        IStream *pStream = NULL;
        CreateStreamOnHGlobal(hGlobal, TRUE, &pStream);
        Gdiplus::Image *image = Gdiplus::Image::FromStream(pStream);
        int iw = image->GetWidth();
        int ih = image->GetHeight();

        //double dScaleFactor = 1.043002;
        double dScaleFactor = 1.103;
        double dScaleFactorX = dScaleFactor*image->GetHorizontalResolution()/300.0;
        double dScaleFactorY = dScaleFactor*image->GetVerticalResolution()/300.0;
        double dw = (double)iw/dScaleFactorX;
        double dh = (double)ih/dScaleFactorY;

        Gdiplus::PixelFormat gdiFormat = image->GetPixelFormat();
        Gdiplus::Bitmap *bmp = new Gdiplus::Bitmap((int)dw, (int)dh, PixelFormat32bppARGB);
        Gdiplus::Graphics *pgraph = Gdiplus::Graphics::FromImage(bmp);

        Gdiplus::Matrix *matrix = new Gdiplus::Matrix(dScaleFactorX, 0.0f, 0.0f, dScaleFactorY, 0.0f, 0.0f);
        pgraph->SetTransform(matrix);
        pgraph->DrawImage(image, 0, 0);
        pgraph->ResetTransform();
        pgraph->Flush();
        Gdiplus::BitmapData pBmpData;
        Gdiplus::Rect *rc = new Gdiplus::Rect(0, 0, (int)dw, (int)dh);
        bmp->LockBits(rc, Gdiplus::ImageLockModeRead, PixelFormat32bppARGB, &pBmpData);
        cairo_format_t cFormat = GetCairoFormat(PixelFormat32bppARGB);
        if(cFormat != CAIRO_FORMAT_INVALID)
        {
          cairo_surface_t *cs = cairo_image_surface_create_for_data((unsigned char*)pBmpData.Scan0, cFormat,
            pBmpData.Width, pBmpData.Height, pBmpData.Stride);
          cairo_t *cr2 = cairo_create(pcs);

          cairo_matrix_t cMat = {dRat*cPrim.cPt1.x, dRat*cPrim.cPt2.x,
            dRat*cPrim.cPt1.y, dRat*cPrim.cPt2.y,
            cPrim.cPt3.x, cPrim.cPt3.y};
          cairo_set_matrix(cr2, &cMat);
          cairo_set_source_surface(cr2, cs, 0.0, 0.0);
          cairo_identity_matrix(cr2);
          cairo_paint(cr2);
          cairo_destroy(cr2);
        }
        bmp->UnlockBits(&pBmpData);
        delete bmp;
        delete rc;
        delete image;
        delete matrix;
        //delete pgraph;
        pStream->Release();
#else
        GInputStream *pStream = g_memory_input_stream_new_from_data(pData, iDataSize, NULL);
        GError *pErr = NULL;
        GdkPixbuf *pPixBuf = gdk_pixbuf_new_from_stream(pStream, NULL, &pErr);
        if(pPixBuf)
        {
          cairo_t *cr2 = cairo_create(pcs);
          cairo_matrix_t cMat = {cPrim.cPt1.x, cPrim.cPt2.x, cPrim.cPt1.y, cPrim.cPt2.y,
            cPrim.cPt3.x, cPrim.cPt3.y};
          cairo_set_matrix(cr2, &cMat);
          gdk_cairo_set_source_pixbuf(cr2, pPixBuf, 0.0, 0.0);
          cairo_identity_matrix(cr2);
          cairo_paint(cr2);
          cairo_destroy(cr2);
          g_object_unref(pPixBuf);
        }
        else
        {
          // handle error
          g_error_free(pErr);
        }
        g_input_stream_close(pStream, NULL, NULL);
#endif
      }
      else ExportPrimitive(pct, &cPrim);
    }
    pObj->GetNextPrimitive(&cPrim, dRat, -2);
  }

  for(int i = 0; i < pObj->GetDimenCount(); i++)
  {
    pObj->GetFirstPrimitive(&cPrim, dRat, i);
    while(cPrim.iType > 0)
    {
      if(cPrim.iType == 10) ExportDimText(pct, &cPrim, pObj, dScale, pUnits, dRat);
      else ExportPrimitive(pct, &cPrim);
      pObj->GetNextPrimitive(&cPrim, dRat, i);
    }
  }
}

void ExportCairoFile(int iType, FILE *pFile, PDataList pDrawData, PDUnitList pUnits)
{
  CDFileAttrs cFileAttrs;
  pDrawData->GetFileAttrs(&cFileAttrs);

  double dMmToPt = dPtToIn/dMmToIn;

  double dWidth = cFileAttrs.dWidth*dMmToPt;
  double dHeight = cFileAttrs.dHeight*dMmToPt;
  cairo_surface_t *pcs = NULL;

  switch(iType)
  {
  case 0:
    pcs = cairo_pdf_surface_create_for_stream(WriteCairoStream,
      pFile, dWidth, dHeight);
    break;
  case 1:
  case 2:
    pcs = cairo_ps_surface_create_for_stream(WriteCairoStream,
      pFile, dWidth, dHeight);
    break;
  case 3:
    dMmToPt = 600.0/dMmToIn;
    dWidth = cFileAttrs.dWidth*dMmToPt;
    dHeight = cFileAttrs.dHeight*dMmToPt;
    pcs =  cairo_image_surface_create(CAIRO_FORMAT_ARGB32, (int)dWidth, (int)dHeight);
    break;
  case 4:
    pcs = cairo_svg_surface_create_for_stream(WriteCairoStream,
      pFile, dWidth, dHeight);
    break;
  }

  if(!pcs) return;

  if(iType == 2) cairo_ps_surface_set_eps(pcs, true);

  cairo_t *pct = cairo_create(pcs);

  CDRect cRect;
  cRect.cPt1 = 0;
  cRect.cPt2.x = cFileAttrs.dWidth;
  cRect.cPt2.y = cFileAttrs.dHeight;
  pDrawData->BuildAllPrimitives(&cRect);

  cairo_set_line_join(pct, CAIRO_LINE_JOIN_ROUND);

  PDObject pObj;
  int n = pDrawData->GetCount();
  for(int i = 0; i < n; i++)
  {
    pObj = pDrawData->GetItem(i);
    ExportObject(pObj, pct, pcs, &cFileAttrs, dMmToPt, pUnits);
  }

  cairo_identity_matrix(pct);

  //cairo_save(pct);
  cairo_destroy(pct);
  cairo_surface_flush(pcs);
  if(iType == 3)
  {
    cairo_surface_write_to_png_stream(pcs, WriteCairoStream, pFile);
  }
  cairo_surface_finish(pcs);
  cairo_surface_destroy(pcs);
}
