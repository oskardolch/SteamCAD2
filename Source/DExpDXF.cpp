#include "DExpDXF.hpp"
#include "../dxflib/dl_dxf.h"
#include <math.h>
#include <string.h>
#include "DMath.hpp"

//#include <windows.h>

typedef struct CDXFLineType
{
    char sName[32];
    int iNumOfDashes;
    int iPatLens[6];
} *PDXFLineType;

typedef struct CDXFLayer
{
    int iLineType;
    int iWidth;
    int iCount;
} *PDXFLayer;

int AddLineType(int iTypes, PDXFLineType *ppLineTypes, CDLineStyle cLS, int *piDataSize)
{
    PDXFLineType pLineTypes = *ppLineTypes;
    int j, i = 0;
    bool bFound = false;
    while(!bFound && (i < iTypes))
    {
        bFound = (pLineTypes[i].iNumOfDashes == cLS.iSegments);
        j = 0;
        while(bFound && (j < cLS.iSegments))
        {
            bFound = (Round(1000.0*cLS.dPattern[j]) == abs(pLineTypes[i].iPatLens[j]));
            j++;
        }
        i++;
    }
    if(bFound) return i - 1;

    if(iTypes >= *piDataSize)
    {
        *piDataSize += 16;
        *ppLineTypes = (PDXFLineType)realloc(*ppLineTypes, (*piDataSize)*sizeof(CDXFLineType));
        pLineTypes = *ppLineTypes;
    }

    sprintf(pLineTypes[iTypes].sName, "DASHTYPE%d", iTypes);
    pLineTypes[iTypes].iNumOfDashes = cLS.iSegments;
    for(j = 0; j < cLS.iSegments; j++)
    {
        if(j % 2) pLineTypes[iTypes].iPatLens[j] = -Round(1000.0*cLS.dPattern[j]);
        else pLineTypes[iTypes].iPatLens[j] = Round(1000.0*cLS.dPattern[j]);
    }
    return iTypes;
}

int AddLayer(int iLineType, int iLayers, PDXFLayer *ppLayers, CDLineStyle cLS, int *piDataSize)
{
    PDXFLayer pLayers = *ppLayers;
    //int iWidth = Round(100.0*fabs(cLS.dWidth));
    int i = 0;
    bool bFound = false;
    while(!bFound && (i < iLayers))
    {
        bFound = (pLayers[i].iLineType == iLineType);// && (pLayers[i].iWidth == iWidth);
        i++;
    }
    if(bFound)
    {
        pLayers[--i].iCount++;
        return i;
    }

    if(iLayers >= *piDataSize)
    {
        *piDataSize += 16;
        *ppLayers = (PDXFLayer)realloc(*ppLayers, (*piDataSize)*sizeof(CDXFLayer));
        pLayers = *ppLayers;
    }

    pLayers[iLayers].iLineType = iLineType;
    pLayers[iLayers].iWidth = 1; //iWidth;
    pLayers[iLayers].iCount = 1;
    return iLayers;
}

int GetLineTypes(PDataList pDrawData, PDXFLineType *ppLineTypes,
    PDXFLayer *ppLayers, int *piLayers)
{
    int iDataSize = 16;
    int iDataLen = 1;
    PDXFLineType pLineTypes = (PDXFLineType)malloc(iDataSize*sizeof(CDXFLineType));
    strcpy(pLineTypes[0].sName, "CONTINUOUS");
    pLineTypes[0].iNumOfDashes = 0;

    int iLayerSize = 16;
    int iLayerLen = 0;
    PDXFLayer pLayers = (PDXFLayer)malloc(iLayerSize*sizeof(CDXFLayer));

    PDObject pObj;
    CDLineStyle cLS;
    int n = pDrawData->GetCount();
    int iLineType, iLayer;
    for(int i = 0; i < n; i++)
    {
        pObj = pDrawData->GetItem(i);
        cLS = pObj->GetLineStyle();
        iLineType = AddLineType(iDataLen, &pLineTypes, cLS, &iDataSize);
        if(iLineType >= iDataLen) iDataLen++;
        iLayer = AddLayer(iLineType, iLayerLen, &pLayers, cLS, &iLayerSize);
        if(iLayer >= iLayerLen) iLayerLen++;
        pObj->SetAuxInt(iLayer);
    }

    int iBaseLayer = 0;
    for(int i = 1; i < iLayerLen; i++)
    {
        if(pLayers[i].iCount > pLayers[iBaseLayer].iCount) iBaseLayer = i;
    }
    if(iBaseLayer > 0)
    {
        CDXFLayer cTmpLayer = pLayers[0];
        pLayers[0] = pLayers[iBaseLayer];
        pLayers[iBaseLayer] = cTmpLayer;

        for(int i = 0; i < n; i++)
        {
            pObj = pDrawData->GetItem(i);
            iLayer = pObj->GetAuxInt();
            if(iLayer == iBaseLayer) pObj->SetAuxInt(0);
            else if(iLayer == 0) pObj->SetAuxInt(iBaseLayer);
        }
    }

    *ppLineTypes = pLineTypes;
    *ppLayers = pLayers;
    *piLayers = iLayerLen;
    return iDataLen;
}

void ExportDimArrow()
{
}

void DXFExportPrimitive(DL_Dxf *pdxf, DL_WriterA *dw, int iWidth, double dPageHeight, PDPrimitive pPrim,
    char *sLayerName)
{
    double da1, da2;

    switch(pPrim->iType)
    {
    case 1:
        pdxf->writeLine(*dw,
            DL_LineData(pPrim->cPt1.x, dPageHeight - pPrim->cPt1.y, 0.0,
                pPrim->cPt2.x, dPageHeight - pPrim->cPt2.y, 0.0),
            DL_Attributes(sLayerName, 256, iWidth, "BYLAYER", 1.0));
        break;
    case 2:
        da2 = 180.0*atan2(pPrim->cPt1.y - pPrim->cPt3.y, pPrim->cPt3.x - pPrim->cPt1.x)/M_PI;
        da1 = 180.0*atan2(pPrim->cPt1.y - pPrim->cPt4.y, pPrim->cPt4.x - pPrim->cPt1.x)/M_PI;

        pdxf->writeArc(*dw,
            DL_ArcData(pPrim->cPt1.x, dPageHeight - pPrim->cPt1.y, 0.0,
                pPrim->cPt2.x - pPrim->cPt1.x, da2, da1),
            DL_Attributes(sLayerName, 256, iWidth, "BYLAYER", 1.0));
        break;
    case 3:
        pdxf->writeCircle(*dw,
            DL_CircleData(pPrim->cPt1.x, dPageHeight - pPrim->cPt1.y, 0.0,
                pPrim->cPt2.x - pPrim->cPt1.x),
            DL_Attributes(sLayerName, 256, iWidth, "BYLAYER", 1.0));
        break;
    case 4:
    case 5:
        break;
    case 9:
        ExportDimArrow();
        break;
    }
}

void DXFExportBSpline(DL_Dxf *pdxf, DL_WriterA *dw, int iWidth, double dPageHeight, bool bClosed,
    int iCtrls, double *pdKnots, PDPoint pPoints, char *sLayerName)
{

	if(iCtrls < 3)
	{
		if(iCtrls > 1)
		{
            pdxf->writeLine(*dw,
                DL_LineData(pPoints[0].x, dPageHeight - pPoints[0].y, 0.0,
                    pPoints[1].x, dPageHeight - pPoints[1].y, 0.0),
                DL_Attributes(sLayerName, 256, iWidth, "BYLAYER", 1.0));
		}
		return;
	}

	// Number of knots (= number of control points + spline degree + 1)
	int iKnots = iCtrls + 3;

	int flags;
	if(bClosed) flags = 11;
	else flags = 8;

	// write spline header:
	pdxf->writeSpline(*dw, DL_SplineData(2, iKnots, iCtrls, 0, flags),
        DL_Attributes(sLayerName, 256, iWidth, "BYLAYER", 1.0));

	// write spline knots:
	for(int i = 0; i < iKnots; i++)
	{
		pdxf->writeKnot(*dw, DL_KnotData(pdKnots[i]));
	}

	// write spline control points:
	for(int i = 0; i < iCtrls; i++)
	{
		pdxf->writeControlPoint(*dw, DL_ControlPointData(pPoints[i].x, dPageHeight - pPoints[i].y, 0.0, 1.0));
	}
}

void DXFExportDimText()
{
}

void DXFWriteObject(DL_Dxf *pdxf, DL_WriterA *dw, PDObject pObj, double dPageHeight)
{
    int iType = pObj->GetType();

    CDLineStyle cLS = pObj->GetLineStyle();
    char sLayer[32];
    int iLayer = pObj->GetAuxInt();
    if(iLayer > 0) sprintf(sLayer, "LAYER%d", iLayer);
    else strcpy(sLayer, "0");

    int iWidth = Round(100.0*cLS.dWidth);

    bool bVisible = (cLS.dWidth > -0.00001);

    if((iType == 3) || (iType > 4))
    {
        int iParts = pObj->GetBSplineParts();
        if(iParts > 0)
        {
            int *piCtrls = (int*)malloc(iParts*sizeof(int));
            PDPoint *ppPoints = (PDPoint*)malloc(iParts*sizeof(PDPoint));
            double **ppdKnots = (double**)malloc(iParts*sizeof(double*));
            bool bClosed = pObj->GetBSplines(iParts, 1.0, piCtrls, ppdKnots, ppPoints);

            for(int i = 0; i < iParts; i++)
            {
                DXFExportBSpline(pdxf, dw, iWidth, dPageHeight, bClosed, piCtrls[i], ppdKnots[i], ppPoints[i], sLayer);
                free(ppPoints[i]);
                free(ppdKnots[i]);
            }
            free(ppPoints);
            free(piCtrls);
        }

        bVisible = false;
    }

    CDPrimitive cPrim;
    pObj->GetFirstPrimitive(&cPrim, 1.0, -2);

    while(cPrim.iType > 0)
    {
        if(cPrim.iType == 10) DXFExportDimText();
        else if(bVisible) DXFExportPrimitive(pdxf, dw, iWidth, dPageHeight, &cPrim, sLayer);
        pObj->GetNextPrimitive(&cPrim, 1.0, -2);
    }

    //PDDimension pDim;
    int iNodes;
    CDPoint cPts[2], cPt1, cPt2;
    double dAng;
    for(int i = 0; i < pObj->GetDimenCount(); i++)
    {
        iNodes = 0;
        //pDim = pObj->GetDimen(i);
        pObj->GetFirstPrimitive(&cPrim, 1.0, i);
        while(cPrim.iType > 0)
        {
            if(cPrim.iType == 10)
            {
                cPt1 = cPrim.cPt1;
                DXFExportDimText();
            }
            else if(cPrim.iType == 9)
            {
                if(iNodes < 2) cPts[iNodes++] = cPrim.cPt2;
            }
            //DXFExportPrimitive(pdxf, dw, iWidth, dPageHeight, &cPrim, sLayer);
            pObj->GetNextPrimitive(&cPrim, 1.0, i);
        }
        if(iNodes > 1)
        {
            iType = pObj->GetType();
            if(iType == 1)
            {
                cPt2 = (cPts[0] + cPts[1])/2.0;
                dAng = atan2(cPts[1].y - cPts[0].y, cPts[1].x - cPts[0].x);
                pdxf->writeDimLinear(*dw, DL_DimensionData(cPt2.x, dPageHeight - cPt2.y, 0.0, cPt1.x, dPageHeight - cPt1.y, 0.0,
                    0, 2, 1, 2.0, "<>", "Standard", 0.0, 1.0, 1.0),
                    DL_DimLinearData(cPts[0].x, dPageHeight - cPts[0].y, 0.0, cPts[1].x, dPageHeight - cPts[1].y, 0.0, dAng, 0.0),
                    DL_Attributes());
            }
        }
    }
}

#ifdef _WIN32_WINNT
void ExportDXFFile(FILE *pFile, PDataList pDrawData, PDUnitList pUnits)
#else
void ExportDXFFile(char *sFileName, PDataList pDrawData, PDUnitList pUnits)
#endif
{
    DL_Dxf dxf;
    DL_Codes::version exportVersion = DL_Codes::AC1015;
#ifdef _WIN32_WINNT
    DL_WriterA* dw = dxf.out(pFile, exportVersion);
#else
    DL_WriterA* dw = dxf.out(sFileName, exportVersion);
#endif
    if(!dw) return;

    PDXFLineType pLineTypes;
    PDXFLayer pLayers;
    int iLayers = 0;
    int iLineTypes = GetLineTypes(pDrawData, &pLineTypes, &pLayers, &iLayers);

    CDFileAttrs cFileAttrs;
    pDrawData->GetFileAttrs(&cFileAttrs);

    CDRect cRect;
    cRect.cPt1 = 0;
    cRect.cPt2.x = cFileAttrs.dWidth;
    cRect.cPt2.y = cFileAttrs.dHeight;
    pDrawData->BuildAllPrimitives(&cRect, false);

    dxf.writeHeader(*dw);

    // int variable:
    dw->dxfString(9, "$INSUNITS");
    dw->dxfInt(70, 4);
    // real (double, float) variable:
    dw->dxfString(9, "$DIMEXE");
    dw->dxfReal(40, 1.25);
    // string variable:
    dw->dxfString(9, "$TEXTSTYLE");
    dw->dxfString(7, "Standard");
    // vector variable:
    dw->dxfString(9, "$LIMMIN");
    dw->dxfReal(10, 0.0);
    dw->dxfReal(20, 0.0);
    dw->dxfString(9, "$LIMMAX");
    dw->dxfReal(10, cFileAttrs.dWidth);
    dw->dxfReal(20, cFileAttrs.dHeight);
    dw->dxfString(9, "$LWDISPLAY");
    dw->dxfReal(290, 1.0);
    dw->dxfString(9, "$DIMLFAC");
    dw->dxfReal(40, cFileAttrs.dScaleDenom/cFileAttrs.dScaleNom);

    dw->sectionEnd();

    dw->sectionTables();
    dxf.writeVPort(*dw);

    dw->tableLinetypes(iLineTypes + 2);
    double dLen;
    double dPat[6];

    dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0, dPat));
    dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0, dPat));

    for(int i = 0; i < iLineTypes; i++)
    {
        dLen = 0;
        for(int j = 0; j < pLineTypes[i].iNumOfDashes; j++)
        {
            dPat[j] = (double)pLineTypes[i].iPatLens[j]/1000.0;
            dLen += fabs(dPat[j]);
        }
        dxf.writeLinetype(*dw, DL_LinetypeData(
            pLineTypes[i].sName, "", 0, pLineTypes[i].iNumOfDashes, dLen, dPat));
    }
    dw->tableEnd();

    char sBuf[32];
    dw->tableLayers(iLayers);
    if(iLayers > 0)
    {
        dxf.writeLayer(*dw,
            DL_LayerData("0", 0),
            DL_Attributes(
                std::string(""), // leave empty
                DL_Codes::white, // color
                pLayers[0].iWidth, // default width
                pLineTypes[pLayers[0].iLineType].sName, // line style
                1.0
            )
        );
    }
    for(int i = 1; i < iLayers; i++)
    {
        sprintf(sBuf, "LAYER%d", i);
        dxf.writeLayer(*dw,
            DL_LayerData(sBuf, 0),
            DL_Attributes(
                std::string(""), // leave empty
                DL_Codes::white, // color
                pLayers[i].iWidth, // default width
                pLineTypes[pLayers[i].iLineType].sName, // line style
                1.0
            )
        );
    }
    dw->tableEnd();

    dw->tableStyle(1);
    dxf.writeStyle(*dw, DL_StyleData("Standard", 0, 0.0, 0.75, 0.0, 0, 2.5, "txt", ""));
    dw->tableEnd();

    dxf.writeView(*dw);
    dxf.writeUcs(*dw);

    dw->tableAppid(1);
    dw->tableAppidEntry(0x12);
    dw->dxfString(2, "ACAD");
    dw->dxfInt(70, 0);
    dw->tableEnd();

    dxf.writeDimStyle(*dw, 2.5, 1.25, 0.625, 0.625, 2.5);

    dxf.writeBlockRecord(*dw);
    //dxf.writeBlockRecord(*dw, "myblock1");
    //dxf.writeBlockRecord(*dw, "myblock2");
    dw->tableEnd();

    dw->sectionEnd();

    dw->sectionBlocks();
    dxf.writeBlock(*dw,
        DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Model_Space");
    dxf.writeBlock(*dw,
        DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space");
    dxf.writeBlock(*dw,
        DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space0");
    /*dxf.writeBlock(*dw,
        DL_BlockData("myblock1", 0, 0.0, 0.0, 0.0));
    // ...
    // write block entities e.g. with dxf.writeLine(), ..
    // ...
    dxf.writeEndBlock(*dw, "myblock1");
    dxf.writeBlock(*dw,
        DL_BlockData("myblock2", 0, 0.0, 0.0, 0.0));
    // ...
    // write block entities e.g. with dxf.writeLine(), ..
    // ...
    dxf.writeEndBlock(*dw, "myblock2");*/
    dw->sectionEnd();

    dw->sectionEntities();
    // write all your entities..

    PDObject pObj;
    int n = pDrawData->GetCount();
    for(int i = 0; i < n; i++)
    {
        pObj = pDrawData->GetItem(i);
        DXFWriteObject(&dxf, dw, pObj, cFileAttrs.dHeight);
    }

    dw->sectionEnd();

    dxf.writeObjects(*dw);
    dxf.writeObjectsEnd(*dw);

    dw->dxfEOF();
    dw->close();

    free(pLayers);
    free(pLineTypes);
    delete dw;
}

