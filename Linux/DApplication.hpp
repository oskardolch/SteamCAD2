#ifndef _DAPPLICATION_HPP_
#define _DAPPLICATION_HPP_

#include <gtk/gtk.h>

#include "../Source/DDrawTypes.hpp"
#include "DFileSetupDlg.hpp"
#include "DLineStyleDlg.hpp"
#include "DDimEditDlg.hpp"
#include "DStatDlg.hpp"
#include "DSnapDlg.hpp"
#include "DScaleDlg.hpp"

typedef class CDApplication
{
private:
  GtkWidget *m_pMainWnd;
  gint m_iLeft, m_iTop, m_iWidth, m_iHeight;
  GdkGravity m_grav;
  double m_dDeviceToUnitScale;

  bool m_bSettingProps;

  PDataList m_pDrawObjects;
  PDataList m_pUndoObjects;
  gchar *m_sFileName;
  bool m_bHasChanged;
  PDObject m_pActiveObject;
  PDObject m_pHighObject;
  PDObject m_pSelForDimen;
  int m_iHighDimen;
  int m_iRedoCount;
  CDFileSetupRec m_cFSR;
  gchar *m_sLastPath;
  double m_dDrawScale;
  double m_dUnitScale;
  double m_dwPage, m_dhPage;
  GdkPoint m_cViewOrigin;
  GdkPoint m_cZoomOrig;
  GdkPoint m_cLastMovePt;
  GdkPoint m_cLastSnapPt;
  GdkPoint m_cLastDownPt;
  CDPoint m_cLastDrawPt;
  CDLine m_cLastDynPt;
  long m_lSelColor;
  long m_lHighColor;
  long m_lActiveColor;

  int m_iDrawMode;
  int m_iButton; // 0 nothing, 1 LButon, 2 MButton, 3 RButton
  int m_iToolMode;
  int m_iSelectTolerance;
  int m_iSnapTolerance;
  int m_iDrawGridMode;
  cairo_surface_t *m_pcs;
  cairo_pattern_t *m_pcp;
  gboolean m_bRenderDirect;
  CDLine m_cMeasPoint1, m_cMeasPoint2;
  gboolean m_bPaperUnits;

  int m_iLastExportType;

  GtkWidget *m_pStatSep1;
  GtkWidget *m_pStatLab1;
  GtkWidget *m_pStatEdt1;
  GtkWidget *m_pStatLab2;
  GtkWidget *m_pStatEdt2;
  GtkWidget *m_pStatSep2;
  GtkWidget *m_pStatLab3;
  GtkWidget *m_pSnapEnableMnu;
  GtkWidget *m_pSnapDisableMnu;

  GtkAccelGroup *m_pAccelGroup;

  char m_sStatus1Base[64];
  char m_sStatus1Msg[128];
  char m_sStatus2Base[64];
  char m_sStatus2Msg[128];
  char m_sStatus3Msg[128];
  double m_dRestrictValue;
  int m_iRestrictSet;

  double m_dSavedAngle;
  double m_dSavedDist;

  PDFileSetupDlg m_pFileSetupDlg;
  PDLineStyleDlg m_pLineStyleDlg;
  PDDimEditDlg m_pDimEditDlg;
  PDStatDlg m_pStatDlg;
  PDSnapDlg m_pSnapDlg;
  PDScaleDlg m_pScaleDlg;
	
  void SaveSettings();
  void RestoreSettings();
  GtkWidget* GetMenuBar();
  GtkWidget* GetDrawing();
  GtkWidget* GetStatusBar();

  void SetStatusBarMsg(int iPanel, const gchar *pMsg);
  void FileNewCmd(bool bFromAccel);
  void FileOpenCmd(bool bFromAccel);
  void FileSaveCmd(bool bFromAccel);
  void FileSaveAsCmd(bool bFromAccel);
  void FileSaveSelCmd(bool bFromAccel);
  void FileIncludeCmd(bool bFromAccel);
  void FileExportCmd(bool bFromAccel);
  void FilePropsCmd(bool bFromAccel);
  void FileQuitCmd(bool bFromAccel);

  void EditDeleteCmd(GtkWidget *widget, bool bFromAccel);
  void EditCopyParCmd(GtkWidget *widget, bool bFromAccel);
  void EditMoveCmd(GtkWidget *widget);
  void EditRotateCmd(GtkWidget *widget);
  void EditMirrorCmd(GtkWidget *widget);
  void EditLineStyleCmd(GtkWidget *widget);
  void EditToggleSnapCmd(GtkWidget *widget);
  void EditPaperUnitsCmd(GtkWidget *widget, bool bFromAccel);
  void EditUndoCmd(GtkWidget *widget);
  void EditRedoCmd(GtkWidget *widget);
  void EditConfirmCmd(GtkWidget *widget);

  void ViewFitCmd(GtkWidget *widget);
  void ViewNormalCmd(GtkWidget *widget);
  void ViewGridCmd(GtkWidget *widget, bool bFromAccel, int iFlag);

  bool PromptForSave(GtkWidget *widget);
  bool SaveFile(GtkWidget *widget, gchar **psFile, bool bSelectOnly);
  void SetTitle(GtkWidget *widget, bool bForce);
  void FilePropsToData(PDFileAttrs pFAttrs);
  void DataToFileProps();
  void GetPageDims();
  bool LoadFile(GtkWidget *widget, gchar **psFile, bool bClear);

  long CodeRGBColor(unsigned char *pColor);
  long SetColorAlpha(long lColor, int iAlpha); // 0 <= iAlpha <= 100
  void SetLColor(cairo_t *cr, long lColor);
  void DrawDimArrow(cairo_t *cr, PDPrimitive pPrim);
  void DrawDimText(cairo_t *cr, PDPrimitive pPrim, PDObject pObj, long dwColor,
      double dLineWidth);
  void DrawPrimitive(cairo_t *cr, PDPrimitive pPrim);
  void DrawObject(cairo_t *cr, PDObject pObj, int iMode, int iDimen);

  void SetMode(int iNewMode, bool bFromAccel);
  void SetTool(int iNewTool);
  void ToolsStatCmd();
  void ToolsScaleCmd();

  void PathCreateCmd();
  void PathBreakCmd();
  void PathAreaCmd();
  void PathGroupCmd();
  void PathUngroupCmd();
  void PathMoveUpCmd();
  void PathMoveDownCmd();
  void PathMoveTopCmd();
  void PathMoveBottomCmd();

  void DrawCross(cairo_t *cr);

  int GetDynMode();
  gboolean GetUpdateRegion(PDPtrList pPolygons, GdkRectangle *pRect);
  void DrawSizeRectClip(cairo_t *cr, double x, double y);

  void StartNewObject(gboolean bShowEdit);
  void CopyIniFiles(const char *psConfDir);
public:	
  CDApplication(const char *psConfDir);
  ~CDApplication();
  GtkWidget* GetMainWindow();
  void SetPosition(gint iLeft, gint iTop, gint iWidth, gint iHeight, GdkGravity iGrav);
  void SetFileSetupDlg(gint iLeft, gint iTop);
  void SetLineStyleDlg(gint iLeft, gint iTop);
  void SetDimEditDlg(gint iLeft, gint iTop);
  void SetStatDlg(gint iLeft, gint iTop);
  void SetSnapDlg(gint iLeft, gint iTop);
  void SetScaleDlg(gint iLeft, gint iTop);
  void SetDrawSettings(gboolean bPaperUnits, gint iLastExportType, gint iDrawGridMode, const gchar *sPath);
  gboolean Terminate();
  void SetPageSettings(PDFileSetupRec pFSR);
  void SetPaperSize(PDFileSetupRec pFSR);
  void SetLengthUnit(PDFileSetupRec pFSR);
  void SetAngularUnit(PDFileSetupRec pFSR);
  void SetPaperUnit(PDFileSetupRec pFSR);
  void SetGraphUnit(PDFileSetupRec pFSR);
  void SetDimensioning(PDFileSetupRec pFSR);
  void Configure(GtkWidget *widget, GdkEventConfigure *event);
  void Paint(GtkWidget *widget, GdkEventExpose *event);
  void MouseMove(GtkWidget *widget, GdkEventMotion *event, gboolean bForce);
  void MouseWheel(GtkWidget *widget, GdkEventScroll *event);
  void MouseLButtonDown(GtkWidget *widget, GdkEventButton *event);
  void MouseLButtonUp(GtkWidget *widget, GdkEventButton *event);
  void MouseMButtonDown(GtkWidget *widget, GdkEventButton *event);
  void MouseMButtonUp(GtkWidget *widget, GdkEventButton *event);
  void MouseRButtonDown(GtkWidget *widget, GdkEventButton *event);
  void MouseRButtonUp(GtkWidget *widget, GdkEventButton *event);
  void MouseLDblClick(GtkWidget *widget, GdkEventButton *event);

  void Edit1Changed(GtkEntry *entry);

  void FileCommand(int iCmd, bool bFromAccel);
  void ModeCommand(int iCmd, bool bFromAccel);
  void EditCommand(int iCmd, bool bFromAccel);
  void ViewCommand(int iCmd, bool bFromAccel);
  void ToolsCommand(int iCmd, bool bFromAccel);
  void PathCommand(int iCmd, bool bFromAccel);

  void EnableSnap();
  void DisableSnap();
} *PDApplication;

#endif

