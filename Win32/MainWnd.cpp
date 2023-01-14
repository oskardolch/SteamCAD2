#include "MainWnd.hpp"
#include "SteamCAD2.rh"
#include <wchar.h>
#include <math.h>
#include <commctrl.h>
#include <shobjidl.h>

#include "XMLUtils.hpp"
#include "../Source/DMath.hpp"
#include "../Source/DExpCairo.hpp"
#include "../Source/DParser.hpp"
#include "../Source/DExpDXF.hpp"
#include "shlobjidl_core.hpp"

//#define SQR(X) ((X)*(X))

double GetPtDist(LPPOINT pPt, int x, int y)
{
  return fabs(pPt->x - x) + fabs(pPt->y - y);
}


LRESULT CALLBACK MainWndProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
  CMainWnd *mw = NULL;
  if(uMsg == WM_CREATE) mw = (CMainWnd*)((LPCREATESTRUCT)lParam)->lpCreateParams;
  else  mw = (CMainWnd*)GetWindowLongPtr(hwnd, GWLP_USERDATA);

  switch(uMsg)
  {
  case WM_CREATE:
    return mw->WMCreate(hwnd, (LPCREATESTRUCT)lParam);
  case WM_COMMAND:
    return mw->WMCommand(hwnd, HIWORD(wParam), LOWORD(wParam),
      (HWND)lParam);
  case WM_PAINT:
    return mw->WMPaint(hwnd, (HDC)wParam);
  case WM_SIZE:
    return mw->WMSize(hwnd, wParam, LOWORD(lParam), HIWORD(lParam));
  case WM_LBUTTONDBLCLK:
    return mw->WMLButtonDblClk(hwnd, wParam, (short int)LOWORD(lParam), (short int)HIWORD(lParam));
  case WM_MBUTTONDOWN:
    return mw->WMMButtonDown(hwnd, wParam, (short int)LOWORD(lParam), (short int)HIWORD(lParam));
  case WM_MBUTTONUP:
    return mw->WMMButtonUp(hwnd, wParam, (short int)LOWORD(lParam), (short int)HIWORD(lParam));
  case WM_LBUTTONDOWN:
    return mw->WMLButtonDown(hwnd, wParam, (short int)LOWORD(lParam), (short int)HIWORD(lParam));
  case WM_LBUTTONUP:
    return mw->WMLButtonUp(hwnd, wParam, (short int)LOWORD(lParam), (short int)HIWORD(lParam));
  case WM_RBUTTONDOWN:
    return mw->WMRButtonDown(hwnd, wParam, (short int)LOWORD(lParam), (short int)HIWORD(lParam));
  case WM_RBUTTONUP:
    return mw->WMRButtonUp(hwnd, wParam, (short int)LOWORD(lParam), (short int)HIWORD(lParam));
  case WM_MOUSEMOVE:
    return mw->WMMouseMove(hwnd, wParam, (short int)LOWORD(lParam), (short int)HIWORD(lParam));
  case WM_MOUSEWHEEL:
    return mw->WMMouseWheel(hwnd, LOWORD(wParam), (short int)HIWORD(wParam),
      (short int)LOWORD(lParam), (short int)HIWORD(lParam));
  case WM_CLOSE:
    return mw->WMClose(hwnd);
  case WM_DESTROY:
    return mw->WMDestroy(hwnd);
  /*case WM_CHAR:
    return mw->WMChar(hwnd, (wchar_t)wParam, lParam);
  case WM_KEYDOWN:
    return mw->WMKeyDown(hwnd, (int)wParam, lParam);*/
  default:
    return CallWindowProc(DefWindowProc, hwnd, uMsg, wParam, lParam);
  }
}

LRESULT CALLBACK StatusWndProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
  WNDPROC pwPrevProc = (WNDPROC)GetWindowLongPtr(hwnd, GWLP_USERDATA);

  switch(uMsg)
  {
  case WM_COMMAND:
    return SendMessage(GetParent(hwnd), uMsg, wParam, lParam);
  default:
    return CallWindowProc(pwPrevProc, hwnd, uMsg, wParam, lParam);
  }
}

CMainWnd::CMainWnd(HINSTANCE hInstance)
{
  m_hInstance = hInstance;
  m_sAppPath = NULL;
  GetAppPath();

  WNDCLASSEX wc;
  wc.cbSize = sizeof(WNDCLASSEX);
  wc.style = CS_DBLCLKS | CS_OWNDC;
  wc.lpfnWndProc = MainWndProc;
  wc.cbClsExtra = 0;
  wc.cbWndExtra = 0;
  wc.hInstance = hInstance;
  wc.hIcon = LoadIcon(hInstance, L"ICO_1_MAIN");
  wc.hCursor = LoadCursor(NULL, IDC_ARROW);
  wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
  wc.lpszMenuName = L"MAINMENU";
  wc.lpszClassName = L"DRAWMAINFORM";
  wc.hIconSm = NULL;

  RegisterClassEx(&wc);

  m_hWnd = 0;
  //m_hToolBar = 0;
  m_hStatus = 0;
  m_hPlusCur = LoadCursor(hInstance, L"CUR_PLUS");

  m_iDrawMode = modSelect;
  m_iButton = 0;
  m_iToolMode = tolNone;

  m_cViewOrigin.x = 0.0;
  m_cViewOrigin.y = 0.0;
  m_cLastSnapPt.x = -100;
  m_cLastSnapPt.y = -100;
  m_pDrawBuffer = NULL;

  m_redPen = new Pen(Color(255, 255, 0, 0), 1);

  m_lSelColor = 0xFF24E907; //0x00008888;
  m_lHighColor = 0xFFEDD52C; //0x00888800;
  m_lActiveColor = 0xFFFF0000;
  m_lSelFillColor = 0xF0127503; //0x00008888;
  m_lHighFillColor = 0xF0776B16; //0x00888800;
  m_lActiveFillColor = 0xF07F0000;

  m_pDrawObjects = new CDataList();
  m_pUndoObjects = new CDataList();
  m_iRedoCount = 0;

  m_pActiveObject = NULL;
  m_pHighObject = NULL;
  m_pSelForDimen = NULL;
  m_bPaperUnits = false;
  m_iDrawGridMode = 0;
  m_iHighDimen = -2;

  wcscpy(m_cFSR.cPaperSize.wsPaperSizeName, L"A4");
  m_cFSR.cPaperSize.dPaperWidth = 210.0;
  m_cFSR.cPaperSize.dPaperHeight = 297.0;
  m_cFSR.bPortrait = false;
  wcscpy(m_cFSR.cLenUnit.wsName, L"millimeter");
  wcscpy(m_cFSR.cLenUnit.wsAbbrev, L"mm");
  m_cFSR.cLenUnit.dBaseToUnit = 1.0;
  wcscpy(m_cFSR.cLenUnit.wsAbbrev2, L"");
  m_cFSR.dScaleNomin = 1.0;
  m_cFSR.dScaleDenom = 1.0;
  wcscpy(m_cFSR.cAngUnit.wsName, L"degree");
  wcscpy(m_cFSR.cAngUnit.wsAbbrev, L"deg");
  m_cFSR.cAngUnit.dBaseToUnit = 1.0;
  wcscpy(m_cFSR.cAngUnit.wsAbbrev2, L"\xB0");
  wcscpy(m_cFSR.cPaperUnit.wsName, L"millimeter");
  wcscpy(m_cFSR.cPaperUnit.wsAbbrev, L"mm");
  m_cFSR.cPaperUnit.dBaseToUnit = 1.0;
  wcscpy(m_cFSR.cPaperUnit.wsAbbrev2, L"");
  wcscpy(m_cFSR.cGraphUnit.wsName, L"millimeter");
  wcscpy(m_cFSR.cGraphUnit.wsAbbrev, L"mm");
  m_cFSR.cGraphUnit.dBaseToUnit = 1.0;
  wcscpy(m_cFSR.cGraphUnit.wsAbbrev2, L"");
  //m_cFSR.iAngUnit = 0;
  m_cFSR.dAngGrid = 15.0;
  m_cFSR.dXGrid = 10.0;
  m_cFSR.dYGrid = 10.0;
  m_cFSR.dDefLineWidth = 0.25;
  m_cFSR.iArrowType = 1;
  m_cFSR.dArrowLen = 4.0;
  m_cFSR.dArrowWidth = 2.0;
  m_cFSR.bFontAttrs = 0;
  m_cFSR.dFontSize = 5.0;
  m_cFSR.dBaseLine = 1.0;
  wcscpy(m_cFSR.wsFontFace, L"Arial");
  wcscpy(m_cFSR.wsLengthMask, L"[D:2]");
  wcscpy(m_cFSR.wsAngleMask, L"[r:2]\xB0");

  m_wsStatus2Msg[0] = 0;
  m_wsStatus3Msg[0] = 0;
  m_iRestrictSet = -1;
  m_iRestrictSet2 = -1;
  m_wsFileName[0] = 0;
  m_iLastExportType = 0;

  m_pFileSetupDlg = new CDFileSetupDlg(m_hInstance, m_sAppPath);
  m_pLineStyleDlg = new CDLineStyleDlg(m_hInstance);
  m_pDimEditDlg = new CDDimEditDlg(m_hInstance);
  m_pScaleDlg = new CDScaleDlg(m_hInstance);
  m_pStatDlg = new CDStatDlg(m_hInstance);
  m_pSnapDlg = new CDSnapDlg(m_hInstance);

  m_cMeasPoint1.bIsSet = false;
  m_cMeasPoint2.bIsSet = false;
  m_cMeasPoint3.bIsSet = false;
  m_cLastDynPt.bIsSet = false;
  m_bHasChanged = true;

  // clipboard
  m_iClipboardFormat = RegisterClipboardFormat(L"_STEAMCAD_GEOMETRY");
  m_iClipDataLen = 0;
  m_hClipData = 0;

  m_iRegRasterCount = 0;

  GdiplusStartupInput gdiplusStartupInput;
  GdiplusStartup(&m_gdiplusToken, &gdiplusStartupInput, NULL);
  CoInitialize(NULL);
}

CMainWnd::~CMainWnd()
{
  CoUninitialize();
  GdiplusShutdown(m_gdiplusToken);
  if(m_pActiveObject) delete m_pActiveObject;

  delete m_pSnapDlg;
  delete m_pStatDlg;
  delete m_pScaleDlg;
  delete m_pDimEditDlg;
  delete m_pLineStyleDlg;
  delete m_pFileSetupDlg;

  delete m_pUndoObjects;
  delete m_pDrawObjects;
  //delete m_pToolBar;

  if(m_iClipDataLen > 0) GlobalFree(m_hClipData);

  if(m_pDrawBuffer) delete m_pDrawBuffer;
  delete m_redPen;

  free(m_sAppPath);
}

HWND CMainWnd::DisplayWindow()
{
  m_hWnd = CreateWindowEx(WS_EX_APPWINDOW, L"DRAWMAINFORM", L"Steam CAD",
    //WS_POPUPWINDOW | WS_CAPTION | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_THICKFRAME,
    WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
    CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL,
    m_hInstance, (LPVOID)this);

  LoadSettings(m_hWnd);

  if(fabs(m_cFSR.dScaleDenom) > g_dPrec)
    m_dDrawScale = m_cFSR.dScaleNomin/m_cFSR.dScaleDenom;
  else m_dDrawScale = 1.0;

  GetPageDims();
  CDFileAttrs cFAttrs;
  FilePropsToData(&cFAttrs);
  m_pDrawObjects->SetFileAttrs(&cFAttrs, true);
  SetTitle(m_hWnd, true);

  HMENU hMenu = GetMenu(m_hWnd);
  //UpdateSnapMenu(hMenu);

  UINT uCheck = MF_BYCOMMAND | MF_CHECKED;
  CheckMenuItem(hMenu, IDM_MODESELECT, uCheck);

  ViewFitCmd(m_hWnd, 0, 0);

  uCheck = MF_BYCOMMAND;
  if(m_bPaperUnits) uCheck |= MF_CHECKED;
  else uCheck |= MF_UNCHECKED;
  CheckMenuItem(hMenu, IDM_EDITPAPERUNITS, uCheck);

  uCheck = MF_BYCOMMAND;
  if(m_iDrawGridMode & 1) uCheck |= MF_CHECKED;
  else uCheck |= MF_UNCHECKED;
  CheckMenuItem(hMenu, IDM_VIEWGRIDPTS, uCheck);

  uCheck = MF_BYCOMMAND;
  if(m_iDrawGridMode & 2) uCheck |= MF_CHECKED;
  else uCheck |= MF_UNCHECKED;
  CheckMenuItem(hMenu, IDM_VIEWGRIDLNS, uCheck);
  return(m_hWnd);
}

//extern HWND g_hStatus;

LRESULT CMainWnd::WMCreate(HWND hwnd, LPCREATESTRUCT lpcs)
{
  SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)lpcs->lpCreateParams);

  /*m_hToolBar = CreateWindowEx(0, TOOLBARCLASSNAME, (LPCTSTR)NULL,
    WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, hwnd, (HMENU)IDC_STATUS,
    m_hInstance, NULL);

  m_pToolBar = new CDToolbar(m_hToolBar, m_hInstance, GetMenu(hwnd));*/

  m_hStatus = CreateWindowEx(WS_EX_CONTROLPARENT, STATUSCLASSNAME, (LPCTSTR)NULL,
    WS_CHILD | WS_VISIBLE, 0, 0, 0, 0, hwnd, (HMENU)IDC_STATUS,
    m_hInstance, NULL);
    SendMessage(m_hStatus, SB_SETUNICODEFORMAT, (WPARAM)TRUE, 0);
//g_hStatus = m_hStatus;

  INT iWidth[3] = {150, 480, -1};
  SendMessage(m_hStatus, SB_SETPARTS, 3, (LPARAM)&iWidth);

  /*m_hProg = CreateWindowEx(0, PROGRESS_CLASS, NULL,
    WS_CHILD, 0, 0, 0, 0, m_hStatus,
    (HMENU)0, m_hInstance, NULL);*/

  GetDeviceToUnitScale(hwnd);

  RECT rc;
  /*GetClientRect(m_hToolBar, &rc);
  m_iToolBarHeight = rc.bottom + 2;*/
  m_iToolBarHeight = 0;
  m_cViewOrigin.y = m_iToolBarHeight;

  GetClientRect(m_hStatus, &rc);
  m_iStatusHeight = rc.bottom;

  HFONT hFnt = (HFONT)SendMessage(m_hStatus, WM_GETFONT, 0, 0);

  m_hEdt1 = CreateWindowEx(WS_EX_CONTROLPARENT, L"EDIT", NULL,
    WS_CHILD | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 280, 2, 50,
    m_iStatusHeight - 3, m_hStatus, (HMENU)IDC_EDT1, m_hInstance, NULL);
  SendMessage(m_hEdt1, WM_SETFONT, (WPARAM)hFnt, 0);
  SendMessage(m_hEdt1, EM_LIMITTEXT, 64, 0);
  m_hEdt2 = CreateWindowEx(WS_EX_CONTROLPARENT, L"EDIT", NULL,
    WS_CHILD | WS_BORDER | ES_AUTOHSCROLL | WS_TABSTOP, 425, 2, 50, m_iStatusHeight - 3,
    m_hStatus, (HMENU)IDC_EDT2, m_hInstance, NULL);
  SendMessage(m_hEdt2, WM_SETFONT, (WPARAM)hFnt, 0);
  SendMessage(m_hEdt2, EM_LIMITTEXT, 64, 0);

  m_hLab1 = CreateWindowEx(0, L"STATIC", NULL,
    WS_CHILD | SS_LEFT, 340, 4, 80, m_iStatusHeight - 5,
    m_hStatus, (HMENU)IDC_LAB1, m_hInstance, NULL);
  wchar_t buf[64];
  LoadString(m_hInstance, IDS_NUMCOPIES, buf, 64);
  SendMessage(m_hLab1, WM_SETFONT, (WPARAM)hFnt, 0);
  SendMessage(m_hLab1, WM_SETTEXT, 0, (LPARAM)buf);

  m_hChB1 = CreateWindowEx(0, L"BUTTON", NULL,
    WS_CHILD | BS_AUTOCHECKBOX, 340, 4, 160, m_iStatusHeight - 5,
    m_hStatus, (HMENU)IDC_CHB1, m_hInstance, NULL);
  LoadString(m_hInstance, IDS_KEEPPATHDIR, buf, 64);
  SendMessage(m_hChB1, WM_SETFONT, (WPARAM)hFnt, 0);
  SendMessage(m_hChB1, WM_SETTEXT, 0, (LPARAM)buf);
  SendMessage(m_hChB1, BM_SETCHECK, (WPARAM)BST_CHECKED, 0);

  WNDPROC wPrevProc = (WNDPROC)SetWindowLongPtr(m_hStatus, GWLP_WNDPROC, (LONG_PTR)StatusWndProc);
  SetWindowLongPtr(m_hStatus, GWLP_USERDATA, (LONG_PTR)wPrevProc);

  return(0);
}

LRESULT CMainWnd::WMCommand(HWND hwnd, WORD wNotifyCode, WORD wID, HWND hwndCtl)
{
  switch(wID)
  {
  case IDM_FILENEW:
    return FileNewCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_FILEOPEN:
    return FileOpenCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_FILESAVE:
    return FileSaveCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_FILESAVEAS:
    return FileSaveAsCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_FILESAVESEL:
    return FileSaveSelCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_FILEINCLUDE:
    return FileIncludeCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_FILEEXPORT:
    return FileExportCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_FILEPROPS:
    return FilePropsCmd(hwnd, wNotifyCode, hwndCtl);
  //case IDM_FILEPRINTSET:
  //  return FilePrintSetCmd(hwnd, wNotifyCode, hwndCtl);
  //case IDM_FILEPRINT:
  //  return FilePrintCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_FILEEXIT:
    return FileExitCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_ENABLESNAP:
    m_pHighObject->SetSnapTo(true);
    return 0;
  case IDM_DISABLESNAP:
    m_pHighObject->SetSnapTo(false);
    return 0;
  case IDM_MODESELECT:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modSelect);
  case IDM_MODELINE:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modLine);
  case IDM_MODERECT:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modRectangle);
  case IDM_MODECIRCLE:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modCircle);
  case IDM_MODEELLIPSE:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modEllipse);
  case IDM_MODEARCELLIPSE:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modArcElps);
  case IDM_MODEHYPERBOLA:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modHyperbola);
  case IDM_MODEPARABOLA:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modParabola);
  case IDM_MODESPLINE:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modSpline);
  case IDM_MODEEVEOLVENT:
    return ModeCmd(hwnd, wNotifyCode, hwndCtl, modEvolvent);
  case IDM_MODEDIMEN:
    return ToolsCmd(hwnd, wNotifyCode, hwndCtl, tolDimen);
  case IDM_EDITCOPY:
    return EditCopyCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITCUT:
    return EditCutCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITPASTE:
    return EditPasteCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITDELETE:
    return EditDeleteCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITDELLASTPT:
    return EditDelLastPtCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITCOPYPAR:
    return EditCopyParCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITMOVE:
    return EditMoveCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITROTATE:
    return EditRotateCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITMIRROR:
    return EditMirrorCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITDISTRIBUTE:
    return EditDistributeCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITLINESTYLE:
    return EditLineStyleCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITTOGGLESNAP:
    return EditToggleSnapCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITPAPERUNITS:
    return EditPaperUnitsCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITUNDO:
    return EditUndoCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITREDO:
    return EditRedoCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_EDITCONFIRM:
    return EditConfirmCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_VIEWFITALL:
    return ViewFitCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_VIEWACTSIZE:
    return ViewActSizeCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_VIEWGRIDPTS:
    return ViewGridCmd(hwnd, wNotifyCode, hwndCtl, 1);
  case IDM_VIEWGRIDLNS:
    return ViewGridCmd(hwnd, wNotifyCode, hwndCtl, 2);
  /*case IDM_SNAPELEMENT:
  case IDM_SNAPENDPOINT:
  case IDM_SNAPMIDPOINT:
  case IDM_SNAPINTERSECT:
    return SnapCmd(hwnd, wNotifyCode, hwndCtl, wID - IDM_SNAPELEMENT);*/
  case IDM_TOOLSKNIFE:
    return ToolsCmd(hwnd, wNotifyCode, hwndCtl, tolKnife);
  case IDM_TOOLSROUND:
    return ToolsCmd(hwnd, wNotifyCode, hwndCtl, tolRound);
  case IDM_TOOLSEXTEND:
    return ToolsCmd(hwnd, wNotifyCode, hwndCtl, tolExtend);
  case IDM_TOOLSCONFLICTS:
    return ToolsCmd(hwnd, wNotifyCode, hwndCtl, tolConflict);
  case IDM_TOOLSEDITSPLINE:
    return ToolsCmd(hwnd, wNotifyCode, hwndCtl, tolEditSpline);
  case IDM_TOOLSMEASURE:
    return ToolsCmd(hwnd, wNotifyCode, hwndCtl, tolMeas);
  case IDM_TOOLSMEASUREANGLE:
    return ToolsCmd(hwnd, wNotifyCode, hwndCtl, tolMeasAngle);
  case IDM_TOOLSMEASURELENGTH:
    return ToolsCmd(hwnd, wNotifyCode, hwndCtl, tolMeasLength);
  //case IDM_TOOLSBREAK:
  //  return ToolsBreakCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_TOOLSCALE:
    return ToolsScaleCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_TOOLSTAT:
    return ToolsStatCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_PATHCREATE:
    return PathCreateCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_PATHBREAK:
    return PathBreakCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_PATHAREA:
    return PathAreaCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_PATHGROUP:
    return PathGroupCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_PATHUNGROUP:
    return PathUngroupCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_PATHMOVEUP:
    return PathMoveUpCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_PATHMOVEDOWN:
    return PathMoveDownCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_PATHMOVETOP:
    return PathMoveTopCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_PATHMOVEBOTTOM:
    return PathMoveBottomCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_RASTERIMPORT:
    return RasterImportCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_RASTERREGISTER:
    return RasterRegisterCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_RASTERHIDE:
    return RasterHideCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_RASTERSHOW:
    return RasterShowCmd(hwnd, wNotifyCode, hwndCtl);
  case IDM_HELPCONTENT:
    return HelpContentCmd(hwnd, wNotifyCode, hwndCtl);
  case IDC_EDT1:
    return Edit1Cmd(hwnd, wNotifyCode, hwndCtl);
  case IDC_EDT2:
    return Edit2Cmd(hwnd, wNotifyCode, hwndCtl);
  default:
    return(0);
  }
}

LRESULT CMainWnd::WMClose(HWND hwnd)
{
  if(!PromptForSave(hwnd)) return 0;

  SaveSettings(hwnd);
  DestroyWindow(hwnd);
  return(0);
}

LRESULT CMainWnd::WMSize(HWND hwnd, WPARAM fwSizeType, WORD nWidth, WORD nHeight)
{
  SendMessage(m_hStatus, WM_SIZE, 0, 0);

  /*RECT R1, rc;
  m_pToolBar->Resize(m_hToolBar, &R1, 0);

  SendMessage(m_hStatus, SB_GETRECT, 0, (LPARAM)&rc);
  SetWindowPos(m_hProg, NULL, rc.left, rc.top, rc.right - rc.left,
    rc.bottom - rc.top, SWP_NOZORDER);

  InvalidateRect(hwnd, NULL, TRUE);*/

  if(m_pDrawBuffer) delete m_pDrawBuffer;
  m_pDrawBuffer = new Bitmap(nWidth, nHeight);

  CDRect cdr;
  cdr.cPt1.x = -m_cViewOrigin.x/m_dUnitScale;
  cdr.cPt1.y = (m_iToolBarHeight - m_cViewOrigin.y)/m_dUnitScale;
  cdr.cPt2.x = (nWidth - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt2.y = (nHeight - m_iStatusHeight - m_cViewOrigin.y)/m_dUnitScale;
  m_pDrawObjects->BuildAllPrimitives(&cdr);

  m_iButton = 0;

  return(0);
}

LRESULT CMainWnd::WMDestroy(HWND hwnd)
{
  PostQuitMessage(0);
  return(0);
}

void CMainWnd::GetAppPath()
{
  wchar_t *buf = GetCommandLine();
  int slen = wcslen(buf);
  wchar_t *newbuf = (wchar_t*)malloc((slen + 1)*sizeof(wchar_t));
  wcscpy(newbuf, buf);
  wchar_t *bufstart = newbuf, *bufend = NULL;
  if(bufstart[0] == '"') bufend = wcschr(++bufstart, '"');
  else bufend = wcschr(bufstart, ' ');

  wchar_t *qbufstart = bufend;
  qbufstart++;

  if(bufend) *bufend = 0;
  bufend = wcsrchr(bufstart, '\\');

  slen = bufend - bufstart + 1;
  m_sAppPath = (wchar_t*)malloc((slen + 1)*sizeof(wchar_t));
  wcsncpy(m_sAppPath, bufstart, slen);
  m_sAppPath[slen] = 0;
}

void CMainWnd::LoadSettings(HWND hwnd)
{
  LPWSTR ininame = (LPWSTR)malloc((wcslen(m_sAppPath) + 32)*sizeof(wchar_t));
  wcscpy(ininame, m_sAppPath);
  wcscat(ininame, L"SteamCAD2.xml");

  CXMLReader* pRdr = new CXMLReader(ininame);

  IXMLDOMElement* pElem = pRdr->OpenSection(L"MainForm");
  int i;

  if(pElem)
  {
    WINDOWPLACEMENT wndpl;
    wndpl.length = sizeof(WINDOWPLACEMENT);
    wndpl.flags = 0;
    wndpl.showCmd = SW_SHOWNORMAL;
    wndpl.rcNormalPosition.left = 20;
    wndpl.rcNormalPosition.top = 20;
    wndpl.rcNormalPosition.right = 400;
    wndpl.rcNormalPosition.bottom = 300;
    if(pRdr->GetIntValue(pElem, L"WindowState", &i)) wndpl.showCmd = i;
    if(pRdr->GetIntValue(pElem, L"Left", &i)) wndpl.rcNormalPosition.left = i;
    if(pRdr->GetIntValue(pElem, L"Top", &i)) wndpl.rcNormalPosition.top = i;
    if(pRdr->GetIntValue(pElem, L"Right", &i)) wndpl.rcNormalPosition.right = i;
    if(pRdr->GetIntValue(pElem, L"Bottom", &i)) wndpl.rcNormalPosition.bottom = i;

    pElem->Release();

    SetWindowPlacement(hwnd, &wndpl);
  }

  pElem = pRdr->OpenSection(L"DrawSettings");
  if(pElem)
  {
    if(pRdr->GetIntValue(pElem, L"PaperUnits", &i)) m_bPaperUnits = i;
    if(pRdr->GetIntValue(pElem, L"LastExportType", &i)) m_iLastExportType = i;
    if(pRdr->GetIntValue(pElem, L"DrawGridMode", &i)) m_iDrawGridMode = i;

    pElem->Release();
  }

  double d;
  BYTE b;
  IXMLDOMElement *pE1;

  pElem = pRdr->OpenSection(L"PageSettings");
  if(pElem)
  {
    pE1 = pRdr->OpenSubSection(pElem, L"PaperSize");
    if(pE1)
    {
      pRdr->GetStringValueBuf(pE1, L"PaperName", m_cFSR.cPaperSize.wsPaperSizeName, 64);
      if(pRdr->GetDoubleValue(pE1, L"PaperWidth", &d)) m_cFSR.cPaperSize.dPaperWidth = d;
      if(pRdr->GetDoubleValue(pE1, L"PaperHeight", &d)) m_cFSR.cPaperSize.dPaperHeight = d;
      pE1->Release();
    }
    if(pRdr->GetByteValue(pElem, L"Portrait", &b)) m_cFSR.bPortrait = b;
    pE1 = pRdr->OpenSubSection(pElem, L"LengthUnit");
    if(pE1)
    {
      pRdr->GetStringValueBuf(pE1, L"UnitName", m_cFSR.cLenUnit.wsName, 32);
      pRdr->GetStringValueBuf(pE1, L"UnitAbbrev", m_cFSR.cLenUnit.wsAbbrev, 8);
      if(pRdr->GetDoubleValue(pE1, L"UnitScale", &d)) m_cFSR.cLenUnit.dBaseToUnit = d;
      pRdr->GetStringValueBuf(pE1, L"UnitAbbrev2", m_cFSR.cLenUnit.wsAbbrev2, 8);
      pE1->Release();
    }
    if(pRdr->GetDoubleValue(pElem, L"ScaleNomin", &d)) m_cFSR.dScaleNomin = d;
    if(pRdr->GetDoubleValue(pElem, L"ScaleDenom", &d)) m_cFSR.dScaleDenom = d;
    pE1 = pRdr->OpenSubSection(pElem, L"AngularUnit");
    if(pE1)
    {
      pRdr->GetStringValueBuf(pE1, L"UnitName", m_cFSR.cAngUnit.wsName, 32);
      pRdr->GetStringValueBuf(pE1, L"UnitAbbrev", m_cFSR.cAngUnit.wsAbbrev, 8);
      if(pRdr->GetDoubleValue(pE1, L"UnitScale", &d)) m_cFSR.cAngUnit.dBaseToUnit = d;
      pRdr->GetStringValueBuf(pE1, L"UnitAbbrev2", m_cFSR.cAngUnit.wsAbbrev2, 8);
      pE1->Release();
    }
    pE1 = pRdr->OpenSubSection(pElem, L"PaperUnit");
    if(pE1)
    {
      pRdr->GetStringValueBuf(pE1, L"UnitName", m_cFSR.cPaperUnit.wsName, 32);
      pRdr->GetStringValueBuf(pE1, L"UnitAbbrev", m_cFSR.cPaperUnit.wsAbbrev, 8);
      if(pRdr->GetDoubleValue(pE1, L"UnitScale", &d)) m_cFSR.cPaperUnit.dBaseToUnit = d;
      pRdr->GetStringValueBuf(pE1, L"UnitAbbrev2", m_cFSR.cPaperUnit.wsAbbrev2, 8);
      pE1->Release();
    }
    pE1 = pRdr->OpenSubSection(pElem, L"GraphUnit");
    if(pE1)
    {
      pRdr->GetStringValueBuf(pE1, L"UnitName", m_cFSR.cGraphUnit.wsName, 32);
      pRdr->GetStringValueBuf(pE1, L"UnitAbbrev", m_cFSR.cGraphUnit.wsAbbrev, 8);
      if(pRdr->GetDoubleValue(pE1, L"UnitScale", &d)) m_cFSR.cGraphUnit.dBaseToUnit = d;
      pRdr->GetStringValueBuf(pE1, L"UnitAbbrev2", m_cFSR.cGraphUnit.wsAbbrev2, 8);
      pE1->Release();
    }
    if(pRdr->GetDoubleValue(pElem, L"AngularGrid", &d)) m_cFSR.dAngGrid = d;
    if(pRdr->GetDoubleValue(pElem, L"XGrid", &d)) m_cFSR.dXGrid = d;
    if(pRdr->GetDoubleValue(pElem, L"YGrid", &d)) m_cFSR.dYGrid = d;
    if(pRdr->GetDoubleValue(pElem, L"DefLineWidth", &d)) m_cFSR.dDefLineWidth = d;
    pE1 = pRdr->OpenSubSection(pElem, L"Dimensioning");
    if(pE1)
    {
      if(pRdr->GetIntValue(pE1, L"ArrowType", &i)) m_cFSR.iArrowType = i;
      if(pRdr->GetDoubleValue(pE1, L"ArrowLength", &d)) m_cFSR.dArrowLen = d;
      if(pRdr->GetDoubleValue(pE1, L"ArrowWidth", &d)) m_cFSR.dArrowWidth = d;
      if(pRdr->GetIntValue(pE1, L"FontAttrs", &i)) m_cFSR.bFontAttrs = i;
      if(pRdr->GetDoubleValue(pE1, L"FontSize", &d)) m_cFSR.dFontSize = d;
      if(pRdr->GetDoubleValue(pE1, L"BaseLine", &d)) m_cFSR.dBaseLine = d;
      pRdr->GetStringValueBuf(pE1, L"FontFace", m_cFSR.wsFontFace, 64);
      pRdr->GetStringValueBuf(pE1, L"LengthMask", m_cFSR.wsLengthMask, 64);
      pRdr->GetStringValueBuf(pE1, L"AngleMask", m_cFSR.wsAngleMask, 64);
      pE1->Release();
    }
    pElem->Release();
  }

  m_pFileSetupDlg->RestoreSettings(pRdr);
  m_pLineStyleDlg->RestoreSettings(pRdr);
  m_pDimEditDlg->RestoreSettings(pRdr);
  m_pScaleDlg->RestoreSettings(pRdr);
  m_pStatDlg->RestoreSettings(pRdr);
  m_pSnapDlg->RestoreSettings(pRdr);

  delete pRdr;
  return;
}

void CMainWnd::SaveSettings(HWND hwnd)
{
  //wchar_t wsBuf[64];

  LPWSTR ininame = (LPWSTR)malloc((wcslen(m_sAppPath) + 40)*sizeof(wchar_t));
  wcscpy(ininame, m_sAppPath);
  wcscat(ininame, L"SteamCAD2.xml");

  CXMLWritter* pWrit = new CXMLWritter(ininame);
  pWrit->WriteComment(L"SteamCAD2 Workspace Settings");
  pWrit->CreateRoot(L"Settings");

  IXMLDOMElement* pElem = NULL;
  IXMLDOMElement* pE1 = NULL;

  WINDOWPLACEMENT wndpl;
  wndpl.length = sizeof(WINDOWPLACEMENT);
  wndpl.flags = 0;
  wndpl.showCmd = 0;
  GetWindowPlacement(hwnd, &wndpl);

  pElem = pWrit->CreateSection(L"MainForm");
  pWrit->AddIntValue(pElem, L"WindowState", wndpl.showCmd);
  pWrit->AddIntValue(pElem, L"Left", wndpl.rcNormalPosition.left);
  pWrit->AddIntValue(pElem, L"Top", wndpl.rcNormalPosition.top);
  pWrit->AddIntValue(pElem, L"Right", wndpl.rcNormalPosition.right);
  pWrit->AddIntValue(pElem, L"Bottom", wndpl.rcNormalPosition.bottom);
  pElem->Release();

  pElem = pWrit->CreateSection(L"DrawSettings");
  pWrit->AddIntValue(pElem, L"PaperUnits", (int)m_bPaperUnits);
  pWrit->AddIntValue(pElem, L"LastExportType", (int)m_iLastExportType);
  pWrit->AddIntValue(pElem, L"DrawGridMode", (int)m_iDrawGridMode);
  pElem->Release();

  pElem = pWrit->CreateSection(L"PageSettings");
  pE1 = pWrit->CreateSubSection(pElem, L"PaperSize");
  pWrit->AddStringValue(pE1, L"PaperName", m_cFSR.cPaperSize.wsPaperSizeName);
  pWrit->AddDoubleValue(pE1, L"PaperWidth", m_cFSR.cPaperSize.dPaperWidth);
  pWrit->AddDoubleValue(pE1, L"PaperHeight", m_cFSR.cPaperSize.dPaperHeight);
  pE1->Release();
  pWrit->AddByteValue(pElem, L"Portrait", m_cFSR.bPortrait);
  pE1 = pWrit->CreateSubSection(pElem, L"LengthUnit");
  pWrit->AddStringValue(pE1, L"UnitName", m_cFSR.cLenUnit.wsName);
  pWrit->AddStringValue(pE1, L"UnitAbbrev", m_cFSR.cLenUnit.wsAbbrev);
  pWrit->AddDoubleValue(pE1, L"UnitScale", m_cFSR.cLenUnit.dBaseToUnit);
  pWrit->AddStringValue(pE1, L"UnitAbbrev2", m_cFSR.cLenUnit.wsAbbrev2);
  pE1->Release();
  pWrit->AddDoubleValue(pElem, L"ScaleNomin", m_cFSR.dScaleNomin);
  pWrit->AddDoubleValue(pElem, L"ScaleDenom", m_cFSR.dScaleDenom);
  pE1 = pWrit->CreateSubSection(pElem, L"AngularUnit");
  pWrit->AddStringValue(pE1, L"UnitName", m_cFSR.cAngUnit.wsName);
  pWrit->AddStringValue(pE1, L"UnitAbbrev", m_cFSR.cAngUnit.wsAbbrev);
  pWrit->AddDoubleValue(pE1, L"UnitScale", m_cFSR.cAngUnit.dBaseToUnit);
  pWrit->AddStringValue(pE1, L"UnitAbbrev2", m_cFSR.cAngUnit.wsAbbrev2);
  pE1->Release();
  pE1 = pWrit->CreateSubSection(pElem, L"PaperUnit");
  pWrit->AddStringValue(pE1, L"UnitName", m_cFSR.cPaperUnit.wsName);
  pWrit->AddStringValue(pE1, L"UnitAbbrev", m_cFSR.cPaperUnit.wsAbbrev);
  pWrit->AddDoubleValue(pE1, L"UnitScale", m_cFSR.cPaperUnit.dBaseToUnit);
  pWrit->AddStringValue(pE1, L"UnitAbbrev2", m_cFSR.cPaperUnit.wsAbbrev2);
  pE1->Release();
  pE1 = pWrit->CreateSubSection(pElem, L"GraphUnit");
  pWrit->AddStringValue(pE1, L"UnitName", m_cFSR.cGraphUnit.wsName);
  pWrit->AddStringValue(pE1, L"UnitAbbrev", m_cFSR.cGraphUnit.wsAbbrev);
  pWrit->AddDoubleValue(pE1, L"UnitScale", m_cFSR.cGraphUnit.dBaseToUnit);
  pWrit->AddStringValue(pE1, L"UnitAbbrev2", m_cFSR.cGraphUnit.wsAbbrev2);
  pE1->Release();
  //pWrit->AddIntValue(pElem, L"AngularUnit", m_cFSR.iAngUnit);
  pWrit->AddDoubleValue(pElem, L"AngularGrid", m_cFSR.dAngGrid);
  pWrit->AddDoubleValue(pElem, L"XGrid", m_cFSR.dXGrid);
  pWrit->AddDoubleValue(pElem, L"YGrid", m_cFSR.dYGrid);
  pWrit->AddDoubleValue(pElem, L"DefLineWidth", m_cFSR.dDefLineWidth);
  pE1 = pWrit->CreateSubSection(pElem, L"Dimensioning");
  pWrit->AddIntValue(pE1, L"ArrowType", m_cFSR.iArrowType);
  pWrit->AddDoubleValue(pE1, L"ArrowLength", m_cFSR.dArrowLen);
  pWrit->AddDoubleValue(pE1, L"ArrowWidth", m_cFSR.dArrowWidth);
  pWrit->AddIntValue(pE1, L"FontAttrs", m_cFSR.bFontAttrs);
  pWrit->AddDoubleValue(pE1, L"FontSize", m_cFSR.dFontSize);
  pWrit->AddDoubleValue(pE1, L"BaseLine", m_cFSR.dBaseLine);
  pWrit->AddStringValue(pE1, L"FontFace", m_cFSR.wsFontFace);
  pWrit->AddStringValue(pE1, L"LengthMask", m_cFSR.wsLengthMask);
  pWrit->AddStringValue(pE1, L"AngleMask", m_cFSR.wsAngleMask);
  pE1->Release();
  pElem->Release();

  m_pFileSetupDlg->SaveSettings(pWrit);
  m_pLineStyleDlg->SaveSettings(pWrit);
  m_pDimEditDlg->SaveSettings(pWrit);
  m_pScaleDlg->SaveSettings(pWrit);
  m_pStatDlg->SaveSettings(pWrit);
  m_pSnapDlg->SaveSettings(pWrit);

  pWrit->Save();
  delete pWrit;

  free(ininame);
}

LRESULT CMainWnd::WMPaint(HWND hwnd, HDC hdc)
{
  RECT rc;
  if(!GetUpdateRect(hwnd, &rc, TRUE)) return(0);

  Graphics graphics(m_pDrawBuffer);
  graphics.SetSmoothingMode(SmoothingModeAntiAlias);
  SolidBrush whiteBrush(Color::White);
  graphics.FillRectangle(&whiteBrush, (INT)rc.left, (INT)(rc.top - m_iToolBarHeight),
    (INT)(rc.right - rc.left), (INT)(rc.bottom - rc.top + m_iToolBarHeight));

  if(m_iDrawGridMode > 0)
  {
    double dx = m_dUnitScale*m_cFSR.dXGrid;
    double dy = m_dUnitScale*m_cFSR.dYGrid;
    if((dx > 5) && (dy > 5))
    {
      RECT cr;
      GetClientRect(hwnd, &cr);
      HPEN hlPen;

      int iMin = -m_cViewOrigin.x/m_dUnitScale/m_cFSR.dXGrid;
      int iMax = (cr.right - m_cViewOrigin.x)/m_dUnitScale/m_cFSR.dXGrid;
      int jMin = -m_cViewOrigin.y/m_dUnitScale/m_cFSR.dYGrid;
      int jMax = (cr.bottom - m_cViewOrigin.y)/m_dUnitScale/m_cFSR.dYGrid;

      double dGray;
      int iGray;

      if(m_iDrawGridMode & 2)
      {
        if((dx < 200) || (dy < 200))
        {
          if(dx < dy) dGray = 0.7 + 0.2*(200.0 - dx)/195.0;
          else dGray = 0.7 + 0.2*(200.0 - dy)/195.0;
        }
        else dGray = 0.7;
        iGray = 255*dGray;

        Pen grayPen(Color(255, iGray, iGray, iGray), 1);
        for(int i = iMin; i <= iMax; i++)
        {
          dx = m_cViewOrigin.x + (double)i*m_dUnitScale*m_cFSR.dXGrid;
          graphics.DrawLine(&grayPen, (REAL)dx, (REAL)0.0, (REAL)dx, (REAL)cr.bottom);
        }
        for(int j = jMin; j <= jMax; j++)
        {
          dy = m_cViewOrigin.y + (double)j*m_dUnitScale*m_cFSR.dYGrid;
          graphics.DrawLine(&grayPen, (REAL)0, (REAL)dy, (REAL)cr.right, (REAL)dy);
        }
      }

      if(m_iDrawGridMode & 1)
      {
        if(m_iDrawGridMode & 2)
        {
          if((dx < 200) || (dy < 200))
          {
            if(dx < dy) dGray = 0.5 + 0.5*(200.0 - dx)/195.0;
            else dGray = 0.5 + 0.5*(200.0 - dy)/195.0;
          }
          else dGray = 0.5;
        }
        else
        {
          if((dx < 200) || (dy < 200))
          {
            if(dx < dy) dGray = 0.3 + 0.3*(200.0 - dx)/195.0;
            else dGray = 0.3 + 0.3*(200.0 - dy)/195.0;
          }
          else dGray = 0.3;
        }
        iGray = 255*dGray;

        Pen grayPen(Color(255, iGray, iGray, iGray), 1);
        for(int i = iMin; i <= iMax; i++)
        {
          dx = m_cViewOrigin.x + (double)i*m_dUnitScale*m_cFSR.dXGrid;
          for(int j = jMin; j <= jMax; j++)
          {
            dy = m_cViewOrigin.y + (double)j*m_dUnitScale*m_cFSR.dYGrid;
            graphics.DrawEllipse(&grayPen, (REAL)(dx - 1), (REAL)(dy - 1), (REAL)2.0, (REAL)2.0);
          }
        }
      }
    }
  }

  Pen brownPen(Color(255, 128, 76, 0), 1);
  graphics.DrawRectangle(&brownPen, (REAL)m_cViewOrigin.x, (REAL)m_cViewOrigin.y,
    (REAL)(m_dUnitScale*m_dwPage), (REAL)(m_dUnitScale*m_dhPage));

  CDRect cdr;
  cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
  cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

  m_pDrawObjects->BuildAllPrimitives(&cdr);

  PDObject pObj;
  int iObjs = m_pDrawObjects->GetCount();
  for(int i = 0; i < iObjs; i++)
  {
    pObj = m_pDrawObjects->GetItem(i);
    DrawObject(hwnd, &graphics, pObj, 0, -2);
  }

  PAINTSTRUCT ps;
  HDC ldc = BeginPaint(hwnd, &ps);

  Graphics dstgraph(ldc);
  dstgraph.SetSmoothingMode(SmoothingModeAntiAlias);
  dstgraph.DrawImage(m_pDrawBuffer, 0, 0);

  if(m_pHighObject) DrawObject(hwnd, &dstgraph, m_pHighObject, 2, m_iHighDimen);

  int iDynMode = GetDynMode();
  CDLine cPtX;
  cPtX.cOrigin = m_cLastDrawPt;
  if(iDynMode == 1)
  {
    cPtX.bIsSet = m_cLastDynPt.bIsSet;
    cPtX.cDirection = m_cLastDynPt.cOrigin;
  }
  else if(iDynMode == 2)
  {
    cPtX.cDirection.x = 0.0;
    if(IS_LENGTH_VAL(m_iRestrictSet))
    {
      cPtX.cDirection.x = 1.0;
      cPtX.cDirection.y = m_dSavedDist;
    }
  }

  if((m_iDrawMode > modSelect) || (m_iToolMode > tolNone))
  {
    dstgraph.DrawLine(m_redPen, (REAL)(m_cLastSnapPt.x - 10), (REAL)m_cLastSnapPt.y,
      (REAL)(m_cLastSnapPt.x + 10), (REAL)m_cLastSnapPt.y);
    dstgraph.DrawLine(m_redPen, (REAL)m_cLastSnapPt.x, (REAL)(m_cLastSnapPt.y - 10),
      (REAL)m_cLastSnapPt.x, (REAL)(m_cLastSnapPt.y + 10));
    if(m_pActiveObject)
    {
      m_pActiveObject->BuildPrimitives(cPtX, iDynMode, &cdr, 0, NULL, NULL);
      DrawObject(hwnd, &dstgraph, m_pActiveObject, 1, -2);
    }
  }

  if(m_iDrawMode == modRegRaster)
  {
    CDPoint cPt1, cPt2;
    Pen greenPen(Color(255, 0, 255, 0), 1);
    for(int i = 0; i < m_iRegRasterCount/2; i++)
    {
      cPt1.x = m_cViewOrigin.x + m_dUnitScale*m_cRegRasterPoints[2*i].x;
      cPt1.y = m_cViewOrigin.y + m_dUnitScale*m_cRegRasterPoints[2*i].y;
      cPt2.x = m_cViewOrigin.x + m_dUnitScale*m_cRegRasterPoints[2*i + 1].x;
      cPt2.y = m_cViewOrigin.y + m_dUnitScale*m_cRegRasterPoints[2*i + 1].y;
      DrawRegRasterLine(&dstgraph, &greenPen, cPt1, cPt2);
    }
    if((m_iRegRasterCount % 2) > 0)
    {
      cPt1.x = m_cViewOrigin.x + m_dUnitScale*m_cRegRasterPoints[m_iRegRasterCount - 1].x;
      cPt1.y = m_cViewOrigin.y + m_dUnitScale*m_cRegRasterPoints[m_iRegRasterCount - 1].y;
      cPt2.x = m_cViewOrigin.x + m_dUnitScale*m_cLastDrawPt.x;
      cPt2.y = m_cViewOrigin.y + m_dUnitScale*m_cLastDrawPt.y;
      DrawRegRasterLine(&dstgraph, &greenPen, cPt1, cPt2);
    }
  }

  EndPaint(hwnd, &ps);

  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
  cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

  m_pDrawObjects->BuildAllPrimitives(&cdr);
  if(m_pActiveObject)
  {
    m_pActiveObject->BuildPrimitives(cPtX, iDynMode, &cdr, 0, NULL, NULL);
  }

  //SendMessage(m_hStatus, WM_PAINT, 0, 0);
  return 0;
}

bool CMainWnd::PromptForSave(HWND hWnd)
{
  if(!m_pDrawObjects->GetChanged()) return true;

  wchar_t wsCaption[64];
  wchar_t wsPrompt[256];
  LoadString(m_hInstance, IDS_WARNING, wsCaption, 64);
  LoadString(m_hInstance, IDS_FILECHANGED, wsPrompt, 256);

  int iRes = MessageBox(hWnd, wsPrompt, wsCaption, MB_YESNOCANCEL | MB_ICONWARNING);
  if(iRes == IDCANCEL) return false;
  if(iRes == IDNO) return true;

  return SaveFile(hWnd, m_wsFileName, false);
}

bool CMainWnd::SaveFile(HWND hWnd, LPWSTR wsFile, bool bSelectOnly)
{
// if we ever want to return to Win XP or earlier
  /*bool bSave = true;
  if(!wsFile[0])
  {
    wchar_t wsFilter[128], wsCurDir[1];
    wsCurDir[0] = 0;
    LoadString(m_hInstance, IDS_STEAMDRAWFILTER, wsFilter, 128);

    int n = wcslen(wsFilter);
    for(int i = 0; i < n; i++)
    {
      if(wsFilter[i] == 1) wsFilter[i] = 0;
    }

    OPENFILENAME ofn = {sizeof(OPENFILENAME), hWnd, m_hInstance, wsFilter,
      NULL, 0, 0, wsFile, MAX_PATH, NULL, 0, wsCurDir, NULL,
      OFN_ENABLESIZING | OFN_EXPLORER | OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST,
      0, 0, L"sc2", 0, NULL, NULL};

    bSave = GetSaveFileName(&ofn);
  }
  if(!bSave) return false;

  unsigned char cVer = 2;
  LPWSTR sDot = wcsrchr(wsFile, '.');
  if(sDot)
  {
    if(wcsicmp(sDot, L".sdr") == 0) cVer = 1;
  }

  // save the file
  FILE *pf = _wfopen(wsFile, L"wb");
  m_pDrawObjects->SaveToFile(pf, true, bSelectOnly, cVer);
  fclose(pf);

  return true;*/
  bool bSave = true;
  if(!wsFile[0])
  {
    bSave = false;
    IFileSaveDialog *pFileOpen = NULL;
    HRESULT hr = CoCreateInstance(CLSID_FileSaveDialog, NULL, CLSCTX_INPROC_SERVER, IID_IFileSaveDialog, (void**)&pFileOpen);
    if(SUCCEEDED(hr))
    {
      wchar_t wsFilter[128];
      LoadString(m_hInstance, IDS_STEAMDRAWFILTER, wsFilter, 128);

      int iFilterLen = 0;
      int n = wcslen(wsFilter);
      for(int i = 0; i < n; i++)
      {
        if(wsFilter[i] == 1) iFilterLen++;
      }
      iFilterLen /= 2;
      COMDLG_FILTERSPEC *pFilter = NULL;
      if(iFilterLen > 0)
      {
        pFilter = (COMDLG_FILTERSPEC*)malloc(iFilterLen*sizeof(COMDLG_FILTERSPEC));
        int iFilterPos = 0;
        pFilter[iFilterPos++].pszName = wsFilter;
        for(int i = 0; i < n; i++)
        {
          if(wsFilter[i] == 1)
          {
            wsFilter[i] = 0;
            if(iFilterPos < 2*iFilterLen)
            {
              if(iFilterPos % 2 > 0) pFilter[iFilterPos/2].pszSpec = &wsFilter[i + 1];
              else pFilter[iFilterPos/2].pszName = &wsFilter[i + 1];
              iFilterPos++;
            }
          }
        }
        pFileOpen->SetFileTypes(iFilterLen, pFilter);
      }
      pFileOpen->SetDefaultExtension(L"sc2");
      hr = pFileOpen->Show(NULL);
      if(SUCCEEDED(hr))
      {
        IShellItem *pItem;
        hr = pFileOpen->GetResult(&pItem);
        if(SUCCEEDED(hr))
        {
          PWSTR pszFilePath;
          hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);
          if(SUCCEEDED(hr))
          {
            wcscpy(wsFile, pszFilePath);
            bSave = true;
            CoTaskMemFree(pszFilePath);
          }
          pItem->Release();
        }
      }
      if(pFilter) free(pFilter);
      pFileOpen->Release();
    }
  }
  if(!bSave) return false;

  unsigned char cVer = 2;
  LPWSTR sDot = wcsrchr(wsFile, '.');
  if(sDot)
  {
    if(wcsicmp(sDot, L".sdr") == 0) cVer = 1;
  }

  // save the file
  FILE *pf = _wfopen(wsFile, L"wb");
  m_pDrawObjects->SaveToFile(pf, true, bSelectOnly, cVer);
  fclose(pf);

  return true;
}

bool CMainWnd::LoadFile(HWND hWnd, LPWSTR wsFile, bool bClear)
{
// if we ever want to return to Win XP or earlier
/*  wchar_t wsFilter[128], wsCurDir[1];
  wsCurDir[0] = 0;
  LoadString(m_hInstance, IDS_STEAMDRAWFILTER, wsFilter, 128);

  int n = wcslen(wsFilter);
  for(int i = 0; i < n; i++)
  {
    if(wsFilter[i] == 1) wsFilter[i] = 0;
  }

  OPENFILENAME ofn = {sizeof(OPENFILENAME), hWnd, m_hInstance, wsFilter,
    NULL, 0, 0, wsFile, MAX_PATH, NULL, 0, wsCurDir, NULL,
    OFN_ENABLESIZING | OFN_EXPLORER | OFN_FILEMUSTEXIST,
    0, 0, L"sc2", 0, NULL, NULL};

  if(GetOpenFileName(&ofn))
  {
    // load the file
    FILE *pf = _wfopen(wsFile, L"rb");
    bool bRead = m_pDrawObjects->ReadFromFile(pf, true, bClear);
    fclose(pf);
    if(bRead)
    {
      if(bClear)
      {
        DataToFileProps();
        GetPageDims();
        m_pUndoObjects->ClearAll();
        m_iRedoCount = 0;
      }

      RECT rc;
      GetClientRect(hWnd, &rc);
      rc.top += m_iToolBarHeight;
      rc.bottom -= m_iStatusHeight;

      InvalidateRect(hWnd, &rc, TRUE);
      SetTitle(hWnd, true);
    }
    return bRead;
  }
  return false;*/

  bool bRes = false;
  IFileOpenDialog *pFileOpen = NULL;
  HRESULT hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_INPROC_SERVER, IID_IFileOpenDialog, (void**)&pFileOpen);
  if(SUCCEEDED(hr))
  {
    wchar_t wsFilter[128];
    LoadString(m_hInstance, IDS_STEAMDRAWFILTER, wsFilter, 128);

    int iFilterLen = 0;
    int n = wcslen(wsFilter);
    for(int i = 0; i < n; i++)
    {
      if(wsFilter[i] == 1) iFilterLen++;
    }
    iFilterLen /= 2;
    COMDLG_FILTERSPEC *pFilter = NULL;
    if(iFilterLen > 0)
    {
      pFilter = (COMDLG_FILTERSPEC*)malloc(iFilterLen*sizeof(COMDLG_FILTERSPEC));
      int iFilterPos = 0;
      pFilter[iFilterPos++].pszName = wsFilter;
      for(int i = 0; i < n; i++)
      {
        if(wsFilter[i] == 1)
        {
          wsFilter[i] = 0;
          if(iFilterPos < 2*iFilterLen)
          {
            if(iFilterPos % 2 > 0) pFilter[iFilterPos/2].pszSpec = &wsFilter[i + 1];
            else pFilter[iFilterPos/2].pszName = &wsFilter[i + 1];
            iFilterPos++;
          }
        }
      }
      pFileOpen->SetFileTypes(iFilterLen, pFilter);
    }
    hr = pFileOpen->Show(NULL);
    if(SUCCEEDED(hr))
    {
      IShellItem *pItem;
      hr = pFileOpen->GetResult(&pItem);
      if(SUCCEEDED(hr))
      {
        PWSTR pszFilePath;
        hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);
        if(SUCCEEDED(hr))
        {
          // load the file
          FILE *pf = _wfopen(pszFilePath, L"rb");
          bool bRead = m_pDrawObjects->ReadFromFile(pf, true, bClear);
          fclose(pf);
          if(bRead)
          {
            wcscpy(wsFile, pszFilePath);
            if(bClear)
            {
              DataToFileProps();
              GetPageDims();
              m_pUndoObjects->ClearAll();
              m_iRedoCount = 0;
            }

            RECT rc;
            GetClientRect(hWnd, &rc);
            rc.top += m_iToolBarHeight;
            rc.bottom -= m_iStatusHeight;

            InvalidateRect(hWnd, &rc, FALSE);
            SetTitle(hWnd, true);
          }
          bRes = bRead;

          CoTaskMemFree(pszFilePath);
        }
        pItem->Release();
      }
    }
    if(pFilter) free(pFilter);
    pFileOpen->Release();
  }
  return bRes;
}

LRESULT CMainWnd::FileNewCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(!PromptForSave(hwnd)) return 0;

  if(m_pActiveObject) delete m_pActiveObject;
  m_pActiveObject = NULL;
  m_pHighObject = NULL;
  m_pDrawObjects->ClearAll();
  m_pUndoObjects->ClearAll();
  m_wsFileName[0] = 0;
  m_iRedoCount = 0;

  CDFileAttrs cFAttrs;
  FilePropsToData(&cFAttrs);
  m_pDrawObjects->SetFileAttrs(&cFAttrs, true);

  InvalidateRect(hwnd, NULL, FALSE);
  SetTitle(hwnd, true);
  return 0;
}

LRESULT CMainWnd::FileOpenCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(!PromptForSave(hwnd)) return 0;

  LoadFile(hwnd, m_wsFileName, true);
  return 0;
}

LRESULT CMainWnd::FileSaveCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(SaveFile(hwnd, m_wsFileName, false))
  {
    if(m_wsFileName) SetTitle(hwnd, true);
  }
  return 0;
}

LRESULT CMainWnd::FileSaveAsCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  wchar_t wsNewName[MAX_PATH];
  wsNewName[0] = 0;
  if(SaveFile(hwnd, wsNewName, false))
  {
    wcscpy(m_wsFileName, wsNewName);
    SetTitle(hwnd, true);
  }
  return 0;
}

LRESULT CMainWnd::FileSaveSelCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  wchar_t wsNewName[MAX_PATH];
  wsNewName[0] = 0;
  SaveFile(hwnd, wsNewName, true);
  return 0;
}

LRESULT CMainWnd::FileIncludeCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  wchar_t wsNewName[MAX_PATH];
  wsNewName[0] = 0;
  LoadFile(hwnd, wsNewName, false);
  return 0;
}

LRESULT CMainWnd::FileExportCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  bool bSave = true;
  wchar_t wsFile[MAX_PATH];

  wsFile[0] = 0;
  wchar_t wsFilter[256], wsCurDir[1];
  wsCurDir[0] = 0;
  LoadString(m_hInstance, IDS_STEAMEXPORTFILTER, wsFilter, 256);

  int n = wcslen(wsFilter);
  for(int i = 0; i < n; i++)
  {
    if(wsFilter[i] == 1) wsFilter[i] = 0;
  }

  if(m_wsFileName[0])
  {
    wchar_t *wsSlash = wcsrchr(m_wsFileName, '\\');
    if(wsSlash) wcscpy(wsFile, &wsSlash[1]);
    else wcscpy(wsFile, m_wsFileName);

    wchar_t *wsDot = wcsrchr(wsFile, '.');
    if(wsDot) *wsDot = 0;
  }

  wchar_t sDefExt[4];
  switch(m_iLastExportType)
  {
  case 0:
    wcscpy(sDefExt, L"pdf");
    break;
  case 1:
    wcscpy(sDefExt, L"ps");
    break;
  case 2:
    wcscpy(sDefExt, L"eps");
    break;
  case 3:
    wcscpy(sDefExt, L"png");
    break;
  case 4:
    wcscpy(sDefExt, L"svg");
    break;
  default:
    sDefExt[0] = 0;
  }

  OPENFILENAME ofn = {sizeof(OPENFILENAME), hwnd, m_hInstance, wsFilter,
    NULL, 0, m_iLastExportType + 1, wsFile, MAX_PATH, NULL, 0, wsCurDir, NULL,
    OFN_ENABLESIZING | OFN_EXPLORER | OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST,
    0, 0, sDefExt, 0, NULL, NULL};

  if(!GetSaveFileName(&ofn)) return 0;

  m_iLastExportType = ofn.nFilterIndex - 1;

  // export to the file
  FILE *pf = _wfopen(wsFile, L"wb");
  if(m_iLastExportType < 5)
    ExportCairoFile(m_iLastExportType, pf, m_pDrawObjects, m_pFileSetupDlg->GetUnitList());
  else
    ExportDXFFile(pf, m_pDrawObjects, m_pFileSetupDlg->GetUnitList());
  fclose(pf);

  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  CDRect cdr;
  cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
  cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

  m_pDrawObjects->BuildAllPrimitives(&cdr);
  return 0;
}

void CMainWnd::FilePropsToData(PDFileAttrs pFAttrs)
{
  pFAttrs->dWidth = m_cFSR.cPaperSize.dPaperWidth;
  pFAttrs->dHeight = m_cFSR.cPaperSize.dPaperHeight;
  if(m_cFSR.bPortrait)
  {
    if(pFAttrs->dWidth > pFAttrs->dHeight)
    {
      pFAttrs->dWidth = m_cFSR.cPaperSize.dPaperHeight;
      pFAttrs->dHeight = m_cFSR.cPaperSize.dPaperWidth;
    }
  }
  else if(pFAttrs->dWidth < pFAttrs->dHeight)
  {
    pFAttrs->dWidth = m_cFSR.cPaperSize.dPaperHeight;
    pFAttrs->dHeight = m_cFSR.cPaperSize.dPaperWidth;
  }
  pFAttrs->dScaleNom = m_cFSR.dScaleNomin;
  pFAttrs->dScaleDenom = m_cFSR.dScaleDenom;
  pFAttrs->iArrowType = m_cFSR.iArrowType;
  pFAttrs->cArrowDim.x = m_cFSR.dArrowLen;
  pFAttrs->cArrowDim.y = m_cFSR.dArrowWidth;
  pFAttrs->dFontSize = m_cFSR.dFontSize;
  pFAttrs->dBaseLine = m_cFSR.dBaseLine;
  pFAttrs->bFontAttrs = m_cFSR.bFontAttrs;
  WideCharToMultiByte(CP_UTF8, 0, m_cFSR.wsFontFace, -1, pFAttrs->sFontFace, 64, NULL, NULL);
  WideCharToMultiByte(CP_UTF8, 0, m_cFSR.wsLengthMask, -1, pFAttrs->sLengthMask, 64, NULL, NULL);
  WideCharToMultiByte(CP_UTF8, 0, m_cFSR.wsAngleMask, -1, pFAttrs->sAngleMask, 64, NULL, NULL);
}

void CMainWnd::DataToFileProps()
{
  CDFileAttrs cFAttrs;
  m_pDrawObjects->GetFileAttrs(&cFAttrs);

  if(cFAttrs.dHeight > cFAttrs.dWidth)
  {
    m_cFSR.bPortrait = true;
    m_cFSR.cPaperSize.dPaperWidth = cFAttrs.dHeight;
    m_cFSR.cPaperSize.dPaperHeight = cFAttrs.dWidth;
  }
  else
  {
    m_cFSR.bPortrait = false;
    m_cFSR.cPaperSize.dPaperWidth = cFAttrs.dWidth;
    m_cFSR.cPaperSize.dPaperHeight = cFAttrs.dHeight;
  }

  PDPaperSize pSize = m_pFileSetupDlg->FindPaper(cFAttrs.dWidth, cFAttrs.dHeight);
  if(pSize) wcscpy(m_cFSR.cPaperSize.wsPaperSizeName, pSize->wsPaperSizeName);

  m_cFSR.dScaleNomin = cFAttrs.dScaleNom;
  m_cFSR.dScaleDenom = cFAttrs.dScaleDenom;

  if(fabs(m_cFSR.dScaleDenom) > g_dPrec)
    m_dDrawScale = m_cFSR.dScaleNomin/m_cFSR.dScaleDenom;
  else m_dDrawScale = 1.0;
}

LRESULT CMainWnd::FilePropsCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pFileSetupDlg->ShowDialog(hwnd, &m_cFSR))
  {
    if(fabs(m_cFSR.dScaleDenom) > g_dPrec)
      m_dDrawScale = m_cFSR.dScaleNomin/m_cFSR.dScaleDenom;
    else m_dDrawScale = 1.0;

    GetPageDims();

    CDFileAttrs cFAttrs;
    FilePropsToData(&cFAttrs);
    // set new file attributes
    m_pDrawObjects->SetFileAttrs(&cFAttrs, false);

    RECT rc;
    GetClientRect(hwnd, &rc);
    rc.top += m_iToolBarHeight;
    rc.bottom -= m_iStatusHeight;

    CDRect cdr;
    cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
    cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
    cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
    cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

    m_pDrawObjects->BuildAllPrimitives(&cdr);
    InvalidateRect(hwnd, &rc, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

/*LRESULT CMainWnd::FilePrintSetCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  PAGESETUPDLG cps;
  cps.lStructSize = sizeof(PAGESETUPDLG);
  cps.hwndOwner = hwnd;
  cps.hDevMode = NULL;
  cps.hDevNames = NULL;
  cps.Flags = 0;
  cps.hInstance = m_hInstance;
  PageSetupDlg(&cps);
  return 0;
}

LRESULT CMainWnd::FilePrintCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  return 0;
}*/

LRESULT CMainWnd::FileExitCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  SendMessage(hwnd, WM_CLOSE, 0, 0);
  return 0;
}

UINT MapDrawModeToMenu(int iDrawMode)
{
  switch(iDrawMode)
  {
  case modLine:
    return IDM_MODELINE;
  case modCircle:
    return IDM_MODECIRCLE;
  case modEllipse:
    return IDM_MODEELLIPSE;
  case modArcElps:
    return IDM_MODEARCELLIPSE;
  case modHyperbola:
    return IDM_MODEHYPERBOLA;
  case modParabola:
    return IDM_MODEPARABOLA;
  case modSpline:
    return IDM_MODESPLINE;
  case modEvolvent:
    return IDM_MODEEVEOLVENT;
  case modRectangle:
    return IDM_MODERECT;
  default:
    return IDM_MODESELECT;
  }
}

LRESULT CMainWnd::ModeCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl, int iMode)
{
  if((wNotifyCode == 1) && (iMode > 0) && (IsWindowVisible(m_hEdt1) || IsWindowVisible(m_hEdt2)))
  {
    HWND hFocus = GetFocus();
    char c = 0;
    switch(iMode)
    {
    case 1:
      c = 'l';
      break;
    case 2:
      c = 'c';
      break;
    case 3:
      c = 'e';
      break;
    case 4:
      c = 'a';
      break;
    case 5:
      c = 'h';
      break;
    case 6:
      c = 'p';
      break;
    case 7:
      c = 's';
      break;
    case 8:
      c = 'v';
      break;
    }
    if(hFocus == m_hEdt1)
    {
      SendMessage(m_hEdt1, WM_CHAR, c, 0);
    }
    else if(hFocus == m_hEdt2)
    {
      SendMessage(m_hEdt2, WM_CHAR, c, 0);
    }
    return 0;
  }

  if(GetCursor() == m_hPlusCur) SetCursor(LoadCursor(NULL, IDC_ARROW));

  ShowWindow(m_hEdt1, SW_HIDE);
  ShowWindow(m_hEdt2, SW_HIDE);
  ShowWindow(m_hLab1, SW_HIDE);
  ShowWindow(m_hChB1, SW_HIDE);

  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;
  HDC hdc;

  if(m_pActiveObject)
  {
    /*hdc = GetDC(hwnd);
    IntersectClipRect(hdc, rc.left, rc.top, rc.right, rc.bottom);

    HBRUSH hPrevBr = (HBRUSH)SelectObject(hdc, GetStockObject(HOLLOW_BRUSH));
    int iPrevROP = SetROP2(hdc, R2_NOTXORPEN);
    DrawObject(hwnd, hdc, m_pActiveObject, 1, -2);
    SetROP2(hdc, iPrevROP);
    SelectObject(hdc, hPrevBr);
    SelectClipRgn(hdc, NULL);
    ReleaseDC(hwnd, NULL);*/

    if(m_iToolMode == tolExtend) m_pActiveObject->CancelSplineEdit();
    else if(m_iToolMode != tolEditSpline) delete m_pActiveObject;
    m_pActiveObject = NULL;
  }
  else if(m_pSelForDimen)
  {
    m_pSelForDimen->DiscardDimen();
    m_pSelForDimen = NULL;
  }

  if(((m_iDrawMode + m_iToolMode > 0) && (iMode == 0)) ||
    ((m_iDrawMode + m_iToolMode == 0) && (iMode > 0)))
  {
    DrawCross(hwnd);
  }

  m_iToolMode = tolNone;

  HMENU hMenu = GetMenu(hwnd);

  UINT uCheck = MF_BYCOMMAND | MF_UNCHECKED;
  CheckMenuItem(hMenu, MapDrawModeToMenu(m_iDrawMode), uCheck);

  m_iDrawMode = iMode;
  m_iRegRasterCount = 0;

  StartNewObject(hwnd);
  if(!m_pActiveObject && (m_iDrawMode))
  {
    m_iDrawMode = modSelect;
    DrawCross(hwnd);
  }

  uCheck = MF_BYCOMMAND | MF_CHECKED;
  CheckMenuItem(hMenu, MapDrawModeToMenu(m_iDrawMode), uCheck);
  return 0;
}

LRESULT CMainWnd::EditCopyCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_iClipDataLen > 0)
  {
    GlobalFree(m_hClipData);
    m_hClipData = 0;
    m_iClipDataLen = 0;
  }

  if(!OpenClipboard(m_hWnd)) return FALSE;
  EmptyClipboard();

  int iLen = m_pDrawObjects->GetStreamSize(2);
  m_hClipData = GlobalAlloc(GMEM_DDESHARE, iLen*sizeof(unsigned char));
  if(m_hClipData == NULL)
  {
    CloseClipboard();
    return 0;
  }

  unsigned char *pDataCopy = (unsigned char*)GlobalLock(m_hClipData);
  m_pDrawObjects->SaveToStream(pDataCopy, 2);
  GlobalUnlock(m_hClipData);

  // Place the handle on the clipboard.
  SetClipboardData(m_iClipboardFormat, m_hClipData);
  CloseClipboard();
  return 0;
}

LRESULT CMainWnd::EditCutCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  EditCopyCmd(hwnd, wNotifyCode, hwndCtl);
  if(m_pDrawObjects->DeleteSelected(m_pUndoObjects, NULL))
  {
    InvalidateRect(m_hWnd, NULL, TRUE);
    SetTitle(m_hWnd, false);
  }
  return 0;
}

LRESULT CMainWnd::EditPasteCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(!IsClipboardFormatAvailable(m_iClipboardFormat)) return 0;
  if(!OpenClipboard(m_hWnd)) return 0;

  HGLOBAL hglb = GetClipboardData(m_iClipboardFormat);
  if(hglb != NULL)
  {
    unsigned char *plData = (unsigned char*)GlobalLock(hglb);
    if(plData != NULL)
    {
      if(m_pDrawObjects->ReadFromStream(plData, 2))
      {
        InvalidateRect(m_hWnd, NULL, TRUE);
        SetTitle(m_hWnd, false);
      }
      GlobalUnlock(hglb);
    }
  }
  CloseClipboard();
  return 0;
}

LRESULT CMainWnd::EditDeleteCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_iDrawMode == modSpline)
  {
    m_pActiveObject->RemoveLastPoint();
    return 0;
  }

  if(m_iToolMode == tolEditSpline)
  {
    if(m_pActiveObject->RemoveSplinePoint())
    {
      m_pDrawObjects->SetChanged();
      InvalidateRect(hwnd, NULL, FALSE);
      SetTitle(hwnd, false);
    }
    return 0;
  }

  if(m_iDrawMode + m_iToolMode > 0)
  {
    HWND hFocus = GetFocus();
    if(hFocus == m_hEdt1) SendMessage(m_hEdt1, WM_KEYDOWN, VK_DELETE, 0);
    if(hFocus == m_hEdt2) SendMessage(m_hEdt2, WM_KEYDOWN, VK_DELETE, 0);
    return 0;
  }

  m_pActiveObject = NULL;
  m_pHighObject = NULL;

  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  CDRect cdr;
  cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
  cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

  if(m_pDrawObjects->DeleteSelected(m_pUndoObjects, &cdr))
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }

  return 0;
}

LRESULT CMainWnd::EditDelLastPtCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  return 0;
}

void CMainWnd::DrawCross(HWND hWnd)
{
  RECT rc;
  GetClientRect(hWnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  HDC hdc = GetDC(hWnd);
  IntersectClipRect(hdc, rc.left, rc.top, rc.right, rc.bottom);

  Bitmap bmp(rc.right - rc.left, rc.bottom - rc.top);
  Graphics graphics(&bmp);
  graphics.SetSmoothingMode(SmoothingModeAntiAlias);
  graphics.DrawImage(m_pDrawBuffer, (INT)rc.left, (INT)rc.top);

  graphics.DrawLine(m_redPen, (REAL)(m_cLastSnapPt.x - 10), (REAL)m_cLastSnapPt.y,
  (REAL)(m_cLastSnapPt.x + 10), (REAL)m_cLastSnapPt.y);
  graphics.DrawLine(m_redPen, (REAL)m_cLastSnapPt.x, (REAL)(m_cLastSnapPt.y - 10),
  (REAL)m_cLastSnapPt.x, (REAL)(m_cLastSnapPt.y + 10));

  Graphics dstgraph(hdc);
  dstgraph.DrawImage(&bmp, 0, 0);

  SelectClipRgn(hdc, NULL);
  ReleaseDC(hWnd, NULL);
}

LRESULT CMainWnd::EditCopyParCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  int iSel = m_pDrawObjects->GetSelectCount(2);
  if(iSel != 1) return 0;

  PDObject pObj = m_pDrawObjects->GetSelected(0);
  if(!pObj) return 0;

  PDObject pNewObj = NULL;
  int iType = pObj->GetType();
  pNewObj = pObj->Copy();
  if(!pNewObj) return 0;

  m_pActiveObject = pNewObj;
  m_iToolMode = tolCopyPar;

  DrawCross(hwnd);

  LoadString(m_hInstance, IDS_DISTANCE, m_wsStatus2Base, 64);
  wcscpy(m_wsStatus2Msg, m_wsStatus2Base);
  SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);

  ShowWindow(m_hEdt1, SW_SHOW);
  SendMessage(m_hEdt1, EM_SETSEL, 0, (LPARAM)-1);
  SetFocus(m_hEdt1);
  return 0;
}

LRESULT CMainWnd::EditMoveCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  DrawCross(hwnd);
  LoadString(m_hInstance, IDS_DISTANCE, m_wsStatus2Base, 64);
  SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Base);
  ShowWindow(m_hEdt1, SW_SHOW);
  ShowWindow(m_hEdt2, SW_SHOW);

  wchar_t wBuf[64];
  LoadString(m_hInstance, IDS_NUMCOPIES, wBuf, 64);
  SendMessage(m_hLab1, WM_SETTEXT, 0, (LPARAM)wBuf);

  ShowWindow(m_hLab1, SW_SHOW);

  SendMessage(m_hEdt1, EM_SETSEL, 0, (LPARAM)-1);
  SetFocus(m_hEdt1);

  SendMessage(m_hEdt1, WM_GETTEXT, 64, (LPARAM)wBuf);
  char sBuf[64];
  WideCharToMultiByte(CP_UTF8, 0, wBuf, -1, sBuf, 64, NULL, NULL);
  m_iRestrictSet = ParseInputString(sBuf, m_pFileSetupDlg->GetUnitList(), &m_dRestrictValue);
  if(IS_LENGTH_VAL(m_iRestrictSet))
    LoadString(m_hInstance, IDS_SELLINETOMOVE, m_wsStatus2Msg, 128);
  else LoadString(m_hInstance, IDS_SELPOINTFROMMOVE, m_wsStatus2Msg, 128);
  SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus2Msg);
  m_iToolMode = tolMove;
  return 0;
}

LRESULT CMainWnd::EditRotateCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  DrawCross(hwnd);
  LoadString(m_hInstance, IDS_ANGLE, m_wsStatus2Base, 64);
  SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Base);
  ShowWindow(m_hEdt1, SW_SHOW);
  ShowWindow(m_hEdt2, SW_SHOW);

  wchar_t wBuf[64];
  LoadString(m_hInstance, IDS_NUMCOPIES, wBuf, 64);
  SendMessage(m_hLab1, WM_SETTEXT, 0, (LPARAM)wBuf);

  ShowWindow(m_hLab1, SW_SHOW);
  SendMessage(m_hEdt1, EM_SETSEL, 0, (LPARAM)-1);
  SetFocus(m_hEdt1);
  LoadString(m_hInstance, IDS_SELPOINTTOROTATE, m_wsStatus2Msg, 128);
  SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus2Msg);
  m_iToolMode = tolRotate;
  return 0;
}

LRESULT CMainWnd::EditMirrorCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  DrawCross(hwnd);
  LoadString(m_hInstance, IDS_SELLINETOMIRROR, m_wsStatus2Msg, 128);
  SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus2Msg);
  m_iToolMode = tolMirror;
  return 0;
}

LRESULT CMainWnd::EditDistributeCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  DrawCross(hwnd);
  LoadString(m_hInstance, IDS_NUMCOPIES, m_wsStatus2Base, 64);
  SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Base);
  ShowWindow(m_hEdt1, SW_SHOW);
  ShowWindow(m_hChB1, SW_SHOW);
  SendMessage(m_hEdt1, EM_SETSEL, 0, (LPARAM)-1);
  SetFocus(m_hEdt1);
  LoadString(m_hInstance, IDS_SELPATHTODISTRIBUTE, m_wsStatus2Msg, 128);
  SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus2Msg);
  m_iToolMode = tolDistribute;
  return 0;
}

LRESULT CMainWnd::EditLineStyleCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  CDLineStyleRec cLSRec;
  CDDimension cDimen;
  cDimen.psLab = NULL;
  int iMask = m_pDrawObjects->GetSelectedLineStyle(&cLSRec.cLineStyle);
  if(iMask > -1)
  {
    memcpy(&cLSRec.cUnit, &m_cFSR.cGraphUnit, sizeof(CDFileUnit));
    cLSRec.bWidthSet = (iMask & 1);
    cLSRec.bExcSet = (iMask & 2);
    cLSRec.bPatSet = (iMask & 4);
    cLSRec.bCapSet = (iMask & 8);
    cLSRec.bJoinSet = (iMask & 16);
    cLSRec.bColorSet = (iMask & 32);
    cLSRec.bFillColorSet = (iMask & 64);
    cLSRec.bBlurSet = (iMask & 128);
    cLSRec.bWidthChanged = false;
    cLSRec.bExcChanged = false;
    cLSRec.bPatChanged = false;
    cLSRec.bCapChanged = false;
    cLSRec.bJoinChanged = false;
    cLSRec.bColorChanged = false;
    cLSRec.bFillColorChanged = false;
    cLSRec.bBlurChanged = false;
    if(m_pLineStyleDlg->ShowDialog(hwnd, &cLSRec) == IDOK)
    {
      iMask = 0;
      if(cLSRec.bWidthSet && cLSRec.bWidthChanged) iMask |= 1;
      if(cLSRec.bExcSet && cLSRec.bExcChanged) iMask |= 2;
      if(cLSRec.bPatSet && cLSRec.bPatChanged) iMask |= 4;
      if(cLSRec.bCapSet && cLSRec.bCapChanged) iMask |= 8;
      if(cLSRec.bJoinSet && cLSRec.bJoinChanged) iMask |= 16;
      if(cLSRec.bColorSet && cLSRec.bColorChanged) iMask |= 32;
      if(cLSRec.bFillColorSet && cLSRec.bFillColorChanged) iMask |= 64;
      if(cLSRec.bBlurSet && cLSRec.bBlurChanged) iMask |= 128;
      if(m_pDrawObjects->SetSelectedLineStyle(iMask, &cLSRec.cLineStyle))
      {
        RECT rc;
        GetClientRect(hwnd, &rc);
        rc.top += m_iToolBarHeight;
        rc.bottom -= m_iStatusHeight;

        CDRect cdr;
        cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
        cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
        cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
        cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

        m_pDrawObjects->BuildAllPrimitives(&cdr);
        InvalidateRect(hwnd, NULL, FALSE);
        SetTitle(hwnd, false);
      }
    }
  }
  else if(m_pDrawObjects->GetSelectedDimen(&cDimen))
  {
    if(m_pDimEditDlg->ShowDialog(hwnd, &cDimen, m_pFileSetupDlg->GetUnitList(),
      &m_cFSR.cGraphUnit) == IDOK)
    {
      if(m_pDrawObjects->SetSelectedDimen(&cDimen))
      {
        RECT rc;
        GetClientRect(hwnd, &rc);
        rc.top += m_iToolBarHeight;
        rc.bottom -= m_iStatusHeight;

        CDRect cdr;
        cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
        cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
        cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
        cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

        m_pDrawObjects->BuildAllPrimitives(&cdr);
        InvalidateRect(hwnd, NULL, FALSE);
        SetTitle(hwnd, false);
      }
    }
    if(cDimen.psLab) free(cDimen.psLab);
  }
  return 0;
}

LRESULT CMainWnd::EditToggleSnapCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  bool bSnapEnabled = m_pDrawObjects->GetSelSnapEnabled();
  if(m_pSnapDlg->ShowDialog(hwnd, &bSnapEnabled))
  {
    m_pDrawObjects->SetSelSnapEnabled(bSnapEnabled);
  }
  return 0;
}

LRESULT CMainWnd::EditPaperUnitsCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  HMENU hMnu = GetMenu(hwnd);
  UINT uiCheck = MF_BYCOMMAND;
  if(m_bPaperUnits)
  {
    m_bPaperUnits = false;
    uiCheck |= MF_UNCHECKED;
  }
  else
  {
    m_bPaperUnits = true;
    uiCheck |= MF_CHECKED;
  }
  CheckMenuItem(hMnu, IDM_EDITPAPERUNITS, uiCheck);
  return 0;
}

LRESULT CMainWnd::EditUndoCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  int iCnt = m_pUndoObjects->GetCount();
  if(iCnt < 1) return 0;

  PDObject pObj = m_pUndoObjects->GetItem(iCnt - 1);
  m_pUndoObjects->ClearSelection();
  pObj->SetSelected(true, false, -1);

  if(m_pUndoObjects->DeleteSelected(m_pDrawObjects, NULL))
  {
    m_iRedoCount++;
    m_pDrawObjects->SetChanged();
    SetTitle(hwnd, false);

    InvalidateRect(hwnd, NULL, FALSE);
  }

  return 0;
}

LRESULT CMainWnd::EditRedoCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_iRedoCount < 1) return 0;
  int iCnt = m_pDrawObjects->GetCount();
  if(iCnt < 1) return 0;

  PDObject pObj = m_pDrawObjects->GetItem(iCnt - 1);
  m_pDrawObjects->ClearSelection();
  pObj->SetSelected(true, false, -1);

  if(m_pDrawObjects->DeleteSelected(m_pUndoObjects, NULL))
  {
    m_iRedoCount--;
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }

  return 0;
}

LRESULT CMainWnd::EditConfirmCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_iDrawMode == modSpline)
  {
    if(m_pActiveObject)
    {
      if(m_pActiveObject->HasEnoughPoints())
      {
        CDLine cTmpPt;
        cTmpPt.bIsSet = false;
        m_pActiveObject->BuildCache(cTmpPt, 0);
        if((m_iToolMode != tolEditSpline) && (m_iToolMode != tolExtend))
        {
          m_pDrawObjects->Add(m_pActiveObject);
          m_pActiveObject = NULL;
          StartNewObject(hwnd);
        }
        else
        {
          m_pActiveObject->SetAuxInt(0);
          m_pActiveObject = NULL;
          m_pDrawObjects->SetChanged();
          m_iDrawMode = modSelect;
          m_iToolMode = tolNone;
        }
        SetTitle(hwnd, false);
        InvalidateRect(hwnd, NULL, FALSE);
      }
      return 0;
    }
  }

  bool bConfirm = (m_iDrawMode == modLine) || (m_iDrawMode == modCircle) ||
    (m_iToolMode == tolRound) || (m_iToolMode == tolCopyPar) || (m_iDrawMode == modRectangle);
  if(bConfirm)
  {
    wchar_t wBuf[64];
    SendMessage(m_hEdt1, WM_GETTEXT, 64, (LPARAM)wBuf);
    char sBuf[64];
    WideCharToMultiByte(CP_UTF8, 0, wBuf, -1, sBuf, 64, NULL, NULL);
    m_iRestrictSet = ParseInputString(sBuf, m_pFileSetupDlg->GetUnitList(), &m_dRestrictValue);
    if(m_iDrawMode == modRectangle)
    {
      SendMessage(m_hEdt2, WM_GETTEXT, 64, (LPARAM)wBuf);
      WideCharToMultiByte(CP_UTF8, 0, wBuf, -1, sBuf, 64, NULL, NULL);
      m_iRestrictSet2 = ParseInputString(sBuf, m_pFileSetupDlg->GetUnitList(), &m_dRestrictValue2);
    }
  }
  return 0;
}

void CMainWnd::GetPageDims()
{
  m_dwPage = m_cFSR.cPaperSize.dPaperWidth;
  m_dhPage = m_cFSR.cPaperSize.dPaperHeight;
  if(m_cFSR.bPortrait)
  {
    if(m_dwPage > m_dhPage)
    {
      m_dwPage = m_cFSR.cPaperSize.dPaperHeight;
      m_dhPage = m_cFSR.cPaperSize.dPaperWidth;
    }
  }
  else if(m_dwPage < m_dhPage)
  {
    m_dwPage = m_cFSR.cPaperSize.dPaperHeight;
    m_dhPage = m_cFSR.cPaperSize.dPaperWidth;
  }
}

LRESULT CMainWnd::ViewFitCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  double dwWin = rc.right - rc.left - 20;
  double dhWin = rc.bottom - rc.top - 20;

  double drw = dwWin/m_dwPage;
  double drh = dhWin/m_dhPage;

  if(drw < drh)
  {
    m_dUnitScale = drw;
    m_cViewOrigin.x = 10;
    m_cViewOrigin.y = m_iToolBarHeight + (dhWin + 20 - m_dUnitScale*m_dhPage)/2;
  }
  else
  {
    m_dUnitScale = drh;
    m_cViewOrigin.x = (dwWin + 20 - m_dUnitScale*m_dwPage)/2;
    m_cViewOrigin.y = 10 + m_iToolBarHeight;
  }

  InvalidateRect(hwnd, &rc, FALSE);
  return 0;
}

LRESULT CMainWnd::ViewActSizeCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  CDPoint cOrigOff;
  int idx = (rc.right - rc.left)/2.0;
  int idy = (rc.bottom - rc.top)/2.0;
  cOrigOff.x = (idx - m_cViewOrigin.x)/m_dUnitScale;
  cOrigOff.y = (idy - m_cViewOrigin.y)/m_dUnitScale;

  m_dUnitScale = m_dDeviceToUnitScale;

  m_cViewOrigin.x = (int)(idx - cOrigOff.x*m_dUnitScale);
  m_cViewOrigin.y = (int)(idy - cOrigOff.y*m_dUnitScale);

  InvalidateRect(hwnd, &rc, FALSE);
  return 0;
}

LRESULT CMainWnd::ViewGridCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl, int iType)
{
  HMENU hMnu = GetMenu(hwnd);
  UINT uiCheck = MF_BYCOMMAND;
  BOOL bChecked = (m_iDrawGridMode & iType);
  if(bChecked)
  {
    m_iDrawGridMode &= ~iType;
    uiCheck |= MF_UNCHECKED;
  }
  else
  {
    m_iDrawGridMode |= iType;
    uiCheck |= MF_CHECKED;
  }
  CheckMenuItem(hMnu, IDM_VIEWGRIDPTS + iType - 1, uiCheck);
  InvalidateRect(hwnd, NULL, FALSE);
  return 0;
}

/*LRESULT CMainWnd::SnapCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl, int iSnap)
{
  int iSnapBit = (1 << iSnap);
  if(m_iSnapType & iSnapBit) m_iSnapType &= ~iSnapBit;
  else m_iSnapType |= iSnapBit;
  UpdateSnapMenu(GetMenu(hwnd));
  return 0;
}*/

LRESULT CMainWnd::ToolsCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl, int iTool)
{
  if((wNotifyCode == 1) && (iTool == tolDimen) && (IsWindowVisible(m_hEdt1) || IsWindowVisible(m_hEdt2)))
  {
    HWND hFocus = GetFocus();
    char c = 'd';
    if(hFocus == m_hEdt1)
    {
      SendMessage(m_hEdt1, WM_CHAR, c, 0);
    }
    else if(hFocus == m_hEdt2)
    {
      SendMessage(m_hEdt2, WM_CHAR, c, 0);
    }
    return 0;
  }

  if(GetCursor() == m_hPlusCur) SetCursor(LoadCursor(NULL, IDC_ARROW));

  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;
  HDC hdc;

  if(m_pActiveObject)
  {
    /*hdc = GetDC(hwnd);
    IntersectClipRect(hdc, rc.left, rc.top, rc.right, rc.bottom);

    HBRUSH hPrevBr = (HBRUSH)SelectObject(hdc, GetStockObject(HOLLOW_BRUSH));
    int iPrevROP = SetROP2(hdc, R2_NOTXORPEN);
    DrawObject(hwnd, hdc, m_pActiveObject, 1, -2);
    SetROP2(hdc, iPrevROP);
    SelectObject(hdc, hPrevBr);
    SelectClipRgn(hdc, NULL);
    ReleaseDC(hwnd, NULL);*/

    if(m_iToolMode == tolExtend) m_pActiveObject->CancelSplineEdit();
    else if(m_iToolMode != tolEditSpline) delete m_pActiveObject;
    m_pActiveObject = NULL;
  }
  else if(m_pSelForDimen)
  {
    m_pSelForDimen->DiscardDimen();
    m_pSelForDimen = NULL;
  }

  if(iTool == tolDimen)
  {
    if(m_pDrawObjects->GetSelectCount(2) != 1)
    {
      wchar_t sCap[64];
      wchar_t sMsg[128];
      LoadString(m_hInstance, IDS_WARNING, sCap, 64);
      LoadString(m_hInstance, IDS_ONEOBJFORDIMEN, sMsg, 128);
      MessageBox(hwnd, sMsg, sCap, MB_OK | MB_ICONWARNING);
      return 0;
    }
    m_pSelForDimen = m_pDrawObjects->GetSelected(0);
  }

  if(iTool == tolEditSpline)
  {
    int iCnt = m_pDrawObjects->GetSelectCount(2);
    if(iCnt == 1) m_pActiveObject = m_pDrawObjects->GetSelected(0);
    if(!m_pActiveObject || (m_pActiveObject->GetType() != dtSpline))
    {
      m_pActiveObject = NULL;
      wchar_t sCap[64];
      wchar_t sMsg[128];
      LoadString(m_hInstance, IDS_WARNING, sCap, 64);
      LoadString(m_hInstance, IDS_ONESPLINETOEDIT, sMsg, 128);
      MessageBox(hwnd, sMsg, sCap, MB_OK | MB_ICONWARNING);
      return 0;
    }
  }

  bool bDrawCross = ((m_iDrawMode + m_iToolMode > 0) && (iTool == 0)) ||
    ((m_iDrawMode + m_iToolMode == 0) && (iTool > 0));
  if(bDrawCross)
  {
    DrawCross(hwnd);
  }

  m_iToolMode = iTool;
  m_iDrawMode = modSelect;

  if(m_pActiveObject && (m_iToolMode == tolEditSpline))
  {
    //m_iDrawMode = modSpline;
    return 0;
  }

  StartNewObject(hwnd);
  if(!m_pActiveObject && (m_iToolMode == tolRound))
  {
    m_iToolMode = tolNone;
    bDrawCross = false;
  }

  return 0;
}

LRESULT CMainWnd::HelpContentCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  MessageBox(hwnd, L"Help is not implemented yet", L"Information", MB_OK);
  return 0;
}

/*void CMainWnd::UpdateSnapMenu(HMENU hMenu)
{
  int iSnapBit = 1;
  int iSnapCount = IDM_SNAPINTERSECT - IDM_SNAPELEMENT + 1;
  UINT uCheck;
  for(int i = 0; i < iSnapCount; i++)
  {
    uCheck = MF_BYCOMMAND;
    if(m_iSnapType & iSnapBit) uCheck |= MF_CHECKED;
    else uCheck |= MF_UNCHECKED;
    CheckMenuItem(hMenu, IDM_SNAPELEMENT + i, uCheck);
    iSnapBit <<= 1;
  }
}*/

LRESULT CMainWnd::WMMButtonDown(HWND hwnd, WPARAM fwKeys, int xPos, int yPos)
{
  if(m_iButton > 0) return 0;

  m_iButton = 2;
  m_cZoomOrig.x = xPos - m_cViewOrigin.x;
  m_cZoomOrig.y = yPos - m_cViewOrigin.y;
  m_cLastMovePt.x = xPos;
  m_cLastMovePt.y = yPos;
  SetCapture(hwnd);
  return 0;
}

LRESULT CMainWnd::WMMButtonUp(HWND hwnd, WPARAM fwKeys, int xPos, int yPos)
{
  RECT rc;
  if(m_iButton == 2)
  {
    ReleaseCapture();

    //m_cViewOrigin.x = (xPos - m_cZoomOrig.x);
    //m_cViewOrigin.y = (yPos - m_cZoomOrig.y);

    GetClientRect(hwnd, &rc);
    rc.top += m_iToolBarHeight;
    rc.bottom -= m_iStatusHeight;
    InvalidateRect(hwnd, &rc, FALSE);
  }
  m_iButton = 0;
  return 0;
}

LRESULT CMainWnd::WMLButtonDown(HWND hwnd, WPARAM fwKeys, int xPos, int yPos)
{
  if(m_iButton > 0) return 0;

  if(m_iToolMode == tolEditSpline)
  {
    double dx = (xPos - m_cViewOrigin.x)/m_dUnitScale;
    double dy = (yPos - m_cViewOrigin.y)/m_dUnitScale;
    double dTol = (double)m_iSnapTolerance/m_dUnitScale;
    m_pActiveObject->SelSplinePoint(dx, dy, dTol);
  }

  m_cLastDownPt.x = xPos;
  m_cLastDownPt.y = yPos;
  m_cLastMovePt.x = xPos;
  m_cLastMovePt.y = yPos;
  m_iButton = 1;

  return 0;
}

LRESULT CMainWnd::WMLButtonUp(HWND hwnd, WPARAM fwKeys, int xPos, int yPos)
{
  if(m_iButton != 1) return 0;

  m_iButton = 0;

  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  CDRect cdr;
  cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
  cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
  cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

  double dTol = (double)m_iSnapTolerance/m_dUnitScale;
  CDPoint cDistPt, cPt1, cPt2;
  wchar_t wsBuf[128];
  double dNorm, d1, d2, dx1, dy1, dAng;
  wchar_t *wsUnit;
  int i;

  if(m_iDrawMode + m_iToolMode < 1)
  {
    if(!(fwKeys & MK_CONTROL)) m_pDrawObjects->ClearSelection();

    if(GetPtDist(&m_cLastDownPt, xPos, yPos) > 4) // select by rectangle
    {
      CDRect cdr1;
      cdr1.cPt1.x = (m_cLastDownPt.x - m_cViewOrigin.x)/m_dUnitScale;
      cdr1.cPt1.y = (m_cLastDownPt.y - m_cViewOrigin.y)/m_dUnitScale;
      cdr1.cPt2.x = (xPos - m_cViewOrigin.x)/m_dUnitScale;
      cdr1.cPt2.y = (yPos - m_cViewOrigin.y)/m_dUnitScale;
      m_pDrawObjects->SelectByRectangle(&cdr1, 2);
    }
    else
    {
      if(m_pHighObject)
        m_pHighObject->SetSelected(true, fwKeys & MK_CONTROL, m_iHighDimen);
    }
    InvalidateRect(hwnd, &rc, FALSE);
  }
  else if((m_iToolMode > 20) && (m_iToolMode != tolRound) && (m_iDrawMode == modSelect))
  {
    if(GetPtDist(&m_cLastDownPt, xPos, yPos) > 4) return 0;

    int iExtend = 0;

    switch(m_iToolMode)
    {
    case tolKnife:
      if(m_pDrawObjects->CutSelected(m_cLastDrawPt, dTol, &cdr))
      {
        InvalidateRect(hwnd, &rc, FALSE);
        SetTitle(hwnd, false);
      }
      break;
    case tolExtend:
      iExtend = m_pDrawObjects->ExtendSelected(m_cLastDrawPt, dTol, &cdr);
      if(iExtend > 0)
      {
        if(iExtend > 1)
        {
          m_iDrawMode = modSpline;
          m_pActiveObject = m_pDrawObjects->GetSelected(0);
        }
        InvalidateRect(hwnd, &rc, FALSE);
        SetTitle(hwnd, false);
      }
      break;
    case tolConflict:
      if(m_pDrawObjects->SetCrossSelected(m_cLastDrawPt, dTol, &cdr))
      {
        InvalidateRect(hwnd, &rc, FALSE);
        SetTitle(hwnd, false);
      }
      break;
    case tolMeas:
      if(!m_cMeasPoint1.bIsSet)
      {
        m_cMeasPoint1.bIsSet = true;
        m_cMeasPoint1.cOrigin = m_cLastDrawPt;
      }
      else if(!m_cMeasPoint2.bIsSet)
      {
        m_cMeasPoint2.bIsSet = true;
        m_cMeasPoint2.cOrigin = m_cLastDrawPt;
        cDistPt = m_cMeasPoint2.cOrigin - m_cMeasPoint1.cOrigin;
        if(m_bPaperUnits)
        {
          cDistPt /= m_cFSR.cPaperUnit.dBaseToUnit;
          wsUnit = m_cFSR.cPaperUnit.wsAbbrev;
        }
        else
        {
          cDistPt /= m_dDrawScale;
          cDistPt /= m_cFSR.cLenUnit.dBaseToUnit;
          wsUnit = m_cFSR.cLenUnit.wsAbbrev;
        }
        dNorm = GetNorm(cDistPt);
        swprintf(wsBuf, L"dx: %.3f, dy: %.3f, dist: %.4f (%s)", fabs(cDistPt.x),
          fabs(cDistPt.y), dNorm, wsUnit);
        SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)wsBuf);
      }
      else
      {
        m_cMeasPoint1.bIsSet = true;
        m_cMeasPoint1.cOrigin = m_cLastDrawPt;
        m_cMeasPoint2.bIsSet = false;
        SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)L"");
      }
      break;
    case tolMeasAngle:
      if(!m_cMeasPoint1.bIsSet)
      {
        m_cMeasPoint1.bIsSet = true;
        m_cMeasPoint1.cOrigin = m_cLastDrawPt;
      }
      else if(!m_cMeasPoint2.bIsSet)
      {
        m_cMeasPoint2.bIsSet = true;
        m_cMeasPoint2.cOrigin = m_cLastDrawPt;
      }
      else if(!m_cMeasPoint3.bIsSet)
      {
        m_cMeasPoint3.bIsSet = true;
        m_cMeasPoint3.cOrigin = m_cLastDrawPt;
        cPt1 = m_cMeasPoint2.cOrigin - m_cMeasPoint1.cOrigin;
        cPt2 = m_cMeasPoint3.cOrigin - m_cMeasPoint1.cOrigin;
        d1 = GetNorm(cPt1);
        d2 = GetNorm(cPt2);
        if((d1 > g_dPrec) && (d2 > g_dPrec))
        {
          dx1 = (cPt1.x*cPt2.x + cPt1.y*cPt2.y)/d1/d2;
          dy1 = (cPt2.y*cPt1.x - cPt1.y*cPt2.x)/d1/d2;
          dAng = -180.0*atan2(dy1, dx1)/M_PI;
          wsUnit = m_cFSR.cAngUnit.wsAbbrev;
          swprintf(wsBuf, L"angle: %.4f (%s)", dAng*m_cFSR.cAngUnit.dBaseToUnit, wsUnit);
          SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)wsBuf);
        }
      }
      else
      {
        m_cMeasPoint1.bIsSet = true;
        m_cMeasPoint1.cOrigin = m_cLastDrawPt;
        m_cMeasPoint2.bIsSet = false;
        m_cMeasPoint3.bIsSet = false;
        SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)L"");
      }
      break;
    case tolMeasLength:
      i = m_pDrawObjects->GetSelectedLength(&dNorm);
      if(i > 1)
      {
        wcscpy(wsBuf, L"No object selected");
      }
      else if(i > 0)
      {
        wcscpy(wsBuf, L"Infinity");
      }
      else
      {
        if(m_bPaperUnits)
        {
          dNorm /= m_cFSR.cPaperUnit.dBaseToUnit;
          wsUnit = m_cFSR.cPaperUnit.wsAbbrev;
        }
        else
        {
          dNorm /= m_dDrawScale;
          dNorm /= m_cFSR.cLenUnit.dBaseToUnit;
          wsUnit = m_cFSR.cLenUnit.wsAbbrev;
        }
        swprintf(wsBuf, L"Length: %.4f (%s)", dNorm, wsUnit);
      }
      SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)wsBuf);
      break;
    }
  }
  else
  {
    if(GetPtDist(&m_cLastDownPt, xPos, yPos) > 4) return 0;

    //RECT rc;
    //GetClientRect(hwnd, &rc);
    //rc.top += m_iToolBarHeight;
    //rc.bottom -= m_iStatusHeight;

    wchar_t buf[64];
    float f;
    int i;
    bool bdValSet = false;
    double dVal = 0;
    int iCop = 0;
    bool bKeepOrient = true;
    PDObject pSelLine = NULL;
    CDLine cLine;

    if((m_iToolMode > tolCopyPar) && (m_iToolMode < tolMirror))
    {
      SendMessage(m_hEdt1, WM_GETTEXT, 64, (LPARAM)buf);
      if(swscanf(buf, L"%f", &f) == 1)
      {
        dVal = f;
        bdValSet = true;
      }
      SendMessage(m_hEdt2, WM_GETTEXT, 64, (LPARAM)buf);
      if(swscanf(buf, L"%d", &i) == 1) iCop = i;
    }
    else if(m_iToolMode == tolDistribute)
    {
      SendMessage(m_hEdt1, WM_GETTEXT, 64, (LPARAM)buf);
      if(swscanf(buf, L"%d", &i) == 1) iCop = i;
      i = SendMessage(m_hChB1, BM_GETCHECK, 0, 0);
      if(i != BST_CHECKED) bKeepOrient = false;
    }

    if(m_iToolMode == tolMove)
    {
      if(m_cMeasPoint1.bIsSet)
      {
        cLine.bIsSet = true;
        cLine.cOrigin = m_cMeasPoint1.cOrigin;
        cDistPt = m_cLastDrawPt - m_cMeasPoint1.cOrigin;
        dNorm = GetNorm(cDistPt);
        if(dNorm > g_dPrec)
        {
          cLine.cDirection = cDistPt/dNorm;
          if(m_pDrawObjects->MoveSelected(cLine, dNorm, iCop, &cdr, true))
          {
            InvalidateRect(hwnd, &rc, FALSE);
            SetTitle(hwnd, false);
            StartNewObject(hwnd);
          }
        }
        m_iToolMode = tolNone;
        m_cMeasPoint1.bIsSet = false;
      }
      else
      {
        if(bdValSet)
        {
          pSelLine = m_pDrawObjects->SelectLineByPoint(m_cLastDrawPt, dTol);
          if(pSelLine)
          {
            cLine = pSelLine->GetLine();
            m_iToolMode = tolNone;
            if(m_bPaperUnits)
              dVal *= m_cFSR.cPaperUnit.dBaseToUnit;
            else dVal *= m_dDrawScale*m_cFSR.cLenUnit.dBaseToUnit;
            if(m_pDrawObjects->MoveSelected(cLine, dVal, iCop, &cdr, false))
            {
              InvalidateRect(hwnd, &rc, FALSE);
              SetTitle(hwnd, false);
              StartNewObject(hwnd);
            }
          }
        }
        else
        {
          m_cMeasPoint1.cOrigin = m_cLastDrawPt;
          m_cMeasPoint1.bIsSet = true;
          LoadString(m_hInstance, IDS_SELPOINTTOMOVE, m_wsStatus2Msg, 128);
          SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus2Msg);
        }
      }
    }
    else if(m_iToolMode == tolRotate)
    {
      if(bdValSet)
      {
        //if(m_cFSR.iAngUnit < 1) dVal *= M_PI/180.0;
        dVal *= M_PI/180.0/m_cFSR.cAngUnit.dBaseToUnit;

        m_iToolMode = tolNone;
        if(m_pDrawObjects->RotateSelected(m_cLastDrawPt, -dVal, iCop, &cdr))
        {
          InvalidateRect(hwnd, &rc, FALSE);
          SetTitle(hwnd, false);
          StartNewObject(hwnd);
        }
      }
      else if(m_cMeasPoint2.bIsSet)
      {
        CDPoint cDir = m_cMeasPoint2.cOrigin - m_cMeasPoint1.cOrigin;
        double dNorm = GetNorm(cDir);
        if(dNorm > g_dPrec)
        {
          cDir /= dNorm;
          CDPoint cPt1 = Rotate(m_cLastDrawPt - m_cMeasPoint1.cOrigin, cDir, false);
          dVal = atan2(cPt1.y, cPt1.x);
          m_cMeasPoint1.bIsSet = false;
          m_cMeasPoint2.bIsSet = false;
          m_iToolMode = tolNone;
          if(m_pDrawObjects->RotateSelected(m_cMeasPoint1.cOrigin, dVal, iCop, &cdr))
          {
            InvalidateRect(hwnd, &rc, FALSE);
            SetTitle(hwnd, false);
            StartNewObject(hwnd);
          }
        }
      }
      else if(m_cMeasPoint1.bIsSet)
      {
        m_cMeasPoint2.bIsSet = true;
        m_cMeasPoint2.cOrigin = m_cLastDrawPt;
        LoadString(m_hInstance, IDS_SELPOINTTOROT, m_wsStatus2Msg, 128);
        SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus2Msg);
      }
      else
      {
        m_cMeasPoint1.bIsSet = true;
        m_cMeasPoint1.cOrigin = m_cLastDrawPt;
        LoadString(m_hInstance, IDS_SELPOINTFROMROT, m_wsStatus2Msg, 128);
        SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus2Msg);
      }
    }
    else if(m_iToolMode == tolMirror)
    {
      pSelLine = m_pDrawObjects->SelectLineByPoint(m_cLastDrawPt, dTol);
      if(pSelLine)
      {
        cLine = pSelLine->GetLine();
        m_iToolMode = tolNone;
        if(m_pDrawObjects->MirrorSelected(cLine, &cdr))
        {
          InvalidateRect(hwnd, &rc, FALSE);
          SetTitle(hwnd, false);
          StartNewObject(hwnd);
        }
      }
    }
    else if(m_iToolMode == tolDistribute)
    {
      int iRes = m_pDrawObjects->Distribute(iCop, bKeepOrient, m_cLastDrawPt);
      if(iRes > 0)
      {
        wchar_t sCap[64];
        wchar_t sMsg[128];
        LoadString(m_hInstance, IDS_WARNING, sCap, 64);
        switch(iRes)
        {
        case 1:
          LoadString(m_hInstance, IDS_ONEOBJECTFORDISTR, sMsg, 128);
          break;
        case 2:
          LoadString(m_hInstance, IDS_BOUNDOBJFORDIST, sMsg, 128);
          break;
        case 3:
          LoadString(m_hInstance, IDS_BOUNDPATHTODISTR, sMsg, 128);
          break;
        case 4:
          LoadString(m_hInstance, IDS_NOCLOSEDPATHFORRUBBER, sMsg, 128);
          break;
        }
        MessageBox(hwnd, sMsg, sCap, MB_OK | MB_ICONWARNING);
      }
      else
      {
        m_iToolMode = tolNone;
        InvalidateRect(hwnd, &rc, FALSE);
        SetTitle(hwnd, false);
        StartNewObject(hwnd);
      }
    }
    else if(m_iToolMode == tolDimen)
    {
      if(m_pDrawObjects->AddDimen(m_pSelForDimen, m_cLastDrawPt, dTol, &cdr))
      {
        InvalidateRect(hwnd, &rc, FALSE);
        SetTitle(hwnd, false);
      }
    }
    else if(m_iDrawMode == modRegRaster)
    {
      if(m_iRegRasterCount < 6)
      {
        m_cRegRasterPoints[m_iRegRasterCount++] = m_cLastDrawPt;
      }
      if(m_iRegRasterCount == 6)
      {
        PDObject pImage = m_pDrawObjects->GetSelected(0);
        pImage->RegisterRaster(m_cRegRasterPoints);
        m_iRegRasterCount = 0;
        m_iDrawMode = modSelect;
        m_pDrawObjects->SetChanged();
        InvalidateRect(hwnd, &rc, FALSE);
        SetTitle(hwnd, false);
        SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)L"");
      }
      else
      {
        wchar_t sMessage[128];
        if(m_iRegRasterCount < 2) LoadString(m_hInstance, IDS_REGFIRSTLINE, sMessage, 64);
        else if(m_iRegRasterCount < 4) LoadString(m_hInstance, IDS_REGSECONDLINE, sMessage, 64);
        else LoadString(m_hInstance, IDS_REGTHIRDLINE, sMessage, 64);
        SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)sMessage);
      }
    }
    else if(m_iToolMode == tolEditSpline)
    {
      if(m_pActiveObject->InsertSplinePoint(m_cLastDrawPt.x, m_cLastDrawPt.y, dTol))
      {
        SetTitle(hwnd, false);
        InvalidateRect(hwnd, NULL, FALSE);
      }
    }
    else
    {
      int iCtrl = 0;
      double dOffset;
      if(m_bPaperUnits) dOffset = m_dRestrictValue*m_cFSR.cPaperUnit.dBaseToUnit;
      else dOffset = m_dRestrictValue*m_dDrawScale*m_cFSR.cLenUnit.dBaseToUnit;
      if(m_iToolMode == tolCopyPar)
      {
        iCtrl = 2;
        if(fwKeys & MK_SHIFT) iCtrl = 3;
        if(IS_LENGTH_VAL(m_iRestrictSet))
        {
          iCtrl = 4;
          PDObject pSelObj = m_pDrawObjects->GetSelected(0);
          dOffset += pSelObj->GetOffset();
        }
      }
      if(m_cLastDynPt.bIsSet)
      {
        CDInputPoint cInPt;
        m_pActiveObject->GetPoint(0, 0, &cInPt);
        cInPt.cPoint = m_cLastDynPt.cOrigin;
        m_pActiveObject->SetPoint(0, 0, cInPt);
      }
      if(m_pActiveObject->AddPoint(m_cLastDrawPt.x, m_cLastDrawPt.y, iCtrl, dOffset))
      {
        m_pDrawObjects->Add(m_pActiveObject);
        SetTitle(hwnd, false);
        m_pActiveObject = NULL;
        m_iToolMode = tolNone;
        m_wsStatus2Base[0] = 0;
        wcscpy(m_wsStatus2Msg, m_wsStatus2Base);
        SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);
        m_wsStatus3Base[0] = 0;
        wcscpy(m_wsStatus3Msg, m_wsStatus3Base);
        //SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus3Msg);
        SendMessage(m_hLab1, WM_SETTEXT, 0, (LPARAM)m_wsStatus3Msg);

        InvalidateRect(hwnd, &rc, FALSE);
        StartNewObject(hwnd);
      }
    }
  }

  m_cLastDynPt.bIsSet = false;
  return 0;
}

LRESULT CMainWnd::WMRButtonDown(HWND hwnd, WPARAM fwKeys, int xPos, int yPos)
{
  if(m_iButton > 0) return 0;

  m_cLastDownPt.x = xPos;
  m_cLastDownPt.y = yPos;
  m_cLastMovePt.x = xPos;
  m_cLastMovePt.y = yPos;
  m_iButton = 3;

  return 0;
}

LRESULT CMainWnd::WMRButtonUp(HWND hwnd, WPARAM fwKeys, int xPos, int yPos)
{
  if(m_iButton != 3) return 0;

  m_iButton = 0;

  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  if(m_iDrawMode + m_iToolMode < 1) // selection
  {
    if(GetPtDist(&m_cLastDownPt, xPos, yPos) > 4) // select by rectangle
    {
      if(!(fwKeys & MK_CONTROL)) m_pDrawObjects->ClearSelection();
      //MessageBox(0, L"Dobry", L"Debug", MB_OK);
      CDRect cdr1;
      cdr1.cPt1.x = (m_cLastDownPt.x - m_cViewOrigin.x)/m_dUnitScale;
      cdr1.cPt1.y = (m_cLastDownPt.y - m_cViewOrigin.y)/m_dUnitScale;
      cdr1.cPt2.x = (xPos - m_cViewOrigin.x)/m_dUnitScale;
      cdr1.cPt2.y = (yPos - m_cViewOrigin.y)/m_dUnitScale;
      m_pDrawObjects->SelectByRectangle(&cdr1, 1); //, pRegions);

      InvalidateRect(hwnd, &rc, FALSE);
    }
    else
    {
      if(m_pHighObject)
      {
        HMENU hMenu = LoadMenu(m_hInstance, L"POPUPMENU");
        if(m_pHighObject->GetSnapTo()) hMenu = GetSubMenu(hMenu, 1);
        else hMenu = GetSubMenu(hMenu, 0);
        POINT pt = {xPos, yPos};
        ClientToScreen(hwnd, &pt);
        TrackPopupMenu(hMenu, TPM_RIGHTALIGN | TPM_BOTTOMALIGN | TPM_RIGHTBUTTON,
          pt.x, pt.y, 0, hwnd, NULL);
      }
    }
  }
  else if(m_iToolMode > tolNone)
  {
  }
  else
  {
    if(GetPtDist(&m_cLastDownPt, xPos, yPos) > 4) return 0;

    if(m_cLastDynPt.bIsSet)
    {
      CDInputPoint cInPt;
      m_pActiveObject->GetPoint(0, 0, &cInPt);
      cInPt.cPoint = m_cLastDynPt.cOrigin;
      m_pActiveObject->SetPoint(0, 0, cInPt);
    }
    if(m_pActiveObject->AddPoint(m_cLastDrawPt.x, m_cLastDrawPt.y, 1, 0.0))
    {
      m_pDrawObjects->Add(m_pActiveObject);
      SetTitle(hwnd, false);
      m_pActiveObject = NULL;
      m_iToolMode = tolNone;
      m_wsStatus2Base[0] = 0;
      wcscpy(m_wsStatus2Msg, m_wsStatus2Base);
      SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);
      m_wsStatus3Base[0] = 0;
      wcscpy(m_wsStatus3Msg, m_wsStatus3Base);
      //SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus3Msg);
      SendMessage(m_hLab1, WM_SETTEXT, 0, (LPARAM)m_wsStatus3Msg);

      InvalidateRect(hwnd, &rc, FALSE);
      StartNewObject(hwnd);
    }
  }

  m_cLastDynPt.bIsSet = false;
  return 0;
}

bool PointsEqual(POINT cPt1, POINT cPt2)
{
  return (cPt1.x == cPt2.x) && (cPt1.y == cPt2.y);
}

ARGB EncodeColor(DWORD dwColor)
{
  unsigned char red = dwColor & 0xFF;
  unsigned char green = (dwColor >> 8) & 0xFF;
  unsigned char blue = (dwColor >> 16) & 0xFF;
  unsigned char alpha = (dwColor >> 24) & 0xFF;
  return blue | (green << 8) | (red << 16) | (alpha << 24);
}

void CMainWnd::DrawDimArrow(Graphics *graphics, Pen *pen, GraphicsPath *path, PDPrimitive pPrim)
{
  pen->SetLineCap(LineCapRound, LineCapRound, DashCapRound);
  pen->SetLineJoin(LineJoinRound);

  Color clr;
  pen->GetColor(&clr);
  SolidBrush hBr(clr);

  CDPoint cStartPt, cEndPt, cPoly[3];
  int iType = Round(pPrim->cPt1.x);

  switch(iType)
  {
  case 1:
    cStartPt.x = pPrim->cPt3.x + m_cViewOrigin.x;
    cStartPt.y = pPrim->cPt3.y + m_cViewOrigin.y;
    cEndPt.x = pPrim->cPt2.x + m_cViewOrigin.x;
    cEndPt.y = pPrim->cPt2.y + m_cViewOrigin.y;
    path->AddLine((REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)cEndPt.x, (REAL)cEndPt.y);
    cStartPt = cEndPt;
    cEndPt.x = pPrim->cPt4.x + m_cViewOrigin.x;
    cEndPt.y = pPrim->cPt4.y + m_cViewOrigin.y;
    path->AddLine((REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)cEndPt.x, (REAL)cEndPt.y);
    graphics->DrawPath(pen, path);
    path->Reset();
    break;
  case 2:
    cStartPt.x = pPrim->cPt3.x + m_cViewOrigin.x;
    cStartPt.y = pPrim->cPt3.y + m_cViewOrigin.y;
    cEndPt.x = pPrim->cPt2.x + m_cViewOrigin.x;
    cEndPt.y = pPrim->cPt2.y + m_cViewOrigin.y;
    path->AddLine((REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)cEndPt.x, (REAL)cEndPt.y);
    cStartPt = cEndPt;
    cEndPt.x = pPrim->cPt4.x + m_cViewOrigin.x;
    cEndPt.y = pPrim->cPt4.y + m_cViewOrigin.y;
    path->AddLine((REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)cEndPt.x, (REAL)cEndPt.y);
    path->CloseFigure();
    graphics->FillPath(&hBr, path);
    path->Reset();
    break;
  case 3:
    cStartPt.y = Round(pPrim->cPt3.y - pPrim->cPt2.y);
    path->AddEllipse((REAL)(pPrim->cPt2.x + m_cViewOrigin.x - cStartPt.y),
      (REAL)(pPrim->cPt2.y + m_cViewOrigin.y - cStartPt.y),
      (REAL)(2.0*cStartPt.y),
      (REAL)(2.0*cStartPt.y));
    graphics->FillPath(&hBr, path);
    path->Reset();
    break;
  case 4:
  case 5:
    cStartPt.x = pPrim->cPt3.x + m_cViewOrigin.x;
    cStartPt.y = pPrim->cPt3.y + m_cViewOrigin.y;
    cEndPt.x = pPrim->cPt4.x + m_cViewOrigin.x;
    cEndPt.y = pPrim->cPt4.y + m_cViewOrigin.y;
    graphics->DrawLine(pen, (REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)cEndPt.x, (REAL)cEndPt.y);
    break;
  }
}

void CMainWnd::DrawDimText(HWND hWnd, Graphics *graphics, PDPrimitive pPrim, PDObject pObj,
  DWORD dwColor, double dLineWidth)
{
  int iPos = Round(pPrim->cPt2.y);

  char sBuf[64];
  char *psBuf = sBuf;
  int iLen = pObj->PreParseDimText(iPos, psBuf, 64, m_dDrawScale, m_pFileSetupDlg->GetUnitList());
  if(iLen > 0)
  {
    psBuf = (char*)malloc(iLen*sizeof(char));
    pObj->PreParseDimText(iPos, psBuf, iLen, m_dDrawScale, m_pFileSetupDlg->GetUnitList());
  }

  int iLen2 = strlen(psBuf);

  CDFileAttrs cFileAttrs;
  if(iPos < 0) m_pDrawObjects->GetFileAttrs(&cFileAttrs);
  else pObj->GetDimFontAttrs(iPos, &cFileAttrs);

  double da = 0.4*m_dUnitScale*cFileAttrs.dFontSize;

  bool bDiam = (psBuf[0] == '*');
  int iStart = 0;
  if(bDiam) iStart = 1;
  if(iStart >= iLen2)
  {
    psBuf[iStart] = '?';
    psBuf[iStart + 1] = 0;
    iLen2++;
  }

  int iwLen = MultiByteToWideChar(CP_UTF8, 0, &psBuf[iStart], -1, NULL, 0) + 1;
  LPWSTR wsBufStart = (LPWSTR)malloc(iwLen*sizeof(wchar_t));
  MultiByteToWideChar(CP_UTF8, 0, &psBuf[iStart], -1, wsBufStart, iwLen);
  int iLenStart = wcslen(wsBufStart);
  int iLenNom = 0;
  int iLenDenom = 0;
  int iLenEnd = 0;

  LPWSTR wsFrac = wcschr(wsBufStart, '_');
  wchar_t wsBufNom[4];
  wchar_t wsBufDenom[4];
  LPWSTR wsBufEnd = NULL;

  if(wsFrac)
  {
    wsBufEnd = wsFrac + 1;
    int i = 0;
    while((*wsBufEnd >= '0') && (*wsBufEnd <= '9') && (i < 3))
    {
      wsBufNom[i++] = *(wsBufEnd++);
    }
    wsBufNom[i] = 0;

    while(*wsBufEnd && (*wsBufEnd != '/')) wsBufEnd++;
    if(*wsBufEnd) wsBufEnd++;
    i = 0;
    while((*wsBufEnd >= '0') && (*wsBufEnd <= '9') && (i < 3))
    {
      wsBufDenom[i++] = *(wsBufEnd++);
    }
    wsBufDenom[i] = 0;
    *wsFrac = 0;

    iLenStart = wcslen(wsBufStart);
    iLenNom = wcslen(wsBufNom);
    iLenDenom = wcslen(wsBufDenom);
    iLenEnd = wcslen(wsBufEnd);
  }

  HDC ldc = graphics->GetHDC();
  LOGFONT lFnt;
  lFnt.lfHeight = -Round(1.6*m_dUnitScale*cFileAttrs.dFontSize);
  lFnt.lfWidth = 0;
  lFnt.lfEscapement = 0;
  lFnt.lfOrientation = 0;
  lFnt.lfWeight = FW_NORMAL;
  if(cFileAttrs.bFontAttrs & 8) lFnt.lfWeight = FW_BOLD;
  lFnt.lfItalic = cFileAttrs.bFontAttrs & 1;
  lFnt.lfUnderline = cFileAttrs.bFontAttrs & 2;
  lFnt.lfStrikeOut = cFileAttrs.bFontAttrs & 4;
  lFnt.lfCharSet = DEFAULT_CHARSET;
  lFnt.lfOutPrecision = OUT_DEFAULT_PRECIS;
  lFnt.lfClipPrecision = CLIP_DEFAULT_PRECIS;
  lFnt.lfQuality = DEFAULT_QUALITY;
  lFnt.lfPitchAndFamily = DEFAULT_PITCH;
  MultiByteToWideChar(CP_UTF8, 0, cFileAttrs.sFontFace, -1, lFnt.lfFaceName, LF_FACESIZE);
  Font *phFntSm = NULL;
  Font hFnt(ldc, &lFnt);
  if(wsFrac)
  {
    lFnt.lfHeight = -Round(0.96*m_dUnitScale*cFileAttrs.dFontSize);
    phFntSm = new Font(ldc, &lFnt);
  }

  graphics->ReleaseHDC(ldc);

  graphics->TranslateTransform((REAL)(m_cViewOrigin.x + pPrim->cPt1.x), (REAL)(m_cViewOrigin.y + pPrim->cPt1.y));
  double dPi2 = M_PI/2.0;
  graphics->RotateTransform((REAL)(180.0*(pPrim->cPt2.x - dPi2)/M_PI));
  graphics->SetTextRenderingHint(TextRenderingHintAntiAlias);

  PointF cOrig(0.0, 0.0);
  RectF cRect;
  SizeF cSizeStart, cSizeEnd, cSizeNom, cSizeDenom;

  Status st = graphics->MeasureString(wsBufStart, iLenStart, &hFnt, cOrig, &cRect);
  cRect.GetSize(&cSizeStart);

  double dTextWidth = cSizeStart.Width;

  if(wsFrac)
  {
    graphics->MeasureString(wsBufEnd, iLenEnd, &hFnt, cOrig, &cRect);
    cRect.GetSize(&cSizeEnd);
    graphics->MeasureString(wsBufNom, iLenNom, phFntSm, cOrig, &cRect);
    cRect.GetSize(&cSizeNom);
    graphics->MeasureString(wsBufDenom, iLenDenom, phFntSm, cOrig, &cRect);
    cRect.GetSize(&cSizeDenom);
    dTextWidth += cSizeEnd.Width + cSizeNom.Width + cSizeDenom.Width - 2.0*da;
  }

  REAL rWidth = (REAL)(m_dUnitScale*dLineWidth);
  Pen hPen(Color(EncodeColor(dwColor)), rWidth);

  double dx = -dTextWidth/2.0;
  if(bDiam)
  {
    dTextWidth += 2.5*da;
    dx = da - dTextWidth/2.0;
    graphics->DrawEllipse(&hPen, (REAL)(dx - da), (REAL)(-2.3*da), (REAL)(2.0*da), (REAL)(2.0*da));
    graphics->DrawLine(&hPen, (REAL)(dx - da), (REAL)(-0.3*da), (REAL)(dx + da), (REAL)(-2.3*da));
    dx += 1.5*da;
  }

  SolidBrush hBr(Color(EncodeColor(dwColor)));

  cOrig.X = (REAL)dx;
  cOrig.Y = -0.7*cSizeStart.Height;
  graphics->DrawString(wsBufStart, iLenStart, &hFnt, cOrig, &hBr);

  PDDimension pDim = pObj->GetDimen(iPos);
  pDim->cExt.cPt1.x = -dTextWidth/2.0/m_dUnitScale;
  pDim->cExt.cPt1.y = 0.0;
  pDim->cExt.cPt2.x = dTextWidth/2.0/m_dUnitScale;
  pDim->cExt.cPt2.y = 1.15*cFileAttrs.dFontSize;

  if(wsFrac)
  {
    dx += (cSizeStart.Width - 1.0*da);
    cOrig.X = (REAL)dx;
    cOrig.Y = -0.7*cSizeNom.Height - (REAL)(1.7*da);
    graphics->DrawString(wsBufNom, iLenNom, phFntSm, cOrig, &hBr);

    dx += (cSizeNom.Width + 0.0*da);
    cOrig.X = (REAL)dx;
    cOrig.Y = -0.7*cSizeDenom.Height + (REAL)(0.4*da);
    graphics->DrawString(wsBufDenom, iLenDenom, phFntSm, cOrig, &hBr);

    graphics->DrawLine(&hPen, (REAL)(dx - 0.7*da), (REAL)(-0.8*da), (REAL)(dx + 0.7*da), (REAL)(-2.2*da));

    dx += (cSizeDenom.Width - 1.0*da);
    cOrig.X = (REAL)dx;
    cOrig.Y = -0.7*cSizeEnd.Height;
    graphics->DrawString(wsBufEnd, iLenEnd, &hFnt, cOrig, &hBr);
  }

  free(wsBufStart);

  graphics->ResetTransform();

  if(iLen > 0) free(psBuf);
}

void CMainWnd::DrawPrimitive(Graphics *graphics, Pen *pen, GraphicsPath *path, PDPrimitive pPrim)
{
  double dr;
  //POINT pPts[3];

  CDPoint cStartPt, cEndPt;
  CDPoint cPt1, cPt2;
  //Pen locPen(Color(255, 0, 0, 255), 2.5);

  switch(pPrim->iType)
  {
  case 1:
    cStartPt.x = pPrim->cPt1.x + m_cViewOrigin.x;
    cStartPt.y = pPrim->cPt1.y + m_cViewOrigin.y;
    cEndPt.x = pPrim->cPt2.x + m_cViewOrigin.x;
    cEndPt.y = pPrim->cPt2.y + m_cViewOrigin.y;
    if(!path)
      graphics->DrawLine(pen, (REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)cEndPt.x, (REAL)cEndPt.y);
    else
      path->AddLine((REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)cEndPt.x, (REAL)cEndPt.y);
    break;
  case 2:
    cStartPt.x = pPrim->cPt1.x + m_cViewOrigin.x + pPrim->cPt2.x*cos(pPrim->cPt3.x);
    cStartPt.y = pPrim->cPt1.y + m_cViewOrigin.y + pPrim->cPt2.x*sin(pPrim->cPt3.x);
    cEndPt.x = pPrim->cPt1.x + m_cViewOrigin.x + pPrim->cPt2.x*cos(pPrim->cPt3.y);
    cEndPt.y = pPrim->cPt1.y + m_cViewOrigin.y + pPrim->cPt2.x*sin(pPrim->cPt3.y);

    dr = GetDist(cStartPt, cEndPt);
    if((dr > 2) || (fabs(pPrim->cPt3.y - pPrim->cPt3.x) > 0.001))
    {
      dr = pPrim->cPt2.x;
      if(fabs(pPrim->cPt4.x - 1.0) < 0.2)
      {
        cStartPt.x = pPrim->cPt3.y*180.0/M_PI;
        cStartPt.y = (pPrim->cPt3.x - pPrim->cPt3.y)*180.0/M_PI;
        if(cStartPt.y > 0.0) cStartPt.y -= 360.0;
      }
      else
      {
        cStartPt.x = pPrim->cPt3.x*180.0/M_PI;
        cStartPt.y = (pPrim->cPt3.y - pPrim->cPt3.x)*180.0/M_PI;
        if(cStartPt.y < 0.0) cStartPt.y += 360.0;
      }
      if(!path)
        graphics->DrawArc(pen, (REAL)(m_cViewOrigin.x + pPrim->cPt1.x - dr),
          (REAL)(m_cViewOrigin.y + pPrim->cPt1.y - dr),
          (REAL)2.0*dr, (REAL)2.0*dr, (REAL)cStartPt.x, (REAL)cStartPt.y);
      else
        path->AddArc((REAL)(m_cViewOrigin.x + pPrim->cPt1.x - dr),
          (REAL)(m_cViewOrigin.y + pPrim->cPt1.y - dr),
          (REAL)2.0*dr, (REAL)2.0*dr, (REAL)cStartPt.x, (REAL)cStartPt.y);
    }
    else
    {
      if(fabs(pPrim->cPt4.x - 1.0) < 0.2)
      {
        if(!path)
          graphics->DrawLine(pen, (REAL)cEndPt.x, (REAL)cEndPt.y, (REAL)cStartPt.x, (REAL)cStartPt.y);
        else
          path->AddLine((REAL)cEndPt.x, (REAL)cEndPt.y, (REAL)cStartPt.x, (REAL)cStartPt.y);
      }
      else
      {
        if(!path)
          graphics->DrawLine(pen, (REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)cEndPt.x, (REAL)cEndPt.y);
        else
          path->AddLine((REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)cEndPt.x, (REAL)cEndPt.y);
      }
    }
    break;
  case 3:
    //dr = (pPrim->cPt2.x - pPrim->cPt1.x);
    dr = pPrim->cPt2.x;
    if(!path)
      graphics->DrawEllipse(pen, (REAL)(m_cViewOrigin.x + pPrim->cPt1.x - dr),
        (REAL)(m_cViewOrigin.y + pPrim->cPt1.y - dr), (REAL)2.0*dr, (REAL)2.0*dr);
    else
      path->AddEllipse((REAL)(m_cViewOrigin.x + pPrim->cPt1.x - dr),
        (REAL)(m_cViewOrigin.y + pPrim->cPt1.y - dr), (REAL)2.0*dr, (REAL)2.0*dr);
    break;
  case 4:
    cPt1 = (pPrim->cPt1 + 2.0*pPrim->cPt2)/3.0;
    cPt2 = (pPrim->cPt3 + 2.0*pPrim->cPt2)/3.0;
    if(!path)
      graphics->DrawBezier(pen, (REAL)(m_cViewOrigin.x + pPrim->cPt1.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt1.y),
        (REAL)(m_cViewOrigin.x + cPt1.x),
        (REAL)(m_cViewOrigin.y + cPt1.y),
        (REAL)(m_cViewOrigin.x + cPt2.x),
        (REAL)(m_cViewOrigin.y + cPt2.y),
        (REAL)(m_cViewOrigin.x + pPrim->cPt3.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt3.y));
    else
      path->AddBezier((REAL)(m_cViewOrigin.x + pPrim->cPt1.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt1.y),
        (REAL)(m_cViewOrigin.x + cPt1.x),
        (REAL)(m_cViewOrigin.y + cPt1.y),
        (REAL)(m_cViewOrigin.x + cPt2.x),
        (REAL)(m_cViewOrigin.y + cPt2.y),
        (REAL)(m_cViewOrigin.x + pPrim->cPt3.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt3.y));
    break;
  case 5:
    if(!path)
      graphics->DrawBezier(pen, (REAL)(m_cViewOrigin.x + pPrim->cPt1.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt1.y),
        (REAL)(m_cViewOrigin.x + pPrim->cPt2.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt2.y),
        (REAL)(m_cViewOrigin.x + pPrim->cPt3.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt3.y),
        (REAL)(m_cViewOrigin.x + pPrim->cPt4.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt4.y));
    else
      path->AddBezier((REAL)(m_cViewOrigin.x + pPrim->cPt1.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt1.y),
        (REAL)(m_cViewOrigin.x + pPrim->cPt2.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt2.y),
        (REAL)(m_cViewOrigin.x + pPrim->cPt3.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt3.y),
        (REAL)(m_cViewOrigin.x + pPrim->cPt4.x),
        (REAL)(m_cViewOrigin.y + pPrim->cPt4.y));
    break;
  case 7:
    cStartPt.x = pPrim->cPt1.x + m_cViewOrigin.x - 6;
    cStartPt.y = pPrim->cPt1.y + m_cViewOrigin.y;
    graphics->DrawLine(pen, (REAL)cStartPt.x, (REAL)cStartPt.y, (REAL)(cStartPt.x + 13), (REAL)cStartPt.y);
    cEndPt.x = pPrim->cPt1.x + m_cViewOrigin.x;
    cEndPt.y = pPrim->cPt1.y + m_cViewOrigin.y + 7;
    graphics->DrawLine(pen, (REAL)cEndPt.x, (REAL)(cEndPt.y - 13), (REAL)cEndPt.x, (REAL)cEndPt.y);
    break;
  case 9:
    DrawDimArrow(graphics, pen, path, pPrim);
    break;
  }
}

void CMainWnd::DrawRegRasterLine(Graphics *graphics, Pen *pen, CDPoint cPt1, CDPoint cPt2)
{
  graphics->DrawLine(pen, (REAL)cPt1.x, (REAL)cPt1.y, (REAL)cPt2.x, (REAL)cPt2.y);
  CDPoint cDir = cPt2 - cPt1;
  double dDist = GetNorm(cDir);
  if(dDist > g_dPrec)
  {
    CDPoint cPt3, cPt4;
    cDir /= dDist;
    cPt3.x = -10.0;
    cPt3.y = 4.0;
    cPt4 = cPt2 + Rotate(cPt3, cDir, true);
    graphics->DrawLine(pen, (REAL)cPt4.x, (REAL)cPt4.y, (REAL)cPt2.x, (REAL)cPt2.y);
    cPt3.y = -4.0;
    cPt4 = cPt2 + Rotate(cPt3, cDir, true);
    graphics->DrawLine(pen, (REAL)cPt4.x, (REAL)cPt4.y, (REAL)cPt2.x, (REAL)cPt2.y);
  }
}

DWORD CodeRGBAColor(unsigned char *pColor)
{
  return pColor[0] | (pColor[1] << 8) | (pColor[2] << 16) | (pColor[3] << 24);
}

void CMainWnd::DrawObject(HWND hWnd, Graphics *graphics, PDObject pObj, int iMode, int iDimen)
{
  if(pObj->GetType() == dtGroup)
  {
    int n = pObj->GetSubObjectCount(false);
    PDObject pObj1;
    for(int i = 0; i < n; i++)
    {
      pObj1 = pObj->GetSubObject(i);
      DrawObject(hWnd, graphics, pObj1, iMode, iDimen);
    }
    return;
  }

  bool bSel = pObj->GetSelected();
  CDLineStyle cStyle = pObj->GetLineStyle();

  DWORD dwColor = CodeRGBAColor(cStyle.cColor);
  if(iMode == 1) dwColor = m_lActiveColor;
  else if(iMode == 2) dwColor = m_lHighColor;
  else if(bSel) dwColor = m_lSelColor;

  DWORD dwFillColor = CodeRGBAColor(cStyle.cFillColor);
  if((iMode == 1) || (m_iToolMode == tolEditSpline)) dwFillColor = m_lActiveFillColor;
  else if(iMode == 2) dwFillColor = m_lHighFillColor;
  else if(bSel) dwFillColor = m_lSelFillColor;

  REAL rWidth = fabs(cStyle.dWidth)*m_dUnitScale;
  REAL rPtRad = rWidth;
  if(rPtRad < 2.0) rPtRad = 2.0;
  REAL rDashFactor = rWidth;
  if(rDashFactor < 1.0) rDashFactor = 1.0;

  LineCap lc = LineCapRound;
  if(cStyle.cCapType == 0) lc = LineCapFlat;
  else if(cStyle.cCapType == 2) lc = LineCapSquare;
  DashCap dc = DashCapRound;
  if(cStyle.cCapType != 1) dc = DashCapFlat;

  LineJoin lj = LineJoinRound;
  if(cStyle.cJoinType == 2) lj = LineJoinBevel;
  else if(cStyle.cJoinType == 0) lj = LineJoinMiter;

  Color hLineCol(EncodeColor(dwColor));

  Pen hPen(hLineCol, rWidth);
  hPen.SetLineCap(lc, lc, dc);
  hPen.SetLineJoin(lj);

  Pen hPtPen(Color(EncodeColor(dwColor)), 0.0);
  Pen hCentPen(Color(EncodeColor(0xFF888888)), 0.0);
  GraphicsPath hPath(FillModeWinding);
  SolidBrush hBrush(Color(EncodeColor(dwFillColor)));

  // this does not work
  /*if((cStyle.dBlur > g_dPrec) && (iMode < 1) && !bSel)
  {
    GraphicsPath hBrPath;
    hBrPath.AddEllipse(0.0, 0.0, rWidth, rWidth);
    PathGradientBrush pthGrBrush(&hBrPath);
    pthGrBrush.SetCenterColor(hLineCol);

    // Set the color along the entire boundary of the path to aqua.
    //Color colors[] = {Color(cStyle.cColor[0], cStyle.cColor[1], cStyle.cColor[2], 0)};
    Color colors[] = {hLineCol};
    int colCnt = 1;
    pthGrBrush.SetSurroundColors(colors, &colCnt);

    hPen.SetBrush(&pthGrBrush);
  }*/

  CDPrimitive cPrim;
  PDDimension pDim;
  pObj->GetFirstPrimitive(&cPrim, m_dUnitScale, iDimen);

  if(iDimen < -1)
  {
    while(cPrim.iType > 0)
    {
      if(cPrim.iType == 6)
      {
        graphics->DrawEllipse(&hPtPen, (REAL)(cPrim.cPt1.x + m_cViewOrigin.x - rPtRad),
          (REAL)(cPrim.cPt1.y + m_cViewOrigin.y - rPtRad), (REAL)2.0*rPtRad, (REAL)2.0*rPtRad);
      }
      else if(cPrim.iType == 7)
      {
        if(iMode == 0) DrawPrimitive(graphics, &hCentPen, NULL, &cPrim);
        else DrawPrimitive(graphics, &hPtPen, NULL, &cPrim);
      }
      else if(cPrim.iType == 8)
      {
        graphics->DrawRectangle(&hPtPen, (REAL)(cPrim.cPt1.x + m_cViewOrigin.x - rPtRad),
          (REAL)(cPrim.cPt1.y + m_cViewOrigin.y - rPtRad), (REAL)2.0*rPtRad, (REAL)2.0*rPtRad);
      }
      else if(cPrim.iType == 10)
      {
        DrawDimText(hWnd, graphics, &cPrim, pObj, dwColor, fabs(cStyle.dWidth));
      }
      else if(cPrim.iType == 11)
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
          hPath.Reset();
          //if(fabs(cPrim.cPt2.x - 1.0) < 0.2)
          //  cairo_move_to(cr, m_cViewOrigin.x + cPrim.cPt3.x, m_cViewOrigin.y + cPrim.cPt3.y);
        }
        if(fabs(cPrim.cPt1.y - 1.0) < 0.2)
        {
          hPath.StartFigure();
          //if(fabs(cPrim.cPt2.x - 1.0) < 0.2)
          //  cairo_move_to(cr, m_cViewOrigin.x + cPrim.cPt3.x, m_cViewOrigin.y + cPrim.cPt3.y);
        }
        if(fabs(cPrim.cPt1.y - 2.0) < 0.2)
        {
          hPath.CloseFigure();
          if(fabs(cPrim.cPt1.x) < 0.2) hPath.StartFigure();
        }
        if(fabs(cPrim.cPt1.x - 2.0) < 0.2)
        {
          if(cPrim.cPt2.x > g_dPrec)
          {
            //REAL dDash[6];
            double dSegLen;
            double dCapOffset = 0.0;
            for(int i = 0; i < cStyle.iSegments; i++)
            {
              dSegLen = cStyle.dPattern[i];
              if(i % 2 == 0)
              {
                if(dSegLen < g_dDashMin)
                {
                  dSegLen = g_dDashMin;
                  dCapOffset = g_dDashMin;
                }
                m_pdDashPattern[i] = (REAL)(1.0 + m_dUnitScale*cPrim.cPt2.x*dSegLen/rDashFactor);
              }
              else
              {
                m_pdDashPattern[i] = (REAL)(-1.0 + m_dUnitScale*cPrim.cPt2.x*(dSegLen - dCapOffset)/rDashFactor);
                dCapOffset = 0.0;
              }
            }
            hPen.SetDashPattern(m_pdDashPattern, cStyle.iSegments);
            hPen.SetDashOffset((REAL)cPrim.cPt2.y/rDashFactor);
          }
          graphics->DrawPath(&hPen, &hPath);
          hPath.Reset();
          hPen.SetDashStyle(DashStyleSolid);
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
          hPath.Reset();
          //if(fabs(cPrim.cPt2.x - 1.0) < 0.2)
          //  cairo_move_to(cr, m_cViewOrigin.x + cPrim.cPt3.x, m_cViewOrigin.y + cPrim.cPt3.y);
        }
        if(fabs(cPrim.cPt1.y - 1.0) < 0.2)
        {
          hPath.StartFigure();
          //if(fabs(cPrim.cPt2.x - 1.0) < 0.2)
          //  cairo_move_to(cr, m_cViewOrigin.x + cPrim.cPt3.x, m_cViewOrigin.y + cPrim.cPt3.y);
        }
        if(fabs(cPrim.cPt1.y - 2.0) < 0.2)
        {
          hPath.CloseFigure();
          if(fabs(cPrim.cPt1.x) < 0.2) hPath.StartFigure();
        }
        if(fabs(cPrim.cPt1.x - 2.0) < 0.2)
        {
          hPath.CloseFigure();
          graphics->FillPath(&hBrush, &hPath);
          hPath.Reset();
        }
      }
      else if(cPrim.iType == 13)
      {
        int iVisible = pObj->GetAuxInt();
        if((iMode < 1) && (iVisible < 1))
        {
          int iImageSize = 0;
          unsigned char *pImageData = pObj->GetRasterData(&iImageSize);
          HGLOBAL hGlobal = GlobalAlloc(GMEM_MOVEABLE, iImageSize);
          void *ptr = GlobalLock(hGlobal);
          memcpy(ptr, pImageData, iImageSize);
          GlobalUnlock(hGlobal);
          IStream *pStream = NULL;
          CreateStreamOnHGlobal(hGlobal, TRUE, &pStream);
          Gdiplus::Image *image = Gdiplus::Image::FromStream(pStream);

          //double dScaleFactor = 3.125;
          double dScaleFactor = 1.043002;
          double dScaleFactorX = dScaleFactor*image->GetHorizontalResolution()/100.0;
          double dScaleFactorY = dScaleFactor*image->GetVerticalResolution()/100.0;
          Matrix *matrix = new Matrix(dScaleFactorX*cPrim.cPt1.x, dScaleFactorX*cPrim.cPt2.x,
            dScaleFactorY*cPrim.cPt1.y, dScaleFactorY*cPrim.cPt2.y,
            cPrim.cPt3.x + m_cViewOrigin.x, cPrim.cPt3.y + m_cViewOrigin.y);
          graphics->SetTransform(matrix);
          graphics->DrawImage(image, (REAL)0.0, (REAL)0.0);
          graphics->ResetTransform();
          pStream->Release();
          delete image;
          delete matrix;
        }
      }
      else if(cPrim.iType == 14)
      {
        //if(((m_iDrawMode == modSpline) || (m_iToolMode == tolEditSpline)) && (pObj == m_pActiveObject))
        // only draw handles when editing spline
        if((m_iToolMode == tolEditSpline) && (pObj == m_pActiveObject))
        {
          int iCnt = (int)cPrim.cPt4.x;
          int iMask = (int)cPrim.cPt4.y;
          REAL rSquareSize = 1.5*rWidth;
          if(rSquareSize < 3.0) rSquareSize = 3.0;
          CDPoint cPt;
          bool bFill;
          for(int i = 0; i < iCnt; i++)
          {
            switch(i)
            {
            case 0:
              cPt = cPrim.cPt1;
              bFill = iMask & 1;
              break;
            case 1:
              cPt = cPrim.cPt2;
              bFill = iMask & 2;
              break;
            case 2:
              cPt = cPrim.cPt3;
              bFill = iMask & 4;
              break;
            }
            hPath.StartFigure();
            hPath.AddRectangle(Rect((INT)(cPt.x + m_cViewOrigin.x - rSquareSize),
              (INT)(cPt.y + m_cViewOrigin.y - rSquareSize),
              (INT)2.0*rSquareSize, (INT)2.0*rSquareSize));
            hPath.CloseFigure();
            if(bFill) graphics->FillPath(&hBrush, &hPath);
            else graphics->DrawPath(&hPen, &hPath);
            hPath.Reset();
          }
        }
      }
      else DrawPrimitive(graphics, &hPen, &hPath, &cPrim);
      pObj->GetNextPrimitive(&cPrim, m_dUnitScale, iDimen);
    }

    if(iMode == 0)
    {
      for(int i = 0; i < pObj->GetDimenCount(); i++)
      {
        pDim = pObj->GetDimen(i);
        if(pDim->bSelected) dwColor = m_lSelColor;
        else dwColor = CodeRGBAColor(cStyle.cColor);
        hPen.SetColor(Color(EncodeColor(dwColor)));

        pObj->GetFirstPrimitive(&cPrim, m_dUnitScale, i);
        while(cPrim.iType > 0)
        {
          if(cPrim.iType == 10)
          {
            DrawDimText(hWnd, graphics, &cPrim, pObj, dwColor, fabs(cStyle.dWidth));
          }
          else DrawPrimitive(graphics, &hPen, &hPath, &cPrim);
          pObj->GetNextPrimitive(&cPrim, m_dUnitScale, i);
        }
      }
    }
  }
  else
  {
    if((iMode < 1) && (iDimen > -1))
    {
      pDim = pObj->GetDimen(iDimen);
      if(pDim->bSelected) dwColor = m_lSelColor;
      else dwColor = CodeRGBAColor(cStyle.cColor);
      hPen.SetColor(Color(EncodeColor(dwColor)));
    }

    while(cPrim.iType > 0)
    {
      if(cPrim.iType == 10)
      {
        DrawDimText(hWnd, graphics, &cPrim, pObj, dwColor, fabs(cStyle.dWidth));
      }
      else DrawPrimitive(graphics, &hPen, &hPath, &cPrim);
      pObj->GetNextPrimitive(&cPrim, m_dUnitScale, iDimen);
    }
  }
}

int CMainWnd::GetDynMode()
{
  int iRes = 0;
  if(m_iDrawMode > modSelect) iRes = 1;
  else if(m_iToolMode == tolCopyPar) iRes = 2;
  else if(m_iToolMode == tolRound) iRes = 3;
  else if(m_iToolMode == tolDimen) iRes = 4;
  return iRes;
}

long SetColorAlpha(long lColor, int iAlpha) // 0 <= iAlpha <= 100
{
  unsigned char normAlpha = (unsigned char)Round(255.0*iAlpha/100.0);
  return (lColor & ~0xFF000000) | (normAlpha << 24);
}

LRESULT CMainWnd::WMMouseMove(HWND hwnd, WPARAM fwKeys, int xPos, int yPos)
{
  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  if(m_iButton == 2)
  {
    m_cViewOrigin.x = (xPos - m_cZoomOrig.x);
    m_cViewOrigin.y = (yPos - m_cZoomOrig.y);

    int ix = xPos - m_cLastMovePt.x;
    int iy = yPos - m_cLastMovePt.y;
    m_cLastSnapPt.x += ix;
    m_cLastSnapPt.y += iy;
    ScrollWindowEx(hwnd, ix, iy, &rc, &rc, NULL, NULL, SW_ERASE | SW_INVALIDATE);
    m_cLastMovePt.x = xPos;
    m_cLastMovePt.y = yPos;
    return 0;
  }

  double dx = (xPos - m_cViewOrigin.x)/m_dUnitScale;
  double dy = (yPos - m_cViewOrigin.y)/m_dUnitScale;

  HDC hdc;
  double dTol;
  CDRect cdr;
  int iDynMode = GetDynMode();

  if(m_iDrawMode + m_iToolMode < 1)
  {
    if((m_iButton == 1) || (m_iButton == 3))
    {
      m_cLastMovePt.x = xPos;
      m_cLastMovePt.y = yPos;
      REAL x1 = min(m_cLastDownPt.x, m_cLastMovePt.x);
      REAL y1 = min(m_cLastDownPt.y, m_cLastMovePt.y);
      REAL dw1 = (REAL)abs(m_cLastMovePt.x - m_cLastDownPt.x);
      REAL dh1 = (REAL)abs(m_cLastMovePt.y - m_cLastDownPt.y);
      Bitmap bmp(rc.right - rc.left, rc.bottom - rc.top);
      Graphics graphics(&bmp);
      graphics.SetSmoothingMode(SmoothingModeAntiAlias);
      graphics.DrawImage(m_pDrawBuffer, 0, 0);

      Pen pen(EncodeColor(m_lSelColor), 1);
      long dwColor = SetColorAlpha(m_lSelColor, 35);
      SolidBrush fillSel(EncodeColor(dwColor));
      graphics.FillRectangle(&fillSel, x1, y1, dw1, dh1);
      graphics.DrawRectangle(&pen, x1, y1, dw1, dh1);

      hdc = GetDC(hwnd);
      Graphics dstgraph(hdc);
      dstgraph.DrawImage(&bmp, 0, 0);
      ReleaseDC(hwnd, NULL);
    }
    else if(m_iButton < 1)
    {
      CDPoint cPt = {dx, dy};
      dTol = (double)m_iSelectTolerance/m_dUnitScale;

      int iDimen;
      PDObject pNewHigh = m_pDrawObjects->SelectByPoint(cPt, dTol, &iDimen);

      if((m_pHighObject != pNewHigh) || (iDimen != m_iHighDimen))
      {
        Bitmap bmp(rc.right - rc.left, rc.bottom - rc.top);
        Graphics graphics(&bmp);
        graphics.SetSmoothingMode(SmoothingModeAntiAlias);
        graphics.DrawImage(m_pDrawBuffer, 0, 0);
        if(pNewHigh) DrawObject(hwnd, &graphics, pNewHigh, 2, iDimen);

        hdc = GetDC(hwnd);
        //IntersectClipRect(hdc, rc.left, rc.top, rc.right, rc.bottom);

        Graphics dstgraph(hdc);
        dstgraph.DrawImage(&bmp, 0, 0);

        //SelectClipRgn(hdc, NULL);
        ReleaseDC(hwnd, NULL);

        m_pHighObject = pNewHigh;
        m_iHighDimen = iDimen;
      }
    }
  }

  wchar_t buf[64];
  swprintf(buf, L"%.3f, %.3f", dx/m_cFSR.cPaperUnit.dBaseToUnit, dy/m_cFSR.cPaperUnit.dBaseToUnit);
  SendMessage(m_hStatus, SB_SETTEXT, 0, (LPARAM)buf);

  if((m_iButton > 0) && (m_iToolMode != tolEditSpline)) return 0;

  int iCnt = 0;
  PDObject pObj1, pObj2;

  m_cLastSnapPt.x = xPos;
  m_cLastSnapPt.y = yPos;

  if(m_iDrawMode + m_iToolMode > 0)
  {
    CDLine cSnapPt;
    bool bHasLastPoint = false;
    CDInputPoint cLstInPt;
    CDPoint cDir1, cDir2;
    double dAng1;
    bool bDoSnap = true;

    if((m_iDrawMode == modLine) && m_pActiveObject)
    {
      bHasLastPoint = m_pActiveObject->GetPoint(0, 0, &cLstInPt);
    }

    CDLine cPtX;

    if(fwKeys & MK_CONTROL)
    {
      bDoSnap = false;

      if(bHasLastPoint)
      {
        CDPoint cMainDir = {1.0, 0.0};
        CDLine cPtX;

        iCnt = m_pDrawObjects->GetSelectCount(2);
        if(iCnt == 1)
        {
          pObj1 = m_pDrawObjects->GetSelected(0);
          if(m_cLastDynPt.bIsSet)
          pObj1->GetDistFromPt(m_cLastDynPt.cOrigin, m_cLastDynPt.cOrigin, true, &cPtX, NULL);
          else pObj1->GetDistFromPt(cLstInPt.cPoint, cLstInPt.cPoint, true, &cPtX, NULL);
          if(cPtX.bIsSet) cMainDir = cPtX.cDirection;
        }

        if((fwKeys & MK_SHIFT) && (iCnt == 1))
        {
          cDir2.x = dx;
          cDir2.y = dy;
          cDir1 = pObj1->GetPointToDir(cLstInPt.cPoint, m_dSavedAngle, cDir2);
          m_cLastDynPt.bIsSet = true;
          m_cLastDynPt.cOrigin = cDir1;
          m_cLastDrawPt = cDir2;
          bDoSnap = true;
        }
        else
        {
          m_cLastDynPt.bIsSet = false;
          cDir1.x = dx - cLstInPt.cPoint.x;
          cDir1.y = dy - cLstInPt.cPoint.y;
          cDir2 = Rotate(cDir1, cMainDir, false);
          dAng1 = atan2(cDir2.y, cDir2.x);
          dAng1 *= 180.0/M_PI/m_cFSR.cAngUnit.dBaseToUnit;
          double dAng2 = m_cFSR.dAngGrid*(Round((double)dAng1/m_cFSR.dAngGrid));
          dAng2 *= M_PI*m_cFSR.cAngUnit.dBaseToUnit/180.0;

          m_dSavedAngle = dAng2;

          CDPoint cDir3 = {cos(dAng2), sin(dAng2)};
          cSnapPt.cOrigin = Rotate(cDir2, cDir3, false);
          cSnapPt.cOrigin.y = 0.0;
          cDir1 = Rotate(cSnapPt.cOrigin, cDir3, true);
          m_cLastDrawPt = cLstInPt.cPoint + Rotate(cDir1, cMainDir, true);
        }
      }
      else
      {
        dx = m_cFSR.dXGrid*(Round((double)dx/m_cFSR.dXGrid));
        dy = m_cFSR.dYGrid*(Round((double)dy/m_cFSR.dYGrid));
        m_cLastDrawPt.x = dx;
        m_cLastDrawPt.y = dy;
      }
      m_cLastSnapPt.x = m_cViewOrigin.x + m_cLastDrawPt.x*m_dUnitScale;
      m_cLastSnapPt.y = m_cViewOrigin.y + m_cLastDrawPt.y*m_dUnitScale;
    }
    else m_cLastDynPt.bIsSet = false;

    int iRestrict = 0;
    double dRestrictVal[2];
    dRestrictVal[0] = m_dRestrictValue;
    dRestrictVal[1] = m_dRestrictValue2;

    if(bDoSnap)
    {
      m_cLastDrawPt.x = (m_cLastSnapPt.x - m_cViewOrigin.x)/m_dUnitScale;
      m_cLastDrawPt.y = (m_cLastSnapPt.y - m_cViewOrigin.y)/m_dUnitScale;
      dTol = (double)m_iSnapTolerance/m_dUnitScale;

      int iSnapType = 0;
      if(m_iToolMode == tolConflict) iSnapType = 1;
      if(m_pDrawObjects->GetSnapPoint(iSnapType, m_cLastDrawPt,
        dTol, &cSnapPt, m_pActiveObject) > 0)
      {
        if((fwKeys & MK_SHIFT) && (iCnt == 1))
        {
          for(int i = 0; i < 4; i++)
          {
            cDir2 = cSnapPt.cOrigin;
            cDir1 = pObj1->GetPointToDir(cLstInPt.cPoint, m_dSavedAngle, cDir2);
            m_cLastDynPt.bIsSet = true;
            m_cLastDynPt.cOrigin = cDir1;
            m_cLastDrawPt = cDir2;
            m_pDrawObjects->GetSnapPoint(iSnapType, m_cLastDrawPt, dTol, &cSnapPt,
              m_pActiveObject);
          }
        }

        m_cLastDrawPt = cSnapPt.cOrigin;
        m_cLastSnapPt.x = m_cViewOrigin.x + m_cLastDrawPt.x*m_dUnitScale;
        m_cLastSnapPt.y = m_cViewOrigin.y + m_cLastDrawPt.y*m_dUnitScale;
      }

      if(m_pActiveObject)
      {
        if((m_iDrawMode == modLine) && (iDynMode != 2))
        {
          if(IS_ANGLE_VAL(m_iRestrictSet)) iRestrict |= 1;
          if(iRestrict & 1)
          {
            if(m_iRestrictSet == 0) dRestrictVal[0] /= m_cFSR.cAngUnit.dBaseToUnit;
            if(m_iRestrictSet != 3) dRestrictVal[0] *= M_PI/180.0;
          }
        }
        else
        {
          if(IS_LENGTH_VAL(m_iRestrictSet)) iRestrict |= 1;
          if(iRestrict & 1)
          {
            if(m_iRestrictSet == 0)
            {
              if(m_bPaperUnits)
                dRestrictVal[0] *= m_cFSR.cPaperUnit.dBaseToUnit;
              else dRestrictVal[0] *= m_cFSR.cLenUnit.dBaseToUnit;
            }
            if(!m_bPaperUnits) dRestrictVal[0] *= m_dDrawScale;
          }
          if(IS_LENGTH_VAL(m_iRestrictSet2)) iRestrict |= 2;
          if(iRestrict & 2)
          {
            if(m_iRestrictSet2 == 0)
            {
              if(m_bPaperUnits)
                dRestrictVal[1] *= m_cFSR.cPaperUnit.dBaseToUnit;
              else dRestrictVal[1] *= m_cFSR.cLenUnit.dBaseToUnit;
            }
            if(!m_bPaperUnits) dRestrictVal[1] *= m_dDrawScale;
          }
        }

        if(iDynMode == 3)
        {
          iCnt = m_pDrawObjects->GetSelectCount(2);
          if(iCnt == 2)
          {
            pObj1 = m_pDrawObjects->GetSelected(0);
            pObj2 = m_pDrawObjects->GetSelected(1);
            m_pActiveObject->BuildRound(pObj1, pObj2, m_cLastDrawPt, iRestrict & 1,
              dRestrictVal[0]);
          }
        }

        iRestrict = m_pActiveObject->GetRestrictPoint(m_cLastDrawPt,
          iDynMode, iRestrict, dRestrictVal, &cSnapPt.cOrigin);
      }

      if(iRestrict > 0)
      {
        m_cLastDrawPt = cSnapPt.cOrigin;
        m_cLastSnapPt.x = m_cViewOrigin.x + m_cLastDrawPt.x*m_dUnitScale;
        m_cLastSnapPt.y = m_cViewOrigin.y + m_cLastDrawPt.y*m_dUnitScale;
      }
    }

    Bitmap bmp(rc.right - rc.left, rc.bottom - rc.top);
    Graphics graphics(&bmp);
    graphics.SetSmoothingMode(SmoothingModeAntiAlias);
    graphics.DrawImage(m_pDrawBuffer, 0, 0);

    graphics.DrawLine(m_redPen, (REAL)(m_cLastSnapPt.x - 10), (REAL)m_cLastSnapPt.y,
      (REAL)(m_cLastSnapPt.x + 10), (REAL)m_cLastSnapPt.y);
    graphics.DrawLine(m_redPen, (REAL)m_cLastSnapPt.x, (REAL)(m_cLastSnapPt.y - 10),
      (REAL)m_cLastSnapPt.x, (REAL)(m_cLastSnapPt.y + 10));

    cPtX.cOrigin = m_cLastDrawPt;
    if(iDynMode == 1)
    {
      cPtX.bIsSet = m_cLastDynPt.bIsSet;
      cPtX.cDirection = m_cLastDynPt.cOrigin;
    }
    else if(iDynMode == 2)
    {
      cPtX.cDirection.x = 0.0;
      if(fwKeys & MK_SHIFT) cPtX.cDirection.x = -1.0;
      if(iRestrict > 0)
      {
        cPtX.cDirection.x = 1.0;
        cPtX.cDirection.y = dRestrictVal[0];
        m_dSavedDist = dRestrictVal[0];
      }
    }

    if(m_pActiveObject)
    {
      double dVal[2];
      boolean bDynValSet = false;
      if(iRestrict & 1)
      {
        dVal[0] = m_dRestrictValue;
        if((m_iDrawMode == modLine) && (iDynMode != 2))
        {
          swprintf(m_wsStatus2Msg, L"%s %.2f %s", m_wsStatus2Base, dVal[0],
            m_cFSR.cAngUnit.wsAbbrev);
        }
        else
        {
          if(m_bPaperUnits)
          {
            swprintf(m_wsStatus2Msg, L"%s %.2f %s", m_wsStatus2Base, dVal[0],
              m_cFSR.cPaperUnit.wsAbbrev);
          }
          else
          {
            swprintf(m_wsStatus2Msg, L"%s %.2f %s", m_wsStatus2Base, dVal[0],
              m_cFSR.cLenUnit.wsAbbrev);
          }
        }
        SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);
      }
      else
      {
        bDynValSet = m_pActiveObject->GetDynValue(m_cLastDrawPt, iDynMode, dVal);
        if(bDynValSet)
        {
          m_dSavedDist = dVal[0];

          if((m_iDrawMode == modLine) && (iDynMode != 2))
          {
            dVal[0] *= m_cFSR.cAngUnit.dBaseToUnit*180.0/M_PI;
            swprintf(m_wsStatus2Msg, L"%s %.2f %s", m_wsStatus2Base, dVal[0],
              m_cFSR.cAngUnit.wsAbbrev);
          }
          else
          {
            if(m_bPaperUnits)
            {
              dVal[0] /= m_cFSR.cPaperUnit.dBaseToUnit;
              swprintf(m_wsStatus2Msg, L"%s %.2f %s", m_wsStatus2Base, dVal[0],
                m_cFSR.cPaperUnit.wsAbbrev);
            }
            else
            {
              dVal[0] /= m_dDrawScale;
              dVal[0] /= m_cFSR.cLenUnit.dBaseToUnit;
              swprintf(m_wsStatus2Msg, L"%s %.2f %s", m_wsStatus2Base, dVal[0],
                m_cFSR.cLenUnit.wsAbbrev);
            }
          }
          SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);
        }
      }
      if(m_iDrawMode == modRectangle)
      {
        if(iRestrict & 2)
        {
          dVal[1] = m_dRestrictValue;
          if(m_bPaperUnits)
          {
            swprintf(m_wsStatus3Msg, L"%s %.2f %s", m_wsStatus3Base, dVal[1],
              m_cFSR.cPaperUnit.wsAbbrev);
          }
          else
          {
            swprintf(m_wsStatus3Msg, L"%s %.2f %s", m_wsStatus3Base, dVal[1],
              m_cFSR.cLenUnit.wsAbbrev);
          }
          //SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus3Msg);
          SendMessage(m_hLab1, WM_SETTEXT, 0, (LPARAM)m_wsStatus3Msg);
        }
        else
        {
          bDynValSet = m_pActiveObject->GetDynValue(m_cLastDrawPt, iDynMode, dVal);
          if(!bDynValSet) bDynValSet = m_pActiveObject->GetDynValue(m_cLastDrawPt, iDynMode, dVal);
          if(bDynValSet)
          {
            if(m_bPaperUnits)
            {
              dVal[1] /= m_cFSR.cPaperUnit.dBaseToUnit;
              swprintf(m_wsStatus3Msg, L"%s %.2f %s", m_wsStatus3Base, dVal[1],
                m_cFSR.cPaperUnit.wsAbbrev);
            }
            else
            {
              dVal[1] /= m_dDrawScale;
              dVal[1] /= m_cFSR.cLenUnit.dBaseToUnit;
              swprintf(m_wsStatus3Msg, L"%s %.2f %s", m_wsStatus3Base, dVal[1],
                m_cFSR.cLenUnit.wsAbbrev);
            }
            //SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus3Msg);
            SendMessage(m_hLab1, WM_SETTEXT, 0, (LPARAM)m_wsStatus3Msg);
          }
        }
      }

      cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
      cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
      cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
      cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

      m_pActiveObject->BuildPrimitives(cPtX, iDynMode, &cdr, 0, NULL, NULL);

      if(m_iToolMode == tolEditSpline)
      {
        if(m_iButton == 1)
        {
          if(m_pActiveObject->MoveSplinePoint(cPtX.cOrigin))
          {
            InvalidateRect(hwnd, NULL, FALSE);
          }
        }
        else
        {
          dTol = (double)m_iSelectTolerance/m_dUnitScale;

          if(m_pActiveObject->HighlightSplinePoint(cPtX, dTol) > 0)
          {
            if(GetCursor() == m_hPlusCur) SetCursor(LoadCursor(NULL, IDC_ARROW));
          }
          else
          {
            CDLine cSplinePt;
            if(fabs(m_pActiveObject->GetDistFromPt(cPtX.cOrigin, cPtX.cOrigin, 0, &cSplinePt, NULL)) < dTol)
            {
              if(GetCursor() != m_hPlusCur) SetCursor(m_hPlusCur);
            }
            else
            {
              if(GetCursor() == m_hPlusCur) SetCursor(LoadCursor(NULL, IDC_ARROW));
            }
          }
        }
      }

      DrawObject(hwnd, &graphics, m_pActiveObject, 1, -2);
    }
    else if(iDynMode == 4)
    {
      iCnt = m_pDrawObjects->GetSelectCount(2);
      if(iCnt == 1)
      {
        //SendMessage(g_hStatus, SB_SETTEXT, 2, (LPARAM)L"Dobry 1");
        pObj1 = m_pDrawObjects->GetSelected(0);
        CDFileAttrs cFAttrs;
        FilePropsToData(&cFAttrs);
        // we actualy don't need the drawing scale for the dimension,
        // so we will use it to pass the view scale
        cFAttrs.dScaleDenom = m_dUnitScale;

        cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
        cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
        cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
        cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

        pObj1->BuildPrimitives(cPtX, iDynMode, &cdr, 0, &cFAttrs, NULL);
        DrawObject(hwnd, &graphics, pObj1, 1, -1);
      }
    }

    if(m_iDrawMode == modRegRaster)
    {
      CDPoint cPt1, cPt2;
      Pen greenPen(Color(255, 0, 255, 0), 1.0f);
      for(int i = 0; i < m_iRegRasterCount/2; i++)
      {
        cPt1.x = m_cViewOrigin.x + m_dUnitScale*m_cRegRasterPoints[2*i].x;
        cPt1.y = m_cViewOrigin.y + m_dUnitScale*m_cRegRasterPoints[2*i].y;
        cPt2.x = m_cViewOrigin.x + m_dUnitScale*m_cRegRasterPoints[2*i + 1].x;
        cPt2.y = m_cViewOrigin.y + m_dUnitScale*m_cRegRasterPoints[2*i + 1].y;
        DrawRegRasterLine(&graphics, &greenPen, cPt1, cPt2);
      }
      if((m_iRegRasterCount % 2) > 0)
      {
        cPt1.x = m_cViewOrigin.x + m_dUnitScale*m_cRegRasterPoints[m_iRegRasterCount - 1].x;
        cPt1.y = m_cViewOrigin.y + m_dUnitScale*m_cRegRasterPoints[m_iRegRasterCount - 1].y;
        cPt2.x = m_cViewOrigin.x + m_dUnitScale*m_cLastDrawPt.x;
        cPt2.y = m_cViewOrigin.y + m_dUnitScale*m_cLastDrawPt.y;
        DrawRegRasterLine(&graphics, &greenPen, cPt1, cPt2);
      }
    }

    hdc = GetDC(hwnd);
    //IntersectClipRect(hdc, rc.left, rc.top, rc.right, rc.bottom);
    Graphics dstgraph(hdc);
    dstgraph.DrawImage(&bmp, 0, 0);
    ReleaseDC(hwnd, NULL);

    if(m_iToolMode == tolMeas)
    {
      if(m_cMeasPoint1.bIsSet && !m_cMeasPoint2.bIsSet)
      {
        CDPoint cDistPt = m_cLastDrawPt - m_cMeasPoint1.cOrigin;
        wchar_t *wsUnit;
        if(m_bPaperUnits)
        {
          cDistPt /= m_cFSR.cPaperUnit.dBaseToUnit;
          wsUnit = m_cFSR.cPaperUnit.wsAbbrev;
        }
        else
        {
          cDistPt /= m_dDrawScale;
          cDistPt /= m_cFSR.cLenUnit.dBaseToUnit;
          wsUnit = m_cFSR.cLenUnit.wsAbbrev;
        }
        double dNorm = GetNorm(cDistPt);
        swprintf(m_wsStatus2Msg, L"dx: %.3f, dy: %.3f, dist: %.4f (%s)", fabs(cDistPt.x),
          fabs(cDistPt.y), dNorm, wsUnit);
        SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);
      }
    }
    else if(m_iToolMode == tolMeasAngle)
    {
      if(m_cMeasPoint1.bIsSet && m_cMeasPoint2.bIsSet && !m_cMeasPoint3.bIsSet)
      {
        CDPoint cPt1 = m_cMeasPoint2.cOrigin - m_cMeasPoint1.cOrigin;
        CDPoint cPt2 = m_cLastDrawPt - m_cMeasPoint1.cOrigin;
        double d1 = GetNorm(cPt1);
        double d2 = GetNorm(cPt2);
        if((d1 > g_dPrec) && (d2 > g_dPrec))
        {
          double dx1 = (cPt1.x*cPt2.x + cPt1.y*cPt2.y)/d1/d2;
          double dy1 = (cPt2.y*cPt1.x - cPt1.y*cPt2.x)/d1/d2;
          double dAng = -180*atan2(dy1, dx1)/M_PI;
          wchar_t *wsUnit = m_cFSR.cAngUnit.wsAbbrev;
          swprintf(m_wsStatus2Msg, L"angle: %.4f (%s)", dAng*m_cFSR.cAngUnit.dBaseToUnit, wsUnit);
          SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);
        }
      }
    }
  }
  return 0;
}

LRESULT CMainWnd::WMMouseWheel(HWND hwnd, WORD fwKeys, int zDelta, int xPos, int yPos)
{
  if(m_iButton == 0)
  {
    double dRatio = exp((double)zDelta/600.0);

    POINT cPt = {xPos, yPos};
    MapWindowPoints(HWND_DESKTOP, hwnd, &cPt, 1);

    m_cViewOrigin.x = cPt.x + (m_cViewOrigin.x - cPt.x)*dRatio;
    m_cViewOrigin.y = cPt.y + (m_cViewOrigin.y - cPt.y)*dRatio;
    m_dUnitScale *= dRatio;

    RECT rc;
    GetClientRect(hwnd, &rc);
    rc.top += m_iToolBarHeight;
    rc.bottom -= m_iStatusHeight;
    InvalidateRect(hwnd, &rc, FALSE);
  }
  return 0;
}

LRESULT CMainWnd::WMLButtonDblClk(HWND hwnd, WPARAM fwKeys, int xPos, int yPos)
{
  //MessageBox(hwnd, L"WMLButtonDblClk", L"Debug", MB_OK);
  //if((m_iDrawMode > modSelect) || (m_iToolMode == tolCopyPar))
  if(m_iDrawMode > modSelect)
  {
    if(m_pActiveObject)
    {
      if(m_pActiveObject->HasEnoughPoints())
      {
        if(m_iToolMode == tolExtend)
        {
          m_iDrawMode = modSelect;
          m_iToolMode = tolNone;
          m_pActiveObject = NULL;
        }
        else
        {
          m_pDrawObjects->Add(m_pActiveObject);
          m_pActiveObject = NULL;
          StartNewObject(hwnd);
        }
        RECT rc;
        GetClientRect(hwnd, &rc);
        rc.top += m_iToolBarHeight;
        rc.bottom -= m_iStatusHeight;
        InvalidateRect(hwnd, &rc, FALSE);

        SetTitle(hwnd, false);
      }
    }
  }
  else if(m_iDrawMode + m_iToolMode < 1)
  {
    EditLineStyleCmd(hwnd, 0, 0);
  }
  return 0;
}

/*LRESULT CMainWnd::WMChar(HWND hwnd, wchar_t chCharCode, LPARAM lKeyData)
{
    float f;
    int iLen;
    if(chCharCode == VK_RETURN)
    {
        iLen = wcslen(m_wsStatus2Base);
        m_wsStatus2Msg[iLen] = 0;
        m_bRestrictSet = false;
    }
    else
    {
        int iLen = wcslen(m_wsStatus2Msg);
        if(iLen < 127)
        {
            m_wsStatus2Msg[iLen] = chCharCode;
            m_wsStatus2Msg[iLen + 1] = 0;
        }

        m_bRestrictSet = true;
        iLen = wcslen(m_wsStatus2Base);
        if(swscanf(&m_wsStatus2Msg[iLen], L"%f", &f) == 1) m_dRestrictValue = f;
        else m_bRestrictSet = false;
    }
    SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);
    WMMouseMove(hwnd, 0, m_cLastMovePt.x, m_cLastMovePt.y);
    return 0;
}

LRESULT CMainWnd::WMKeyDown(HWND hwnd, int nVirtKey, LPARAM lKeyData)
{
    float f;
    int iLen;
    bool bRedraw = false;

    if((nVirtKey == VK_DECIMAL) || (nVirtKey == VK_OEM_PERIOD))
    {
        iLen = wcslen(m_wsStatus2Msg);
        if(iLen < 127)
        {
            m_wsStatus2Msg[iLen] = '.';
            m_wsStatus2Msg[iLen + 1] = 0;
        }
        m_bRestrictSet = true;
        iLen = wcslen(m_wsStatus2Base);
        if(swscanf(&m_wsStatus2Msg[iLen], L"%f", &f) == 1) m_dRestrictValue = f;
        else m_bRestrictSet = false;
        bRedraw = true;
    }
    else if(nVirtKey == VK_BACK)
    {
        iLen = wcslen(m_wsStatus2Msg);
        if(iLen > wcslen(m_wsStatus2Base)) m_wsStatus2Msg[iLen - 1] = 0;

        m_bRestrictSet = true;
        iLen = wcslen(m_wsStatus2Base);
        if(swscanf(&m_wsStatus2Msg[iLen], L"%f", &f) == 1) m_dRestrictValue = f;
        else m_bRestrictSet = false;
        bRedraw = true;
    }
    if(bRedraw)
    {
        SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);
        WMMouseMove(hwnd, 0, m_cLastMovePt.x, m_cLastMovePt.y);
    }
    return 0;
}*/

void CMainWnd::StartNewObject(HWND hWnd)
{
    ShowWindow(m_hEdt1, SW_HIDE);
    ShowWindow(m_hEdt2, SW_HIDE);
    ShowWindow(m_hLab1, SW_HIDE);
    ShowWindow(m_hChB1, SW_HIDE);
    SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)L"");

    m_pActiveObject = NULL;
    m_iRestrictSet = -1;
    m_iRestrictSet2 = -1;

    m_cMeasPoint1.bIsSet = false;
    m_cMeasPoint2.bIsSet = false;
    m_cMeasPoint3.bIsSet = false;

    int iLines = m_pDrawObjects->GetNumOfSelectedLines();
    CDLine cLine1, cLine2;
    int iLinesFlag = 0;

    wchar_t sCap[32];
    wchar_t sMsg[128];
    PDObject pLineObj;

    m_wsStatus2Base[0] = 0;

    if(iLines > 0)
    {
        pLineObj = m_pDrawObjects->GetSelectedLine(0);
        cLine1 = pLineObj->GetLine();
        if(cLine1.bIsSet) iLinesFlag |= 1;
    }

    if(iLines > 1)
    {
        pLineObj = m_pDrawObjects->GetSelectedLine(1);
        cLine2 = pLineObj->GetLine();
        if(cLine2.bIsSet) iLinesFlag |= 2;
    }

    switch(m_iDrawMode)
    {
    case modLine:
        LoadString(m_hInstance, IDS_ANGLE, m_wsStatus2Base, 64);
        m_pActiveObject = new CDObject(dtLine, m_cFSR.dDefLineWidth);
        ShowWindow(m_hEdt1, SW_SHOW);
        SendMessage(m_hEdt1, EM_SETSEL, 0, (LPARAM)-1);
        SetFocus(m_hEdt1);
        break;
    case modCircle:
        LoadString(m_hInstance, IDS_RADIUS, m_wsStatus2Base, 64);
        m_pActiveObject = new CDObject(dtCircle, m_cFSR.dDefLineWidth);
        if(iLinesFlag & 1) m_pActiveObject->SetInputLine(0, cLine1);
        //if(iLinesFlag & 2) m_pActiveObject->SetInputLine(1, cLine2);
        ShowWindow(m_hEdt1, SW_SHOW);
        SendMessage(m_hEdt1, EM_SETSEL, 0, (LPARAM)-1);
        SetFocus(m_hEdt1);
        break;
    case modEllipse:
        m_pActiveObject = new CDObject(dtEllipse, m_cFSR.dDefLineWidth);
        if(iLines == 2)
        {
            if(iLinesFlag & 1) m_pActiveObject->SetInputLine(0, cLine1);
            if(iLinesFlag & 2) m_pActiveObject->SetInputLine(1, cLine2);
        }
        break;
    case modArcElps:
        if(iLines == 2)
        {
            m_pActiveObject = new CDObject(dtArcEllipse, m_cFSR.dDefLineWidth);
            if(iLinesFlag & 1) m_pActiveObject->SetInputLine(0, cLine1);
            if(iLinesFlag & 2) m_pActiveObject->SetInputLine(1, cLine2);
        }
        else
        {
            LoadString(m_hInstance, IDS_WARNING, sCap, 32);
            LoadString(m_hInstance, IDS_TWOLINESFORARCELLIPSE, sMsg, 128);
            MessageBox(hWnd, sMsg, sCap, MB_OK | MB_ICONWARNING);
        }
        break;
    case modHyperbola:
        if(iLines == 2)
        {
            m_pActiveObject = new CDObject(dtHyperbola, m_cFSR.dDefLineWidth);
            if(iLinesFlag & 1) m_pActiveObject->SetInputLine(0, cLine1);
            if(iLinesFlag & 2) m_pActiveObject->SetInputLine(1, cLine2);
        }
        else
        {
            LoadString(m_hInstance, IDS_WARNING, sCap, 32);
            LoadString(m_hInstance, IDS_TWOLINESFORHYPERBOLA, sMsg, 128);
            MessageBox(hWnd, sMsg, sCap, MB_OK | MB_ICONWARNING);
        }
        break;
    case modParabola:
        if(iLines == 1)
        {
            m_pActiveObject = new CDObject(dtParabola, m_cFSR.dDefLineWidth);
            if(iLinesFlag & 1) m_pActiveObject->SetInputLine(0, cLine1);
        }
        else
        {
            LoadString(m_hInstance, IDS_WARNING, sCap, 32);
            LoadString(m_hInstance, IDS_ONELINEFORPARABOLA, sMsg, 128);
            MessageBox(hWnd, sMsg, sCap, MB_OK | MB_ICONWARNING);
        }
        break;
    case modSpline:
        m_pActiveObject = new CDObject(dtSpline, m_cFSR.dDefLineWidth);
        break;
    case modEvolvent:
        iLines = m_pDrawObjects->GetNumOfSelectedCircles();
        if(iLines == 1)
        {
            pLineObj = m_pDrawObjects->GetSelectedCircle(0);
            cLine1 = pLineObj->GetCircle();
            if(cLine1.bIsSet) iLinesFlag |= 1;
            m_pActiveObject = new CDObject(dtEvolvent, m_cFSR.dDefLineWidth);
            if(iLinesFlag & 1) m_pActiveObject->SetInputLine(0, cLine1);
        }
        else
        {
            LoadString(m_hInstance, IDS_WARNING, sCap, 32);
            LoadString(m_hInstance, IDS_ONECIRCFOREVOLV, sMsg, 128);
            MessageBox(hWnd, sMsg, sCap, MB_OK | MB_ICONWARNING);
        }
        break;
    case modRectangle:
        LoadString(m_hInstance, IDS_WIDTH, m_wsStatus2Base, 64);
        LoadString(m_hInstance, IDS_HEIGHT, m_wsStatus3Base, 64);
        wcscpy(m_wsStatus3Msg, m_wsStatus3Base);
        //SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus3Msg);
        SendMessage(m_hLab1, WM_SETTEXT, 0, (LPARAM)m_wsStatus3Msg);
        m_pActiveObject = new CDObject(dtRect, m_cFSR.dDefLineWidth);
        ShowWindow(m_hEdt1, SW_SHOW);
        SendMessage(m_hEdt1, EM_SETSEL, 0, (LPARAM)-1);
        ShowWindow(m_hEdt2, SW_SHOW);
        SendMessage(m_hEdt2, EM_SETSEL, 0, (LPARAM)-1);
        ShowWindow(m_hLab1, SW_SHOW);
        SetFocus(m_hEdt1);
        break;
    }

    if(m_iToolMode == tolRound)
    {
        int iCnt = m_pDrawObjects->GetSelectCount(2);
        if(iCnt == 2)
        {
            m_pActiveObject = new CDObject(dtCircle, m_cFSR.dDefLineWidth);
            LoadString(m_hInstance, IDS_RADIUS, m_wsStatus2Base, 64);
            ShowWindow(m_hEdt1, SW_SHOW);
            SendMessage(m_hEdt1, EM_SETSEL, 0, (LPARAM)-1);
            SetFocus(m_hEdt1);
        }
        else
        {
            LoadString(m_hInstance, IDS_WARNING, sCap, 32);
            LoadString(m_hInstance, IDS_TWOOBJSFORROUND, sMsg, 128);
            MessageBox(hWnd, sMsg, sCap, MB_OK | MB_ICONWARNING);
        }
    }

    wcscpy(m_wsStatus2Msg, m_wsStatus2Base);
    SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)m_wsStatus2Msg);
}

void CMainWnd::GetDeviceToUnitScale(HWND hWnd)
{
    HDC hdc = GetDC(0);
    int iLogPizelsX = GetDeviceCaps(hdc, LOGPIXELSX);
    //int iLogPixelsY = GetDeviceCaps(hdc, LOGPIXELSY);
    ReleaseDC(0, NULL);

    m_iSelectTolerance = (int)1.0*iLogPizelsX/g_dMmToIn;
    m_iSnapTolerance = (int)2.0*iLogPizelsX/g_dMmToIn;

    m_dDeviceToUnitScale = (double)iLogPizelsX/g_dMmToIn;
}

LRESULT CMainWnd::Edit1Cmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
    if(wNotifyCode == EN_CHANGE)
    {
        wchar_t wBuf[64];
        SendMessage(hwndCtl, WM_GETTEXT, 64, (LPARAM)wBuf);

        int iOldRest = m_iRestrictSet;

        char sBuf[64];
        WideCharToMultiByte(CP_UTF8, 0, wBuf, -1, sBuf, 64, NULL, NULL);
        m_iRestrictSet = ParseInputString(sBuf, m_pFileSetupDlg->GetUnitList(), &m_dRestrictValue);

        WMMouseMove(hwnd, 0, m_cLastSnapPt.x, m_cLastSnapPt.y);

        if((m_iToolMode == tolMove) && !m_cMeasPoint1.bIsSet && (iOldRest != m_iRestrictSet))
        {
            if(IS_LENGTH_VAL(m_iRestrictSet))
                LoadString(m_hInstance, IDS_SELLINETOMOVE, m_wsStatus2Msg, 128);
            else LoadString(m_hInstance, IDS_SELPOINTFROMMOVE, m_wsStatus2Msg, 128);
            SendMessage(m_hStatus, SB_SETTEXT, 2, (LPARAM)m_wsStatus2Msg);
        }
    }
    return 0;
}

LRESULT CMainWnd::Edit2Cmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(wNotifyCode == EN_CHANGE)
  {
    wchar_t wBuf[64];
    SendMessage(hwndCtl, WM_GETTEXT, 64, (LPARAM)wBuf);

    char sBuf[64];
    WideCharToMultiByte(CP_UTF8, 0, wBuf, -1, sBuf, 64, NULL, NULL);
    m_iRestrictSet2 = ParseInputString(sBuf, m_pFileSetupDlg->GetUnitList(), &m_dRestrictValue2);

    WMMouseMove(hwnd, 0, m_cLastSnapPt.x, m_cLastSnapPt.y);
  }
  return 0;
}

void CMainWnd::SetTitle(HWND hWnd, bool bForce)
{
    bool bNewHasChanged = m_pDrawObjects->GetChanged();
    if((m_bHasChanged == bNewHasChanged) && !bForce) return;

    m_bHasChanged = bNewHasChanged;

    int iLen = wcslen(L"SteamCAD2 - ");
    LPWSTR wsFileName = NULL;
    if(m_wsFileName[0])
    {
        wsFileName = wcsrchr(m_wsFileName, '\\');
        if(wsFileName) wsFileName++;
        else wsFileName = m_wsFileName;
        iLen += wcslen(wsFileName);
    }
    else iLen += wcslen(L"new file");
    if(m_bHasChanged) iLen++;

    LPWSTR wsCap = (LPWSTR)malloc((iLen + 1)*sizeof(wchar_t));
    wcscpy(wsCap, L"SteamCAD2 - ");
    if(wsFileName) wcscat(wsCap, wsFileName);
    else wcscat(wsCap, L"new file");
    if(m_bHasChanged) wcscat(wsCap, L"*");

    SendMessage(hWnd, WM_SETTEXT, 0, (LPARAM)wsCap);
    free(wsCap);
}

/*LRESULT CMainWnd::ToolsBreakCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
    RECT rc;
    GetClientRect(hwnd, &rc);
    rc.top += m_iToolBarHeight;
    rc.bottom -= m_iStatusHeight;

    CDRect cdr;
    cdr.cPt1.x = (rc.left - m_cViewOrigin.x)/m_dUnitScale;
    cdr.cPt1.y = (rc.top - m_cViewOrigin.y)/m_dUnitScale;
    cdr.cPt2.x = (rc.right - m_cViewOrigin.x)/m_dUnitScale;
    cdr.cPt2.y = (rc.bottom - m_cViewOrigin.y)/m_dUnitScale;

    PDPtrList pRegions = new CDPtrList();
    pRegions->SetDblVal(m_dUnitScale);

    if(m_pDrawObjects->BreakSelObjects(&cdr, pRegions));
    {
        HRGN hRgn = GetUpdateRegion(pRegions);
        //InvalidateRect(hwnd, NULL, true);
        if(hRgn)
        {
            InvalidateRgn(hwnd, hRgn, TRUE);
            DeleteObject(hRgn);
        }
        SetTitle(hwnd, false);
    }

    ClearPolygonList(pRegions);
    delete pRegions;
    return 0;
}*/

LRESULT CMainWnd::ToolsScaleCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
    CDFileAttrs cFA;
    m_pDrawObjects->GetFileAttrs(&cFA);

    CDScaleRec cSR;
    cSR.bScaleDraw = false;
    cSR.dScaleNom = cFA.dScaleNom;
    cSR.dScaleDenom = cFA.dScaleDenom;
    cSR.bScaleWidth = true;
    cSR.bScalePattern = true;
    cSR.bScaleArrows = true;
    cSR.bScaleLabels = true;
    cSR.bChangeDimUnits = false;

    char sMaskBuf[64];

    int iRes = m_pDrawObjects->GetUnitMask(1, sMaskBuf, m_pFileSetupDlg->GetUnitList());
    if(iRes < 0) wcscpy(cSR.wsLenMask, m_cFSR.wsLengthMask);
    else MultiByteToWideChar(CP_UTF8, 0, sMaskBuf, -1, cSR.wsLenMask, 64);

    iRes = m_pDrawObjects->GetUnitMask(2, sMaskBuf, m_pFileSetupDlg->GetUnitList());
    if(iRes < 0) wcscpy(cSR.wsAngMask, m_cFSR.wsAngleMask);
    else MultiByteToWideChar(CP_UTF8, 0, sMaskBuf, -1, cSR.wsAngMask, 64);

    if(m_pScaleDlg->ShowDialog(hwnd, &cSR) == IDOK)
    {
        bool bRedraw = false;
        if(cSR.bScaleDraw)
        {
            bRedraw = true;
            m_pDrawObjects->RescaleDrawing(cSR.dScaleNom, cSR.dScaleDenom,
                cSR.bScaleWidth, cSR.bScalePattern, cSR.bScaleArrows, cSR.bScaleLabels);
            m_pUndoObjects->ClearAll();
            DataToFileProps();
        }
        if(cSR.bChangeDimUnits)
        {
            bRedraw = true;
            WideCharToMultiByte(CP_UTF8, 0, cSR.wsLenMask, -1, sMaskBuf, 64, NULL, NULL);
            m_pDrawObjects->ChangeUnitMask(1, sMaskBuf, m_pFileSetupDlg->GetUnitList());
            WideCharToMultiByte(CP_UTF8, 0, cSR.wsAngMask, -1, sMaskBuf, 64, NULL, NULL);
            m_pDrawObjects->ChangeUnitMask(2, sMaskBuf, m_pFileSetupDlg->GetUnitList());
        }
        if(bRedraw) InvalidateRect(hwnd, NULL, FALSE);
    }
    return 0;
}

LRESULT CMainWnd::ToolsStatCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  int iStats[9];
  for(int i = 0; i < 9; i++) iStats[i] = 0;

  m_pDrawObjects->GetStatistics(iStats);

  CDStatRec cSR;
  cSR.iLines = iStats[1];
  cSR.iCircles = iStats[2];
  cSR.iEllips = iStats[3];
  cSR.iArcEllips = iStats[4];
  cSR.iHypers = iStats[5];
  cSR.iParabs = iStats[6];
  cSR.iSplines = iStats[7];
  cSR.iEvolvs = iStats[8];
  cSR.iDimens = iStats[0];

  m_pStatDlg->ShowDialog(hwnd, &cSR);
  return 0;
}

LRESULT CMainWnd::PathCreateCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pDrawObjects->CreatePath() > 0)
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

LRESULT CMainWnd::PathBreakCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pDrawObjects->BreakSelObjects())
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

LRESULT CMainWnd::PathAreaCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pDrawObjects->CreateArea())
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

LRESULT CMainWnd::PathGroupCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pDrawObjects->Group())
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

LRESULT CMainWnd::PathUngroupCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pDrawObjects->Ungroup())
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

LRESULT CMainWnd::PathMoveUpCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pDrawObjects->MoveUp())
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

LRESULT CMainWnd::PathMoveDownCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pDrawObjects->MoveDown())
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

LRESULT CMainWnd::PathMoveTopCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pDrawObjects->MoveTop())
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

LRESULT CMainWnd::PathMoveBottomCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  if(m_pDrawObjects->MoveBottom())
  {
    InvalidateRect(hwnd, NULL, FALSE);
    SetTitle(hwnd, false);
  }
  return 0;
}

LRESULT CMainWnd::RasterImportCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  IFileOpenDialog *pFileOpen = NULL;
  HRESULT hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_INPROC_SERVER, IID_IFileOpenDialog, (void**)&pFileOpen);
  if(SUCCEEDED(hr))
  {
    wchar_t wsFilter[128];
    LoadString(m_hInstance, IDS_RASTERFORMATFILTER, wsFilter, 128);

    int iFilterLen = 0;
    int n = wcslen(wsFilter);
    for(int i = 0; i < n; i++)
    {
      if(wsFilter[i] == 1) iFilterLen++;
    }
    iFilterLen /= 2;
    COMDLG_FILTERSPEC *pFilter = NULL;
    if(iFilterLen > 0)
    {
      pFilter = (COMDLG_FILTERSPEC*)malloc(iFilterLen*sizeof(COMDLG_FILTERSPEC));
      int iFilterPos = 0;
      pFilter[iFilterPos++].pszName = wsFilter;
      for(int i = 0; i < n; i++)
      {
        if(wsFilter[i] == 1)
        {
          wsFilter[i] = 0;
          if(iFilterPos < 2*iFilterLen)
          {
            if(iFilterPos % 2 > 0) pFilter[iFilterPos/2].pszSpec = &wsFilter[i + 1];
            else pFilter[iFilterPos/2].pszName = &wsFilter[i + 1];
            iFilterPos++;
          }
        }
      }
      pFileOpen->SetFileTypes(iFilterLen, pFilter);
    }
    hr = pFileOpen->Show(NULL);
    if(SUCCEEDED(hr))
    {
      IShellItem *pItem;
      hr = pFileOpen->GetResult(&pItem);
      if(SUCCEEDED(hr))
      {
        PWSTR pszFilePath;
        hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);
        if(SUCCEEDED(hr))
        {
          Gdiplus::Image *image = Gdiplus::Image::FromFile(pszFilePath);
          SizeF cSize;
          image->GetPhysicalDimension(&cSize);

          int iw = image->GetWidth();
          int ih = image->GetHeight();
          if((iw > 0) && (ih > 0))
          {
            double dOff = 0.1*m_dwPage;
            if(m_dhPage < m_dwPage) dOff = 0.1*m_dhPage;
            double dImageWidth = m_dwPage - 2.0*dOff;
            double dImageHeight = dImageWidth*(double)ih/(double)iw;
            if(dImageHeight > m_dhPage - 2.0*dOff)
            {
              dImageHeight = m_dhPage - 2.0*dOff;
              dImageWidth = dImageHeight*(double)iw/(double)ih;
            }

            PDObject pImage = new CDObject(dtRaster, m_cFSR.dDefLineWidth);
            pImage->AddPoint(0.0, 0.0, 0, 0.0);
            pImage->AddPoint(dOff, dOff, 1, 0.0);
            pImage->AddPoint((double)iw, 0.0, 0, 0.0);
            pImage->AddPoint(dOff + dImageWidth, dOff, 1, 0.0);
            pImage->AddPoint(0.0, (double)ih, 0, 0.0);
            pImage->AddPoint(dOff, dOff + dImageHeight, 1, 0.0);

            FILE *pf = _wfopen(pszFilePath, L"rb");
            pImage->BuildRasterCache(iw, ih, pf);
            fclose(pf);

            m_pDrawObjects->Add(pImage);

            RECT rc;
            GetClientRect(hwnd, &rc);
            rc.top += m_iToolBarHeight;
            rc.bottom -= m_iStatusHeight;

            InvalidateRect(hwnd, &rc, FALSE);
            SetTitle(hwnd, true);
          }
          CoTaskMemFree(pszFilePath);
        }
        pItem->Release();
      }
    }
    if(pFilter) free(pFilter);
    pFileOpen->Release();
  }
  return 0;
}

LRESULT CMainWnd::RasterRegisterCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  PDObject pImage = NULL;
  int iSel = m_pDrawObjects->GetSelectCount(2);
  if(iSel == 1)
  {
    pImage = m_pDrawObjects->GetSelected(0);
  }
  wchar_t sMessage[128];
  if(!pImage || (pImage->GetType() != dtRaster))
  {
    wchar_t sCaption[64];
    LoadString(m_hInstance, IDS_WARNING, sCaption, 64);
    LoadString(m_hInstance, IDS_ONERASTERFORREG, sMessage, 128);
    MessageBox(hwnd, sMessage, sCaption, MB_OK);
    return 0;
  }
  m_iRegRasterCount = 0;
  LoadString(m_hInstance, IDS_REGFIRSTLINE, sMessage, 64);
  SendMessage(m_hStatus, SB_SETTEXT, 1, (LPARAM)sMessage);
  m_iDrawMode = modRegRaster;
  return 0;
}

LRESULT CMainWnd::RasterHideCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  PDObject pImage = NULL;
  int iSel = m_pDrawObjects->GetSelectCount(2);
  if(iSel == 1)
  {
    pImage = m_pDrawObjects->GetSelected(0);
  }
  if(!pImage || (pImage->GetType() != dtRaster)) return 0;

  pImage->SetAuxInt(1);

  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  InvalidateRect(hwnd, &rc, FALSE);
  SetTitle(hwnd, true);
  return 0;
}

LRESULT CMainWnd::RasterShowCmd(HWND hwnd, WORD wNotifyCode, HWND hwndCtl)
{
  PDObject pImage = NULL;
  int iSel = m_pDrawObjects->GetSelectCount(2);
  if(iSel == 1)
  {
    pImage = m_pDrawObjects->GetSelected(0);
  }
  if(!pImage || (pImage->GetType() != dtRaster)) return 0;

  pImage->SetAuxInt(0);

  RECT rc;
  GetClientRect(hwnd, &rc);
  rc.top += m_iToolBarHeight;
  rc.bottom -= m_iStatusHeight;

  InvalidateRect(hwnd, &rc, FALSE);
  SetTitle(hwnd, true);
  return 0;
}
