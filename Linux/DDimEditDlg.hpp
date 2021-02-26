#ifndef _DDIMEDITDLG_HPP_
#define _DDIMEDITDLG_HPP_

#include <gtk/gtk.h>

#include "../Source/DDataTypes.hpp"
#include "../Source/DParser.hpp"
#include "DFileSetupDlg.hpp"

typedef class CDDimEditDlg
{
private:
  gint m_iX;
  gint m_iY;
  gboolean m_bSettingUp;
  double m_dCurFontSize;
  char m_bCurFntAttrs;
  gchar m_sCurFntName[64];
  double m_dCurLeftLen;
  double m_dCurLeftWidth;
  double m_dCurRightLen;
  double m_dCurRightWidth;
  PDDimension m_pDimen;
  PDUnitList m_pUnits;
  PDFileUnit m_pUnit;

  GtkWidget *m_pDlg;
  GtkWidget *m_pLeftArrowCB;
  GtkWidget *m_pRightArrowCB;
  GtkWidget *m_pLeftLenEdt;
  GtkWidget *m_pRightLenEdt;
  GtkWidget *m_pLeftWidthEdt;
  GtkWidget *m_pRightWidthEdt;
  GtkWidget *m_pFntFaceCB;
  GtkWidget *m_pBoldChB;
  GtkWidget *m_pItalicChB;
  GtkWidget *m_pUnderChB;
  GtkWidget *m_pFntSizeEdt;
  GtkWidget *m_pTextEdt;
  GtkWidget *m_pFntSampleLab;
public:
  CDDimEditDlg();
  ~CDDimEditDlg();
  bool ShowDialog(GtkWidget *pWndParent, PDDimension pDimen, PDUnitList pUnits, PDFileUnit pUnit);
  void SaveSettings(FILE *fp);
  void RestoreSettings(gint iLeft, gint iTop);
  gboolean Configure(GtkWidget *widget, GdkEvent *event);

  void OKBtnClick(GtkButton *button);
  void GridEditChanged(GtkEntry *entry, gint id);
  void FontAttrChBChange(GtkToggleButton *togglebutton, gint iMask);
  void FontSet(GtkFontButton *widget);
} *PDDimEditDlg;

#endif

