#ifndef _DSCALEDLG_HPP_
#define _DSCALEDLG_HPP_

#include <gtk/gtk.h>

typedef struct CDScaleRec
{
  gboolean bScaleDraw;
  double dScaleNom;
  double dScaleDenom;
  gboolean bScaleWidth;
  gboolean bScalePattern;
  gboolean bScaleArrows;
  gboolean bScaleLabels;
  gboolean bChangeDimUnits;
  gchar sLenMask[64];
  gchar sAngMask[64];
} *PDScaleRec;

typedef class CDScaleDlg
{
private:
  gint m_iX;
  gint m_iY;
  gboolean m_bSettingUp;
  PDScaleRec m_pSR;

  GtkWidget *m_pDlg;
  GtkWidget *m_pScaleDrawChB;
  GtkWidget *m_pNewNomEdt;
  GtkWidget *m_pNewDenomEdt;
  GtkWidget *m_pScaleWidthChB;
  GtkWidget *m_pScalePatChB;
  GtkWidget *m_pScaleArrowChB;
  GtkWidget *m_pScaleLabChB;
  GtkWidget *m_pChangeDimChB;
  GtkWidget *m_pLenMaskEdt;
  GtkWidget *m_pAngMaskEdt;
public:
  CDScaleDlg();
  ~CDScaleDlg();
  gboolean ShowDialog(GtkWidget *pWndParent, PDScaleRec pSR);
  void SaveSettings(FILE *fp);
  void RestoreSettings(gint iLeft, gint iTop);
  gboolean Configure(GtkWidget *widget, GdkEvent *event);
  void OKBtnClick(GtkButton *button);
  void ScaleDrawChange(GtkToggleButton *togglebutton);
  void ChangeUnitsChange(GtkToggleButton *togglebutton);
} *PDScaleDlg;

#endif
