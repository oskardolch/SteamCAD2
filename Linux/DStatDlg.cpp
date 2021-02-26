#include "DStatDlg.hpp"
#include "DLxBaseDefs.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

gboolean sd_configure(GtkWidget *widget, GdkEvent *event, PDStatDlg pApp)
{
  return pApp->Configure(widget, event);
}


// CDStatDlg

CDStatDlg::CDStatDlg()
{
  m_iX = -100;
  m_iY = -100;
  m_bSettingUp = FALSE;
}

CDStatDlg::~CDStatDlg()
{
}

gboolean CDStatDlg::ShowDialog(GtkWidget *pWndParent, PDStatRec pSR)
{
  m_bSettingUp = TRUE;

  GtkWidget *pDlg = gtk_dialog_new_with_buttons(_("Statistics"), GTK_WINDOW(pWndParent),
    GTK_DIALOG_MODAL, GTK_STOCK_OK, GTK_RESPONSE_OK, NULL);
  g_signal_connect(G_OBJECT(pDlg), "configure-event", G_CALLBACK(sd_configure), this);

  GtkWidget *pCA = gtk_dialog_get_content_area(GTK_DIALOG(pDlg));

  GtkWidget *pTbl = gtk_table_new(10, 2, FALSE);
  gtk_table_set_col_spacings(GTK_TABLE(pTbl), 3);
  gtk_table_set_row_spacings(GTK_TABLE(pTbl), 3);
  gtk_table_set_row_spacing(GTK_TABLE(pTbl), 0, 10);
  gtk_container_set_border_width(GTK_CONTAINER(pTbl), 4);
  gtk_container_add(GTK_CONTAINER(pCA), pTbl);
  gtk_widget_show(pTbl);

  GtkWidget *pLab = gtk_label_new(_("Total segments:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 0, 1);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Lines:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 1, 2);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Circles:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 2, 3);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Ellipses:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 3, 4);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Arc Ellipses:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 4, 5);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Hyperbolas:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 5, 6);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Parabolas:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 6, 7);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Splines:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 7, 8);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Evolvents:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 8, 9);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Dimensions:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 9, 10);
  gtk_widget_show(pLab);

  int iTotal = pSR->iLines + pSR->iCircles + pSR->iEllips + pSR->iArcEllips +
    pSR->iHypers + pSR->iParabs + pSR->iSplines + pSR->iEvolvs; // + pSR->iDimens;
  gchar buf[32];

  sprintf(buf, "%d", iTotal);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 0, 1);
  gtk_widget_show(pLab);

  sprintf(buf, "%d", pSR->iLines);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 1, 2);
  gtk_widget_show(pLab);

  sprintf(buf, "%d", pSR->iCircles);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 2, 3);
  gtk_widget_show(pLab);

  sprintf(buf, "%d", pSR->iEllips);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 3, 4);
  gtk_widget_show(pLab);

  sprintf(buf, "%d", pSR->iArcEllips);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 4, 5);
  gtk_widget_show(pLab);

  sprintf(buf, "%d", pSR->iHypers);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 5, 6);
  gtk_widget_show(pLab);

  sprintf(buf, "%d", pSR->iParabs);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 6, 7);
  gtk_widget_show(pLab);

  sprintf(buf, "%d", pSR->iSplines);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 7, 8);
  gtk_widget_show(pLab);

  sprintf(buf, "%d", pSR->iEvolvs);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 8, 9);
  gtk_widget_show(pLab);

  sprintf(buf, "%d", pSR->iDimens);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 9, 10);
  gtk_widget_show(pLab);

  gtk_window_set_resizable(GTK_WINDOW(pDlg), FALSE);
  if(m_iX > -100) gtk_window_move(GTK_WINDOW(pDlg), m_iX, m_iY);

  m_bSettingUp = FALSE;

  gboolean bRes = (gtk_dialog_run(GTK_DIALOG(pDlg)) == GTK_RESPONSE_OK);

  if(m_iX < -90) gdk_window_get_position(pDlg->window, &m_iX, &m_iY);

  gtk_widget_destroy(pDlg);

  return bRes;
}

void CDStatDlg::SaveSettings(FILE *fp)
{
  if(m_iX < -90) return;

  gchar sbuf[256];
  sprintf(sbuf, "  <StatDlg Left=\"%d\" Top=\"%d\"/>\n", m_iX, m_iY);
  fwrite(sbuf, sizeof(gchar), strlen(sbuf), fp);
  return;
}

void CDStatDlg::RestoreSettings(gint iLeft, gint iTop)
{
  m_iX = iLeft;
  m_iY = iTop;
  return;
}

gboolean CDStatDlg::Configure(GtkWidget *widget, GdkEvent *event)
{
  if(m_bSettingUp) return FALSE;

  m_iX = event->configure.x;
  m_iY = event->configure.y;
  return FALSE;
}

