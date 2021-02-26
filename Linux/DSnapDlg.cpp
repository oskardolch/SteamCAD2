#include "DSnapDlg.hpp"
#include "DLxBaseDefs.h"

#include <string.h>

gboolean snd_configure(GtkWidget *widget, GdkEvent *event, PDSnapDlg pApp)
{
  return pApp->Configure(widget, event);
}


// CDSnapDlg

CDSnapDlg::CDSnapDlg()
{
  m_iX = -100;
  m_iY = -100;
  m_bSettingUp = FALSE;
}

CDSnapDlg::~CDSnapDlg()
{
}

gboolean CDSnapDlg::ShowDialog(GtkWidget *pWndParent, gboolean *pbEnableSnap)
{
  m_bSettingUp = TRUE;

  GtkWidget *pDlg = gtk_dialog_new_with_buttons(_("Line Style"), GTK_WINDOW(pWndParent),
    GTK_DIALOG_MODAL, GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL, GTK_STOCK_OK, GTK_RESPONSE_OK, NULL);
  g_signal_connect(G_OBJECT(pDlg), "configure-event", G_CALLBACK(snd_configure), this);

  GtkWidget *pCA = gtk_dialog_get_content_area(GTK_DIALOG(pDlg));

  GtkWidget *pTbl = gtk_table_new(2, 1, FALSE);
  gtk_table_set_col_spacings(GTK_TABLE(pTbl), 3);
  gtk_table_set_row_spacings(GTK_TABLE(pTbl), 3);
  gtk_container_set_border_width(GTK_CONTAINER(pTbl), 4);
  gtk_container_add(GTK_CONTAINER(pCA), pTbl);
  gtk_widget_show(pTbl);

  GtkWidget *pEnableRB = gtk_radio_button_new_with_label(NULL, _("Enable snap"));
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pEnableRB, 0, 1, 0, 1);
  gtk_widget_show(pEnableRB);

  GtkWidget *pDisableRB = gtk_radio_button_new_with_label_from_widget(GTK_RADIO_BUTTON(pEnableRB), _("Disable snap"));
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pDisableRB, 0, 1, 1, 2);
  gtk_widget_show(pDisableRB);

  if(*pbEnableSnap) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(pDisableRB), TRUE);
  else gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(pEnableRB), TRUE);

  gtk_window_set_resizable(GTK_WINDOW(pDlg), FALSE);
  if(m_iX > -100) gtk_window_move(GTK_WINDOW(pDlg), m_iX, m_iY);

  m_bSettingUp = FALSE;

  gboolean bRes = (gtk_dialog_run(GTK_DIALOG(pDlg)) == GTK_RESPONSE_OK);

  if(m_iX < -90) gdk_window_get_position(pDlg->window, &m_iX, &m_iY);
  if(bRes)
    *pbEnableSnap = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(pEnableRB));

  gtk_widget_destroy(pDlg);

  return bRes;
}

void CDSnapDlg::SaveSettings(FILE *fp)
{
  if(m_iX < -90) return;

  gchar sbuf[256];
  sprintf(sbuf, "  <SnapDlg Left=\"%d\" Top=\"%d\"/>\n", m_iX, m_iY);
  fwrite(sbuf, sizeof(gchar), strlen(sbuf), fp);
  return;
}

void CDSnapDlg::RestoreSettings(gint iLeft, gint iTop)
{
  m_iX = iLeft;
  m_iY = iTop;
  return;
}

gboolean CDSnapDlg::Configure(GtkWidget *widget, GdkEvent *event)
{
  if(m_bSettingUp) return FALSE;

  m_iX = event->configure.x;
  m_iY = event->configure.y;
  return FALSE;
}

