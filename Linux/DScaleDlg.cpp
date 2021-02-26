#include "DScaleDlg.hpp"
#include "DFileSetupDlg.hpp"
#include "../Source/DMath.hpp"
#include "DLxBaseDefs.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

gboolean sud_configure(GtkWidget *widget, GdkEvent *event, PDScaleDlg pApp)
{
  return pApp->Configure(widget, event);
}

void sud_okbtn_clicked(GtkButton *button, PDScaleDlg pApp)
{
  pApp->OKBtnClick(button);
}

void sud_scale_toggled(GtkToggleButton *togglebutton, PDScaleDlg pApp)
{
  pApp->ScaleDrawChange(togglebutton);
}

void sud_units_toggled(GtkToggleButton *togglebutton, PDScaleDlg pApp)
{
  pApp->ChangeUnitsChange(togglebutton);
}


// CDScaleDlg

CDScaleDlg::CDScaleDlg()
{
  m_iX = -100;
  m_iY = -100;
  m_bSettingUp = FALSE;
}

CDScaleDlg::~CDScaleDlg()
{
}

gboolean CDScaleDlg::ShowDialog(GtkWidget *pWndParent, PDScaleRec pSR)
{
  m_pSR = pSR;

  m_bSettingUp = TRUE;

  m_pDlg = gtk_dialog_new_with_buttons(_("Scale and Units"), GTK_WINDOW(pWndParent),
    GTK_DIALOG_MODAL, GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL, NULL);
  g_signal_connect(G_OBJECT(m_pDlg), "configure-event", G_CALLBACK(sud_configure), this);

  GtkWidget *pAA = gtk_dialog_get_action_area(GTK_DIALOG(m_pDlg));
  GtkWidget *pBtn = gtk_button_new_from_stock("gtk-ok");
  gtk_box_pack_end(GTK_BOX(pAA), pBtn, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(pBtn), "clicked", G_CALLBACK(sud_okbtn_clicked), this);
  gtk_widget_set_can_default(pBtn, TRUE);
  gtk_widget_show(pBtn);

  GtkWidget *pCA = gtk_dialog_get_content_area(GTK_DIALOG(m_pDlg));

  GtkWidget *pBox1 = gtk_hbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(pCA), pBox1, TRUE, TRUE, 0);
  gtk_widget_show(pBox1);

  GtkWidget *pFrm = gtk_frame_new(_("Drawing scale"));
  gtk_box_pack_start(GTK_BOX(pBox1), pFrm, TRUE, TRUE, 0);
  gtk_widget_show(pFrm);

  GtkWidget *pTbl = gtk_table_new(7, 4, FALSE);
  gtk_table_set_col_spacings(GTK_TABLE(pTbl), 3);
  gtk_table_set_row_spacings(GTK_TABLE(pTbl), 3);
  gtk_container_set_border_width(GTK_CONTAINER(pFrm), 2);
  gtk_container_set_border_width(GTK_CONTAINER(pTbl), 4);
  gtk_container_add(GTK_CONTAINER(pFrm), pTbl);
  gtk_widget_show(pTbl);

  m_pScaleDrawChB = gtk_check_button_new_with_label(_("Scale drawing"));
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pScaleDrawChB, 0, 4, 0, 1);
  g_signal_connect(G_OBJECT(m_pScaleDrawChB), "toggled", G_CALLBACK(sud_scale_toggled), this);
  gtk_widget_show(m_pScaleDrawChB);

  GtkWidget *pLab = gtk_label_new(_("Old scale:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 1, 2);
  gtk_widget_show(pLab);

  gchar buf[32];
  FormatFloatStr(m_pSR->dScaleNom, buf);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 1, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 1, 2, 1, 2);
  gtk_widget_show(pLab);

  m_pNewNomEdt = gtk_entry_new();
  gtk_widget_set_size_request(m_pNewNomEdt, 40, -1);
  gtk_entry_set_max_length(GTK_ENTRY(m_pNewNomEdt), 32);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pNewNomEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pNewNomEdt, 1, 2, 2, 3);
  gtk_widget_show(m_pNewNomEdt);
  gtk_entry_set_text(GTK_ENTRY(m_pNewNomEdt), buf);

  pLab = gtk_label_new(":");
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 2, 3, 1, 2);
  gtk_widget_show(pLab);

  FormatFloatStr(m_pSR->dScaleDenom, buf);
  pLab = gtk_label_new(buf);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 3, 4, 1, 2);
  gtk_widget_show(pLab);

  m_pNewDenomEdt = gtk_entry_new();
  gtk_widget_set_size_request(m_pNewDenomEdt, 40, -1);
  gtk_entry_set_max_length(GTK_ENTRY(m_pNewDenomEdt), 32);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pNewDenomEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pNewDenomEdt, 3, 4, 2, 3);
  gtk_widget_show(m_pNewDenomEdt);
  gtk_entry_set_text(GTK_ENTRY(m_pNewDenomEdt), buf);

  pLab = gtk_label_new(_("New scale:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 2, 3);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(":");
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 2, 3, 2, 3);
  gtk_widget_show(pLab);

  m_pScaleWidthChB = gtk_check_button_new_with_label(_("Scale line widths"));
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pScaleWidthChB, 0, 4, 3, 4);
  gtk_widget_show(m_pScaleWidthChB);

  m_pScalePatChB = gtk_check_button_new_with_label(_("Scale line patterns"));
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pScalePatChB, 0, 4, 4, 5);
  gtk_widget_show(m_pScalePatChB);

  m_pScaleArrowChB = gtk_check_button_new_with_label(_("Scale dimension arrows"));
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pScaleArrowChB, 0, 4, 5, 6);
  gtk_widget_show(m_pScaleArrowChB);

  m_pScaleLabChB = gtk_check_button_new_with_label(_("Scale labels"));
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pScaleLabChB, 0, 4, 6, 7);
  gtk_widget_show(m_pScaleLabChB);

  pFrm = gtk_frame_new(_("Display units"));
  gtk_box_pack_start(GTK_BOX(pBox1), pFrm, TRUE, TRUE, 0);
  gtk_widget_show(pFrm);

  pTbl = gtk_table_new(5, 1, FALSE);
  gtk_table_set_col_spacings(GTK_TABLE(pTbl), 3);
  gtk_table_set_row_spacings(GTK_TABLE(pTbl), 3);
  gtk_container_set_border_width(GTK_CONTAINER(pFrm), 2);
  gtk_container_set_border_width(GTK_CONTAINER(pTbl), 4);
  gtk_container_add(GTK_CONTAINER(pFrm), pTbl);
  gtk_widget_show(pTbl);

  m_pChangeDimChB = gtk_check_button_new_with_label(_("Change dimensions"));
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pChangeDimChB, 0, 1, 0, 1);
  g_signal_connect(G_OBJECT(m_pChangeDimChB), "toggled", G_CALLBACK(sud_units_toggled), this);
  gtk_widget_show(m_pChangeDimChB);

  pLab = gtk_label_new("New length mask");
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 1, 2);
  gtk_widget_show(pLab);

  m_pLenMaskEdt = gtk_entry_new();
  gtk_entry_set_max_length(GTK_ENTRY(m_pLenMaskEdt), 64);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pLenMaskEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pLenMaskEdt, 0, 1, 2, 3);
  gtk_widget_show(m_pLenMaskEdt);
  gtk_entry_set_text(GTK_ENTRY(m_pLenMaskEdt), m_pSR->sLenMask);

  pLab = gtk_label_new("New angular mask");
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 3, 4);
  gtk_widget_show(pLab);

  m_pAngMaskEdt = gtk_entry_new();
  gtk_entry_set_max_length(GTK_ENTRY(m_pAngMaskEdt), 64);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pAngMaskEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pAngMaskEdt, 0, 1, 4, 5);
  gtk_widget_show(m_pAngMaskEdt);
  gtk_entry_set_text(GTK_ENTRY(m_pAngMaskEdt), m_pSR->sAngMask);

  gtk_widget_grab_default(pBtn);
  gtk_window_set_default(GTK_WINDOW(m_pDlg), pBtn);

  gtk_window_set_resizable(GTK_WINDOW(m_pDlg), FALSE);
  if(m_iX > -100) gtk_window_move(GTK_WINDOW(m_pDlg), m_iX, m_iY);

  m_bSettingUp = FALSE;

  ScaleDrawChange(GTK_TOGGLE_BUTTON(m_pScaleDrawChB));
  ChangeUnitsChange(GTK_TOGGLE_BUTTON(m_pChangeDimChB));

  gboolean bRes = (gtk_dialog_run(GTK_DIALOG(m_pDlg)) == GTK_RESPONSE_OK);

  if(m_iX < -90) gdk_window_get_position(m_pDlg->window, &m_iX, &m_iY);

  gtk_widget_destroy(m_pDlg);

  return bRes;
}

void CDScaleDlg::SaveSettings(FILE *fp)
{
  if(m_iX < -90) return;

  gchar sbuf[256];
  sprintf(sbuf, "  <ScaleDlg Left=\"%d\" Top=\"%d\"/>\n", m_iX, m_iY);
  fwrite(sbuf, sizeof(gchar), strlen(sbuf), fp);
  return;
}

void CDScaleDlg::RestoreSettings(gint iLeft, gint iTop)
{
  m_iX = iLeft;
  m_iY = iTop;
  return;
}

gboolean CDScaleDlg::Configure(GtkWidget *widget, GdkEvent *event)
{
  if(m_bSettingUp) return FALSE;

  m_iX = event->configure.x;
  m_iY = event->configure.y;
  return FALSE;
}

void CDScaleDlg::OKBtnClick(GtkButton *button)
{
  float f;
  GtkWidget *msg_dlg;

  m_pSR->bScaleDraw = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_pScaleDrawChB));

  if(m_pSR->bScaleDraw)
  {
    if((sscanf(gtk_entry_get_text(GTK_ENTRY(m_pNewNomEdt)), "%f", &f) != 1) || (f < g_dPrec))
    {
      msg_dlg = gtk_message_dialog_new(GTK_WINDOW(m_pDlg), GTK_DIALOG_MODAL,
        GTK_MESSAGE_WARNING, GTK_BUTTONS_OK, _("Invalid number"));
      gtk_dialog_run(GTK_DIALOG(msg_dlg));
      gtk_widget_destroy(msg_dlg);
      gtk_widget_grab_focus(m_pNewNomEdt);
      return;
    }

    m_pSR->dScaleNom = f;

    if((sscanf(gtk_entry_get_text(GTK_ENTRY(m_pNewDenomEdt)), "%f", &f) != 1) || (f < g_dPrec))
    {
      msg_dlg = gtk_message_dialog_new(GTK_WINDOW(m_pDlg), GTK_DIALOG_MODAL,
        GTK_MESSAGE_WARNING, GTK_BUTTONS_OK, _("Invalid number"));
      gtk_dialog_run(GTK_DIALOG(msg_dlg));
      gtk_widget_destroy(msg_dlg);
      gtk_widget_grab_focus(m_pNewDenomEdt);
      return;
    }

    m_pSR->dScaleDenom = f;
  }

  m_pSR->bScaleWidth = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_pScaleWidthChB));
  m_pSR->bScalePattern = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_pScalePatChB));
  m_pSR->bScaleArrows = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_pScaleArrowChB));
  m_pSR->bScaleLabels = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_pScaleLabChB));

  m_pSR->bChangeDimUnits = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_pChangeDimChB));
  strcpy(m_pSR->sLenMask, gtk_entry_get_text(GTK_ENTRY(m_pLenMaskEdt)));
  strcpy(m_pSR->sAngMask, gtk_entry_get_text(GTK_ENTRY(m_pAngMaskEdt)));

  gtk_dialog_response(GTK_DIALOG(m_pDlg), GTK_RESPONSE_OK);
  return;
}

void CDScaleDlg::ScaleDrawChange(GtkToggleButton *togglebutton)
{
  if(m_bSettingUp) return;

  gboolean bCheck = gtk_toggle_button_get_active(togglebutton);

  gtk_widget_set_sensitive(m_pNewNomEdt, bCheck);
  gtk_widget_set_sensitive(m_pNewDenomEdt, bCheck);
  gtk_widget_set_sensitive(m_pScaleWidthChB, bCheck);
  gtk_widget_set_sensitive(m_pScalePatChB, bCheck);
  gtk_widget_set_sensitive(m_pScaleArrowChB, bCheck);
  gtk_widget_set_sensitive(m_pScaleLabChB, bCheck);
}

void CDScaleDlg::ChangeUnitsChange(GtkToggleButton *togglebutton)
{
  if(m_bSettingUp) return;

  gboolean bCheck = gtk_toggle_button_get_active(togglebutton);

  gtk_widget_set_sensitive(m_pLenMaskEdt, bCheck);
  gtk_widget_set_sensitive(m_pAngMaskEdt, bCheck);
  return;
}

