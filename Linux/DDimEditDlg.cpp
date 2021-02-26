#include "DDimEditDlg.hpp"
#include "DLxBaseDefs.h"

#include <stdio.h>
#include <malloc.h>
#include <string.h>

typedef struct CDedGridEdtData
{
  PDDimEditDlg pApp;
  gint id;
} *PDedGridEdtData;

void ded_gridedt_changed(GtkEntry *entry, PDedGridEdtData pData)
{
  pData->pApp->GridEditChanged(entry, pData->id);
}

void ded_fontattrchb_toggled(GtkToggleButton *togglebutton, PDedGridEdtData pData)
{
  pData->pApp->FontAttrChBChange(togglebutton, pData->id);
}

gboolean ded_configure(GtkWidget *widget, GdkEvent *event, PDDimEditDlg pApp)
{
  return pApp->Configure(widget, event);
}

void ded_font_set(GtkFontButton *widget, PDDimEditDlg pApp)
{
  pApp->FontSet(widget);
}

void ded_okbtn_clicked(GtkButton *button, PDDimEditDlg pApp)
{
  pApp->OKBtnClick(button);
}


// CDDimEditDlg

CDDimEditDlg::CDDimEditDlg()
{
  m_iX = -100;
  m_iY = -100;
  m_bSettingUp = FALSE;
}

CDDimEditDlg::~CDDimEditDlg()
{
}

bool CDDimEditDlg::ShowDialog(GtkWidget *pWndParent, PDDimension pDimen, PDUnitList pUnits,
  PDFileUnit pUnit)
{
  m_pDimen = pDimen;
  m_pUnits = pUnits;
  m_pUnit = pUnit;

  m_dCurLeftLen = m_pDimen->cArrowDim1.x/m_pUnit->dBaseToUnit;
  m_dCurLeftWidth = m_pDimen->cArrowDim1.y/m_pUnit->dBaseToUnit;
  m_dCurRightLen = m_pDimen->cArrowDim2.x/m_pUnit->dBaseToUnit;
  m_dCurRightWidth = m_pDimen->cArrowDim2.y/m_pUnit->dBaseToUnit;

  m_dCurFontSize = m_pDimen->dFontSize/m_pUnit->dBaseToUnit;
  m_bCurFntAttrs = m_pDimen->bFontAttrs;
  strcpy(m_sCurFntName, m_pDimen->psFontFace);

  m_bSettingUp = TRUE;

  m_pDlg = gtk_dialog_new_with_buttons(_("Edit Dimension"), GTK_WINDOW(pWndParent),
    GTK_DIALOG_MODAL, GTK_STOCK_CANCEL, GTK_RESPONSE_CANCEL, NULL);
  g_signal_connect(G_OBJECT(m_pDlg), "configure-event", G_CALLBACK(ded_configure), this);

  GtkWidget *pAA = gtk_dialog_get_action_area(GTK_DIALOG(m_pDlg));
  GtkWidget *pBtn = gtk_button_new_from_stock("gtk-ok");
  gtk_box_pack_end(GTK_BOX(pAA), pBtn, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(pBtn), "clicked", G_CALLBACK(ded_okbtn_clicked), this);
  gtk_widget_set_can_default(pBtn, TRUE);
  gtk_widget_show(pBtn);

  GtkWidget *pCA = gtk_dialog_get_content_area(GTK_DIALOG(m_pDlg));

  GtkWidget *pFrm = gtk_frame_new(_("Arrows"));
  gtk_box_pack_start(GTK_BOX(pCA), pFrm, TRUE, TRUE, 0);
  gtk_widget_show(pFrm);

  GtkWidget *pTbl = gtk_table_new(4, 6, FALSE);
  gtk_table_set_col_spacings(GTK_TABLE(pTbl), 3);
  gtk_table_set_row_spacings(GTK_TABLE(pTbl), 3);
  gtk_table_set_col_spacing(GTK_TABLE(pTbl), 2, 10);
  gtk_container_set_border_width(GTK_CONTAINER(pFrm), 2);
  gtk_container_set_border_width(GTK_CONTAINER(pTbl), 4);
  gtk_container_add(GTK_CONTAINER(pFrm), pTbl);
  gtk_widget_show(pTbl);

  GtkWidget *pLab = gtk_label_new(_("Left shape:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 3, 0, 1);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Right shape:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 1);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 3, 6, 0, 1);
  gtk_widget_show(pLab);

  m_pLeftArrowCB = gtk_combo_box_text_new();
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pLeftArrowCB, 0, 3, 1, 2);
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pLeftArrowCB), _("None"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pLeftArrowCB), _("Standard"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pLeftArrowCB), _("Filled"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pLeftArrowCB), _("Point"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pLeftArrowCB), _("Slash"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pLeftArrowCB), _("Backslash"));
  gtk_combo_box_set_active(GTK_COMBO_BOX(m_pLeftArrowCB), m_pDimen->iArrowType1);
  gtk_widget_show(m_pLeftArrowCB);

  m_pRightArrowCB = gtk_combo_box_text_new();
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pRightArrowCB, 3, 6, 1, 2);
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pRightArrowCB), _("None"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pRightArrowCB), _("Standard"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pRightArrowCB), _("Filled"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pRightArrowCB), _("Point"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pRightArrowCB), _("Slash"));
  gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(m_pRightArrowCB), _("Backslash"));
  gtk_combo_box_set_active(GTK_COMBO_BOX(m_pRightArrowCB), m_pDimen->iArrowType2);
  gtk_widget_show(m_pRightArrowCB);

  pLab = gtk_label_new(_("Left length:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 2, 3);
  gtk_widget_show(pLab);

  m_pLeftLenEdt = gtk_entry_new();
  gtk_widget_set_size_request(m_pLeftLenEdt, 50, -1);
  gtk_entry_set_max_length(GTK_ENTRY(m_pLeftLenEdt), 32);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pLeftLenEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pLeftLenEdt, 1, 2, 2, 3);
  CDedGridEdtData cLeftLenData = {this, 1};
  g_signal_connect(G_OBJECT(m_pLeftLenEdt), "changed", G_CALLBACK(ded_gridedt_changed), &cLeftLenData);
  gtk_widget_show(m_pLeftLenEdt);

  gchar buf[32];
  FormatFloatStr(m_dCurLeftLen, buf);
  gtk_entry_set_text(GTK_ENTRY(m_pLeftLenEdt), buf);

  pLab = gtk_label_new(m_pUnit->sAbbrev);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 2, 3, 2, 3);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Right length:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 3, 4, 2, 3);
  gtk_widget_show(pLab);

  m_pRightLenEdt = gtk_entry_new();
  gtk_widget_set_size_request(m_pRightLenEdt, 50, -1);
  gtk_entry_set_max_length(GTK_ENTRY(m_pRightLenEdt), 32);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pRightLenEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pRightLenEdt, 4, 5, 2, 3);
  CDedGridEdtData cRightLenData = {this, 2};
  g_signal_connect(G_OBJECT(m_pRightLenEdt), "changed", G_CALLBACK(ded_gridedt_changed), &cRightLenData);
  gtk_widget_show(m_pRightLenEdt);

  FormatFloatStr(m_dCurRightLen, buf);
  gtk_entry_set_text(GTK_ENTRY(m_pRightLenEdt), buf);

  pLab = gtk_label_new(m_pUnit->sAbbrev);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 5, 6, 2, 3);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Left width:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 3, 4);
  gtk_widget_show(pLab);

  m_pLeftWidthEdt = gtk_entry_new();
  gtk_widget_set_size_request(m_pLeftWidthEdt, 50, -1);
  gtk_entry_set_max_length(GTK_ENTRY(m_pLeftWidthEdt), 32);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pLeftWidthEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pLeftWidthEdt, 1, 2, 3, 4);
  CDedGridEdtData cLeftWidthData = {this, 3};
  g_signal_connect(G_OBJECT(m_pLeftWidthEdt), "changed", G_CALLBACK(ded_gridedt_changed), &cLeftWidthData);
  gtk_widget_show(m_pLeftWidthEdt);

  FormatFloatStr(m_dCurLeftWidth, buf);
  gtk_entry_set_text(GTK_ENTRY(m_pLeftWidthEdt), buf);

  pLab = gtk_label_new(m_pUnit->sAbbrev);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 2, 3, 3, 4);
  gtk_widget_show(pLab);

  pLab = gtk_label_new(_("Right width:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 3, 4, 3, 4);
  gtk_widget_show(pLab);

  m_pRightWidthEdt = gtk_entry_new();
  gtk_widget_set_size_request(m_pRightWidthEdt, 50, -1);
  gtk_entry_set_max_length(GTK_ENTRY(m_pRightWidthEdt), 32);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pRightWidthEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pRightWidthEdt, 4, 5, 3, 4);
  CDedGridEdtData cRightWidthData = {this, 4};
  g_signal_connect(G_OBJECT(m_pRightWidthEdt), "changed", G_CALLBACK(ded_gridedt_changed), &cRightWidthData);
  gtk_widget_show(m_pRightWidthEdt);

  FormatFloatStr(m_dCurRightWidth, buf);
  gtk_entry_set_text(GTK_ENTRY(m_pRightWidthEdt), buf);

  pLab = gtk_label_new(m_pUnit->sAbbrev);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 5, 6, 3, 4);
  gtk_widget_show(pLab);

  GtkWidget *pBox1 = gtk_hbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(pCA), pBox1, TRUE, TRUE, 0);
  gtk_widget_show(pBox1);

  pFrm = gtk_frame_new(_("Font"));
  gtk_box_pack_start(GTK_BOX(pBox1), pFrm, TRUE, TRUE, 0);
  gtk_widget_show(pFrm);

  pTbl = gtk_table_new(3, 3, FALSE);
  gtk_table_set_col_spacings(GTK_TABLE(pTbl), 3);
  gtk_table_set_row_spacings(GTK_TABLE(pTbl), 3);
  gtk_container_set_border_width(GTK_CONTAINER(pFrm), 2);
  gtk_container_set_border_width(GTK_CONTAINER(pTbl), 4);
  gtk_container_add(GTK_CONTAINER(pFrm), pTbl);
  gtk_widget_show(pTbl);

  pLab = gtk_label_new(_("Face:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 0, 1);
  gtk_widget_show(pLab);

  gchar sBuf2[128];
  strcpy(sBuf2, m_sCurFntName);
  if(m_bCurFntAttrs & 8) strcat(sBuf2, " Bold");
  if(m_bCurFntAttrs & 1) strcat(sBuf2, " Italic");
  strcat(sBuf2, " 12");
  m_pFntFaceCB = gtk_font_button_new_with_font(sBuf2);
  gtk_font_button_set_show_size(GTK_FONT_BUTTON(m_pFntFaceCB), FALSE);
  //gtk_font_button_set_use_size(GTK_FONT_BUTTON(m_pFntFaceCB), FALSE);
  gtk_font_button_set_show_style(GTK_FONT_BUTTON(m_pFntFaceCB), FALSE);
  g_signal_connect(G_OBJECT(m_pFntFaceCB), "font-set", G_CALLBACK(ded_font_set), this);

  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pFntFaceCB, 1, 3, 0, 1);
  gtk_widget_show(m_pFntFaceCB);

  m_pBoldChB = gtk_check_button_new_with_label(_("Bold"));
  if(m_bCurFntAttrs & 8) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(m_pBoldChB), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pBoldChB, 0, 1, 1, 2);
  CDedGridEdtData cBoldChBData = {this, 8};
  g_signal_connect(G_OBJECT(m_pBoldChB), "toggled", G_CALLBACK(ded_fontattrchb_toggled), &cBoldChBData);
  gtk_widget_show(m_pBoldChB);

  m_pItalicChB = gtk_check_button_new_with_label(_("Italic"));
  if(m_bCurFntAttrs & 1) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(m_pItalicChB), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pItalicChB, 1, 2, 1, 2);
  CDedGridEdtData cItalicChBData = {this, 1};
  g_signal_connect(G_OBJECT(m_pItalicChB), "toggled", G_CALLBACK(ded_fontattrchb_toggled), &cItalicChBData);
  gtk_widget_show(m_pItalicChB);

  m_pUnderChB = gtk_check_button_new_with_label(_("Underline"));
  if(m_bCurFntAttrs & 2) gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(m_pUnderChB), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pUnderChB, 2, 3, 1, 2);
  CDedGridEdtData cUnderChBData = {this, 2};
  g_signal_connect(G_OBJECT(m_pUnderChB), "toggled", G_CALLBACK(ded_fontattrchb_toggled), &cUnderChBData);
  gtk_widget_show(m_pUnderChB);

  pLab = gtk_label_new(_("Size:"));
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 0, 1, 2, 3);
  gtk_widget_show(pLab);

  m_pFntSizeEdt = gtk_entry_new();
  gtk_widget_set_size_request(m_pFntSizeEdt, 40, -1);
  gtk_entry_set_max_length(GTK_ENTRY(m_pFntSizeEdt), 32);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pFntSizeEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pFntSizeEdt, 1, 2, 2, 3);
  CDedGridEdtData cFntSizeEdtData = {this, 5};
  g_signal_connect(G_OBJECT(m_pFntSizeEdt), "changed", G_CALLBACK(ded_gridedt_changed), &cFntSizeEdtData);
  gtk_widget_show(m_pFntSizeEdt);

  FormatFloatStr(m_dCurFontSize/m_pUnit->dBaseToUnit, buf);
  gtk_entry_set_text(GTK_ENTRY(m_pFntSizeEdt), buf);

  pLab = gtk_label_new(m_pUnit->sAbbrev);
  gtk_misc_set_alignment(GTK_MISC(pLab), 0, 0.5);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), pLab, 2, 3, 2, 3);
  gtk_widget_show(pLab);

  pFrm = gtk_frame_new(_("Text"));
  gtk_box_pack_start(GTK_BOX(pBox1), pFrm, TRUE, TRUE, 0);
  gtk_widget_show(pFrm);

  pTbl = gtk_table_new(3, 1, FALSE);
  gtk_table_set_col_spacings(GTK_TABLE(pTbl), 3);
  gtk_table_set_row_spacings(GTK_TABLE(pTbl), 3);
  gtk_container_set_border_width(GTK_CONTAINER(pFrm), 2);
  gtk_container_set_border_width(GTK_CONTAINER(pTbl), 4);
  gtk_container_add(GTK_CONTAINER(pFrm), pTbl);
  gtk_widget_show(pTbl);

  m_pTextEdt = gtk_entry_new();
  //gtk_widget_set_size_request(m_pTextEdt, 40, -1);
  gtk_entry_set_max_length(GTK_ENTRY(m_pTextEdt), 64);
  gtk_entry_set_activates_default(GTK_ENTRY(m_pTextEdt), TRUE);
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pTextEdt, 0, 1, 0, 1);
  gtk_widget_show(m_pTextEdt);

  if(m_pDimen->psLab)
    gtk_entry_set_text(GTK_ENTRY(m_pTextEdt), m_pDimen->psLab);

  m_pFntSampleLab = gtk_label_new(_("Sample 0123456"));
  gtk_table_attach_defaults(GTK_TABLE(pTbl), m_pFntSampleLab, 0, 1, 1, 3);
  gtk_widget_show(m_pFntSampleLab);

  PangoFontDescription *ppfd = pango_font_description_from_string(
    gtk_font_button_get_font_name(GTK_FONT_BUTTON(m_pFntFaceCB)));
  pango_font_description_set_size(ppfd, 12288);
  gtk_widget_modify_font(m_pFntSampleLab, ppfd);
  pango_font_description_free(ppfd);

  gtk_widget_grab_default(pBtn);
  gtk_window_set_default(GTK_WINDOW(m_pDlg), pBtn);

  gtk_window_set_resizable(GTK_WINDOW(m_pDlg), FALSE);
  if(m_iX > -100) gtk_window_move(GTK_WINDOW(m_pDlg), m_iX, m_iY);

  m_bSettingUp = FALSE;

  gboolean bRes = (gtk_dialog_run(GTK_DIALOG(m_pDlg)) == GTK_RESPONSE_OK);

  if(m_iX < -90) gdk_window_get_position(m_pDlg->window, &m_iX, &m_iY);

  gtk_widget_destroy(m_pDlg);

  return bRes;
}

void CDDimEditDlg::SaveSettings(FILE *fp)
{
  if(m_iX < -90) return;

  gchar sbuf[256];
  sprintf(sbuf, "  <DimEditDlg Left=\"%d\" Top=\"%d\"/>\n", m_iX, m_iY);
  fwrite(sbuf, sizeof(gchar), strlen(sbuf), fp);
  return;
}

void CDDimEditDlg::RestoreSettings(gint iLeft, gint iTop)
{
  m_iX = iLeft;
  m_iY = iTop;
  return;
}

gboolean CDDimEditDlg::Configure(GtkWidget *widget, GdkEvent *event)
{
  if(m_bSettingUp) return FALSE;

  m_iX = event->configure.x;
  m_iY = event->configure.y;
  return FALSE;
}

void CDDimEditDlg::OKBtnClick(GtkButton *button)
{
  const gchar *buf = gtk_entry_get_text(GTK_ENTRY(m_pTextEdt));
  int iValRes = ValidateMask(buf, m_pUnits);
  if(iValRes != 0)
  {
    char sMsgBuf[64];
    switch(iValRes)
    {
    case 1:
      strcpy(sMsgBuf, _("Precision should not be negative"));
      break;
    case 2:
      strcpy(sMsgBuf, _("Precision should not be greater than 16"));
      break;
    case 3:
      strcpy(sMsgBuf, _("Precision value cannot be parsed"));
      break;
    default:
      strcpy(sMsgBuf, _("Invalid mask (unknown reason)"));
    }
    GtkWidget *msg_dlg = gtk_message_dialog_new(GTK_WINDOW(m_pDlg), GTK_DIALOG_MODAL,
    GTK_MESSAGE_WARNING, GTK_BUTTONS_OK, "%s", sMsgBuf);
    gtk_dialog_run(GTK_DIALOG(msg_dlg));
    gtk_widget_destroy(msg_dlg);
    gtk_widget_grab_focus(m_pTextEdt);
    return;
  }
  else
  {
    if(m_pDimen->psLab) free(m_pDimen->psLab);
    m_pDimen->psLab = NULL;
    if(buf)
    {
      int iLen = strlen(buf);
      m_pDimen->psLab = (char*)malloc((iLen + 1)*sizeof(char));
      strcpy(m_pDimen->psLab, buf);
    }
  }

  m_pDimen->iArrowType1 = gtk_combo_box_get_active(GTK_COMBO_BOX(m_pLeftArrowCB));
  m_pDimen->cArrowDim1.x = m_dCurLeftLen*m_pUnit->dBaseToUnit;
  m_pDimen->cArrowDim1.y = m_dCurLeftWidth*m_pUnit->dBaseToUnit;

  m_pDimen->iArrowType2 = gtk_combo_box_get_active(GTK_COMBO_BOX(m_pRightArrowCB));
  m_pDimen->cArrowDim2.x = m_dCurRightLen*m_pUnit->dBaseToUnit;
  m_pDimen->cArrowDim2.y = m_dCurRightWidth*m_pUnit->dBaseToUnit;

  m_pDimen->dFontSize = m_dCurFontSize*m_pUnit->dBaseToUnit;
  m_pDimen->bFontAttrs = m_bCurFntAttrs;
  strcpy(m_pDimen->psFontFace, m_sCurFntName);

  gtk_dialog_response(GTK_DIALOG(m_pDlg), GTK_RESPONSE_OK);
  return;
}

void CDDimEditDlg::GridEditChanged(GtkEntry *entry, gint id)
{
  if(m_bSettingUp) return;

  float f;
  if(sscanf(gtk_entry_get_text(entry), "%f", &f) == 1)
  {
    switch(id)
    {
    case 1:
      m_dCurLeftLen = f;
      break;
    case 2:
      m_dCurRightLen = f;
      break;
    case 3:
      m_dCurLeftWidth = f;
      break;
    case 4:
      m_dCurRightWidth = f;
      break;
    case 5:
      m_dCurFontSize = f;
      break;
    }
  }
  return;
}

void CDDimEditDlg::FontAttrChBChange(GtkToggleButton *togglebutton, gint iMask)
{
  if(m_bSettingUp) return;

  if(gtk_toggle_button_get_active(togglebutton)) m_bCurFntAttrs |= iMask;
  else m_bCurFntAttrs &= ~iMask;

  gchar sBuf2[128];
  strcpy(sBuf2, m_sCurFntName);
  if(m_bCurFntAttrs & 8) strcat(sBuf2, " Bold");
  if(m_bCurFntAttrs & 1) strcat(sBuf2, " Italic");
  strcat(sBuf2, " 12");

  m_bSettingUp = TRUE;
  gtk_font_button_set_font_name(GTK_FONT_BUTTON(m_pFntFaceCB), sBuf2);

  PangoFontDescription *ppfd = pango_font_description_from_string(gtk_font_button_get_font_name(GTK_FONT_BUTTON(m_pFntFaceCB)));
  gtk_widget_modify_font(m_pFntSampleLab, ppfd);
  m_bSettingUp = FALSE;

  pango_font_description_free(ppfd);
  return;
}

void CDDimEditDlg::FontSet(GtkFontButton *widget)
{
  if(m_bSettingUp) return;

  PangoFontDescription *ppfd = pango_font_description_from_string(gtk_font_button_get_font_name(widget));
  PangoStyle pst = pango_font_description_get_style(ppfd);
  PangoWeight pwe = pango_font_description_get_weight(ppfd);
  strcpy(m_sCurFntName, pango_font_description_get_family(ppfd));

  m_bSettingUp = TRUE;

  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(m_pBoldChB), pwe > PANGO_WEIGHT_MEDIUM);
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(m_pItalicChB), pst > PANGO_STYLE_NORMAL);

  pango_font_description_set_size(ppfd, 12288);
  gtk_widget_modify_font(m_pFntSampleLab, ppfd);

  m_bSettingUp = FALSE;

  pango_font_description_free(ppfd);
}

