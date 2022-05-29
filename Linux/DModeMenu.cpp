#include <string.h>
#include <gdk/gdkkeysyms.h>
#include "DLxBaseDefs.h"
#include "DFileMenu.hpp"
#include "DApplication.hpp"

static void mode_sel_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODESELECT, false);
  return;
}

static void mode_sel_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODESELECT, true);
  return;
}

static void mode_line_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODELINE, false);
  return;
}

static void mode_line_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODELINE, true);
  return;
}

static void mode_rect_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODERECT, false);
  return;
}

static void mode_rect_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODERECT, true);
  return;
}

static void mode_circ_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODECIRCLE, false);
  return;
}

static void mode_circ_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODECIRCLE, true);
  return;
}

static void mode_elps_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEELLIPSE, false);
  return;
}

static void mode_elps_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEELLIPSE, true);
  return;
}

static void mode_arcelps_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEARCELLIPSE, false);
  return;
}

static void mode_arcelps_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEARCELLIPSE, true);
  return;
}

static void mode_hyper_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEHYPERBOLA, false);
  return;
}

static void mode_hyper_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEHYPERBOLA, true);
  return;
}

static void mode_parab_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEPARABOLA, false);
  return;
}

static void mode_parab_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEPARABOLA, true);
  return;
}

static void mode_spline_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODESPLINE, false);
  return;
}

static void mode_spline_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODESPLINE, true);
  return;
}

static void mode_evolv_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEEVEOLVENT, false);
  return;
}

static void mode_evolv_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEEVEOLVENT, true);
  return;
}

static void mode_dimen_click(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEDIMEN, false);
  return;
}

static void mode_dimen_accel(PDApplication pApp)
{
  pApp->ModeCommand(IDM_MODEDIMEN, true);
  return;
}

void CreateModeMenu(void *pPtr, GtkMenuShell *pMenuBar, GtkAccelGroup *pAccel, GtkAccelGroup *pEscAccel)
{
  PDApplication pApp = (PDApplication)pPtr;

  GtkMenuShell *menu = (GtkMenuShell*)gtk_menu_new();

  GtkWidget *menu_item;
  GClosure *pClos;
  GtkWidget *menu_label;

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Select"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);
  gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(menu_item), TRUE);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_sel_accel), pApp, NULL);
  gtk_accel_group_connect(pEscAccel, GDK_KEY_Escape, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_sel_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Line"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_line_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_L, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_line_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Rectangle"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_rect_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_R, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_rect_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Circle"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_circ_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_C, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_circ_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Ellipse"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_elps_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_E, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_elps_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Arc Ellipse"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_arcelps_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_A, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_arcelps_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Hyperbola"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_hyper_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_H, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_hyper_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Parabola"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_parab_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_P, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_parab_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Spline"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_spline_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_S, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_spline_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("E_volvent"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_evolv_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_V, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_evolv_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_check_menu_item_new_with_mnemonic(_("_Dimension"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(mode_dimen_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_D, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(mode_dimen_click), pApp);
  gtk_widget_show(menu_item);

  char buf[128];
  strcpy(buf, _("Mode"));
  GtkWidget *mode_menu = gtk_menu_item_new_with_mnemonic(buf);
  gtk_widget_show(mode_menu);
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(mode_menu), GTK_WIDGET(menu));
  gtk_menu_shell_append(pMenuBar, mode_menu);
}

