#include <string.h>
#include <gdk/gdkkeysyms.h>
#include "DLxBaseDefs.h"
#include "DPathMenu.hpp"
#include "DApplication.hpp"

static void path_create_click(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHCREATE, false);
  return;
}

static void path_create_accel(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHCREATE, true);
  return;
}

static void path_break_click(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHBREAK, false);
  return;
}

static void path_break_accel(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHBREAK, true);
  return;
}

static void path_area_click(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHAREA, false);
  return;
}

static void path_area_accel(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHAREA, true);
  return;
}

static void path_group_click(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHGROUP, false);
  return;
}

static void path_group_accel(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHGROUP, true);
  return;
}

static void path_ungroup_click(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHUNGROUP, false);
  return;
}

static void path_ungroup_accel(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHUNGROUP, true);
  return;
}

static void path_moveup_click(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHMOVEUP, false);
  return;
}

static void path_moveup_accel(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHMOVEUP, true);
  return;
}

static void path_movedown_click(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHMOVEDOWN, false);
  return;
}

static void path_movedown_accel(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHMOVEDOWN, true);
  return;
}

static void path_movetop_click(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHMOVETOP, false);
  return;
}

static void path_movetop_accel(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHMOVETOP, true);
  return;
}

static void path_movebottom_click(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHMOVEBOTTOM, false);
  return;
}

static void path_movebottom_accel(PDApplication pApp)
{
  pApp->PathCommand(IDM_PATHMOVEBOTTOM, true);
  return;
}

void CreatePathMenu(void *pPtr, GtkMenuShell *pMenuBar, GtkAccelGroup *pAccel)
{
  PDApplication pApp = (PDApplication)pPtr;

  GtkMenuShell *menu = (GtkMenuShell*)gtk_menu_new();

  GtkWidget *menu_item;
  GClosure *pClos;
  GtkWidget *menu_label;

  menu_item = gtk_menu_item_new_with_mnemonic(_("Create _path"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(path_create_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_H, GDK_MOD1_MASK, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(path_create_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("Create _area"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(path_area_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_A, GDK_CONTROL_MASK, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(path_area_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Break apart"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(path_break_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_B, GDK_MOD1_MASK, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(path_break_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Group"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(path_group_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_G, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(path_group_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Ungroup"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(path_ungroup_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_U, GDK_MOD1_MASK, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(path_ungroup_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_separator_menu_item_new();
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Move up"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(path_moveup_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_Up, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(path_moveup_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("Move _down"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(path_movedown_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_Down, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(path_movedown_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("Move _top"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(path_movetop_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_Page_Up, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(path_movetop_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("Move _bottom"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(path_movebottom_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_Page_Down, (GdkModifierType)0, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(path_movebottom_click), pApp);
  gtk_widget_show(menu_item);

  char buf[128];
  strcpy(buf, _("_Paths"));
  GtkWidget *tools_menu = gtk_menu_item_new_with_mnemonic(buf);
  gtk_widget_show(tools_menu);
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(tools_menu), GTK_WIDGET(menu));
  gtk_menu_shell_append(pMenuBar, tools_menu);
  return;
}

