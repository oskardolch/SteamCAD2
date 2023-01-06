#include <string.h>
#include <gdk/gdkkeysyms.h>
#include "DLxBaseDefs.h"
#include "DRasterMenu.hpp"
#include "DApplication.hpp"

static void raster_import_click(PDApplication pApp)
{
  pApp->RasterCommand(IDM_RASTERIMPORT, false);
  return;
}

//static void raster_import_accel(PDApplication pApp)
//{
//  pApp->RasterCommand(IDM_RASTERIMPORT, true);
//  return;
//}

static void raster_register_click(PDApplication pApp)
{
  pApp->RasterCommand(IDM_RASTERREGISTER, false);
  return;
}

//static void raster_register_accel(PDApplication pApp)
//{
//  pApp->RasterCommand(IDM_RASTERREGISTER, true);
//  return;
//}

static void raster_hide_click(PDApplication pApp)
{
  pApp->RasterCommand(IDM_RASTERHIDE, false);
  return;
}

static void raster_hide_accel(PDApplication pApp)
{
  pApp->RasterCommand(IDM_RASTERHIDE, true);
  return;
}

static void raster_show_click(PDApplication pApp)
{
  pApp->RasterCommand(IDM_RASTERSHOW, false);
  return;
}

static void raster_show_accel(PDApplication pApp)
{
  pApp->RasterCommand(IDM_RASTERSHOW, true);
  return;
}

void CreateRasterMenu(void *pPtr, GtkMenuShell *pMenuBar, GtkAccelGroup *pAccel)
{
  PDApplication pApp = (PDApplication)pPtr;

  GtkMenuShell *menu = (GtkMenuShell*)gtk_menu_new();

  GtkWidget *menu_item;
  GClosure *pClos;
  GtkWidget *menu_label;

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Import raster"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  //pClos = g_cclosure_new_swap(G_CALLBACK(raster_import_accel), pApp, NULL);
  //gtk_accel_group_connect(pAccel, GDK_H, GDK_MOD1_MASK, GTK_ACCEL_MASK, pClos);
  //menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  //gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(raster_import_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Register raster"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  //pClos = g_cclosure_new_swap(G_CALLBACK(raster_register_accel), pApp, NULL);
  //gtk_accel_group_connect(pAccel, GDK_A, GDK_CONTROL_MASK, GTK_ACCEL_MASK, pClos);
  //menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  //gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(raster_register_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Hide raster"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(raster_hide_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_H, GDK_CONTROL_MASK, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(raster_hide_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Show raster"));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(raster_show_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, GDK_R, GDK_MOD1_MASK, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(raster_show_click), pApp);
  gtk_widget_show(menu_item);

  char buf[128];
  strcpy(buf, _("Ras_ters"));
  GtkWidget *raster_menu = gtk_menu_item_new_with_mnemonic(buf);
  gtk_widget_show(raster_menu);
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(raster_menu), GTK_WIDGET(menu));
  gtk_menu_shell_append(pMenuBar, raster_menu);
  return;
}

