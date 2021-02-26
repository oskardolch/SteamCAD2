#include <string.h>
#include <gdk/gdkkeysyms.h>
#include "DLxBaseDefs.h"
#include "DFileMenu.hpp"
#include "DApplication.hpp"

static void file_new_click(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILENEW, false);
  return;
}

static void file_new_accel(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILENEW, true);
  return;
}

static void file_open_click(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILEOPEN, false);
  return;
}

static void file_open_accel(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILEOPEN, true);
  return;
}

static void file_save_click(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILESAVE, false);
  return;
}

static void file_save_accel(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILESAVE, true);
  return;
}

static void file_save_as_click(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILESAVEAS, false);
  return;
}

static void file_save_sel_click(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILESAVESEL, false);
  return;
}

static void file_include_click(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILEINCLUDE, false);
  return;
}

static void file_export_click(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILEEXPORT, false);
  return;
}

static void file_props_click(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILEPROPS, false);
  return;
}

static void file_quit_click(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILEEXIT, false);
  return;
}

static void file_quit_accel(PDApplication pApp)
{
  pApp->FileCommand(IDM_FILEEXIT, true);
  return;
}

void CreateFileMenu(void *pPtr, GtkMenuShell *pMenuBar, GtkAccelGroup *pAccel)
{
  PDApplication pApp = (PDApplication)pPtr;

  /* Init the menu-widget, and remember -- never
  * gtk_show_widget() the menu widget!!
  * This is the menu that holds the menu items, the one that
  * will pop up when you click on the "Root Menu" in the app */
  GtkMenuShell *menu = (GtkMenuShell*)gtk_menu_new();

  GtkWidget *menu_item;
  GtkStockItem gtkSI;
  GClosure *pClos;
  GtkWidget *menu_label;

  if(gtk_stock_lookup(GTK_STOCK_NEW, &gtkSI))
  {
    menu_item = gtk_image_menu_item_new_from_stock(GTK_STOCK_NEW, NULL);
  }
  else
  {
    menu_item = gtk_menu_item_new_with_mnemonic(_("_New"));
    gtkSI.keyval = GDK_N;
    gtkSI.modifier = GDK_CONTROL_MASK;
  }
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(file_new_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, gtkSI.keyval, gtkSI.modifier, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(file_new_click), pApp);
  gtk_widget_show(menu_item);

  if(gtk_stock_lookup(GTK_STOCK_OPEN, &gtkSI))
  {
    menu_item = gtk_image_menu_item_new_from_stock(GTK_STOCK_OPEN, NULL);
  }
  else
  {
    menu_item = gtk_menu_item_new_with_mnemonic (_("_Open"));
    gtkSI.keyval = GDK_O;
    gtkSI.modifier = GDK_CONTROL_MASK;
  }
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(file_open_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, gtkSI.keyval, gtkSI.modifier, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(file_open_click), pApp);
  gtk_widget_show(menu_item);

  if(gtk_stock_lookup(GTK_STOCK_SAVE, &gtkSI))
  {
    menu_item = gtk_image_menu_item_new_from_stock(GTK_STOCK_SAVE, NULL);
  }
  else
  {
    menu_item = gtk_menu_item_new_with_mnemonic(_("_Save"));
    gtkSI.keyval = GDK_S;
    gtkSI.modifier = GDK_CONTROL_MASK;
  }
  gtk_menu_shell_append(GTK_MENU_SHELL (menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(file_save_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, gtkSI.keyval, gtkSI.modifier, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(file_save_click), pApp);
  gtk_widget_show(menu_item);

  if(gtk_stock_lookup(GTK_STOCK_SAVE_AS, &gtkSI))
  {
    menu_item = gtk_image_menu_item_new_from_stock(GTK_STOCK_SAVE_AS, pAccel);
  }
  else
  {
    menu_item = gtk_menu_item_new_with_mnemonic(_("Save _As ..."));
  }
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(file_save_as_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_separator_menu_item_new();
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("Save se_lection ..."));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(file_save_sel_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Include ..."));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(file_include_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_menu_item_new_with_mnemonic(_("_Export ..."));
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(file_export_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_separator_menu_item_new();
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);
  gtk_widget_show(menu_item);

  if(gtk_stock_lookup(GTK_STOCK_PROPERTIES, &gtkSI))
  {
    menu_item = gtk_image_menu_item_new_from_stock(GTK_STOCK_PROPERTIES, pAccel);
  }
  else
  {
    menu_item = gtk_menu_item_new_with_mnemonic(_("_Properties"));
  }
  gtk_menu_shell_append(GTK_MENU_SHELL (menu), menu_item);

  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(file_props_click), pApp);
  gtk_widget_show(menu_item);

  menu_item = gtk_separator_menu_item_new();
  gtk_menu_shell_append(GTK_MENU_SHELL (menu), menu_item);
  gtk_widget_show(menu_item);

  if(gtk_stock_lookup(GTK_STOCK_QUIT, &gtkSI))
  {
    menu_item = gtk_image_menu_item_new_from_stock(GTK_STOCK_QUIT, NULL);
  }
  else
  {
    menu_item = gtk_menu_item_new_with_mnemonic (_("_Quit"));
    gtkSI.keyval = GDK_Q;
    gtkSI.modifier = GDK_CONTROL_MASK;
  }
  gtk_menu_shell_append(GTK_MENU_SHELL(menu), menu_item);

  pClos = g_cclosure_new_swap(G_CALLBACK(file_quit_accel), pApp, NULL);
  gtk_accel_group_connect(pAccel, gtkSI.keyval, gtkSI.modifier, GTK_ACCEL_MASK, pClos);
  menu_label = gtk_bin_get_child(GTK_BIN(menu_item));
  gtk_accel_label_set_accel_closure(GTK_ACCEL_LABEL(menu_label), pClos);
  g_signal_connect_swapped(G_OBJECT(menu_item), "activate", G_CALLBACK(file_quit_click), pApp);
  gtk_widget_show(menu_item);

  char buf[128];
  if(gtk_stock_lookup(GTK_STOCK_FILE, &gtkSI)) strcpy(buf, gtkSI.label);
  else strcpy(buf, _("_File"));
  /* This is the root menu, and will be the label
  * displayed on the menu bar.  There won't be a signal handler attached,
  * as it only pops up the rest of the menu when pressed. */
  GtkWidget *file_menu = gtk_menu_item_new_with_mnemonic(buf);
  gtk_widget_show(file_menu);

  /* Now we specify that we want our newly created "menu" to be the menu
  * for the "root menu" */
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(file_menu), GTK_WIDGET(menu));

  /* And finally we append the menu-item to the menu-bar -- this is the
  * "root" menu-item I have been raving about =) */
  gtk_menu_shell_append(pMenuBar, file_menu);

  return;
}

