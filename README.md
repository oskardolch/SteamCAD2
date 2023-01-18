# SteamCAD2
SteamCAD2 is an evolution of SteamCAD, a 2D CAD especially designed to draw steam
locomotives.

## What's new in SteamCAD2
SteamCAD was a first version of a 2D CAD versatile enough to draw an arbitrary black
and white techical drawing. The aim of SteamCAD2 is to fix some issues and also add
new functionality. These are namely:

* the whole core was completely rebuilt to improve the rendering speed
* creating paths consisting of one or more segments, this is especially usefull when
rendering patterned lines
* added abilities to control line join shapes and dash endings
* the MS Windows rendering was changed from GDI to GDI+, thus giving Windows users
nearly the same experience like the Linux users
* the previous point allowed to use the internal backend dash line rendering, which
will improve the performance and also the postprocessing in export formats
* the lines can have an arbitrary colour in SteamCAD2
* filled areas can be created
* raster images can be imported and registered for the drawing
* spline editing capability added

SteamCAD2 is 2D CAD especially designed to draw steam locomotives like this example:
![264.0](/Images/264_0.svg)

[SteamCAD2.pdf](/SteamCAD2.pdf) is the full manual

Sample drawings can be found in the Samples folder
