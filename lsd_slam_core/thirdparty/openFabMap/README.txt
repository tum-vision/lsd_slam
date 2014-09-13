/*------------------------------------------------------------------------
 Copyright 2012 Arren Glover [aj.glover@qut.edu.au]

 This file is part of OpenFABMAP. http://code.google.com/p/openfabmap/

 OpenFABMAP is free software: you can redistribute it and/or modify it under
 the terms of the GNU General Public License as published by the Free Software
 Foundation, either version 3 of the License, or (at your option) any later
 version.

 OpenFABMAP is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 details.

 For published work which uses all or part of OpenFABMAP, please cite:
 http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=5509547&tag=1

 Original Algorithm by Mark Cummins and Paul Newman:
 http://ijr.sagepub.com/content/27/6/647.short
 http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=5613942
 http://ijr.sagepub.com/content/30/9/1100.abstract

 You should have received a copy of the GNU General Public License along with
 OpenFABMAP. If not, see http://www.gnu.org/licenses/.
------------------------------------------------------------------------*/

Author Contact [aj.glover@qut.edu.au]

version 2.01

please see  http://code.google.com/p/openfabmap/ for more information

OPENCV2.4 Compatibility
if using openCV2.4 you will need to replace
//#define OPENCV2P4
with
#define OPENCV2P4
at the beggining of openFABMAPcli.cpp

Installation Instructions (using Cmake)

Windows (Visual Studio 2008)

1. install openCV2.3 [http://opencv.willowgarage.com/wiki/]
2. install cmake [www.cmake.org/]
3. open the cmake gui, specify the source directory (the directory this README is in), a build directory for the code, and click configure
4. you may have to specify the location of opencv2.3 in UngroupedEntries->OPENCV_PATH.
5. click configure in the cmake gui again
6. click generate
7. open the visual studio solution, for default running right-click on openFABMAPexe project and select 'Set as StartUp project'. Compile openFABMAP within Visual studio.
8. add required .dll files from openCV2.3 to your build/bin directory (respective debug versions for debug mode).
9. you also may need an extra tbb .dll which can be downloaded here: [http://threadingbuildingblocks.org/ver.php?fid=171]
10. under openFABMAPcli->properties->Debugging->command arguments specify the path to the settings file (e.g. "-s samples\settings.yml")
11. Alter the settings file for your data
12. run exampleopenFABMAP in your build/bin directory (respective debug versions for debug mode).


Linux (g++)

1. install openCV2.3
2. get cmake and install it using your package manager
3. install cmakecurses using your package manager
2. make a build directory for your generated code
3. use the command line to change into this directory
4. run 'cmake /path/to/your/build/dir'
5. Hopefully openCV was found. If not, you may have to specify the directory manually using ccmake. Try using the wizard option cmake -i.
6. run 'make' in your build directory
5. Alter the settings file for your application
6. run openFABMAPcli in your build/bin directory




