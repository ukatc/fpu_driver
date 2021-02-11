#*****************************************************************************
# E.S.O. - VISTA project
#
# "@(#) $Id: Makefile 339118 2021-02-10 16:00:39Z sbeard $"
#
# Makefile of vci
#
# who       when      what
# --------  --------  ----------------------------------------------
# sbeard    07/02/08  Original for VIRCAM
# sbeard    01/07/19  First version for MOONS2 fibre positioner
#                     devices.
# sbeard    24/07/19  Added scripts for path analysis device.
#

#*****************************************************************************
# This Makefile follows VLT Standards (see Makefile(5) for more).
#*****************************************************************************
# REMARKS
#------------------------------------------------------------------------

#
# user definable C-compilation flags
USER_CFLAGS = -DLOG_LEVEL=5 

#
# additional include and library search paths
#USER_INC = 
#USER_LIB = 
MAKE_DOXYGEN=off

#
# MODULE CODE DESCRIPTION:
# ------------------------
# As a general rule:  public file are "cleaned" and "installed"  
#                     local (_L) are not "installed".

#
# C programs (public and local)
# -----------------------------
EXECUTABLES     =
EXECUTABLES_L   = 

#
# special compilation flags for single c sources
#yyyyy_CFLAGS   = 

#
# Includes (.h) files (public only)
# ---------------------------------
INCLUDES        = 

#
# Libraries (public and local)
# ----------------------------
LIBRARIES       =
LIBRARIES_L     =

#
# Scripts (public and local)
# ----------------------------
SCRIPTS         = 
SCRIPTS_L       =

# Python modules
PY_MODULES     = mock_controller \
	         fpu_sim \
	         codec \
	         protocol_constants

# Python scripts (public and local)
PY_SCRIPTS     = mock_gateway.py
PY_SCRIPTS_L   =

#
#
# TCL scripts (public and local)
# ------------------------------
TCL_SCRIPTS     = 
TCL_SCRIPTS_L   =

#
# <brief description of tttttt tcl-script>
tttttt_OBJECTS  =
tttttt_TCLSH    = 
tttttt_LIBS     = 

#
# TCL libraries (public and local)
# ------------------------------
TCL_LIBRARIES   =
TCL_LIBRARIES_L =

#
# <brief description of tttlll library>
tttlll_OBJECTS  = 

#
# UIF panels (public and local)
# ----------------------------
PANELS   =
PANELS_L = 

#
# On-Line Database Files
# ----------------------
DBL_CLASSES =
DBL_BCF =
DBL_FLAGS =
vcam_DBL_FLAGS =

#
# man pages to be done
# --------------------
MANSECTIONS =

#
# local man pages
# ---------------
MANl = $(SCRIPTS)

#
# ASCII file to be converted into Framemaker-MIF
# --------------------
ASCII_TO_MIF = 

#
# INS_ROOT files to be installed
#-------------------------------
INS_ROOT_FILES = 

#
# other files to be installed
#----------------------------
INSTALL_FILES =

#
# list of all possible C-sources (used to create automatic dependencies)
# ------------------------------
CSOURCENAMES = \
	$(foreach exe, $(EXECUTABLES) $(EXECUTABLES_L), $($(exe)_OBJECTS)) \
	$(foreach lib, $(LIBRARIES) $(LIBRARIES_L), $($(lib)_OBJECTS))

#
#>>>>> END OF standard rules

#
# CHOSE PLATFORM
# --------------
# Default is UNIX, for VxVorks application next line MUST BE UNCOMMENTED
#MAKE_VXWORKS = on   

#
# INCLUDE STANDARDS
# -----------------
MAKEDIR = $(VLTROOT)/include
include $(MAKEDIR)/vltMakefile

#
# TARGETS
# -------
all:	do_all
	@echo " . . . 'all' done" 

clean : clean_all
	@echo " . . . clean done"

man   : do_man 
	@echo " . . . man page(s) done"

install : install_all
	@echo " . . . installation done"

#___oOo___
