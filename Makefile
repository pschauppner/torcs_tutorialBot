##############################################################################
#
#    file                 : Makefile
#    created              : Thu Jun 8 20:31:55 CDT 2017
#    copyright            : (C) 2002 Paul Schauppner
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = tutorialBot
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp tutorialDriver.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml car1-stock1.rgb logo.rgb
SHIPSUBDIRS =

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-tutorialBot_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-tutorialBot_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
