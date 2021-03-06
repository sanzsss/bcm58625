# $Id$
# Copyright © 2016 Broadcom.
#The term “Broadcom” refers to Broadcom Limited and/or its
#subsidiaries.
#
#This program is free software; you can redistribute it and/or
#modify it under the terms of the GNU General Public License as
#published by the Free Software Foundation version 2.
#
#This program is distributed "as is" WITHOUT ANY WARRANTY of any
#kind, whether express or implied; without even the implied warranty
#of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.
#
# PHYMOD make tools
#

# Compiler command for generating dependencies
PHYMOD_DEPEND ?= $(CC) -M -P $(CPPFLAGS) $< 

# Perl is required for portable build tools
ifndef PHYMOD_PERL
PHYMOD_PERL = perl
endif

ifeq (n/a,$(PHYMOD_PERL))
#
# If perl is not available, try building using standard UNIX utilities
#
RM      = rm -f
MKDIR   = mkdir -p
CP      = cp -d
ECHO    = echo
PRINTF  = printf
else
#
# If perl is available, use portable build tools
#
MKTOOL  = $(PHYMOD_PERL) $(PHYMOD)/make/mktool.pl
RM      = $(MKTOOL) -rm
MKDIR   = $(MKTOOL) -md
FOREACH = $(MKTOOL) -foreach
CP      = $(MKTOOL) -cp
MAKEDEP = $(MKTOOL) -dep 
ECHO    = $(MKTOOL) -echo
PRINTF  = $(MKTOOL) -echo -n
MKBEEP  = $(MKTOOL) -beep
endif
