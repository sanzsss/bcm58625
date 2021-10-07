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
# PHYMOD default make rules. These can optionally be overridden by
# letting PHYMOD_MAKE_RULES point to a different rules file.
#

ifdef PHYMOD_MAKE_RULES

include $(PHYMOD_MAKE_RULES)

else

targetlibsoname = $(LIBNAME).so.${SHAREDLIBVER}
targetlibrealname = ${targetlibsoname}
targetlibso = ${LIBDIR}/${targetlibrealname}

$(BLDDIR)/%.$(OBJSUFFIX): %.c $(BLDDIR)/.tree
	@$(ECHO) 'Compiling $(LOCALDIR)/$<'
	$(Q)$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(LIBDIR)/$(LIBNAME).$(LIBSUFFIX): $(BOBJS) $(BLDDIR)/.tree 
	@$(ECHO) 'Building library $(LIBNAME)...'
	$(Q)$(AR) $(ARFLAGS) $@ $(BOBJS)
ifeq ($(LINUX_MAKE_SHARED_LIB),1)
	$(CC) -shared -Wl,-soname,${targetlibsoname} -o ${targetlibso} ${BOBJS} -lc -lnsl -lpthread -lm -lrt
endif # LINUX_MAKE_SHARED_LIB #
endif
