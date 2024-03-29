# Makefile for libcoap
#
# Copyright (C) 2010 Olaf Bergmann <bergmann@tzi.org>
# 
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

# the library's version
VERSION:=0.07

# tools

SHELL = /bin/sh
MKDIR = mkdir

abs_builddir = /opt/tinyos/support/sdk/c/coap
top_builddir = .
package = libcoap-0.07

# files and flags
SOURCES:= pdu.c net.c debug.c encode.c uri.c list.c subscribe.c str.c
OBJECTS:= $(patsubst %.c, %.o, $(SOURCES))
HEADERS:=coap.h config.h debug.h pdu.h net.h encode.h uri.h list.h mem.h subscribe.h str.h 
#CFLAGS:=-g -Wall -ansi -pedantic
CFLAGS:=-Wall -ansi -pedantic -g -O2
DISTDIR=$(top_builddir)/$(package)
SUBDIRS:=examples doc
FILES:=Makefile.in configure configure.in config.h.in $(SOURCES) $(HEADERS)
LIB:=libcoap.a
LDFLAGS:=
ARFLAGS:=cru
examples:=examples
doc:=doc


.PHONY: all dirs clean distclean .gitignore doc

.SUFFIXES:
.SUFFIXES:      .c .o

all:	$(LIB) dirs

check:	
	echo DISTDIR: $(DISTDIR)
	echo top_builddir: $(top_builddir)
	$(MAKE) -C examples check

dirs:	$(SUBDIRS)
	for dir in $^; do \
		$(MAKE) -C $$dir ; \
	done

$(LIB):	$(OBJECTS)
	$(AR) $(ARFLAGS) $@ $^ 
	ranlib $@

clean:
	@rm -f $(PROGRAM) main.o $(LIB) $(OBJECTS)
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean ; \
	done

doc:	
	$(MAKE) -C doc

distclean:	clean
	@rm -rf $(DISTDIR)
	@rm -f *~ $(DISTDIR).tar.gz

dist:	$(FILES) $(SUBDIRS)
	test -d $(DISTDIR) || mkdir $(DISTDIR)
	cp $(FILES) $(DISTDIR)
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir dist; \
	done
	tar czf $(package).tar.gz $(DISTDIR)

.gitignore:
	echo "core\n*~\n*.[oa]\n*.gz\n*.cap\n$(PROGRAM)\n$(DISTDIR)\n.gitignore" >$@
