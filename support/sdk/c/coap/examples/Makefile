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

abs_builddir = /opt/tinyos/support/sdk/c/coap/examples
top_builddir = ..
top_srcdir = ..
# files and flags
PROGRAMS:=tiny coap-server coap-client
SOURCES:= tiny.c client.c server.c
OBJECTS:= $(patsubst %.c, %.o, $(SOURCES))
#CFLAGS:=-g -Wall -ansi -pedantic -I..
CFLAGS:=-g -Wall -I$(top_srcdir) -g -O2
DISTDIR?=$(top_builddir)/libcoap-0.07
FILES:=Makefile.in $(SOURCES)
LDFLAGS:=-L$(top_builddir) -lcoap 
libcoap =$(top_builddir)/libcoap.a
CFLAGS += -DSHOWREALVALUES

.PHONY: clean distclean

.SUFFIXES:
.SUFFIXES:      .c .o

all:	$(PROGRAMS)

check:	
	echo DISTDIR: $(DISTDIR)
	echo top_builddir: $(top_builddir)

tiny:	tiny.o $(libcoap)
	$(CC) -o $@ $< $(LDFLAGS)

coap-client: client.o $(libcoap)
	$(CC) -o $@ $< $(LDFLAGS)

coap-server:	server.o $(libcoap)
	$(CC) -o $@ $< $(LDFLAGS)

clean:
	@rm -f $(PROGRAMS) $(OBJECTS)

distclean:	clean
	@rm -rf $(DISTDIR)
	@rm -f *~ 

dist:	$(FILES)
	test -d $(DISTDIR)/examples || mkdir $(DISTDIR)/examples
	cp $(FILES) $(DISTDIR)/examples
