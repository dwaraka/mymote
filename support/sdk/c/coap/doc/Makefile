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
DOXYGEN= :

top_builddir = ..

DISTDIR?=$(top_builddir)/libcoap-0.07
FILES:=Makefile.in Doxyfile.in html

doc:	Doxyfile
	$(DOXYGEN) >./doxygen.out 2>&1 

clean:
	@rm -rf html

distclean:	clean
	@rm -rf $(DISTDIR)
	@rm -f *~ 

dist:	doc
	test -d $(DISTDIR)/doc || mkdir $(DISTDIR)/doc
	cp -r $(FILES) $(DISTDIR)/doc
