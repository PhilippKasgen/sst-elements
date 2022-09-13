# -*- Makefile -*-
#
#

AM_CPPFLAGS += \
	$(MPI_CPPFLAGS) \
	-I$(top_srcdir)/src

compdir = $(pkglibdir)
comp_LTLIBRARIES = libmemHierarchyTester.la

libmemHierarchyTester_la_SOURCES = \
	Check.h \
	Check.cc \
	CheckTable.h \
	CheckTable.cc \
	memHierarchyTester.h \
	memHierarchyTester.cc

EXTRA_DIST = \
	test/utils.py \
	test/test.py \
	test/base.cfg

sstdir = $(includedir)/sst/elements/memHierarchyTester
nobase_sst_HEADERS = \
	Check.h \
	CheckTable.h


install-exec-hook:
	$(SST_REGISTER_TOOL) SST_ELEMENT_SOURCE     memHierarchyTester=$(abs_srcdir)
#	$(SST_REGISTER_TOOL) SST_ELEMENT_TESTS      memHierarchyTester=$(abs_srcdir)/test
