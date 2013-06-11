ACLOCAL_AMFLAGS = -I m4

if HAVE_CURL
  OPT_EXAMPLES=examples
else
  OPT_EXAMPLES=
endif

# Order is important:
SUBDIRS = third_party src testdata $(OPT_EXAMPLES) msvc xcode

EXTRA_DIST = \
	AUTHORS \
	COPYING \
	DEPENDENCIES
