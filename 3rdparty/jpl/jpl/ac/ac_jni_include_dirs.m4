dnl Modified by Peter Green <plugwash@p10link.net> to correct include paths
dnl with OpenJDK on all architectures other than i386 and amd64


dnl Original version is available from the GNU Autoconf Macro Archive at:
dnl http://www.gnu.org/software/ac-archive/htmldoc/ac_jni_include_dirs.html
dnl


AC_DEFUN([AC_JNI_INCLUDE_DIR],[

JNI_INCLUDE_DIRS=""

test "x$JAVAC" = x && AC_MSG_ERROR(['$JAVAC' undefined])

case "$JAVAC" in
    /*)	_ACJNI_JAVAC="$JAVAC"
	;;
     *) AC_PATH_PROG(_ACJNI_JAVAC, $JAVAC, no)
        ;;
esac

AC_PATH_PROG(_ACJNI_JAVAC, $JAVAC, no)
test "x$_ACJNI_JAVAC" = xno && AC_MSG_ERROR([$JAVAC could not be found in path])

_ACJNI_FOLLOW_SYMLINKS("$_ACJNI_JAVAC")
_JTOPDIR=`echo "$_ACJNI_FOLLOWED" | sed -e 's://*:/:g' -e 's:/[[^/]]*$::'`

case "$host_os" in
        darwin*)        _JTOPDIR=`echo "$_JTOPDIR" | sed -e 's:/[[^/]]*$::'`
                        _JINC="$_JTOPDIR/Headers";;
        *)              _JINC="$_JTOPDIR/include";;
esac
if test -f "$_JINC/jni.h"; then
        JNI_INCLUDE_DIRS="$JNI_INCLUDE_DIRS $_JINC"
else
        _JTOPDIR=`echo "$_JTOPDIR" | sed -e 's:/[[^/]]*$::'`
        if test -f "$_JTOPDIR/include/jni.h"; then
                JNI_INCLUDE_DIRS="$JNI_INCLUDE_DIRS $_JTOPDIR/include"
        else
                AC_MSG_ERROR([cannot find java include files])
        fi
fi

# get the likely subdirectories for system specific java includes
case "$host_os" in
bsdi*)          _JNI_INC_SUBDIRS="bsdos";;
linux*)         _JNI_INC_SUBDIRS="linux genunix";;
osf*)           _JNI_INC_SUBDIRS="alpha";;
solaris*)       _JNI_INC_SUBDIRS="solaris";;
*)              _JNI_INC_SUBDIRS="genunix";;
esac

# add any subdirectories that are present
for JINCSUBDIR in $_JNI_INC_SUBDIRS
do
        if test -d "$_JTOPDIR/include/$JINCSUBDIR"; then
                JNI_INCLUDE_DIRS="$JNI_INCLUDE_DIRS $_JTOPDIR/include/$JINCSUBDIR"
        fi
done

case "$host_cpu" in
	i?86)
		_JNI_LIBDIRS="lib/i386"
		_JNI_LIBSUBDIRS="client"
		;;
	x86_64)
		_JNI_LIBDIRS="lib/amd64"
		_JNI_LIBSUBDIRS="server"
		;;
	powerpc)
		case "$host_os" in
		linux*)
			_JNI_LIBDIRS="lib/ppc bin"
			_JNI_LIBSUBDIRS="server classic"
			;;
		*)
			_JNI_LIBDIRS=""
		esac
		;;
	*)	
		# Fallback option should work on all architectures except
		# amd64 and powerpc which are special cased above.
		_JNI_LIBDIRS="lib/$host_cpu"
		_JNI_LIBSUBDIRS="server"
esac

for d in $_JNI_LIBDIRS; do
	for subd in $_JNI_LIBSUBDIRS; do
		echo "Trying $_JTOPDIR/jre/$d/$subd"
		if test -d $_JTOPDIR/jre/$d/$subd; then
			JNI_CLIENT_DIRS="$JNI_CLIENT_DIRS $_JTOPDIR/jre/$d/$subd $_JTOPDIR/jre/$d"
		fi
	done
done

])

# _ACJNI_FOLLOW_SYMLINKS <path>
# Follows symbolic links on <path>,
# finally setting variable _ACJNI_FOLLOWED
# --------------------
AC_DEFUN([_ACJNI_FOLLOW_SYMLINKS],[
# find the include directory relative to the javac executable
_cur="$1"
while ls -ld "$_cur" 2>/dev/null | grep " -> " >/dev/null; do
        AC_MSG_CHECKING(symlink for $_cur)
        _slink=`ls -ld "$_cur" | sed 's/.* -> //'`
        case "$_slink" in
        /*) _cur="$_slink";;
        # 'X' avoids triggering unwanted echo options.
        *) _cur=`echo "X$_cur" | sed -e 's/^X//' -e 's:[[^/]]*$::'`"$_slink";;
        esac
        AC_MSG_RESULT($_cur)
done
_ACJNI_FOLLOWED="$_cur"
])# _ACJNI
