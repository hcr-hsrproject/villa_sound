dnl Process this file with autoconf to produce a configure script. -*-m4-*-

AC_PREREQ([2.65])
AC_INIT([hark-villa], [0.1], no-define, [hark-villa])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([src/VERSION.cc])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([Makefile src/Makefile])
AX_CXX_COMPILE_STDCXX(11, [], [mandatory])
AM_INIT_AUTOMAKE
LT_INIT

dnl HARK PACKAGE NAME and VERSION
HARK_MAJOR_VERSION=2
HARK_MINOR_VERSION=3
HARK_MICRO_VERSION=0
HARK_VERSION=$HARK_MAJOR_VERSION.$HARK_MINOR_VERSION.$HARK_MICRO_VERSION
HARK_INTERFACE_AGE=0
HARK_BINARY_AGE=0
PACKAGE=hark-villa
VERSION=$HARK_VERSION
AC_SUBST([HARK_MAJOR_VERSION])
AC_SUBST([HARK_MINOR_VERSION])
AC_SUBST([HARK_MICRO_VERSION])
AC_SUBST([HARK_VERSION])
AC_SUBST([HARK_INTERFACE_AGE])
AC_SUBST([HARK_BINARY_AGE])

dnl libtool versioning
LT_RELEASE=$HARK_MAJOR_VERSION.$HARK_MINOR_VERSION.$HARK_MICRO_VERSION
LT_CURRENT=`expr $HARK_MICRO_VERSION - $HARK_INTERFACE_AGE`
LT_REVISION=$HARK_INTERFACE_AGE
LT_AGE=`expr $HARK_BINARY_AGE - $HARK_INTERFACE_AGE`
AC_SUBST([LT_RELEASE])
AC_SUBST([LT_CURRENT])
AC_SUBST([LT_REVISION])
AC_SUBST([LT_AGE])

dnl Set mic channels and length
AC_ARG_VAR([MIC_CHANNELS], [microphone channels])
AM_CONDITIONAL([MIC_CHANNELS_SET], [test ! -z "$MIC_CHANNELS"])
AM_COND_IF([MIC_CHANNELS_SET], [], [
    AC_MSG_ERROR(["Error: Must set MIC_CHANNELS])
])
AC_DEFINE_UNQUOTED([MIC_CHANNELS], [$MIC_CHANNELS], [mic channels])

AC_ARG_VAR([MIC_FRAME_LENGTH], [microphone frame length])
AM_CONDITIONAL([MIC_FRAME_LENGTH_SET], [test ! -z "$MIC_FRAME_LENGTH"])
AM_COND_IF([MIC_FRAME_LENGTH_SET], [], [
    AC_MSG_ERROR(["Error: Must set MIC_FRAME_LENGTH])
])
AC_DEFINE_UNQUOTED([MIC_FRAME_LENGTH], [$MIC_FRAME_LENGTH], [mic frame length])


dnl TAGS VERSION and SVN REVISION
echo 'Searching SVN Revision Recursively ...'
SVN_REVISION=`\`flowdesigner-config --bindir\`/fdmodule_version.pl --printsvn`
AC_DEFINE_UNQUOTED([SVN_REVISION], "${SVN_REVISION}", [svn revision number])
AC_DEFINE_UNQUOTED([TAGS_VERSION], "${HARK_VERSION}", [tags version number])
echo "Found Revision: ${SVN_REVISION}"

dnl autoheader template was migrated from acconfig.h
AH_TEMPLATE([ENABLE_ROS], [])

AM_MAINTAINER_MODE

AC_CANONICAL_HOST

AC_PROG_CXX
AC_PROG_F77
AC_LANG([C++])
AC_PROG_MAKE_SET
AC_C_BIGENDIAN

dnl boost_thread judgement
BOOST_THREAD_JUDGE([1.46.0])

AC_PATH_PROG(FLOWDESIGNER_CONFIG,flowdesigner-config,no,$PATH:$FLOWDESIGNER_HOME/bin:$prefix/bin)
if test "$FLOWDESIGNER_CONFIG" = no; then
  AC_MSG_ERROR([Cannot find flowdesigner-config])
fi

FLOWDESIGNER_LIB=`$FLOWDESIGNER_CONFIG --libs libflow`
FLOWDESIGNER_INCLUDE=`$FLOWDESIGNER_CONFIG --cflags config libflow | sed 's/\"/\\\"/g'`
FLOWDESIGNER_DATA=`$FLOWDESIGNER_CONFIG --datadir`/$PACKAGE
FLOWDESIGNER_BIN=`$FLOWDESIGNER_CONFIG --bindir`

if test "x$prefix" = "xNONE"; then
  echo $FLOWDESIGNER_DATA
else
  FLOWDESIGNER_DATA=$prefix/lib/flowdesigner/toolbox/$PACKAGE
fi

AC_SUBST([FLOWDESIGNER_LIB])
AC_SUBST([FLOWDESIGNER_INCLUDE])
AC_SUBST([FLOWDESIGNER_DATA])
AC_SUBST([FLOWDESIGNER_BIN])

PKG_CHECK_MODULES(GTK,gtk+-2.0)
AC_SUBST([GTK_CFLAGS])
AC_SUBST([GTK_LIBS])

PKG_CHECK_MODULES([LIBHARKIO3], [libharkio3], [have_libharkio3=yes], [have_libharkio3=no])
CPPFLAGS="${CPPFLAGS} `pkg-config libharkio3 --cflags-only-I`"
LDFLAGS="${LDFLAGS} `pkg-config libharkio3 --libs-only-L`"
PKG_CHECK_MODULES([LIBZIP], [libzip],
[
   CPPFLAGS="${CPPFLAGS} `pkg-config libzip --cflags-only-I`"
],
[ 
   AC_MSG_ERROR(["Error: libzip not found])
])

dnl add HARK include path
AC_ARG_WITH(
  [hark-inc],
  [  --with-hark-inc=DIR          location of HARK include files],
  [CPPFLAGS="${CPPFLAGS} -I ${withval}"]
)

dnl add ROS include path
AC_ARG_ENABLE(
  [ros],
  [  --enable-ros         enable ROS],
  [
    case "${enableval}" in
      yes) ros=true ;;
      no)  ros=false ;;
      *) AC_MSG_ERROR([bad value ${enableval} for --enable-ros]) ;;
    esac
  ],
  [ros=false]
)
AM_CONDITIONAL([ENABLE_ROS], [test x$ros = xtrue])
if test "$enable_ros" = "yes"; then
  AC_DEFINE([ENABLE_ROS])
  if echo "`rosstack find ros`" | grep "cturtle"; then
    echo "ros cturtle cflags found"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roscpp`/include -I`rospack find roscpp`/srv/cpp -I`rospack find roscpp`/msg/cpp -I`rospack find roscpp`/msg_gen/cpp/include -I`rospack find roscpp`/srv_gen/cpp/include"   
    CPPFLAGS="${CPPFLAGS} -I`rospack find xmlrpcpp`/src"
    CPPFLAGS="${CPPFLAGS} -I`rospack find rosconsole`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roslib`/msg/cpp -I`rospack find roslib`/include -I`rospack find roslib`/msg_gen/cpp/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find rospack` -I`rospack find rospack`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find std_msgs`/include -I`rospack find std_msgs`/msg/cpp -I`rospack find std_msgs`/srv/cpp -I`rospack find std_msgs`/msg_gen/cpp/include -I`rospack find std_msgs`/srv_gen/cpp/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find dynamic_reconfigure`/include -I`rospack find dynamic_reconfigure`/srv/cpp -I`rospack find dynamic_reconfigure`/msg/cpp -I`rospack find dynamic_reconfigure`/msg_gen/cpp/include -I`rospack find dynamic_reconfigure`/srv_gen/cpp/include"
  fi
  if echo "`rosstack find ros`" | grep "diamondback"; then
    echo "ros diamondback cflags found"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roscpp`/include -I`rospack find roscpp`/srv/cpp -I`rospack find roscpp`/msg/cpp -I`rospack find roscpp`/msg_gen/cpp/include -I`rospack find roscpp`/srv_gen/cpp/include"   
    CPPFLAGS="${CPPFLAGS} -I`rospack find rostime`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find cpp_common`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roscpp_serialization`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roscpp_traits`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find rosgraph_msgs`/msg_gen/cpp/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find xmlrpcpp`/src"
    CPPFLAGS="${CPPFLAGS} -I`rospack find rosconsole`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roslib`/msg/cpp -I`rospack find roslib`/include -I`rospack find roslib`/msg_gen/cpp/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find rospack` -I`rospack find rospack`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find std_msgs`/include -I`rospack find std_msgs`/msg/cpp -I`rospack find std_msgs`/srv/cpp -I`rospack find std_msgs`/msg_gen/cpp/include -I`rospack find std_msgs`/srv_gen/cpp/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find dynamic_reconfigure`/include -I`rospack find dynamic_reconfigure`/srv/cpp -I`rospack find dynamic_reconfigure`/msg/cpp -I`rospack find dynamic_reconfigure`/msg_gen/cpp/include -I`rospack find dynamic_reconfigure`/srv_gen/cpp/include"
  fi
  if echo "`rosstack find ros`" | grep "electric"; then
    echo "ros electric cflags found"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roscpp`/include -I`rospack find roscpp`/srv/cpp -I`rospack find roscpp`/msg/cpp -I`rospack find roscpp`/msg_gen/cpp/include -I`rospack find roscpp`/srv_gen/cpp/include"   
    CPPFLAGS="${CPPFLAGS} -I`rospack find rostime`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find cpp_common`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roscpp_serialization`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roscpp_traits`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find rosgraph_msgs`/msg_gen/cpp/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find xmlrpcpp`/src"
    CPPFLAGS="${CPPFLAGS} -I`rospack find rosconsole`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find roslib`/msg/cpp -I`rospack find roslib`/include -I`rospack find roslib`/msg_gen/cpp/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find rospack` -I`rospack find rospack`/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find std_msgs`/include -I`rospack find std_msgs`/msg/cpp -I`rospack find std_msgs`/srv/cpp -I`rospack find std_msgs`/msg_gen/cpp/include -I`rospack find std_msgs`/srv_gen/cpp/include"
    CPPFLAGS="${CPPFLAGS} -I`rospack find dynamic_reconfigure`/include -I`rospack find dynamic_reconfigure`/srv/cpp -I`rospack find dynamic_reconfigure`/msg/cpp -I`rospack find dynamic_reconfigure`/msg_gen/cpp/include -I`rospack find dynamic_reconfigure`/srv_gen/cpp/include"
  fi
  if echo "`rosstack find ros`" | grep "fuerte"; then
    echo "ros fuerte cflags found"
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rostime --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config cpp_common --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp_serialization --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp_traits --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rosgraph_msgs --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config xmlrpcpp --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rosconsole --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config roslib --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config rospack --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config std_msgs --cflags`"
    CPPFLAGS="${CPPFLAGS} -I`rospack find dynamic_reconfigure`/include -I`rospack find dynamic_reconfigure`/srv/cpp -I`rospack find dynamic_reconfigure`/msg/cpp -I`rospack find dynamic_reconfigure`/msg_gen/cpp/include -I`rospack find dynamic_reconfigure`/srv_gen/cpp/include"
  fi
  if echo "`rosversion -d`" | grep "groovy"; then
    echo "ros groovy cflags found"
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rostime --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config cpp_common --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp_serialization --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp_traits --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rosgraph_msgs --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config xmlrpcpp --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rosconsole --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config roslib --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config rospack --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config std_msgs --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config dynamic_reconfigure --cflags`"
  fi
  if echo "`rosversion -d`" | grep "hydro"; then
    echo "ros hydro cflags found"
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rostime --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config cpp_common --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp_serialization --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp_traits --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rosgraph_msgs --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config xmlrpcpp --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rosconsole --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config roslib --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config rospack --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config std_msgs --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config dynamic_reconfigure --cflags`"
  fi
  if echo "`rosversion -d`" | grep "indigo"; then
    echo "ros indigo cflags found"
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rostime --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config cpp_common --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp_serialization --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config roscpp_traits --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rosgraph_msgs --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config xmlrpcpp --cflags`"      
    CPPFLAGS="${CPPFLAGS} `pkg-config rosconsole --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config roslib --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config rospack --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config std_msgs --cflags`"
    CPPFLAGS="${CPPFLAGS} `pkg-config dynamic_reconfigure --cflags`"
  fi
  CPPFLAGS="${CPPFLAGS} -I`rospack find hark_msgs`/msg/cpp -I`rospack find hark_msgs`/srv/cpp -I`rospack find hark_msgs`/msg_gen/cpp/include -I`rospack find hark_msgs`/srv_gen/cpp/include"
  CPPFLAGS="${CPPFLAGS} -I`rospack find hark_params`/include -I`rospack find hark_params`/srv/cpp -I`rospack find hark_params`/msg/cpp -I`rospack find hark_params`/msg_gen/cpp/include -I`rospack find hark_params`/srv_gen/cpp/include -I`rospack find hark_params`/cfg/cpp"

  if echo "`rosstack find ros`" | grep "cturtle"; then
    echo "ros cturtle libs found"
    LDFLAGS="${LDFLAGS} -L`rospack find roscpp`/lib -lros -Wl,-rpath,`rospack find roscpp`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find xmlrpcpp`/lib -lXmlRpc -Wl,-rpath,`rospack find xmlrpcpp`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find rosconsole`/lib -lrosconsole -Wl,-rpath,`rospack find rosconsole`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find roslib`/lib -lroslib -Wl,-rpath,`rospack find roslib`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find rospack`/lib -lrospack -lrosstack -Wl,-rpath,`rospack find rospack`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find dynamic_reconfigure`/lib -ldynamic_reconfigure_config_init_mutex -Wl,-rpath,`rospack find dynamic_reconfigure`/lib"
  fi
  if echo "`rosstack find ros`" | grep "diamondback"; then
    echo "ros diamondback libs found"
    LDFLAGS="${LDFLAGS} -L`rospack find roscpp`/lib -lros -Wl,-rpath,`rospack find roscpp`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find rostime`/lib -lrostime -Wl,-rpath,`rospack find rostime`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find cpp_common`/lib -lcpp_common -Wl,-rpath,`rospack find cpp_common`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find roscpp_serialization`/lib -lroscpp_serialization -Wl,-rpath,`rospack find roscpp_serialization`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find xmlrpcpp`/lib -lXmlRpc -Wl,-rpath,`rospack find xmlrpcpp`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find rosconsole`/lib -lrosconsole -Wl,-rpath,`rospack find rosconsole`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find roslib`/lib -lroslib -Wl,-rpath,`rospack find roslib`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find rospack`/lib -lrospack -lrosstack -Wl,-rpath,`rospack find rospack`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find dynamic_reconfigure`/lib -ldynamic_reconfigure_config_init_mutex -Wl,-rpath,`rospack find dynamic_reconfigure`/lib"
  fi
  if echo "`rosstack find ros`" | grep "electric"; then
    echo "ros electric libs found"
    LDFLAGS="${LDFLAGS} -L`rospack find roscpp`/lib -lros -Wl,-rpath,`rospack find roscpp`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find rostime`/lib -lrostime -Wl,-rpath,`rospack find rostime`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find cpp_common`/lib -lcpp_common -Wl,-rpath,`rospack find cpp_common`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find roscpp_serialization`/lib -lroscpp_serialization -Wl,-rpath,`rospack find roscpp_serialization`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find xmlrpcpp`/lib -lXmlRpc -Wl,-rpath,`rospack find xmlrpcpp`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find rosconsole`/lib -lrosconsole -Wl,-rpath,`rospack find rosconsole`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find roslib`/lib -lroslib -Wl,-rpath,`rospack find roslib`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find rospack`/lib -lrospack -lrosstack -Wl,-rpath,`rospack find rospack`/lib"
    LDFLAGS="${LDFLAGS} -L`rospack find dynamic_reconfigure`/lib -ldynamic_reconfigure_config_init_mutex -Wl,-rpath,`rospack find dynamic_reconfigure`/lib"
  fi
  if echo "`rosstack find ros`" | grep "fuerte"; then
    echo "ros fuerte libs found"
    LDFLAGS="${LDFLAGS} `pkg-config roscpp --libs`"      
    LDFLAGS="${LDFLAGS} `pkg-config rostime --libs`"      
    LDFLAGS="${LDFLAGS} `pkg-config cpp_common --libs`"      
    LDFLAGS="${LDFLAGS} `pkg-config roscpp_serialization --libs`"
    LDFLAGS="${LDFLAGS} `pkg-config xmlrpcpp --libs`"
    LDFLAGS="${LDFLAGS} `pkg-config rosconsole --libs`"
    LDFLAGS="${LDFLAGS} `pkg-config roslib --libs`"
    LDFLAGS="${LDFLAGS} `pkg-config rospack --libs`"
    LDFLAGS="${LDFLAGS} -L`rospack find dynamic_reconfigure`/lib -ldynamic_reconfigure_config_init_mutex -Wl,-rpath,`rospack find dynamic_reconfigure`/lib"
  fi
  if echo "`rosversion -d`" | grep "groovy"; then
    echo "ros groovy libs found"
    LDFLAGS="${LDFLAGS} `pkg-config roscpp --libs-only-L` -lroscpp"      
    LDFLAGS="${LDFLAGS} `pkg-config rostime --libs-only-L` -lrostime"      
    LDFLAGS="${LDFLAGS} `pkg-config cpp_common --libs-only-L` -lcpp_common"      
    LDFLAGS="${LDFLAGS} `pkg-config roscpp_serialization --libs-only-L` -lroscpp_serialization"
    LDFLAGS="${LDFLAGS} `pkg-config xmlrpcpp --libs-only-L` -lxmlrpcpp"
    LDFLAGS="${LDFLAGS} `pkg-config rosconsole --libs-only-L` -lrosconsole"
    LDFLAGS="${LDFLAGS} `pkg-config roslib --libs-only-L` -lroslib"
    LDFLAGS="${LDFLAGS} `pkg-config rospack --libs-only-L` -lrospack -ltinyxml"
    LDFLAGS="${LDFLAGS} -lpython2.7 -lboost_filesystem-mt -lboost_program_options-mt -lboost_system-mt"
    LDFLAGS="${LDFLAGS} `pkg-config dynamic_reconfigure --libs-only-L` -ldynamic_reconfigure_config_init_mutex -lroscpp_serialization -lrostime"
dnl    LDFLAGS="${LDFLAGS} -lboost_date_time-mt -lboost_system-mt -lboost_thread -lpthread"
    LDFLAGS="${LDFLAGS} -lboost_date_time-mt -lboost_system-mt $BOOST_THREAD_JUDGE_LIB -lpthread"
  fi
  if echo "`rosversion -d`" | grep "hydro"; then
    echo "ros hydro libs found"
    LDFLAGS="${LDFLAGS} `pkg-config roscpp --libs-only-L` -lroscpp"      
    LDFLAGS="${LDFLAGS} `pkg-config rostime --libs-only-L` -lrostime"      
    LDFLAGS="${LDFLAGS} `pkg-config cpp_common --libs-only-L` -lcpp_common"      
    LDFLAGS="${LDFLAGS} `pkg-config roscpp_serialization --libs-only-L` -lroscpp_serialization"
    LDFLAGS="${LDFLAGS} `pkg-config xmlrpcpp --libs-only-L` -lxmlrpcpp"
    LDFLAGS="${LDFLAGS} `pkg-config rosconsole --libs-only-L` -lrosconsole"
    LDFLAGS="${LDFLAGS} `pkg-config roslib --libs-only-L` -lroslib"
    LDFLAGS="${LDFLAGS} `pkg-config rospack --libs-only-L` -lrospack -ltinyxml"
    LDFLAGS="${LDFLAGS} -lpython2.7 -lboost_filesystem-mt -lboost_program_options-mt -lboost_system-mt"
    LDFLAGS="${LDFLAGS} `pkg-config dynamic_reconfigure --libs-only-L` -ldynamic_reconfigure_config_init_mutex -lroscpp_serialization -lrostime"
dnl    LDFLAGS="${LDFLAGS} -lboost_date_time-mt -lboost_system-mt -lboost_thread -lpthread"
    LDFLAGS="${LDFLAGS} -lboost_date_time-mt -lboost_system-mt $BOOST_THREAD_JUDGE_LIB -lpthread"
  fi
  if echo "`rosversion -d`" | grep "indigo"; then
    echo "ros indigo libs found"
    LDFLAGS="${LDFLAGS} `pkg-config roscpp --libs-only-L` -lroscpp"      
    LDFLAGS="${LDFLAGS} `pkg-config rostime --libs-only-L` -lrostime"      
    LDFLAGS="${LDFLAGS} `pkg-config cpp_common --libs-only-L` -lcpp_common"      
    LDFLAGS="${LDFLAGS} `pkg-config roscpp_serialization --libs-only-L` -lroscpp_serialization"
    LDFLAGS="${LDFLAGS} `pkg-config xmlrpcpp --libs-only-L` -lxmlrpcpp"
    LDFLAGS="${LDFLAGS} `pkg-config rosconsole --libs-only-L` -lrosconsole"
    LDFLAGS="${LDFLAGS} `pkg-config roslib --libs-only-L` -lroslib"
    LDFLAGS="${LDFLAGS} `pkg-config rospack --libs-only-L` -lrospack -ltinyxml"
dnl    LDFLAGS="${LDFLAGS} -lpython2.7 -lboost_filesystem-mt -lboost_program_options-mt -lboost_system-mt"
    LDFLAGS="${LDFLAGS} `pkg-config dynamic_reconfigure --libs-only-L` -ldynamic_reconfigure_config_init_mutex -lroscpp_serialization -lrostime"
dnl    LDFLAGS="${LDFLAGS} -lboost_date_time-mt -lboost_system-mt -lboost_thread -lpthread"
dnl    LDFLAGS="${LDFLAGS} -lboost_date_time-mt -lboost_system-mt $BOOST_THREAD_JUDGE_LIB -lpthread"
  fi
dnl  LDFLAGS="${LDFLAGS} -lboost_thread -lboost_signals-mt -lboost_thread -llog4cxx"
  LDFLAGS="${LDFLAGS} $BOOST_THREAD_JUDGE_LIB -llog4cxx"
dnl  LDFLAGS="${LDFLAGS} -lboost_signals-mt"
fi

dnl add KYOTO ROS include path
AC_ARG_ENABLE(
  [kros],
  [  --enable-kros         enable KYOTO ROS],
  [
    case "${enableval}" in
      yes) kros=true ;;
      no)  kros=false ;;
      *) AC_MSG_ERROR([bad value ${enableval} for --enable-kros]) ;;
    esac
  ],
  [kros=false]
)
AM_CONDITIONAL([ENABLE_KROS], [test x$kros = xtrue])
if test "$enable_kros" = "yes"; then
   AC_DEFINE([ENABLE_KROS], [], [Description])
   CPPFLAGS="${CPPFLAGS} -I`rospack find thereminist`/msg_gen/cpp/include"
   CPPFLAGS="${CPPFLAGS} -I`rospack find audioFeatures`/msg_gen/cpp/include"
   CPPFLAGS="${CPPFLAGS} -I`rospack find trackerCombination`/msg_gen/cpp/include"
fi

AC_OUTPUT

dnl Stolen from gtk+:

dnl ncludedir=$includedir/flowdesigner
dnl atadir=$datadir/flowdesigner
dnl ibdir=$datadir/lib

dnl AC_OVERFLOW_CHECKS
dnl LDFLAGS=`flowdesigner-config config libflow audio_blocks --libs`
dnl CXXFLAGS=`flowdesigner-config config libflow audio_blocks --cflags`

dnl ## legacy arguments
dnl AM_CONFIG_HEADER(config.h)
dnl AM_INIT_AUTOMAKE($PACKAGE, $VERSION, no-define)

dnl ## already included libtool macros in m4 search path
dnl AC_DISABLE_STATIC
dnl AM_PROG_LIBTOOL

dnl AM_C_PROTOTYPES

dnl ## replaced to AC_LANG([C++])
dnl AC_LANG_CPLUSPLUS

dnl AC_INST_EXTENSIONS

dnl AM_PATH_GTK_2_0(2.0.0,,AC_MSG_ERROR([GTK+ is required.]))

