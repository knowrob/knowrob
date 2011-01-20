
macro(fix_ros_destination_path var)
  # This macro iterates over all its parameters which should be
  # variable names and fixes the ros paths so that they point to the
  # post-install path and not the path during build time. This is
  # necessary because the debian build process builds packages at a
  # different location than their final destination.
  set(__ros_distro_dir "$ENV{ROS_DISTRO_DIR}")
  set(__ros_distro_dir_final "$ENV{ROS_DISTRO_DIR_FINAL}")

  if( __ros_distro_dir AND __ros_distro_dir_final  )
    # only replace if we are building a debian package
    string(REPLACE
      ${__ros_distro_dir} ${__ros_distro_dir_final}
      ${var} ${${var}})
  endif( __ros_distro_dir AND __ros_distro_dir_final )
endmacro(fix_ros_destination_path var)

