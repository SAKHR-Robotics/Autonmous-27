# ==============================================================================
# Autonomous Rover (Autonomous-27) Workspace Aliases & Helper Functions
# ==============================================================================

# Help banner function
help() {
  echo -e "\n\033[1;36m======================================================================\033[0m"
  echo -e "         \033[1;33m🤖 Autonomous Rover (Autonomous-27) Commands & Aliases\033[0m"
  echo -e "\033[1;36m======================================================================\033[0m"
  echo -e "  \033[1;32mhelp\033[0m / \033[1;32mal\033[0m    : Show this commands & aliases cheatsheet"
  echo -e "  \033[1;32mbld\033[0m          : Build all packages (colcon build --symlink-install)"
  echo -e "  \033[1;32mbldpkg <pkg>\033[0m  : Build only a specific package in seconds"
  echo -e "  \033[1;32mbldclean\033[0m      : Delete build, install, and log directories (clean wipe)"
  echo -e "  \033[1;32msros\033[0m          : Source ROS 2 Jazzy & Workspace overlay"
  echo -e "  \033[1;32msb\033[0m / \033[1;32msrc\033[0m      : Reload shell environment (source ~/.bashrc)"
  echo -e "  \033[1;32medital\033[0m        : Open ~/.bash_aliases to view or add shortcuts"
  echo -e "  \033[1;32msim\033[0m           : Launch Rover, Mars Yard World & Teleop GUI"
  echo -e "  \033[1;32mrviz\033[0m          : Launch RViz2 (preconfigured rover visualization)"
  echo -e "  \033[1;32mteleop\033[0m        : Launch keyboard teleoperation node"
  echo -e "\033[1;36m======================================================================\033[0m\n"
}

alias al='help'

# Shell Sourcing & Editing
alias sb='source ~/.bashrc && echo "✅ Shell environment reloaded!"'
alias src='source ~/.bashrc && echo "✅ Shell environment reloaded!"'
alias edital='nano ~/.bash_aliases'

# Git container safe directory
git config --global --add safe.directory /workspace 2>/dev/null

# Colcon Isolated Build Configuration
# Ensures build, install, and log are placed in /root/ros_build (outside /workspace)
mkdir -p /root/.colcon /root/ros_build/log
cat << 'EOF' > /root/.colcon/defaults.yaml
build:
  build-base: /root/ros_build/build
  install-base: /root/ros_build/install
  symlink-install: true
EOF

# ROS 2 & Colcon Build Aliases
alias bld='colcon --log-base /root/ros_build/log build --symlink-install --build-base /root/ros_build/build --install-base /root/ros_build/install'
alias bldpkg='colcon --log-base /root/ros_build/log build --symlink-install --build-base /root/ros_build/build --install-base /root/ros_build/install --packages-select'
unalias bldclean 2>/dev/null
bldclean() {
  rm -rf /root/ros_build/build /root/ros_build/install /root/ros_build/log 2>/dev/null
  echo "🧹 Cleaned build, install, and log contents from /root/ros_build!"
}
alias sros='source /opt/ros/jazzy/setup.bash && if [ -f /root/ros_build/install/setup.bash ]; then source /root/ros_build/install/setup.bash; elif [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && echo "✅ ROS 2 Jazzy and Workspace sourced!"'

# Quick Launch Shortcuts
alias sim='bash /workspace/testing/LunchWorld\&Rover.sh'
alias rviz='ros2 launch my_robot_description rviz_only.launch.py'
alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
