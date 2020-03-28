srcp2_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [[ "$(whoami)" == "user" ]]; then
  ###############################
  # In container commands       #
  ###############################
  function init() {
    cd "${srcp2_dir}"
    source "init_workspace.sh"
    catkin build
    source "../../devel/setup.bash"
  }

  # Print the ros enviorment vars
  function rosenv() {
    env | grep -i ros
  }

  # A two-line colored Bash prompt (PS1) with Git branch and a line decoration
  # which adjusts automatically to the width of the terminal.
  # Recognizes and shows Git, SVN and Fossil branch/revision.
  # Screenshot: http://img194.imageshack.us/img194/2154/twolineprompt.png
  # Michal Kottman, 2012

  RESET="\[\033[0m\]"
  RED="\[\033[0;31m\]"
  GREEN="\[\033[01;32m\]"
  BLUE="\[\033[01;34m\]"
  YELLOW="\[\033[0;33m\]"

  PS_LINE=`printf -- '- %.0s' {1..200}`
  function parse_git_branch {
    PS_BRANCH=''
    PS_FILL=${PS_LINE:0:$COLUMNS}
    if [ -d .svn ]; then
      PS_BRANCH="(svn r$(svn info|awk '/Revision/{print $2}'))"
      return
    elif [ -f _FOSSIL_ -o -f .fslckout ]; then
      PS_BRANCH="(fossil $(fossil status|awk '/tags/{print $2}'))"
      return
    fi
    ref=$(git symbolic-ref HEAD 2> /dev/null) || return
    PS_BRANCH="(git ${ref#refs/heads/})"
  }
  PROMPT_COMMAND="parse_git_branch"
  PS_INFO="$GREEN\u@\h$RESET:$BLUE\w"
  PS_GIT="$YELLOW\$PS_BRANCH"
  PS_TIME="\[\033[\$((COLUMNS-10))G\] $RED[\t]"
  export PS1="\${PS_FILL}\[\033[0G\]${PS_INFO} ${PS_GIT} ${PS_TIME}\n${RESET}\$ "

else
  ###############################
  # Out container commands      #
  ###############################
  function start_srcp2() {
    sudo -E "${srcp2_dir}/docker/run_srcp2.sh"
  }

  function stop_srcp2() {
    sudo -E "${srcp2_dir}/docker/stop_srcp2.sh"
  }

  function srcp2_sim() {
    sudo "${srcp2_dir}/submodules/srcp2-competitors/docker/scripts/launch/roslaunch_docker" $@
  }
fi
