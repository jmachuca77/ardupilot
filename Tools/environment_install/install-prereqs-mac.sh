#!/bin/bash

function progress {
    tput setaf 2
    echo $*
    tput sgr0
}

progress Checking homebrew...
$(which -s brew)
if [[ $? != 0 ]] ; then
    progress installing homebrew...
    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
else
    progress Homebrew installed
fi

progress Installing Xcode Command Line Tools
#Install Xcode Dependencies
xcode-select --install

progress Installing Homebrew Packages
brew tap ardupilot/homebrew-ardupilot
brew update
brew install pyenv
brew install coreutils
brew install gcc-arm-none-eabi
brew install gawk

progress Setting up pyenv...
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bash_profile
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bash_profile
echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> ~/.bash_profile

echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.zshrc
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.zshrc
echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> ~/.zshrc

source $HOME/.zshrc

pyenv init

progress Installing Python 3.9.1 and setting it as pyenv global
env PYTHON_CONFIGURE_OPTS="--enable-framework" pyenv install 3.9.1
pyenv global 3.9.1

# May need to reload shell or call pyenv init 
pyenv versions
python --version

progress Checking pip...
$(which -s pip)
if [[ $? != 0 ]] ; then
    progress Installing pip...
    # Easy install does not support python 2.x anymore
    # sudo easy_install pip
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
    sudo -H python get-pip.py
else
    progress pip already installed checking for updates...
    sudo -H pip install --upgrade pip
    sudo -H pip install --upgrade setuptools
fi

progress pip installed, version:
pip --version

progress Installing brew version of wxpython
brew install wxpython

progress Installing python packages...
pip install wheel
pip install --user pyserial future empy mavproxy pexpect

pythonpath="export PATH=$HOME/Library/Python/3.9/bin:\$PATH";
echo $pythonpath >> ~/.bash_profile
echo $pythonpath >> ~/.zshrc
eval $pythonpath

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")
ARDUPILOT_TOOLS="Tools/autotest"

exportline="export PATH=$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH";

grep -Fxq "$exportline" ~/.bash_profile 2>/dev/null || {
   read -p "`tput setaf 11`Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [N/y]?`tput sgr0`" -n 1 -r
   if [[ $REPLY =~ ^[Yy]$ ]] ; then
       echo $exportline >> ~/.bash_profile
       echo $exportline >> ~/.zshrc
       eval $exportline
   else
       progress Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH.
   fi
}

git submodule update --init --recursive

echo "`tput setaf 11`Finished!, please restart your terminal for changes to take effect.`tput sgr0`"
