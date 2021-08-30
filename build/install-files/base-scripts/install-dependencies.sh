#!/bin/bash

# Bash will exit on first error
set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"

# These should already be set...

! [ "$IS_UBUNTU" = "1" ] && ! [ "$IS_FEDORA" = "1" ] \
    && echo "Failed to determine distribution, aborting" \
    && exit 1
[ "$IS_UBUNTU" = "1" ] && [ "$IS_FEDORA" = "1" ] \
    && echo "Cannot be both Ubuntu and Fedora at the same time!" \
    && exit 1

# --------------------------------------------------------- Install dependencies

install_distro_dependencies()
{
    [ "$IS_UBUNTU" = "1" ] && \
        $PPWD/install-dependencies_ubuntu.sh && return 0
    [ "$IS_FEDORA" = "1" ] && \
        $PPWD/install-dependencies_fedora.sh && return 0
    return 1
}

# ----------------------------------------------- Install pip3 and pip3 packages

install_pip3_and_pip3_packages()
{
    if [ "$IS_UBUNTU" = "1" ] ; then
        sudo apt-get -y remove python3-pip
        sudo apt-get -y install python3-pip
    fi

    if [ "$IS_FEDORA" = "1" ]; then
        sudo nice ionice -c3 "$PPWD/base-scripts/build-python.sh"
        PIP="/opt/llvm/$VERSION/local/bin/pip3"
    else
        PIP=$(which pip3)
    fi

    # Broken library introduced in default awscli version
    sudo apt-get remove awscli || true
    sudo "$PIP" install --upgrade awscli boto3 requests
    return 0
}

install_distro_dependencies
install_pip3_and_pip3_packages

