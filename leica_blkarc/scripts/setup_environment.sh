#! /bin/bash
set -o errtrace

help()
{
   echo "Setup python environment and install the BLK ARC Module API Python package."
   echo
   echo "Syntax: ./setup_environment.sh [-h]"
   echo
   echo "h     Print this Help."
   echo
}

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

pushd "${DIR}/.." || exit
    # create a virtual environment
    virtualenv .venv -p python3.8

    # activate the virtual environment
    source .venv/bin/activate

    # install the requirements
    pip install --upgrade pip
    python3 -m pip install -r requirements.txt

    # build the blk_arc_sample_wrapper package
    ./src/python_sample_wrapper/install.sh

    # show the details of the installed package
    pip show blk_arc_sample_wrapper
popd || exit
