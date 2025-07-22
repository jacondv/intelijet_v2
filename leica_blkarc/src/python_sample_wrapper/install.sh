#! /usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
BASEDIR=$( dirname ${SCRIPT_DIR} )
BLK_INTERFACE_DIR=${BASEDIR}/python_sample_wrapper

pushd ${BLK_INTERFACE_DIR} || exit

    pushd ../blk_grpc_interface || exit
        ./generate_python_grpc_bindings.sh
    popd || exit

    pip install -e .
popd || exit
