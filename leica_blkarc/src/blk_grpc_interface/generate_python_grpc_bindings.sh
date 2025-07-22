#! /usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
BASEDIR=$( dirname ${SCRIPT_DIR} )
BLK_INTERFACE_DIR=${BASEDIR}/blk_grpc_interface
API_DIR=${BLK_INTERFACE_DIR}/protos

pushd ${BLK_INTERFACE_DIR} || exit
    rm -rf ${BLK_INTERFACE_DIR}/blk_arc_grpc

    PROTOFILES=($API_DIR/blk_arc_grpc/*.proto)
    python3 -m grpc_tools.protoc -I${API_DIR} --python_out=$BLK_INTERFACE_DIR --grpc_python_out=$BLK_INTERFACE_DIR "${PROTOFILES[@]}"

    pip install -e .
popd || exit
