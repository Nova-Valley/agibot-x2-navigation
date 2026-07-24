#!/bin/bash
set -e

PRJ_SRC_ROOT=$(cd $(dirname $0)/../.. && pwd)
AIMDK_MSGS_DIR=${PRJ_SRC_ROOT}/aimdk_msgs

cd $(dirname $0)

# Create interface directory if it doesn't exist
mkdir -p ${AIMDK_MSGS_DIR}/interface

# Clean and copy files
rm -rf ${AIMDK_MSGS_DIR}/interface/*
cp -r ${AIMDK_MSGS_DIR}/src/interface/ros2/* ${AIMDK_MSGS_DIR}/interface/

# Remove internal directories if they exist
find ${AIMDK_MSGS_DIR}/interface -mindepth 1 -type d -name "internal" -exec rm -rf {} \; 2>/dev/null || true

exit 0
