#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
rostopic pub /detect vision_msgs/Detection3DArray -f $DIR/detection3Darray.yaml