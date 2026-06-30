DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
export IGN_GAZEBO_RESOURCE_PATH="$DIR/models:$IGN_GAZEBO_RESOURCE_PATH"
pkill -f "ign gazebo"
ign gazebo -v 4 "$DIR/ellipse_road.sdf"
