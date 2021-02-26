#!/bin/sh

FILE_PATH=${ROS_PACKAGE_PATH%:*}/robot_ws/perception/cmd/beamer/beamer_app.go
echo "Building: "$FILE_PATH
go build $FILE_PATH
echo "Built: "${ROS_PACKAGE_PATH%:*}"/robot_ws/perception/beamer_app"
echo "Launching..."

${ROS_PACKAGE_PATH%:*}/robot_ws/perception/beamer_app

while  true
do
    sleep 1
done

