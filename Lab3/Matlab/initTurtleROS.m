function [posePub, poseSub] = initTurtleROS()
    posePub = publishROS('/turtle1/cmd_vel','geometry_msgs/Twist');
    poseSub = subscribeROS('/turtle1/pose','turtlesim/Pose');
end