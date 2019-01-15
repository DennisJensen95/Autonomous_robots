cmake_minimum_required(VERSION 3.13)
project(Autonomous_robots C)

set(CMAKE_C_STANDARD 11)

add_executable(Autonomous_robots
        serverif.c
        square.c
        square_acceleration.c
        square_follow_line_without_center.c
        square_org.c
        square_with_linesensor.c
        square_without_controller.c)
