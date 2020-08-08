# auvic_rostesting

This package contains the rostest nodes used for integration testing. Integration testing is done to check if nodes are communicating "correctly" with one another. One such example would be requesting the "sailorMan" node within the devices package to retrieve a list of active ECUs (Electronic Controller Units). Another would be sending data through the virtual can bus (vcan0) and seeing if the monitor package correctly published the frame to the correct topic. The examples are endless.

Test Nodes can be written in either C++ and Python using gtest and unittest, respectively. My personal opinion would be to use unittest since it is much easier to get started, however, we have many examples within the communication folder if you are interested in creating them in C++.

Tests are run using "catkin_make run_tests" or "catkin_make run_tests_< projectname >_ gtest _< testName >"

Dont add integration tests to the CI. They are too large and should be run on your local machine. Add the local tests as shown in monitor pkg.
