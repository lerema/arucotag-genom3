# How to use the marker generation script

The associated python files are used to generate the Aruco marker in a png image or as a gazebo model.

#### Prerequisites

* Python3
* python-opencv
* python-opencv-contrib

#### Description

The scrip generates the specified marker as a png file in the current directory.
If the `-g` option is passed, it instead generates a gazebo model for this marker in the path specified with `-p` (default: `~/.gazebo/models/`).

#### Available dictionaries

* `4_50`, generating `cv::aruco::DICT_4X4_50`
* `4_100`, generating `cv::aruco::DICT_4X4_100`
* `4_250`, generating `cv::aruco::DICT_4X4_250`
* `4_1000`, generating `cv::aruco::DICT_4X4_1000`
* `5_50`, generating `cv::aruco::DICT_5X5_50`
* `5_100`, generating `cv::aruco::DICT_5X5_100`
* `5_250`, generating `cv::aruco::DICT_5X5_250`
* `5_1000`, generating `cv::aruco::DICT_5X5_1000`
* `6_50`, generating `cv::aruco::DICT_6X6_50`
* `6_100`, generating `cv::aruco::DICT_6X6_100`
* `6_250`, generating `cv::aruco::DICT_6X6_250`
* `6_1000`, generating `cv::aruco::DICT_6X6_1000`
* `7_50`, generating `cv::aruco::DICT_7X7_50`
* `7_100`, generating `cv::aruco::DICT_7X7_100`
* `7_250`, generating `cv::aruco::DICT_7X7_250`
* `7_1000`, generating `cv::aruco::DICT_7X7_1000`
* `ORIGINAL`, generating `cv::aruco::DICT_ARUCO_ORIGINAL`
* `16h5`, generating `cv::aruco::DICT_APRILTAG_16h5`
* `25h9`, generating `cv::aruco::DICT_APRILTAG_25h9`
* `36h10`, generating `cv::aruco::DICT_APRILTAG_36h10`
* `36h11`, generating `cv::aruco::DICT_APRILTAG_36h11`
