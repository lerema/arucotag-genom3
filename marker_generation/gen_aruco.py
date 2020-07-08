#! /usr/bin/env python3
#
# Copyright (c) 2020 LAAS/CNRS
# All rights reserved.
#
# Redistribution  and  use  in  source  and binary  forms,  with  or  without
# modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
# THE SOFTWARE  IS PROVIDED 'AS IS'  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
# WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
# MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
# ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
# WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
# IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#                                                  Martin Jacquet - June 2020
#
import cv2
from cv2 import aruco

from os import replace, makedirs
from os.path import join, exists, expanduser
from shutil import rmtree
import argparse

home = expanduser("~")

parser = argparse.ArgumentParser(description="Generate aruco source files.")
parser.add_argument("index", help="Id of marker to draw.", type=int)
parser.add_argument(
    "-l", dest="length", help="Size of gazebo model (meter).", default=1.0, type=float
)
parser.add_argument(
    "-g", dest="gazebo", help="Generate gazebo model.", action="store_true"
)
parser.add_argument(
    "-d",
    dest="aruco_dict",
    help="Dictionary to uses. List of available dicts can be found in the provided README.",
    default="DICT_6X6_250",
    type=str,
)
parser.add_argument(
    "-p",
    dest="path",
    help="Gazebo model marker output path.",
    default=join(home, ".gazebo/models"),
    type=str,
)
args = parser.parse_args()

# Dico
dic_o_carré = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_4X4_1000": aruco.DICT_4X4_1000,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_5X5_250": aruco.DICT_5X5_250,
    "DICT_5X5_1000": aruco.DICT_5X5_1000,
    "DICT_6X6_50": aruco.DICT_6X6_50,
    "DICT_6X6_100": aruco.DICT_6X6_100,
    "DICT_6X6_250": aruco.DICT_6X6_250,
    "DICT_6X6_1000": aruco.DICT_6X6_1000,
    "DICT_7X7_50": aruco.DICT_7X7_50,
    "DICT_7X7_100": aruco.DICT_7X7_100,
    "DICT_7X7_250": aruco.DICT_7X7_250,
    "DICT_7X7_1000": aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": aruco.DICT_APRILTAG_36h11,
}
try:
    dict = aruco.Dictionary_get(dic_o_carré[args.aruco_dict])
except KeyError:
    print("Invalid dictionary.")
    exit()

marker = aruco.drawMarker(dict, args.index, 700)
markername = args.aruco_dict.replace("DICT", "aruco") + "_" + str(args.index)
if not args.gazebo:
    cv2.imwrite(markername + ".png", marker)
else:
    path = join(args.path, markername)
    if exists(path):
        rmtree(path)
    makedirs(path)
    makedirs(join(path, "materials"))
    makedirs(join(path, "materials", "scripts"))
    makedirs(join(path, "materials", "textures"))

    cv2.imwrite(join(path, "materials", "textures", markername + ".png"), marker)

    with open("template/model.sdf", "r") as f:
        sdf_template = f.read()
    with open("template/model.config", "r") as f:
        config_template = f.read()
    with open("template/template.material", "r") as f:
        material_template = f.read()

    with open(join(path, "model.sdf"), "w") as f:
        f.write(
            sdf_template.replace("TEMPLATE_MARKER", markername).replace(
                "TEMPLATE_LENGTH", str(args.length)
            )
        )

    with open(join(path, "model.config"), "w") as f:
        f.write(config_template.replace("TEMPLATE_MARKER", markername))

    with open(join(path, "materials", "scripts", markername + ".material"), "w") as f:
        f.write(material_template.replace("TEMPLATE_MARKER", markername))
