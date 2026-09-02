#!/usr/bin/env python3
"""Generate a deterministic printable ArUco marker image."""
import argparse
from pathlib import Path
import cv2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, default=0, dest="marker_id")
    parser.add_argument("--size", type=int, default=600, help="Square output size in pixels")
    parser.add_argument("--dictionary", default="DICT_6X6_250")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if not hasattr(cv2, "aruco"):
        parser.error("OpenCV lacks cv2.aruco; install opencv-contrib-python.")
    if args.size < 16:
        parser.error("--size must be at least 16 pixels")
    try:
        dictionary_id = getattr(cv2.aruco, args.dictionary)
    except AttributeError:
        parser.error(f"Unknown ArUco dictionary: {args.dictionary}")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if args.marker_id < 0 or args.marker_id >= dictionary.bytesList.shape[0]:
        parser.error(f"--id must be in [0, {dictionary.bytesList.shape[0] - 1}]")
    image = (cv2.aruco.generateImageMarker(dictionary, args.marker_id, args.size)
             if hasattr(cv2.aruco, "generateImageMarker")
             else cv2.aruco.drawMarker(dictionary, args.marker_id, args.size))
    output = args.output or Path(f"aruco_id_{args.marker_id}.png")
    if not cv2.imwrite(str(output), image):
        parser.error(f"Could not write {output}")
    print(output)


if __name__ == "__main__":
    main()
