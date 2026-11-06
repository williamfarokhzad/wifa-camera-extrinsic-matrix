# Camera extrinsic matrix audit in Python

Find the position of a camera, check the direction of a supplied transform and project known 3D points. The lab uses four small JSON fixtures. It runs offline with the Python standard library. There is no camera connection or calibration solver.

[Full illustrated lesson at WIFA Robotics](https://wifarobotics.com/learn/camera-extrinsic-matrix/)

## Set up from zero

1. Install [Python 3.11 or newer](https://www.python.org/downloads/). In a macOS or Linux terminal run `python3 --version`. In Windows PowerShell run `py -3 --version` and use `py -3` in place of `python3` in every command below. If Windows has no launcher, use `python` after checking its version.
2. On GitHub choose **Code → Download ZIP**, extract the ZIP, then open a terminal in the extracted folder. Run `ls` and check that you see `extrinsics.py` and the `examples` folder.
3. Run the examples. No packages or virtual environment are required.

```sh
python3 extrinsics.py examples/parallel.json
python3 extrinsics.py examples/side_view.json
python3 extrinsics.py examples/depth_and_bounds.json
python3 extrinsics.py examples/reflection.json
python3 -m unittest discover -s tests -v
```

| Scene | Check against the JSON output |
| --- | --- |
| Parallel | Camera centre (0.2, −0.1, 0.5) m; optical axis at (320, 240) px; right and down point at (440, 330) px |
| Side view | Camera centre (1, 0.25, 2) m; right and down point at (520, 330) px |
| Depth and bounds | No pixel for zero or negative depth; (0, 240) and (620, 240) inside; (640, 240) outside |
| Reflection | Intentional exit code 2 with `rotation determinant must be +1` |

On macOS or Linux, inspect an exit code with `echo $?` immediately after the command. In PowerShell use `$LASTEXITCODE`. A valid scene can contain points outside the frame and still exit successfully. Nine unit tests include an intentional frame direction mistake, a geometric check using independent axis directions and a CLI error check.

## Make a scene

Copy `examples/side_view.json` to a new file. Name the input direction explicitly. Use `world_to_camera` for a matrix that accepts a world point and returns a camera point. Use `camera_to_world` for a camera pose that accepts camera coordinates and returns world coordinates. The script converts either to world to camera internally.

The matrix is a rigid 4 by 4 transform with last row `[0, 0, 0, 1]`. All points and translations use metres, and the camera axes are right, down and forward. Enter `[fx, fy, cx, cy]` and `[width, height]` in pixels. Intrinsics are for a pinhole model with zero skew and no distortion. Add at least one named 3D world point to `points`.

Try changing only the side view direction label to `world_to_camera`. The matrix is still a valid rotation, but the optical axis point comes out behind the camera. Restore the label; it should project to (320, 240). For an in front point the lab uses continuous bounds `0 <= u < width` and `0 <= v < height` without rounding. Display is rounded to six decimal places after classification.

## Troubleshooting

* **Script or fixture missing:** Open a terminal in the extracted project root and run the commands there.
* **Point behind the camera:** Compare the label with the producer's frame convention and check a known forward point. Do not replace negative depth with its absolute value.
* **Rotation rejected:** Check that the rotation block was copied without scale or reflection. Use full precision from the producer; rotation row checks use an absolute tolerance of 1e−8.
* **Camera centre has the wrong sign:** For a world to camera transform its translation is the world origin expressed in camera coordinates. The camera centre in world coordinates is `-transpose(R) @ t`.
* **Everything projects in the wrong location:** Check that the input point and translation units agree. Then check for a crop or resize and whether your reference pixels are distorted.

## What the output can tell you

The built in examples are synthetic and there is no hardware validation. This checks a supplied transform, not its calibration accuracy. On a real system, use independently surveyed targets across the image and several depths. Compare with undistorted pixel measurements that match the supplied intrinsics, and check the image resolution and camera frame documentation. Positive depth and an in frame pixel do not prove visibility or rule out occlusion. One pixel alone gives no metric depth. The script does not command or connect to a robot.

The two WebP diagrams in `diagrams/` have editable SVG sources in the same folder. Their descriptions are in the SVG metadata and the website image alt text. The CLI has no dependency on these image files.

Primary references: [OpenCV 4.13 camera geometry](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html) and [Lynch and Park, Homogeneous Transformation Matrices](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/).
