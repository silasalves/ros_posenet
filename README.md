Original wrapper: https://github.com/hansonrobotics/ros_posenet

# ROS PoseNet

*ros_posenet* is a [ROS](https://www.ros.org/) wrapper for [PoseNeT](https://github.com/tensorflow/tfjs-models/tree/master/posenet) 
library in [Node.js](https://nodejs.org/).

You can run the official PoseNet demo on your browser by
[clicking here](https://storage.googleapis.com/tfjs-models/demos/posenet/camera.html)

---

**IMPORTANT:** Current implementation is for experiments only

---

## Requirements

  * ROS Kinetic or newer (_tested with Kinetic_)
  * Node.js 8.x or newer (_tested with 12.13.1_)
  * For GPU acceleration: Cuda 9.0 + cuDNN 7.1

## Installation

### Node.js 12 (optional)

Node.js version 12 is currently the most recent LTS (long term support) version. To install it, please follow these instructions:

```bash
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
sudo apt-get update
sudo apt-get install -y nodejs yarn
```

### ros_posenet

Navigate to the `src` directory at your catkin workspace, clone the repository, 
navigate to the repository directory and install the dependencies:

```bash
git clone https://github.com/silasalves/ros_posenet.git
cd ros_posenet
npm install
```

## Configuration

Prior to using ros_posenet, a few settings need to be configured in `launch/posenet.launch`. See the table below for more information about the parameters that need to be set up. For more information on the parameters used by PoseNet, please refer to the [official PoseNet repository](https://github.com/tensorflow/tfjs-models/tree/master/posenet).

| Parameter | Description | Default value |
|-----------|-------------|---------------|
| compressed_image_topic | Name of the input topic with the color images. | /camera/color/image_raw/compressed |
| poses_topic | Name of the output topic for the detected poses. | /poses |
| gpu | Whether PoseNet should run over the GPU (`true`) or CPU (`false`). | false |
| models_path | Absolute path for the downloaded models. If this parameter is commented out or set to `""`, PoseNet will download the models from the Internet on-demand | _empty_
| architecture | Can be either `MobileNetV1` or `ResNet50`. | MobileNetV1 |
| output_stride | Output stride of the model. Can be one of `8`, `16`, `32`. | 16 |
| input_resolution | The size the image is resized to before it is fed into the PoseNet. Can be either an integer (e.g., "500", as in Google's demo) or a two integers separated by `'x'` representing the width and height (e.g., `"640x480"`). | 640x480 |
| multiplier | MobileNetV1 only. Multiplier for the depth for all convolution ops. Can be one of `1.0`, `0.75`, or `0.50`. | 0.75 |
| quant_bytes |  Controls the bytes used for weight quantization. Can be one of `4` (no quantization), `2` or `1`. | 1 |
| flip_horizontal | Whether the input image should be flipped (`true`) or not (`false`). | false |
| multi_pose | Whether PoseNet should look for multiple poses (`true`) or not. (`false`). | true |
| max_detection |  Maximum number of poses being detected by the multi-person algorithm. | 5 |
| min_pose_confidence | Minimum confidence to consider a pose valid, from 0.0 to 1.0. | 0.15 |
| min_part_confidence | Minimum confidence for the body parts, from 0.0 to 1.0. | 0.1 |
| nms_radius | Radius of the Non-Maximum Suppression. | 30 |

### Configuring ros_posenet for off-line usage (optional)

In case you need to run ros_posenet off-line, you will need to download the models to your computers. The script `scripts/download_models.py` can help with that. It downloads the files from Google server and save to a given local directory. The default path is `{ROS_POSENET_DIR}/models`, which will be used in this instructions.

Run the `download_models.py` script to download the files from Google's server:

```
$ rosrun ros_posenet download_models.py
```

It will print the absolute path to the directory the files will be downloaded to.
Type `Y` + `[Return]` to confirm:

```
[INFO] [1576005795.127928]: Saving models to: /home/user/catkin_ws/src/ros_posenet/models
This will overwrite files in case they exist.Continue? (Y/n)
```

Once the script finish running, open `launch/posenet.launch` and un-comment this line:

```xml
<param name="models_path" value="$(find ros_posenet)/models" />
```

If you want to use a custom, you can use the `posenet_download/path` parameter, such as:

```
$ rosrun ros_posenet download_models.py _path:=/MY/CUSTOM/PATH
```

and then set the absolute path in the `launch/posenet.launch` file accordingly.

## Running

After configuring ros_posenet, make sure that both ROS Master and a image provider are running, than execute in the terminal:

```
$ roslaunch ros_posenet posenet.launch
```

To display the detected poses, run:

```
$ rosrun ros_posenet display_keypoints.py
```

## Limitations

 * Only ROS topics with compressed images are supported as inputs.
