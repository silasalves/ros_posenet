#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ros_posenet Model Downloader Script

Downloads the files from Google server and save to the disk. The default path
is ``{ROS_POSENET_DIR}/models``, but you can use a custom path by setting the
``posenet_download/path`` parameter, such as::

    $ rosrun ros_posenet download_models.py _path:=/MY/CUSTOM/PATH
"""

import requests
import os
import sys
import errno
import rospy

# Information about the models available for download.
MODELS = [
    {
        "name": "mobilenet",
        "quant": ['quant1', 'quant2', 'float'], 
        "weights": ['050', '075', '100'],
        "stride": [8, 16]
    },
    {
        "name": "resnet50",
        "quant": ['quant1', 'quant2', 'float'],
        "weights": None,
        "stride": [16, 32]
    }
]

# Original Google URL and path to the models.
BASE_URL = "https://storage.googleapis.com/tfjs-models/savedmodel/posenet"

def build_url_list():
    """Returns the list of available models.

    The list of available models is a combination of BASE_URL and MODELS.
    
    Returns:
        list -- List of URL with the JSON files describing the models. 
    """
    model_urls = []
    for model in MODELS:
        name = model["name"]
        for quant in model["quant"]:
            if model["weights"] is not None:
                for weights in model["weights"]:
                    for stride in model["stride"]:
                        model_url = "%s/%s/%s/%s/model-stride%d.json" % (
                                    BASE_URL, name, quant, 
                                    weights, stride)
                        model_urls.append(model_url)
            else:
                for stride in model["stride"]:
                    model_url = "%s/%s/%s/model-stride%d.json" % (
                                BASE_URL, name, quant, 
                                stride)
                    model_urls.append(model_url)
    return model_urls

def create_local_dir(local_file):
    """Try to create the directory to save a given file if it doesn't exist.
    
    Arguments:
        local_file {str} -- Absolute or relative path to the file.
    """
    if not os.path.exists(os.path.dirname(local_file)):
        try:
            os.makedirs(os.path.dirname(local_file))
        except OSError as e: # Guard against race condition
            if e.errno != errno.EEXIST:
                print(e)
                raise


if __name__ == "__main__":
    """Downloads the files from Google server and save to the disk.
    """
    default_path = os.path.dirname(os.path.realpath(__file__)) + "/../models"
    
    rospy.init_node("ros_posenet_download")
    local_path = os.path.abspath(rospy.get_param("~path", default_path))
    
    rospy.loginfo("Saving models to: %s" % local_path)
    
    res = raw_input("This will overwrite files in case they exist." 
                    "Continue? (Y/n)")
    if res != 'y' and res != 'Y':
        rospy.loginfo("Exiting.")
        sys.exit(1)

    model_urls = build_url_list()

    base_url_len = len(BASE_URL)+1
    for url in model_urls:
        local_file = os.path.join(local_path, url[base_url_len:])
        rospy.loginfo("Downloading %s\nto %s" % (url, local_file))

        create_local_dir(local_file)

        model_file = requests.get(url)
        if model_file.status_code == 200:
            base_weight_url = url[:url.rfind('/')]
            rospy.loginfo("Download complete, saving to disk.")
            with open(local_file, "w") as f:
                f.write(model_file.text)

            for weight_manifest in model_file.json()["weightsManifest"]:
                for weight_file_name in weight_manifest["paths"]:
                    weight_url = "%s/%s" % (base_weight_url, weight_file_name)
                    local_file = os.path.join(
                        local_path, weight_url[base_url_len:])
                    
                    rospy.loginfo("Downloading %s\nto %s" % (
                        weight_url, local_file))
                    
                    weight_file = requests.get(weight_url)
                    if weight_file.status_code == 200:
                        rospy.loginfo("Download complete, saving to disk.")
                        with open(local_file, "wb") as f:
                            f.write(weight_file.content)
                    else:
                        rospy.logerr("Could not download %s\n"
                                     "Returned code: %d" % (
                                     weight_url, weight_file.status_code))
        else:
            rospy.logerr("Could not download %s\nReturned code: %d" % (
                url, model_file.status_code))
    
    rospy.loginfo("Finished downloading all the models.")

