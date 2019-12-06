#!/usr/bin/env python
# -*- coding: utf-8 -*-

import requests
import os
import sys
import errno

# Mobilenet

# Quant = 2
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/mobilenet/quant2/075/model-stride16.json
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/mobilenet/quant2/075/group1-shard1of1.bin

# Quant = 4
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/mobilenet/float/075/model-stride16.json
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/mobilenet/float/075/group1-shard1of2.bin


# RESNET

# Quant = 1
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/resnet50/quant1/model-stride32.json
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/resnet50/quant1/group1-shard1of6.bin

# Quant = 2
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/resnet50/quant2/model-stride32.json
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/resnet50/quant2/group1-shard1of12.bin

# Quant = 4
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/resnet50/float/model-stride32.json
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/resnet50/float/group1-shard1of23.bin

# Stride = 16
# https://storage.googleapis.com/tfjs-models/savedmodel/posenet/resnet50/quant2/model-stride16.json

MODELS = [
    {
        "name": "mobilenet",
        "quant": ['quant1', 'quant2', 'float'], 
        "weights": ['050', '075', '100'],
        "stride": [8, 16] # docs say that 32 is supported, but returns 404 error. Their example also doesn't list 32.
    },
    {
        "name": "resnet50",
        "quant": ['quant1', 'quant2', 'float'],
        "weights": None,
        "stride": [16, 32]
    }
]

LOCAL_PATH = "/home/admin3srp/catkin_ws/src/ros_posenet/models"
BASE_URL = "https://storage.googleapis.com"
INTERMEDIARY_PATH = "tfjs-models/savedmodel/posenet"

def build_url_list():
    model_urls = []
    for model in MODELS:
        name = model["name"]
        for quant in model["quant"]:
            if model["weights"] is not None:
                for weights in model["weights"]:
                    for stride in model["stride"]:
                        model_url = "%s/%s/%s/%s/%s/model-stride%d.json" % (
                                    BASE_URL, INTERMEDIARY_PATH, name, quant, 
                                    weights, stride)
                        model_urls.append(model_url)
            else:
                for stride in model["stride"]:
                    model_url = "%s/%s/%s/%s/model-stride%d.json" % (
                                BASE_URL, INTERMEDIARY_PATH, name, quant, 
                                stride)
                    model_urls.append(model_url)
    return model_urls

def create_local_dir(local_file):
    if not os.path.exists(os.path.dirname(local_file)):
        try:
            os.makedirs(os.path.dirname(local_file))
        except OSError as e: # Guard against race condition
            if e.errno != errno.EEXIST:
                print(e)
                raise


if __name__ == "__main__":
    # Generates main URLs.
    model_urls = build_url_list()
    
    if not create_local_dir(LOCAL_PATH):
        res = raw_input("This will overwrite files in case they exist." 
                        "Continue? (Y/n)")
        if res != 'y' and res != 'Y':
            print("Stopping")
            sys.exit(1)

    base_url_len = len(BASE_URL)+1
    for url in model_urls:
        local_file = os.path.join(LOCAL_PATH, url[base_url_len:])
        print("Downloading %s\nto %s" % (url, local_file))

        create_local_dir(local_file)

        model_file = requests.get(url)
        if model_file.status_code == 200:
            base_weight_url = url[:url.rfind('/')]
            print("Download complete, saving to disk.")
            with open(local_file, "w") as f:
                f.write(model_file.text)

            for weight_manifest in model_file.json()["weightsManifest"]:
                for weight_file_name in weight_manifest["paths"]:
                    weight_url = "%s/%s" % (base_weight_url, weight_file_name)
                    local_file = os.path.join(LOCAL_PATH, weight_url[base_url_len:])
                    
                    print("Downloading %s\nto %s" % (weight_url, local_file))
                    
                    weight_file = requests.get(weight_url)
                    if weight_file.status_code == 200:
                        print("Download complete, saving to disk.")
                        with open(local_file, "wb") as f:
                            f.write(weight_file.content)
                    else:
                        print("Error: could not download %s\nReturned code: %d" % (weight_url, weight_file.status_code))
        else:
            print("error: %d" % model_file.status_code)

