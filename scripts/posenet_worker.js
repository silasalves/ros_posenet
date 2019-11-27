#!/usr/bin/env node

/**
 * ROS PoseNet
 * 
 * TODO: Add description
 */

// Packages required by PoseNet. The PoseNet package itself is loaded in the
// main function, as it requires the CPU or the GPU tensorflow library to be
// loaded first, and that depends on the configuration read from the launch
// file.
const tf = require('@tensorflow/tfjs');
const { parentPort, workerData } = require('worker_threads');

/**
 * Provides the `/posenet` node and output topic.
 * 
 * This function provides the `/posenet` node. It subscribes to an image topic
 * (e.g. `/image_raw`) and feed it to PoseNet to obtain the estimated poses.
 * The estimated poses are then published to an output topic (e.g. `/poses`).
 */
async function main() {
    // If the worker receive a message while loading posenet, it will return
    // the 'loading' status.
    parentPort.on('message', callbackLoading);

    // Load PoseNet dependencies and model.
    if (workerData.paramGPU)
        require('@tensorflow/tfjs-node-gpu');
    else
        require('@tensorflow/tfjs-node');
    const posenet = require('@tensorflow-models/posenet');
        
    const net = await posenet.load({
        architecture: workerData.architecture,
        outputStride: workerData.outputStride,
        inputResolution: workerData.inputResolution,
        multiplier: workerData.multiplier,
        quantBytes: workerData.quantBytes,
    });

    parentPort.removeListener('message', callbackLoading);
    parentPort.on('message', callbackReady);

    // rosnodejs.log.info('PoseNet model loaded.');
    
    // Main function ends here. Bellow you can find the utility functions and
    // callbacks.

    // TODO: documentation
    function callbackLoading(message){
        parentPort.postMessage({
            status: 'loading'
        });
    }

    function callbackReady(message){
        parentPort.postMessage({
            status: 'ready'
        });
    }
    
    // /**
    //  * Callback for the pose detection when a single pose is considered.
    //  * 
    //  * This callback process the input ROS image using PoseNet to detect a
    //  * single pose. The result is published into the output topic.
    //  * @param {sensors_msgs.Image} imgData A ROS image message.
    //  */
    // async function singlePoseCallback(imgData){
    //     let t0 = rosnodejs.Time.toSeconds(imgData.header.stamp)
    //     let t1 = rosnodejs.Time.toSeconds(rosnodejs.Time.now())
    //     if(DEBUG)
    //         console.log("Delay between messages: %f", t1-t0);
    //     if((t1 - t0) > paramMaxDelay)
    //         return;
        
    //     const imgCanvas = formatImage(imgData);
    //     if(DEBUG)
    //         console.time("posenet")
    //     pose = await net.estimateSinglePose(imgCanvas, 
    //         {flipHorizontal: paramFlipHorizontal});
    //     if(DEBUG){
    //         console.timeEnd("posenet");
    //         debugView(imgData, [pose], paramMinPoseConf, paramMinPartConf);
    //     }
        
    //     posePub.publish(buildOutputMessage([pose], imgData.header));
    // }


    // /**
    //  * Callback for the pose detection when multiple poses are considered.
    //  * 
    //  * This callback process the input ROS image using PoseNet to detect
    //  * multiple poses. The result is published into the output topic.
    //  * @param {sensors_msgs.Image} imgData A ROS image message.
    //  */
    // async function multiPoseCallback(imgData){
    //     let t0 = rosnodejs.Time.toSeconds(imgData.header.stamp)
    //     let t1 = rosnodejs.Time.toSeconds(rosnodejs.Time.now())
    //     if(DEBUG)
    //         console.log("Delay between messages: %f", t1-t0);
    //     if((t1 - t0) > paramMaxDelay)
    //         return;

    //     if(DEBUG)
    //         console.time("posenet")

    //     const imgCanvas = formatImage(imgData);
    //     poses = await net.estimateMultiplePoses(imgCanvas, {
    //         flipHorizontal: paramFlipHorizontal,
    //         maxDetections: paramMaxDetection,
    //         scoreThreshold: paramMinPartConf,
    //         nmsRadius: paramNmsRadius});
            
    //     if(DEBUG){
    //         console.timeEnd("posenet");
    //         debugView(imgData, poses, paramMinPoseConf, paramMinPartConf);
    //     }
    //     posePub.publish(buildOutputMessage(poses, imgData.header));
    // }
}


// Executes the main function.
if (require.main === module)
    main();
