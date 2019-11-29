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
//const { parentPort, workerData } = require('worker_threads');
const Mutex = require('async-mutex').Mutex;

const cv = require('opencv4nodejs');
const { createImageData, createCanvas } = require('canvas')
/**
 * Provides the `/posenet` node and output topic.
 * 
 * This function provides the `/posenet` node. It subscribes to an image topic
 * (e.g. `/image_raw`) and feed it to PoseNet to obtain the estimated poses.
 * The estimated poses are then published to an output topic (e.g. `/poses`).
 */
async function main() {
    // Enum with the states of the Posenet ROS Node app.
    const State = Object.freeze({
        LOADING: 'loading',
        WAITING_INPUT: 'ready',
        RUNNING: 'busy',
        TERMINATING: 'terminating' //! Probably doesn't matter.
    });

    const stateMutex = new  Mutex();
    currentState = State.LOADING;

    process.on('message', callbackParentMessage);

    (process.argv[2] == 'true')

    useGPU = (process.argv[2] == 'true');
    multiPose = (process.argv[3] == 'true');
    architecture = process.argv[4];
    outputStride = parseInt(process.argv[5]);
    multiplier = parseFloat(process.argv[7]);
    quantBytes = parseInt(process.argv[8]);
    flipHorizontal = (process.argv[9] == 'true');
    maxDetection = parseInt(process.argv[10]);
    minPartConf = parseFloat(process.argv[11]);
    nmsRadius = parseInt(process.argv[12]);

    separatorIdx = process.argv[7].indexOf('x');
    if (separatorIdx != -1){
        inputResolution = {
            width: process.argv[7].slice(0, separatorIdx),
            height: process.argv[7].slice(separatorIdx+1),
        }
    } else{
        inputResolution = parseInt(process.argv[7]);
    }

    // Load PoseNet dependencies and model.
    if (useGPU){
        require('@tensorflow/tfjs-node-gpu');
        console.log("loading tfjs-gpu");
    }
    else{
        require('@tensorflow/tfjs-node');
        console.log("loading tfjs-cpu");
    }
    const posenet = require('@tensorflow-models/posenet');
        
    const net = await posenet.load({
        architecture: architecture,
        outputStride: outputStride,
        inputResolution: inputResolution,
        multiplier: multiplier,
        quantBytes: quantBytes,
    });

    stateMutex.acquire().then(release => {
        currentState = State.WAITING_INPUT;
        release();
        process.send({type: 'ready'});
    })
    
    
    // Main function ends here. Bellow you can find the utility functions and
    // callbacks.

    function callbackParentMessage(message){
        if(message.hasOwnProperty('type')){
            switch(message.type){
                case 'inquire_state':
                    processInquireStateMessage();
                    break;
                case 'classify':
                    processClassifyMessage(message);
                    break;
                default:
                    process.send({
                        type: 'error', 
                        message: `Unknown message type: '${message.status}'`});
            }
        } else {
            process.send({
                type: 'error', 
                message: 'Ill-formatted message: \'type\' field is missing'});
        }
    }

    function processInquireStateMessage(){
        process.send({
            type: 'state', 
            message: currentState
        });
    }
    
    async function processClassifyMessage(message){
        if(currentState == State.WAITING_INPUT){
            let release = await stateMutex.acquire();
            currentState = State.RUNNING;
            release();

            // console.log(message.image);

            let image000 = formatImage(message.image);

            if(multiPose){
                poses = await net.estimateMultiplePoses(image000, {
                    flipHorizontal: false,
                    maxDetections: 10,
                    scoreThreshold: 0.1,
                    nmsRadius: 30});
            } else{
                poses = await net.estimateSinglePose(image000, 
                            {flipHorizontal: FlipHorizontal});
                poses = [poses];
            }

            console.log(poses)

            release = await stateMutex.acquire();
            currentState = State.WAITING_INPUT;
            release();

            process.send({
                type: 'result', 
                poses: poses,
                metadata: message.metadata
            });
        } else {
            process.send({
                type: 'error', 
                message: `Cannot run the classifier at this point. Current state is '${currentState}'.`
            });
        }
    }

    /**
     * Transforms a ROS image message into a Canvas object, so that it can be
     * inputted into the PoseNet neural network.
     * @param {sensor_msgs.Image} imgData A ROS image message.
     * @returns {Canvas} A Canvas object with the same content as the ROS image. // TODO update
     */
    function formatImage(imgData){
        // Converts the original color mode to RGBA. 
        let conversionCode = null;

        if(imgData.encoding == "rgb8")
            conversionCode = cv.COLOR_RGB2RGBA;
        else if(imgData.encoding == "bgr8")
            conversionCode = cv.COLOR_BGR2RGBA;
        else
            throw "Unknown image format.";
        
        let img = new cv.Mat(Buffer.from(imgData.data), imgData.height, 
                    imgData.width, cv.CV_8UC3).cvtColor(conversionCode);
        
        // Creates the Canvas object, draw and return it.
        const imgCanvas = createCanvas(imgData.width, imgData.height);
        const imgCtx = imgCanvas.getContext('2d');
        let tempImg = createImageData(
            new Uint8ClampedArray(img.getData()),
            imgData.width,
            imgData.height
        );
        imgCtx.putImageData(tempImg, 0, 0);

        cv.imshow('test', img.cvtColor(cv.COLOR_RGB2BGR));
        cv.waitKey(1);
        return imgCanvas; //new Uint8ClampedArray(img.getData());
    }
}


// Executes the main function.
if (require.main === module)
    main();
