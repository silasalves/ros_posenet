#!/usr/bin/env node

/**
 * ros_posenet worker script
 * 
 * This is the ros_posenet worker script that actually loads PoseNet and
 * performs skeleton detection. It is spawned as a child process by
 * posenet_ros_node.js and waits for the parent process to send messages
 * containing the images. After classification, it sends back the results.
 */

// Packages required by PoseNet. The PoseNet package itself is loaded in the
// main function, as it requires the CPU or the GPU tensorflow library to be
// loaded first, and that depends on the configuration read from the launch
// file.
const tf = require('@tensorflow/tfjs');

// Packages required for managing the multiprocessing approach.
const Mutex = require('async-mutex').Mutex;
const { performance } = require('perf_hooks');

// Packages for dealing with images.
const cv = require('opencv4nodejs');
const { createImageData, createCanvas } = require('canvas')
const { debugView } = require('./debug_view');


function buildModelFileName(architecture, quantBytes, multiplier, outputStride){
    //"file:///home/admin3srp/catkin_ws/src/ros_posenet/models/tfjs-models/savedmodel/posenet/mobilenet/float/075/model-stride16.json"
    basePath = "file:///home/admin3srp/catkin_ws/src/ros_posenet/models/tfjs-models/savedmodel/posenet"
    if(quantBytes == 1)
        quantDir = "quant1"
    else if (quantBytes == 2)
        quantDir = "quant2"
    else if (quantBytes == 4)
        quantDir = "float"

    if(architecture=='MobileNetV1'){
        if(multiplier == 0.5)
            multiplierDir = "050"
        else if(multiplier == 0.75)
            multiplierDir = "075"
        else if(multiplier == 1.0)
            multiplierDir = "100"
        return `${basePath}/mobilenet/${quantDir}/${multiplierDir}/model-stride${outputStride}.json`;
    }        
    else if (architecture = 'ResNet50')
        return `${basePath}/resnet50/${quantDir}/model-stride${outputStride}.json`;
}

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
        RUNNING: 'busy'
    });

    const stateMutex = new  Mutex(); // Controls access to currentState.
    currentState = State.LOADING;

    // Registers callback function that processes all parent's messages.
    process.on('message', callbackParentMessage);

    // Retrieves the configuration parameters from the arguments.
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
    minPoseConf = parseFloat(process.argv[13]);
    debug = (process.argv[14] == 'true');

    separatorIdx = process.argv[6].indexOf('x');
    if (separatorIdx != -1){
        inputResolution = {
            width: parseInt(process.argv[6].slice(0, separatorIdx)),
            height: parseInt(process.argv[6].slice(separatorIdx+1)),
        }
    } else{
        inputResolution = parseInt(process.argv[6]);
    }

    // Load PoseNet dependencies and model.
    if (useGPU){
        require('@tensorflow/tfjs-node-gpu');
    }
    else{
        require('@tensorflow/tfjs-node');
    }
    const posenet = require('@tensorflow-models/posenet');

    posenet_config = {
        architecture: architecture,
        outputStride: outputStride,
        inputResolution: inputResolution,
        multiplier: multiplier,
        quantBytes: quantBytes,
        modelUrl: buildModelFileName(architecture, quantBytes, multiplier, outputStride)
        //modelUrl: "file:///home/admin3srp/catkin_ws/src/ros_posenet/models/tfjs-models/savedmodel/posenet/mobilenet/float/075/model-stride16.json"
    };

    console.log(posenet_config)
        
    const net = await posenet.load(posenet_config);

    stateMutex.acquire().then(release => {
        currentState = State.WAITING_INPUT;
        release();
        process.send({type: 'ready'});
    })
    
    
    // Main function ends here. Bellow you can find the utility functions and
    // callbacks.

    /**
     * Process messages from the parent process.
     * 
     * This is the main callback that process all messages sent by the parent
     * process. The messages should always have a `type` field which accepts
     * the following values: [`inquire_state`, `classify`].
     * 
     * @param {object} message 
     */
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

    /**
     * Sends the current state of the FSM to the parent process. 
     */
    function processInquireStateMessage(){
        process.send({
            type: 'state', 
            message: currentState
        });
    }
    
    /**
     * Detect all skeletons on the given image and send results to parent.
     * 
     * The message should contain the fields `image` with the raw compressed
     * image and `metadata` with the image header.
     * 
     * @param {Object} message 
     */
    async function processClassifyMessage(message){
        if(currentState == State.WAITING_INPUT){
            let release = await stateMutex.acquire();
            currentState = State.RUNNING;
            release();

            let image = formatImage(message.image);

            if(multiPose){
                poses = await net.estimateMultiplePoses(image.canvas, {
                    flipHorizontal: false,
                    maxDetections: 10,
                    scoreThreshold: 0.1,
                    nmsRadius: 30});
            } else{
                poses = await net.estimateSinglePose(image.canvas, 
                            {flipHorizontal: FlipHorizontal});
                poses = [poses];
            }

            if(debug)
                debugView(image.cvmat, poses, minPoseConf, minPartConf);

            release = await stateMutex.acquire();
            currentState = State.WAITING_INPUT;
            release();

            t0 = performance.now();
            process.send({
                type: 'result', 
                poses: poses,
                metadata: message.metadata
            });
            t1 = performance.now();
            console.log(`Sending image to parent took ${t1-t0} ms.`);
        } else {
            process.send({
                type: 'error', 
                message: `Cannot run the classifier at this point. ` + 
                         `Current state is '${currentState}'.`
            });
        }
    }

    /**
     * Transforms a ROS image message into a Canvas object, so that it can be
     * inputted into the PoseNet neural network.
     * @param {sensor_msgs.Image} imgData A ROS CompressedImage message.
     * @returns {{canvas:Canvas, cvmat:cv.Mat}} A Canvas object and a OpenCV Mat
     *      with the same content as the ROS image.
     */
    function formatImage(imgData){
        encoding = imgData.format.split(" ");
        encoding = encoding[encoding.length-1];

        if(encoding == "rgb8")
            conversionCode = cv.COLOR_RGB2RGBA;
        else if(encoding == "bgr8")
            conversionCode = cv.COLOR_BGR2RGBA;
        else
            throw "Unknown image format.";
        
        img = cv.imdecode(Buffer.from(imgData.data), cv.IMREAD_COLOR)
                .cvtColor(cv.COLOR_BGR2RGBA);
                
        // Creates the Canvas object, draw and return it.
        const imgCanvas = createCanvas(img.cols, img.rows);
        const imgCtx = imgCanvas.getContext('2d');
        let tempImg = createImageData(
            new Uint8ClampedArray(img.getData()),
            img.cols,
            img.rows
        );
        imgCtx.putImageData(tempImg, 0, 0);

        return {
            canvas: imgCanvas,
            cvmat: img
        };
    }
}


// Executes the main function.
if (require.main === module)
    main();
