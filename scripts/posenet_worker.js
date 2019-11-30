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

const nose = 0
const leftEye = 1
const rightEye = 2
const leftEar = 3
const rightEar = 4
const leftShoulder = 5
const rightShoulder = 6
const leftElbow = 7
const rightElbow = 8
const leftWrist = 9
const rightWrist = 10
const leftHip = 11
const rightHip = 12
const leftKnee = 13
const rightKnee = 14
const leftAnkle = 15
const rightAnkle = 16

const connected_part_names = [
    [leftHip, leftShoulder], [leftElbow, leftShoulder],
    [leftElbow, leftWrist], [leftHip, leftKnee],
    [leftKnee, leftAnkle], [rightHip, rightShoulder],
    [rightElbow, rightShoulder], [rightElbow, rightWrist],
    [rightHip, rightKnee], [rightKnee, rightAnkle],
    [leftShoulder, rightShoulder], [leftHip, rightHip]]

const skeleton_colors=[
    new cv.Vec3(0, 0, 255),
    new cv.Vec3(0, 255, 255),
    new cv.Vec3(0, 255, 255),
    new cv.Vec3(0, 255, 255),
    new cv.Vec3(102, 102, 255),
    new cv.Vec3(128, 128, 128),
    new cv.Vec3(102, 0, 51),
    new cv.Vec3(0, 0, 153),
    new cv.Vec3(102, 102, 0),
    new cv.Vec3(153, 51, 255),
    new cv.Vec3(102, 0, 0),
    new cv.Vec3(0, 0, 0)
]


/**
 * Draws the detected keypoints over the input image and shows to the user.
 * 
 * Used for debugging only.
 * @param {sensor_msgs.Image} imgData A ROS image message.
 * @param {Pose} poses The poses detected by PoseNet.
 */
function debugView (imgData, poses, minPoseScore, minPartScore) {
    if(imgData.encoding == "rgb8")
        conversionCode = cv.COLOR_RGB2BGR;
    else if(imgData.encoding == "bgr8")
        conversionCode = null;
    else
        throw "Unknown image format.";
        
    img = new cv.Mat(Buffer.from(imgData.data), imgData.height, 
            imgData.width, cv.CV_8UC3);
    
    if(conversionCode != null)
        img = img.cvtColor(conversionCode);
    
    console.log(Object.keys(poses).length)
    
    poses.forEach( function(pose, i) {
        if (pose['score'] < minPoseScore)
            return;

        if (i>11)
            i = 11;
        color = skeleton_colors[i]

        connected_part_names.forEach( pair => {
            if (pose['keypoints'][pair[0]]['score'] > 0.2 && pose['keypoints'][pair[1]]['score'] > 0.2) {
                let p0 = pose['keypoints'][pair[0]]['position']
                let p1 = pose['keypoints'][pair[1]]['position']
                img.drawLine(new cv.Point(p0['x'], p0['y']), new cv.Point(p1['x'], p1['y']),
                    color, 8);
            }
        });

        if(pose['score'] > minPartScore){
            pose['keypoints'].forEach(keypoint => {
                if(keypoint['score'] > 0.2)
                    img.drawCircle(new cv.Point(
                        keypoint['position']['x'], 
                        keypoint['position']['y']),
                    5, color, 2, 8, 0);
            });
        }
    });

    cv.imshow('test', img)
    cv.waitKey(1);
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
    minPoseConf = parseFloat(process.argv[13]);

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
        console.log("loading tfjs-gpu");
    }
    else{
        require('@tensorflow/tfjs-node');
        console.log("loading tfjs-cpu");
    }
    const posenet = require('@tensorflow-models/posenet');

    posenet_config = {
        architecture: architecture,
        outputStride: outputStride,
        inputResolution: inputResolution,
        multiplier: multiplier,
        quantBytes: quantBytes,
    };

    console.log(posenet_config);
        
    const net = await posenet.load(posenet_config);

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

            let image = formatImage(message.image);
            console.log(image);

            if(multiPose){
                poses = await net.estimateMultiplePoses(image, {
                    flipHorizontal: false,
                    maxDetections: 10,
                    scoreThreshold: 0.1,
                    nmsRadius: 30});
            } else{
                poses = await net.estimateSinglePose(image, 
                            {flipHorizontal: FlipHorizontal});
                poses = [poses];
            }

            // poses = [];

            //debugView(message.image, poses, minPoseConf, minPartConf);

            release = await stateMutex.acquire();
            currentState = State.WAITING_INPUT;
            release();

            console.time('child -> parent');
            process.send({
                type: 'result', 
                poses: poses,
                metadata: message.metadata
            });
            console.timeEnd('child -> parent');
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
        img = cv.imdecode(Buffer.from(imgData.data), cv.IMREAD_COLOR);
        // console.log(img);
        // cv.imshow('received', img);
        // cv.waitKey(1);

        // Converts the original color mode to RGBA. 
        let conversionCode = null;

        // if(imgData.encoding == "rgb8")
        //     conversionCode = cv.COLOR_RGB2RGBA;
        // else if(imgData.encoding == "bgr8")
        //     conversionCode = cv.COLOR_BGR2RGBA;
        // else
        //     throw "Unknown image format.";
        
        // let img = new cv.Mat(Buffer.from(imgData.data), imgData.height, 
        //             imgData.width, cv.CV_8UC3).cvtColor(conversionCode);

        img = img.cvtColor(cv.COLOR_BGR2RGBA);
        
        // Creates the Canvas object, draw and return it.
        const imgCanvas = createCanvas(img.cols, img.rows);
        const imgCtx = imgCanvas.getContext('2d');
        let tempImg = createImageData(
            new Uint8ClampedArray(img.getData()),
            img.cols,
            img.rows
        );
        imgCtx.putImageData(tempImg, 0, 0);

        return imgCanvas; //new Uint8ClampedArray(img.getData());
    }
}


// Executes the main function.
if (require.main === module)
    main();
