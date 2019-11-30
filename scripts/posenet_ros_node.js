#!/usr/bin/env node

/**
 * ROS PoseNet
 * 
 * This is the main script of the `posenet` package. It reads a ROS color camera 
 * stream (such as `/image_raw`) and then uses PoseNet to detect the skeleton
 * (pose) of a single or multiple users in the image.
 * 
 * The current script supports the two PoseNet models, MobileNetV1 and ResNet50,
 * as well as the single and multi-pose algorithms.
 */

var sizeof = require('object-sizeof')

// Packages required by ROS.
const rosnodejs = require('rosnodejs');
const sensor_msgs = rosnodejs.require('sensor_msgs').msg;
const pose_msgs = rosnodejs.require('ros_posenet').msg;

// Packages required by PoseNet. The PoseNet package itself is loaded in the
// main function, as it requires the CPU or the GPU tensorflow library to be
// loaded first, and that depends on the configuration read from the launch
// file.
const tf = require('@tensorflow/tfjs');
const cv = require('opencv4nodejs');
const { createImageData, createCanvas } = require('canvas')

// const { Worker } = require('worker_threads');
const cp = require('child_process');
const Mutex = require('async-mutex').Mutex;

// If DEBUG is true, the prediction time will be printed, and an image with the
// detected poses will be shown.
const DEBUG = true;

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
    // const imgCanvas = createCanvas(imgData.width, imgData.height);
    // const imgCtx = imgCanvas.getContext('2d');
    // let tempImg = createImageData( // TODO change var name
    //     new Uint8ClampedArray(img.getData()),
    //     imgData.width,
    //     imgData.height
    // );
    // imgCtx.putImageData(tempImg, 0, 0);
    return new Uint8ClampedArray(img.getData());
}

    
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
        WAITING_INPUT: 'waiting_input',
        WAITING_RESULT: 'waiting_result',
        TERMINATING: 'terminating' //! Probably doesn't matter, since ROS control the CTRL+C event.
    });

    let currentState = State.LOADING;
    const stateMutex = new Mutex();

    let posePub = null;
    let imgSub = null;
    
    // Register node with ros; `rosNode` is used to load the parameters.
    const rosNode = await rosnodejs.initNode("/posenet")
    rosnodejs.log.getLogger('ros').setLevel('debug');

    rosnodejs.log.debug(`Transitioning to state '${currentState}'.`);

    rosnodejs.log.info('Node /posenet registered.');
    
    // Load all parameters from `posenet.launch`.
    const paramImgTopic = await getParam('/posenet/compressed_image_topic', '/image_raw');
    const paramPosesTopic = await getParam('/posenet/poses_topic', '/poses');
    const paramGPU = await getParam('/posenet/gpu', false);
    const paramMaxDelay = await getParam('posenet/max_msg_time_diff', 0.03);
    const paramArchitecture = await getParam('/posenet/architecture', 'MobileNetV1');
    const paramMultiplier = await getParam('/posenet/multiplier', 0.5);
    let paramInputResolution = await getParam('/posenet/input_resolution', 257);
    const paramQuantBytes = await getParam('/posenet/quant_bytes', 4)
    const paramOutputStride = await getParam('/posenet/output_stride', 16);
    const paramFlipHorizontal = await getParam('/posenet/flip_horizontal', false);
    const paramMultiPose = await getParam('/posenet/multi_pose', false);
    const paramMaxDetection = await getParam('/posenet/max_detection', 5);
    const paramMinPoseConf = await getParam('/posenet/min_pose_confidence', 0.1);
    const paramMinPartConf = await getParam('/posenet/min_part_confidence', 0.5);
    const paramNmsRadius = await getParam('/posenet/nms_radius', 30);

    // if (!Number.isInteger(paramInputResolution))
    //     paramInputResolution = JSON.parse(paramInputResolution);
    
    // TODO: document this
    posenet_config = {
        useGPU: paramGPU,
        multiPose: paramMultiPose,
        architecture: paramArchitecture,
        outputStride: paramOutputStride,
        inputResolution: paramInputResolution,
        multiplier: paramMultiplier,
        quantBytes: paramQuantBytes,
        lipHorizontal: paramFlipHorizontal,
        maxDetection: paramMaxDetection,
        minPartConf: paramMinPartConf,
        nmsRadius: paramNmsRadius,
        minPoseConf: paramMinPoseConf
    }

    // const worker = new Worker(__dirname + '/posenet_worker.js', {workerData: posenet_config});
    const worker = cp.fork(`${__dirname}//posenet_worker.js`, Object.values(posenet_config));//, JSON.stringify(posenet_config));
    
    worker.on('message', workerMessageCallback);
    worker.on('error', workerErrorCallback);

    posePub = rosNode.advertise(paramPosesTopic, pose_msgs.Poses);
    imgSub = rosNode.subscribe(paramImgTopic, sensor_msgs.CompressedImage, 
            rosImageCallback, {queueSize: 1});

    // Main function ends here. Bellow you can find the utility functions and
    // callbacks.

    function workerErrorCallback(error){
        rosnodejs.log.fatal(error);
    }

    function workerMessageCallback(message) {
        rosnodejs.log.debug(`Received message with type ${message.type}`);
        if(message.hasOwnProperty('type')){
            switch(message.type){
                case 'ready':
                    processReadyMessage();
                    break;
                case 'result':
                    processResultMessage(message);
                    break;
                case 'error':
                    rosnodejs.log.error(message.message);
                    break;
                default:
                    rosnodejs.log.error(`Unknown message type: '${message.type}'`);
            }
        } else {
            rosnodejs.log.error('Ill-formatted message: \'type\' field is missing');
        }
    }

    function processReadyMessage(){
        rosnodejs.log.debug('Processing the \'ready\' message');
        stateMutex.acquire().then(release => {
            try{
                if(currentState == State.LOADING){
                    currentState = State.WAITING_INPUT;
                    rosnodejs.log.info('PoseNet loaded.');
                    rosnodejs.log.debug(`Transitioning to state '${currentState}'.`);
                }
                else
                    rosnodejs.log.warn(`Received 'ready' message while already running.`);
            } finally{
                release();
            }
        });
    }

    function processResultMessage(message){
        rosnodejs.log.debug(`Processing 'result' message.`);
        stateMutex.acquire().then(release => {
            currentStateCopy = null;
            try{
                currentStateCopy = currentState;
                if(currentState == State.WAITING_RESULT){ //? Do I need to test this, or should I override the state?
                    currentState = State.WAITING_INPUT;
                    rosnodejs.log.debug(`Transitioning to state '${currentState}'.`);
                }
            } finally{
                release();
            }

            if(currentStateCopy != State.WAITING_RESULT)
                rosnodejs.log.warn("Unexpected result. Publishing anyway."); // TODO state that the state is overriddden in case we decide to do that

            posePub.publish(buildOutputMessage(message.poses, message.metadata));
        });
    }

    /**
     * TODO update description.
     * Callback for the pose detection when a single pose is considered.
     * 
     * This callback process the input ROS image using PoseNet to detect a
     * single pose. The result is published into the output topic.
     * @param {sensors_msgs.Image} imgageMsg A ROS image message.
     */
    async function rosImageCallback(imageMsg){
        rosnodejs.log.debug('New image received.');
        // console.time('tensor3d');
        // let tensor = tf.tensor3d(imageMsg.data, [imageMsg.height,imageMsg.width,3], 'int32');
        // // let imgData = formatImage(imageMsg);
        // console.timeEnd('tensor3d');
        // return;
        stateMutex.acquire().then(release => {
            processImage = false
            try{
                if(currentState == State.WAITING_INPUT){
                    currentState = State.WAITING_RESULT;
                    processImage = true;
                    rosnodejs.log.debug(`Transitioning to state '${currentState}'.`);
                }
            } finally{
                release();
            }

            if(processImage){
                t0 = rosnodejs.Time.toSeconds(imageMsg.header.stamp);
                t1 = rosnodejs.Time.toSeconds(rosnodejs.Time.now());

                rosnodejs.log.debug(`Sending image to PoseNet with delay = ${t1-t0}.`);
                //let tensor = tf.tensor3d(imageMsg.data, [imageMsg.height,imageMsg.width,3], 'int32');
                // const imgData = formatImage(imageMsg); // TODO change var name

                msg = {
                    type: 'classify',
                    image: imageMsg, 
                    metadata: imageMsg.header
                };
                console.log(`Msg size = ${sizeof(msg)}`);
                console.time('parent -> child');
                worker.send(msg);
                console.timeEnd('parent -> child');
            } else
                rosnodejs.log.debug(`Currently in state '${currentState}'. Discarding image.`)
        });
    }

    /**
     * ROS function reading parameters from the parameters server.
     * @param {String} key The parameter's name as per the launch file.
     * @param {*} default_value The default value that should be loaded in case
     *                          it is not provided.
     * @returns The value for the given parameter.
     */
    async function getParam (key, default_value){
        if(await rosNode.hasParam(key)){
            const param = await rosNode.getParam(key);
            return param;
        }
        rosnodejs.log.warn('Parameter ' + key +
            ' not found; using default value: ' + default_value);
        return default_value;
    }


    // /**
    //  * Callback for the pose detection when multiple poses are considered.
    //  * 
    //  * This callback process the input ROS image using PoseNet to detect
    //  * multiple poses. The result is published into the output topic.
    //  * @param {sensors_msgs.Image} imgData A ROS image message.
    //  */
    // async function multiPoseCallback(imgData){
    //     console.log("received ROS msg");
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

    /**
     * Converts a Pose object into a ROS message.
     * @param {[Poses]} poses The poses outputted by PoseNet.
     * @returns {pose_msgs.Poses} The ROS message that will be published.
     */
    function buildOutputMessage(poses, header) {
        let msg = new pose_msgs.Poses();
        msg.header.stamp = header.stamp;
        msg.header.frame_id = header.frame_id;
        for(let pIdx = 0; pIdx < poses.length; pIdx++){
            if (poses[pIdx]['score'] > paramMinPoseConf) {
                pose = new pose_msgs.Pose();
                pose.score = poses[pIdx]['score'];
                for(let kIdx = 0; kIdx < poses[pIdx]['keypoints'].length; kIdx++){
                    keypoint = new pose_msgs.Keypoint();
                    keypoint.score = poses[pIdx]['keypoints'][kIdx]['score'];
                    keypoint.part = poses[pIdx]['keypoints'][kIdx]['part'];
                    keypoint.position.x = poses[pIdx]['keypoints'][kIdx]['position']['x'];
                    keypoint.position.y = poses[pIdx]['keypoints'][kIdx]['position']['y'];
                    pose.keypoints.push(keypoint);
                };
                msg.poses.push(pose);
            }
        };
        return msg;
    }
}


// Executes the main function.
if (require.main === module)
    main();
