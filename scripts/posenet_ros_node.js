#!/usr/bin/env node

/**
 * ros_posenet Main Script
 * 
 * This is the main script of the `ros_posenet` package. It connect to ROS to
 * receive color images and spaws a child process that runs PoseNet for
 * skeleton detection.
 * 
 * This distributed architecture was necessary because Node.js is 
 * single-threaded and PoseNet is CPU-intensive. On a singles process,
 * PoseNet would use all the processing time until reaching a result, which
 * prevented rosnodejs from reading the latest messages and managing
 * the buffer. When rosnodejs was finally allowed to process the image buffer,
 * all messages were old (around 0.8 second), so it would either (i) send old
 * images to PoseNet for classification, or (ii) repeatedly discard messages
 * until a recent message is received.
 * 
 * Both behaviors (i) and (ii) are undesirable: behavior (i) increased the
 * response delay of PoseNet by feeding it old messages, whereas behavior (ii)
 * decreased the overall throughput because it wasted time discarding old
 * messages.
 * 
 * To solve this problem, the current distributed implementation using 
 * Node.js child_process was devised. This enables ROS to process the message
 * buffer and keep only the most recent image while PoseNet is working on
 * classification, thus preserving the performance.
 * 
 * Using a Worker Thread would probably increase performance by reducing the
 * communication overhead. However, Worker Threads currently do not support
 * the HTML Canvas element, which is required by PoseNet.
 */

// Packages required by ROS.
const rosnodejs = require('rosnodejs');
const sensor_msgs = rosnodejs.require('sensor_msgs').msg;
const pose_msgs = rosnodejs.require('ros_posenet').msg;

// Packages required by PoseNet. The PoseNet package itself is loaded in the
// main function, as it requires the CPU or the GPU tensorflow library to be
// loaded first, and that depends on the configuration read from the launch
// file.
const cv = require('opencv4nodejs');
const { createImageData, createCanvas } = require('canvas')

const cp = require('child_process');
const Mutex = require('async-mutex').Mutex;
const { performance } = require('perf_hooks');

// If DEBUG is true, the prediction time will be printed, and an image with the
// detected poses will be shown.
const DEBUG = true;


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
        WAITING_RESULT: 'waiting_result'
    });

    const stateMutex = new Mutex(); // Controls access to currentState.
    let currentState = State.LOADING;

    // Register node with ros; `rosNode` is used to load the parameters.
    const rosNode = await rosnodejs.initNode("/posenet")

    if (DEBUG)
        rosnodejs.log.getLogger('ros').setLevel('debug');

    rosnodejs.log.info('Node /posenet registered.');
    rosnodejs.log.debug(`Transitioning to state '${currentState}'.`);


    // Load all parameters from `posenet.launch`.
    const paramImgTopic = await getParam('/posenet/compressed_image_topic',
                                         '/image_raw');
    const paramPosesTopic = await getParam('/posenet/poses_topic', '/poses');
    const paramGPU = await getParam('/posenet/gpu', false);
    const paramMaxDelay = await getParam('posenet/max_msg_time_diff', 0.03);
    const paramArchitecture = await getParam('/posenet/architecture', 
                                             'MobileNetV1');
    const paramMultiplier = await getParam('/posenet/multiplier', 0.5);
    let paramInputResolution = await getParam('/posenet/input_resolution', 257);
    const paramQuantBytes = await getParam('/posenet/quant_bytes', 4)
    const paramOutputStride = await getParam('/posenet/output_stride', 16);
    const paramFlipHorizontal = await getParam('/posenet/flip_horizontal', 
                                               false);
    const paramMultiPose = await getParam('/posenet/multi_pose', false);
    const paramMaxDetection = await getParam('/posenet/max_detection', 5);
    const paramMinPoseConf = await getParam('/posenet/min_pose_confidence',
                                            0.1);
    const paramMinPartConf = await getParam('/posenet/min_part_confidence', 
                                            0.5);
    const paramNmsRadius = await getParam('/posenet/nms_radius', 30);

    // Parameters sent to the child process as command line arguments.
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
        minPoseConf: paramMinPoseConf,
        debug: DEBUG
    }

    // Creates the child process and assigns all the message callbacks
    const worker = cp.fork(`${__dirname}//posenet_worker.js`,
        Object.values(posenet_config),
        {
            silent: true,
            detached: true
        });

    worker.on('message', workerMessageCallback);
    worker.on('error', workerErrorCallback);
    worker.on('exit', workerCloseCallback);
    
    worker.stdout.on('data', (data) => {
        rosnodejs.log.debug(data.toString());
    });

    worker.stderr.on('data', (data) => {
        rosnodejs.log.debug(data.toString());
    });

    // Advertises the output topic and subscribes to the image topic.
    posePub = rosNode.advertise(paramPosesTopic, pose_msgs.Poses);
    imgSub = rosNode.subscribe(paramImgTopic, sensor_msgs.CompressedImage,
        rosImageCallback, { queueSize: 1 });
    
    rosnodejs.on('shutdown', () => {
        rosnodejs.log.info("Shutting down ros_posenet");
    })

    // Main function ends here. Bellow you can find the utility functions and
    // callbacks.

    /**
     * Logs the error returned by the child process and terminates the parent.
     * 
     * @param {*} error Explains the error that occurred with the child process.
     */
    function workerErrorCallback(error) {
        rosnodejs.log.fatal(`The child process returned the ` +
                            `following error message:\n${error}\n\n` +
                            `Terminating the process.`);
        
        rosnodejs.shutdown();
    }

    /**
     * Logs the code and signal of the terminated child process.
     * 
     * @param {int} code   Code returned by the child's main function.
     */
    function workerCloseCallback(code) {
        rosnodejs.log.fatal(`The child process was terminated with the ` +
                            `code: ${code}. \n` +
                            `Terminating the process.`);
        
        rosnodejs.shutdown();
    }

    /**
     * Process the messages received from the child process.
     * 
     * This is the main callback that process all messages received from the 
     * child process. The messages should always have a `type` field which 
     * accepts the following values: [`ready`, `result` and `error`].
     * 
     * @param {object} message 
     */
    function workerMessageCallback(message) {
        rosnodejs.log.debug(`Received message with type ${message.type}`);
        if (message.hasOwnProperty('type')) {
            switch (message.type) {
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
                    rosnodejs.log.error(`Unknown message type: ` + 
                                        `'${message.type}'.`);
            }
        } else {
            rosnodejs.log.error('Ill-formatted message: \'type\' ' + 
                                'field is missing');
        }
    }

    /**
     * Starts sending the input images to the child.
     * 
     * Transits from the 'loading' state to the 'waiting input' state, which
     * allows the parent to send images to the child.
     */
    function processReadyMessage() {
        rosnodejs.log.debug('Processing the \'ready\' message');
        stateMutex.acquire().then(release => {
            try {
                if (currentState == State.LOADING) {
                    currentState = State.WAITING_INPUT;
                    rosnodejs.log.info('PoseNet loaded.');
                    rosnodejs.log.debug(`Transitioning to state ` +
                                        `'${currentState}'.`);
                }
                else
                    rosnodejs.log.warn(`Received 'ready' message while` + 
                                       `parent was already running.`);
            } finally {
                release();
            }
        });
    }

    /**
     * Publishes to ROS the poses received from the child.
     * 
     * @param {object} message Object with the poses returned by PoseNet. 
     */
    function processResultMessage(message) {
        rosnodejs.log.debug(`Processing 'result' message.`);
        stateMutex.acquire().then(release => {
            currentStateCopy = null;
            try {
                rosnodejs.log.debug(`Transitioning from state ` + 
                                    `'${currentState}' to state ` + 
                                    `'${State.WAITING_INPUT}'.`);
                currentStateCopy = currentState;
                if (currentState != State.WAITING_RESULT) {
                    rosnodejs.log.warn(`Unexpected result. FSM will be reset ` + 
                                       `and the result will be published.`);
                }
                currentState = State.WAITING_INPUT;
            } finally {
                release();
            }

            posePub.publish(buildOutputMessage(message.poses, 
                                               message.metadata));
        });
    }

    /**
     * Callback for the pose detection when a single pose is considered.
     * 
     * This callback process the input ROS image using PoseNet to detect a
     * single pose. The result is published into the output topic.
     * @param {sensors_msgs.CompressedImage} imgageMsg A compressed color image.
     */
    async function rosImageCallback(imageMsg) {
        rosnodejs.log.debug('New image received.');

        stateMutex.acquire().then(release => {
            processImage = false
            try {
                if (currentState == State.WAITING_INPUT) {
                    currentState = State.WAITING_RESULT;
                    processImage = true;
                    rosnodejs.log.debug(`Transitioning to state ` +
                                        `'${currentState}'.`);
                }
            } finally {
                release();
            }

            if (processImage) {
                t0 = rosnodejs.Time.toSeconds(imageMsg.header.stamp);
                t1 = rosnodejs.Time.toSeconds(rosnodejs.Time.now());

                rosnodejs.log.debug(`Sending image to PoseNet with delay = ` +
                                    `${t1 - t0}.`);

                msg = {
                    type: 'classify',
                    image: imageMsg,
                    metadata: imageMsg.header
                };

                t0 = performance.now();
                worker.send(msg);
                t1 = performance.now();
                rosnodejs.log.debug(`Sending message to child took ` +
                                    `${t1 - t0} ms`);
            } else
                rosnodejs.log.debug(`Currently in state '${currentState}'. ` +
                                    `Discarding image.`)
        });
    }

    /**
     * ROS function reading parameters from the parameters server.
     * @param {String} key The parameter's name as per the launch file.
     * @param {*} default_value The default value that should be loaded in case
     *                          it is not provided.
     * @returns The value for the given parameter.
     */
    async function getParam(key, default_value) {
        if (await rosNode.hasParam(key)) {
            const param = await rosNode.getParam(key);
            return param;
        }
        rosnodejs.log.warn('Parameter ' + key +
            ' not found; using default value: ' + default_value);
        return default_value;
    }


    /**
     * Converts a Pose object into a ROS message.
     * @param {[Poses]} poses The poses outputted by PoseNet.
     * @returns {pose_msgs.Poses} The ROS message that will be published.
     */
    function buildOutputMessage(poses, header) {
        let msg = new pose_msgs.Poses();
        msg.header.stamp = header.stamp;
        msg.header.frame_id = header.frame_id;
        for (pIdx = 0; pIdx < poses.length; pIdx++) {
            if (poses[pIdx]['score'] > paramMinPoseConf) {
                pose = new pose_msgs.Pose();
                pose.score = poses[pIdx]['score'];
                for (kIdx = 0; kIdx < poses[pIdx]['keypoints'].length; kIdx++) {
                    keypoint = new pose_msgs.Keypoint();
                    keypoint.score = poses[pIdx]['keypoints'][kIdx]['score'];
                    keypoint.part = poses[pIdx]['keypoints'][kIdx]['part'];
                    keypoint.position.x = 
                        poses[pIdx]['keypoints'][kIdx]['position']['x'];
                    keypoint.position.y = 
                        poses[pIdx]['keypoints'][kIdx]['position']['y'];
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
