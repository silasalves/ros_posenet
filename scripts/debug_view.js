#!/usr/bin/env node

/**
 * ros_posenet Debug View
 * 
 * This module just provides the debugView function that draws the detected 
 * skeleton to a OpenCV name window.
 */

const cv = require('opencv4nodejs');

// Body parts detected by PoseNet.
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

// List of connected body parts.
const connected_part_names = [
    [leftHip, leftShoulder], [leftElbow, leftShoulder],
    [leftElbow, leftWrist], [leftHip, leftKnee],
    [leftKnee, leftAnkle], [rightHip, rightShoulder],
    [rightElbow, rightShoulder], [rightElbow, rightWrist],
    [rightHip, rightKnee], [rightKnee, rightAnkle],
    [leftShoulder, rightShoulder], [leftHip, rightHip]]

// List of different colors for drawing several skeletons.
const skeleton_colors=[
    new cv.Vec3(0, 0, 255),
    new cv.Vec3(0, 255, 255),
    new cv.Vec3(255, 0, 0),
    new cv.Vec3(0, 255, 0),
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
 * @param {cv.cvmat} imgData A cvmat image.
 * @param {Pose} poses The poses detected by PoseNet.
 */
exports.debugView = function (imgData, poses, minPoseScore, minPartScore) {
    img = imgData.cvtColor(cv.COLOR_RGB2BGR);
        
    poses.forEach( function(pose, i) {
        if (pose['score'] < minPoseScore)
            return;

        color = skeleton_colors[i % skeleton_colors.length]

        connected_part_names.forEach( pair => {
            if (pose['keypoints'][pair[0]]['score'] > 0.2 && 
                pose['keypoints'][pair[1]]['score'] > 0.2) {
                let p0 = pose['keypoints'][pair[0]]['position']
                let p1 = pose['keypoints'][pair[1]]['position']
                img.drawLine(new cv.Point(p0['x'], p0['y']), 
                             new cv.Point(p1['x'], p1['y']), color, 8);
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

    cv.imshow('ros_posenet_debug', img)//.resize(1080,1920))
    cv.waitKey(1);
}