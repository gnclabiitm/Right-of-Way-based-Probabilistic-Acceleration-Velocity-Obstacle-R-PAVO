// Auto-generated. Do not edit!

// (in-package move_robot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SyncMotionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trigger = null;
    }
    else {
      if (initObj.hasOwnProperty('trigger')) {
        this.trigger = initObj.trigger
      }
      else {
        this.trigger = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SyncMotionRequest
    // Serialize message field [trigger]
    bufferOffset = _serializer.bool(obj.trigger, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SyncMotionRequest
    let len;
    let data = new SyncMotionRequest(null);
    // Deserialize message field [trigger]
    data.trigger = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'move_robot/SyncMotionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f6d1152a533bdef9ec687318c8e489b0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool trigger
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SyncMotionRequest(null);
    if (msg.trigger !== undefined) {
      resolved.trigger = msg.trigger;
    }
    else {
      resolved.trigger = false
    }

    return resolved;
    }
};

class SyncMotionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sync_success = null;
    }
    else {
      if (initObj.hasOwnProperty('sync_success')) {
        this.sync_success = initObj.sync_success
      }
      else {
        this.sync_success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SyncMotionResponse
    // Serialize message field [sync_success]
    bufferOffset = _serializer.bool(obj.sync_success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SyncMotionResponse
    let len;
    let data = new SyncMotionResponse(null);
    // Deserialize message field [sync_success]
    data.sync_success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'move_robot/SyncMotionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24f79b536c77d75cf8239dd37b943368';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool sync_success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SyncMotionResponse(null);
    if (msg.sync_success !== undefined) {
      resolved.sync_success = msg.sync_success;
    }
    else {
      resolved.sync_success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SyncMotionRequest,
  Response: SyncMotionResponse,
  md5sum() { return 'e22398fd32450a580e4c379bfee2569b'; },
  datatype() { return 'move_robot/SyncMotion'; }
};
