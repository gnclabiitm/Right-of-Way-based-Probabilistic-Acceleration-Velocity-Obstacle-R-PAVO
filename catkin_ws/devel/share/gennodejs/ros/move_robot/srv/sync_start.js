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

class sync_startRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.check_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('check_flag')) {
        this.check_flag = initObj.check_flag
      }
      else {
        this.check_flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sync_startRequest
    // Serialize message field [check_flag]
    bufferOffset = _serializer.bool(obj.check_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sync_startRequest
    let len;
    let data = new sync_startRequest(null);
    // Deserialize message field [check_flag]
    data.check_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'move_robot/sync_startRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '90af77130be3c7ed1e0b41f6980836dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool check_flag
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sync_startRequest(null);
    if (msg.check_flag !== undefined) {
      resolved.check_flag = msg.check_flag;
    }
    else {
      resolved.check_flag = false
    }

    return resolved;
    }
};

class sync_startResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.set_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('set_flag')) {
        this.set_flag = initObj.set_flag
      }
      else {
        this.set_flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sync_startResponse
    // Serialize message field [set_flag]
    bufferOffset = _serializer.bool(obj.set_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sync_startResponse
    let len;
    let data = new sync_startResponse(null);
    // Deserialize message field [set_flag]
    data.set_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'move_robot/sync_startResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '919be5a0fb2f3210683a50ba5ac76ad6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool set_flag
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sync_startResponse(null);
    if (msg.set_flag !== undefined) {
      resolved.set_flag = msg.set_flag;
    }
    else {
      resolved.set_flag = false
    }

    return resolved;
    }
};

module.exports = {
  Request: sync_startRequest,
  Response: sync_startResponse,
  md5sum() { return 'f28b7d6543382a4046cd196e0567aca4'; },
  datatype() { return 'move_robot/sync_start'; }
};
