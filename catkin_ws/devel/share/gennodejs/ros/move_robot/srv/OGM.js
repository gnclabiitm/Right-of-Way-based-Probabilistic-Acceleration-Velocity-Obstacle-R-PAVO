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

class OGMRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tb3_id = null;
      this.ogm_node_num = null;
    }
    else {
      if (initObj.hasOwnProperty('tb3_id')) {
        this.tb3_id = initObj.tb3_id
      }
      else {
        this.tb3_id = '';
      }
      if (initObj.hasOwnProperty('ogm_node_num')) {
        this.ogm_node_num = initObj.ogm_node_num
      }
      else {
        this.ogm_node_num = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OGMRequest
    // Serialize message field [tb3_id]
    bufferOffset = _serializer.string(obj.tb3_id, buffer, bufferOffset);
    // Serialize message field [ogm_node_num]
    bufferOffset = _serializer.string(obj.ogm_node_num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OGMRequest
    let len;
    let data = new OGMRequest(null);
    // Deserialize message field [tb3_id]
    data.tb3_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ogm_node_num]
    data.ogm_node_num = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.tb3_id);
    length += _getByteLength(object.ogm_node_num);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'move_robot/OGMRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9c43752e04cc43094d0d29a847153b49';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string tb3_id
    string ogm_node_num
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OGMRequest(null);
    if (msg.tb3_id !== undefined) {
      resolved.tb3_id = msg.tb3_id;
    }
    else {
      resolved.tb3_id = ''
    }

    if (msg.ogm_node_num !== undefined) {
      resolved.ogm_node_num = msg.ogm_node_num;
    }
    else {
      resolved.ogm_node_num = ''
    }

    return resolved;
    }
};

class OGMResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dummy = null;
    }
    else {
      if (initObj.hasOwnProperty('dummy')) {
        this.dummy = initObj.dummy
      }
      else {
        this.dummy = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OGMResponse
    // Serialize message field [dummy]
    bufferOffset = _serializer.string(obj.dummy, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OGMResponse
    let len;
    let data = new OGMResponse(null);
    // Deserialize message field [dummy]
    data.dummy = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.dummy);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'move_robot/OGMResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5eab3f8fe195848369929fb790db61c1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string dummy
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OGMResponse(null);
    if (msg.dummy !== undefined) {
      resolved.dummy = msg.dummy;
    }
    else {
      resolved.dummy = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: OGMRequest,
  Response: OGMResponse,
  md5sum() { return 'a3ef7e635198434f8c5d1fcc111d7ba4'; },
  datatype() { return 'move_robot/OGM'; }
};
