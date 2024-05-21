// Auto-generated. Do not edit!

// (in-package move_robot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class las_mes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.las_dist = null;
      this.las_dist_x = null;
      this.las_dist_y = null;
      this.las_angles = null;
    }
    else {
      if (initObj.hasOwnProperty('las_dist')) {
        this.las_dist = initObj.las_dist
      }
      else {
        this.las_dist = [];
      }
      if (initObj.hasOwnProperty('las_dist_x')) {
        this.las_dist_x = initObj.las_dist_x
      }
      else {
        this.las_dist_x = [];
      }
      if (initObj.hasOwnProperty('las_dist_y')) {
        this.las_dist_y = initObj.las_dist_y
      }
      else {
        this.las_dist_y = [];
      }
      if (initObj.hasOwnProperty('las_angles')) {
        this.las_angles = initObj.las_angles
      }
      else {
        this.las_angles = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type las_mes
    // Serialize message field [las_dist]
    bufferOffset = _arraySerializer.float64(obj.las_dist, buffer, bufferOffset, null);
    // Serialize message field [las_dist_x]
    bufferOffset = _arraySerializer.float64(obj.las_dist_x, buffer, bufferOffset, null);
    // Serialize message field [las_dist_y]
    bufferOffset = _arraySerializer.float64(obj.las_dist_y, buffer, bufferOffset, null);
    // Serialize message field [las_angles]
    bufferOffset = _arraySerializer.float64(obj.las_angles, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type las_mes
    let len;
    let data = new las_mes(null);
    // Deserialize message field [las_dist]
    data.las_dist = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [las_dist_x]
    data.las_dist_x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [las_dist_y]
    data.las_dist_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [las_angles]
    data.las_angles = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.las_dist.length;
    length += 8 * object.las_dist_x.length;
    length += 8 * object.las_dist_y.length;
    length += 8 * object.las_angles.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'move_robot/las_mes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aad5a2de89d4ae32cf5dfd4ccf601549';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] las_dist
    float64[] las_dist_x
    float64[] las_dist_y
    float64[] las_angles
    # float64[] velocities
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new las_mes(null);
    if (msg.las_dist !== undefined) {
      resolved.las_dist = msg.las_dist;
    }
    else {
      resolved.las_dist = []
    }

    if (msg.las_dist_x !== undefined) {
      resolved.las_dist_x = msg.las_dist_x;
    }
    else {
      resolved.las_dist_x = []
    }

    if (msg.las_dist_y !== undefined) {
      resolved.las_dist_y = msg.las_dist_y;
    }
    else {
      resolved.las_dist_y = []
    }

    if (msg.las_angles !== undefined) {
      resolved.las_angles = msg.las_angles;
    }
    else {
      resolved.las_angles = []
    }

    return resolved;
    }
};

module.exports = las_mes;
