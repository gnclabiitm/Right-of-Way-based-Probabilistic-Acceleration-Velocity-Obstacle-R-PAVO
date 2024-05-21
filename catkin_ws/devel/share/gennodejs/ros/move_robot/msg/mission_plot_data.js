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

class mission_plot_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.t_mission = null;
      this.d_min = null;
      this.t_comp = null;
      this.a_rms = null;
    }
    else {
      if (initObj.hasOwnProperty('t_mission')) {
        this.t_mission = initObj.t_mission
      }
      else {
        this.t_mission = 0.0;
      }
      if (initObj.hasOwnProperty('d_min')) {
        this.d_min = initObj.d_min
      }
      else {
        this.d_min = 0.0;
      }
      if (initObj.hasOwnProperty('t_comp')) {
        this.t_comp = initObj.t_comp
      }
      else {
        this.t_comp = 0.0;
      }
      if (initObj.hasOwnProperty('a_rms')) {
        this.a_rms = initObj.a_rms
      }
      else {
        this.a_rms = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mission_plot_data
    // Serialize message field [t_mission]
    bufferOffset = _serializer.float64(obj.t_mission, buffer, bufferOffset);
    // Serialize message field [d_min]
    bufferOffset = _serializer.float64(obj.d_min, buffer, bufferOffset);
    // Serialize message field [t_comp]
    bufferOffset = _serializer.float64(obj.t_comp, buffer, bufferOffset);
    // Serialize message field [a_rms]
    bufferOffset = _serializer.float64(obj.a_rms, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mission_plot_data
    let len;
    let data = new mission_plot_data(null);
    // Deserialize message field [t_mission]
    data.t_mission = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d_min]
    data.d_min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t_comp]
    data.t_comp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a_rms]
    data.a_rms = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'move_robot/mission_plot_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc7d6c5759137cf046d2c010b9c936e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 t_mission
    float64 d_min
    float64 t_comp
    float64 a_rms
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mission_plot_data(null);
    if (msg.t_mission !== undefined) {
      resolved.t_mission = msg.t_mission;
    }
    else {
      resolved.t_mission = 0.0
    }

    if (msg.d_min !== undefined) {
      resolved.d_min = msg.d_min;
    }
    else {
      resolved.d_min = 0.0
    }

    if (msg.t_comp !== undefined) {
      resolved.t_comp = msg.t_comp;
    }
    else {
      resolved.t_comp = 0.0
    }

    if (msg.a_rms !== undefined) {
      resolved.a_rms = msg.a_rms;
    }
    else {
      resolved.a_rms = 0.0
    }

    return resolved;
    }
};

module.exports = mission_plot_data;
