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

class plot_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.curr_velocity_x = null;
      this.curr_velocity_y = null;
      this.des_velocity_x = null;
      this.des_velocity_y = null;
      this.t_iter = null;
      this.delta = null;
      this.t_horizon = null;
      this.avoid_i = null;
      this.a_long = null;
      this.a_lat = null;
      this.vel_mag = null;
      this.a_long_lv = null;
      this.a_lat_wv = null;
      this.accel = null;
    }
    else {
      if (initObj.hasOwnProperty('curr_velocity_x')) {
        this.curr_velocity_x = initObj.curr_velocity_x
      }
      else {
        this.curr_velocity_x = 0.0;
      }
      if (initObj.hasOwnProperty('curr_velocity_y')) {
        this.curr_velocity_y = initObj.curr_velocity_y
      }
      else {
        this.curr_velocity_y = 0.0;
      }
      if (initObj.hasOwnProperty('des_velocity_x')) {
        this.des_velocity_x = initObj.des_velocity_x
      }
      else {
        this.des_velocity_x = 0.0;
      }
      if (initObj.hasOwnProperty('des_velocity_y')) {
        this.des_velocity_y = initObj.des_velocity_y
      }
      else {
        this.des_velocity_y = 0.0;
      }
      if (initObj.hasOwnProperty('t_iter')) {
        this.t_iter = initObj.t_iter
      }
      else {
        this.t_iter = 0.0;
      }
      if (initObj.hasOwnProperty('delta')) {
        this.delta = initObj.delta
      }
      else {
        this.delta = 0.0;
      }
      if (initObj.hasOwnProperty('t_horizon')) {
        this.t_horizon = initObj.t_horizon
      }
      else {
        this.t_horizon = 0.0;
      }
      if (initObj.hasOwnProperty('avoid_i')) {
        this.avoid_i = initObj.avoid_i
      }
      else {
        this.avoid_i = 0.0;
      }
      if (initObj.hasOwnProperty('a_long')) {
        this.a_long = initObj.a_long
      }
      else {
        this.a_long = 0.0;
      }
      if (initObj.hasOwnProperty('a_lat')) {
        this.a_lat = initObj.a_lat
      }
      else {
        this.a_lat = 0.0;
      }
      if (initObj.hasOwnProperty('vel_mag')) {
        this.vel_mag = initObj.vel_mag
      }
      else {
        this.vel_mag = 0.0;
      }
      if (initObj.hasOwnProperty('a_long_lv')) {
        this.a_long_lv = initObj.a_long_lv
      }
      else {
        this.a_long_lv = 0.0;
      }
      if (initObj.hasOwnProperty('a_lat_wv')) {
        this.a_lat_wv = initObj.a_lat_wv
      }
      else {
        this.a_lat_wv = 0.0;
      }
      if (initObj.hasOwnProperty('accel')) {
        this.accel = initObj.accel
      }
      else {
        this.accel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type plot_data
    // Serialize message field [curr_velocity_x]
    bufferOffset = _serializer.float64(obj.curr_velocity_x, buffer, bufferOffset);
    // Serialize message field [curr_velocity_y]
    bufferOffset = _serializer.float64(obj.curr_velocity_y, buffer, bufferOffset);
    // Serialize message field [des_velocity_x]
    bufferOffset = _serializer.float64(obj.des_velocity_x, buffer, bufferOffset);
    // Serialize message field [des_velocity_y]
    bufferOffset = _serializer.float64(obj.des_velocity_y, buffer, bufferOffset);
    // Serialize message field [t_iter]
    bufferOffset = _serializer.float64(obj.t_iter, buffer, bufferOffset);
    // Serialize message field [delta]
    bufferOffset = _serializer.float64(obj.delta, buffer, bufferOffset);
    // Serialize message field [t_horizon]
    bufferOffset = _serializer.float64(obj.t_horizon, buffer, bufferOffset);
    // Serialize message field [avoid_i]
    bufferOffset = _serializer.float64(obj.avoid_i, buffer, bufferOffset);
    // Serialize message field [a_long]
    bufferOffset = _serializer.float64(obj.a_long, buffer, bufferOffset);
    // Serialize message field [a_lat]
    bufferOffset = _serializer.float64(obj.a_lat, buffer, bufferOffset);
    // Serialize message field [vel_mag]
    bufferOffset = _serializer.float64(obj.vel_mag, buffer, bufferOffset);
    // Serialize message field [a_long_lv]
    bufferOffset = _serializer.float64(obj.a_long_lv, buffer, bufferOffset);
    // Serialize message field [a_lat_wv]
    bufferOffset = _serializer.float64(obj.a_lat_wv, buffer, bufferOffset);
    // Serialize message field [accel]
    bufferOffset = _serializer.float64(obj.accel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type plot_data
    let len;
    let data = new plot_data(null);
    // Deserialize message field [curr_velocity_x]
    data.curr_velocity_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [curr_velocity_y]
    data.curr_velocity_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [des_velocity_x]
    data.des_velocity_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [des_velocity_y]
    data.des_velocity_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t_iter]
    data.t_iter = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta]
    data.delta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [t_horizon]
    data.t_horizon = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [avoid_i]
    data.avoid_i = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a_long]
    data.a_long = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a_lat]
    data.a_lat = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_mag]
    data.vel_mag = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a_long_lv]
    data.a_long_lv = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [a_lat_wv]
    data.a_lat_wv = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel]
    data.accel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 112;
  }

  static datatype() {
    // Returns string type for a message object
    return 'move_robot/plot_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6b90cbb3473e15ae3fe0a7a90e7e54aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 curr_velocity_x
    float64 curr_velocity_y
    float64 des_velocity_x
    float64 des_velocity_y
    float64 t_iter
    float64 delta
    float64 t_horizon
    float64 avoid_i
    float64 a_long
    float64 a_lat
    float64 vel_mag
    float64 a_long_lv
    float64 a_lat_wv
    float64 accel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new plot_data(null);
    if (msg.curr_velocity_x !== undefined) {
      resolved.curr_velocity_x = msg.curr_velocity_x;
    }
    else {
      resolved.curr_velocity_x = 0.0
    }

    if (msg.curr_velocity_y !== undefined) {
      resolved.curr_velocity_y = msg.curr_velocity_y;
    }
    else {
      resolved.curr_velocity_y = 0.0
    }

    if (msg.des_velocity_x !== undefined) {
      resolved.des_velocity_x = msg.des_velocity_x;
    }
    else {
      resolved.des_velocity_x = 0.0
    }

    if (msg.des_velocity_y !== undefined) {
      resolved.des_velocity_y = msg.des_velocity_y;
    }
    else {
      resolved.des_velocity_y = 0.0
    }

    if (msg.t_iter !== undefined) {
      resolved.t_iter = msg.t_iter;
    }
    else {
      resolved.t_iter = 0.0
    }

    if (msg.delta !== undefined) {
      resolved.delta = msg.delta;
    }
    else {
      resolved.delta = 0.0
    }

    if (msg.t_horizon !== undefined) {
      resolved.t_horizon = msg.t_horizon;
    }
    else {
      resolved.t_horizon = 0.0
    }

    if (msg.avoid_i !== undefined) {
      resolved.avoid_i = msg.avoid_i;
    }
    else {
      resolved.avoid_i = 0.0
    }

    if (msg.a_long !== undefined) {
      resolved.a_long = msg.a_long;
    }
    else {
      resolved.a_long = 0.0
    }

    if (msg.a_lat !== undefined) {
      resolved.a_lat = msg.a_lat;
    }
    else {
      resolved.a_lat = 0.0
    }

    if (msg.vel_mag !== undefined) {
      resolved.vel_mag = msg.vel_mag;
    }
    else {
      resolved.vel_mag = 0.0
    }

    if (msg.a_long_lv !== undefined) {
      resolved.a_long_lv = msg.a_long_lv;
    }
    else {
      resolved.a_long_lv = 0.0
    }

    if (msg.a_lat_wv !== undefined) {
      resolved.a_lat_wv = msg.a_lat_wv;
    }
    else {
      resolved.a_lat_wv = 0.0
    }

    if (msg.accel !== undefined) {
      resolved.accel = msg.accel;
    }
    else {
      resolved.accel = 0.0
    }

    return resolved;
    }
};

module.exports = plot_data;
