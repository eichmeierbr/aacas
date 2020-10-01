// Auto-generated. Do not edit!

// (in-package lidar_process.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let tracked_obj = require('./tracked_obj.js');

//-----------------------------------------------------------

class tracked_obj_arr {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tracked_obj_arr = null;
    }
    else {
      if (initObj.hasOwnProperty('tracked_obj_arr')) {
        this.tracked_obj_arr = initObj.tracked_obj_arr
      }
      else {
        this.tracked_obj_arr = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type tracked_obj_arr
    // Serialize message field [tracked_obj_arr]
    // Serialize the length for message field [tracked_obj_arr]
    bufferOffset = _serializer.uint32(obj.tracked_obj_arr.length, buffer, bufferOffset);
    obj.tracked_obj_arr.forEach((val) => {
      bufferOffset = tracked_obj.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type tracked_obj_arr
    let len;
    let data = new tracked_obj_arr(null);
    // Deserialize message field [tracked_obj_arr]
    // Deserialize array length for message field [tracked_obj_arr]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tracked_obj_arr = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tracked_obj_arr[i] = tracked_obj.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.tracked_obj_arr.forEach((val) => {
      length += tracked_obj.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_process/tracked_obj_arr';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '16c935949bd8cf174eedb225267fa459';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    tracked_obj[] tracked_obj_arr
    
    ================================================================================
    MSG: lidar_process/tracked_obj
    Header header
    uint32 object_id
    string object_type
    geometry_msgs/Point point
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new tracked_obj_arr(null);
    if (msg.tracked_obj_arr !== undefined) {
      resolved.tracked_obj_arr = new Array(msg.tracked_obj_arr.length);
      for (let i = 0; i < resolved.tracked_obj_arr.length; ++i) {
        resolved.tracked_obj_arr[i] = tracked_obj.Resolve(msg.tracked_obj_arr[i]);
      }
    }
    else {
      resolved.tracked_obj_arr = []
    }

    return resolved;
    }
};

module.exports = tracked_obj_arr;
