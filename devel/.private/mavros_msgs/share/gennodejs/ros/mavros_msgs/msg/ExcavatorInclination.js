// Auto-generated. Do not edit!

// (in-package mavros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ExcavatorInclination {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.boom_deg = null;
      this.forearm_deg = null;
      this.bucket_deg = null;
      this.boom_vel = null;
      this.forearm_vel = null;
      this.bucket_vel = null;
      this.cylinder_boom = null;
      this.cylinder_forearm = null;
      this.cylinder_bucket = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('boom_deg')) {
        this.boom_deg = initObj.boom_deg
      }
      else {
        this.boom_deg = 0.0;
      }
      if (initObj.hasOwnProperty('forearm_deg')) {
        this.forearm_deg = initObj.forearm_deg
      }
      else {
        this.forearm_deg = 0.0;
      }
      if (initObj.hasOwnProperty('bucket_deg')) {
        this.bucket_deg = initObj.bucket_deg
      }
      else {
        this.bucket_deg = 0.0;
      }
      if (initObj.hasOwnProperty('boom_vel')) {
        this.boom_vel = initObj.boom_vel
      }
      else {
        this.boom_vel = 0.0;
      }
      if (initObj.hasOwnProperty('forearm_vel')) {
        this.forearm_vel = initObj.forearm_vel
      }
      else {
        this.forearm_vel = 0.0;
      }
      if (initObj.hasOwnProperty('bucket_vel')) {
        this.bucket_vel = initObj.bucket_vel
      }
      else {
        this.bucket_vel = 0.0;
      }
      if (initObj.hasOwnProperty('cylinder_boom')) {
        this.cylinder_boom = initObj.cylinder_boom
      }
      else {
        this.cylinder_boom = 0.0;
      }
      if (initObj.hasOwnProperty('cylinder_forearm')) {
        this.cylinder_forearm = initObj.cylinder_forearm
      }
      else {
        this.cylinder_forearm = 0.0;
      }
      if (initObj.hasOwnProperty('cylinder_bucket')) {
        this.cylinder_bucket = initObj.cylinder_bucket
      }
      else {
        this.cylinder_bucket = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExcavatorInclination
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [boom_deg]
    bufferOffset = _serializer.float64(obj.boom_deg, buffer, bufferOffset);
    // Serialize message field [forearm_deg]
    bufferOffset = _serializer.float64(obj.forearm_deg, buffer, bufferOffset);
    // Serialize message field [bucket_deg]
    bufferOffset = _serializer.float64(obj.bucket_deg, buffer, bufferOffset);
    // Serialize message field [boom_vel]
    bufferOffset = _serializer.float64(obj.boom_vel, buffer, bufferOffset);
    // Serialize message field [forearm_vel]
    bufferOffset = _serializer.float64(obj.forearm_vel, buffer, bufferOffset);
    // Serialize message field [bucket_vel]
    bufferOffset = _serializer.float64(obj.bucket_vel, buffer, bufferOffset);
    // Serialize message field [cylinder_boom]
    bufferOffset = _serializer.float64(obj.cylinder_boom, buffer, bufferOffset);
    // Serialize message field [cylinder_forearm]
    bufferOffset = _serializer.float64(obj.cylinder_forearm, buffer, bufferOffset);
    // Serialize message field [cylinder_bucket]
    bufferOffset = _serializer.float64(obj.cylinder_bucket, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExcavatorInclination
    let len;
    let data = new ExcavatorInclination(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [boom_deg]
    data.boom_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [forearm_deg]
    data.forearm_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bucket_deg]
    data.bucket_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [boom_vel]
    data.boom_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [forearm_vel]
    data.forearm_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [bucket_vel]
    data.bucket_vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cylinder_boom]
    data.cylinder_boom = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cylinder_forearm]
    data.cylinder_forearm = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cylinder_bucket]
    data.cylinder_bucket = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mavros_msgs/ExcavatorInclination';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1c5cdfd62e264348da655a1f34c898bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Stamped Excavator Inclination message
    #
    # For streaming timestamped data from FCU inclination.
    
    std_msgs/Header header
    float64 boom_deg
    float64 forearm_deg
    float64 bucket_deg
    float64 boom_vel
    float64 forearm_vel
    float64 bucket_vel
    float64 cylinder_boom    
    float64 cylinder_forearm 
    float64 cylinder_bucket  
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExcavatorInclination(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.boom_deg !== undefined) {
      resolved.boom_deg = msg.boom_deg;
    }
    else {
      resolved.boom_deg = 0.0
    }

    if (msg.forearm_deg !== undefined) {
      resolved.forearm_deg = msg.forearm_deg;
    }
    else {
      resolved.forearm_deg = 0.0
    }

    if (msg.bucket_deg !== undefined) {
      resolved.bucket_deg = msg.bucket_deg;
    }
    else {
      resolved.bucket_deg = 0.0
    }

    if (msg.boom_vel !== undefined) {
      resolved.boom_vel = msg.boom_vel;
    }
    else {
      resolved.boom_vel = 0.0
    }

    if (msg.forearm_vel !== undefined) {
      resolved.forearm_vel = msg.forearm_vel;
    }
    else {
      resolved.forearm_vel = 0.0
    }

    if (msg.bucket_vel !== undefined) {
      resolved.bucket_vel = msg.bucket_vel;
    }
    else {
      resolved.bucket_vel = 0.0
    }

    if (msg.cylinder_boom !== undefined) {
      resolved.cylinder_boom = msg.cylinder_boom;
    }
    else {
      resolved.cylinder_boom = 0.0
    }

    if (msg.cylinder_forearm !== undefined) {
      resolved.cylinder_forearm = msg.cylinder_forearm;
    }
    else {
      resolved.cylinder_forearm = 0.0
    }

    if (msg.cylinder_bucket !== undefined) {
      resolved.cylinder_bucket = msg.cylinder_bucket;
    }
    else {
      resolved.cylinder_bucket = 0.0
    }

    return resolved;
    }
};

module.exports = ExcavatorInclination;
