// Auto-generated. Do not edit!

// (in-package jackal_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DriveFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.current = null;
      this.duty_cycle = null;
      this.bridge_temperature = null;
      this.motor_temperature = null;
      this.measured_velocity = null;
      this.measured_travel = null;
      this.driver_fault = null;
    }
    else {
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('duty_cycle')) {
        this.duty_cycle = initObj.duty_cycle
      }
      else {
        this.duty_cycle = 0.0;
      }
      if (initObj.hasOwnProperty('bridge_temperature')) {
        this.bridge_temperature = initObj.bridge_temperature
      }
      else {
        this.bridge_temperature = 0.0;
      }
      if (initObj.hasOwnProperty('motor_temperature')) {
        this.motor_temperature = initObj.motor_temperature
      }
      else {
        this.motor_temperature = 0.0;
      }
      if (initObj.hasOwnProperty('measured_velocity')) {
        this.measured_velocity = initObj.measured_velocity
      }
      else {
        this.measured_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('measured_travel')) {
        this.measured_travel = initObj.measured_travel
      }
      else {
        this.measured_travel = 0.0;
      }
      if (initObj.hasOwnProperty('driver_fault')) {
        this.driver_fault = initObj.driver_fault
      }
      else {
        this.driver_fault = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DriveFeedback
    // Serialize message field [current]
    bufferOffset = _serializer.float32(obj.current, buffer, bufferOffset);
    // Serialize message field [duty_cycle]
    bufferOffset = _serializer.float32(obj.duty_cycle, buffer, bufferOffset);
    // Serialize message field [bridge_temperature]
    bufferOffset = _serializer.float32(obj.bridge_temperature, buffer, bufferOffset);
    // Serialize message field [motor_temperature]
    bufferOffset = _serializer.float32(obj.motor_temperature, buffer, bufferOffset);
    // Serialize message field [measured_velocity]
    bufferOffset = _serializer.float32(obj.measured_velocity, buffer, bufferOffset);
    // Serialize message field [measured_travel]
    bufferOffset = _serializer.float32(obj.measured_travel, buffer, bufferOffset);
    // Serialize message field [driver_fault]
    bufferOffset = _serializer.bool(obj.driver_fault, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DriveFeedback
    let len;
    let data = new DriveFeedback(null);
    // Deserialize message field [current]
    data.current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duty_cycle]
    data.duty_cycle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bridge_temperature]
    data.bridge_temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motor_temperature]
    data.motor_temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_velocity]
    data.measured_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_travel]
    data.measured_travel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [driver_fault]
    data.driver_fault = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jackal_msgs/DriveFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8dd0b7a3cfa20cfc5c054ddd9763609b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message represents feedback data from a single drive unit (driver + motor).
    
    # Current flowing from battery into the MOSFET bridge.
    float32 current
    
    # Instantaneous duty cycle of MOSFET bridge.
    float32 duty_cycle
    
    # Temperatures measured in the MOSFET bridge and on the motor casing, in deg C.
    float32 bridge_temperature
    float32 motor_temperature
    
    # Encoder data
    float32 measured_velocity   # rad/s
    float32 measured_travel     # rad
    
    # True if the underlying driver chip reports a fault condition.
    bool driver_fault
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DriveFeedback(null);
    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.duty_cycle !== undefined) {
      resolved.duty_cycle = msg.duty_cycle;
    }
    else {
      resolved.duty_cycle = 0.0
    }

    if (msg.bridge_temperature !== undefined) {
      resolved.bridge_temperature = msg.bridge_temperature;
    }
    else {
      resolved.bridge_temperature = 0.0
    }

    if (msg.motor_temperature !== undefined) {
      resolved.motor_temperature = msg.motor_temperature;
    }
    else {
      resolved.motor_temperature = 0.0
    }

    if (msg.measured_velocity !== undefined) {
      resolved.measured_velocity = msg.measured_velocity;
    }
    else {
      resolved.measured_velocity = 0.0
    }

    if (msg.measured_travel !== undefined) {
      resolved.measured_travel = msg.measured_travel;
    }
    else {
      resolved.measured_travel = 0.0
    }

    if (msg.driver_fault !== undefined) {
      resolved.driver_fault = msg.driver_fault;
    }
    else {
      resolved.driver_fault = false
    }

    return resolved;
    }
};

module.exports = DriveFeedback;
