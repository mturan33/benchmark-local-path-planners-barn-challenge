;; Auto-generated. Do not edit!


(when (boundp 'jackal_msgs::Feedback)
  (if (not (find-package "JACKAL_MSGS"))
    (make-package "JACKAL_MSGS"))
  (shadow 'Feedback (find-package "JACKAL_MSGS")))
(unless (find-package "JACKAL_MSGS::FEEDBACK")
  (make-package "JACKAL_MSGS::FEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude Feedback.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jackal_msgs::Feedback
  :super ros::object
  :slots (_header _drivers _pcb_temperature _mcu_temperature _commanded_mode _actual_mode ))

(defmethod jackal_msgs::Feedback
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drivers __drivers) (let (r) (dotimes (i 2) (push (instance jackal_msgs::DriveFeedback :init) r)) r))
    ((:pcb_temperature __pcb_temperature) 0.0)
    ((:mcu_temperature __mcu_temperature) 0.0)
    ((:commanded_mode __commanded_mode) 0)
    ((:actual_mode __actual_mode) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drivers __drivers)
   (setq _pcb_temperature (float __pcb_temperature))
   (setq _mcu_temperature (float __mcu_temperature))
   (setq _commanded_mode (round __commanded_mode))
   (setq _actual_mode (round __actual_mode))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:drivers
   (&rest __drivers)
   (if (keywordp (car __drivers))
       (send* _drivers __drivers)
     (progn
       (if __drivers (setq _drivers (car __drivers)))
       _drivers)))
  (:pcb_temperature
   (&optional __pcb_temperature)
   (if __pcb_temperature (setq _pcb_temperature __pcb_temperature)) _pcb_temperature)
  (:mcu_temperature
   (&optional __mcu_temperature)
   (if __mcu_temperature (setq _mcu_temperature __mcu_temperature)) _mcu_temperature)
  (:commanded_mode
   (&optional __commanded_mode)
   (if __commanded_mode (setq _commanded_mode __commanded_mode)) _commanded_mode)
  (:actual_mode
   (&optional __actual_mode)
   (if __actual_mode (setq _actual_mode __actual_mode)) _actual_mode)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; jackal_msgs/DriveFeedback[2] _drivers
    (apply #'+ (send-all _drivers :serialization-length))
    ;; float32 _pcb_temperature
    4
    ;; float32 _mcu_temperature
    4
    ;; int8 _commanded_mode
    1
    ;; int8 _actual_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; jackal_msgs/DriveFeedback[2] _drivers
     (dolist (elem _drivers)
       (send elem :serialize s)
       )
     ;; float32 _pcb_temperature
       (sys::poke _pcb_temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mcu_temperature
       (sys::poke _mcu_temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _commanded_mode
       (write-byte _commanded_mode s)
     ;; int8 _actual_mode
       (write-byte _actual_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; jackal_msgs/DriveFeedback[2] _drivers
   (dotimes (i 2)
     (send (elt _drivers i) :deserialize buf ptr-) (incf ptr- (send (elt _drivers i) :serialization-length))
     )
   ;; float32 _pcb_temperature
     (setq _pcb_temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mcu_temperature
     (setq _mcu_temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _commanded_mode
     (setq _commanded_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _commanded_mode 127) (setq _commanded_mode (- _commanded_mode 256)))
   ;; int8 _actual_mode
     (setq _actual_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _actual_mode 127) (setq _actual_mode (- _actual_mode 256)))
   ;;
   self)
  )

(setf (get jackal_msgs::Feedback :md5sum-) "3bdabb0ef46338ee5672d1b82220ab49")
(setf (get jackal_msgs::Feedback :datatype-) "jackal_msgs/Feedback")
(setf (get jackal_msgs::Feedback :definition-)
      "# This message represents high-frequency feedback from the MCU,
# as necessary to support closed-loop control and thermal monitoring.
# Default publish frequency is 50Hz.

Header header

DriveFeedback[2] drivers

# Temperatures 
float32 pcb_temperature
float32 mcu_temperature

# Commanded control mode, use the TYPE_ constants from jackal_msgs/Drive.
int8 commanded_mode

# Actual control mode. This may differ from the commanded in cases where
# the motor enable is off, the motors are in over-current, etc.
int8 actual_mode

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
MSG: jackal_msgs/DriveFeedback
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

")



(provide :jackal_msgs/Feedback "3bdabb0ef46338ee5672d1b82220ab49")


