;; Auto-generated. Do not edit!


(when (boundp 'jackal_msgs::DriveFeedback)
  (if (not (find-package "JACKAL_MSGS"))
    (make-package "JACKAL_MSGS"))
  (shadow 'DriveFeedback (find-package "JACKAL_MSGS")))
(unless (find-package "JACKAL_MSGS::DRIVEFEEDBACK")
  (make-package "JACKAL_MSGS::DRIVEFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude DriveFeedback.msg.html


(defclass jackal_msgs::DriveFeedback
  :super ros::object
  :slots (_current _duty_cycle _bridge_temperature _motor_temperature _measured_velocity _measured_travel _driver_fault ))

(defmethod jackal_msgs::DriveFeedback
  (:init
   (&key
    ((:current __current) 0.0)
    ((:duty_cycle __duty_cycle) 0.0)
    ((:bridge_temperature __bridge_temperature) 0.0)
    ((:motor_temperature __motor_temperature) 0.0)
    ((:measured_velocity __measured_velocity) 0.0)
    ((:measured_travel __measured_travel) 0.0)
    ((:driver_fault __driver_fault) nil)
    )
   (send-super :init)
   (setq _current (float __current))
   (setq _duty_cycle (float __duty_cycle))
   (setq _bridge_temperature (float __bridge_temperature))
   (setq _motor_temperature (float __motor_temperature))
   (setq _measured_velocity (float __measured_velocity))
   (setq _measured_travel (float __measured_travel))
   (setq _driver_fault __driver_fault)
   self)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:duty_cycle
   (&optional __duty_cycle)
   (if __duty_cycle (setq _duty_cycle __duty_cycle)) _duty_cycle)
  (:bridge_temperature
   (&optional __bridge_temperature)
   (if __bridge_temperature (setq _bridge_temperature __bridge_temperature)) _bridge_temperature)
  (:motor_temperature
   (&optional __motor_temperature)
   (if __motor_temperature (setq _motor_temperature __motor_temperature)) _motor_temperature)
  (:measured_velocity
   (&optional __measured_velocity)
   (if __measured_velocity (setq _measured_velocity __measured_velocity)) _measured_velocity)
  (:measured_travel
   (&optional __measured_travel)
   (if __measured_travel (setq _measured_travel __measured_travel)) _measured_travel)
  (:driver_fault
   (&optional __driver_fault)
   (if __driver_fault (setq _driver_fault __driver_fault)) _driver_fault)
  (:serialization-length
   ()
   (+
    ;; float32 _current
    4
    ;; float32 _duty_cycle
    4
    ;; float32 _bridge_temperature
    4
    ;; float32 _motor_temperature
    4
    ;; float32 _measured_velocity
    4
    ;; float32 _measured_travel
    4
    ;; bool _driver_fault
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _current
       (sys::poke _current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _duty_cycle
       (sys::poke _duty_cycle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bridge_temperature
       (sys::poke _bridge_temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motor_temperature
       (sys::poke _motor_temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_velocity
       (sys::poke _measured_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_travel
       (sys::poke _measured_travel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _driver_fault
       (if _driver_fault (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _current
     (setq _current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _duty_cycle
     (setq _duty_cycle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bridge_temperature
     (setq _bridge_temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motor_temperature
     (setq _motor_temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_velocity
     (setq _measured_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_travel
     (setq _measured_travel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _driver_fault
     (setq _driver_fault (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get jackal_msgs::DriveFeedback :md5sum-) "8dd0b7a3cfa20cfc5c054ddd9763609b")
(setf (get jackal_msgs::DriveFeedback :datatype-) "jackal_msgs/DriveFeedback")
(setf (get jackal_msgs::DriveFeedback :definition-)
      "# This message represents feedback data from a single drive unit (driver + motor).

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



(provide :jackal_msgs/DriveFeedback "8dd0b7a3cfa20cfc5c054ddd9763609b")


