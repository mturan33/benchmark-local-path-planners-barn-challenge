;; Auto-generated. Do not edit!


(when (boundp 'mpc_local_planner_msgs::StateFeedback)
  (if (not (find-package "MPC_LOCAL_PLANNER_MSGS"))
    (make-package "MPC_LOCAL_PLANNER_MSGS"))
  (shadow 'StateFeedback (find-package "MPC_LOCAL_PLANNER_MSGS")))
(unless (find-package "MPC_LOCAL_PLANNER_MSGS::STATEFEEDBACK")
  (make-package "MPC_LOCAL_PLANNER_MSGS::STATEFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude StateFeedback.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mpc_local_planner_msgs::StateFeedback
  :super ros::object
  :slots (_header _state ))

(defmethod mpc_local_planner_msgs::StateFeedback
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state __state)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _state
    (* 8    (length _state)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _state
     (write-long (length _state) s)
     (dotimes (i (length _state))
       (sys::poke (elt _state i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _state i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get mpc_local_planner_msgs::StateFeedback :md5sum-) "a0230db5ba6622f6ac5c5440d8325e69")
(setf (get mpc_local_planner_msgs::StateFeedback :datatype-) "mpc_local_planner_msgs/StateFeedback")
(setf (get mpc_local_planner_msgs::StateFeedback :definition-)
      "std_msgs/Header header

float64[] state

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

")



(provide :mpc_local_planner_msgs/StateFeedback "a0230db5ba6622f6ac5c5440d8325e69")


