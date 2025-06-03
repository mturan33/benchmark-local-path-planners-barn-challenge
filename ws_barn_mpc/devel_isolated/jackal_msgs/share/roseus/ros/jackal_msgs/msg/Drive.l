;; Auto-generated. Do not edit!


(when (boundp 'jackal_msgs::Drive)
  (if (not (find-package "JACKAL_MSGS"))
    (make-package "JACKAL_MSGS"))
  (shadow 'Drive (find-package "JACKAL_MSGS")))
(unless (find-package "JACKAL_MSGS::DRIVE")
  (make-package "JACKAL_MSGS::DRIVE"))

(in-package "ROS")
;;//! \htmlinclude Drive.msg.html


(intern "*MODE_VELOCITY*" (find-package "JACKAL_MSGS::DRIVE"))
(shadow '*MODE_VELOCITY* (find-package "JACKAL_MSGS::DRIVE"))
(defconstant jackal_msgs::Drive::*MODE_VELOCITY* 0)
(intern "*MODE_PWM*" (find-package "JACKAL_MSGS::DRIVE"))
(shadow '*MODE_PWM* (find-package "JACKAL_MSGS::DRIVE"))
(defconstant jackal_msgs::Drive::*MODE_PWM* 1)
(intern "*MODE_EFFORT*" (find-package "JACKAL_MSGS::DRIVE"))
(shadow '*MODE_EFFORT* (find-package "JACKAL_MSGS::DRIVE"))
(defconstant jackal_msgs::Drive::*MODE_EFFORT* 2)
(intern "*MODE_NONE*" (find-package "JACKAL_MSGS::DRIVE"))
(shadow '*MODE_NONE* (find-package "JACKAL_MSGS::DRIVE"))
(defconstant jackal_msgs::Drive::*MODE_NONE* -1)
(intern "*LEFT*" (find-package "JACKAL_MSGS::DRIVE"))
(shadow '*LEFT* (find-package "JACKAL_MSGS::DRIVE"))
(defconstant jackal_msgs::Drive::*LEFT* 0)
(intern "*RIGHT*" (find-package "JACKAL_MSGS::DRIVE"))
(shadow '*RIGHT* (find-package "JACKAL_MSGS::DRIVE"))
(defconstant jackal_msgs::Drive::*RIGHT* 1)
(defclass jackal_msgs::Drive
  :super ros::object
  :slots (_mode _drivers ))

(defmethod jackal_msgs::Drive
  (:init
   (&key
    ((:mode __mode) 0)
    ((:drivers __drivers) (make-array 2 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _drivers __drivers)
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:drivers
   (&optional __drivers)
   (if __drivers (setq _drivers __drivers)) _drivers)
  (:serialization-length
   ()
   (+
    ;; int8 _mode
    1
    ;; float32[2] _drivers
    (* 4    2)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _mode
       (write-byte _mode s)
     ;; float32[2] _drivers
     (dotimes (i 2)
       (sys::poke (elt _drivers i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; float32[2] _drivers
   (dotimes (i (length _drivers))
     (setf (elt _drivers i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get jackal_msgs::Drive :md5sum-) "601cf097cd47c174590c366c6ddd5fb3")
(setf (get jackal_msgs::Drive :datatype-) "jackal_msgs/Drive")
(setf (get jackal_msgs::Drive :definition-)
      "# This message represents a low-level motor command to Jackal.

# Command units dependent on the value of this field
int8 MODE_VELOCITY=0   # velocity command (rad/s of wheels)
int8 MODE_PWM=1        # proportion of full voltage command [-1.0..1.0]
int8 MODE_EFFORT=2     # torque command (Nm)
int8 MODE_NONE=-1      # no control, commanded values ignored.
int8 mode

# Units as above, +ve direction propels chassis forward.
int8 LEFT=0
int8 RIGHT=1
float32[2] drivers

")



(provide :jackal_msgs/Drive "601cf097cd47c174590c366c6ddd5fb3")


