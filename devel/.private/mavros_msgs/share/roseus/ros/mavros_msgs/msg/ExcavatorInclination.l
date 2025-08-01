;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::ExcavatorInclination)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'ExcavatorInclination (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::EXCAVATORINCLINATION")
  (make-package "MAVROS_MSGS::EXCAVATORINCLINATION"))

(in-package "ROS")
;;//! \htmlinclude ExcavatorInclination.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mavros_msgs::ExcavatorInclination
  :super ros::object
  :slots (_header _boom_deg _forearm_deg _bucket_deg _boom_vel _forearm_vel _bucket_vel _cylinder_boom _cylinder_forearm _cylinder_bucket ))

(defmethod mavros_msgs::ExcavatorInclination
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:boom_deg __boom_deg) 0.0)
    ((:forearm_deg __forearm_deg) 0.0)
    ((:bucket_deg __bucket_deg) 0.0)
    ((:boom_vel __boom_vel) 0.0)
    ((:forearm_vel __forearm_vel) 0.0)
    ((:bucket_vel __bucket_vel) 0.0)
    ((:cylinder_boom __cylinder_boom) 0.0)
    ((:cylinder_forearm __cylinder_forearm) 0.0)
    ((:cylinder_bucket __cylinder_bucket) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _boom_deg (float __boom_deg))
   (setq _forearm_deg (float __forearm_deg))
   (setq _bucket_deg (float __bucket_deg))
   (setq _boom_vel (float __boom_vel))
   (setq _forearm_vel (float __forearm_vel))
   (setq _bucket_vel (float __bucket_vel))
   (setq _cylinder_boom (float __cylinder_boom))
   (setq _cylinder_forearm (float __cylinder_forearm))
   (setq _cylinder_bucket (float __cylinder_bucket))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:boom_deg
   (&optional __boom_deg)
   (if __boom_deg (setq _boom_deg __boom_deg)) _boom_deg)
  (:forearm_deg
   (&optional __forearm_deg)
   (if __forearm_deg (setq _forearm_deg __forearm_deg)) _forearm_deg)
  (:bucket_deg
   (&optional __bucket_deg)
   (if __bucket_deg (setq _bucket_deg __bucket_deg)) _bucket_deg)
  (:boom_vel
   (&optional __boom_vel)
   (if __boom_vel (setq _boom_vel __boom_vel)) _boom_vel)
  (:forearm_vel
   (&optional __forearm_vel)
   (if __forearm_vel (setq _forearm_vel __forearm_vel)) _forearm_vel)
  (:bucket_vel
   (&optional __bucket_vel)
   (if __bucket_vel (setq _bucket_vel __bucket_vel)) _bucket_vel)
  (:cylinder_boom
   (&optional __cylinder_boom)
   (if __cylinder_boom (setq _cylinder_boom __cylinder_boom)) _cylinder_boom)
  (:cylinder_forearm
   (&optional __cylinder_forearm)
   (if __cylinder_forearm (setq _cylinder_forearm __cylinder_forearm)) _cylinder_forearm)
  (:cylinder_bucket
   (&optional __cylinder_bucket)
   (if __cylinder_bucket (setq _cylinder_bucket __cylinder_bucket)) _cylinder_bucket)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _boom_deg
    8
    ;; float64 _forearm_deg
    8
    ;; float64 _bucket_deg
    8
    ;; float64 _boom_vel
    8
    ;; float64 _forearm_vel
    8
    ;; float64 _bucket_vel
    8
    ;; float64 _cylinder_boom
    8
    ;; float64 _cylinder_forearm
    8
    ;; float64 _cylinder_bucket
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _boom_deg
       (sys::poke _boom_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _forearm_deg
       (sys::poke _forearm_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bucket_deg
       (sys::poke _bucket_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _boom_vel
       (sys::poke _boom_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _forearm_vel
       (sys::poke _forearm_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bucket_vel
       (sys::poke _bucket_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cylinder_boom
       (sys::poke _cylinder_boom (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cylinder_forearm
       (sys::poke _cylinder_forearm (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cylinder_bucket
       (sys::poke _cylinder_bucket (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _boom_deg
     (setq _boom_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _forearm_deg
     (setq _forearm_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bucket_deg
     (setq _bucket_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _boom_vel
     (setq _boom_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _forearm_vel
     (setq _forearm_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bucket_vel
     (setq _bucket_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cylinder_boom
     (setq _cylinder_boom (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cylinder_forearm
     (setq _cylinder_forearm (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cylinder_bucket
     (setq _cylinder_bucket (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get mavros_msgs::ExcavatorInclination :md5sum-) "1c5cdfd62e264348da655a1f34c898bc")
(setf (get mavros_msgs::ExcavatorInclination :datatype-) "mavros_msgs/ExcavatorInclination")
(setf (get mavros_msgs::ExcavatorInclination :definition-)
      "# Stamped Excavator Inclination message
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

")



(provide :mavros_msgs/ExcavatorInclination "1c5cdfd62e264348da655a1f34c898bc")


