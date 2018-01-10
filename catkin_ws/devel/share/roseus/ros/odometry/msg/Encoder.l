;; Auto-generated. Do not edit!


(when (boundp 'odometry::Encoder)
  (if (not (find-package "ODOMETRY"))
    (make-package "ODOMETRY"))
  (shadow 'Encoder (find-package "ODOMETRY")))
(unless (find-package "ODOMETRY::ENCODER")
  (make-package "ODOMETRY::ENCODER"))

(in-package "ROS")
;;//! \htmlinclude Encoder.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass odometry::Encoder
  :super ros::object
  :slots (_header _left_enc _right_enc ))

(defmethod odometry::Encoder
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_enc __left_enc) 0)
    ((:right_enc __right_enc) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_enc (round __left_enc))
   (setq _right_enc (round __right_enc))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_enc
   (&optional __left_enc)
   (if __left_enc (setq _left_enc __left_enc)) _left_enc)
  (:right_enc
   (&optional __right_enc)
   (if __right_enc (setq _right_enc __right_enc)) _right_enc)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _left_enc
    4
    ;; uint32 _right_enc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _left_enc
       (write-long _left_enc s)
     ;; uint32 _right_enc
       (write-long _right_enc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _left_enc
     (setq _left_enc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _right_enc
     (setq _right_enc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get odometry::Encoder :md5sum-) "7e67c8e7afd2165bbd822f101a08e981")
(setf (get odometry::Encoder :datatype-) "odometry/Encoder")
(setf (get odometry::Encoder :definition-)
      "Header header
uint32 left_enc
uint32 right_enc
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
# 0: no frame
# 1: global frame
string frame_id

")



(provide :odometry/Encoder "7e67c8e7afd2165bbd822f101a08e981")


