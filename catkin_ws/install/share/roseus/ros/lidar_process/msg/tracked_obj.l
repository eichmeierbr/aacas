;; Auto-generated. Do not edit!


(when (boundp 'lidar_process::tracked_obj)
  (if (not (find-package "LIDAR_PROCESS"))
    (make-package "LIDAR_PROCESS"))
  (shadow 'tracked_obj (find-package "LIDAR_PROCESS")))
(unless (find-package "LIDAR_PROCESS::TRACKED_OBJ")
  (make-package "LIDAR_PROCESS::TRACKED_OBJ"))

(in-package "ROS")
;;//! \htmlinclude tracked_obj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lidar_process::tracked_obj
  :super ros::object
  :slots (_header _object_id _object_type _point ))

(defmethod lidar_process::tracked_obj
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:object_id __object_id) 0)
    ((:object_type __object_type) "")
    ((:point __point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _object_id (round __object_id))
   (setq _object_type (string __object_type))
   (setq _point __point)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:object_id
   (&optional __object_id)
   (if __object_id (setq _object_id __object_id)) _object_id)
  (:object_type
   (&optional __object_type)
   (if __object_type (setq _object_type __object_type)) _object_type)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _object_id
    4
    ;; string _object_type
    4 (length _object_type)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _object_id
       (write-long _object_id s)
     ;; string _object_type
       (write-long (length _object_type) s) (princ _object_type s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _object_id
     (setq _object_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _object_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _object_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;;
   self)
  )

(setf (get lidar_process::tracked_obj :md5sum-) "4a88e8c1fe182ef5a25eeeadb70085b9")
(setf (get lidar_process::tracked_obj :datatype-) "lidar_process/tracked_obj")
(setf (get lidar_process::tracked_obj :definition-)
      "Header header
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

")



(provide :lidar_process/tracked_obj "4a88e8c1fe182ef5a25eeeadb70085b9")


