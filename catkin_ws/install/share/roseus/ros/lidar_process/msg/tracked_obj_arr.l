;; Auto-generated. Do not edit!


(when (boundp 'lidar_process::tracked_obj_arr)
  (if (not (find-package "LIDAR_PROCESS"))
    (make-package "LIDAR_PROCESS"))
  (shadow 'tracked_obj_arr (find-package "LIDAR_PROCESS")))
(unless (find-package "LIDAR_PROCESS::TRACKED_OBJ_ARR")
  (make-package "LIDAR_PROCESS::TRACKED_OBJ_ARR"))

(in-package "ROS")
;;//! \htmlinclude tracked_obj_arr.msg.html


(defclass lidar_process::tracked_obj_arr
  :super ros::object
  :slots (_tracked_obj_arr ))

(defmethod lidar_process::tracked_obj_arr
  (:init
   (&key
    ((:tracked_obj_arr __tracked_obj_arr) (let (r) (dotimes (i 0) (push (instance lidar_process::tracked_obj :init) r)) r))
    )
   (send-super :init)
   (setq _tracked_obj_arr __tracked_obj_arr)
   self)
  (:tracked_obj_arr
   (&rest __tracked_obj_arr)
   (if (keywordp (car __tracked_obj_arr))
       (send* _tracked_obj_arr __tracked_obj_arr)
     (progn
       (if __tracked_obj_arr (setq _tracked_obj_arr (car __tracked_obj_arr)))
       _tracked_obj_arr)))
  (:serialization-length
   ()
   (+
    ;; lidar_process/tracked_obj[] _tracked_obj_arr
    (apply #'+ (send-all _tracked_obj_arr :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; lidar_process/tracked_obj[] _tracked_obj_arr
     (write-long (length _tracked_obj_arr) s)
     (dolist (elem _tracked_obj_arr)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; lidar_process/tracked_obj[] _tracked_obj_arr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tracked_obj_arr (let (r) (dotimes (i n) (push (instance lidar_process::tracked_obj :init) r)) r))
     (dolist (elem- _tracked_obj_arr)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get lidar_process::tracked_obj_arr :md5sum-) "16c935949bd8cf174eedb225267fa459")
(setf (get lidar_process::tracked_obj_arr :datatype-) "lidar_process/tracked_obj_arr")
(setf (get lidar_process::tracked_obj_arr :definition-)
      "tracked_obj[] tracked_obj_arr

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

")



(provide :lidar_process/tracked_obj_arr "16c935949bd8cf174eedb225267fa459")


