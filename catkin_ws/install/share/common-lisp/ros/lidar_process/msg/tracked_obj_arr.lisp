; Auto-generated. Do not edit!


(cl:in-package lidar_process-msg)


;//! \htmlinclude tracked_obj_arr.msg.html

(cl:defclass <tracked_obj_arr> (roslisp-msg-protocol:ros-message)
  ((tracked_obj_arr
    :reader tracked_obj_arr
    :initarg :tracked_obj_arr
    :type (cl:vector lidar_process-msg:tracked_obj)
   :initform (cl:make-array 0 :element-type 'lidar_process-msg:tracked_obj :initial-element (cl:make-instance 'lidar_process-msg:tracked_obj))))
)

(cl:defclass tracked_obj_arr (<tracked_obj_arr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tracked_obj_arr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tracked_obj_arr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_process-msg:<tracked_obj_arr> is deprecated: use lidar_process-msg:tracked_obj_arr instead.")))

(cl:ensure-generic-function 'tracked_obj_arr-val :lambda-list '(m))
(cl:defmethod tracked_obj_arr-val ((m <tracked_obj_arr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_process-msg:tracked_obj_arr-val is deprecated.  Use lidar_process-msg:tracked_obj_arr instead.")
  (tracked_obj_arr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tracked_obj_arr>) ostream)
  "Serializes a message object of type '<tracked_obj_arr>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracked_obj_arr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tracked_obj_arr))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tracked_obj_arr>) istream)
  "Deserializes a message object of type '<tracked_obj_arr>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracked_obj_arr) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracked_obj_arr)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'lidar_process-msg:tracked_obj))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tracked_obj_arr>)))
  "Returns string type for a message object of type '<tracked_obj_arr>"
  "lidar_process/tracked_obj_arr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tracked_obj_arr)))
  "Returns string type for a message object of type 'tracked_obj_arr"
  "lidar_process/tracked_obj_arr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tracked_obj_arr>)))
  "Returns md5sum for a message object of type '<tracked_obj_arr>"
  "16c935949bd8cf174eedb225267fa459")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tracked_obj_arr)))
  "Returns md5sum for a message object of type 'tracked_obj_arr"
  "16c935949bd8cf174eedb225267fa459")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tracked_obj_arr>)))
  "Returns full string definition for message of type '<tracked_obj_arr>"
  (cl:format cl:nil "tracked_obj[] tracked_obj_arr~%~%================================================================================~%MSG: lidar_process/tracked_obj~%Header header~%uint32 object_id~%string object_type~%geometry_msgs/Point point~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tracked_obj_arr)))
  "Returns full string definition for message of type 'tracked_obj_arr"
  (cl:format cl:nil "tracked_obj[] tracked_obj_arr~%~%================================================================================~%MSG: lidar_process/tracked_obj~%Header header~%uint32 object_id~%string object_type~%geometry_msgs/Point point~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tracked_obj_arr>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracked_obj_arr) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tracked_obj_arr>))
  "Converts a ROS message object to a list"
  (cl:list 'tracked_obj_arr
    (cl:cons ':tracked_obj_arr (tracked_obj_arr msg))
))
