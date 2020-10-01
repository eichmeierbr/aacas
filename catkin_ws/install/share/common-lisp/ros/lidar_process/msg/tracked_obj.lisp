; Auto-generated. Do not edit!


(cl:in-package lidar_process-msg)


;//! \htmlinclude tracked_obj.msg.html

(cl:defclass <tracked_obj> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (object_id
    :reader object_id
    :initarg :object_id
    :type cl:integer
    :initform 0)
   (object_type
    :reader object_type
    :initarg :object_type
    :type cl:string
    :initform "")
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass tracked_obj (<tracked_obj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tracked_obj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tracked_obj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_process-msg:<tracked_obj> is deprecated: use lidar_process-msg:tracked_obj instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <tracked_obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_process-msg:header-val is deprecated.  Use lidar_process-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <tracked_obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_process-msg:object_id-val is deprecated.  Use lidar_process-msg:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'object_type-val :lambda-list '(m))
(cl:defmethod object_type-val ((m <tracked_obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_process-msg:object_type-val is deprecated.  Use lidar_process-msg:object_type instead.")
  (object_type m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <tracked_obj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_process-msg:point-val is deprecated.  Use lidar_process-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tracked_obj>) ostream)
  "Serializes a message object of type '<tracked_obj>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_type))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tracked_obj>) istream)
  "Deserializes a message object of type '<tracked_obj>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tracked_obj>)))
  "Returns string type for a message object of type '<tracked_obj>"
  "lidar_process/tracked_obj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tracked_obj)))
  "Returns string type for a message object of type 'tracked_obj"
  "lidar_process/tracked_obj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tracked_obj>)))
  "Returns md5sum for a message object of type '<tracked_obj>"
  "4a88e8c1fe182ef5a25eeeadb70085b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tracked_obj)))
  "Returns md5sum for a message object of type 'tracked_obj"
  "4a88e8c1fe182ef5a25eeeadb70085b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tracked_obj>)))
  "Returns full string definition for message of type '<tracked_obj>"
  (cl:format cl:nil "Header header~%uint32 object_id~%string object_type~%geometry_msgs/Point point~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tracked_obj)))
  "Returns full string definition for message of type 'tracked_obj"
  (cl:format cl:nil "Header header~%uint32 object_id~%string object_type~%geometry_msgs/Point point~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tracked_obj>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:length (cl:slot-value msg 'object_type))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tracked_obj>))
  "Converts a ROS message object to a list"
  (cl:list 'tracked_obj
    (cl:cons ':header (header msg))
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':object_type (object_type msg))
    (cl:cons ':point (point msg))
))
