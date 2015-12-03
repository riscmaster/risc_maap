; Auto-generated. Do not edit!


(cl:in-package risc_msgs-msg)


;//! \htmlinclude Waypoints.msg.html

(cl:defclass <Waypoints> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (Obj
    :reader Obj
    :initarg :Obj
    :type (cl:vector risc_msgs-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'risc_msgs-msg:Waypoint :initial-element (cl:make-instance 'risc_msgs-msg:Waypoint))))
)

(cl:defclass Waypoints (<Waypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Waypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Waypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name risc_msgs-msg:<Waypoints> is deprecated: use risc_msgs-msg:Waypoints instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Waypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader risc_msgs-msg:header-val is deprecated.  Use risc_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'Obj-val :lambda-list '(m))
(cl:defmethod Obj-val ((m <Waypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader risc_msgs-msg:Obj-val is deprecated.  Use risc_msgs-msg:Obj instead.")
  (Obj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Waypoints>) ostream)
  "Serializes a message object of type '<Waypoints>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Obj))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'Obj))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Waypoints>) istream)
  "Deserializes a message object of type '<Waypoints>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Obj) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Obj)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'risc_msgs-msg:Waypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Waypoints>)))
  "Returns string type for a message object of type '<Waypoints>"
  "risc_msgs/Waypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Waypoints)))
  "Returns string type for a message object of type 'Waypoints"
  "risc_msgs/Waypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Waypoints>)))
  "Returns md5sum for a message object of type '<Waypoints>"
  "28fbf348f14b39c66bdb32d0c92bb26c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Waypoints)))
  "Returns md5sum for a message object of type 'Waypoints"
  "28fbf348f14b39c66bdb32d0c92bb26c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Waypoints>)))
  "Returns full string definition for message of type '<Waypoints>"
  (cl:format cl:nil "Header header~%~%Waypoint[] Obj~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: risc_msgs/Waypoint~%string name~%float64 x~%float64 y~%float64 z~%float64 heading~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Waypoints)))
  "Returns full string definition for message of type 'Waypoints"
  (cl:format cl:nil "Header header~%~%Waypoint[] Obj~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: risc_msgs/Waypoint~%string name~%float64 x~%float64 y~%float64 z~%float64 heading~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Waypoints>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Obj) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Waypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'Waypoints
    (cl:cons ':header (header msg))
    (cl:cons ':Obj (Obj msg))
))
