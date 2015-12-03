; Auto-generated. Do not edit!


(cl:in-package risc_msgs-msg)


;//! \htmlinclude Risc_rois.msg.html

(cl:defclass <Risc_rois> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (landmarks
    :reader landmarks
    :initarg :landmarks
    :type (cl:vector risc_msgs-msg:Risc_roi)
   :initform (cl:make-array 0 :element-type 'risc_msgs-msg:Risc_roi :initial-element (cl:make-instance 'risc_msgs-msg:Risc_roi)))
   (quads
    :reader quads
    :initarg :quads
    :type (cl:vector risc_msgs-msg:Risc_roi)
   :initform (cl:make-array 0 :element-type 'risc_msgs-msg:Risc_roi :initial-element (cl:make-instance 'risc_msgs-msg:Risc_roi))))
)

(cl:defclass Risc_rois (<Risc_rois>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Risc_rois>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Risc_rois)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name risc_msgs-msg:<Risc_rois> is deprecated: use risc_msgs-msg:Risc_rois instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Risc_rois>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader risc_msgs-msg:name-val is deprecated.  Use risc_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'landmarks-val :lambda-list '(m))
(cl:defmethod landmarks-val ((m <Risc_rois>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader risc_msgs-msg:landmarks-val is deprecated.  Use risc_msgs-msg:landmarks instead.")
  (landmarks m))

(cl:ensure-generic-function 'quads-val :lambda-list '(m))
(cl:defmethod quads-val ((m <Risc_rois>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader risc_msgs-msg:quads-val is deprecated.  Use risc_msgs-msg:quads instead.")
  (quads m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Risc_rois>) ostream)
  "Serializes a message object of type '<Risc_rois>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'landmarks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'landmarks))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'quads))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'quads))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Risc_rois>) istream)
  "Deserializes a message object of type '<Risc_rois>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'landmarks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'landmarks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'risc_msgs-msg:Risc_roi))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'quads) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'quads)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'risc_msgs-msg:Risc_roi))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Risc_rois>)))
  "Returns string type for a message object of type '<Risc_rois>"
  "risc_msgs/Risc_rois")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Risc_rois)))
  "Returns string type for a message object of type 'Risc_rois"
  "risc_msgs/Risc_rois")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Risc_rois>)))
  "Returns md5sum for a message object of type '<Risc_rois>"
  "f66579dedd062ccc57f1aced22cbbd3a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Risc_rois)))
  "Returns md5sum for a message object of type 'Risc_rois"
  "f66579dedd062ccc57f1aced22cbbd3a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Risc_rois>)))
  "Returns full string definition for message of type '<Risc_rois>"
  (cl:format cl:nil "string name~%Risc_roi[] landmarks~%Risc_roi[] quads~%~%================================================================================~%MSG: risc_msgs/Risc_roi~%string name~%~%bool visible~%~%int32 x~%~%int32 y~%~%float32 width~%~%float32 height~%~%float64 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Risc_rois)))
  "Returns full string definition for message of type 'Risc_rois"
  (cl:format cl:nil "string name~%Risc_roi[] landmarks~%Risc_roi[] quads~%~%================================================================================~%MSG: risc_msgs/Risc_roi~%string name~%~%bool visible~%~%int32 x~%~%int32 y~%~%float32 width~%~%float32 height~%~%float64 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Risc_rois>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'landmarks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'quads) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Risc_rois>))
  "Converts a ROS message object to a list"
  (cl:list 'Risc_rois
    (cl:cons ':name (name msg))
    (cl:cons ':landmarks (landmarks msg))
    (cl:cons ':quads (quads msg))
))
