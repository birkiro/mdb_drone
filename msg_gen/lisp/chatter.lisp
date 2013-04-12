; Auto-generated. Do not edit!


(cl:in-package mdb_drone-msg)


;//! \htmlinclude chatter.msg.html

(cl:defclass <chatter> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:fixnum
    :initform 0))
)

(cl:defclass chatter (<chatter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <chatter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'chatter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mdb_drone-msg:<chatter> is deprecated: use mdb_drone-msg:chatter instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <chatter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mdb_drone-msg:msg-val is deprecated.  Use mdb_drone-msg:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <chatter>) ostream)
  "Serializes a message object of type '<chatter>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <chatter>) istream)
  "Deserializes a message object of type '<chatter>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<chatter>)))
  "Returns string type for a message object of type '<chatter>"
  "mdb_drone/chatter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chatter)))
  "Returns string type for a message object of type 'chatter"
  "mdb_drone/chatter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<chatter>)))
  "Returns md5sum for a message object of type '<chatter>"
  "eb49cab3792a4224ca6b01f0d7099286")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'chatter)))
  "Returns md5sum for a message object of type 'chatter"
  "eb49cab3792a4224ca6b01f0d7099286")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<chatter>)))
  "Returns full string definition for message of type '<chatter>"
  (cl:format cl:nil "uint8 msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'chatter)))
  "Returns full string definition for message of type 'chatter"
  (cl:format cl:nil "uint8 msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <chatter>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <chatter>))
  "Converts a ROS message object to a list"
  (cl:list 'chatter
    (cl:cons ':msg (msg msg))
))
