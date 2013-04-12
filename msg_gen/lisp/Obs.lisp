; Auto-generated. Do not edit!


(cl:in-package mdb_drone-msg)


;//! \htmlinclude Obs.msg.html

(cl:defclass <Obs> (roslisp-msg-protocol:ros-message)
  ((msg_z_hat
    :reader msg_z_hat
    :initarg :msg_z_hat
    :type cl:float
    :initform 0.0))
)

(cl:defclass Obs (<Obs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Obs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Obs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mdb_drone-msg:<Obs> is deprecated: use mdb_drone-msg:Obs instead.")))

(cl:ensure-generic-function 'msg_z_hat-val :lambda-list '(m))
(cl:defmethod msg_z_hat-val ((m <Obs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mdb_drone-msg:msg_z_hat-val is deprecated.  Use mdb_drone-msg:msg_z_hat instead.")
  (msg_z_hat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Obs>) ostream)
  "Serializes a message object of type '<Obs>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'msg_z_hat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Obs>) istream)
  "Deserializes a message object of type '<Obs>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'msg_z_hat) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Obs>)))
  "Returns string type for a message object of type '<Obs>"
  "mdb_drone/Obs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Obs)))
  "Returns string type for a message object of type 'Obs"
  "mdb_drone/Obs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Obs>)))
  "Returns md5sum for a message object of type '<Obs>"
  "a133b612fe54b4bd455c20b8fba7e786")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Obs)))
  "Returns md5sum for a message object of type 'Obs"
  "a133b612fe54b4bd455c20b8fba7e786")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Obs>)))
  "Returns full string definition for message of type '<Obs>"
  (cl:format cl:nil "float32 msg_z_hat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Obs)))
  "Returns full string definition for message of type 'Obs"
  (cl:format cl:nil "float32 msg_z_hat~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Obs>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Obs>))
  "Converts a ROS message object to a list"
  (cl:list 'Obs
    (cl:cons ':msg_z_hat (msg_z_hat msg))
))
