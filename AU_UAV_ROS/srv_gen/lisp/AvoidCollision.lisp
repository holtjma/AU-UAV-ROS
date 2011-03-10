; Auto-generated. Do not edit!


(cl:in-package AU_UAV_ROS-srv)


;//! \htmlinclude AvoidCollision-request.msg.html

(cl:defclass <AvoidCollision-request> (roslisp-msg-protocol:ros-message)
  ((newCommand
    :reader newCommand
    :initarg :newCommand
    :type cl:string
    :initform ""))
)

(cl:defclass AvoidCollision-request (<AvoidCollision-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AvoidCollision-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AvoidCollision-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AU_UAV_ROS-srv:<AvoidCollision-request> is deprecated: use AU_UAV_ROS-srv:AvoidCollision-request instead.")))

(cl:ensure-generic-function 'newCommand-val :lambda-list '(m))
(cl:defmethod newCommand-val ((m <AvoidCollision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AU_UAV_ROS-srv:newCommand-val is deprecated.  Use AU_UAV_ROS-srv:newCommand instead.")
  (newCommand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AvoidCollision-request>) ostream)
  "Serializes a message object of type '<AvoidCollision-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'newCommand))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'newCommand))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AvoidCollision-request>) istream)
  "Deserializes a message object of type '<AvoidCollision-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'newCommand) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'newCommand) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AvoidCollision-request>)))
  "Returns string type for a service object of type '<AvoidCollision-request>"
  "AU_UAV_ROS/AvoidCollisionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AvoidCollision-request)))
  "Returns string type for a service object of type 'AvoidCollision-request"
  "AU_UAV_ROS/AvoidCollisionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AvoidCollision-request>)))
  "Returns md5sum for a message object of type '<AvoidCollision-request>"
  "b1a913e97494a4afadf2e84f3984dc4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AvoidCollision-request)))
  "Returns md5sum for a message object of type 'AvoidCollision-request"
  "b1a913e97494a4afadf2e84f3984dc4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AvoidCollision-request>)))
  "Returns full string definition for message of type '<AvoidCollision-request>"
  (cl:format cl:nil "string newCommand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AvoidCollision-request)))
  "Returns full string definition for message of type 'AvoidCollision-request"
  (cl:format cl:nil "string newCommand~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AvoidCollision-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'newCommand))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AvoidCollision-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AvoidCollision-request
    (cl:cons ':newCommand (newCommand msg))
))
;//! \htmlinclude AvoidCollision-response.msg.html

(cl:defclass <AvoidCollision-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass AvoidCollision-response (<AvoidCollision-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AvoidCollision-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AvoidCollision-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AU_UAV_ROS-srv:<AvoidCollision-response> is deprecated: use AU_UAV_ROS-srv:AvoidCollision-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AvoidCollision-response>) ostream)
  "Serializes a message object of type '<AvoidCollision-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AvoidCollision-response>) istream)
  "Deserializes a message object of type '<AvoidCollision-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AvoidCollision-response>)))
  "Returns string type for a service object of type '<AvoidCollision-response>"
  "AU_UAV_ROS/AvoidCollisionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AvoidCollision-response)))
  "Returns string type for a service object of type 'AvoidCollision-response"
  "AU_UAV_ROS/AvoidCollisionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AvoidCollision-response>)))
  "Returns md5sum for a message object of type '<AvoidCollision-response>"
  "b1a913e97494a4afadf2e84f3984dc4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AvoidCollision-response)))
  "Returns md5sum for a message object of type 'AvoidCollision-response"
  "b1a913e97494a4afadf2e84f3984dc4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AvoidCollision-response>)))
  "Returns full string definition for message of type '<AvoidCollision-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AvoidCollision-response)))
  "Returns full string definition for message of type 'AvoidCollision-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AvoidCollision-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AvoidCollision-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AvoidCollision-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AvoidCollision)))
  'AvoidCollision-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AvoidCollision)))
  'AvoidCollision-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AvoidCollision)))
  "Returns string type for a service object of type '<AvoidCollision>"
  "AU_UAV_ROS/AvoidCollision")
