; Auto-generated. Do not edit!


(cl:in-package AU_UAV_ROS-msg)


;//! \htmlinclude TelemetryUpdate.msg.html

(cl:defclass <TelemetryUpdate> (roslisp-msg-protocol:ros-message)
  ((telemetryHeader
    :reader telemetryHeader
    :initarg :telemetryHeader
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (update
    :reader update
    :initarg :update
    :type cl:string
    :initform ""))
)

(cl:defclass TelemetryUpdate (<TelemetryUpdate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TelemetryUpdate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TelemetryUpdate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AU_UAV_ROS-msg:<TelemetryUpdate> is deprecated: use AU_UAV_ROS-msg:TelemetryUpdate instead.")))

(cl:ensure-generic-function 'telemetryHeader-val :lambda-list '(m))
(cl:defmethod telemetryHeader-val ((m <TelemetryUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AU_UAV_ROS-msg:telemetryHeader-val is deprecated.  Use AU_UAV_ROS-msg:telemetryHeader instead.")
  (telemetryHeader m))

(cl:ensure-generic-function 'update-val :lambda-list '(m))
(cl:defmethod update-val ((m <TelemetryUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AU_UAV_ROS-msg:update-val is deprecated.  Use AU_UAV_ROS-msg:update instead.")
  (update m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TelemetryUpdate>) ostream)
  "Serializes a message object of type '<TelemetryUpdate>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'telemetryHeader) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'update))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'update))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TelemetryUpdate>) istream)
  "Deserializes a message object of type '<TelemetryUpdate>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'telemetryHeader) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'update) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'update) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TelemetryUpdate>)))
  "Returns string type for a message object of type '<TelemetryUpdate>"
  "AU_UAV_ROS/TelemetryUpdate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TelemetryUpdate)))
  "Returns string type for a message object of type 'TelemetryUpdate"
  "AU_UAV_ROS/TelemetryUpdate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TelemetryUpdate>)))
  "Returns md5sum for a message object of type '<TelemetryUpdate>"
  "72b8d75e83b2990685728c91cbd65231")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TelemetryUpdate)))
  "Returns md5sum for a message object of type 'TelemetryUpdate"
  "72b8d75e83b2990685728c91cbd65231")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TelemetryUpdate>)))
  "Returns full string definition for message of type '<TelemetryUpdate>"
  (cl:format cl:nil "Header telemetryHeader~%string update~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TelemetryUpdate)))
  "Returns full string definition for message of type 'TelemetryUpdate"
  (cl:format cl:nil "Header telemetryHeader~%string update~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TelemetryUpdate>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'telemetryHeader))
     4 (cl:length (cl:slot-value msg 'update))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TelemetryUpdate>))
  "Converts a ROS message object to a list"
  (cl:list 'TelemetryUpdate
    (cl:cons ':telemetryHeader (telemetryHeader msg))
    (cl:cons ':update (update msg))
))
