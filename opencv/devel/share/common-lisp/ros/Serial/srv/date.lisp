; Auto-generated. Do not edit!


(cl:in-package Serial-srv)


;//! \htmlinclude date-request.msg.html

(cl:defclass <date-request> (roslisp-msg-protocol:ros-message)
  ((mood
    :reader mood
    :initarg :mood
    :type cl:integer
    :initform 0))
)

(cl:defclass date-request (<date-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <date-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'date-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Serial-srv:<date-request> is deprecated: use Serial-srv:date-request instead.")))

(cl:ensure-generic-function 'mood-val :lambda-list '(m))
(cl:defmethod mood-val ((m <date-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Serial-srv:mood-val is deprecated.  Use Serial-srv:mood instead.")
  (mood m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <date-request>) ostream)
  "Serializes a message object of type '<date-request>"
  (cl:let* ((signed (cl:slot-value msg 'mood)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <date-request>) istream)
  "Deserializes a message object of type '<date-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mood) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<date-request>)))
  "Returns string type for a service object of type '<date-request>"
  "Serial/dateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'date-request)))
  "Returns string type for a service object of type 'date-request"
  "Serial/dateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<date-request>)))
  "Returns md5sum for a message object of type '<date-request>"
  "bd888d811c7cb3be0c1173dc8d4d390b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'date-request)))
  "Returns md5sum for a message object of type 'date-request"
  "bd888d811c7cb3be0c1173dc8d4d390b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<date-request>)))
  "Returns full string definition for message of type '<date-request>"
  (cl:format cl:nil "int32 mood~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'date-request)))
  "Returns full string definition for message of type 'date-request"
  (cl:format cl:nil "int32 mood~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <date-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <date-request>))
  "Converts a ROS message object to a list"
  (cl:list 'date-request
    (cl:cons ':mood (mood msg))
))
;//! \htmlinclude date-response.msg.html

(cl:defclass <date-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0))
)

(cl:defclass date-response (<date-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <date-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'date-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Serial-srv:<date-response> is deprecated: use Serial-srv:date-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <date-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Serial-srv:x-val is deprecated.  Use Serial-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <date-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Serial-srv:y-val is deprecated.  Use Serial-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <date-response>) ostream)
  "Serializes a message object of type '<date-response>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <date-response>) istream)
  "Deserializes a message object of type '<date-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<date-response>)))
  "Returns string type for a service object of type '<date-response>"
  "Serial/dateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'date-response)))
  "Returns string type for a service object of type 'date-response"
  "Serial/dateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<date-response>)))
  "Returns md5sum for a message object of type '<date-response>"
  "bd888d811c7cb3be0c1173dc8d4d390b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'date-response)))
  "Returns md5sum for a message object of type 'date-response"
  "bd888d811c7cb3be0c1173dc8d4d390b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<date-response>)))
  "Returns full string definition for message of type '<date-response>"
  (cl:format cl:nil "int32 x~%int32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'date-response)))
  "Returns full string definition for message of type 'date-response"
  (cl:format cl:nil "int32 x~%int32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <date-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <date-response>))
  "Converts a ROS message object to a list"
  (cl:list 'date-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'date)))
  'date-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'date)))
  'date-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'date)))
  "Returns string type for a service object of type '<date>"
  "Serial/date")